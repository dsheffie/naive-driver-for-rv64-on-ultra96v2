#ifndef __GIO_SERVICE_H__
#define __GIO_SERVICE_H__
/* ARM side of the ARM-served GIO window (rtl/gio_window.sv, docs/newport-on-henry.md).
 *
 * ONE source for both the board driver and henry_tb, like ring_service.h (whose
 * ring_io_t / ring_rd32 / barriers this reuses).  The PL turns every CPU access to the
 * window into a 16-byte record in the capture ring; this drains the ring in order into
 * a device model and answers READ records, which the PL holds the CPU load for.
 *
 * Record (LE): w0 = {type[31:30] (0 STORE, 1 READ, 2 VDMA STORE, 3 VDMA MARKER),
 * mask[29:22], dwoff[21:0]}; dwoff[21]
 * = space (0 the Newport window, 1 the IOC2 8042 keyboard/mouse controller -> dev2),
 * recno (1, 2, 3, ...), lane[0..7] = the bytes at window offset dwoff*8 + i.  Record n
 * lives in slot (n-1) mod 65536; prod (= recno of the last record written) is at
 * staging +0x60 and may lag the records; cons is ours, doorbelled after each batch.
 *
 * The device sees big-endian accesses the way indysim's models do: offset = the first
 * valid byte, size 1/2/4/8, and the value "in guest memory order", i.e. the bytes in
 * address order loaded little-endian (the CPU path byte-swaps).  A mask that is not
 * one naturally aligned 1/2/4/8-byte access (lwl/swr-style partials) is split into
 * single bytes.
 *
 * MC VDMA (memory -> GIO): the PL brackets each transfer's doublewords (type 2) with a
 * BEGIN and an END marker (type 3, mask bit 0 = BEGIN).  The CPU keeps running during
 * the transfer, so CPU records can fall between the markers; we treat them as REX3 does
 * while a host-data DMA owns it: the device is told (dev.dma) so it reports busy to
 * reads, and the CPU's stores to it are held and applied after END, in order. */
#include <cstdint>
#include <cstdio>
#include "ring_service.h"

enum {
  GIO_DB_ANS_LO = 0x20, GIO_DB_ANS_HI = 0x21, GIO_DB_ANS_SEQ = 0x22, GIO_DB_CONS = 0x23,
  GIO_DB_IRQ = 0x2a,
};
static const uint32_t GIO_RING  = 0x200000;            /* staging offset of the capture ring */
static const uint32_t GIO_N     = 65536;
static const uint32_t GIO_PROD  = 0x00060;
/* PS read 0x21: {held load pending, its recno[30:0]} -- the one GIO load the core waits on */
static const uint32_t GIO_RD_HELD = 0x21;
/* REX3 STATUS (window offset 0x1338, the high word of its doubleword): the only READ
 * answered ahead of the records queued before it -- see gio_answer_early() */
static const uint32_t GIO_STATUS_DWOFF = 0x1338 / 8, GIO_STATUS_MASK = 0x0f;
static const uint32_t REX3_STATUS_GFXBUSY = 1u << 3;

struct gio_dev_t {
  void *ctx;
  void (*write)(void *ctx, uint32_t off, uint64_t v, uint32_t sz);
  uint64_t (*read)(void *ctx, uint32_t off, uint32_t sz);
  void (*dma)(void *ctx, bool on);   /* a VDMA transfer starts / ends (may be null) */
};

struct gio_held_t {
  uint32_t w0;
  uint64_t lanes;
};
static const uint32_t GIO_HELD_N = 8192;   /* CPU stores held during one VDMA transfer */

struct gio_svc_t {
  ring_io_t io;
  gio_dev_t dev;             /* space 0: the window at 0x1f0f0000 (Newport) */
  gio_dev_t dev2;            /* space 1: the IOC2 8042 (offsets = the window's for 0x1fbd9840) */
  uint32_t cons;             /* ARM-owned: records consumed */
  uint32_t irq;              /* last IRQ level doorbelled */
  uint32_t (*held_rd)(void *ctx);   /* PS read GIO_RD_HELD (null: never answer early) */
  uint32_t early_seq;        /* recno of a READ already answered ahead of the drain, 0 none */
  uint32_t early_no;         /* held recno already found not answerable early */
  uint64_t early_reads;
  uint64_t stores, reads, split, bad_recno;
  uint64_t dma_stores, dma_xfers, held_stores, held_overflow;
  bool in_dma;
  bool no_hold;              /* debug: apply CPU stores at once even during a transfer */
  bool log;                  /* debug: one line per VDMA transfer + the CPU reads inside it */
  uint32_t xfer_dws, xfer_reads;
  uint32_t nheld;
  gio_held_t held[GIO_HELD_N];
};

static inline void gio_init(gio_svc_t *g, ring_io_t io, gio_dev_t dev, gio_dev_t dev2) {
  memset(g, 0, sizeof(*g));
  g->io = io;
  g->dev = dev;
  g->dev2 = dev2;
  io.doorbell(io.ctx, GIO_DB_CONS, 0);
  io.doorbell(io.ctx, GIO_DB_IRQ, 0);
}

/* the devices' interrupt outputs: bits 7:0 -> INT3 local1 (bit 7 = VRETRACE, Newport),
 * bit 8 -> INT3 mappable 4 (the 8042).  Each device sets only the bits in its mask. */
static const uint32_t GIO_IRQ_LOCAL1 = 0x0ffu;
static const uint32_t GIO_IRQ_KBD    = 0x100u;
static inline void gio_set_irq(gio_svc_t *g, uint32_t mask, uint32_t levels) {
  const uint32_t v = (g->irq & ~mask) | (levels & mask);
  if(v != g->irq) {
    g->irq = v;
    g->io.doorbell(g->io.ctx, GIO_DB_IRQ, v);
  }
}

/* one record's access, split into device calls; returns the answer lanes for a READ */
static inline uint64_t gio_access(gio_svc_t *g, const gio_dev_t &dev, bool rd, uint32_t dwoff, uint32_t mask, uint64_t lanes) {
  const uint32_t first = mask ? (uint32_t)__builtin_ctz(mask) : 0;
  const uint32_t sz = (uint32_t)__builtin_popcount(mask);
  const bool whole = (sz == 1 || sz == 2 || sz == 4 || sz == 8) && (first % sz) == 0 && (mask >> first) == ((1u << sz) - 1);
  uint64_t ans = 0;
  if(whole) {
    const uint32_t off = dwoff * 8 + first;
    const uint64_t vmask = (sz == 8) ? ~0ull : ((1ull << (sz * 8)) - 1);
    if(rd) {
      ans = (dev.read(dev.ctx, off, sz) & vmask) << (first * 8);
    }
    else {
      dev.write(dev.ctx, off, (lanes >> (first * 8)) & vmask, sz);
    }
    return ans;
  }
  g->split++;
  for(uint32_t i = 0; i < 8; i++) {
    if((mask >> i) & 1) {
      if(rd) {
        ans |= (dev.read(dev.ctx, dwoff * 8 + i, 1) & 0xff) << (i * 8);
      }
      else {
        dev.write(dev.ctx, dwoff * 8 + i, (lanes >> (i * 8)) & 0xff, 1);
      }
    }
  }
  return ans;
}

/* The Newport ISR reads REX3 STATUS on every retrace, and the core is held until we
 * answer.  In order, that READ waits behind every record queued ahead of it -- under
 * Doom 500-2100 VDMA doublewords at ~2.35 us each, i.e. 1-16 ms of core stall per
 * retrace (6% of the core).  Of what STATUS returns, only GFXBUSY depends on those
 * records: it is set while the READ sits inside a VDMA transfer.  So answer the held
 * READ now if it is a STATUS read and the nearest VDMA marker before it (looking back
 * at most GIO_EARLY_LOOKBACK records) is an END, with GFXBUSY clear; otherwise leave it
 * in order, which is exact.  Never report a GFXBUSY REX3 would not: IRIX polls STATUS
 * until it clears, and a spurious busy turned one read per retrace into ~3000/s.
 * Every other READ stays in order. */
static const uint32_t GIO_EARLY_LOOKBACK = 32;
static inline void gio_answer_early(gio_svc_t *g, uint32_t prod) {
  const uint32_t p = g->held_rd(g->io.ctx);
  if(!(p >> 31)) {
    return;
  }
  const uint32_t seq = p & 0x7fffffffu;
  const uint32_t ahead = (seq - (g->cons + 1)) & 0x7fffffffu;   /* records queued before it */
  if(ahead == 0 || ahead >= prod - g->cons || seq == (g->early_seq & 0x7fffffffu) || seq == g->early_no) {
    return;
  }
  g->early_no = seq;         /* decided once: answered below or left in order */
  const uint32_t a = GIO_RING + ((seq - 1) % GIO_N) * 16;
  const uint32_t w0 = ring_rd32(g->io, a), recno = ring_rd32(g->io, a + 4);
  if((recno & 0x7fffffffu) != seq || (w0 >> 30) != 1 || ((w0 >> 21) & 1) ||
     (w0 & 0x1fffffu) != GIO_STATUS_DWOFF || ((w0 >> 22) & 0xffu) != GIO_STATUS_MASK) {
    return;
  }
  /* GFXBUSY at the READ's position: the nearest marker before it, else today's state */
  bool in_dma = g->in_dma;
  for(uint32_t i = 1; i <= ahead; i++) {
    if(i > GIO_EARLY_LOOKBACK) {
      return;                /* inside a long run of records: leave it in order */
    }
    const uint32_t m = ring_rd32(g->io, GIO_RING + ((seq - 1 - i) % GIO_N) * 16);
    if((m >> 30) == 3) {
      in_dma = (m >> 22) & 1;
      break;
    }
  }
  if(in_dma) {
    return;
  }
  const uint64_t ans = gio_access(g, g->dev, true, GIO_STATUS_DWOFF, GIO_STATUS_MASK, 0) & ~(uint64_t)REX3_STATUS_GFXBUSY;
  g->reads++;
  g->early_reads++;
  g->early_seq = recno;
  g->io.doorbell(g->io.ctx, GIO_DB_ANS_LO, (uint32_t)ans);
  g->io.doorbell(g->io.ctx, GIO_DB_ANS_HI, (uint32_t)(ans >> 32));
  g->io.doorbell(g->io.ctx, GIO_DB_ANS_SEQ, recno);
}

/* Drain [cons, prod).  Returns records consumed. */
static inline uint32_t gio_poll(gio_svc_t *g) {
  const uint32_t prod = ring_rd32(g->io, GIO_PROD);
  if(prod == g->cons) {
    return 0;
  }
  ring_rmb();
  if(g->held_rd && prod - g->cons > 1) {
    gio_answer_early(g, prod);
  }
  uint32_t n = 0;
  while(g->cons != prod) {
    if(g->held_rd && (n & 63) == 63) {
      gio_answer_early(g, ring_rd32(g->io, GIO_PROD));   /* the READ may land mid-drain */
    }
    const uint32_t a = GIO_RING + (g->cons % GIO_N) * 16;
    const uint32_t w0 = ring_rd32(g->io, a), recno = ring_rd32(g->io, a + 4);
    const uint64_t lanes = (uint64_t)ring_rd32(g->io, a + 8) | ((uint64_t)ring_rd32(g->io, a + 12) << 32);
    if(recno != g->cons + 1) {
      if(g->bad_recno++ < 10) {
        fprintf(stderr, "[gio] slot %u: recno %u, expected %u (w0 %08x)\n", g->cons % GIO_N, recno, g->cons + 1, w0);
      }
    }
    const uint32_t type = w0 >> 30;
    const bool rd = type == 1;
    const gio_dev_t &dev = ((w0 >> 21) & 1) ? g->dev2 : g->dev;
    g->cons++;
    n++;
    if(type == 3) {
      if((w0 >> 22) & 1) {
        g->in_dma = true;
        g->dma_xfers++;
        g->xfer_dws = 0;
        g->xfer_reads = 0;
      }
      else {
        g->in_dma = false;
        if(g->log) {
          fprintf(stderr, "[vdma] xfer %llu: %u doublewords, %u CPU reads and %u CPU stores inside\n",
                  (unsigned long long)g->dma_xfers, g->xfer_dws, g->xfer_reads, g->nheld);
        }
        for(uint32_t i = 0; i < g->nheld; i++) {
          gio_access(g, g->dev, false, g->held[i].w0 & 0x1fffffu, (g->held[i].w0 >> 22) & 0xffu, g->held[i].lanes);
        }
        g->nheld = 0;
      }
      if(g->dev.dma) {
        g->dev.dma(g->dev.ctx, g->in_dma);
      }
      continue;
    }
    if(type == 2) {
      gio_access(g, g->dev, false, w0 & 0x1fffffu, (w0 >> 22) & 0xffu, lanes);
      g->dma_stores++;
      g->xfer_dws++;
      continue;
    }
    if(g->in_dma && !g->no_hold && !rd && !((w0 >> 21) & 1)) {
      if(g->nheld < GIO_HELD_N) {
        g->held[g->nheld].w0 = w0;
        g->held[g->nheld].lanes = lanes;
        g->nheld++;
        g->held_stores++;
        g->stores++;
        continue;
      }
      g->held_overflow++;     /* out of room: apply it now */
    }
    if(rd && g->early_seq && recno == g->early_seq) {
      g->early_seq = 0;      /* answered by gio_answer_early() */
      continue;
    }
    const uint64_t ans = gio_access(g, dev, rd, w0 & 0x1fffffu, (w0 >> 22) & 0xffu, lanes);
    if(g->in_dma && rd) {
      g->xfer_reads++;
      if(g->log && g->xfer_reads <= 4) {
        fprintf(stderr, "[vdma]   read off %04x mask %02x -> %016llx\n", (w0 & 0x1fffffu) * 8,
                (w0 >> 22) & 0xffu, (unsigned long long)ans);
      }
    }
    if(rd) {
      /* the load is held until the sequence lands: lanes first, sequence last */
      g->reads++;
      g->io.doorbell(g->io.ctx, GIO_DB_ANS_LO, (uint32_t)ans);
      g->io.doorbell(g->io.ctx, GIO_DB_ANS_HI, (uint32_t)(ans >> 32));
      g->io.doorbell(g->io.ctx, GIO_DB_ANS_SEQ, recno);
    }
    else {
      g->stores++;
    }
  }
  g->io.doorbell(g->io.ctx, GIO_DB_CONS, g->cons);
  return n;
}

#endif /* __GIO_SERVICE_H__ */
