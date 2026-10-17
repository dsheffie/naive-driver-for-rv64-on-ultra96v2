#ifndef __GIO_SERVICE_H__
#define __GIO_SERVICE_H__
/* ARM side of the ARM-served GIO window (rtl/gio_window.sv, docs/newport-on-henry.md).
 *
 * ONE source for both the board driver and henry_tb, like ring_service.h (whose
 * ring_io_t / ring_rd32 / barriers this reuses).  The PL turns every CPU access to the
 * window into a 16-byte record in the capture ring; this drains the ring in order into
 * a device model and answers READ records, which the PL holds the CPU load for.
 *
 * Record (LE): w0 = {type[31:30] (0 STORE, 1 READ), mask[29:22], dwoff[21:0]}; dwoff[21]
 * = space (0 the Newport window, 1 the IOC2 8042 keyboard/mouse controller -> dev2),
 * recno (1, 2, 3, ...), lane[0..7] = the bytes at window offset dwoff*8 + i.  Record n
 * lives in slot (n-1) mod 65536; prod (= recno of the last record written) is at
 * staging +0x60 and may lag the records; cons is ours, doorbelled after each batch.
 *
 * The device sees big-endian accesses the way indysim's models do: offset = the first
 * valid byte, size 1/2/4/8, and the value "in guest memory order", i.e. the bytes in
 * address order loaded little-endian (the CPU path byte-swaps).  A mask that is not
 * one naturally aligned 1/2/4/8-byte access (lwl/swr-style partials) is split into
 * single bytes. */
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

struct gio_dev_t {
  void *ctx;
  void (*write)(void *ctx, uint32_t off, uint64_t v, uint32_t sz);
  uint64_t (*read)(void *ctx, uint32_t off, uint32_t sz);
};

struct gio_svc_t {
  ring_io_t io;
  gio_dev_t dev;             /* space 0: the window at 0x1f0f0000 (Newport) */
  gio_dev_t dev2;            /* space 1: the IOC2 8042 (offsets = the window's for 0x1fbd9840) */
  uint32_t cons;             /* ARM-owned: records consumed */
  uint32_t irq;              /* last IRQ level doorbelled */
  uint64_t stores, reads, split, bad_recno;
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

/* Drain [cons, prod).  Returns records consumed. */
static inline uint32_t gio_poll(gio_svc_t *g) {
  const uint32_t prod = ring_rd32(g->io, GIO_PROD);
  if(prod == g->cons) {
    return 0;
  }
  ring_rmb();
  uint32_t n = 0;
  while(g->cons != prod) {
    const uint32_t a = GIO_RING + (g->cons % GIO_N) * 16;
    const uint32_t w0 = ring_rd32(g->io, a), recno = ring_rd32(g->io, a + 4);
    const uint64_t lanes = (uint64_t)ring_rd32(g->io, a + 8) | ((uint64_t)ring_rd32(g->io, a + 12) << 32);
    if(recno != g->cons + 1) {
      if(g->bad_recno++ < 10) {
        fprintf(stderr, "[gio] slot %u: recno %u, expected %u (w0 %08x)\n", g->cons % GIO_N, recno, g->cons + 1, w0);
      }
    }
    const bool rd = (w0 >> 30) == 1;
    const gio_dev_t &dev = ((w0 >> 21) & 1) ? g->dev2 : g->dev;
    const uint64_t ans = gio_access(g, dev, rd, w0 & 0x1fffffu, (w0 >> 22) & 0xffu, lanes);
    g->cons++;
    n++;
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
