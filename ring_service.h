#ifndef __RING_SERVICE_H__
#define __RING_SERVICE_H__
/* ARM side of the staging-ring device protocol (docs/arm-device-protocol.md).
 *
 * ONE source for both the board driver (axi.cc) and henry_tb, so the testbench
 * exercises exactly the protocol code the board runs.  The caller supplies:
 *   - stg:      pointer to the staging region (window offset 0x1e000000).  On the
 *               board that is c_addr + 0x1e000000 (/dev/mem O_SYNC: Normal non-cacheable);
 *               in henry_tb it is &g_mem[0x1e000000].
 *   - doorbell: write one S00_AXI doorbell register (0x18..0x1c).
 * Nothing here ever touches guest memory.
 *
 * Staging words are little-endian (the ARM's order); on a big-endian host this would
 * need byte swaps, but both the A53 and x86 are little-endian.
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>

enum {
  RING_DB_TX_CONS = 0x18, RING_DB_RX_PROD = 0x19,
  RING_DB_SCSI_CQ_CONS = 0x1a, RING_DB_SCSI_RD_BEATS = 0x1b, RING_DB_SCSI_RD_SEQ = 0x1c,
};
static const uint32_t RING_STG_PA   = 0x1e000000u;
static const uint32_t RING_STG_SIZE = 0x01000000u;     /* 16 MB */
static const uint32_t RING_IDX      = 0x00040;         /* tx_prod, rx_cons, rx_drops */
static const uint32_t RING_TXCQ     = 0x01000;
static const uint32_t RING_RXSQ     = 0x02000;
static const uint32_t RING_TXS      = 0x100000;
static const uint32_t RING_RXS      = 0x180000;
static const uint32_t RING_N        = 256;
static const uint32_t RING_SLOT     = 2048;
static const uint32_t RING_MAGIC    = 0x52494e47u;     /* "RING" */
static const uint8_t  RING_RX_STATUS_GOOD = 0x30;      /* Seeq RS_GOOD|RS_END appended byte */
static const uint32_t RING_SIDX     = 0x00050;         /* scsi_cq_prod */
static const uint32_t RING_SCQ      = 0x03000;         /* 64 x 16 B completions */
static const uint32_t RING_SCQ_N    = 64;
static const uint32_t RING_SRD      = 0x800000;        /* 4 MB: ARM -> device beats */
static const uint32_t RING_SWR      = 0xc00000;        /* 4 MB: device -> ARM beats */
static const uint32_t RING_SBUF     = 0x400000;
static const uint32_t RING_TRC      = 0x400000;        /* ENET_RING_TRACE: 64K x 16 B TX descriptor log */

/* store barrier before a doorbell: staging (Normal-NC) writes must be visible to the
 * PL before the Device-memory register write that publishes them; load barrier after
 * reading a PL-owned index so the entry reads are not satisfied early */
static inline void ring_wmb(void) {
#if defined(__aarch64__)
  asm volatile("dsb st" ::: "memory");
#else
  __sync_synchronize();
#endif
}
static inline void ring_rmb(void) {
#if defined(__aarch64__)
  asm volatile("dsb ld" ::: "memory");
#else
  __sync_synchronize();
#endif
}

struct ring_io_t {
  volatile uint8_t *stg;
  void (*doorbell)(void *ctx, uint32_t reg, uint32_t v);
  void *ctx;
};

struct enet_ring_t {
  ring_io_t io;
  uint32_t tx_cons;          /* ARM-owned */
  uint32_t rx_prod;          /* ARM-owned */
  uint64_t tx_pkts, tx_bytes, tx_trunc, tx_tapfail, rx_pkts, rx_full;
  uint64_t tx_ip, tx_ip_long, tx_ip_short;   /* IPv4 frame length vs IP total length */
  /* optional DEBUG hook on a malformed TX frame: {first descriptor, frame len, IP len} */
  void (*tx_diag)(uint32_t first_desc, uint32_t len, uint32_t want);
  bool tx_trace;             /* the bit was built with ENET_RING_TRACE */
};

static inline uint32_t ring_rd32(const ring_io_t &io, uint32_t off) {
  return *reinterpret_cast<volatile uint32_t*>(io.stg + off);
}
static inline void ring_wr32(const ring_io_t &io, uint32_t off, uint32_t v) {
  *reinterpret_cast<volatile uint32_t*>(io.stg + off) = v;
}

/* Reset the ring state on both sides.  Call once, before the core runs (the PL's
 * indices are 0 out of reset, and the staging region is zeroed here to match). */
static inline void enet_ring_init(enet_ring_t *r, ring_io_t io) {
  memset(r, 0, sizeof(*r));
  r->io = io;
  for(uint32_t off = 0; off < RING_RXSQ + RING_N * 16; off += 4) {
    ring_wr32(io, off, 0);
  }
  ring_wr32(io, 0, RING_MAGIC);
  ring_wmb();
  io.doorbell(io.ctx, RING_DB_TX_CONS, 0);
  io.doorbell(io.ctx, RING_DB_RX_PROD, 0);
}

/* TX: hand every completed packet to the tap.  Returns packets consumed. */
static inline uint32_t enet_ring_tx_poll(enet_ring_t *r, int tap_fd) {
  uint32_t prod = ring_rd32(r->io, RING_IDX + 0);
  if(prod == r->tx_cons) {
    return 0;
  }
  ring_rmb();
  uint32_t n = 0;
  uint8_t buf[RING_SLOT];
  while(r->tx_cons != prod) {
    uint32_t i = r->tx_cons % RING_N;
    uint32_t w0 = ring_rd32(r->io, RING_TXCQ + i * 16);
    uint32_t len = w0 & 0xffffu;
    if(w0 & 0x10000u) {
      r->tx_trunc++;                       /* > 2 KB: not a valid Ethernet frame */
    }
    else if(len >= 14) {
      for(uint32_t k = 0; k < len; k++) {
        buf[k] = r->io.stg[RING_TXS + i * RING_SLOT + k];
      }
      /* sanity: an IPv4 frame is 14 + IP total length (+ pad to 60).  Longer means two
       * packets were merged into one frame, shorter means a truncated packet -- either
       * loses data with no checksum error anywhere */
      if(len >= 34 && buf[12] == 0x08 && buf[13] == 0x00) {
        uint32_t ip_len = ((uint32_t)buf[16] << 8) | buf[17];
        uint32_t want = 14 + ip_len;
        r->tx_ip++;
        if(len < want) {
          if(++r->tx_ip_short <= 10) {
            fprintf(stderr, "[ering] SHORT IPv4 frame: len %u, IP says %u (slot %u)\n", len, want, i);
            if(r->tx_diag) {
              r->tx_diag(ring_rd32(r->io, RING_TXCQ + i * 16 + 4), len, want);
            }
            if(r->tx_trace) {      /* the descriptors exactly as the engine read them */
              uint32_t t0 = ring_rd32(r->io, RING_TXCQ + i * 16 + 12);
              for(uint32_t t = t0; t < t0 + 8; t++) {
                uint32_t a = RING_TRC + (t & 0xffff) * 16;
                uint32_t w0 = ring_rd32(r->io, a), nbdp = ring_rd32(r->io, a + 4);
                uint32_t bc = ring_rd32(r->io, a + 8), dp = ring_rd32(r->io, a + 12);
                fprintf(stderr, "[ering]   trc %u: desc %08x bc %08x (cnt %u%s%s) dp %08x prod %u ndesc %u\n",
                        t, nbdp, bc, bc & 0x3fff, (bc & 0x80000000u) ? " EOX" : "",
                        (bc & 0x40000000u) ? " EOXP" : "", dp, w0 >> 16, w0 & 0xff);
                if(bc & 0xc0000000u) {
                  break;
                }
              }
            }
          }
        }
        else if(len > (want < 60 ? 60 : want)) {
          if(++r->tx_ip_long <= 10) {
            fprintf(stderr, "[ering] LONG IPv4 frame: len %u, IP says %u (slot %u)\n", len, want, i);
          }
        }
      }
      if(tap_fd >= 0) {
        if(::write(tap_fd, buf, len) != (ssize_t)len) {
          r->tx_tapfail++;                   /* tap queue full: dropped, like the wire */
        }
      }
      r->tx_pkts++;
      r->tx_bytes += len;
    }
    r->tx_cons++;
    n++;
  }
  ring_wmb();
  r->io.doorbell(r->io.ctx, RING_DB_TX_CONS, r->tx_cons);
  return n;
}

/* RX: stage one frame for the engine.  Returns false when all 256 slots are in
 * flight (the caller drops the frame, as a NIC with no buffers would). */
static inline bool enet_ring_rx_push(enet_ring_t *r, const uint8_t *frame, uint32_t n) {
  uint32_t cons = ring_rd32(r->io, RING_IDX + 4);
  if(r->rx_prod - cons >= RING_N || n + 3 > RING_SLOT) {
    r->rx_full++;
    return false;
  }
  uint32_t i = r->rx_prod % RING_N;
  volatile uint8_t *s = r->io.stg + RING_RXS + i * RING_SLOT;
  s[0] = 0;                                /* 2-byte pad: the guest's IP alignment */
  s[1] = 0;
  for(uint32_t k = 0; k < n; k++) {
    s[2 + k] = frame[k];
  }
  s[2 + n] = RING_RX_STATUS_GOOD;          /* appended Seeq status byte */
  ring_wr32(r->io, RING_RXSQ + i * 16, n + 3);
  r->rx_prod++;
  r->rx_pkts++;
  ring_wmb();
  r->io.doorbell(r->io.ctx, RING_DB_RX_PROD, r->rx_prod);
  return true;
}

static inline uint32_t enet_ring_rx_drops(const enet_ring_t *r) {
  return ring_rd32(r->io, RING_IDX + 8);
}

/* ---- SCSI (rtl/scsi_stage.sv): per chain, READ data is staged by the ARM and WRITE
 * data is gathered by the engine; every chain posts a completion carrying the
 * shim's req_seq, which pairs it with the request that the ARM is servicing. ---- */
struct scsi_cqe_t {
  uint32_t bytes;            /* buffer bytes the chain moved */
  uint32_t nbdp;             /* chain head */
  uint32_t flags;            /* [0] done [1] starved (short READ) [2] aborted [3] WRITE [4] WRITE truncated */
  uint32_t seq;              /* the request it belongs to */
};
struct scsi_ring_t {
  ring_io_t io;
  uint32_t cq_cons;          /* ARM-owned */
  uint64_t chains, skipped, bytes_rd, bytes_wr;
};

static inline void scsi_ring_init(scsi_ring_t *r, ring_io_t io) {
  memset(r, 0, sizeof(*r));
  r->io = io;
  for(uint32_t off = RING_SCQ; off < RING_SCQ + RING_SCQ_N * 16; off += 4) {
    ring_wr32(io, off, 0);
  }
  ring_wr32(io, RING_SIDX, 0);
  ring_wmb();
  io.doorbell(io.ctx, RING_DB_SCSI_CQ_CONS, 0);
  io.doorbell(io.ctx, RING_DB_SCSI_RD_BEATS, 0);
  io.doorbell(io.ctx, RING_DB_SCSI_RD_SEQ, 0);
}

/* READ: stage up to 4 MB of this request's remaining bytes.  The beat count goes
 * first, the seq LAST: the engine accepts the count only once the seq matches the
 * chain it is running.  Returns the bytes staged. */
static inline uint32_t scsi_ring_stage_read(scsi_ring_t *r, const uint8_t *src, uint32_t n, uint32_t seq) {
  if(n > RING_SBUF) {
    n = RING_SBUF;
  }
  volatile uint8_t *d = r->io.stg + RING_SRD;
  for(uint32_t k = 0; k < n; k++) {
    d[k] = src[k];
  }
  ring_wmb();
  r->io.doorbell(r->io.ctx, RING_DB_SCSI_RD_BEATS, (n + 15) / 16);
  r->io.doorbell(r->io.ctx, RING_DB_SCSI_RD_SEQ, seq);
  r->bytes_rd += n;
  return n;
}

/* Non-blocking: true once the completion for request `seq` has arrived.  Entries of
 * other requests (a no-data command can still run the engine) are consumed and
 * skipped. */
static inline bool scsi_ring_poll(scsi_ring_t *r, uint32_t seq, scsi_cqe_t *e) {
  bool hit = false;
  while(!hit && ring_rd32(r->io, RING_SIDX) != r->cq_cons) {
    ring_rmb();
    uint32_t a = RING_SCQ + (r->cq_cons % RING_SCQ_N) * 16;
    e->bytes = ring_rd32(r->io, a);
    e->nbdp  = ring_rd32(r->io, a + 4);
    e->flags = ring_rd32(r->io, a + 8);
    e->seq   = ring_rd32(r->io, a + 12);
    r->cq_cons++;
    r->io.doorbell(r->io.ctx, RING_DB_SCSI_CQ_CONS, r->cq_cons);
    if(e->seq == seq) {
      hit = true;
      r->chains++;
    }
    else {
      r->skipped++;
    }
  }
  return hit;
}

/* WRITE: the bytes the chain gathered */
static inline void scsi_ring_read_wr(scsi_ring_t *r, uint8_t *dst, uint32_t n) {
  if(n > RING_SBUF) {
    n = RING_SBUF;
  }
  volatile uint8_t *s = r->io.stg + RING_SWR;
  for(uint32_t k = 0; k < n; k++) {
    dst[k] = s[k];
  }
  r->bytes_wr += n;
}

#endif /* __RING_SERVICE_H__ */
