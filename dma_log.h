#ifndef __DMA_LOG_HH__
#define __DMA_LOG_HH__
/* dma_log.h -- binary log of every ARM-side access to guest DRAM.
 *
 * Every host pointer the SCSI and ENET emulation uses to touch guest memory comes
 * from scsi_arm_mem() or enet_arm_mem(), so logging there covers all of it: data
 * deposits (SCSI READ, ENET RX), data collection (SCSI WRITE, ENET TX) and the
 * descriptor reads/writebacks.  The caller sets the context (device, direction,
 * aux) before each walk.  What this CANNOT see is the core itself: its L1/L2
 * fills and writebacks reach DRAM over the PL master, not through the ARM.
 *
 * Enable with DMALOG=<file>; off (one branch) otherwise.  Record = dma_rec_t,
 * 24 bytes, host little-endian.  Decode with dmalog_query.py. */
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include "driver.hh"

enum { DMA_DEV_SYNC = 0, DMA_DEV_SCSI = 1, DMA_DEV_ENET = 2 };
enum { DMA_DIR_RD = 0, DMA_DIR_WR = 1 };   /* WR = the ARM writes guest DRAM */

struct dma_rec_t {
  uint64_t t_ns;    /* CLOCK_MONOTONIC */
  uint32_t pa;      /* guest physical address */
  uint32_t len;
  uint32_t aux;     /* SCSI: lba; ENET TX: chain head nbdp; ENET RX: frame length */
  uint8_t  dev;
  uint8_t  dir;
  uint8_t  desc;    /* 1 = descriptor-sized access (12/16 B); ENET may write +4 of it */
  uint8_t  pad;
};

struct dma_ctx_t {
  uint8_t  dev;
  uint8_t  dir;
  uint32_t aux;
};

static dma_ctx_t g_dma_ctx = {0, 0, 0};
/* For SYNC records: every 1024 records, log the core cycle counter (AXI-Lite regs
 * 0x2A/0x2B = S00 r_cycle, the same count the write-back shadow stamps into tags).
 * A SYNC record is {t_ns, pa = cycle[31:0], len = cycle[63:32], dev = 0}. */
static Driver *g_dma_drv = nullptr;

static inline FILE *dma_log_open() {
  const char *p = getenv("DMALOG");
  if(p == nullptr) {
    return nullptr;
  }
  FILE *f = fopen(p, "wb");
  if(f == nullptr) {
    perror("DMALOG");
    return nullptr;
  }
  static char vbuf[1 << 20];
  setvbuf(f, vbuf, _IOFBF, sizeof(vbuf));
  fprintf(stderr, "[dmalog] logging ARM-side guest DRAM accesses to %s\n", p);
  return f;
}

static inline void dma_log(uint32_t pa, uint32_t len) {
  static FILE *const g_dmalog = dma_log_open();   /* getenv read once */
  static uint64_t g_nrec = 0;
  if(g_dmalog == nullptr) {
    return;
  }
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  dma_rec_t r;
  r.t_ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
  r.pa   = pa;
  r.len  = len;
  r.aux  = g_dma_ctx.aux;
  r.dev  = g_dma_ctx.dev;
  r.dir  = g_dma_ctx.dir;
  r.desc = (len == 12 || len == 16) ? 1 : 0;
  r.pad  = 0;
  fwrite(&r, sizeof(r), 1, g_dmalog);
  if(((g_nrec + 1) & 0x3ff) == 0 && g_dma_drv != nullptr) {
    uint32_t hi0 = g_dma_drv->read32(0x2B), lo = g_dma_drv->read32(0x2A), hi1 = g_dma_drv->read32(0x2B);
    if(hi0 != hi1) {
      lo = g_dma_drv->read32(0x2A);       /* low word wrapped between reads */
    }
    clock_gettime(CLOCK_MONOTONIC, &ts);
    dma_rec_t y = {};
    y.t_ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
    y.pa = lo;
    y.len = hi1;
    y.dev = DMA_DEV_SYNC;
    fwrite(&y, sizeof(y), 1, g_dmalog);
  }
  if((++g_nrec & 0xfff) == 0) {
    fflush(g_dmalog);
  }
}

static inline void dma_ctx_set(uint8_t dev, uint8_t dir, uint32_t aux) {
  g_dma_ctx.dev = dev;
  g_dma_ctx.dir = dir;
  g_dma_ctx.aux = aux;
}
#endif
