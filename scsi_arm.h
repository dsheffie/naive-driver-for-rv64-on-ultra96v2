/* scsi_arm.h -- ARM (PS) side SCSI disk service for the henry SoC FPGA.
 *
 * This is the on-board twin of henry_tb's SCSI model: it runs on the Zynq PS
 * (the axilite-mips driver), polls the scsi_shim doorbell over AXI-lite, walks
 * the HPC3 {BP,BC,DP} descriptor chain in the shared DRAM mmap, and does the
 * disk I/O straight into the guest's buffers -- the SAME contract validated in
 * Verilator (see sim/scsi_service.h, sim/henry_scsi.h, sim/henry_tb.cpp).
 *
 * INTEGRATION (on the board, ~/axilite-mips):
 *   1. Copy this file + scsi_service.h + henry_scsi.h into the driver dir.
 *   2. In axi.cc:  #include "scsi_arm.h"
 *      Once, after the Driver + DRAM mmap are up and the core is running:
 *          static scsi_disk g_disk;  g_disk.open_image("irix65.img");
 *      Each iteration of the main poll loop:
 *          scsi_arm_poll(d, &g_disk, c_addr);
 *      NB: pass the DRAM mmap base (axi.cc's `c_addr` = mmap(phys_addr,memsize)),
 *      NOT d->get_vaddr() -- get_vaddr() is the AXI control-register window.
 *   3. The shim's select->data delay is an AXI reg now -- tune without re-synth:
 *          d->write32(SCSI_W_SELDELAY, 0);   // 0 => shim default (8192)
 *
 * AXI register map -- MUST match ip_hdl/axi_is_the_worst_v1_0_S00_AXI.v:
 *   reads : 0x30 req_seq(doorbell) 0x31-0x33 CDB[0..11] 0x34 nbdp
 *           0x35 {to_device[16], lun[15:8], dest[7:0]}
 *   writes: 0x0D rsp_seq  0x0F rsp_residual  0x10 {tgt_status[15:8],scsi_status[7:0]}
 *           0x11 sel_delay
 */
#ifndef SCSI_ARM_H
#define SCSI_ARM_H

#include "driver.hh"        // Driver: read32(port) / write32(port,val) / get_vaddr()
#include "scsi_service.h"   // scsi_disk, scsi_move, scsi_service_run  (+ henry_scsi.h)
#include "scsi_lastcmd.h"
#include "dma_log.h"         // DMALOG: ARM-side guest DRAM access log    // g_last_scsi -- snapshot for the "scsi" monitor dump
#include <cstdint>
#include <vector>
#include <cstdio>          // CDB logging (IRIX root-mount debug)
#include <unistd.h>        // usleep (chain-coherence re-read diag)
#include <map>             // write->read round-trip map (SCSIHASH diag)
#include <ctime>           // SCSIRINGS completion timeout
#include "ring_service.h"  // staging rings (docs/arm-device-protocol.md), shared with henry_tb

enum {
  SCSI_R_SEQ      = 0x30, SCSI_R_CDB0 = 0x31, SCSI_R_CDB1 = 0x32, SCSI_R_CDB2 = 0x33,
  SCSI_R_NBDP     = 0x34, SCSI_R_TLD  = 0x35,
  SCSI_R_DBG      = 0x38,   // shim debug viz: [31:28]=#resets [27:22]=#SASR-rd [21:16]=#SCMD-wr
                            //   [15:10]=#SASR-wr [9:8]=phase [7]=CIP [6]=BSY [5]=INTRQ [4:0]=SASR
  SCSI_R_RTLREV   = 0x3F,   // hand-bumped RTL build revision (0xYYYYMMDD) -- verify the bitstream is current
  SCSI_W_RSP_SEQ  = 0x0D, SCSI_W_RESID = 0x0F, SCSI_W_STATUS = 0x10, SCSI_W_SELDELAY = 0x11,
};

/* ---- DMA write-range log: physical byte-ranges SCSI DEPOSITED into DRAM (READ = disk->DRAM).
 * Cross-checked at a crash catch (axi.cc) -- was the poison's physical page recently SCSI-DMA'd?
 * (page-reuse + stale-L1 hypothesis).  scsi_arm.h is #included only in axi.cc, so these file-scope
 * statics are shared with the catch handler. ---- */
/* per-physical-4KB-page "ever SCSI-DMA'd" bitmap: complete history (no ring wrap).
 * 512MB SCSI_DRAM_WINDOW / 4KB = 128K pages -> 16KB bitmap. */
static const uint32_t SCSI_DMA_NPAGES = 512u*1024u*1024u / 4096u;   /* 131072 */
static uint8_t  g_scsi_dma_pgbmp[SCSI_DMA_NPAGES / 8];              /* 16 KB */
static uint32_t g_scsi_dma_n = 0;         /* total DMA ranges logged (for reporting) */
static inline void scsi_dma_log_add(uint32_t pa, uint32_t len) {
  uint32_t p0 = pa >> 12, p1 = (pa + len + 0xfffu) >> 12;
  for(uint32_t p = p0; p < p1 && p < SCSI_DMA_NPAGES; p++) g_scsi_dma_pgbmp[p >> 3] |= (uint8_t)(1u << (p & 7));
  g_scsi_dma_n++;
}
static inline bool scsi_dma_page_seen(uint32_t pa) {
  uint32_t p = pa >> 12;
  return p < SCSI_DMA_NPAGES && (g_scsi_dma_pgbmp[p >> 3] & (1u << (p & 7)));
}

/* FPGA AXI DRAM address map -- MUST match henry_tb fpga_map / the M00_AXI fold.
 * A guest physical address (descriptor BP/DP, the chain head NBDP) maps to a
 * DRAM-window offset, then to the host pointer get_vaddr()+offset. */
static const uint64_t SCSI_DRAM_WINDOW = 0x20000000ull;   /* 512 MB */
static const uint32_t SCSI_FPGA_ADDRMASK = 0x1fffffffu;
static inline uint32_t scsi_fpga_map(uint32_t cpuaddr, bool *bad) {
  uint32_t t;
  if(cpuaddr >= 0x08000000u && cpuaddr <= 0x17ffffffu)
    t = cpuaddr & 0x0fffffffu;                  /* 256 MB Low Local Mem */
  else if(cpuaddr >= 0x1f000000u && cpuaddr <= 0x1fffffffu)
    t = 0x10000000u | (cpuaddr & 0x00ffffffu);  /* 16 MB device/PROM shadow */
  else
    t = cpuaddr;                                /* identity */
  *bad = (t > SCSI_FPGA_ADDRMASK);
  return t;
}

/* scsi_move() mem callback: guest PA -> host pointer.  ctx = DRAM mmap base. */
static inline uint8_t *scsi_arm_mem(void *ctx, uint32_t phys, uint32_t len) {
  bool bad = false;
  uint32_t off = scsi_fpga_map(phys, &bad);
  if(bad || (uint64_t)off + len > SCSI_DRAM_WINDOW) return nullptr;
  return reinterpret_cast<uint8_t*>(ctx) + off;
}
/* Same, plus a DMALOG record: used ONLY for the real transfer (scsi_move), so the
 * diagnostic descriptor re-walks below do not pollute the log. */
static inline uint8_t *scsi_arm_mem_log(void *ctx, uint32_t phys, uint32_t len) {
  uint8_t *p = scsi_arm_mem(ctx, phys, len);
  if(p != nullptr) {
    dma_log(phys, len);
  }
  return p;
}

/* ---- SCSIENGINE=1: disk READs go through the RTL scsi_dma engine ----
 * Instead of scsi_move() (the ARM writing guest DRAM directly through the PS port,
 * invisible to the core's caches), the ARM streams the disk bytes as 16-byte beats
 * into the SCSI beat FIFO (AXI 0x20-0x23, push on the 0x23 write; 0x25 bit0 = full)
 * and the engine deposits them -- through the device-snoop shim, which writes back and
 * invalidates each line in the core's caches around the store.  So a READ needs no ARM
 * cache flush: SCSIENGINE_FLUSH=1 keeps the EXTFLUSH flushes on READs anyway (data-path
 * change only, for an A/B).  WRITEs (mem->disk): wait for the engine (which reads every
 * line through the shim), then scsi_move from now-current DRAM -- also no ARM flush.
 * Beat byte order = henry_tb: word i of a beat holds stream bytes 4i..4i+3, byte 4i in
 * [7:0] (a little-endian load on the ARM).
 *
 * Correctness check on every full chunk: wait for the engine to finish its chain
 * (shim phase -> IDLE, SCSI_R_DBG[9:8]; dma_done means every store got its AXI
 * response), then compare the guest buffers in DRAM with the disk bytes, BEFORE the
 * completion doorbell (the guest is still waiting, so nothing races the compare).
 * The first 3 engine READs are logged, then a stats line every 256.
 * A short transfer (data < chain, e.g. INQUIRY) leaves the engine stalled for beats
 * that never come; the shim aborts it at the doorbell as before, and it is not
 * checked. */
static const bool g_scsiengine = getenv("SCSIENGINE") != nullptr;
/* Called from the SCSIENGINE spin loops (beat FIFO full, waiting for the engine) so a long
 * disk burst does not starve the rest of the main loop -- ENET in particular: servicing a
 * SCSI command to completion with ENET idle let TCP back off until the soak STALLED
 * (2026-10-12, with and without EXTFLUSH).  axi.cc points it at enet_arm_poll. */
static void (*g_scsi_idle_hook)(void) = nullptr;
static inline void scsi_idle(uint64_t spin) {
  if(g_scsi_idle_hook && (spin & 63u) == 0) {
    g_scsi_idle_hook();
  }
}
static const bool g_scsiengine_flush = getenv("SCSIENGINE_FLUSH") != nullptr;
/* SCSIRINGS=1: SCSI data through the staging rings (needs a bit built with
 * SCSI_RINGS).  The ARM never touches guest memory: READ data is staged, WRITE data
 * comes back gathered by the engine, and each chain's completion (carrying the
 * shim's req_seq) says how many bytes it moved -- no descriptor walking here. */
static const bool g_scsirings = getenv("SCSIRINGS") != nullptr;
static scsi_ring_t g_sring;
static uint64_t g_sr_timeout = 0;
enum { SCSI_W_BEAT0 = 0x20, SCSI_R_BEATFULL = 0x25 };
static uint64_t g_se_reads = 0, g_se_bytes = 0, g_se_short = 0, g_se_timeout = 0;
static uint64_t g_se_bad_reads = 0, g_se_bad_bytes = 0, g_se_unaligned = 0, g_se_writes = 0;
/* every 256 engine commands, reads OR writes (reads-only went silent once the guest's
 * page cache absorbed the soak's reads) */
static void scsi_eng_stats(void) {
  fprintf(stderr, "[scsieng] reads=%llu writes=%llu bytes=%llu unaligned=%llu short=%llu timeout=%llu bad_reads=%llu bad_bytes=%llu\n",
          (unsigned long long)g_se_reads, (unsigned long long)g_se_writes, (unsigned long long)g_se_bytes,
          (unsigned long long)g_se_unaligned, (unsigned long long)g_se_short,
          (unsigned long long)g_se_timeout, (unsigned long long)g_se_bad_reads,
          (unsigned long long)g_se_bad_bytes);
}

/* Bytes the chain at nbdp describes (up to EOX), capped at maxlen.  Sets *unaligned
 * if any descriptor's BP is not on a 16-byte line (the case the old engine broke). */
static inline uint32_t scsi_chain_bytes(uint8_t *dram, uint32_t nbdp, uint32_t maxlen, bool *unaligned) {
  uint32_t tot = 0;
  *unaligned = false;
  for(int gd = 0; gd < 128 && nbdp && tot < maxlen; gd++) {
    uint8_t *dd = scsi_arm_mem(dram, nbdp, 12);
    if(!dd) {
      break;
    }
    uint32_t bp = hdma_be32(dd + 0), bc = hdma_be32(dd + 4);
    uint32_t cnt = bc & HPC3_BC_COUNT;
    if(cnt && (bp & 15u)) {
      *unaligned = true;
    }
    tot += cnt;
    if(bc & HPC3_BC_EOX) {
      break;
    }
    nbdp = hdma_be32(dd + 8);
  }
  return tot < maxlen ? tot : maxlen;
}

/* Push len bytes as ceil(len/16) beats (the last one zero-padded; the engine only
 * takes the bytes its descriptors ask for).  false = FIFO stayed full (engine wedged). */
static inline bool scsi_engine_push(Driver *d, const uint8_t *src, uint32_t len) {
  for(uint32_t off = 0; off < len; off += 16) {
    uint32_t w[4] = {0, 0, 0, 0};
    memcpy(w, src + off, (len - off) < 16 ? (len - off) : 16);
    uint64_t spin = 0;
    while(d->read32(SCSI_R_BEATFULL) & 1u) {
      if(++spin > 10000000ull) {
        return false;
      }
      scsi_idle(spin);
    }
    d->write32(SCSI_W_BEAT0 + 0, w[0]);
    d->write32(SCSI_W_BEAT0 + 1, w[1]);
    d->write32(SCSI_W_BEAT0 + 2, w[2]);
    d->write32(SCSI_W_BEAT0 + 3, w[3]);   /* 0x23 write pushes the beat */
  }
  return true;
}

/* Wait for the shim to leave PH_BUSY (engine pulsed dma_done).  false = timeout. */
static inline bool scsi_engine_wait_idle(Driver *d) {
  for(uint64_t spin = 0; spin < 10000000ull; spin++) {
    if(((d->read32(SCSI_R_DBG) >> 8) & 3u) == 0) {
      return true;
    }
    scsi_idle(spin + 1);
  }
  return false;
}

/* Compare the chain's destination bytes in DRAM with ref[0..len). */
static inline void scsi_engine_rdchk(uint8_t *dram, uint32_t nbdp, const uint8_t *ref, uint32_t len,
                                     uint32_t seq, uint64_t lba) {
  uint32_t off = 0, bad = 0, first_pa = 0, first_off = 0;
  for(int gd = 0; gd < 128 && nbdp && off < len; gd++) {
    uint8_t *dd = scsi_arm_mem(dram, nbdp, 12);
    if(!dd) {
      break;
    }
    uint32_t bp = hdma_be32(dd + 0), bc = hdma_be32(dd + 4);
    uint32_t cnt = bc & HPC3_BC_COUNT;
    if(cnt > len - off) {
      cnt = len - off;
    }
    uint8_t *b = cnt ? scsi_arm_mem(dram, bp, cnt) : nullptr;
    for(uint32_t i = 0; b && i < cnt; i++) {
      if(b[i] != ref[off + i]) {
        if(bad == 0) {
          first_pa = bp + i;
          first_off = off + i;
        }
        bad++;
      }
    }
    off += cnt;
    if(bc & HPC3_BC_EOX) {
      break;
    }
    nbdp = hdma_be32(dd + 8);
  }
  if(bad) {
    g_se_bad_reads++;
    g_se_bad_bytes += bad;
    fprintf(stderr, "[scsieng] RDCHK MISMATCH seq=%u lba=%llu: %u/%u bytes bad, first pa=%08x (off %u)\n",
            seq, (unsigned long long)lba, bad, off, first_pa, first_off);
  }
}

/* Poll the doorbell once; on a new command, service it end-to-end.  Call every
 * iteration of the driver's main loop.  Returns true iff a command was serviced.
 *
 * `dram` MUST be the DRAM mmap base (axi.cc `c_addr`), not the control-reg window.
 *
 * Ordering (matches the validated sim contract): the PS reads the request AFTER
 * the doorbell, so the guest's descriptor/buffer cache-writebacks have drained
 * to DRAM by the time we read them (the ARM's poll latency is the real-HW analog
 * of henry_tb's SVC_DELAY).  We deposit the data into DRAM, issue a full memory
 * barrier, and only then echo the doorbell -- so IRIX (which already did its
 * pre-DMA dma_cache_inv) reads the fresh data after completion. */

/* ---- ARM-requested whole-cache flush (henry ctrl reg 4 bit 2) ----
 * EXTFLUSH=1 enables it.  The core flushes L1D then L2 (dirty lines written back,
 * everything invalidated) with its pipeline drained, like an injected IRQ.  Done
 * BEFORE the device touches DRAM (so no dirty CPU data IRIX doesn't know about can
 * be written over the deposit, and a disk write reads current data) and AGAIN after
 * the deposit, before the completion doorbell (drops lines the OOO core refilled
 * speculatively during the transfer).  ~/code/murphi/r9999_dma.m: pre + post is the
 * combination that stays correct when IRIX skips its own invalidate.
 * AXI reads: 0x25[31:16] = completed flushes, 0x25[1] = busy, 0x24[31:1] = cycles
 * the last flush took. */
static const bool g_extflush = getenv("EXTFLUSH") != nullptr;
/* XFPAGES=1 (with EXTFLUSH=1): flush only the transfer's pages instead of the whole
 * cache.  The ARM reads the HPC3 descriptor chain FIRST to find them -- it relies on
 * IRIX's own descriptor writeback (its CACHE ops write through to DRAM), which the
 * whole-cache mode did not need.  Pre-transfer: write back + invalidate each page
 * (AXI write 0x3C per page, then 0x3D=0 go).  Post-deposit (disk READ only): drop
 * each page, no writeback (0x3D=1).  More than 16 pages -> the whole-cache flush.
 * 0x25[15:2] counts dirty lines a drop found: nonzero = a page was not clean at the
 * deposit, i.e. a real coherence bug -- reported the moment it moves. */
static const bool g_xfpages = getenv("XFPAGES") != nullptr;
static const uint32_t XF_MAX_PAGES = 16;
static uint64_t g_xf_n = 0, g_xf_cyc = 0, g_xf_max = 0;
static uint64_t g_xf_lists = 0, g_xf_pages = 0, g_xf_whole = 0;
static uint32_t g_xf_dirty = 0;
/* CHAINCHK=1 (validation, with XFPAGES=1): before a pre-transfer page list, read the
 * chain, force a WHOLE flush (pushes out any dirty descriptor line IRIX failed to
 * write back), read it again and compare.  Any mismatch means the page mode's
 * read-the-chain-first assumption is broken.  Costs a whole flush per command. */
static const bool g_chainchk = getenv("CHAINCHK") != nullptr;
static uint64_t g_cc_n = 0, g_cc_bad = 0;

/* distinct 4KB pages covered by the first `len` bytes of the chain at nbdp; returns
 * false (use the whole flush) if there are more than XF_MAX_PAGES or the chain is
 * unreadable */
static inline bool scsi_chain_pages(uint8_t *dram, uint32_t nbdp, uint32_t len,
                                    uint32_t *pg, uint32_t *npg) {
  *npg = 0;
  uint32_t done = 0;
  for(int nd = 0; nd < 128 && nbdp && done < len; nd++) {
    uint8_t *dd = scsi_arm_mem(dram, nbdp, 12);
    if(!dd) {
      return false;
    }
    uint32_t bp = hdma_be32(dd+0), bc = hdma_be32(dd+4);
    uint32_t cnt = bc & HPC3_BC_COUNT;
    if(cnt > len - done) {
      cnt = len - done;
    }
    if(cnt) {
      for(uint32_t p = bp >> 12; p <= ((bp + cnt - 1) >> 12); p++) {
        bool seen = false;
        for(uint32_t i = 0; i < *npg; i++) {
          if(pg[i] == p) {
            seen = true;
          }
        }
        if(!seen) {
          if(*npg == XF_MAX_PAGES) {
            return false;
          }
          pg[(*npg)++] = p;
        }
      }
    }
    done += cnt;
    if(bc & HPC3_BC_EOX) {
      break;
    }
    nbdp = hdma_be32(dd+8);
  }
  return true;
}

static inline void scsi_whole_flush(Driver *d) {
  while(d->read32(0x25) & 2u) {
  }
  uint32_t n0 = d->read32(0x25) >> 16;
  uint32_t cr = d->read32(4);
  d->write32(4, cr | 4u);
  d->write32(4, cr & ~4u);
  while((d->read32(0x25) >> 16) == n0) {
  }
}

/* raw chain snapshot: {bp, bc, nbdp} per descriptor, up to 64 */
static inline int scsi_chain_snap(uint8_t *dram, uint32_t nbdp, uint32_t *w) {
  int nd = 0;
  for(; nd < 64 && nbdp; nd++) {
    uint8_t *dd = scsi_arm_mem(dram, nbdp, 12);
    if(!dd) {
      break;
    }
    w[3*nd+0] = hdma_be32(dd+0);
    w[3*nd+1] = hdma_be32(dd+4);
    w[3*nd+2] = hdma_be32(dd+8);
    if(w[3*nd+1] & HPC3_BC_EOX) {
      nd++;
      break;
    }
    nbdp = w[3*nd+2];
  }
  return nd;
}

static inline void scsi_extflush(Driver *d, uint8_t *dram, uint32_t nbdp, uint32_t len, bool drop) {
  if(!g_extflush) {
    return;
  }
  if(g_xfpages && g_chainchk && !drop) {
    static uint32_t w0[3*64], w1[3*64];
    int n0 = scsi_chain_snap(dram, nbdp, w0);
    scsi_whole_flush(d);
    int n1 = scsi_chain_snap(dram, nbdp, w1);
    g_cc_n++;
    if(n0 != n1 || memcmp(w0, w1, sizeof(uint32_t) * 3 * n0) != 0) {
      g_cc_bad++;
      fprintf(stderr, "[chainchk] MISMATCH #%llu: nbdp 0x%08x ndesc %d -> %d\n",
              (unsigned long long)g_cc_bad, nbdp, n0, n1);
    }
    if((g_cc_n & 1023u) == 0) {
      fprintf(stderr, "[chainchk] %llu chains checked, %llu mismatches\n",
              (unsigned long long)g_cc_n, (unsigned long long)g_cc_bad);
    }
  }
  uint32_t pg[XF_MAX_PAGES], npg = 0;
  /* too many pages -> the whole flush.  After a deposit that writes back instead of
   * dropping, which is safe because the pre-transfer flush left the buffer clean. */
  bool list = g_xfpages && scsi_chain_pages(dram, nbdp, len, pg, &npg);
  uint64_t spins = 0;
  while(d->read32(0x25) & 2u) {                  /* a flush is still running */
    if(++spins == 100000000ull) {
      fprintf(stderr, "[xflush] WARNING: busy never cleared before request\n");
    }
  }
  uint32_t n0 = d->read32(0x25) >> 16;
  if(list) {
    for(uint32_t i = 0; i < npg; i++) {
      d->write32(0x3c, pg[i]);                   /* push a page number */
    }
    d->write32(0x3d, drop ? 1u : 0u);            /* go: walk the list */
    g_xf_lists++;
    g_xf_pages += npg;
  }
  else {
    uint32_t cr = d->read32(4);
    d->write32(4, cr | 4u);                      /* 0->1 edge starts one whole flush */
    d->write32(4, cr & ~4u);
    g_xf_whole++;
  }
  spins = 0;
  while((d->read32(0x25) >> 16) == n0) {
    if(++spins == 100000000ull) {
      fprintf(stderr, "[xflush] WARNING: flush #%llu has not completed\n", (unsigned long long)g_xf_n);
    }
  }
  uint32_t st = d->read32(0x25);
  uint32_t dirty = (st >> 2) & 0x3fffu;
  if(dirty != g_xf_dirty) {
    fprintf(stderr, "[xflush] DIRTY DROP: %u dirty lines found (was %u) -- nbdp 0x%08x len %u, %u pages\n",
            dirty, g_xf_dirty, nbdp, len, npg);
    g_xf_dirty = dirty;
  }
  uint64_t c = d->read32(0x24) >> 1;
  g_xf_n++; g_xf_cyc += c; g_xf_max = (c > g_xf_max) ? c : g_xf_max;
  if((g_xf_n & 1023u) == 0) {
    fprintf(stderr, "[xflush] %llu flushes (%llu page lists, avg %.1f pages; %llu whole), avg %llu cycles, max %llu, dirty drops %u\n",
            (unsigned long long)g_xf_n, (unsigned long long)g_xf_lists,
            g_xf_lists ? (double)g_xf_pages / g_xf_lists : 0.0, (unsigned long long)g_xf_whole,
            (unsigned long long)(g_xf_cyc / g_xf_n), (unsigned long long)g_xf_max, g_xf_dirty);
  }
}

static inline bool scsi_arm_poll(Driver *d, scsi_disk *disk, uint8_t *dram) {
  g_dma_drv = d;                                  /* DMALOG sync records read the cycle counter */
  static uint32_t last_seq = 0;
  uint32_t seq = d->read32(SCSI_R_SEQ);
  if(seq == last_seq) return false;
  last_seq = seq;

  // No root disk attached: answer every command as selection-timeout ("no device")
  // so a disk-less guest (e.g. Linux from initramfs) COMPLETES its SCSI scan instead
  // of hanging -- the shim now waits on this reply (the self-completing engine is gone).
  if(!disk->ok()) {
    d->write32(SCSI_W_RESID,  0);
    d->write32(SCSI_W_STATUS, ST_SELECTION_TIMEOUT);   // scsi_status=0x42, tgt_status=0
    d->write32(SCSI_W_RSP_SEQ, seq);                   // echo doorbell LAST
    return true;
  }

  scsi_req_t req;
  req.seq     = seq;
  req.channel = CH_SCSI0;
  ((uint32_t*)req.cdb)[0] = d->read32(SCSI_R_CDB0);
  ((uint32_t*)req.cdb)[1] = d->read32(SCSI_R_CDB1);
  ((uint32_t*)req.cdb)[2] = d->read32(SCSI_R_CDB2);
  ((uint32_t*)req.cdb)[3] = 0;                 /* only 12 CDB bytes exposed (>= READ10) */
  req.nbdp = d->read32(SCSI_R_NBDP);
  uint32_t tld = d->read32(SCSI_R_TLD);
  req.dest = tld & 0xff; req.lun = (tld >> 8) & 0xff; req.to_device = (tld >> 16) & 1;
  req.xfer_len = 0;

  scsi_rsp_t rsp;
  rsp.seq = seq; rsp.completion = SCSI_DONE_COMPLETE;
  uint32_t moved = 0;
  bool eng_rd = false;          /* SCSIENGINE READ: the device-snoop shim keeps it coherent */
  bool eng_wr = false;          /* SCSIENGINE WRITE: the engine reads every line through the shim first */
#ifdef FAITHFUL_SCSI
  /* Faithful chunked DMA (Stage A): carry the transfer across HPC3 descriptor-chain
   * EOX chunk boundaries.  IRIX programs the WD33C93 count for < the CDB length and
   * pause/resumes >252KB transfers; {buf,pos,total} persist between doorbells, and a
   * doorbell arriving mid-transfer is a RESUME (same buf, new chain from req.nbdp).
   * Mirrors interp_mips select_and_transfer()/pause_transfer(). */
  static std::vector<uint8_t> g_buf;
  static size_t   g_pos = 0, g_total = 0;
  static bool     g_active = false, g_to_dev = false;
  static uint64_t g_wr_lba = 0;
  static const bool g_scsihash = getenv("SCSIHASH") != nullptr;
  static uint64_t g_hash = 0, g_hlba = 0; static uint32_t g_hnblk = 0;  /* per-READ payload hash */
  if(!(g_active && g_pos < g_total)) {          /* NEW command (not a resume) */
    scsi_service_run(&req, &rsp, disk, g_buf, g_to_dev, g_wr_lba);
    g_total = g_buf.size(); g_pos = 0;
    g_active = (rsp.scsi_status == ST_SELECT_TRANSFER_SUCCESS) && !g_buf.empty();
    g_hash = 1469598103934665603ULL;            /* FNV-1a, matches interp's per-READ hash */
    g_hlba = ((uint64_t)req.cdb[2]<<24)|((uint64_t)req.cdb[3]<<16)|((uint64_t)req.cdb[4]<<8)|req.cdb[5];
    g_hnblk = ((uint32_t)req.cdb[7]<<8)|req.cdb[8];
  } else {                                       /* RESUME the paused transfer */
    rsp.scsi_status = ST_SELECT_TRANSFER_SUCCESS; rsp.tgt_status = TGT_GOOD;
  }
  if(g_active) {
    dma_ctx_set(DMA_DEV_SCSI, g_to_dev ? DMA_DIR_RD : DMA_DIR_WR, (uint32_t)g_hlba);
    eng_rd = g_scsiengine && !g_to_dev;
    eng_wr = g_scsiengine && g_to_dev;
    if(!g_scsirings && (!(eng_rd || eng_wr) || g_scsiengine_flush)) {
      scsi_extflush(d, dram, req.nbdp, (uint32_t)(g_total - g_pos), false);   /* pre: before the device touches DRAM */
    }
    if(g_scsirings) {
      uint32_t rem = (uint32_t)(g_total - g_pos);
      if(!g_to_dev) {
        scsi_ring_stage_read(&g_sring, g_buf.data() + g_pos, rem, req.seq);
      }
      scsi_cqe_t e;
      bool got = false;
      struct timespec t0, t1;
      clock_gettime(CLOCK_MONOTONIC, &t0);
      for(uint64_t spin = 0; ; spin++) {
        if(scsi_ring_poll(&g_sring, req.seq, &e)) {
          got = true;
          break;
        }
        scsi_idle(spin);                         /* keep ENET serviced meanwhile */
        if((spin & 0xfff) == 0) {
          clock_gettime(CLOCK_MONOTONIC, &t1);
          if(t1.tv_sec - t0.tv_sec > 10) {
            break;
          }
        }
      }
      if(got) {
        moved = (e.bytes < rem) ? e.bytes : rem;
        if(g_to_dev) {
          scsi_ring_read_wr(&g_sring, g_buf.data() + g_pos, moved);
        }
      }
      else {
        g_sr_timeout++;
        moved = 0;
        fprintf(stderr, "[scsiring] no completion for seq=%u lba=%llu dir=%d (cq_cons=%u)\n",
                req.seq, (unsigned long long)g_hlba, (int)g_to_dev, g_sring.cq_cons);
      }
      if(((g_sring.chains + g_sr_timeout) & 1023u) == 0) {
        fprintf(stderr, "[scsiring] chains=%llu skipped=%llu staged=%llu gathered=%llu timeout=%llu\n",
                (unsigned long long)g_sring.chains, (unsigned long long)g_sring.skipped,
                (unsigned long long)g_sring.bytes_rd, (unsigned long long)g_sring.bytes_wr,
                (unsigned long long)g_sr_timeout);
      }
    } else if(eng_rd) {
      bool unal = false;
      uint32_t chain = scsi_chain_bytes(dram, req.nbdp, 0xffffffffu, &unal);
      uint32_t want = chain < (uint32_t)(g_total - g_pos) ? chain : (uint32_t)(g_total - g_pos);
      g_se_reads++;
      g_se_bytes += want;
      g_se_unaligned += unal ? 1 : 0;
      if(!scsi_engine_push(d, g_buf.data() + g_pos, want)) {
        g_se_timeout++;
        fprintf(stderr, "[scsieng] beat FIFO stuck full: seq=%u lba=%llu want=%u\n",
                req.seq, (unsigned long long)g_hlba, want);
      } else if(want < chain) {
        g_se_short++;                            /* short: the shim aborts the engine */
      } else if(!scsi_engine_wait_idle(d)) {
        g_se_timeout++;
        fprintf(stderr, "[scsieng] engine never finished: seq=%u lba=%llu want=%u dbg=%08x\n",
                req.seq, (unsigned long long)g_hlba, want, d->read32(SCSI_R_DBG));
      } else {
        scsi_engine_rdchk(dram, req.nbdp, g_buf.data() + g_pos, want, req.seq, g_hlba);
      }
      moved = want;
      if(g_se_reads <= 3) {                      /* positive proof the engine path is live */
        fprintf(stderr, "[scsieng] engine READ #%llu seq=%u lba=%llu want=%u chain=%u unaligned=%d\n",
                (unsigned long long)g_se_reads, req.seq, (unsigned long long)g_hlba, want, chain, (int)unal);
      }
      if(((g_se_reads + g_se_writes) & 255u) == 0) {
        scsi_eng_stats();
      }
    } else if(eng_wr) {
      /* WRITE (mem->disk) with no ARM flush: the engine walks this chain and loads every
       * buffer line through the device-snoop shim (PRE-WBINV pushes any dirty L1D/L2 copy
       * to DRAM).  Once the shim is idle (dma_done) DRAM is current, so the ARM reads it
       * with scsi_move exactly as before -- the engine's own disk beats go unused. */
      g_se_writes++;
      if(((g_se_reads + g_se_writes) & 255u) == 0) {
        scsi_eng_stats();
      }
      if(!scsi_engine_wait_idle(d)) {
        g_se_timeout++;
        fprintf(stderr, "[scsieng] WRITE engine never finished: seq=%u lba=%llu dbg=%08x\n",
                req.seq, (unsigned long long)g_wr_lba, d->read32(SCSI_R_DBG));
      }
      moved = scsi_move(scsi_arm_mem_log, dram, req.nbdp,
                        g_buf.data() + g_pos, (uint32_t)(g_total - g_pos), g_to_dev);
      if(g_se_writes <= 3) {
        fprintf(stderr, "[scsieng] engine WRITE #%llu seq=%u lba=%llu moved=%u\n",
                (unsigned long long)g_se_writes, req.seq, (unsigned long long)g_wr_lba, moved);
      }
    } else {
      moved = scsi_move(scsi_arm_mem_log, dram, req.nbdp,
                        g_buf.data() + g_pos, (uint32_t)(g_total - g_pos), g_to_dev);
    }
    if(!g_to_dev && moved && !g_scsirings) {     /* READ: log the physical DRAM ranges we deposited */
      uint32_t nb = req.nbdp, done = 0;
      for(int gd = 0; gd < 128 && nb && done < moved; gd++) {
        uint8_t *dd = scsi_arm_mem(dram, nb, 12); if(!dd) break;
        uint32_t bp = hdma_be32(dd+0), bc = hdma_be32(dd+4);
        uint32_t cnt = bc & HPC3_BC_COUNT; if(cnt > moved - done) cnt = moved - done;
        if(cnt) scsi_dma_log_add(bp, cnt);
        done += cnt; if(bc & HPC3_BC_EOX) break; nb = hdma_be32(dd+8);
      }
    }
    if(g_to_dev) {                               /* WRITE: commit the chunk just read */
      size_t nb = moved / 512, base = g_pos / 512;
      for(size_t b = 0; b < nb; b++)
        disk->block_write(g_wr_lba + base + b, g_buf.data() + g_pos + b * 512);
    }
    if(g_scsihash) {                             /* accumulate FNV-1a payload hash, both directions */
      if(g_to_dev) {                             /* WRITE: hash the bytes the CORE emitted (from g_buf) */
        for(uint32_t i = 0; i < moved; i++) { g_hash ^= g_buf[g_pos+i]; g_hash *= 1099511628211ULL; }
      } else if(g_scsirings) {                   /* READ (rings): hash the bytes staged */
        for(uint32_t i = 0; i < moved; i++) { g_hash ^= g_buf[g_pos+i]; g_hash *= 1099511628211ULL; }
      } else {                                   /* READ: hash the DRAM the core will read back */
        uint32_t nb = req.nbdp, done = 0;
        for(int gd = 0; gd < 128 && nb && done < moved; gd++) {
          uint8_t *dd = scsi_arm_mem(dram, nb, 12); if(!dd) break;
          uint32_t bp = hdma_be32(dd+0), bc = hdma_be32(dd+4);
          uint32_t cnt = bc & HPC3_BC_COUNT; if(cnt > moved - done) cnt = moved - done;
          uint8_t *b = cnt ? scsi_arm_mem(dram, bp, cnt) : nullptr;
          if(b) for(uint32_t i = 0; i < cnt; i++) { g_hash ^= b[i]; g_hash *= 1099511628211ULL; }
          done += cnt; if(bc & HPC3_BC_EOX) break; nb = hdma_be32(dd+8);
        }
      }
    }
    g_pos += moved;
    if(g_pos < g_total) {                         /* chain EOX'd early -> CHUNK PAUSE */
      rsp.completion  = SCSI_DONE_PAUSE;
      rsp.scsi_status = g_to_dev ? 0x48 : 0x49;   /* -> shim: INTRQ + phase 0x46      */
      rsp.residual    = 0;
    } else {                                      /* whole SCSI command delivered     */
      rsp.residual = 0; g_active = false;
      if(g_scsihash) {
        static std::map<uint64_t,uint64_t> g_wmap;   /* (lba<<16|nblk) -> WRITE hash, for round-trip */
        uint64_t key = (g_hlba << 16) | (uint64_t)g_hnblk;
        if(g_to_dev) {                            /* WRITE complete: emit hash + remember for round-trip */
          fprintf(stderr, "[scsiwhash] op=2a lba=%llu nblk=%u bytes=%zu hash=%016llx\n",
                  (unsigned long long)g_hlba, g_hnblk, (size_t)g_total, (unsigned long long)g_hash);
          g_wmap[key] = g_hash;
        } else {                                  /* READ complete: emit hash + round-trip vs prior WRITE */
          fprintf(stderr, "[scsihash] op=28 lba=%llu nblk=%u bytes=%zu hash=%016llx\n",
                  (unsigned long long)g_hlba, g_hnblk, (size_t)g_total, (unsigned long long)g_hash);
          auto it = g_wmap.find(key);
          if(it != g_wmap.end())
            fprintf(stderr, "[rttrip] lba=%llu nblk=%u %s (w=%016llx r=%016llx)\n",
                    (unsigned long long)g_hlba, g_hnblk,
                    it->second == g_hash ? "MATCH" : "**MISMATCH**",
                    (unsigned long long)it->second, (unsigned long long)g_hash);
        }
      }
    }
  }
  std::vector<uint8_t> &buf = g_buf;              /* aliases for g_last_scsi snapshot */
  bool to_dev = g_to_dev; uint64_t wr_lba = g_wr_lba;
#else
  std::vector<uint8_t> buf;
  bool to_dev = false; uint64_t wr_lba = 0;
  scsi_service_run(&req, &rsp, disk, buf, to_dev, wr_lba);
  if(rsp.scsi_status == ST_SELECT_TRANSFER_SUCCESS && !buf.empty()) {
    dma_ctx_set(DMA_DEV_SCSI, to_dev ? DMA_DIR_RD : DMA_DIR_WR, 0);
    scsi_extflush(d, dram, req.nbdp, (uint32_t)buf.size(), false);   /* pre: before the device touches DRAM */
    moved = scsi_move(scsi_arm_mem_log, dram, req.nbdp,
                      buf.data(), (uint32_t)buf.size(), to_dev);
    if(to_dev) {                                /* WRITE: buf now holds DRAM data */
      size_t nb = moved / 512;
      for(size_t b = 0; b < nb; b++)
        disk->block_write(wr_lba + b, buf.data() + b * 512);
    }
    rsp.residual = (uint32_t)buf.size() - moved;
  }
#endif

  /* IRIX root-mount debug: log every serviced SCSI command + its outcome. */
  static const bool g_scsidbg = getenv("SCSIDBG") != nullptr;
  if(g_scsidbg) fprintf(stderr, "[cdb] seq=%u op=0x%02x cdb=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x "
                  "dest=%u lun=%u to_dev=%d wr_lba=%llu status=0x%02x tgt=0x%02x "
                  "bufsz=%u moved=%u nbdp=0x%08x\n",
          req.seq, (unsigned)req.cdb[0],
          (unsigned)req.cdb[0],(unsigned)req.cdb[1],(unsigned)req.cdb[2],(unsigned)req.cdb[3],
          (unsigned)req.cdb[4],(unsigned)req.cdb[5],(unsigned)req.cdb[6],(unsigned)req.cdb[7],
          (unsigned)req.cdb[8],(unsigned)req.cdb[9],
          (unsigned)req.dest, (unsigned)req.lun, (int)to_dev, (unsigned long long)wr_lba,
          (unsigned)rsp.scsi_status, (unsigned)rsp.tgt_status,
          (unsigned)buf.size(), (unsigned)moved, req.nbdp);
  fflush(stderr);

  /* snapshot for the "scsi" monitor command (full viz into this command) */
  g_last_scsi.n_cmds++;
  g_last_scsi.seq = req.seq;
  memcpy(g_last_scsi.cdb, req.cdb, 12);
  g_last_scsi.dest = req.dest; g_last_scsi.lun = req.lun;
  g_last_scsi.to_device = to_dev; g_last_scsi.wr_lba = wr_lba;
  g_last_scsi.bufsz = (uint32_t)buf.size(); g_last_scsi.moved = moved;
  g_last_scsi.residual = rsp.residual;
  g_last_scsi.scsi_status = rsp.scsi_status; g_last_scsi.tgt_status = rsp.tgt_status;
  g_last_scsi.nbdp = req.nbdp;
  { uint32_t nb = req.nbdp, nd = 0;              /* re-walk the chain (read-only) */
    for(int gd = 0; gd < 16 && nb; gd++) {
      uint8_t *dd = scsi_arm_mem(dram, nb, 12);
      if(!dd) break;
      uint32_t bp = hdma_be32(dd + 0), bc = hdma_be32(dd + 4), nx = hdma_be32(dd + 8);
      g_last_scsi.desc[nd].bp    = bp;
      g_last_scsi.desc[nd].count = bc & HPC3_BC_COUNT;
      g_last_scsi.desc[nd].next  = nx;
      g_last_scsi.desc[nd].eox   = (bc & HPC3_BC_EOX) ? 1 : 0;
      nd++;
      if(bc & HPC3_BC_EOX) break;
      nb = nx;
    }
    g_last_scsi.n_desc = nd;
  }
  g_last_scsi.valid = 1;

  /* --- CHAIN DIAG: dump the descriptor chain the ARM walked for this READ, then
   * re-walk it after a barrier + short delay.  If the two totals differ, the ARM's
   * first walk saw a STALE chain (IRIX's descriptor write-back hadn't landed) -- a
   * descriptor-coherence bug that truncates the transfer with a spurious early EOX. */
  if(g_scsidbg && !to_dev) {
    uint32_t nb = req.nbdp, tot1 = 0; int nd = 0;
    fprintf(stderr, "[chain] seq=%u nbdp=0x%08x moved=%u total=%zu:",
            req.seq, req.nbdp, moved, (size_t)g_total);
    for(; nd < 64 && nb; nd++) {
      uint8_t *dd = scsi_arm_mem(dram, nb, 12); if(!dd) break;
      uint32_t bp = hdma_be32(dd+0), bc = hdma_be32(dd+4);
      uint32_t cnt = bc & HPC3_BC_COUNT; bool eox = (bc & HPC3_BC_EOX) != 0;
      if(nd < 6) fprintf(stderr, " [%d bp=%08x cnt=%u%s]", nd, bp, cnt, eox?" EOX":"");
      tot1 += cnt; if(eox) { nd++; break; } nb = hdma_be32(dd+8);
    }
    __sync_synchronize(); usleep(200);          /* let any in-flight writeback land */
    uint32_t nb2 = req.nbdp, tot2 = 0; int nd2 = 0;
    for(; nd2 < 64 && nb2; nd2++) {
      uint8_t *dd = scsi_arm_mem(dram, nb2, 12); if(!dd) break;
      uint32_t bc = hdma_be32(dd+4); tot2 += (bc & HPC3_BC_COUNT);
      if(bc & HPC3_BC_EOX) { nd2++; break; } nb2 = hdma_be32(dd+8);
    }
    fprintf(stderr, "  ndesc=%d chain_bytes=%u | reread ndesc=%d bytes=%u %s\n",
            nd, tot1, nd2, tot2,
            (tot1 != tot2) ? "<<< STALE CHAIN (coherence) >>>" : "(consistent)");
    fflush(stderr);
  }

  __sync_synchronize();                         /* DRAM writes visible before completion */
  if(moved && !g_scsirings && !(g_xfpages && to_dev) && (!(eng_rd || eng_wr) || g_scsiengine_flush)) {
    /* post: drop speculative refills of the buffer.  Page mode skips it for a disk
     * WRITE: the device only read memory, so there is nothing to drop. */
    scsi_extflush(d, dram, req.nbdp, moved, true);
  }
  d->write32(SCSI_W_RESID,   rsp.residual);
  d->write32(SCSI_W_STATUS,  ((uint32_t)rsp.tgt_status << 8) | rsp.scsi_status);
  d->write32(SCSI_W_RSP_SEQ, rsp.seq);          /* doorbell echo LAST -> shim completes */
  return true;
}

#endif /* SCSI_ARM_H */
