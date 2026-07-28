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
#include "scsi_lastcmd.h"    // g_last_scsi -- snapshot for the "scsi" monitor dump
#include <cstdint>
#include <vector>
#include <cstdio>          // CDB logging (IRIX root-mount debug)
#include <unistd.h>        // usleep (chain-coherence re-read diag)
#include <map>             // write->read round-trip map (SCSIHASH diag)

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
static inline bool scsi_arm_poll(Driver *d, scsi_disk *disk, uint8_t *dram) {
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
    moved = scsi_move(scsi_arm_mem, dram, req.nbdp,
                      g_buf.data() + g_pos, (uint32_t)(g_total - g_pos), g_to_dev);
    if(!g_to_dev && moved) {                     /* READ: log the physical DRAM ranges we deposited */
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
    moved = scsi_move(scsi_arm_mem, dram, req.nbdp,
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
  d->write32(SCSI_W_RESID,   rsp.residual);
  d->write32(SCSI_W_STATUS,  ((uint32_t)rsp.tgt_status << 8) | rsp.scsi_status);
  d->write32(SCSI_W_RSP_SEQ, rsp.seq);          /* doorbell echo LAST -> shim completes */
  return true;
}

#endif /* SCSI_ARM_H */
