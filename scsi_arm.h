/* scsi_arm.h -- ARM (PS) side SCSI disk service for the henry SoC FPGA.
 *
 * This is the on-board twin of henry_tb's SCSI model: it runs on the Zynq PS
 * (the axilite-mips driver), polls the scsi_shim doorbell over AXI-lite, does the
 * disk I/O, and moves the data through the staging rings
 * (henry docs/arm-device-protocol.md).  The ARM never touches guest memory: the
 * RTL scsi_dma engine walks the HPC3 {BP,BC,DP} chain through the device-snoop
 * shim, scsi_stage feeds it READ data from staging and gathers WRITE data into
 * staging, and each chain posts a completion carrying the shim's req_seq and the
 * bytes it moved.  Same servicer source as henry_tb (ring_service.h).
 *
 * INTEGRATION (on the board, ~/axilite-mips):
 *   In axi.cc:  #include "scsi_arm.h"
 *      Once, after the Driver + DRAM mmap are up, before the core runs:
 *          scsi_ring_init(&g_sring, io);   g_disk.open_image("irix65.img");
 *      Each iteration of the main poll loop:
 *          scsi_arm_poll(d, &g_disk);
 *
 * AXI register map -- MUST match ip_hdl/axi_is_the_worst_v1_0_S00_AXI.v:
 *   reads : 0x30 req_seq(doorbell) 0x31-0x33 CDB[0..11] 0x34 nbdp
 *           0x35 {to_device[16], lun[15:8], dest[7:0]}
 *   writes: 0x0D rsp_seq  0x0F rsp_residual  0x10 {tgt_status[15:8],scsi_status[7:0]}
 *           0x11 sel_delay
 *   ring doorbells 0x1a-0x1c: see ring_service.h
 */
#ifndef SCSI_ARM_H
#define SCSI_ARM_H

#include "driver.hh"        // Driver: read32(port) / write32(port,val)
#include "scsi_service.h"   // scsi_disk, scsi_service_run  (+ henry_scsi.h)
#include "scsi_lastcmd.h"   // g_last_scsi -- snapshot for the "scsi" monitor dump
#include <cstdint>
#include <cstring>
#include <vector>
#include <cstdio>
#include <map>             // write->read round-trip map (SCSIHASH diag)
#include <ctime>           // completion timeout

/* SCSI_RDCHK=1: after each READ chain completes, walk its descriptor chain in guest
 * DRAM and compare every landed byte with the disk bytes we served (henry_tb's
 * scsi_read_check, on silicon).  Guest PA 0x08000000-0x17ffffff is window offset
 * PA & 0x0fffffff; the window base is the staging pointer minus RING_STG_PA. */
static long g_rdchk_reads = 0, g_rdchk_bad = 0;
static void scsi_read_check(const std::vector<uint8_t> &buf, size_t pos, size_t len,
                            uint32_t nbdp, uint64_t lba);
#include "ring_service.h"  // staging rings (docs/arm-device-protocol.md), shared with henry_tb

enum {
  SCSI_R_SEQ      = 0x30, SCSI_R_CDB0 = 0x31, SCSI_R_CDB1 = 0x32, SCSI_R_CDB2 = 0x33,
  SCSI_R_NBDP     = 0x34, SCSI_R_TLD  = 0x35,
  SCSI_R_DBG      = 0x38,   // shim debug viz: [31:28]=#resets [27:22]=#SASR-rd [21:16]=#SCMD-wr
                            //   [15:10]=#SASR-wr [9:8]=phase [7]=CIP [6]=BSY [5]=INTRQ [4:0]=SASR
  SCSI_R_RTLREV   = 0x3F,   // hand-bumped RTL build revision (0xYYYYMMDD) -- verify the bitstream is current
  SCSI_W_RSP_SEQ  = 0x0D, SCSI_W_RESID = 0x0F, SCSI_W_STATUS = 0x10, SCSI_W_SELDELAY = 0x11,
};
/* the RTL revision this driver speaks to: staging rings only, no beat conduit */
static const uint32_t HENRY_RTL_REV = 0x20261018u;   /* + MC VDMA records (types 2/3) in the gio ring */

/* Called while waiting for a chain's completion so a long disk burst does not starve
 * the rest of the main loop -- ENET in particular: servicing a SCSI command to
 * completion with ENET idle let TCP back off until the soak STALLED (2026-10-12).
 * axi.cc points it at the ENET ring service. */
static void (*g_scsi_idle_hook)(void) = nullptr;
static inline void scsi_idle(uint64_t spin) {
  if(g_scsi_idle_hook && (spin & 63u) == 0) {
    g_scsi_idle_hook();
  }
}
static scsi_ring_t g_sring;

static void scsi_read_check(const std::vector<uint8_t> &buf, size_t pos, size_t len,
                            uint32_t nbdp, uint64_t lba) {
  volatile uint8_t *win = g_sring.io.stg - RING_STG_PA;
  auto map = [](uint32_t pa, bool *ok) -> uint32_t {
    *ok = (pa >= 0x08000000u && pa <= 0x17ffffffu);
    return pa & 0x0fffffffu;
  };
  auto be32 = [win](uint32_t off) -> uint32_t {
    return ((uint32_t)win[off] << 24) | ((uint32_t)win[off + 1] << 16) |
           ((uint32_t)win[off + 2] << 8) | (uint32_t)win[off + 3];
  };
  const uint32_t nbdp0 = nbdp;
  size_t off = 0;
  long bad = 0;
  uint32_t first_pa = 0;
  size_t first_off = 0;
  uint8_t first_got = 0, first_want = 0;
  for(int guard = 0; guard < 256 && off < len; guard++) {
    bool ok = false;
    uint32_t d = map(nbdp, &ok);
    if(!ok) {
      break;
    }
    uint32_t bp = be32(d), bc = be32(d + 4), dp = be32(d + 8);
    size_t cnt = bc & 0x3fff;
    for(size_t i = 0; i < cnt && off < len; i++, off++) {
      uint32_t a = map(bp + (uint32_t)i, &ok);
      if(!ok) {
        break;
      }
      if(win[a] != buf[pos + off]) {
        if(bad == 0) {
          first_pa = bp + (uint32_t)i;
          first_off = off;
          first_got = win[a];
          first_want = buf[pos + off];
        }
        bad++;
      }
    }
    if((bc & 0x80000000u) || dp == 0) {
      break;
    }
    nbdp = dp;
  }
  g_rdchk_reads++;
  if(bad) {
    g_rdchk_bad++;
    if(g_rdchk_bad <= 50) {
      fprintf(stderr, "[scsi-rdchk] MISMATCH lba=%llu %ld/%zu bytes, first pa=%08x (stream off %zu) got %02x want %02x, nbdp=%08x\n",
              (unsigned long long)lba, bad, off, first_pa, first_off, first_got, first_want, nbdp0);
    }
  }
  if((g_rdchk_reads & 63) == 0) {
    fprintf(stderr, "[scsi-rdchk] %ld reads checked, %ld mismatched\n", g_rdchk_reads, g_rdchk_bad);
  }
}
static uint64_t g_sr_timeout = 0;

/* The shim only forwards Select-And-Transfer (0x08/0x09) to this service.  The
 * Linux wd33c93 driver opens every target with Select-with-ATN (0x06) and steps the
 * bus phases itself, which the shim drops: each command times out and the guest
 * resets the controller to recover.  The shim has no command latch to report it
 * (and is not worth LUTs), so infer it from what it does count: controller resets
 * (SCSI_R_DBG [31:28], saturating at 15) climbing while the doorbell stands still.
 * Fix on the guest side: force sync_stat to SS_SET in wd33c93.c so it uses 0x08
 * (the henry patch in linux-mips-ip22-64k).  Read the debug word only every 64K
 * polls -- this runs every pass of the main loop. */
static inline void scsi_check_sel_atn(Driver *d, uint32_t seq) {
  static uint32_t polls = 0, seq_at = 0, resets_at = 0;
  static bool warned = false;
  if(warned || (++polls & 0xffffu) != 0) {
    return;
  }
  const uint32_t resets = (d->read32(SCSI_R_DBG) >> 28) & 0xfu;
  if(seq != seq_at) {
    seq_at = seq;
    resets_at = resets;
    return;
  }
  if(resets >= resets_at + 3) {
    warned = true;
    fprintf(stderr, "\n[scsi] ERROR: the guest has reset the WD33C93 %u times without a "
                    "Select-and-Transfer -- it is most likely using Select-with-ATN (0x06), "
                    "which this SCSI shim does not support (Linux wd33c93: patch sync_stat to "
                    "SS_SET so it uses 0x08).  Its SCSI commands will all time out.\n",
            resets - resets_at);
  }
}

static inline bool scsi_arm_poll(Driver *d, scsi_disk *disk) {
  static uint32_t last_seq = 0;
  uint32_t seq = d->read32(SCSI_R_SEQ);
  scsi_check_sel_atn(d, seq);
  if(seq == last_seq) return false;
  last_seq = seq;

  // No root disk attached: answer every command as selection-timeout ("no device")
  // so a disk-less guest (e.g. Linux from initramfs) COMPLETES its SCSI scan instead
  // of hanging -- the shim waits on this reply.
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
  /* Faithful chunked DMA: carry the transfer across HPC3 descriptor-chain EOX chunk
   * boundaries.  IRIX programs the WD33C93 count for < the CDB length and
   * pause/resumes >252KB transfers; {buf,pos,total} persist between doorbells, and a
   * doorbell arriving mid-transfer is a RESUME (same buf, new chain from req.nbdp).
   * Mirrors interp_mips select_and_transfer()/pause_transfer(). */
  static const bool g_rdchk = getenv("SCSI_RDCHK") != nullptr;
  static std::vector<uint8_t> g_buf;
  static size_t   g_pos = 0, g_total = 0;
  static bool     g_active = false, g_to_dev = false;
  static uint64_t g_wr_lba = 0;
  static const bool g_scsihash = getenv("SCSIHASH") != nullptr;
  static uint64_t g_hash = 0, g_hlba = 0; static uint32_t g_hnblk = 0;  /* per-command payload hash */
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
    /* READ data is staged; WRITE data comes back gathered by the engine.  The chain's
     * completion (paired by the shim's req_seq) says how many bytes it moved. */
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
      scsi_idle(spin);                           /* keep ENET serviced meanwhile */
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
      else if(g_rdchk) {
        scsi_read_check(g_buf, g_pos, moved, e.nbdp, g_hlba);
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
    if(g_to_dev) {                               /* WRITE: commit the chunk just gathered */
      size_t nb = moved / 512, base = g_pos / 512;
      for(size_t b = 0; b < nb; b++) {
        disk->block_write(g_wr_lba + base + b, g_buf.data() + g_pos + b * 512);
      }
      { /* SCSIWDUMP=<file>: append every written block as {uint64 lba (LE), 512 bytes},
         * the same format as indysim's, to diff guest writes block by block */
        static FILE *g_wd = getenv("SCSIWDUMP") ? fopen(getenv("SCSIWDUMP"), "wb") : nullptr;
        if(g_wd) {
          for(size_t b = 0; b < nb; b++) {
            uint64_t l = g_wr_lba + base + b;
            fwrite(&l, 8, 1, g_wd);
            fwrite(g_buf.data() + g_pos + b * 512, 512, 1, g_wd);
          }
          fflush(g_wd);
        }
      }
    }
    if(g_scsihash) {                             /* FNV-1a payload hash: WRITE gathered / READ staged */
      for(uint32_t i = 0; i < moved; i++) {
        g_hash ^= g_buf[g_pos+i];
        g_hash *= 1099511628211ULL;
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
          if(it != g_wmap.end()) {
            fprintf(stderr, "[rttrip] lba=%llu nblk=%u %s (w=%016llx r=%016llx)\n",
                    (unsigned long long)g_hlba, g_hnblk,
                    it->second == g_hash ? "MATCH" : "**MISMATCH**",
                    (unsigned long long)it->second, (unsigned long long)g_hash);
          }
        }
      }
    }
  }

  /* IRIX root-mount debug: log every serviced SCSI command + its outcome. */
  static const bool g_scsidbg = getenv("SCSIDBG") != nullptr;
  if(g_scsidbg) {
    fprintf(stderr, "[cdb] seq=%u op=0x%02x cdb=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x "
            "dest=%u lun=%u to_dev=%d wr_lba=%llu status=0x%02x tgt=0x%02x "
            "bufsz=%u moved=%u nbdp=0x%08x\n",
            req.seq, (unsigned)req.cdb[0],
            (unsigned)req.cdb[0],(unsigned)req.cdb[1],(unsigned)req.cdb[2],(unsigned)req.cdb[3],
            (unsigned)req.cdb[4],(unsigned)req.cdb[5],(unsigned)req.cdb[6],(unsigned)req.cdb[7],
            (unsigned)req.cdb[8],(unsigned)req.cdb[9],
            (unsigned)req.dest, (unsigned)req.lun, (int)g_to_dev, (unsigned long long)g_wr_lba,
            (unsigned)rsp.scsi_status, (unsigned)rsp.tgt_status,
            (unsigned)g_buf.size(), (unsigned)moved, req.nbdp);
    fflush(stderr);
  }

  /* snapshot for the "scsi" monitor command.  The chain itself is guest memory, which
   * the ARM no longer reads, so the descriptor list stays empty. */
  g_last_scsi.n_cmds++;
  g_last_scsi.seq = req.seq;
  memcpy(g_last_scsi.cdb, req.cdb, 12);
  g_last_scsi.dest = req.dest; g_last_scsi.lun = req.lun;
  g_last_scsi.to_device = g_to_dev; g_last_scsi.wr_lba = g_wr_lba;
  g_last_scsi.bufsz = (uint32_t)g_buf.size(); g_last_scsi.moved = moved;
  g_last_scsi.residual = rsp.residual;
  g_last_scsi.scsi_status = rsp.scsi_status; g_last_scsi.tgt_status = rsp.tgt_status;
  g_last_scsi.nbdp = req.nbdp;
  g_last_scsi.n_desc = 0;
  g_last_scsi.valid = 1;

  d->write32(SCSI_W_RESID,   rsp.residual);
  d->write32(SCSI_W_STATUS,  ((uint32_t)rsp.tgt_status << 8) | rsp.scsi_status);
  d->write32(SCSI_W_RSP_SEQ, rsp.seq);          /* doorbell echo LAST -> shim completes */
  return true;
}

#endif /* SCSI_ARM_H */
