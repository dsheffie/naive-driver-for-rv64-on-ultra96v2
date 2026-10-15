/* enet_arm.h -- ARM (PS) side ethernet tap service for the henry SoC FPGA.
 *
 * On-board twin of henry_tb's ENET service (sim/enet_service.h): runs on the Zynq
 * PS (the naive AXI-lite driver), polls the enet_shim mailbox over AXI-lite, walks
 * the HPC3 {BP,BC,DP} descriptor chain in the shared DRAM mmap, and bridges frames
 * to a host TAP -- the SAME contract validated in Verilator (sim/enet_service.h).
 * The tx/rx frame-move logic is shared verbatim via enet_service.h.
 *
 * INTEGRATION (axi.cc, ~/code/naive-driver-for-rv64-on-ultra96v2 / the board driver):
 *   1. Copy this file + enet_service.h + scsi_service.h + henry_scsi.h into the driver dir.
 *   2. In axi.cc:  #include "enet_arm.h"
 *      Once, after the Driver + DRAM mmap are up and the core is running:
 *          static enet_tap g_tap;  g_tap.open_tap("tap0");
 *      Each iteration of the main poll loop (alongside scsi_arm_poll):
 *          enet_arm_poll(d, &g_tap, c_addr);
 *      NB: pass the DRAM mmap base (axi.cc `c_addr`), NOT d->get_vaddr() (the AXI
 *      control-register window) -- same rule as scsi_arm.h.
 *   3. Create the host tap once (persistent, PS-owned):
 *          ip tuntap add dev tap0 mode tap user root
 *          ip addr add 192.168.7.1/24 dev tap0 && ip link set tap0 up   (or bridge it)
 *
 * AXI register map -- MUST match ip_hdl/axi_is_the_worst_v1_0_S00_AXI.v:
 *   reads : 0x39 tx_req_seq(doorbell)  0x3C tx_nbdp  0x3D rx_arm_seq  0x3E rx_nbdp
 *   writes: 0x12 tx_rsp_seq  0x13 rx_rsp_seq  0x14 rx_crbdp
 */
#ifndef ENET_ARM_H
#define ENET_ARM_H

#include "driver.hh"        // Driver: read32(port) / write32(port,val)
#include "enet_service.h"
#include "dma_log.h"        // DMALOG: ARM-side guest DRAM access log   // enet_tap, enet_tx_run, enet_rx_inject (+ henry_scsi.h)
#include <cstdint>
#include <unistd.h>

enum {
  ENET_R_TX_REQ_SEQ = 0x39, ENET_R_TX_NBDP = 0x3C,
  ENET_R_RX_ARM_SEQ = 0x3D, ENET_R_RX_NBDP = 0x3E,
  ENET_W_TX_RSP_SEQ = 0x12, ENET_W_RX_RSP_SEQ = 0x13, ENET_W_RX_CRBDP = 0x14,
  ENET_W_TX_CRBDP = 0x15,
};

/* guest PA -> DRAM-window offset (the SAME fold as scsi_fpga_map / henry_tb fpga_map
 * / the M00_AXI address map). */
static const uint64_t ENET_DRAM_WINDOW = 0x20000000ull;   /* 512 MB */
static inline uint32_t enet_fpga_map(uint32_t cpuaddr, bool *bad) {
  uint32_t t;
  if(cpuaddr >= 0x08000000u && cpuaddr <= 0x17ffffffu)
    t = cpuaddr & 0x0fffffffu;                  /* 256 MB Low Local Mem */
  else if(cpuaddr >= 0x1f000000u && cpuaddr <= 0x1fffffffu)
    t = 0x10000000u | (cpuaddr & 0x00ffffffu);  /* 16 MB device/PROM shadow */
  else
    t = cpuaddr;                                /* identity */
  *bad = (t > 0x1fffffffu);
  return t;
}
/* enet_service.h mem callback: guest PA -> host pointer.  ctx = DRAM mmap base. */
static inline uint8_t *enet_arm_mem(void *ctx, uint32_t phys, uint32_t len) {
  bool bad = false;
  uint32_t off = enet_fpga_map(phys, &bad);
  if(bad || (uint64_t)off + len > ENET_DRAM_WINDOW) return nullptr;
  return reinterpret_cast<uint8_t*>(ctx) + off;
}
/* Same, plus a DMALOG record: used ONLY by the real TX walk and RX deposit. */
static inline uint8_t *enet_arm_mem_log(void *ctx, uint32_t phys, uint32_t len) {
  uint8_t *p = enet_arm_mem(ctx, phys, len);
  if(p != nullptr) {
    dma_log(phys, len);
  }
  return p;
}

/* Poll the ENET mailbox once.  Call every iteration of the driver's main loop.
 * `dram` MUST be the DRAM mmap base (axi.cc `c_addr`), not the control-reg window.
 *
 * Ordering (matches the validated sim contract, scsi_arm.h): read the request AFTER
 * the doorbell so the guest's TX-descriptor cache-writebacks have drained; for RX,
 * deposit the frame into DRAM, memory-barrier, then write crbdp + bump rsp_seq LAST
 * so the shim raises the IRQ only after the frame (and CRBDP) are visible. */
/* ENETFLUSH=1 (with EXTFLUSH=1): whole-cache flush around the host-served ENET
 * path too, like SCSI -- the ARM memcpys frames to/from guest DRAM, so without it
 * TX can read stale data and RX can be shadowed by stale or dirty cache lines. */
static const bool g_enetflush = getenv("ENETFLUSH") != nullptr;

#include "enet_arm_engine.h"
/* ENETENGINE=1: the RTL enet_dma engine walks both rings through the snoop shim, so
 * every guest memory access is coherent and no flush is needed.  The ARM only moves
 * frame bytes between the tap and the engine's beat FIFOs. */
static const bool g_enetengine = getenv("ENETENGINE") != nullptr;
static uint64_t g_ee_chains = 0, g_ee_pkts = 0, g_ee_rx = 0, g_ee_rxdrop = 0, g_ee_badlen = 0, g_ee_torn = 0;
static inline void enet_eng_stats(void) {
  fprintf(stderr, "[eneteng] chains=%llu pkts=%llu rx=%llu rxdrop=%llu badlen=%llu torn=%llu\n",
          (unsigned long long)g_ee_chains, (unsigned long long)g_ee_pkts, (unsigned long long)g_ee_rx,
          (unsigned long long)g_ee_rxdrop, (unsigned long long)g_ee_badlen, (unsigned long long)g_ee_torn);
}

/* Split the engine's TX byte stream into packets.  The engine packs every descriptor's
 * bytes back to back but knows nothing of EOXP, and a chain can hold several packets
 * (see enet_tx_run).  By tx_done the engine has read every descriptor through the
 * snoop shim (written back + invalidated in the CPU caches), so the copy in DRAM is
 * current: walk it for the per-descriptor counts, EOXP boundaries and the chain end,
 * stopping exactly where the engine stopped (tx_len bytes on a packet boundary, EOX,
 * a null link, or 255 descriptors).
 * The data itself comes only from the beats.  Returns tx_end (the TX CRBDP). */
static inline uint32_t enet_eng_emit(enet_tap *tap, uint8_t *dram, uint32_t nbdp, uint32_t txlen) {
  uint32_t off = 0, pkt = 0, sum = 0, end = nbdp;
  bool ok = true;
  for(int n = 0; n < 255; n++) {
    uint8_t *dsc = enet_arm_mem(dram, nbdp, 12);
    if(!dsc) {
      ok = false;
      break;
    }
    uint32_t bc   = hdma_be32(dsc + 4);
    uint32_t next = hdma_be32(dsc + 8);
    sum += bc & HPC3_BC_COUNT;
    end = next;
    if((bc & (HPC3_BC_EOXP | HPC3_BC_EOX)) && sum > pkt) {
      if(sum > g_etx_len) {
        ok = false;
        break;
      }
      if(sum - pkt >= 14) {
        ssize_t w = ::write(tap->fd, g_etx_buf + pkt, sum - pkt);
        (void)w;
      }
      if(!(bc & HPC3_BC_EOXP)) {
        /* torn chain: the packet ended on a bare EOX -- the guest was still building
         * it when the engine walked (the host path deferred these, ENET_TX_INCOMPLETE) */
        if(++g_ee_torn <= 20) {
          fprintf(stderr, "[eneteng] TORN chain: desc 0x%08x bc 0x%08x pkt %u..%u txlen %u\n",
                  nbdp, bc, pkt, sum, txlen);
        }
      }
      g_ee_pkts++;
      pkt = sum;
    }
    /* Stop where the ENGINE stopped, not where the chain ends now: IRIX appends to a
     * chain (clearing the old tail's EOX) after the engine has already finished it,
     * and walking on would publish a TX CRBDP past what was transmitted.  The engine
     * always ends on a packet boundary with exactly txlen bytes. */
    if((bc & HPC3_BC_EOX) || next == 0 ||
       ((bc & HPC3_BC_EOXP) && sum == txlen)) {
      break;
    }
    nbdp = next;
  }
  off = sum;
  if(!ok || off != txlen || ((txlen + 15u) & ~15u) != g_etx_len) {
    /* the walk disagrees with what the engine sent: say so, and send what the engine
     * reported as one frame rather than nothing */
    g_ee_badlen++;
    fprintf(stderr, "[eneteng] TX len mismatch: desc sum %u engine %u beats %u ok %d\n",
            off, txlen, g_etx_len, (int)ok);
    if(pkt == 0 && txlen >= 14 && txlen <= g_etx_len) {
      ssize_t w = ::write(tap->fd, g_etx_buf, txlen);
      (void)w;
    }
  }
  return end;
}

static inline uint64_t enet_now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

static inline bool enet_arm_poll_engine(Driver *d, enet_tap *tap, uint8_t *dram) {
  static uint32_t last_tx_seq = 0, last_arm_seq = 0, rx_rsp_seq = 0;
  static bool rx_armed = false;
  bool did_work = false;
  /* IRIX's ec_watchdog resets the interface (an Oerr, and an RTO for TCP) when TX
   * stays ACTIVE with no progress across a 2 s if_timer tick: log every service gap
   * and every doorbell-to-echo latency that gets anywhere near that */
  static uint64_t last_poll_ms = 0, tx_seen_ms = 0;
  uint64_t now = enet_now_ms();
  if(last_poll_ms != 0 && now - last_poll_ms > 500) {
    fprintf(stderr, "[eneteng] poll gap %llu ms\n", (unsigned long long)(now - last_poll_ms));
  }
  last_poll_ms = now;

  /* ---- TX: the engine already walks the chain on the guest's kick ---- */
  uint32_t tx_seq = d->read32(ENET_R_TX_REQ_SEQ);
  if(tx_seq != last_tx_seq) {
    uint32_t nbdp = d->read32(ENET_R_TX_NBDP);
    did_work = true;
    if(tx_seen_ms == 0) {
      tx_seen_ms = now;
    }
    if(enet_arm_collect_tx(d)) {
      uint32_t tx_end = enet_eng_emit(tap, dram, nbdp, g_etx_ready_len);
      g_etx_len = 0;
      g_etx_ready = false;
      d->write32(ENET_W_TX_CRBDP, tx_end);       /* reap bound, before the echo */
      last_tx_seq = tx_seq;
      __sync_synchronize();
      d->write32(ENET_W_TX_RSP_SEQ, tx_seq);     /* echo -> shim clears ACTIVE + TX IRQ */
      if(enet_now_ms() - tx_seen_ms > 500) {
        fprintf(stderr, "[eneteng] TX latency %llu ms (txlen %u)\n",
                (unsigned long long)(enet_now_ms() - tx_seen_ms), g_etx_ready_len);
      }
      tx_seen_ms = 0;
      if((++g_ee_chains & 1023u) == 0) {
        enet_eng_stats();
      }
    }
  }

  /* ---- RX: the RTL tracks the ring position itself; just wait for the first arm ---- */
  uint32_t arm_seq = d->read32(ENET_R_RX_ARM_SEQ);
  if(arm_seq != last_arm_seq) {
    last_arm_seq = arm_seq;
    rx_armed = true;
  }
  if(rx_armed) {
    uint8_t fr[ENET_FRAME_MAX];
    static uint8_t rxb[ENET_FRAME_MAX + 3];
    for(int k = 0; k < 8; k++) {
      ssize_t n = ::read(tap->fd, fr, sizeof(fr));
      if(n < 14) {
        break;
      }
      bool is_bcast = (fr[0]&fr[1]&fr[2]&fr[3]&fr[4]&fr[5]) == 0xff;
      if((fr[0] & 1) && !is_bcast) {
        continue;                                 /* stray multicast -> drop, as the host path */
      }
      rxb[0] = 0;
      rxb[1] = 0;                                 /* 2-byte pad */
      memcpy(rxb + 2, fr, (size_t)n);
      rxb[2 + n] = ENET_RX_STATUS_GOOD;           /* appended Seeq status */
      if(enet_arm_push_frame(d, rxb, (uint32_t)n + 3)) {
        d->write32(ENET_W_RX_CRBDP, d->read32(ENET_R_CRBDP));   /* guest reads it at HPC3 0x18000 */
        d->write32(ENET_W_RX_RSP_SEQ, ++rx_rsp_seq);              /* -> shim raises RX IRQ */
        g_ee_rx++;
        did_work = true;
      }
      else {
        g_ee_rxdrop++;                            /* ring full (or wedged): drop, like HW */
      }
    }
  }
  return did_work;
}

static inline bool enet_arm_poll(Driver *d, enet_tap *tap, uint8_t *dram) {
  g_dma_drv = d;                                  /* DMALOG sync records read the cycle counter */
  if(!tap->ok()) { return false; }
  if(g_enetengine) {
    return enet_arm_poll_engine(d, tap, dram);
  }
  static uint32_t last_tx_seq = 0, last_arm_seq = 0, rx_rsp_seq = 0, rx_nbdp = 0;
  bool did_work = false;

  /* ---- TX doorbell: assemble the frame from the {BP,BC,DP} chain -> tap ---- */
  /* ENETDBG: split guest-side (doorbell never rings) from ARM-side (rings but the
   * chain walk yields nothing).  Read-once getenv -- this is a hot poll loop. */
  static const bool g_enet_dbg = getenv("ENETDBG") != nullptr;
  static uint32_t tx_defer = 0;                 /* consecutive torn-chain deferrals */
  uint32_t tx_seq = d->read32(ENET_R_TX_REQ_SEQ);
  if(g_enet_dbg) {
    static uint64_t polls = 0;
    if((++polls % 2000000ull) == 0) {
      fprintf(stderr, "[enetdbg] poll=%llu tx_req_seq=%u last=%u rx_nbdp=0x%08x\n",
              (unsigned long long)polls, tx_seq, last_tx_seq, rx_nbdp);
    }
  }
  if(tx_seq != last_tx_seq) {
    uint32_t nbdp = d->read32(ENET_R_TX_NBDP);
    /* Publish the chain's FIRST descriptor as the TX CRBDP the instant the doorbell is
     * seen, BEFORE walking.  if_ecintr can run at any point in the in-flight window --
     * an RX interrupt arriving while TX is still ACTIVE -- and bounds its reap walk
     * with this register.  Until now it still held the PREVIOUS pass's tx_end; measured
     * in interp_mips that bound lands up to 96 descriptors AHEAD of the DMA, so the walk
     * passes descriptors that have not been transmitted yet.  The chain head is always
     * conservative: everything before it is genuinely done, nothing after it is.
     * (History: while this register was never written at all the guest computed a
     * constant 0xa0000000, which can never equal a descriptor pointer, so the reap loop
     * overran the ring and faulted at PC 0x88058338.  Publishing it once per pass fixed
     * the common case but left this in-flight window.) */
    d->write32(ENET_W_TX_CRBDP, nbdp);
    /* Snapshot the chain BEFORE the walk: enet_tx_run OR-writes ETXD into every
     * descriptor it touches, so a post-walk read cannot show the guest's value. */
    if(g_enet_dbg) {
      /* Linear dump of the whole 16-entry TX ring (stride 0x80), ignoring EOX, so
       * we can see whether the zeroed cntinfo is one descriptor or all of them. */
      for(int k = 0; k < 16; k++) {
        uint32_t a = nbdp + (uint32_t)(k * 0x80);
        uint8_t *dsc = enet_arm_mem(dram, a, 16);
        if(!dsc) {
          fprintf(stderr, "[enetdbg]   ring[%2d] @0x%08x -> NULL\n", k, a);
          break;
        }
        fprintf(stderr, "[enetdbg]   ring[%2d] @0x%08x bp=0x%08x cntinfo=0x%08x next=0x%08x w3=0x%08x\n",
                k, a, hdma_be32(dsc + 0), hdma_be32(dsc + 4), hdma_be32(dsc + 8), hdma_be32(dsc + 12));
      }
    }
    /* Race check: does the guest's cntinfo show up LATE?  If the uncached tx_ctrl
     * store (the doorbell) can overtake the descriptor writeback, the value will
     * appear a few ms after the kick.  Poll before walking; report how long. */
    if(g_enet_dbg) {
      uint8_t *dsc = enet_arm_mem(dram, nbdp, 12);
      if(dsc) {
        uint32_t first = hdma_be32(dsc + 4);
        if(!(first & HPC3_BC_COUNT)) {
          int ms = 0;
          uint32_t v = first;
          for(; ms < 300; ms++) {
            usleep(1000);
            v = hdma_be32(dsc + 4);
            if(v != first) { break; }
          }
          fprintf(stderr, "[enetdbg]   wait: cntinfo 0x%08x -> 0x%08x after %d ms%s\n",
                  first, v, ms, (ms >= 300) ? " (TIMEOUT: never arrived)" : "");
        }
      }
    }
    uint32_t tx_end = nbdp;
    if(g_enetflush) {
      scsi_extflush(d, dram, 0, 0, false);       /* write back dirty frame data before we read it */
    }
    dma_ctx_set(DMA_DEV_ENET, DMA_DIR_RD, nbdp);
    uint32_t txlen = enet_tx_run(enet_arm_mem_log, dram, nbdp, tap->fd, &tx_end);
    if(g_enet_dbg) {
      fprintf(stderr, "[enetdbg] TX seq=%u nbdp=0x%08x -> len=%u\n", tx_seq, nbdp, txlen);
    }
    /* Settle window, NOT an indefinite hold.  A chain the guest is mid-write finishes
     * within a few polls; but some chains legitimately end on a bare EOX with no EOXP
     * (52 of 15750 observed), and holding those for long blocks TX and throttles the
     * whole bridge -- 20000 cost ~5x throughput.  A short window keeps the race fix
     * while letting a genuine EOX-only chain through almost immediately. */
    if(txlen == ENET_TX_INCOMPLETE && tx_defer < 16) {
      /* Torn chain: do NOT consume the doorbell and do NOT echo -- the shim keeps TX
       * ACTIVE and we re-walk on the next poll, once the guest has finished writing
       * the EOXP terminator.  Bounded so a genuinely malformed chain cannot wedge TX. */
      tx_defer++;
      /* We DID work (walked the chain); say so, or the caller's adaptive backoff
       * ramps to ~1 ms/iter and starves the bridge -- the same poll-rate starvation
       * that caused 82.9% packet loss in July.  Staying hot also means we re-walk
       * almost immediately, which is the whole point of deferring. */
      did_work = true;
      return did_work;
    }
    /* Chain complete: every descriptor up to tx_end is transmitted, so the bound can
     * advance.  On the INCOMPLETE path above we deliberately leave CRBDP at the chain
     * head -- a torn chain has descriptors the guest must NOT reap yet. */
    d->write32(ENET_W_TX_CRBDP, tx_end);
    tx_defer = 0;
    last_tx_seq = tx_seq;
    __sync_synchronize();
    d->write32(ENET_W_TX_RSP_SEQ, tx_seq);        /* echo -> shim clears ACTIVE + TX IRQ */
    did_work = true;
  }

  /* ---- RX arm: (re)capture the ring head the driver just armed ---- */
  uint32_t arm_seq = d->read32(ENET_R_RX_ARM_SEQ);
  if(arm_seq != last_arm_seq) {
    last_arm_seq = arm_seq;
    rx_nbdp = d->read32(ENET_R_RX_NBDP);
  }

  /* ---- RX free-run: drain the tap, inject each frame into the guest RX ring ---- */
  if(rx_nbdp) {
    uint8_t fr[ENET_FRAME_MAX];
    for(int k = 0; k < 8; k++) {
      ssize_t n = ::read(tap->fd, fr, sizeof(fr));
      if(n < 14) { break; }                       /* EAGAIN / no frame / runt */
      /* Address filter: the Seeq's own filtering (rx_cmd match mode + station
       * MAC) isn't visible to the PS -- neither is routed over AXI.  But on this
       * point-to-point /24 tap every wanted frame is unicast-to-guest or
       * broadcast (ARP); the only inbound junk is the host's L2 multicast
       * (IPv6 ND '33:33:..', mDNS/IGMP '01:00:5e:..').  Drop non-broadcast
       * multicast (dst byte0 bit0 set, not the all-ones broadcast) so we don't
       * churn the guest RX ring with frames IRIX would just software-filter.
       * (A faithful enet_addr_filter would need the shim to export station +
       * rx_cmd -> a resynth; unicast is never dropped here, so it can't break
       * connectivity.) */
      bool is_bcast = (fr[0]&fr[1]&fr[2]&fr[3]&fr[4]&fr[5]) == 0xff;
      if((fr[0] & 1) && !is_bcast) { continue; }   /* stray multicast -> drop */
      uint32_t crbdp = 0;
      if(g_enetflush) {
        scsi_extflush(d, dram, 0, 0, false);     /* pre: no dirty line may land on the deposit */
      }
      dma_ctx_set(DMA_DEV_ENET, DMA_DIR_WR, (uint32_t)n);
      if(enet_rx_inject(enet_arm_mem_log, dram, rx_nbdp, fr, (uint32_t)n, crbdp)) {
        __sync_synchronize();                      /* frame in DRAM before we signal */
        if(g_enetflush) {
          scsi_extflush(d, dram, 0, 0, true);    /* post: drop lines refilled during the deposit */
        }
        d->write32(ENET_W_RX_CRBDP,   crbdp);      /* guest reads this at HPC3 0x18000 */
        d->write32(ENET_W_RX_RSP_SEQ, ++rx_rsp_seq);  /* -> shim raises RX channel IRQ */
        did_work = true;
      }
      /* ring full (inject returned false) -> drop the frame, like real hardware */
    }
  }
  return did_work;
}

#endif /* ENET_ARM_H */
