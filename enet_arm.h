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
#include "enet_service.h"   // enet_tap, enet_tx_run, enet_rx_inject (+ henry_scsi.h)
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

/* Poll the ENET mailbox once.  Call every iteration of the driver's main loop.
 * `dram` MUST be the DRAM mmap base (axi.cc `c_addr`), not the control-reg window.
 *
 * Ordering (matches the validated sim contract, scsi_arm.h): read the request AFTER
 * the doorbell so the guest's TX-descriptor cache-writebacks have drained; for RX,
 * deposit the frame into DRAM, memory-barrier, then write crbdp + bump rsp_seq LAST
 * so the shim raises the IRQ only after the frame (and CRBDP) are visible. */
static inline bool enet_arm_poll(Driver *d, enet_tap *tap, uint8_t *dram) {
  if(!tap->ok()) { return false; }
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
    uint32_t txlen = enet_tx_run(enet_arm_mem, dram, nbdp, tap->fd, &tx_end);
    /* Publish the TX CRBDP BEFORE echoing the doorbell: the guest's completion handler
     * runs off the TX interrupt that the echo raises, and it needs a valid terminator
     * the moment it walks.  Unpublished (== 0) the guest computed 0xa0000000, which can
     * never equal a descriptor pointer, so its loop overran the ring and faulted in
     * m_freem's chain walk (PC 0x88058338). */
    d->write32(ENET_W_TX_CRBDP, tx_end);
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
      if(enet_rx_inject(enet_arm_mem, dram, rx_nbdp, fr, (uint32_t)n, crbdp)) {
        __sync_synchronize();                      /* frame in DRAM before we signal */
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
