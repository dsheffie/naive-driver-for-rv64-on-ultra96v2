/* ---- enet_arm.h RX/TX core, rewritten for the enet_dma ENGINE -------------------
 * WAS: enet_rx_inject()/enet_tx_run() walked the {BP,BC,DP} ring and wrote guest
 * DRAM DIRECTLY through the mmap, bypassing the RTL entirely -- so ENET traffic was
 * invisible to the L1/L2 and coherence rested on the sgiseeq driver remembering to
 * call dma_cache_inv.  That is unfixable once the L1 exceeds the page size (a
 * physical line then lives at several L1 indices and the kernel can only name one).
 * NOW: the RTL engine owns the ring walk and every memory access; the PS is just the
 * media, exactly like the disk.  Frame bytes cross as 16-byte beats over S00_AXI.
 * Enabled by ENETENGINE=1 (needs a bit built with ENABLE_ENET_DMA).
 *
 * Register map -- MUST match ip_hdl/axi_is_the_worst_v1_0_S00_AXI.v:
 *   W 0x2a-0x2d RX beat words 0-3 (writing 0x2d pushes)  W 0x2e RX go, [13:0]=len
 *   W 0x2f TX beat pop
 *   R 0x02 rx_beat_full   R 0x06 crbdp (RX desc just filled)
 *   R 0x05 status {tx_len[17:4], tx_done[2], rx_dropped[1], rx_done[0]}: the done bits
 *          are READ-TO-CLEAR; tx_len holds the exact byte count of the last TX chain.
 *   R 0x20-0x23 TX beat words 0-3   R 0x24[0] tx_beat_valid
 * Beat byte order: memory byte b of a beat is bits [8b+7:8b] of the 128-bit word, i.e.
 * byte k is word k/4, bits 8*(k&3) -- the DRAM/engine convention henry_tb uses.
 */
#ifndef ENET_ARM_ENGINE_H
#define ENET_ARM_ENGINE_H

enum {
  ENET_W_RXB0 = 0x2a, ENET_W_RXB1 = 0x2b, ENET_W_RXB2 = 0x2c, ENET_W_RXB3 = 0x2d,
  ENET_W_RX_GO = 0x2e, ENET_W_TX_POP = 0x2f,
  ENET_R_RXB_FULL = 0x02, ENET_R_STATUS = 0x05, ENET_R_CRBDP = 0x06,
  ENET_R_TXB0 = 0x20, ENET_R_TXB1 = 0x21, ENET_R_TXB2 = 0x22, ENET_R_TXB3 = 0x23,
  ENET_R_TXB_VALID = 0x24,
};
enum { ENET_ST_RX_DONE = 1u, ENET_ST_RX_DROP = 2u, ENET_ST_TX_DONE = 4u };

/* The status word is read-to-clear and shared by RX and TX, so whichever side reads
 * it must not lose the other side's bit: accumulate here, consumers clear their own. */
static uint32_t g_enet_st = 0, g_enet_st_txlen = 0;
static inline uint32_t enet_st_poll(Driver *d) {
  uint32_t v = d->read32(ENET_R_STATUS);
  g_enet_st |= v & 7u;
  if(v & ENET_ST_TX_DONE) {
    g_enet_st_txlen = (v >> 4) & 0x3fffu;          /* latched at tx_done, same read */
  }
  return g_enet_st;
}

/* TX collection state: the engine starts walking the chain on the guest's kick, on its
 * own; the ARM pops beats as they appear (non-blocking, across polls) and finishes the
 * frame once tx_done is seen AND the FIFO is empty. */
/* a chain can hold a whole TX ring of MSS segments: measured 23475 B, then 68578 B
 * (~45 segments).  The engine stops at 255 descriptors; 1 MB covers that at up to
 * 4 KB each, and an overflow is reported, never silently dropped. */
static uint8_t  g_etx_buf[(1u << 20) + 16];
static uint32_t g_etx_len = 0;
static bool     g_etx_ready = false;          /* a whole chain is in g_etx_buf */
static uint32_t g_etx_ready_len = 0;

/* Pop whatever beats are queued.  Returns true when the whole chain is in g_etx_buf;
 * g_etx_ready_len then holds the chain's exact byte count.  Also called from the RX
 * push loops: the engine is ONE state machine, so a TX chain stalled on a full beat
 * FIFO blocks the RX frame behind it until the ARM pops -- an RX push that does not
 * drain TX deadlocks against it. */
static inline bool enet_arm_collect_tx(Driver *d) {
  if(g_etx_ready) {
    return true;
  }
  for(;;) {
    if(!(d->read32(ENET_R_TXB_VALID) & 1u)) {
      break;
    }
    uint32_t w[4] = { d->read32(ENET_R_TXB0), d->read32(ENET_R_TXB1),
                      d->read32(ENET_R_TXB2), d->read32(ENET_R_TXB3) };
    d->write32(ENET_W_TX_POP, 1);
    if(g_etx_len + 16 <= sizeof(g_etx_buf)) {
      for(int k = 0; k < 16; k++) {
        g_etx_buf[g_etx_len + k] = (uint8_t)(w[k/4] >> (8 * (k & 3)));
      }
      g_etx_len += 16;
    }
    else {
      fprintf(stderr, "[eneteng] TX beat buffer OVERFLOW at %u bytes\n", g_etx_len);
    }
  }
  /* done is latched only after the last beat was pushed, but re-check the FIFO after
   * seeing it: a beat may have landed between the drain above and the status read */
  if(!(enet_st_poll(d) & ENET_ST_TX_DONE) || (d->read32(ENET_R_TXB_VALID) & 1u)) {
    return false;
  }
  g_enet_st &= ~ENET_ST_TX_DONE;
  /* the engine's tx_len is 14 bits and wraps on a big chain; no beat is ever lost
   * (the engine stalls on a full FIFO), so the beat count pins the length to a 16 B
   * window and tx_len picks the byte within it */
  uint32_t len = g_enet_st_txlen;
  while(((len + 15u) & ~15u) < g_etx_len) {
    len += 16384u;
  }
  g_etx_ready_len = len;
  g_etx_ready = true;
  return true;
}

/* Push one frame to the engine.  `buf` is ALREADY formatted [2 pad][frame][1 status]
 * -- the engine writes those bytes verbatim to BP and does the beat/mask arithmetic.
 * Beats are fed INCREMENTALLY: `go` fires first and the engine stalls in S_RX_BEAT
 * until they arrive (a 1514B frame fits no sane FIFO).  Returns true if delivered. */
static inline bool enet_arm_push_frame(Driver *d, const uint8_t *buf, uint32_t len) {
  d->write32(ENET_W_RX_GO, len & 0x3fffu);            /* start the ring walk */
  uint32_t nbeats = (len + 15u) / 16u;
  for(uint32_t b = 0; b < nbeats; b++) {
    for(int spin = 0; d->read32(ENET_R_RXB_FULL) & 1u; spin++) {
      enet_arm_collect_tx(d);                         /* the engine may be mid-TX */
      if(spin > 1000000) {
        /* the beat stream is now out of step with the engine: loud, not a "drop" */
        fprintf(stderr, "[eneteng] RX push WEDGED at beat %u/%u\n", b, nbeats);
        return false;
      }
    }
    uint32_t w[4] = {0,0,0,0};
    for(int k = 0; k < 16; k++) {
      uint32_t off = b*16u + (uint32_t)k;
      uint8_t  v   = (off < len) ? buf[off] : 0u;
      w[k/4] |= ((uint32_t)v) << (8 * (k & 3));
    }
    d->write32(ENET_W_RXB0, w[0]); d->write32(ENET_W_RXB1, w[1]);
    d->write32(ENET_W_RXB2, w[2]); d->write32(ENET_W_RXB3, w[3]);  /* 0x2d pushes */
  }
  for(int spin = 0; spin < 2000000; spin++) {         /* wait for the engine verdict */
    enet_arm_collect_tx(d);
    uint32_t st = enet_st_poll(d);
    if(st & ENET_ST_RX_DONE) {
      g_enet_st &= ~ENET_ST_RX_DONE;
      return true;
    }
    if(st & ENET_ST_RX_DROP) {
      g_enet_st &= ~ENET_ST_RX_DROP;
      return false;                                   /* ring full, like real HW */
    }
  }
  fprintf(stderr, "[eneteng] RX verdict never came (len %u)\n", len);
  return false;
}

#endif /* ENET_ARM_ENGINE_H */
