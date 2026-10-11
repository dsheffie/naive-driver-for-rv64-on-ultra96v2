/* ---- enet_arm.h RX/TX core, rewritten for the enet_dma ENGINE -------------------
 * WAS: enet_rx_inject()/enet_tx_run() walked the {BP,BC,DP} ring and wrote guest
 * DRAM DIRECTLY through the mmap, bypassing the RTL entirely -- so ENET traffic was
 * invisible to the L1/L2 and coherence rested on the sgiseeq driver remembering to
 * call dma_cache_inv.  That is unfixable once the L1 exceeds the page size (a
 * physical line then lives at several L1 indices and the kernel can only name one).
 * NOW: the RTL engine owns the ring walk and every memory access; the PS is just the
 * media, exactly like the disk.  Frame bytes cross as 16-byte beats over S00_AXI.
 *
 * Register map -- MUST match ip_hdl/axi_is_the_worst_v1_0_S00_AXI.v:
 *   W 0x2a-0x2d RX beat words 0-3 (writing 0x2d pushes)  W 0x2e RX go, [13:0]=len
 *   W 0x2f TX beat pop
 *   R 0x02 rx_beat_full   R 0x05 status {tx_done,rx_dropped,rx_done} READ-TO-CLEAR
 *   R 0x06 crbdp          R 0x20-0x23 TX beat words 0-3   R 0x24 tx_beat_valid
 */
enum {
  ENET_W_RXB0 = 0x2a, ENET_W_RXB1 = 0x2b, ENET_W_RXB2 = 0x2c, ENET_W_RXB3 = 0x2d,
  ENET_W_RX_GO = 0x2e, ENET_W_TX_POP = 0x2f,
  ENET_R_RXB_FULL = 0x02, ENET_R_STATUS = 0x05, ENET_R_CRBDP = 0x06,
  ENET_R_TXB0 = 0x20, ENET_R_TXB1 = 0x21, ENET_R_TXB2 = 0x22, ENET_R_TXB3 = 0x23,
  ENET_R_TXB_VALID = 0x24,
};
enum { ENET_ST_RX_DONE = 1u, ENET_ST_RX_DROP = 2u, ENET_ST_TX_DONE = 4u };

/* Push one frame to the engine.  `buf` is ALREADY formatted [2 pad][frame][1 status]
 * -- the engine writes those bytes verbatim to BP and does the beat/mask arithmetic.
 * Beats are fed INCREMENTALLY: `go` fires first and the engine stalls in S_RX_BEAT
 * until they arrive (a 1514B frame fits no sane FIFO).  Returns true if delivered. */
static inline bool enet_arm_push_frame(Driver *d, const uint8_t *buf, uint32_t len) {
  d->write32(ENET_W_RX_GO, len & 0x3fffu);            /* start the ring walk */
  uint32_t nbeats = (len + 15u) / 16u;
  for(uint32_t b = 0; b < nbeats; b++) {
    for(int spin = 0; d->read32(ENET_R_RXB_FULL) & 1u; spin++) {
      if(spin > 100000) { return false; }             /* engine wedged -> bail */
    }
    uint32_t w[4] = {0,0,0,0};
    for(int k = 0; k < 16; k++) {
      uint32_t off = b*16u + (uint32_t)k;
      uint8_t  v   = (off < len) ? buf[off] : 0u;
      w[3 - (k/4)] |= ((uint32_t)v) << (8 * (3 - (k & 3)));   /* MSB-first, BE */
    }
    d->write32(ENET_W_RXB0, w[0]); d->write32(ENET_W_RXB1, w[1]);
    d->write32(ENET_W_RXB2, w[2]); d->write32(ENET_W_RXB3, w[3]);  /* 0x2d pushes */
  }
  for(int spin = 0; spin < 200000; spin++) {          /* wait for the engine verdict */
    uint32_t st = d->read32(ENET_R_STATUS);           /* READ-TO-CLEAR: read once */
    if(st & ENET_ST_RX_DONE) { return true; }
    if(st & ENET_ST_RX_DROP) { return false; }        /* ring full, like real HW */
  }
  return false;
}

/* Drain TX beats the engine produced while walking the chain.  Returns bytes gathered.
 * BUG (RTL, 2026-08-06): tx_done at status bit2 is the engine's RAW 1-CYCLE PULSE --
 * it is NOT latched sticky like rx_done/rx_dropped, so this poll WILL miss it.  Until
 * S00_AXI latches it, treat "tx_beat_valid went empty after >=1 beat" as end-of-frame. */
static inline uint32_t enet_arm_drain_tx(Driver *d, uint8_t *buf, uint32_t cap) {
  uint32_t len = 0;
  for(int idle = 0; idle < 2000 && len + 16 <= cap; ) {
    if(!(d->read32(ENET_R_TXB_VALID) & 1u)) {
      /* WAIT for the FIRST beat too.  This is only ever called from inside
       * 	x_seq != last_tx_seq\, so a transmit IS pending -- but the engine still
       * has to walk the {BP,BC,DP} chain before it can push beat 0.  The original
       * \else break\ gave up instantly on an empty FIFO, returned 0, and the
       * caller then echoed tx_rsp_seq anyway: the shim cleared ACTIVE, the doorbell
       * advanced, and the frame was silently DROPPED.  Symptom on silicon: the guest
       * counted Opkts with Oerrs=0 while tap0 RX never moved. */
      idle++;
      continue;
    }
    idle = 0;
    uint32_t w[4] = { d->read32(ENET_R_TXB0), d->read32(ENET_R_TXB1),
                      d->read32(ENET_R_TXB2), d->read32(ENET_R_TXB3) };
    for(int k = 0; k < 16; k++) {
      buf[len + k] = (uint8_t)(w[3 - (k/4)] >> (8 * (3 - (k & 3))));
    }
    len += 16;
    d->write32(ENET_W_TX_POP, 1);
  }
  return len;
}
