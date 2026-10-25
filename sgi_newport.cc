/* sgi_newport.cc -- SGI Newport (XL8/XL24) graphics, served by the ARM for henry's
 * GIO window (docs/newport-on-henry.md).  Copied from indysim's model of the same
 * name; only the host interface differs (np_host_t instead of the ISS state_t).
 *
 * A condensed port of MAME src/devices/bus/gio64/newport.cpp (BSD-3-Clause,
 * copyright Ryan Holtz): REX3 register file + drawing engine, the DCB slaves
 * (VC2, XMAP9 x2, CMAP x2, Bt445 RAMDAC) and the RB2 framebuffer.  Like MAME,
 * every draw completes instantly and the GFIFO/BFIFO always read empty, so
 * STATUS shows GFXBUSY only while an MC VDMA transfer's host data is still arriving
 * (set_in_dma), and BACKBUSY only for the DCB timeout below.
 *
 * REX3 is a 64-bit device: MAME's rex3_r/rex3_w take the 8-byte line plus which
 * 32-bit half is touched (big-endian: bits 63:32 live at line+0, 31:0 at
 * line+4).  A byte/half access places its value in that half's byte lane with
 * the other bytes zero (= MAME's data/mem_mask), except the host data port,
 * which merges.  Any access to line+0x800 ("GO") runs the programmed command
 * after the register access.
 *
 * Display: render() walks the VC2 DID tables per scanline and the XMAP9 mode
 * table per DID, like MAME's screen_update, then applies the RAMDAC LUT.  The
 * Bt445 LUT starts as identity (MAME starts it at zero, which relies on the
 * real PROM loading it; henry's ARCS PROM does not).  Vertical retrace fires
 * every vblank_period host time units (the host drives vblank()) and is raised on INT3
 * local1 bit 0x80 when VC2 display-control bit0 enables it; reading REX3
 * STATUS acknowledges it, as on MAME. */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <map>
#include <string>
#include "sgi_newport.hh"

#define BITN(x, n) (((x) >> (n)) & 1u)

#ifdef NP_PROF
/* -DNP_PROF (replay tools only): raw tick counters, read by tools/np_replay */
uint64_t g_np_prof_cmd = 0, g_np_prof_pix = 0, g_np_prof_rows = 0;
/* NP_FBROWS=R folds framebuffer rows mod R (timing experiments only: the result is
 * wrong, the instruction stream is the same, the working set fits the L2) */
static const int g_np_fbrows = getenv("NP_FBROWS") ? atoi(getenv("NP_FBROWS")) : 0;
#define NP_ROW(y) (g_np_fbrows ? ((y) % g_np_fbrows) : (y))
#define NP_TICK() np_ticks()
#define NP_ACC(var, t0) (var += np_ticks() - (t0))
#else
#define NP_ROW(y) (y)
#define NP_TICK() 0
#define NP_ACC(var, t0) (void)(t0)
#endif

static const bool g_np_log = getenv("NEWPORT_LOG") != nullptr;
/* NEWPORT_LOG=<n>: trace the first n register accesses; counts per register at exit */
static const uint64_t g_np_trace_n = g_np_log ? strtoull(getenv("NEWPORT_LOG"), nullptr, 0) : 0;
static uint64_t g_np_acc = 0, g_np_cmds = 0;
static uint64_t g_np_rd[0x400], g_np_wr[0x400];
/* GO commands by kind: {opcode, adrmode, host colour, shade, rgb, blend, depth, hostdepth, pattern, fastclear} */
static std::map<uint32_t, uint64_t> g_np_kinds;

enum {
  DCB_VC2 = 0, DCB_CMAP01 = 1, DCB_CMAP0 = 2, DCB_CMAP1 = 3, DCB_XMAP01 = 4,
  DCB_XMAP0 = 5, DCB_XMAP1 = 6, DCB_RAMDAC = 7, DCB_CC1 = 8, DCB_AB1 = 9, DCB_PCD = 12
};
enum {
  STATUS_GFXBUSY = (1u << 3),
  STATUS_BACKBUSY = (1u << 4),
  STATUS_VRINT = (1u << 5)
};
/* An access to an absent DCB slave (the Presenter flat-panel adapter) never gets
 * DCB_ACK_N, so BACKBUSY stays up.  IRIX tells "absent" from "present" only in
 * ng1_i2cProbe: still BACKBUSY after 5 STATUS reads = absent (then a DCBRESET write).
 * Every other PCD access -- the retrace handler re-probes the panel every 64 frames
 * via fp_reinit -> ng1_i2cPanelID -- spins on BACKBUSY with no bound, so on real
 * hardware it must clear soon after.  The REX3 spec gives no timeout; MAME uses 1 ms
 * of wall clock, which cost ~4 ms of core per second under Doom.  So count STATUS
 * reads instead: clear on the DCB_TIMEOUT_READS'th, past the probe's 5, and
 * independent of how long a read takes on the host. */
static const uint32_t DCB_TIMEOUT_READS = 8;
static const uint32_t s_host_shifts[4] = { 8, 8, 16, 32 };

/* ---------------- trace (NEWPORT_TRACE=<file>) ----------------
 * type 0 access: size = bytes, rw = 0 read / 1 write (| 2 when issued by the MC
 *        graphics DMA), offset = board offset, value = the access as the CPU sees it
 * type 1 GO:     aux = opcode | adrmode << 2, offset = DRAWMODE0, value = pixels
 *        the stepper produced (drawn, read back or copied)
 * type 2 DMA:    rw = 1 to host (board -> memory) / 0 from host, offset = GIO
 *        address, value = bytes moved */
struct __attribute__((packed)) trace_rec_t {
  uint8_t type, size, rw, aux;
  uint32_t offset;
  uint64_t icnt, value;
};
static FILE *trace_open() {
  const char *p = getenv("NEWPORT_TRACE");
  if(p == nullptr) {
    return nullptr;
  }
  FILE *f = fopen(p, "wb");
  if(f == nullptr) {
    fprintf(stderr, "[newport] cannot write trace %s\n", p);
    return nullptr;
  }
  static char buf[1 << 20];
  setvbuf(f, buf, _IOFBF, sizeof(buf));
  fprintf(stderr, "[newport] tracing to %s\n", p);
  return f;
}
static FILE *const g_trace = trace_open();
static inline void trace_put(uint8_t type, uint8_t size, uint8_t rw, uint8_t aux, uint32_t offset,
                             uint64_t icnt, uint64_t value) {
  const trace_rec_t r = { type, size, rw, aux, offset, icnt, value };
  fwrite(&r, sizeof(r), 1, g_trace);
}

/* ---------------- per-GO draw capture (NEWPORT_DRAWCAP=<file>) ----------------
 * A zstd-compressed stream of records, version 1, host byte order:
 *   header  u32 magic 'NPGO' (0x4f47504e), u16 version (1), u16 kind
 *   kind 1  GO:  u64 icnt, u32 guest pc of the GO access, u8 asid, u8 opcode,
 *                u8 adrmode, u8 0, u64 pixels, u32 rex_size,
 *                rex_t before [rex_size], rb_snap_t before (16 B),
 *                u32 n, touch_t[n] {u32 addr (bit31 = cidaux), u32 old, u32 new},
 *                rex_t after [rex_size], rb_snap_t after
 *            Every framebuffer word the GO read or wrote appears once with its
 *            pre-image, so record + replay is self-contained: the GO's host data
 *            (host_dataport + shift) and readback results live in rex_t.
 *   kind 2  checksum: u64 icnt, u64 GOs so far, u64 FNV-1a(rgbci), u64 FNV-1a(cidaux)
 *            (every NEWPORT_DRAWCAP_SUM GOs, default 20000, and at exit) */
static FILE *drawcap_open() {
  const char *p = getenv("NEWPORT_DRAWCAP");
  if(p == nullptr) {
    return nullptr;
  }
  const std::string cmd = std::string("zstd -q -3 -T0 -f -o '") + p + "'";
  FILE *f = popen(cmd.c_str(), "w");
  if(f == nullptr) {
    fprintf(stderr, "[newport] cannot start %s\n", cmd.c_str());
    return nullptr;
  }
  static char buf[1 << 20];
  setvbuf(f, buf, _IOFBF, sizeof(buf));
  fprintf(stderr, "[newport] draw capture to %s\n", p);
  return f;
}
static FILE *const g_drawcap = drawcap_open();
static const uint64_t g_drawcap_sum = getenv("NEWPORT_DRAWCAP_SUM") ? strtoull(getenv("NEWPORT_DRAWCAP_SUM"), nullptr, 0) : 20000;
static uint64_t g_drawcap_n = 0;
static const uint32_t DRAWCAP_MAGIC = 0x4f47504e;

static uint64_t fnv64(const uint32_t *p, size_t n) {
  uint64_t h = 1469598103934665603ull;
  const uint8_t *b = reinterpret_cast<const uint8_t *>(p);
  for(size_t i = 0; i < n * 4; i++) {
    h = (h ^ b[i]) * 1099511628211ull;
  }
  return h;
}

static void cap_hdr(uint16_t kind) {
  const uint32_t m = DRAWCAP_MAGIC;
  const uint16_t v = 1;
  fwrite(&m, 4, 1, g_drawcap);
  fwrite(&v, 2, 1, g_drawcap);
  fwrite(&kind, 2, 1, g_drawcap);
}

size_t sgi_newport::rex_size() {
  return sizeof(rex3_t);
}

void sgi_newport::get_state(uint8_t *rex_out, rb_snap_t *r) const {
  memcpy(rex_out, &rex, sizeof(rex));
  memset(r, 0, sizeof(*r));
  r->global_mask = rb.global_mask;
  r->write_mask = rb.write_mask;
  r->blend = rb.blend;
  r->fast_clear = rb.fast_clear;
  r->plane_enable = rb.plane_enable;
  r->draw_depth = rb.draw_depth;
  r->logicop = rb.logicop;
  r->src_shift = rb.src_shift;
  r->dest_aux = (rb.dest == rb.cidaux) ? 1 : 0;
}

void sgi_newport::set_rb(const rb_snap_t &r) {
  rb.global_mask = r.global_mask;
  rb.write_mask = r.write_mask;
  rb.blend = r.blend;
  rb.fast_clear = r.fast_clear;
  rb.plane_enable = r.plane_enable;
  rb.draw_depth = r.draw_depth;
  rb.logicop = r.logicop;
  rb.src_shift = r.src_shift;
  rb.dest = r.dest_aux ? rb.cidaux : rb.rgbci;
  rb.ptr = rb.dest;
}

void sgi_newport::set_state(const uint8_t *rex_in, const rb_snap_t &r) {
  memcpy(&rex, rex_in, sizeof(rex));
  disp_gen.fetch_add(1, std::memory_order_release);
  set_rb(r);
}

void sgi_newport::touch_note(const uint32_t *p) {
  const bool aux = (p >= rb.cidaux && p < rb.cidaux + FB_W * FB_H);
  const uint32_t a = (uint32_t)(p - (aux ? rb.cidaux : rb.rgbci)) | (aux ? 0x80000000u : 0);
  if(touched_idx.emplace(a, (uint32_t)touched.size()).second) {
    touched.push_back({a, *p, 0});
  }
}

const std::vector<sgi_newport::touch_t> &sgi_newport::touch_end() {
  touching = false;
  for(auto &t : touched) {
    t.new_v = *fb_word(t.addr);
  }
  return touched;
}

void sgi_newport::trace_dma(uint64_t icnt, bool to_host, uint32_t gio, uint32_t bytes) {
  if(g_trace) {
    trace_put(2, 0, to_host ? 1 : 0, 0, gio, icnt, bytes);
  }
}

/* ---------------- XMAP9 ---------------- */
uint32_t sgi_newport::xmap9_t::read(uint32_t r) {
  switch(r)
    {
    case 0: return config;
    case 1: return revision;
    case 2: return 2;                          /* FIFO availability */
    case 3: return cursor_cmap;
    case 4: return popup_cmap;
    case 5: {
      const uint32_t m = mode[(mode_idx & 0x7c) >> 2];
      switch(mode_idx & 3)
        {
        case 0: return (m >> 16) & 0xff;
        case 1: return (m >> 8) & 0xff;
        case 2: return m & 0xff;
        default: return 0;
        }
    }
    case 7: return mode_idx;
    default: return 0;
    }
}

void sgi_newport::xmap9_t::write(uint32_t r, uint32_t d) {
  switch(r)
    {
    case 0: config = d & 0xff; break;
    case 3: cursor_cmap = d & 0xff; break;
    case 4: popup_cmap = d & 0xff; break;
    case 5: mode[(d >> 24) & 0x1f] = d & 0xffffff; break;
    case 7: mode_idx = d & 0xff; break;
    default: break;
    }
}

/* ---------------- CMAP ---------------- */
uint32_t sgi_newport::cmap_t::read(uint32_t r) {
  switch(r)
    {
    case 4: return status;
    case 6: return revision;
    default: return 0;
    }
}

void sgi_newport::cmap_t::write(uint32_t r, uint32_t d) {
  switch(r)
    {
    case 0: idx = (uint16_t)d; break;
    case 2: pal[idx] = d >> 8; break;      /* MAME does not auto-increment */
    default: break;
    }
}

/* ---------------- VC2 ---------------- */
static const uint32_t VC2_RAM_SIZE = 0x8000;

uint32_t sgi_newport::vc2_t::read(uint32_t r) {
  if(r == 1) {
    switch(reg_idx)
      {
      case 0x00: return vid_entry;
      case 0x01: return cursor_entry;
      case 0x02: return cursor_x;
      case 0x03: return cursor_y;
      case 0x04: return cur_cursor_x;
      case 0x05: return did_entry;
      case 0x06: return scanline_len;
      case 0x07: return ram_addr;
      case 0x08: return vt_frame_ptr;
      case 0x09: return vt_line_ptr;
      case 0x0a: return vt_line_run;
      case 0x0b: return vt_line_count;
      case 0x0c: return cursor_table_ptr;
      case 0x0d: return work_cursor_y;
      case 0x0e: return did_frame_ptr;
      case 0x0f: return did_line_ptr;
      case 0x10: return display_ctrl;
      case 0x1f: return config;
      default: return 0;
      }
  }
  if(r == 3) {
    const uint16_t v = ram[ram_addr];
    ram_addr = (ram_addr + 1) & (VC2_RAM_SIZE - 1);
    return v;
  }
  return 0;
}

void sgi_newport::vc2_t::write(uint32_t r, uint32_t d, uint32_t mask) {
  if(mask == 0xff) {                         /* 1-byte transfer: register select */
    if(r == 0) {
      reg_idx = (uint8_t)d;
    }
    return;
  }
  if(mask == 0xffff) {                       /* 2-byte transfer: RAM word */
    if(r == 3) {
      ram[ram_addr] = (uint16_t)d;
      ram_addr = (ram_addr + 1) & (VC2_RAM_SIZE - 1);
    }
    return;
  }
  if(r != 0) {                               /* 4/3-byte transfer: {index, data16, x} */
    return;
  }
  reg_idx = (uint8_t)(d >> 24);
  const uint16_t v = (uint16_t)(d >> 8);
  switch(reg_idx)
    {
    case 0x00: vid_entry = v; update_screen_size(); break;
    case 0x01: cursor_entry = v; break;
    case 0x02: cursor_x = v; break;
    case 0x03: cursor_y = v; cur_cursor_x = cursor_x; break;
    case 0x04: cur_cursor_x = v; break;
    case 0x05: did_entry = v; break;
    case 0x06: scanline_len = v; break;
    case 0x07: ram_addr = v & (VC2_RAM_SIZE - 1); break;
    case 0x08: vt_frame_ptr = v; break;
    case 0x09: vt_line_ptr = v; break;
    case 0x0a: vt_line_run = v; break;
    case 0x0b: vt_line_count = v; break;
    case 0x0c: cursor_table_ptr = v; break;
    case 0x0d: work_cursor_y = v; break;
    case 0x0e: did_frame_ptr = v; break;
    case 0x0f: did_line_ptr = v; break;
    case 0x10:
      display_ctrl = v;
      /* cursor function + cursor enable, glyph mode */
      cursor_on = BITN(v, 4) && BITN(v, 7) && !BITN(v, 8);
      break;
    case 0x1f: config = v; break;
    default: break;
    }
}

/* Find the first/last displayed line from the video timing tables: a line is
 * displayed when any run of its line sequence has the "active video" state bit
 * (bit 6 of the low state byte -- MAME's decode_vt_line/update_screen_size,
 * which test bit 7 of the doubled state).  Lines within one sequence entry
 * share a pattern, so each entry is decoded once. */
static bool vt_line_active(const uint16_t *ram, uint32_t p) {
  uint32_t sc = 0;
  bool eol_sa = false, eol_sb = false;
  for(int guard = 0; guard < 4096 && !(eol_sa && eol_sb); guard++) {
    const uint16_t w = ram[p & (VC2_RAM_SIZE - 1)];
    eol_sa = BITN(w, 15) != 0;
    if(!BITN(w, 7)) {
      const uint16_t w1 = ram[(p + 1) & (VC2_RAM_SIZE - 1)];
      eol_sb = BITN(w1, 15) != 0;
      sc = w1 & 0x7f;
      p += 2;
    }
    else {
      eol_sb = true;
      p++;
    }
    const uint32_t run = (ram[p & (VC2_RAM_SIZE - 1)] & 0x7f00) >> 7;
    if(run != 0 && BITN(sc, 6)) {
      return true;
    }
  }
  return false;
}

void sgi_newport::vc2_t::update_screen_size() {
  readout_y0 = 0;
  readout_y1 = 0;
  uint32_t entry = vid_entry, line = 0;
  bool y_started = false, y_done = false;
  for(int guard = 0; guard < 4096 && !y_done; guard++) {
    const uint32_t seq_ptr = ram[entry & (VC2_RAM_SIZE - 1)];
    const uint32_t seq_len = ram[(entry + 1) & (VC2_RAM_SIZE - 1)];
    if(seq_len == 0) {
      break;
    }
    const bool active = vt_line_active(ram, seq_ptr);
    for(uint32_t i = 0; i < seq_len && line < 2048; i++, line++) {
      if(active && !y_started) {
        y_started = true;
        readout_y0 = (int)line;
      }
      else if(!active && y_started && !y_done) {
        readout_y1 = (int)line;
        y_done = true;
      }
    }
    entry += 2;
  }
  if(y_started && !y_done) {
    readout_y1 = (int)line;                    /* active through the end of the table */
  }
  if(g_np_log) {
    fprintf(stderr, "[newport] vc2 video entry %04x -> active lines %d..%d\n", vid_entry, readout_y0, readout_y1);
  }
}

uint8_t sgi_newport::vc2_t::cursor_pixel(int x, int y) {
  if(!cursor_on) {
    return 0;
  }
  if(x < (int)cursor_x - 31 || x > (int)cursor_x || y < (int)cursor_y - 31 || y > (int)cursor_y) {
    return 0;
  }
  x -= (int)cursor_x - 31;
  y -= (int)cursor_y - 31;
  const bool mono64 = BITN(display_ctrl, 9) != 0;
  const int size = mono64 ? 64 : 32;
  if(x < 0 || y < 0 || x >= size || y >= size) {
    return 0;
  }
  const int shift = 15 - (x % 16);
  if(mono64) {
    return BITN(ram[(cursor_entry + y * 4 + x / 16) & (VC2_RAM_SIZE - 1)], shift);
  }
  const uint32_t a = cursor_entry + y * 2 + x / 16;
  return BITN(ram[a & (VC2_RAM_SIZE - 1)], shift) | (BITN(ram[(a + 64) & (VC2_RAM_SIZE - 1)], shift) << 1);
}

/* ---------------- board ---------------- */
sgi_newport::sgi_newport(const np_host_t &host, uint64_t vblank_period) : host(host), vblank_period(vblank_period) {
  static const bool g_xl24 = getenv("NEWPORT_XL24") != nullptr;
  if(this->vblank_period == 0) {
    this->vblank_period = 1000000ull;
  }
  next_vblank = this->vblank_period;
  memset(&rex, 0, sizeof(rex));
  rex.draw_mode1 = 0x3002f001;
  rex.dcb_mode = 0x00000780;
  rex.x_window = 0x1000;
  rex.y_window = 0x1000;
  rb.rgbci = new uint32_t[FB_W * FB_H]();
  rb.cidaux = new uint32_t[FB_W * FB_H]();
  rb.dest = rb.rgbci;
  rb.ptr = rb.rgbci;
  rb.global_mask = g_xl24 ? 0x00ffffffu : 0x000000ffu;
  for(int i = 0; i < 2; i++) {
    xmap[i].revision = g_xl24 ? 3 : 1;
    /* CMAP revision byte, as the IP24 PROM and Linux newport_con decode it: bit 7 = 8-bit
     * board, bits 6:4 = board revision, bits 2:0 = CMAP revision.  The PROM only trusts
     * the 8/24-bit flag (and sets up 1280x1024) at board revision >= 2: MAME's XL24 0x02
     * (rev 0) gave a 1024x768 setup, and 0xa2 (bit 7 set) made hinv say "Indy 8-bit".
     * CMAP1's upper nibble is the monitor-ID sense instead (0xa = 1280x1024).
     * Keep in step with indysim sgi_newport.cc. */
    cmap[i].revision = g_xl24 ? (i == 0 ? 0x22 : 0xa2) : 0xa1;
    cmap[i].pal = new uint32_t[0x10000]();
  }
  vc2.ram = new uint16_t[VC2_RAM_SIZE]();
  for(uint32_t i = 0; i < 256; i++) {          /* identity gamma (see header comment) */
    lut_r[i] = i << 16;
    lut_g[i] = i << 8;
    lut_b[i] = i;
  }
  rb_set_flags(rex.draw_mode1);
  fprintf(stderr, "[newport] %s at 0x%08x, vblank every %llu\n", g_xl24 ? "XL24" : "XL8", BASE,
          (unsigned long long)vblank_period);
}

void sgi_newport::drawcap_checksum() {
  cap_hdr(2);
  const uint64_t v[4] = { now(), g_drawcap_n, fnv64(rb.rgbci, FB_W * FB_H), fnv64(rb.cidaux, FB_W * FB_H) };
  fwrite(v, 8, 4, g_drawcap);
}

sgi_newport::~sgi_newport() {
  if(g_drawcap) {
    drawcap_checksum();
    pclose(g_drawcap);
  }
  if(g_np_log) {
    fprintf(stderr, "[newport] %llu GO commands; by kind:\n", (unsigned long long)g_np_cmds);
    static const char *ops[4] = {"noop", "read", "draw", "scr2scr"};
    static const char *adr[8] = {"span", "block", "iline", "fline", "aline", "adr5", "adr6", "adr7"};
    for(const auto &kv : g_np_kinds) {
      const uint32_t k = kv.first;
      fprintf(stderr, "[newport]   %-7s %-5s%s%s%s%s%s%s%s depth=%u hostdepth=%u : %llu\n", ops[k & 3], adr[(k >> 2) & 7],
              BITN(k, 5) ? " setup" : "", BITN(k, 6) ? " host" : "", BITN(k, 7) ? " shade" : "", BITN(k, 8) ? " rgb" : "",
              BITN(k, 9) ? " blend" : "", ((k >> 14) & 3) ? " pattern" : "", BITN(k, 16) ? " fastclear" : "", (k >> 10) & 3, (k >> 12) & 3,
              (unsigned long long)kv.second);
    }
    fprintf(stderr, "[newport] register access counts (offset: reads/writes):\n");
    for(uint32_t i = 0; i < 0x400; i++) {
      if(g_np_rd[i] || g_np_wr[i]) {
        fprintf(stderr, "[newport]   %04x: %llu/%llu\n", i * 8, (unsigned long long)g_np_rd[i], (unsigned long long)g_np_wr[i]);
      }
    }
  }
  static const char *g_ppm = getenv("NEWPORT_PPM");
  if(g_ppm) {
    dump_ppm(g_ppm);
  }
  delete [] rb.rgbci;
  delete [] rb.cidaux;
  for(int i = 0; i < 2; i++) {
    delete [] cmap[i].pal;
  }
  delete [] vc2.ram;
}

void sgi_newport::vblank(uint64_t t) {
  next_vblank = t + vblank_period;
  nframes++;
  if(BITN(vc2.display_ctrl, 0)) {             /* VC2 vertical-retrace interrupt enable */
    rex.status |= STATUS_VRINT;
    set_vrint(true);
  }
  static const char *g_ppm = getenv("NEWPORT_PPM");
  static const uint64_t g_every = getenv("NEWPORT_PPM_EVERY") ? strtoull(getenv("NEWPORT_PPM_EVERY"), nullptr, 0) : 0;
  if(g_ppm && g_every && (nframes % g_every) == 0) {
    dump_ppm(g_ppm);
  }
}

/* ---------------- guest bus interface ---------------- */
uint64_t sgi_newport::read(uint32_t offs, size_t sz) {
  const uint32_t line = offs >> 3, b = offs & 7;
  const uint32_t shift = (8 - b - (uint32_t)sz) * 8;
  const uint64_t mask = (sz == 8) ? ~0ull : (((1ull << (sz * 8)) - 1) << shift);
  const uint64_t r = rex3_r(line, (mask >> 32) != 0, (uint32_t)mask != 0);
  const uint64_t v = (sz == 8) ? r : ((r >> shift) & ((1ull << (sz * 8)) - 1));
  if(g_trace) {
    trace_put(0, (uint8_t)sz, in_dma ? 2 : 0, 0, offs, now(), v);
  }
  switch(sz)
    {
    case 8: return __builtin_bswap64(v);
    case 4: return __builtin_bswap32((uint32_t)v);
    case 2: return __builtin_bswap16((uint16_t)v);
    default: return v;
    }
}

void sgi_newport::write(uint32_t offs, uint64_t v, size_t sz) {
  disp_gen.fetch_add(1, std::memory_order_release);
  uint64_t g = 0;
  switch(sz)
    {
    case 8: g = __builtin_bswap64(v); break;
    case 4: g = __builtin_bswap32((uint32_t)v); break;
    case 2: g = __builtin_bswap16((uint16_t)v); break;
    default: g = v & 0xff; break;
    }
  if(g_trace) {
    trace_put(0, (uint8_t)sz, in_dma ? 3 : 1, 0, offs, now(), g);
  }
  const uint32_t line = offs >> 3, b = offs & 7;
  const uint32_t shift = (8 - b - (uint32_t)sz) * 8;
  const uint64_t mask = (sz == 8) ? ~0ull : (((1ull << (sz * 8)) - 1) << shift);
  rex3_w(line, (sz == 8) ? g : (g << shift), mask, (mask >> 32) != 0, (uint32_t)mask != 0);
}

/* ---------------- DCB ---------------- */
uint32_t sgi_newport::dcb_read() {
  uint32_t r = 0;
  switch(rex.dcb_slave_select)
    {
    case DCB_VC2: r = vc2.read(rex.dcb_reg_select); break;
    case DCB_CMAP0: r = cmap[0].read(rex.dcb_reg_select); break;
    case DCB_CMAP1: r = cmap[1].read(rex.dcb_reg_select); break;
    case DCB_XMAP0: r = xmap[0].read(rex.dcb_reg_select); break;
    case DCB_XMAP1: r = xmap[1].read(rex.dcb_reg_select); break;
    case DCB_PCD:
      rex.status |= STATUS_BACKBUSY;
      backbusy_reads = DCB_TIMEOUT_READS;
      break;
    case DCB_RAMDAC:
    case DCB_CC1:
    case DCB_AB1:
      break;                                   /* not modelled */
    default:
      r = rex.dcb_data_msw;
      break;
    }
  if(BITN(rex.dcb_mode, 3)) {
    rex.dcb_reg_select = (rex.dcb_reg_select + 1) & 7;
  }
  return r;
}

void sgi_newport::dcb_write(uint32_t d) {
  rex.dcb_data_msw = d;
  switch(rex.dcb_slave_select)
    {
    case DCB_VC2: vc2.write(rex.dcb_reg_select, d, rex.dcb_mask); break;
    case DCB_CMAP01: cmap[0].write(rex.dcb_reg_select, d); cmap[1].write(rex.dcb_reg_select, d); break;
    case DCB_CMAP0: cmap[0].write(rex.dcb_reg_select, d); break;
    case DCB_CMAP1: cmap[1].write(rex.dcb_reg_select, d); break;
    case DCB_XMAP01: xmap[0].write(rex.dcb_reg_select, d); xmap[1].write(rex.dcb_reg_select, d); break;
    case DCB_XMAP0: xmap[0].write(rex.dcb_reg_select, d); break;
    case DCB_XMAP1: xmap[1].write(rex.dcb_reg_select, d); break;
    case DCB_RAMDAC:
      if(rex.dcb_reg_select == 0) {
        lut_idx = (uint8_t)d;
      }
      else if(rex.dcb_reg_select == 1) {
        lut_r[lut_idx] = ((d >> 8) & 0xff) << 16;
        lut_g[lut_idx] = ((d >> 16) & 0xff) << 8;
        lut_b[lut_idx] = (d >> 24) & 0xff;
        lut_idx++;
      }
      break;
    case DCB_PCD:
      rex.status |= STATUS_BACKBUSY;
      backbusy_reads = DCB_TIMEOUT_READS;
      break;
    default:
      break;
    }
  if(BITN(rex.dcb_mode, 3)) {
    rex.dcb_reg_select = (rex.dcb_reg_select + 1) & 7;
  }
}

/* ---------------- REX3 register file ---------------- */
uint64_t sgi_newport::rex3_r(uint32_t line, bool hi, bool lo) {
  uint32_t h = 0, l = 0;
  if(g_np_log) {
    g_np_rd[line & 0x3ff]++;
  }
  switch((line & ~0x100u) * 8)
    {
    case 0x0000: h = rex.draw_mode1; l = rex.draw_mode0; break;
    case 0x0008: h = rex.ls_mode; l = rex.ls_pattern; break;
    case 0x0010: h = rex.ls_pattern_saved; l = rex.z_pattern; break;
    case 0x0018: h = rex.color_back; l = rex.color_vram; break;
    case 0x0020: h = rex.alpha_ref; break;
    case 0x0028: h = rex.smask_x[0]; l = rex.smask_y[0]; break;
    case 0x0030: h = rex.setup; break;
    case 0x0100: h = (uint32_t)rex.x_start; l = (uint32_t)rex.y_start; break;
    case 0x0108: h = (uint32_t)rex.x_end; l = (uint32_t)rex.y_end; break;
    case 0x0110: h = (uint16_t)rex.x_save; l = rex.xy_move; break;
    case 0x0118: h = rex.bres_d; l = rex.bres_s1; break;
    case 0x0120: h = rex.bres_octant_inc1; l = rex.bres_round_inc2; break;
    case 0x0128: h = rex.bres_e1; l = rex.bres_s2; break;
    case 0x0130: h = rex.a_weight0; l = rex.a_weight1; break;
    case 0x0138: h = rex.x_start_f; l = rex.y_start_f; break;
    case 0x0140: h = rex.x_end_f; l = rex.y_end_f; break;
    case 0x0148: h = (uint32_t)(int32_t)rex.x_start_i; l = rex.x_end_f; break;
    case 0x0150: h = rex.xy_start_i; l = rex.xy_end_i; break;
    case 0x0158: h = rex.x_start_end_i; break;
    case 0x0200: h = rex.color_red; l = rex.color_alpha; break;
    case 0x0208: h = rex.color_green; l = rex.color_blue; break;
    case 0x0210: h = (uint32_t)rex.slope_red; l = (uint32_t)rex.slope_alpha; break;
    case 0x0218: h = (uint32_t)rex.slope_green; l = (uint32_t)rex.slope_blue; break;
    case 0x0220: h = rex.write_mask; l = rex.color_i; break;
    case 0x0228: h = rex.zero_overflow; l = (uint32_t)rex.slope_red; break;
    case 0x0230:                               /* host data port: full 64 bits */
      h = (uint32_t)(rex.host_dataport >> 32);
      l = (uint32_t)rex.host_dataport;
      break;
    case 0x0238: h = rex.dcb_mode; break;
    case 0x0240:
      if(hi) {
        h = dcb_read();
      }
      l = rex.dcb_data_lsw;
      break;
    case 0x1300: h = rex.smask_x[1]; l = rex.smask_y[1]; break;
    case 0x1308: h = rex.smask_x[2]; l = rex.smask_y[2]; break;
    case 0x1310: h = rex.smask_x[3]; l = rex.smask_y[3]; break;
    case 0x1318: h = rex.smask_x[4]; l = rex.smask_y[4]; break;
    case 0x1320: h = rex.top_scanline; l = rex.xy_window; break;
    case 0x1328: h = rex.clip_mode; break;
    case 0x1330: h = rex.config; break;
    case 0x1338:
      if(rex.status & STATUS_BACKBUSY) {
        if(backbusy_reads) {
          backbusy_reads--;
        }
        if(backbusy_reads == 0) {
          rex.status &= ~STATUS_BACKBUSY;
        }
      }
      if(hi) {                                 /* STATUS: read acknowledges retrace */
        h = rex.status | 3;                    /* | 3 = REX3 version bits, as MAME */
        if(in_dma) {
          h |= STATUS_GFXBUSY;                 /* a host-data DMA is still arriving */
        }
        rex.status &= ~STATUS_VRINT;
        set_vrint(false);
      }
      l = rex.status;                          /* USER STATUS: no side effect */
      break;
    default:
      if(g_np_log) {
        fprintf(stderr, "[newport] unknown REX3 read @%04x\n", line * 8);
      }
      return 0;
    }
  if(line & 0x100) {
    do_command();
  }
  if(g_np_log && g_np_acc++ < g_np_trace_n) {
    fprintf(stderr, "[np] icnt=%llu pc=%08x R %04x %s%s -> %08x %08x\n", (unsigned long long)now(), host_pc(),
            line * 8, hi ? "H" : "", lo ? "L" : "", h, l);
  }
  return ((uint64_t)(hi ? h : 0) << 32) | (lo ? l : 0);
}

void sgi_newport::rex3_w(uint32_t line, uint64_t data, uint64_t mask, bool hi, bool lo) {
  const uint32_t h = (uint32_t)(data >> 32), l = (uint32_t)data;
  if(g_np_log) {
    g_np_wr[line & 0x3ff]++;
    if(g_np_acc++ < g_np_trace_n) {
      fprintf(stderr, "[np] icnt=%llu pc=%08x W %04x %s%s <- %08x %08x\n", (unsigned long long)now(), host_pc(),
              line * 8, hi ? "H" : "", lo ? "L" : "", h, l);
    }
  }
  switch((line & ~0x100u) * 8)
    {
    case 0x0000:
      if(hi) {
        rex.draw_mode1 = h;
        rex.plane_enable = h & 7;
        rex.plane_depth = (h >> 3) & 3;
        rex.rwpacked = BITN(h, 7);
        rex.hostdepth = (h >> 8) & 3;
        rex.rwdouble = BITN(h, 10);
        rex.sfactor = (h >> 19) & 7;
        rex.dfactor = (h >> 22) & 7;
        rex.blend_back = BITN(h, 25);
        rex.host_shift = 64 - s_host_shifts[rex.hostdepth];
        rb_set_flags(h);
      }
      if(lo) {
        rex.draw_mode0 = l;
        rex.color_host = BITN(l, 6);
      }
      break;
    case 0x0008:
      if(hi) { rex.ls_mode = h & 0xfffffff; }
      if(lo) { rex.ls_pattern = l; }
      break;
    case 0x0010:
      if(hi) { rex.ls_pattern_saved = h; }
      if(lo) { rex.z_pattern = l; }
      break;
    case 0x0018:
      if(hi) { rex.color_back = h; }
      if(lo) { rex.color_vram = l; }
      break;
    case 0x0020:
      if(hi) { rex.alpha_ref = h & 0xff; }
      break;
    case 0x0028:
      if(hi) { rex.smask_x[0] = h; }
      if(lo) { rex.smask_y[0] = l; }
      break;
    case 0x0030:
      if(hi) { rex.setup = h; do_setup(); }
      if(lo) { rex.step_z = 1; }
      break;
    case 0x0038:
      if(hi) { rex.ls_pattern = rex.ls_pattern_saved; }
      if(lo) { rex.ls_pattern_saved = rex.ls_pattern; }
      break;
    case 0x0100:
      if(hi) { write_x_start((int32_t)h); rex.x_save = rex.x_start_i; }
      if(lo) { write_y_start((int32_t)l); }
      break;
    case 0x0108:
      if(hi) { write_x_end((int32_t)h); }
      if(lo) { write_y_end((int32_t)l); }
      break;
    case 0x0110:
      if(hi) { rex.x_save = (int16_t)h; }
      if(lo) {
        rex.xy_move = l;
        rex.x_move = (int16_t)(l >> 16);
        rex.y_move = (int16_t)l;
      }
      break;
    case 0x0118:
      if(hi) { rex.bres_d = h & 0x7ffffff; }
      if(lo) { rex.bres_s1 = l & 0x1ffff; }
      break;
    case 0x0120:
      if(hi) { rex.bres_octant_inc1 = h & 0x70fffff; }
      if(lo) { rex.bres_round_inc2 = l & 0xff1fffff; }
      break;
    case 0x0128:
      if(hi) { rex.bres_e1 = h & 0xffff; }
      if(lo) { rex.bres_s2 = l & 0x3ffffff; }
      break;
    case 0x0130:
      if(hi) { rex.a_weight0 = h; }
      if(lo) { rex.a_weight1 = l; }
      break;
    case 0x0138:
      if(hi) { write_x_start((int32_t)h & 0x007fff80); rex.x_save = rex.x_start_i; }
      if(lo) { write_y_start((int32_t)l & 0x007fff80); }
      break;
    case 0x0140:
      if(hi) { write_x_end((int32_t)h & 0x007fff80); }
      if(lo) { write_y_end((int32_t)l & 0x007fff80); }
      break;
    case 0x0148:
      if(hi) { write_x_start(((int32_t)(int16_t)h) << 11); rex.x_save = rex.x_start_i; }
      if(lo) { write_x_end((int32_t)l & 0x007fff80); }
      break;
    case 0x0150:
      if(hi) {
        write_x_start(((int32_t)(int16_t)(h >> 16)) << 11);
        write_y_start(((int32_t)(int16_t)h) << 11);
        rex.x_save = rex.x_start_i;
      }
      if(lo) {
        write_x_end(((int32_t)(int16_t)(l >> 16)) << 11);
        write_y_end(((int32_t)(int16_t)l) << 11);
      }
      break;
    case 0x0158:
      if(hi) {
        write_x_start(((int32_t)(int16_t)(h >> 16)) << 11);
        write_x_end(((int32_t)(int16_t)h) << 11);
        rex.x_save = rex.x_start_i;
      }
      break;
    case 0x0200:
      if(hi) {
        rex.color_red = h & 0xffffff;
        rex.curr_red = rex.color_red;
        if(!BITN(rex.draw_mode1, 15)) {
          switch(rex.plane_depth)
            {
            case 0: rex.color_i = (h >> 11) & 0xf; break;
            case 1: rex.color_i = (h >> 11) & 0xff; break;
            case 2: rex.color_i = (h >> 9) & 0xfff; break;
            default: break;
            }
        }
      }
      if(lo) { rex.color_alpha = l & 0xfffff; rex.curr_alpha = rex.color_alpha; }
      break;
    case 0x0208:
      if(hi) { rex.color_green = h & 0xfffff; rex.curr_green = rex.color_green; }
      if(lo) { rex.color_blue = l & 0xfffff; rex.curr_blue = rex.color_blue; }
      break;
    case 0x0210:
      if(hi) {                                 /* sign-magnitude -> MAME's encoding */
        const uint32_t d = h & 0x807fffff;
        rex.slope_red = BITN(d, 31) ? (int32_t)((0x00800000 - (d & 0x7fffff)) | 0x00800000) : (int32_t)(d & 0x7fffff);
      }
      if(lo) {
        const uint32_t d = l & 0x8007ffff;
        rex.slope_alpha = BITN(d, 31) ? (int32_t)((0x00080000 - (d & 0x7ffff)) | 0x00080000) : (int32_t)(d & 0x7ffff);
      }
      break;
    case 0x0218:
      if(hi) {
        const uint32_t d = h & 0x8007ffff;
        rex.slope_green = BITN(d, 31) ? (int32_t)((0x00080000 - (d & 0x7ffff)) | 0x00080000) : (int32_t)(d & 0x7ffff);
      }
      if(lo) {
        const uint32_t d = l & 0x8007ffff;
        rex.slope_blue = BITN(d, 31) ? (int32_t)((0x00080000 - (d & 0x7ffff)) | 0x00080000) : (int32_t)(d & 0x7ffff);
      }
      break;
    case 0x0220:
      if(hi) { rex.write_mask = h & 0xffffff; rb.write_mask = rex.write_mask; }
      if(lo) {
        rex.color_i = l;
        if(BITN(rex.draw_mode1, 15)) {
          rex.color_red = (l & 0xff) << 11;
          rex.color_green = (l & 0xff00) << 3;
          rex.color_blue = (l & 0xff0000) >> 5;
          rex.curr_red = rex.color_red;
          rex.curr_green = rex.color_green;
          rex.curr_blue = rex.color_blue;
        }
      }
      break;
    case 0x0228:
      if(hi) { rex.zero_overflow = h; }
      if(lo) { rex.slope_red = (int32_t)l; }
      break;
    case 0x0230:                               /* host data port merges by mask */
      rex.host_dataport = (rex.host_dataport & ~mask) | (data & mask);
      rex.host_shift = 64 - s_host_shifts[rex.hostdepth];
      break;
    case 0x0238:
      if(hi) {
        switch(h & 3)
          {
          case 1: rex.dcb_mask = 0x000000ff; break;
          case 2: rex.dcb_mask = 0x0000ffff; break;
          default: rex.dcb_mask = 0xffffffff; break;
          }
        rex.dcb_reg_select = (h >> 4) & 7;
        rex.dcb_slave_select = (h >> 7) & 0xf;
        rex.dcb_mode = h & 0x1fffffff;
      }
      break;
    case 0x0240:
      if(hi) { dcb_write(h); }
      if(lo) { rex.dcb_data_lsw = l; }
      break;
    case 0x1300:
      if(hi) { rex.smask_x[1] = h; }
      if(lo) { rex.smask_y[1] = l; }
      break;
    case 0x1308:
      if(hi) { rex.smask_x[2] = h; }
      if(lo) { rex.smask_y[2] = l; }
      break;
    case 0x1310:
      if(hi) { rex.smask_x[3] = h; }
      if(lo) { rex.smask_y[3] = l; }
      break;
    case 0x1318:
      if(hi) { rex.smask_x[4] = h; }
      if(lo) { rex.smask_y[4] = l; }
      break;
    case 0x1320:
      if(hi) { rex.top_scanline = h & 0x3ff; }
      if(lo) {
        rex.xy_window = l;
        rex.x_window = (int16_t)(l >> 16);
        rex.y_window = (int16_t)l;
      }
      break;
    case 0x1328:
      if(hi) { rex.clip_mode = h & 0x1fff; }
      break;
    case 0x1330:
      if(hi) { rex.config = h & 0x1fffff; }
      break;
    case 0x1340:                               /* reset DCB / flush BFIFO: the backend is idle */
      rex.status &= ~STATUS_BACKBUSY;
      backbusy_reads = 0;
      break;
    default:
      if(g_np_log) {
        fprintf(stderr, "[newport] unknown REX3 write @%04x = %016llx\n", line * 8, (unsigned long long)data);
      }
      break;
    }
  if(line & 0x100) {
    do_command();
  }
}

void sgi_newport::write_x_start(int32_t v) {
  rex.x_start = v & 0x07ffff80;
  rex.x_start_frac = (v >> 7) & 0xf;
  rex.x_start_i = (int16_t)(v >> 11);
  rex.x_start_f = (uint32_t)v & 0x007fff80;
  rex.xy_start_i = (rex.xy_start_i & 0x0000ffff) | ((uint32_t)(uint16_t)rex.x_start_i << 16);
}

void sgi_newport::write_y_start(int32_t v) {
  rex.y_start = v & 0x07ffff80;
  rex.y_start_frac = (v >> 7) & 0xf;
  rex.y_start_i = (int16_t)(v >> 11);
  rex.y_start_f = (uint32_t)v & 0x007fff80;
  rex.xy_start_i = (rex.xy_start_i & 0xffff0000) | (uint16_t)rex.y_start_i;
}

void sgi_newport::write_x_end(int32_t v) {
  rex.x_end = v & 0x07ffff80;
  rex.x_end_frac = (v >> 7) & 0xf;
  rex.x_end_i = (int16_t)(v >> 11);
  rex.x_end_f = (uint32_t)v & 0x007fff80;
  rex.xy_end_i = (rex.xy_end_i & 0x0000ffff) | ((uint32_t)(uint16_t)rex.x_end_i << 16);
}

void sgi_newport::write_y_end(int32_t v) {
  rex.y_end = v & 0x07ffff80;
  rex.y_end_frac = (v >> 7) & 0xf;
  rex.y_end_i = (int16_t)(v >> 11);
  rex.y_end_f = (uint32_t)v & 0x007fff80;
  rex.xy_end_i = (rex.xy_end_i & 0xffff0000) | (uint16_t)rex.y_end_i;
}

/* ---------------- RB2 ---------------- */
void sgi_newport::rb_set_flags(uint32_t m1) {
  rb.blend = BITN(m1, 18);
  rb.fast_clear = BITN(m1, 17);
  const bool dblsrc = BITN(m1, 5);
  rb.plane_enable = m1 & 7;
  rb.draw_depth = (m1 >> 3) & 3;
  rb.logicop = (m1 >> 28) & 0xf;
  /* store shift by planes/depth/source buffer (MAME rb2_device::set_flags) */
  static const uint8_t s_shift[8][4][2] = {
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{0, 0}, {0, 8}, {0, 12}, {0, 0}},
    {{0, 0}, {0, 8}, {0, 12}, {0, 0}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
    {{8, 16}, {8, 16}, {8, 16}, {8, 16}},
    {{2, 6}, {2, 6}, {2, 6}, {2, 6}},
    {{0, 4}, {0, 4}, {0, 4}, {0, 4}},
    {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
  };
  rb.src_shift = s_shift[rb.plane_enable][rb.draw_depth][dblsrc ? 1 : 0];
  switch(rb.plane_enable)
    {
    case 1:
    case 2:
      rb.dest = rb.rgbci;
      break;
    case 4:
    case 5:
    case 6:
      rb.dest = rb.cidaux;
      break;
    default:
      break;                                   /* none/invalid: keep the last buffer */
    }
}

void sgi_newport::rb_write(uint32_t src) {
  switch(rb.draw_depth)                        /* replicate into every lane */
    {
    case 0: src |= src << 4; src |= src << 8; src |= src << 16; break;
    case 1: src |= src << 8; src |= src << 16; break;
    case 2: src |= src << 12; break;
    default: break;
    }
  if(rb.plane_enable == 4) {
    src <<= 8;                                 /* overlay */
  }
  else if(rb.plane_enable == 5) {
    src <<= 2;                                 /* popup */
  }
  if(rb.blend || rb.fast_clear) {
    rb_store(src);
    return;
  }
  const uint32_t dst = rb_read();
  uint32_t v = 0;
  switch(rb.logicop)
    {
    case 0: v = 0; break;
    case 1: v = src & dst; break;
    case 2: v = src & ~dst; break;
    case 3: v = src; break;
    case 4: v = ~src & dst; break;
    case 5: v = dst; break;
    case 6: v = src ^ dst; break;
    case 7: v = src | dst; break;
    case 8: v = ~(src | dst); break;
    case 9: v = ~(src ^ dst); break;
    case 10: v = ~dst; break;
    case 11: v = src | ~dst; break;
    case 12: v = ~src; break;
    case 13: v = ~src | dst; break;
    case 14: v = ~(src & dst); break;
    default: v = 0xffffff; break;
    }
  rb_store(v);
}

/* ---------------- REX3 drawing ---------------- */
/* BGR colour conversion between plane depths (0=4, 1=8, 2=12, 3=24 bits):
 * unpack to 8-bit r/g/b, repack -- MAME's convert_*bpp_bgr_to_*bpp table. */
uint32_t sgi_newport::convert_depth(uint32_t c, uint8_t from, uint8_t to) {
  if(from == to) {
    return c;
  }
  uint32_t r = 0, g = 0, b = 0;
  switch(from)
    {
    case 0:
      r = 0xff * BITN(c, 0);
      g = (0xaa * BITN(c, 2)) | (0x55 * BITN(c, 1));
      b = 0xff * BITN(c, 3);
      break;
    case 1:
      r = (0x92 * BITN(c, 2)) | (0x49 * BITN(c, 1)) | (0x24 * BITN(c, 0));
      g = (0x92 * BITN(c, 5)) | (0x49 * BITN(c, 4)) | (0x24 * BITN(c, 3));
      b = (0xaa * BITN(c, 7)) | (0x55 * BITN(c, 6));
      break;
    case 2:
      r = 0x11 * (c & 0xf);
      g = 0x11 * ((c >> 4) & 0xf);
      b = 0x11 * ((c >> 8) & 0xf);
      break;
    default:
      r = c & 0xff;
      g = (c >> 8) & 0xff;
      b = (c >> 16) & 0xff;
      break;
    }
  r &= 0xff;
  g &= 0xff;
  b &= 0xff;
  switch(to)
    {
    case 0: return (BITN(b, 7) << 3) | ((g & 0xc0) >> 5) | BITN(r, 7);
    case 1: return (b & 0xc0) | ((g & 0xe0) >> 2) | ((r & 0xe0) >> 5);
    case 2: return ((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4);
    default: return (b << 16) | (g << 8) | r;
    }
}

uint32_t sgi_newport::host_color() {
  static const uint32_t s_masks[4] = { 0xf, 0xff, 0xfff, 0xffffffff };
  uint32_t color = (uint32_t)(rex.host_dataport >> rex.host_shift) & s_masks[rex.hostdepth];
  if(rex.rwpacked) {
    if((rex.rwdouble && rex.host_shift > 0) || rex.host_shift > 32) {
      rex.host_shift -= s_host_shifts[rex.hostdepth];
    }
    else {
      rex.host_shift = 64 - s_host_shifts[rex.hostdepth];
    }
  }
  color = convert_depth(color, rex.hostdepth, rex.plane_depth);
  if(BITN(rex.draw_mode1, 11)) {
    color = __builtin_bswap32(color);
  }
  return color;
}

bool sgi_newport::clip_pass(int16_t x, int16_t y) {
  if(BITN(rex.clip_mode, 0)) {
    const int16_t min_x = (int16_t)(rex.smask_x[0] >> 16), max_x = (int16_t)rex.smask_x[0];
    const int16_t min_y = (int16_t)(rex.smask_y[0] >> 16), max_y = (int16_t)rex.smask_y[0];
    if(x < min_x || y < min_y || x > max_x || y > max_y) {
      return false;
    }
  }
  x += rex.x_window - 0x1000;
  y += rex.y_window - 0x1000;
  if(rex.clip_mode & 0x1e) {
    int bit = 1;
    for(; bit < 5; bit++) {
      if(!BITN(rex.clip_mode, bit)) {
        continue;
      }
      const int16_t min_x = (int16_t)(rex.smask_x[bit] >> 16) - 0x1000;
      const int16_t min_y = (int16_t)(rex.smask_y[bit] >> 16) - 0x1000;
      const int16_t max_x = (int16_t)rex.smask_x[bit] - 0x1000;
      const int16_t max_y = (int16_t)rex.smask_y[bit] - 0x1000;
      if(x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
        break;
      }
    }
    if(bit == 5) {
      return false;
    }
  }
  if(x < 0 || y < 0 || x >= (int)FB_W || y >= (int)DISP_H) {
    return false;
  }
  /* CLIPMODE[12:9] CIDMATCH: one bit per clip ID (the pixel's CID planes, aux bits
   * [1:0]); a pixel is drawn only if its CID's bit is set.  The X server writes 0xf
   * (all pass); a direct-rendering client (Twilight drawing the root, GL windows)
   * writes one bit so it touches only the pixels its window owns.  MAME has no CID
   * test, so a full-screen root draw ran over every window. */
  const uint32_t cidmatch = (rex.clip_mode >> 9) & 0xf;
  if(cidmatch != 0xf) {
    const uint32_t cid = rb.cidaux[(uint32_t)(NP_ROW(y) * FB_W + x)] & 3;
    if(!BITN(cidmatch, cid)) {
      return false;
    }
  }
  return true;
}

void sgi_newport::output_pixel(int16_t x, int16_t y, uint32_t color) {
  go_pixels++;
  if(!clip_pass(x, y)) {
    return;
  }
  x += rex.x_window - 0x1000;
  y += rex.y_window - 0x1000;
  rb_set_addr((uint32_t)(NP_ROW(y) * FB_W + x));
  if(BITN(rex.draw_mode1, 18)) {
    blend_pixel(color);
  }
  else {
    rb_write(color);
  }
}

void sgi_newport::blend_pixel(uint32_t src) {
  const uint32_t dst = rb_read();
  float sa = ((src >> 24) & 0xff) / 255.0f;
  float sr = 0, sg = 0, sb = 0, dr = 0, dg = 0, db = 0;
  auto back = [&]() {
    db = (uint8_t)(rex.color_back >> 16) / 255.0f;
    dg = (uint8_t)(rex.color_back >> 8) / 255.0f;
    dr = (uint8_t)(rex.color_back >> 0) / 255.0f;
  };
  switch(rex.plane_depth)
    {
    case 1:
    case 2:
      if(rex.plane_depth == 1) {
        sb = (float)((0xaa * BITN(src, 7)) | (0x55 * BITN(src, 6)));
        sg = (float)((0x92 * BITN(src, 5)) | (0x49 * BITN(src, 4)) | (0x24 * BITN(src, 3)));
        sr = (float)((0x92 * BITN(src, 2)) | (0x49 * BITN(src, 1)) | (0x24 * BITN(src, 0)));
      }
      else {
        sb = (((src >> 8) & 15) * 0x11) / 255.0f;
        sg = (((src >> 4) & 15) * 0x11) / 255.0f;
        sr = (((src >> 0) & 15) * 0x11) / 255.0f;
      }
      if(rex.blend_back) {
        back();
      }
      else {
        db = (((dst >> 8) & 15) * 0x11) / 255.0f;
        dg = (((dst >> 4) & 15) * 0x11) / 255.0f;
        dr = (((dst >> 0) & 15) * 0x11) / 255.0f;
      }
      break;
    case 3:
      sb = (uint8_t)(src >> 16) / 255.0f;
      sg = (uint8_t)(src >> 8) / 255.0f;
      sr = (uint8_t)(src >> 0) / 255.0f;
      if(rex.blend_back) {
        back();
      }
      else {
        db = (uint8_t)(dst >> 16) / 255.0f;
        dg = (uint8_t)(dst >> 8) / 255.0f;
        dr = (uint8_t)(dst >> 0) / 255.0f;
      }
      break;
    default:
      break;
    }
  float sbb = 0, sgb = 0, srb = 0, dbb = 0, dgb = 0, drb = 0;
  const bool sfa = BITN(rex.draw_mode1, 27);
  switch(rex.sfactor)
    {
    case 1: sbb = sb; sgb = sg; srb = sr; break;
    case 2: sbb = sb * db; sgb = sg * dg; srb = sr * dr; break;
    case 3: sbb = sb * (1 - db); sgb = sg * (1 - dg); srb = sr * (1 - dr); break;
    case 4:
      if(sfa) { sbb = sb * sa; sgb = sg * sa; srb = sr * sa; }
      else { sbb = sb; sgb = sg; srb = sr; }
      break;
    case 5:
      if(sfa) { sbb = sb * (1 - sa); sgb = sg * (1 - sa); srb = sr * (1 - sa); }
      break;
    default: break;
    }
  switch(rex.dfactor)
    {
    case 1: dbb = db; dgb = dg; drb = dr; break;
    case 2: dbb = db * sb; dgb = dg * sg; drb = dr * sr; break;
    case 3: dbb = db * (1 - sb); dgb = dg * (1 - sg); drb = dr * (1 - sr); break;
    case 4: dbb = db * sa; dgb = dg * sa; drb = dr * sa; break;
    case 5: dbb = db * (1 - sa); dgb = dg * (1 - sa); drb = dr * (1 - sa); break;
    default: break;
    }
  auto q = [](float v) -> uint32_t {
    return v > 1.0f ? 255u : (v < 0.0f ? 0u : (uint32_t)(v * 255.0f));
  };
  const uint32_t b = q(sbb + dbb), g = q(sgb + dgb), r = q(srb + drb);
  if(rex.plane_depth == 2) {
    rb_write(((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4));
  }
  else if(rex.plane_depth == 3) {
    rb_write((b << 16) | (g << 8) | r);
  }
}

uint32_t sgi_newport::rgb_color(int16_t x, int16_t y) {
  return rgb_from(rex.curr_red, rex.curr_green, rex.curr_blue, rex.curr_alpha, x, y);
}

/* the shading colour for iterator values (cr, cg, cb, ca) at (x, y): shared by the
 * per-pixel path and the span kernels */
inline uint32_t sgi_newport::rgb_from(uint32_t cr, uint32_t cg, uint32_t cb, uint32_t ca, int16_t x, int16_t y) {
  static const uint8_t s_bayer[4][4] = { {0, 12, 3, 15}, {8, 4, 11, 7}, {2, 14, 1, 13}, {10, 6, 9, 5} };
  auto clamp = [](uint32_t c) -> uint32_t {
    const uint32_t v = (c >> 11) & 0x1ff;
    if(v >= 0x180 || BITN(c, 31)) {
      return 0;
    }
    return v > 0xff ? 0xff : v;
  };
  const uint32_t red = clamp(cr), green = clamp(cg), blue = clamp(cb);
  const uint32_t alpha = clamp(ca) << 24;
  if(!BITN(rex.draw_mode1, 15)) {
    return default_color(cr);
  }
  if(BITN(rex.draw_mode1, 16)) {               /* dithering */
    const uint8_t bay = s_bayer[x & 3][y & 3];
    switch(rex.plane_depth)
      {
      case 0: {
        const uint8_t sr = (red >> 3) - (red >> 4), sg = (green >> 2) - (green >> 4), sb = (blue >> 3) - (blue >> 4);
        uint32_t dr = BITN(sr, 4), dg = (sg >> 4) & 3, db = BITN(sb, 4);
        if((sr & 0xf) > bay) { dr++; }
        if((sg & 0xf) > bay) { dg++; }
        if((sb & 0xf) > bay) { db++; }
        if(dr > 1) { dr = 1; }
        if(dg > 3) { dg = 3; }
        if(db > 1) { db = 1; }
        const uint32_t c = (db << 3) | (dg << 1) | dr;
        return alpha | (c << 4) | c;
      }
      case 1: {
        const uint8_t sr = (red >> 1) - (red >> 4), sg = (green >> 1) - (green >> 4), sb = (blue >> 2) - (blue >> 4);
        uint32_t dr = (sr >> 4) & 7, dg = (sg >> 4) & 7, db = (sb >> 4) & 3;
        if((sr & 0xf) > bay) { dr++; }
        if((sg & 0xf) > bay) { dg++; }
        if((sb & 0xf) > bay) { db++; }
        if(dr > 7) { dr = 7; }
        if(dg > 7) { dg = 7; }
        if(db > 3) { db = 3; }
        return alpha | (db << 6) | (dg << 3) | dr;
      }
      case 2: {
        const uint32_t sr = red - (red >> 4), sg = green - (green >> 4), sb = blue - (blue >> 4);
        uint32_t dr = (sr >> 4) & 15, dg = (sg >> 4) & 15, db = (sb >> 4) & 15;
        if((sr & 0xf) > bay) { dr++; }
        if((sg & 0xf) > bay) { dg++; }
        if((sb & 0xf) > bay) { db++; }
        if(dr > 15) { dr = 15; }
        if(dg > 15) { dg = 15; }
        if(db > 15) { db = 15; }
        const uint32_t c = (db << 8) | (dg << 4) | dr;
        return alpha | (c << 12) | c;
      }
      default:
        return alpha | (blue << 16) | (green << 8) | red;
      }
  }
  return alpha | convert_depth((blue << 16) | (green << 8) | red, 3, rex.plane_depth);
}

void sgi_newport::iterate_shade() {
  auto sext = [](int32_t v, int bits) -> int32_t {
    return (int32_t)((uint32_t)v << (32 - bits)) >> (32 - bits);
  };
  if(rex.slope_red & 0x7fffff) { rex.curr_red += sext(rex.slope_red, 24); }
  if(rex.slope_green & 0x7ffff) { rex.curr_green += sext(rex.slope_green, 20); }
  if(rex.slope_blue & 0x7ffff) { rex.curr_blue += sext(rex.slope_blue, 20); }
  if(rex.slope_alpha & 0x7ffff) { rex.curr_alpha += sext(rex.slope_alpha, 20); }
  if(!BITN(rex.draw_mode0, 21)) {              /* CI clamp */
    return;
  }
  if(BITN(rex.draw_mode1, 15)) {
    auto cl = [](uint32_t &c) {
      const uint32_t v = (c >> 11) & 0x1ff;
      if(v >= 0x180 || BITN(c, 31)) {
        c = 0;
      }
      else if(v > 0xff) {
        c = 0x7ffff;
      }
    };
    cl(rex.curr_red);
    cl(rex.curr_green);
    cl(rex.curr_blue);
    cl(rex.curr_alpha);
    return;
  }
  switch((rex.draw_mode1 >> 3) & 3)
    {
    case 0: if(BITN(rex.color_red, 15)) { rex.color_red = 0x00007fff; } break;
    case 1: if(BITN(rex.color_red, 19)) { rex.color_red = 0x0007ffff; } break;
    case 2: if(BITN(rex.color_red, 21)) { rex.color_red = 0x001fffff; } break;
    default: break;
    }
}

uint32_t sgi_newport::default_color(uint32_t src) {
  uint32_t c = BITN(rex.draw_mode1, 17) ? rex.color_vram : src;
  switch(rex.plane_depth)
    {
    case 0: c &= 0xf; c |= c << 4; c |= c << 8; c |= c << 16; break;
    case 1: c &= 0xff; c |= c << 8; c |= c << 16; break;
    case 2:
      if(BITN(rex.draw_mode1, 15)) {
        c = ((rex.color_vram & 0xf00000) >> 12) | ((rex.color_vram & 0xf000) >> 8) | ((rex.color_vram & 0xf0) >> 4);
      }
      else {
        c &= 0xfff;
      }
      c |= c << 12;
      break;
    default: c = rex.color_vram & 0xffffff; break;
    }
  return c;
}

/* REX3 Bresenham octant {XMAJOR, XDEC, YDEC}: the per-octant step table and
 * the octant computed by line setup (MAME get_octant / s_bresenham_infos). */
struct bres_t { int16_t incrx1, incrx2, incry1, incry2; uint8_t loop; };
static const bres_t s_bres[8] = {
  {0, 1, -1, -1, 1}, {0, 1, 1, 1, 1}, {0, -1, -1, -1, 1}, {0, -1, 1, 1, 1},
  {1, 1, 0, -1, 0}, {1, 1, 0, 1, 0}, {-1, -1, 0, -1, 0}, {-1, -1, 0, 1, 0}
};

static inline int32_t sext20(int32_t v) {
  return (int32_t)((uint32_t)v << 12) >> 12;
}

void sgi_newport::do_setup() {
  const int32_t x1 = sext20(rex.x_start >> 7), y1 = sext20(rex.y_start >> 7);
  const int32_t x2 = sext20(rex.x_end >> 7), y2 = sext20(rex.y_end >> 7);
  const int32_t dx = std::abs(x1 - x2), dy = std::abs(y1 - y2);
  uint32_t oct = 0;
  if(y1 > y2) { oct |= 1; }
  if(x1 > x2) { oct |= 2; }
  if(dx > dy) { oct |= 4; }
  rex.bres_octant_inc1 = (rex.bres_octant_inc1 & ~(7u << 24)) | (oct << 24);
}

void sgi_newport::do_fline(uint32_t color) {
  const int32_t x1 = sext20(rex.x_start >> 7), y1 = sext20(rex.y_start >> 7);
  const int32_t x2 = sext20(rex.x_end >> 7), y2 = sext20(rex.y_end >> 7);
  const int32_t x10 = x1 & ~0xf, y10 = y1 & ~0xf, x20 = x2 & ~0xf, y20 = y2 & ~0xf;
  const bool iterate_one = (rex.draw_mode0 & 0x300) == 0;
  const bool skip_first = BITN(rex.draw_mode0, 10), skip_last = BITN(rex.draw_mode0, 11);
  const bool shade = BITN(rex.draw_mode0, 18), rgbmode = BITN(rex.draw_mode1, 15);
  int32_t x = x10, y = y10;
  int16_t x1f = rex.x_start_frac, y1f = rex.y_start_frac;
  int32_t dx = std::abs(x1 - x2), dy = std::abs(y1 - y2);
  const int32_t dx_i = std::abs(x10 - x20) - 1, dy_i = std::abs(y10 - y20) - 1;
  const uint8_t oct = (rex.bres_octant_inc1 >> 24) & 7;
  const bres_t &bi = s_bres[oct];
  int32_t loop = bi.loop ? dy_i : dx_i;
  if(BITN(rex.draw_mode0, 15) && loop > 32) {
    loop = 32;
  }
  const int32_t x_major = 1 - bi.loop;
  const int32_t incr1 = bi.loop ? (2 * dx) : (2 * dy);
  const int32_t incr2 = bi.loop ? (2 * (dx - dy)) : (2 * (dy - dx));
  int32_t d = bi.loop ? (3 * dx - 2 * dy) : (3 * dy - 2 * dx);
  switch(oct)
    {
    case 1: { std::swap(x1f, y1f); std::swap(dx, dy); break; }
    case 3: { const int16_t t = 0x10 - x1f; x1f = y1f; y1f = t; std::swap(dx, dy); break; }
    case 7: x1f = 0x10 - x1f; break;
    case 6: x1f = 0x10 - x1f; y1f = 0x10 - y1f; break;
    case 2: { const int16_t t = 0x10 - x1f; x1f = 0x10 - y1f; y1f = t; std::swap(dx, dy); break; }
    case 0: { const int16_t t = 0x10 - y1f; y1f = x1f; x1f = t; std::swap(dx, dy); break; }
    case 4: y1f = 0x10 - y1f; break;
    default: break;
    }
  d += 2 * (((dx * y1f) >> 4) - ((dy * x1f) >> 4));
  const int32_t e = d - 2 * dx;
  if(e > 0) {
    d = e;
    x += bi.incrx2 * (1 - x_major);
    y += bi.incry2 * x_major;
  }
  auto plot = [&](int32_t px, int32_t py) {
    const int16_t x16 = (int16_t)(px >> 4), y16 = (int16_t)(py >> 4);
    output_pixel(x16, y16, (shade || rgbmode) ? rgb_color(x16, y16) : color);
    if(shade) {
      iterate_shade();
    }
  };
  auto step = [&]() {
    if(d < 0) {
      x += bi.incrx1;
      y -= bi.incry1;
      d += incr1;
    }
    else {
      x += bi.incrx2;
      y -= bi.incry2;
      d += incr2;
    }
  };
  if(!skip_first || iterate_one) {
    plot(x, y);
    step();
    if(iterate_one) {
      write_x_start(x << 7);
      write_y_start(y << 7);
      return;
    }
  }
  for(int32_t i = 1; i < loop; i++) {
    plot(x, y);
    step();
  }
  if(!skip_last) {
    plot(x2, y2);
    write_x_start(x2);
    write_y_start(y2);
  }
  else {
    write_x_start(x << 7);
    write_y_start(y << 7);
  }
}

void sgi_newport::do_iline(uint32_t color) {
  const int32_t x2 = rex.x_end_i, y2 = rex.y_end_i;
  const bool iterate_one = (rex.draw_mode0 & 0x300) == 0;
  const bool skip_first = BITN(rex.draw_mode0, 10), skip_last = BITN(rex.draw_mode0, 11);
  const bool shade = BITN(rex.draw_mode0, 18), rgbmode = BITN(rex.draw_mode1, 15);
  int32_t x = rex.x_start_i, y = rex.y_start_i;
  const int32_t dx = std::abs(x2 - x), dy = std::abs(y2 - y);
  const uint8_t oct = (rex.bres_octant_inc1 >> 24) & 7;
  const bres_t &bi = s_bres[oct];
  int32_t loop = bi.loop ? dy : dx;
  if(BITN(rex.draw_mode0, 15) && loop > 32) {
    loop = 32;
  }
  const int32_t incr1 = 2 * (bi.loop ? dx : dy);
  const int32_t incr2 = 2 * (bi.loop ? (dx - dy) : (dy - dx));
  int32_t d = incr1 - (bi.loop ? dy : dx);
  auto plot = [&](int32_t px, int32_t py) {
    const int16_t x16 = (int16_t)px, y16 = (int16_t)py;
    output_pixel(x16, y16, (shade || rgbmode) ? rgb_color(x16, y16) : color);
    if(shade) {
      iterate_shade();
    }
  };
  auto step = [&]() {
    if(d < 0) {
      x += bi.incrx1;
      y -= bi.incry1;
      d += incr1;
    }
    else {
      x += bi.incrx2;
      y -= bi.incry2;
      d += incr2;
    }
  };
  if(!skip_first || iterate_one) {
    plot(x, y);
    step();
    if(iterate_one) {
      write_x_start(x << 11);
      write_y_start(y << 11);
      return;
    }
  }
  for(int32_t i = 1; i < loop; i++) {
    plot(x, y);
    step();
  }
  if(!skip_last) {
    plot(x2, y2);
    write_x_start(x2 << 11);
    write_y_start(y2 << 11);
  }
  else {
    write_x_start(x << 11);
    write_y_start(y << 11);
  }
}

uint32_t sgi_newport::do_pixel_read() {
  go_pixels++;
  const int16_t sx = rex.x_start_i + rex.x_window - 0x1000;
  const int16_t sy = rex.y_start_i + rex.y_window - 0x1000;
  uint32_t r = 0;
  if(sx >= 0 && sy >= 0 && sx < (int)FB_W && sy < (int)FB_H) {
    rb_set_addr((uint32_t)(sy * FB_W + sx));
    r = rb_read();
  }
  if(rex.plane_enable == 1 || rex.plane_enable == 2) {
    r = convert_depth(r, rex.plane_depth, rex.hostdepth);
  }
  rex.x_start_i++;
  const int16_t ddy = (rex.y_end_i < rex.y_start_i) ? -1 : 1;
  if(rex.x_start_i > rex.x_end_i) {
    rex.y_start_i += ddy;
    rex.x_start_i = rex.x_save;
  }
  write_x_start(rex.x_start_i << 11);
  write_y_start(rex.y_start_i << 11);
  return r;
}

uint64_t sgi_newport::do_pixel_word_read() {
  const uint16_t xs = (uint16_t)(rex.xy_start_i >> 16), xe = (uint16_t)(rex.xy_end_i >> 16);
  const bool dbl = BITN(rex.draw_mode1, 10);
  uint16_t width = (uint16_t)(xe - xs + 1);
  static const uint16_t s_max[4] = { 8, 4, 2, 1 };
  static const uint32_t s_bits[4] = { 4, 8, 16, 32 };
  static const uint32_t s_pmask[4] = { 0xf, 0xff, 0xfff, 0xffffffff };
  const uint32_t hd = (rex.draw_mode1 >> 8) & 3;
  const uint16_t maxw = (uint16_t)(s_max[hd] * (dbl ? 2 : 1));
  if(width > maxw) {
    width = maxw;
  }
  uint64_t r = 0;
  int32_t shift = 64 - (int32_t)s_bits[hd];
  for(uint16_t i = 0; i < width; i++) {
    r |= (uint64_t)(do_pixel_read() & s_pmask[hd]) << shift;
    shift -= (int32_t)s_bits[hd];
  }
  return r;
}

/* ---------------- span kernels ----------------
 * One row segment of a span/block draw, as lane arrays instead of per-pixel calls:
 * colour generation, pattern expansion, clipping, lane replication, logic op and
 * write-mask merge are each a plain loop over fixed-width arrays, so the compiler
 * can vectorize them (SSE/AVX2 or NEON) with no intrinsics.  Applies only when the
 * result provably equals the per-pixel path; returns false (state untouched) when
 * it does not apply and the caller runs the per-pixel loop.  NEWPORT_SCALAR=1 forces
 * the per-pixel path everywhere (A/B and correctness oracle). */
static const bool g_np_scalar = getenv("NEWPORT_SCALAR") != nullptr;


static const int SPAN_MAX = 2048;

/* rgb_from for RGB mode with dithering, at plane depth D, branch-free so a loop of
 * them vectorizes; bay = the 4x4 ordered-dither threshold for the lane */
static inline uint32_t span_clamp(uint32_t c) {
  const uint32_t v = (c >> 11) & 0x1ff;
  const uint32_t lim = v > 0xff ? 0xff : v;
  return (v >= 0x180 || (c >> 31)) ? 0 : lim;
}
template<int D>
static inline uint32_t dither_lane(uint32_t cr, uint32_t cg, uint32_t cb, uint32_t ca, uint32_t bay) {
  const uint32_t red = span_clamp(cr), green = span_clamp(cg), blue = span_clamp(cb);
  const uint32_t alpha = span_clamp(ca) << 24;
  if(D == 0) {
    const uint32_t sr = (red >> 3) - (red >> 4), sg = (green >> 2) - (green >> 4), sb = (blue >> 3) - (blue >> 4);
    uint32_t dr = ((sr >> 4) & 1) + ((sr & 0xf) > bay), dg = ((sg >> 4) & 3) + ((sg & 0xf) > bay);
    uint32_t db = ((sb >> 4) & 1) + ((sb & 0xf) > bay);
    dr = dr > 1 ? 1 : dr;
    dg = dg > 3 ? 3 : dg;
    db = db > 1 ? 1 : db;
    const uint32_t c = (db << 3) | (dg << 1) | dr;
    return alpha | (c << 4) | c;
  }
  if(D == 1) {
    const uint32_t sr = (red >> 1) - (red >> 4), sg = (green >> 1) - (green >> 4), sb = (blue >> 2) - (blue >> 4);
    uint32_t dr = ((sr >> 4) & 7) + ((sr & 0xf) > bay), dg = ((sg >> 4) & 7) + ((sg & 0xf) > bay);
    uint32_t db = ((sb >> 4) & 3) + ((sb & 0xf) > bay);
    dr = dr > 7 ? 7 : dr;
    dg = dg > 7 ? 7 : dg;
    db = db > 3 ? 3 : db;
    return alpha | (db << 6) | (dg << 3) | dr;
  }
  if(D == 2) {
    const uint32_t sr = red - (red >> 4), sg = green - (green >> 4), sb = blue - (blue >> 4);
    uint32_t dr = ((sr >> 4) & 15) + ((sr & 0xf) > bay), dg = ((sg >> 4) & 15) + ((sg & 0xf) > bay);
    uint32_t db = ((sb >> 4) & 15) + ((sb & 0xf) > bay);
    dr = dr > 15 ? 15 : dr;
    dg = dg > 15 ? 15 : dg;
    db = db > 15 ? 15 : db;
    const uint32_t c = (db << 8) | (dg << 4) | dr;
    return alpha | (c << 12) | c;
  }
  return alpha | (blue << 16) | (green << 8) | red;
}

template<int D>
static void dither_span(uint32_t *col, const uint8_t *pat, const uint32_t *cr, const uint32_t *cg, const uint32_t *cb,
                        const uint32_t *ca, const uint32_t *bay, uint32_t back, int n) {
  for(int k = 0; k < n; k++) {
    const uint32_t v = dither_lane<D>(cr[k], cg[k], cb[k], ca[k], bay[k]);
    col[k] = pat[k] ? v : back;
  }
}

bool sgi_newport::span_fast(int16_t &start_x, int16_t y, int16_t prim_end_x, int16_t end_x, bool stop_on_x,
                            uint32_t pattern, bool opaque, bool shade, bool rgbmode, bool fastclear, uint32_t color) {
  /* lanes in this segment: the first k >= 1 (mod 2^16) reaching prim_end_x or end_x */
  int n = 1;
  if(stop_on_x) {
    int k1 = (uint16_t)(prim_end_x - start_x), k2 = (uint16_t)(end_x - start_x);
    if(k1 == 0) {
      k1 = 65536;
    }
    if(k2 == 0) {
      k2 = 65536;
    }
    n = k1 < k2 ? k1 : k2;
  }
  const int x0 = start_x;
  const int xw0 = x0 + rex.x_window - 0x1000;
  if(n > SPAN_MAX || x0 + n - 1 > 32767 || xw0 < -32768 || xw0 + n - 1 > 32767 || BITN(rex.draw_mode1, 18)) {
    return false;                              /* coordinate wrap or blend: per-pixel path */
  }
  static uint32_t col[SPAN_MAX], cr[SPAN_MAX], cg[SPAN_MAX], cb[SPAN_MAX], ca[SPAN_MAX];
  static uint8_t wr[SPAN_MAX], pat[SPAN_MAX], pass[SPAN_MAX];
  for(int k = 0; k < n; k++) {
    pat[k] = (uint8_t)((pattern >> ((31 - k) & 31)) & 1);
    wr[k] = pat[k] | (opaque ? 1 : 0);
  }
  /* colour generation */
  if(rex.color_host) {
    for(int k = 0; k < n; k++) {               /* host data is consumed in lane order */
      col[k] = pat[k] ? host_color() : rex.color_back;
    }
  }
  else if((shade || rgbmode) && !fastclear) {
    if(shade && BITN(rex.draw_mode0, 21)) {    /* CI clamp: the iterators clamp per step */
      for(int k = 0; k < n; k++) {
        iterate_shade();
        cr[k] = rex.curr_red;
        cg[k] = rex.curr_green;
        cb[k] = rex.curr_blue;
        ca[k] = rex.curr_alpha;
      }
    }
    else if(shade) {                           /* lane k = start + (k+1) * slope */
      auto sx = [](int32_t v, int bits) -> uint32_t {
        return (uint32_t)((int32_t)((uint32_t)v << (32 - bits)) >> (32 - bits));
      };
      const uint32_t sr = (rex.slope_red & 0x7fffff) ? sx(rex.slope_red, 24) : 0;
      const uint32_t sg = (rex.slope_green & 0x7ffff) ? sx(rex.slope_green, 20) : 0;
      const uint32_t sb = (rex.slope_blue & 0x7ffff) ? sx(rex.slope_blue, 20) : 0;
      const uint32_t sa = (rex.slope_alpha & 0x7ffff) ? sx(rex.slope_alpha, 20) : 0;
      const uint32_t r0 = rex.curr_red, g0 = rex.curr_green, b0 = rex.curr_blue, a0 = rex.curr_alpha;
      for(int k = 0; k < n; k++) {
        const uint32_t m = (uint32_t)(k + 1);
        cr[k] = r0 + m * sr;
        cg[k] = g0 + m * sg;
        cb[k] = b0 + m * sb;
        ca[k] = a0 + m * sa;
      }
      rex.curr_red = cr[n - 1];
      rex.curr_green = cg[n - 1];
      rex.curr_blue = cb[n - 1];
      rex.curr_alpha = ca[n - 1];
    }
    else {
      for(int k = 0; k < n; k++) {
        cr[k] = rex.curr_red;
        cg[k] = rex.curr_green;
        cb[k] = rex.curr_blue;
        ca[k] = rex.curr_alpha;
      }
    }
    if(rgbmode && BITN(rex.draw_mode1, 16)) {  /* RGB + dither: the vector kernel */
      static const uint8_t s_bayer[4][4] = { {0, 12, 3, 15}, {8, 4, 11, 7}, {2, 14, 1, 13}, {10, 6, 9, 5} };
      static uint32_t bay[SPAN_MAX];
      for(int k = 0; k < n; k++) {
        bay[k] = s_bayer[(x0 + k) & 3][y & 3];
      }
      switch(rex.plane_depth)
        {
        case 0: dither_span<0>(col, pat, cr, cg, cb, ca, bay, rex.color_back, n); break;
        case 1: dither_span<1>(col, pat, cr, cg, cb, ca, bay, rex.color_back, n); break;
        case 2: dither_span<2>(col, pat, cr, cg, cb, ca, bay, rex.color_back, n); break;
        default: dither_span<3>(col, pat, cr, cg, cb, ca, bay, rex.color_back, n); break;
        }
    }
    else {
      for(int k = 0; k < n; k++) {
        col[k] = pat[k] ? rgb_from(cr[k], cg[k], cb[k], ca[k], (int16_t)(x0 + k), y) : rex.color_back;
      }
    }
  }
  else {
    for(int k = 0; k < n; k++) {
      col[k] = pat[k] ? color : rex.color_back;
    }
  }
  /* every written lane counts, clipped or not (as output_pixel does) */
  uint32_t nw = 0;
  for(int k = 0; k < n; k++) {
    nw += wr[k];
  }
  go_pixels += nw;
  /* clip, once per row: lane predicates */
  const int yw = (int16_t)(y + rex.y_window - 0x1000);
  bool row_ok = yw >= 0 && yw < (int)DISP_H;
  int y_min0 = 0, y_max0 = 0;
  const bool m0 = BITN(rex.clip_mode, 0);
  if(m0) {
    y_min0 = (int16_t)(rex.smask_y[0] >> 16);
    y_max0 = (int16_t)rex.smask_y[0];
    row_ok = row_ok && y >= y_min0 && y <= y_max0;
  }
  int x_lo[4], x_hi[4], nm = 0;                 /* masks 1-4 whose y range holds this row */
  const bool any_m = (rex.clip_mode & 0x1e) != 0;
  if(any_m) {
    for(int b = 1; b < 5; b++) {
      if(!BITN(rex.clip_mode, b)) {
        continue;
      }
      const int16_t mny = (int16_t)((int16_t)(rex.smask_y[b] >> 16) - 0x1000);
      const int16_t mxy = (int16_t)((int16_t)rex.smask_y[b] - 0x1000);
      if(yw >= mny && yw <= mxy) {
        x_lo[nm] = (int16_t)((int16_t)(rex.smask_x[b] >> 16) - 0x1000);
        x_hi[nm] = (int16_t)((int16_t)rex.smask_x[b] - 0x1000);
        nm++;
      }
    }
    if(nm == 0) {
      row_ok = false;
    }
  }
  if(!row_ok) {
    start_x = (int16_t)(x0 + n);
    return true;
  }
  const int x_min0 = m0 ? (int16_t)(rex.smask_x[0] >> 16) : -65536;
  const int x_max0 = m0 ? (int16_t)rex.smask_x[0] : 65536;
  const uint32_t cidmatch = (rex.clip_mode >> 9) & 0xf;     /* as clip_pass */
  const uint32_t *cidrow = &rb.cidaux[(uint32_t)NP_ROW(yw) * FB_W];
  for(int k = 0; k < n; k++) {
    const int x = x0 + k, xw = xw0 + k;
    bool ok = wr[k] && x >= x_min0 && x <= x_max0 && xw >= 0 && xw < (int)FB_W;
    if(ok && cidmatch != 0xf) {
      ok = BITN(cidmatch, cidrow[xw] & 3);
    }
    if(any_m) {
      bool hit = false;
      for(int i = 0; i < nm; i++) {
        hit = hit || (xw >= x_lo[i] && xw <= x_hi[i]);
      }
      ok = ok && hit;
    }
    pass[k] = ok ? 1 : 0;
  }
  /* write: lanes [kl, kh) are inside the framebuffer row */
  const int kl = xw0 < 0 ? -xw0 : 0;
  const int kh = (xw0 + n > (int)FB_W) ? (int)FB_W - xw0 : n;
  if(kl >= kh) {
    start_x = (int16_t)(x0 + n);
    return true;
  }
  uint32_t *row = &rb.dest[(uint32_t)NP_ROW(yw) * FB_W + (uint32_t)xw0];
  if(touching) {
    for(int k = kl; k < kh; k++) {
      if(pass[k]) {
        touch_note(&row[k]);
      }
    }
  }
  const uint32_t m = rb.write_mask & rb.global_mask & plane_mask();
  const uint32_t op = rb.logicop;
  const uint32_t t0 = BITN(op, 0) ? ~0u : 0, t1 = BITN(op, 1) ? ~0u : 0;
  const uint32_t t2 = BITN(op, 2) ? ~0u : 0, t3 = BITN(op, 3) ? ~0u : 0;
  const bool direct = rb.blend || rb.fast_clear;
  const uint8_t depth = rb.draw_depth, plane = rb.plane_enable, sh = rb.src_shift;
  for(int k = kl; k < kh; k++) {
    uint32_t v = col[k];
    if(depth == 0) {
      v |= v << 4;
      v |= v << 8;
      v |= v << 16;
    }
    else if(depth == 1) {
      v |= v << 8;
      v |= v << 16;
    }
    else if(depth == 2) {
      v |= v << 12;
    }
    v = (plane == 4) ? (v << 8) : ((plane == 5) ? (v << 2) : v);
    const uint32_t old = row[k];
    const uint32_t d = old >> sh;
    /* logic op as its truth table: equal to the 16-way switch after masking (the
     * write mask never exceeds 24 bits) */
    const uint32_t lv = (v & d & t0) | (v & ~d & t1) | (~v & d & t2) | (~v & ~d & t3);
    const uint32_t nv = direct ? v : lv;
    const uint32_t merged = (old & ~m) | (nv & m);
    row[k] = pass[k] ? merged : old;
  }
  start_x = (int16_t)(x0 + n);
  return true;
}

void sgi_newport::span_or_block(bool block) {
  const uint32_t mode0 = rex.draw_mode0, mode1 = rex.draw_mode1;
  int16_t start_x = rex.x_start_i, start_y = rex.y_start_i;
  int16_t end_x = rex.x_end_i, end_y = rex.y_end_i;
  const uint8_t oct = (rex.bres_octant_inc1 >> 24) & 7;
  const int16_t dx = BITN(oct, 1) ? -1 : 1, dy = BITN(oct, 0) ? -1 : 1;
  end_x += dx;
  if(block) {
    end_y += dy;
  }
  int16_t prim_end_x = end_x;
  bool stop_on_x = BITN(mode0, 8);
  const bool stop_on_y = BITN(mode0, 9);
  if(BITN(mode0, 15) && (block ? ((end_x - start_x) >= 32) : (std::abs(end_x - start_x) > 32))) {
    prim_end_x = start_x + 32 * dx;
  }
  if(rex.color_host && rex.rwpacked) {
    stop_on_x = true;
    static const int16_t s_max_host[2][4] = { {4, 4, 2, 1}, {8, 8, 4, 2} };
    const int16_t max_len = s_max_host[rex.rwdouble ? 1 : 0][rex.hostdepth];
    if(std::abs(prim_end_x - start_x) > max_len) {
      prim_end_x = start_x + dx * max_len;
    }
  }
  const bool shade = BITN(mode0, 18), rgbmode = BITN(mode1, 15);
  const bool opaque = BITN(mode0, 16) || BITN(mode0, 17), fastclear = BITN(mode1, 17);
  const uint32_t pattern = BITN(mode0, 12) ? rex.z_pattern : (BITN(mode0, 13) ? rex.ls_pattern : 0xffffffffu);
  const uint32_t color = default_color(rex.color_i);
  const bool lr_abort = BITN(mode0, 19) && dx < 0;
  auto reset_color = [&]() {
    rex.curr_red = rex.color_red;
    rex.curr_alpha = rex.color_alpha;
    rex.curr_green = rex.color_green;
    rex.curr_blue = rex.color_blue;
  };
  const bool fast = !g_np_scalar && dx == 1 && !lr_abort;
  do {
    uint32_t bit = 31;
    const uint64_t tp = NP_TICK();
    if(fast && span_fast(start_x, start_y, prim_end_x, end_x, stop_on_x, pattern, opaque, shade, rgbmode, fastclear,
                         color)) {
      /* the row segment is done */
    }
    else {
      do {
        if(lr_abort) {
          break;
        }
        if(shade) {
          iterate_shade();
        }
        if(BITN(pattern, bit)) {
          if(rex.color_host) {
            output_pixel(start_x, start_y, host_color());
          }
          else if((shade || rgbmode) && !fastclear) {
            output_pixel(start_x, start_y, rgb_color(start_x, start_y));
          }
          else {
            output_pixel(start_x, start_y, color);
          }
        }
        else if(opaque) {
          output_pixel(start_x, start_y, rex.color_back);
        }
        bit = (bit - 1) & 0x1f;
        start_x += dx;
      } while(start_x != prim_end_x && start_x != end_x && stop_on_x);
    }
    NP_ACC(g_np_prof_pix, tp);
#ifdef NP_PROF
    g_np_prof_rows++;
#endif
    if((dx > 0 && start_x >= end_x) || (dx < 0 && start_x <= end_x) || lr_abort) {
      reset_color();
      if(block) {
        start_x = rex.x_save;
        start_y += dy;
      }
    }
  } while(block && start_y != end_y && stop_on_y);
  write_x_start(start_x << 11);
  if(block) {
    write_y_start(start_y << 11);
  }
}

void sgi_newport::scr2scr() {
  const uint32_t mode0 = rex.draw_mode0;
  const bool stop_on_x = BITN(mode0, 8), stop_on_y = BITN(mode0, 9);
  int16_t start_x = rex.x_start_i, start_y = rex.y_start_i;
  int16_t end_x = rex.x_end_i, end_y = rex.y_end_i;
  const uint8_t oct = (rex.bres_octant_inc1 >> 24) & 7;
  const int16_t dx = BITN(oct, 1) ? -1 : 1, dy = BITN(oct, 0) ? -1 : 1;
  end_x += dx;
  end_y += dy;
  do {
    do {
      const int32_t sx = start_x + rex.x_window - 0x1000, sy = start_y + rex.y_window - 0x1000;
      uint32_t src = 0;
      if(sx >= 0 && sy >= 0 && sx < (int32_t)FB_W && sy < (int32_t)FB_H) {
        rb_set_addr((uint32_t)(sy * FB_W + sx));
        src = rb_read();
      }
      output_pixel(start_x + rex.x_move, start_y + rex.y_move, src);
      start_x += dx;
    } while(start_x != end_x && stop_on_x);
    if(start_x == end_x) {
      start_x = rex.x_save;
      start_y += dy;
    }
  } while(start_y != end_y && stop_on_y);
  write_x_start(start_x << 11);
  write_y_start(start_y << 11);
}

void sgi_newport::do_command() {
  const uint64_t tc = NP_TICK();
  do_command_timed();
  NP_ACC(g_np_prof_cmd, tc);
}

void sgi_newport::do_command_timed() {
  const uint32_t mode0 = rex.draw_mode0;
  g_np_cmds++;
  if(g_np_log) {
    const uint32_t m1 = rex.draw_mode1;
    const uint32_t k = (mode0 & 0x3f) | (BITN(mode0, 6) << 6) | (BITN(mode0, 18) << 7) | (BITN(m1, 15) << 8) |
                       (BITN(m1, 18) << 9) | (((m1 >> 3) & 3) << 10) | (((m1 >> 8) & 3) << 12) |
                       (((mode0 >> 12) & 3) << 14) | (BITN(m1, 17) << 16);
    g_np_kinds[k]++;
  }
  const uint8_t opcode = mode0 & 3, adrmode = (mode0 >> 2) & 7;
  go_pixels = 0;
  if(g_drawcap) {
    static std::vector<uint8_t> before, after;
    rb_snap_t rb0, rb1;
    before.resize(sizeof(rex));
    after.resize(sizeof(rex));
    get_state(before.data(), &rb0);
    touch_begin();
    do_command_body(opcode, adrmode);
    const std::vector<touch_t> &t = touch_end();
    get_state(after.data(), &rb1);
    cap_hdr(1);
    const uint64_t icnt = now();
    const uint32_t pc = host_pc();
    const uint8_t hdr[4] = { (uint8_t)(host.asid ? host.asid(host.ctx) : 0), opcode, adrmode, 0 };
    const uint32_t rs = (uint32_t)sizeof(rex), n = (uint32_t)t.size();
    fwrite(&icnt, 8, 1, g_drawcap);
    fwrite(&pc, 4, 1, g_drawcap);
    fwrite(hdr, 4, 1, g_drawcap);
    fwrite(&go_pixels, 8, 1, g_drawcap);
    fwrite(&rs, 4, 1, g_drawcap);
    fwrite(before.data(), rs, 1, g_drawcap);
    fwrite(&rb0, sizeof(rb0), 1, g_drawcap);
    fwrite(&n, 4, 1, g_drawcap);
    fwrite(t.data(), sizeof(touch_t), n, g_drawcap);
    fwrite(after.data(), rs, 1, g_drawcap);
    fwrite(&rb1, sizeof(rb1), 1, g_drawcap);
    if((++g_drawcap_n % g_drawcap_sum) == 0) {
      drawcap_checksum();
    }
  }
  else {
    do_command_body(opcode, adrmode);
  }
  if(g_trace) {
    trace_put(1, 0, 0, (uint8_t)(opcode | (adrmode << 2)), mode0, now(), go_pixels);
  }
}

void sgi_newport::do_command_body(uint8_t opcode, uint8_t adrmode) {
  const uint32_t mode0 = rex.draw_mode0;
  switch(opcode)
    {
    case 1:                                    /* read from framebuffer */
      if(BITN(mode0, 5)) {
        do_setup();
      }
      {
        const uint64_t tp = NP_TICK();
        rex.host_dataport = do_pixel_word_read();
        NP_ACC(g_np_prof_pix, tp);
      }
      break;
    case 2:                                    /* draw */
      if(BITN(mode0, 5)) {
        do_setup();
      }
      switch(adrmode)
        {
        case 0: span_or_block(false); break;
        case 1: span_or_block(true); break;
        case 2: {
          const uint64_t tp = NP_TICK();
          do_iline(default_color(rex.color_i));
          NP_ACC(g_np_prof_pix, tp);
          break;
        }
        case 3: {
          const uint64_t tp = NP_TICK();
          do_fline(default_color(rex.color_i));
          NP_ACC(g_np_prof_pix, tp);
          break;
        }
        case 4: {
          const uint64_t tp = NP_TICK();
          do_iline(default_color(rex.color_i));
          NP_ACC(g_np_prof_pix, tp);
          break;
        }
        default: break;
        }
      break;
    case 3:                                    /* screen to screen copy */
      if(adrmode < 2) {
        if(BITN(mode0, 5)) {
          do_setup();
        }
        const uint64_t tp = NP_TICK();
        scr2scr();
        NP_ACC(g_np_prof_pix, tp);
      }
      break;
    default:
      break;
    }
}

/* ---------------- display ---------------- */
void sgi_newport::render(uint32_t *dst) {
  const uint32_t *pal = cmap[0].pal;
  const uint16_t cursor_msb = (uint16_t)(xmap[0].cursor_cmap << 5);
  const uint16_t popup_msb = (uint16_t)(xmap[0].popup_cmap << 5);
  const bool is_8bpp = BITN(xmap[0].config, 2) != 0;
  const uint32_t rows = (vc2.readout_y1 > vc2.readout_y0) ? (uint32_t)(vc2.readout_y1 - vc2.readout_y0) : DISP_H;
  for(uint32_t y = 0; y < DISP_H; y++) {
    uint32_t *d = &dst[y * DISP_W];
    if(y >= rows) {
      memset(d, 0, DISP_W * 4);
      continue;
    }
    const uint32_t *ci = &rb.rgbci[y * FB_W];
    const uint32_t *aux = &rb.cidaux[y * FB_W];
    /* the DID line for this scanline, then its first entry.  Local pointers (MAME
     * advances the VC2's own): render() writes no device state, so a display thread
     * may call it while another thread drives the board */
    const uint16_t did_frame_ptr = (uint16_t)(vc2.did_entry + y);
    uint16_t did_line_ptr = vc2.ram[did_frame_ptr & (VC2_RAM_SIZE - 1)];
    uint16_t did = vc2.ram[did_line_ptr & (VC2_RAM_SIZE - 1)];
    uint32_t te = 0, pix_mode = 0, pix_size = 0, aux_mode = 0, aux_msb = 0, ci_msb = 0;
    auto load_mode = [&]() {
      te = xmap[0].mode[did & 0x1f];
      pix_mode = (te >> 8) & 3;
      pix_size = (te >> 10) & 3;
      aux_mode = (te >> 16) & 7;
      aux_msb = (te >> 11) & 0x1f00;
      switch(pix_mode)
        {
        case 0: ci_msb = (te & 0xf8) << 5; break;
        case 1: ci_msb = 0x1d00; break;
        case 2: ci_msb = 0x1e00; break;
        default: ci_msb = 0x1f00; break;
        }
      did_line_ptr++;
      did = vc2.ram[did_line_ptr & (VC2_RAM_SIZE - 1)];
    };
    load_mode();
    for(uint32_t x = 0; x < DISP_W; x++) {
      if((uint16_t)x == (did >> 5)) {
        load_mode();
      }
      const uint32_t c = ci[x], a = aux[x];
      uint32_t out = 0;
      bool done = true;
      const uint8_t cur = vc2.cursor_pixel((int)x, (int)y);
      if(cur) {
        out = pal[cursor_msb | cur];
      }
      else if(a & 0xcc) {
        out = pal[popup_msb | ((a >> 2) & 3)];
      }
      else {
        done = false;
        if(aux_mode != 0) {
          if(is_8bpp) {
            switch(aux_mode)
              {
              case 1: out = pal[aux_msb | ((a >> 8) & 3)]; done = true; break;
              case 2: if((a >> 8) & 3) { out = pal[aux_msb | ((a >> 8) & 3)]; done = true; } break;
              case 6: {
                const uint32_t p = (a >> (BITN(te, 1) ? 9 : 8)) & 1;
                if(p) { out = pal[aux_msb | p]; done = true; }
                break;
              }
              case 7: {
                const uint32_t p = (a >> 8) & 1;
                out = p ? pal[aux_msb | p] : pal[aux_msb | ((a >> 9) & 1)];
                done = true;
                break;
              }
              default: break;
              }
          }
          else {
            switch(aux_mode)
              {
              case 1: out = pal[aux_msb | ((a >> 8) & 0xff)]; done = true; break;
              case 2: {
                const uint32_t p = ((a >> 8) & 0xf) | ((a >> 16) & 0xf0);
                if(p) { out = pal[aux_msb | p]; done = true; }
                break;
              }
              case 6: {
                const uint32_t p = (a >> (BITN(te, 1) ? 12 : 8)) & 0xf;
                if(p) { out = pal[aux_msb | p]; done = true; }
                break;
              }
              case 7: {
                const uint32_t p = (a >> 8) & 0xf;
                out = p ? pal[aux_msb | p] : pal[aux_msb | ((a >> 12) & 0xf)];
                done = true;
                break;
              }
              default: break;
              }
          }
        }
      }
      if(!done) {
        if(pix_mode == 0) {                    /* colour index */
          switch(pix_size)
            {
            case 0: out = pal[ci_msb | ((c >> (BITN(te, 0) ? 4 : 0)) & 0xf)]; break;
            case 1: out = pal[ci_msb | (c & 0xff)]; break;
            case 2: out = pal[(ci_msb & 0x1000) | ((c >> (BITN(te, 0) ? 12 : 0)) & 0xfff)]; break;
            default: out = 0; break;
            }
        }
        else {                                 /* RGB maps 0/1/2 (BGR in the framebuffer) */
          uint32_t rgb = 0;
          switch(pix_size)
            {
            case 0: rgb = convert_depth((c >> (BITN(te, 0) ? 4 : 0)) & 0xf, 0, 3); break;
            case 1: rgb = convert_depth((c >> (BITN(te, 0) ? 8 : 0)) & 0xff, 1, 3); break;
            case 2: rgb = convert_depth((c >> (BITN(te, 0) ? 12 : 0)) & 0xfff, 2, 3); break;
            default: rgb = c & 0xffffff; break;
            }
          out = ((rgb & 0xff) << 16) | (rgb & 0xff00) | ((rgb >> 16) & 0xff);
          /* XMAP9 spec 4.2: an RGB pixel goes through the CMAP one component at a time --
           * red indexes the red byte of map entry ci_msb + R (0x1d00/0x1e00/0x1f00 for
           * maps 0/1/2), green and blue likewise.  Maps 1/2 are the guest's gamma tables
           * (MAME painted them solid green/magenta); map 0 stays direct for now.  Keep in
           * step with indysim sgi_newport.cc. */
          if(pix_mode != 1) {
            out = (pal[ci_msb | ((out >> 16) & 0xff)] & 0xff0000) |
                  (pal[ci_msb | ((out >> 8) & 0xff)] & 0x00ff00) |
                  (pal[ci_msb | (out & 0xff)] & 0x0000ff);
          }
        }
      }
      d[x] = lut_r[(out >> 16) & 0xff] | lut_g[(out >> 8) & 0xff] | lut_b[out & 0xff];
    }
  }
}

void sgi_newport::dump_ppm(const char *path) {
  uint32_t *buf = new uint32_t[DISP_W * DISP_H];
  render(buf);
  FILE *f = fopen(path, "wb");
  if(f == nullptr) {
    fprintf(stderr, "[newport] cannot write %s\n", path);
    delete [] buf;
    return;
  }
  fprintf(f, "P6\n%u %u\n255\n", DISP_W, DISP_H);
  uint8_t *row = new uint8_t[DISP_W * 3];
  for(uint32_t y = 0; y < DISP_H; y++) {
    for(uint32_t x = 0; x < DISP_W; x++) {
      const uint32_t p = buf[y * DISP_W + x];
      row[3 * x + 0] = (uint8_t)(p >> 16);
      row[3 * x + 1] = (uint8_t)(p >> 8);
      row[3 * x + 2] = (uint8_t)p;
    }
    fwrite(row, 1, DISP_W * 3, f);
  }
  fclose(f);
  delete [] row;
  delete [] buf;
  fprintf(stderr, "[newport] frame %llu -> %s\n", (unsigned long long)nframes, path);
}
