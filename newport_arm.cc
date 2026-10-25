/* newport_arm.cc -- the ARM half of Newport on henry (docs/newport-on-henry.md).
 *
 * The PL turns every CPU access to the GIO window 0x1f0f0000-0x1f0f1fff into a record
 * in the capture ring (staging 0x1E200000) and holds a load until its answer arrives.
 * gio_service.h (the SAME source henry_tb runs) drains the ring into sgi_newport (the
 * same model as henry_tb and indysim) and answers the READs; this file supplies the
 * thread, the time base, the retrace timer and the optional X11 window.
 *
 * AXI-Lite from two threads: Driver::write32 is one 32-bit store to the register
 * window, i.e. one independent AXI-Lite write.  This thread writes only the GIO
 * registers (0x20-0x23, 0x2a) and reads only 0x21 (the held GIO load, a plain
 * register with no select), so nothing it does can interleave with a select-then-read
 * sequence on the main thread (GPR readback 14/0xe, ...).  No lock. */
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <ctime>
#include <pthread.h>
#include <sched.h>
#include "driver.hh"
#include "gio_service.h"
#include "sgi_newport.hh"
#include "sgi_ps2.hh"
#include "newport_arm.hh"
#ifdef NEWPORT_X11
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#endif

/* time base: 10 ns units (the core's 100 MHz cycle, as in henry_tb) -- the model's
 * DCB absent-slave timeout is 100000 units = 1 ms, as MAME */
static uint64_t np_now() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ((uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec) / 10;
}
static const uint64_t VBLANK_PERIOD = 100000000ull / 60;   /* 60 Hz */
static const uint64_t IDLE_SPIN = 100000;                  /* 1 ms of empty polls, then back off */
static const long IDLE_SLEEP_NS = 20000;
static const long FRAME_NS = 40000000;                     /* X11 window: 25 Hz */
static const int EDGE_PUSH = 64;                           /* pointer edge re-sync (pixels) */

static Driver *g_d = nullptr;
static sgi_newport *g_np = nullptr;
/* the IOC2 8042 keyboard/mouse (gio ring space 1).  Only the service thread touches the
 * model; the X thread hands it host input through g_input. */
static sgi_ps2 *g_ps2 = nullptr;
struct host_input_t {
  bool key;                  /* else mouse */
  std::string name;
  bool down;
  int dx, dy;
  uint8_t buttons;
};
static std::mutex g_input_lock;
static std::deque<host_input_t> g_input;
static gio_svc_t g_gio;
static std::atomic<bool> g_stop(false);
/* heap-allocated and never destroyed: an exit() elsewhere in mips-axi must not run
 * ~thread on a joinable thread (std::terminate) */
static std::thread *g_svc_thread = nullptr, *g_x11_thread = nullptr;
static uint64_t g_vblanks = 0, g_naps = 0, g_frames = 0, g_puts = 0, g_skipped = 0;

/* Idle back-off.  A READ holds the MIPS core until we answer it, so every nap that
 * a READ lands in is pure core stall: Doom under Xsgi spent 6% of the core in the
 * Newport ISR's REX3 STATUS load, 1-16 ms per load.  Three rules, all cheap:
 *  - never nap while a device IRQ is asserted: the guest ISR's status READ is
 *    microseconds away (the read is what drops VRINT);
 *  - never nap past the next retrace: we raise it ourselves;
 *  - the idle-spin window adapts: a READ seen right after we started napping means
 *    we gave up too early, so double it; halve it after each quiet window.
 * NP_SVC_FIXED=1 restores the old fixed 1 ms window, for A/B. */
static const uint64_t SPIN_MIN = 10000, SPIN_MAX = 1000000;   /* 0.1 ms .. 10 ms */
static const uint64_t STAT_PERIOD = 1000000000ull;            /* 10 s */

static void svc_main() {
  static const bool fixed = getenv("NP_SVC_FIXED") != nullptr;
  uint64_t last_busy = np_now(), spin = IDLE_SPIN, nap_start = 0;
  /* READ service stats: polls that found a READ, and how stale that poll was */
  uint64_t last_poll = last_busy, stat_at = last_busy + STAT_PERIOD;
  uint64_t rd0 = 0, early0 = 0, rd_late = 0, gap_max = 0, naps0 = 0;
  while(!g_stop.load(std::memory_order_relaxed)) {
    const uint64_t rd_before = g_gio.reads;
    const uint32_t n = gio_poll(&g_gio);
    const uint64_t t = np_now();
    if(g_gio.reads != rd_before) {
      const uint64_t gap = t - last_poll;
      if(gap > 100000) {
        rd_late++;                 /* the READ waited > 1 ms on us */
      }
      if(gap > gap_max) {
        gap_max = gap;
      }
      if(!fixed && nap_start && (t - nap_start) < spin) {
        spin = std::min(spin * 2, SPIN_MAX);
      }
      nap_start = 0;
    }
    last_poll = t;
    if(t >= stat_at) {
      fprintf(stderr, "[gio-svc] %llu reads (%llu early), %llu waited >1ms, worst poll gap %.2f ms, %llu naps, spin %.2f ms%s\n",
              (unsigned long long)(g_gio.reads - rd0), (unsigned long long)(g_gio.early_reads - early0),
              (unsigned long long)rd_late, gap_max / 1e5,
              (unsigned long long)(g_naps - naps0), spin / 1e5, fixed ? " (fixed)" : "");
      rd0 = g_gio.reads;
      early0 = g_gio.early_reads;
      naps0 = g_naps;
      rd_late = 0;
      gap_max = 0;
      stat_at = t + STAT_PERIOD;
    }
    if(t >= g_np->next_vblank_at()) {
      g_np->vblank(t);
      g_vblanks++;
      {
        std::lock_guard<std::mutex> lk(g_input_lock);
        while(!g_input.empty()) {
          const host_input_t &h = g_input.front();
          if(h.key) {
            g_ps2->key(h.name, h.down);
          }
          else {
            g_ps2->mouse(h.dx, h.dy, h.buttons);
          }
          g_input.pop_front();
        }
      }
      g_ps2->service(t);           /* one queued kbd/mouse event per retrace */
    }
    if(n) {
      last_busy = t;               /* records pending or a load held: never sleep here */
    }
    else if(fixed) {
      if(t - last_busy > IDLE_SPIN) {
        const struct timespec ts = { 0, IDLE_SLEEP_NS };
        nanosleep(&ts, nullptr);
        g_naps++;
      }
    }
    else if(g_gio.irq != 0) {
      last_busy = t;               /* an ISR read is imminent */
    }
    else if(t - last_busy > spin) {
      /* the nap must end before the next retrace (10 ns units -> ns, minus 50 us) */
      const uint64_t to_vb = g_np->next_vblank_at() > t ? (g_np->next_vblank_at() - t) * 10 : 0;
      if(to_vb > 50000 + IDLE_SLEEP_NS) {
        if(!nap_start) {
          nap_start = t;
        }
        const struct timespec ts = { 0, IDLE_SLEEP_NS };
        nanosleep(&ts, nullptr);
        g_naps++;
      }
      if(nap_start && (t - nap_start) > spin) {
        spin = std::max(spin / 2, SPIN_MIN);   /* a quiet window: back off sooner */
        nap_start = t;
      }
    }
  }
}

#ifdef NEWPORT_X11
/* X keysym (unshifted) -> sgi_ps2 key name; nullptr = not on the keyboard */
static const char *x_key_name(KeySym k) {
  static char one[2];
  if((k >= XK_a && k <= XK_z) || (k >= XK_0 && k <= XK_9)) {
    one[0] = (char)k;
    one[1] = 0;
    return one;
  }
  switch(k)
    {
    case XK_minus: return "-"; case XK_equal: return "="; case XK_bracketleft: return "[";
    case XK_bracketright: return "]"; case XK_semicolon: return ";"; case XK_apostrophe: return "'";
    case XK_grave: return "`"; case XK_backslash: return "\\"; case XK_comma: return ",";
    case XK_period: return "."; case XK_slash: return "/";
    case XK_Escape: return "esc"; case XK_BackSpace: return "backspace"; case XK_Tab: return "tab";
    case XK_Return: return "enter"; case XK_space: return "space"; case XK_Caps_Lock: return "capslock";
    case XK_Control_L: case XK_Control_R: return "ctrl";
    case XK_Shift_L: return "shift"; case XK_Shift_R: return "rshift";
    case XK_Alt_L: case XK_Alt_R: case XK_Meta_L: case XK_Meta_R: return "alt";
    case XK_F1: return "f1"; case XK_F2: return "f2"; case XK_F3: return "f3"; case XK_F4: return "f4";
    case XK_F5: return "f5"; case XK_F6: return "f6"; case XK_F7: return "f7"; case XK_F8: return "f8";
    case XK_F9: return "f9"; case XK_F10: return "f10"; case XK_F11: return "f11"; case XK_F12: return "f12";
    case XK_Insert: return "insert"; case XK_Delete: return "delete"; case XK_Home: return "home";
    case XK_End: return "end"; case XK_Prior: return "pgup"; case XK_Next: return "pgdn";
    case XK_Up: return "up"; case XK_Down: return "down"; case XK_Left: return "left"; case XK_Right: return "right";
    default: return nullptr;
    }
}

static void push_input(const host_input_t &h) {
  std::lock_guard<std::mutex> lk(g_input_lock);
  g_input.push_back(h);
}
#endif

#ifdef NEWPORT_X11
/* XPutImage of the rows that changed since the last frame (everything after an
 * Expose); render() writes no device state, so it runs here, off the service thread */
static void x11_main(std::string disp) {
  const int w = sgi_newport::DISP_W, h = sgi_newport::DISP_H;
  Display *dpy = XOpenDisplay(disp.c_str());
  if(dpy == nullptr) {
    fprintf(stderr, "[newport] cannot open X display %s: running headless\n", disp.c_str());
    return;
  }
  const int scr = DefaultScreen(dpy);
  Visual *vis = DefaultVisual(dpy, scr);
  if(DefaultDepth(dpy, scr) != 24 || vis->red_mask != 0xff0000 || vis->blue_mask != 0xff) {
    fprintf(stderr, "[newport] display %s is not 24-bit RGB888: running headless\n", disp.c_str());
    XCloseDisplay(dpy);
    return;
  }
  Window win = XCreateSimpleWindow(dpy, RootWindow(dpy, scr), 0, 0, w, h, 0, 0, BlackPixel(dpy, scr));
  XStoreName(dpy, win, "Newport (henry)");
  XSelectInput(dpy, win, ExposureMask | KeyPressMask | KeyReleaseMask |
               ButtonPressMask | ButtonReleaseMask | PointerMotionMask);
  /* PS/2 mice are relative: send pointer deltas, coalesced per frame */
  int last_x = -1, last_y = -1, acc_dx = 0, acc_dy = 0;
  uint8_t buttons = 0;
  XMapWindow(dpy, win);
  GC gc = DefaultGC(dpy, scr);
  std::vector<uint32_t> cur((size_t)w * h), prev((size_t)w * h, 0);
  XImage *img = XCreateImage(dpy, vis, 24, ZPixmap, 0, reinterpret_cast<char *>(cur.data()), w, h, 32, w * 4);
  bool full = true;
  uint64_t last_gen = ~0ULL;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  fprintf(stderr, "[newport] window on %s\n", disp.c_str());
  while(!g_stop.load(std::memory_order_relaxed)) {
    while(XPending(dpy)) {
      XEvent e;
      XNextEvent(dpy, &e);
      if(e.type == Expose) {
        full = true;
      }
      else if(e.type == KeyPress || e.type == KeyRelease) {
        /* X autorepeat sends Release+Press pairs; the guest repeats keys itself */
        if(e.type == KeyRelease && XPending(dpy)) {
          XEvent nx;
          XPeekEvent(dpy, &nx);
          if(nx.type == KeyPress && nx.xkey.time == e.xkey.time && nx.xkey.keycode == e.xkey.keycode) {
            XNextEvent(dpy, &nx);
            continue;
          }
        }
        const char *nm = x_key_name(XLookupKeysym(&e.xkey, 0));
        if(nm) {
          push_input({true, nm, e.type == KeyPress, 0, 0, 0});
        }
      }
      else if(e.type == MotionNotify) {
        if(last_x >= 0) {
          acc_dx += e.xmotion.x - last_x;
          acc_dy += e.xmotion.y - last_y;
        }
        last_x = e.xmotion.x;
        last_y = e.xmotion.y;
        /* edge re-sync: the guest cursor stops at the screen edge while ours may keep
         * going, which leaves a lasting offset; at our edge, push the guest past its
         * edge too so both are clamped to the same place */
        if(last_x <= 0) {
          acc_dx -= EDGE_PUSH;
        }
        else if(last_x >= w - 1) {
          acc_dx += EDGE_PUSH;
        }
        if(last_y <= 0) {
          acc_dy -= EDGE_PUSH;
        }
        else if(last_y >= h - 1) {
          acc_dy += EDGE_PUSH;
        }
      }
      else if(e.type == ButtonPress || e.type == ButtonRelease) {
        /* X 1/2/3 = left/middle/right -> PS/2 bits 0x01 / 0x04 / 0x02 */
        static const uint8_t s_bit[4] = { 0, 0x01, 0x04, 0x02 };
        if(e.xbutton.button >= 1 && e.xbutton.button <= 3) {
          const uint8_t b = s_bit[e.xbutton.button];
          buttons = (e.type == ButtonPress) ? (uint8_t)(buttons | b) : (uint8_t)(buttons & ~b);
          push_input({false, "", false, acc_dx, acc_dy, buttons});
          acc_dx = 0;
          acc_dy = 0;
        }
      }
    }
    if(acc_dx != 0 || acc_dy != 0) {
      push_input({false, "", false, acc_dx, acc_dy, buttons});
      acc_dx = 0;
      acc_dy = 0;
    }
    /* nothing written to the board since the last frame we drew: the picture is
     * unchanged, so skip the full-frame render + diff (25 Hz of A53 work otherwise).
     * Read the generation BEFORE render: a write landing mid-render moves it again,
     * so the next tick redraws */
    const uint64_t gen = g_np->display_gen();
    if(!full && gen == last_gen) {
      g_frames++;
      g_skipped++;
      next.tv_nsec += FRAME_NS;
      if(next.tv_nsec >= 1000000000) {
        next.tv_sec++;
        next.tv_nsec -= 1000000000;
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
      continue;
    }
    last_gen = gen;
    g_np->render(cur.data());
    int y0 = 0, y1 = h - 1;
    if(!full) {
      while(y0 < h && memcmp(&cur[(size_t)y0 * w], &prev[(size_t)y0 * w], w * 4) == 0) {
        y0++;
      }
      while(y1 > y0 && memcmp(&cur[(size_t)y1 * w], &prev[(size_t)y1 * w], w * 4) == 0) {
        y1--;
      }
    }
    if(y0 < h) {
      XPutImage(dpy, win, gc, img, 0, y0, 0, y0, w, y1 - y0 + 1);
      XFlush(dpy);
      memcpy(&prev[(size_t)y0 * w], &cur[(size_t)y0 * w], (size_t)(y1 - y0 + 1) * w * 4);
      g_puts++;
    }
    full = false;
    g_frames++;
    next.tv_nsec += FRAME_NS;
    if(next.tv_nsec >= 1000000000) {
      next.tv_sec++;
      next.tv_nsec -= 1000000000;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
  }
  img->data = nullptr;               /* the vector owns the pixels */
  XDestroyImage(img);
  XCloseDisplay(dpy);
}
#endif

void newport_arm_start(Driver *d, uint8_t *stg, const char *display, int cpu) {
  g_d = d;
  np_host_t host;
  host.now = [](void *) -> uint64_t { return np_now(); };
  host.irq = [](void *, uint8_t local1) { gio_set_irq(&g_gio, GIO_IRQ_LOCAL1, local1); };
  g_np = new sgi_newport(host, VBLANK_PERIOD);
  ps2_host_t phost;
  phost.now = [](void *) -> uint64_t { return np_now(); };
  phost.irq = [](void *, bool level) { gio_set_irq(&g_gio, GIO_IRQ_KBD, level ? GIO_IRQ_KBD : 0); };
  phost.ppm = [](void *, const char *path) { g_np->dump_ppm(path); };
  g_ps2 = new sgi_ps2(phost);
  ring_io_t io;
  io.stg = stg;
  io.doorbell = [](void *, uint32_t reg, uint32_t v) { g_d->write32(reg, v); };
  io.ctx = nullptr;
  gio_dev_t dev;
  dev.ctx = g_np;
  dev.write = [](void *ctx, uint32_t off, uint64_t v, uint32_t sz) { static_cast<sgi_newport *>(ctx)->write(off, v, sz); };
  dev.read = [](void *ctx, uint32_t off, uint32_t sz) -> uint64_t { return static_cast<sgi_newport *>(ctx)->read(off, sz); };
  /* debug knobs, read once: NP_NO_GFXBUSY=1 REX3 never reports busy during a VDMA
   * transfer; NP_NO_HOLD=1 CPU stores inside a transfer apply at once; NP_VDMA_LOG=1 a
   * line per transfer (gio_service.h) */
  static const bool g_no_gfxbusy = getenv("NP_NO_GFXBUSY") != nullptr;
  dev.dma = nullptr;
  if(!g_no_gfxbusy) {
    dev.dma = [](void *ctx, bool on) { static_cast<sgi_newport *>(ctx)->set_in_dma(on); };
  }
  gio_dev_t dev2;
  dev2.ctx = g_ps2;
  dev2.dma = nullptr;
  /* 8042: data at byte +3 (0x1fbd9843), command/status at +7 (0x1fbd9847) */
  dev2.write = [](void *ctx, uint32_t off, uint64_t v, uint32_t) { static_cast<sgi_ps2 *>(ctx)->write(((off >> 2) & 1) != 0, (uint8_t)v); };
  dev2.read = [](void *ctx, uint32_t off, uint32_t) -> uint64_t { return static_cast<sgi_ps2 *>(ctx)->read(((off >> 2) & 1) != 0); };
  gio_init(&g_gio, io, dev, dev2);
  g_gio.no_hold = getenv("NP_NO_HOLD") != nullptr;
  g_gio.log = getenv("NP_VDMA_LOG") != nullptr;
  /* NP_NO_EARLY=1: answer REX3 STATUS reads in ring order too (gio_answer_early) */
  if(getenv("NP_NO_EARLY") == nullptr) {
    g_gio.held_rd = [](void *) -> uint32_t { return g_d->read32(GIO_RD_HELD); };
  }
  fprintf(stderr, "[newport] VDMA: REX3 busy during transfers %s, CPU stores held %s, log %s\n",
          g_no_gfxbusy ? "OFF" : "on", g_gio.no_hold ? "OFF" : "on", g_gio.log ? "on" : "off");
  g_svc_thread = new std::thread(svc_main);
  if(cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if(pthread_setaffinity_np(g_svc_thread->native_handle(), sizeof(set), &set) != 0) {
      fprintf(stderr, "[newport] cannot pin the GIO service thread to cpu %d\n", cpu);
    }
  }
  printf("[newport] GIO window service on cpu %d, ring at staging +0x%06x\n", cpu, GIO_RING);
  if(display && display[0]) {
#ifdef NEWPORT_X11
    g_x11_thread = new std::thread(x11_main, std::string(display));
#else
    fprintf(stderr, "[newport] built without X11: --newport-display ignored\n");
#endif
  }
}

void newport_arm_stop() {
  if(g_np == nullptr) {
    return;
  }
  g_stop = true;
  g_svc_thread->join();
  if(g_x11_thread) {
    g_x11_thread->join();
  }
  fprintf(stderr, "[newport] %llu stores, %llu reads, %llu split, %llu bad recno; %llu vblanks, %llu naps; "
          "window %llu frames (%llu unchanged, not rendered), %llu puts\n",
          (unsigned long long)g_gio.stores, (unsigned long long)g_gio.reads, (unsigned long long)g_gio.split,
          (unsigned long long)g_gio.bad_recno, (unsigned long long)g_vblanks, (unsigned long long)g_naps,
          (unsigned long long)g_frames, (unsigned long long)g_skipped, (unsigned long long)g_puts);
  delete g_np;                       /* NEWPORT_PPM: the final frame */
  g_np = nullptr;
}
