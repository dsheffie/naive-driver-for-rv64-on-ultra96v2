#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <set>
#include <cstring>
#include <ctime>
#include <csignal>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <fcntl.h>
#include <termios.h>
#include "scsi_arm.h"
#include "enet_arm.h"

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <capstone/capstone.h>

#include "helper.hh"
#include "driver.hh"
#include "mon.hh"
#include "helper.hh"
#include "saveState.hh"
#include "disassemble.hh"

#define POLL_FREQ ((1UL<<12)-1)
#define STEP (1U<<31)

#define CONTROL_REG 0
#define STATUS_REG 1
#define RAM_REG 2
#define PC_REG 5

#define MB ((1UL<<20))

static const uint32_t control = 0xA0050000;
static uint32_t g_l2nc = 0;   /* ctrl bit20 = L2 no-cache (set BEFORE go via L2_NOCACHE env); OR'd into every control write */
static uint32_t g_deeptrace = 0;  /* ctrl bit21 = DRAM control-flow deep trace (TRACE_DEEP env); OR'd into control writes so it survives re-arms */
static const bool g_characterize = getenv("RT_CHARACTERIZE") != nullptr;
static const bool g_bp_dump = getenv("RT_BP_DUMP") != nullptr;  /* ARMRT: a bp_pc retire-match ALWAYS dumps+stops (backward-window catch on an off-golden trigger pc) */  /* ARMRT: log each crash's sp/VA + re-arm (many samples/boot) instead of full-dump+stop */


static const uint64_t memsize = 496*MB;
static uint64_t phys_addr = ~0UL;

static uint8_t *c_addr = nullptr;
static uint32_t rdbe(uint32_t pa){ return ((uint32_t)c_addr[pa]<<24)|((uint32_t)c_addr[pa+1]<<16)|((uint32_t)c_addr[pa+2]<<8)|(uint32_t)c_addr[pa+3]; }
static volatile sig_atomic_t g_dumpreq = 0;
static int g_armed = 0;
static uint32_t g_wp_pc[16]; static uint32_t g_wp_n = 0;   // watchpoint freeze ring
static uint32_t g_sc_val[32], g_sc_res[32], g_sc_addr[32]; static uint32_t g_sc_n = 0;  // SC ring
static void sigusr1_handler(int){ g_dumpreq = 1; }

/* --- XPATH cross-path experiment (MIPS->ARM): the instant we see a console
 *     char (S00 leg) we read ring[xk] from DDR (M00 leg) and check ==xk. --- */
static bool     xpath_mode = false, x_started = false;
static unsigned xk = 0, x_ok = 0, x_bad = 0, x_shown = 0;
static bool done = false;


scsi_disk g_scsi_disk;
enet_tap g_enet_tap;


inline bool cpu_stopped(const rvstatus &rs) {
  return rs.s.break_ or rs.s.ud or rs.s.bad_addr or rs.s.monitor;
}

static inline uint64_t read64(Driver *d, int r) {
  return ((static_cast<uint64_t>(d->read32(r+1)) << 32) | d->read32(r));
}

inline double timestamp() {
  struct timeval t;
  gettimeofday(&t, nullptr);
  return t.tv_sec + static_cast<double>(t.tv_usec)*1e-6;
}

static Driver *d = nullptr;

static inline void report_status() {}

static inline bool read_char_fifo() {
  int v = d->read32(0x3a) & 255;
  int wptr =v&0xf, rptr = (v>>4)&0xf;
  if(wptr == rptr) {
    return false;
  }
  if(xpath_mode) {
    int sc = d->read32(0x3b) & 255;        /* observe the signal (char) on the S00 leg */
    if(!x_started) {
      if(sc == 0x02) { x_started = true; xk = 0; }   /* START marker: align to the first dot */
    } else if(sc == '.') {
      /* DATA read from DDR (M00 leg), right after observing the signal */
      volatile uint32_t *ring = (volatile uint32_t*)(c_addr + 0x09000000);
      uint32_t w = __builtin_bswap32(ring[xk & 0xFFFFu]);   /* MIPS is big-endian */
      if(w == xk) x_ok++;
      else { x_bad++; if(x_shown < 12){ printf("XPATH MISMATCH k=%u got=0x%x\n", xk, w); std::fflush(nullptr); x_shown++; } }
      xk++;
    }
    d->write32(0x3a, 1);
    d->write32(0x3a, 0);
    return true;
  }
  int c = d->read32(0x3b);
  int cc = (c==0 ? '\n' : c);
  printf("%c", c==0 ? '\n' : c);
  mon_console_out(cc);
  std::fflush(nullptr);
  /* Auto-quit at end-of-run so the guest can be looped unattended: the go/spec
   * harness prints "fpga_done" and the kernel prints "reboot: Power down".
   * Rolling tail buffer -> strstr for either sentinel, then clean exit (atexit
   * restore_tty runs; the flock on /tmp/mips-axi.lock releases on process exit). */
  {
    static char et[32] = {0};
    size_t n = strlen(et);
    if(n >= sizeof(et) - 1) { memmove(et, et + 1, sizeof(et) - 2); n--; }
    et[n] = (char)cc;
    et[n + 1] = '\0';
    if(strstr(et, "fpga_done") /*|| strstr(et, "Power down")*/) {
      std::fflush(nullptr);
      exit(0);
    }
  }
  d->write32(0x3a, 1);
  d->write32(0x3a, 0);
  return true;
}

/* SCC serial Rx (ARM/PS -> core): deliver a received byte into the core's Rx
 * FIFO.  Flow control: reg 0x3a bit8 = Rx-FIFO full (poll before pushing).  The
 * push request is reg 0x3b bit8 + the byte in [7:0]; S00_AXI edge-detects bit8
 * into a 1-cycle scc_rx_push pulse (mirrors the putchar pop handshake), so we
 * raise then clear it. */
static inline bool scc_rx_full() {
  return (d->read32(0x3a) >> 8) & 1;
}
static inline void scc_rx_push_byte(uint8_t ch) {
  while(scc_rx_full()) { /* spin: core hasn't drained the Rx FIFO yet */ }
  d->write32(0x3b, (1u << 8) | ch);  /* push=1 + byte -> edge -> scc_rx_push */
  d->write32(0x3b, 0);               /* clear push so the next byte re-edges  */
}
/* Pump locally-typed stdin bytes into the core's Rx FIFO so you can type at the
 * guest console.  On a TTY we switch stdin to raw-ish mode -- char-at-a-time, no
 * local echo (the guest echoes) -- and restore the terminal on exit.  ISIG is
 * kept ON so Ctrl-C still stops the driver (not sent to the guest).  Reads are
 * non-blocking so the console-drain poll loop never stalls on a keystroke.
 * If stdin is not a TTY (piped/redirected) we skip the termios dance and just
 * pump bytes, so scripted input still works. */
static struct termios g_saved_tio;
static bool           g_tio_saved = false;
static void restore_tty() {
  if(g_tio_saved) { tcsetattr(0, TCSANOW, &g_saved_tio); g_tio_saved = false; }
}
static inline void pump_stdin_to_rx() {
  static bool init = false;
  if(!init) {
    init = true;
    fcntl(0, F_SETFL, fcntl(0, F_GETFL, 0) | O_NONBLOCK);
    if(isatty(0) && tcgetattr(0, &g_saved_tio) == 0) {
      g_tio_saved = true;
      atexit(restore_tty);
      struct termios raw = g_saved_tio;
      raw.c_lflag &= ~(ICANON | ECHO);  /* char-at-a-time, no local echo */
      raw.c_iflag &= ~(ICRNL);          /* pass CR through unchanged (no CR->NL) */
      tcsetattr(0, TCSANOW, &raw);
    }
  }
  unsigned char ch;
  while(read(0, &ch, 1) == 1) scc_rx_push_byte(ch);
}

void sigintHandler(int id) {
  done = true;
}

uint32_t loadelf(const char* fn, uint8_t *mem, bool sgi_mode);

bool cmdline(int argc,
	     char *argv[],
	     bool &initialize,
	     bool &silent,
	     std::string &chpt_name,
	     uint32_t &max_fetches,
	     uint64_t &max_iters,
	     bool &sgi_mode,
	     bool &single_step,
	     std::string &arcs_image,
	     std::string &start_pc,
	     std::string &cimg_name,
	     std::string &disk_name);

// Silicon checkpoint resume: replicate henry_tb's fpga_map so cimg pages land in DRAM
// exactly where the core (via the AXI master's IP22 fold) will read them.
static uint32_t cimg_fpga_map(uint32_t cpuaddr) {
  if(cpuaddr >= 0x08000000u && cpuaddr <= 0x17ffffffu) return cpuaddr & 0x0fffffffu;
  if(cpuaddr >= 0x1f000000u && cpuaddr <= 0x1fffffffu) return 0x10000000u | (cpuaddr & 0x00ffffffu);
  return cpuaddr;
}
// Load a ckpt2preamble .cimg ([u64 icnt][u32 num][ {u32 pa,4096B} ..]) into DRAM.
static void load_cimg(const char *path, uint8_t *cbase) {
  FILE *f = fopen(path, "rb");
  if(!f) { fprintf(stderr, "cannot open cimg %s\n", path); exit(-1); }
  uint64_t icnt = 0; uint32_t n = 0;
  if(fread(&icnt, 8, 1, f) != 1 || fread(&n, 4, 1, f) != 1) { fprintf(stderr, "bad cimg\n"); exit(-1); }
  uint32_t loaded = 0;
  for(uint32_t i = 0; i < n; i++) {
    uint32_t va = 0; uint8_t data[4096];
    if(fread(&va, 4, 1, f) != 1 || fread(data, 1, 4096, f) != 4096) break;
    uint32_t off = cimg_fpga_map(va);
    if((uint64_t)off + 4096 <= 0x1f000000ULL) { memcpy(cbase + off, data, 4096); loaded++; }
  }
  fclose(f);
  printf("[cimg] loaded %u/%u pages into DRAM (base icnt=%llu)\n", loaded, n, (unsigned long long)icnt);
}

static void dump_registers(Driver *d) {
  printf("pc=%x cause=%u, sr %x |", d->read32(7), d->read32(0x26)&31, d->read32(0x16));
  for(int i=0;i<32;i++){ d->write32(14,i); printf(" %s=%x", getGPRName(i).c_str(), d->read32(0xe)); }
  printf("\n");
}

/* DRAM control-flow deep trace: read the ring back from shared DRAM (0x18000000) and
 * write the raw {from,to} records to a file for offline rept_align_df.py. */
static void dump_dram_trace(Driver* d) {
  const uint32_t BASE   = 0x18000000u;
  const uint32_t RINGSZ = 112u*1024u*1024u;            /* 64 MB (dram_trace TRACE_MASK) */
  uint32_t wbytes = d->read32(0x1c);                  /* trace_ring_wptr: total bytes written */
  bool     ovf    = (d->read32(0x26) >> 11) & 1u;     /* 0x26 bit11 = trace_overflow */
  uint32_t n = (wbytes < RINGSZ) ? wbytes : RINGSZ;
  const char* path = getenv("TRACE_DEEP_FILE"); if(!path) path = "/mnt/rttrace/deep.bin";
  FILE* fp = fopen(path, "wb");
  if(!fp){ fprintf(stderr,"### DEEP-TRACE: cannot open %s\n", path); return; }
  fwrite(c_addr + BASE, 1, n, fp);
  fclose(fp);
  fprintf(stderr,"### DEEP-TRACE dumped %u bytes (wptr=%u overflow=%d wrapped=%d) -> %s\n",
          n, wbytes, (int)ovf, (wbytes>RINGSZ)?1:0, path);
}
static void dump_trace(Driver *d) {
  uint32_t wptr = d->read32(0x19) & 0x1ff;
  printf("=== TRACE BUFFER: %u rows ===\n", wptr);
  for(uint32_t row = 0; row < wptr; row++) {
    uint32_t rec[12];
    for(uint32_t w = 0; w < 12; w++) {
      d->write32(0x17, (row << 4) | w);   // index = {row[7:0],word[3:0]}
      rec[w] = d->read32(0x18);
    }
    for(int s = 0; s < 2; s++) {
      int b = s*6;
      uint32_t fl = rec[b+5];
      if(!((fl >> 6) & 1)) continue;      // valid bit (bit6)
      printf("row %3u s%d: pc=%08x fetch=%u alloc=%u complete=%u retire=%u faulted=%u cause=%u\n",
             row, s, rec[b+0], rec[b+1], rec[b+2], rec[b+3], rec[b+4], (fl>>5)&1, fl&0x1f);
    }
  }
}

int main(int argc, char *argv[]) {
  bool initialize = true, sgi_mode = false, single_step = false, silent = false;
  int fd, steps = 0, us_amt = 1;
  uint32_t pc = 0x0, max_fetches = 0;
  uint64_t max_iters;
  void *vaddr = nullptr;
  std::string chpt_name;
  std::string arcs_image;
  std::string start_pc;
  std::string cimg_name, disk_name;
  rvstatus rs(0);
  /* --- SCSI disk service (PS side) --- */

  if(not(cmdline(argc, argv, initialize, silent, chpt_name, max_fetches, max_iters, sgi_mode, single_step, arcs_image, start_pc, cimg_name, disk_name))) {
    return -1;
  }

  /* Ctrl-C / SIGINT -> graceful stop: break the run loop so the post-run
   * diagnostic dump (last PC, core/L2/L1/AXI states, cycles-since-retire) prints
   * -- lets us snapshot a wedged/stuck boot instead of killing it blind. */
  signal(SIGINT, sigintHandler);
  signal(SIGUSR1, sigusr1_handler);

  /* ---- board access lock ---------------------------------------------------
   * Only one program may drive the AXI core + the shared-DRAM mmap at a time.
   * Two concurrent users corrupt each other's control-register sequencing and
   * wedge the board -- the PS gets CPU-pegged so even ssh stops responding.
   * Take an exclusive, non-blocking flock and bail out clearly if the board is
   * already in use.  A wrapper that already holds the lock (boot_irix.sh) sets
   * MIPS_AXI_LOCK_HELD so we don't self-deadlock against it.  The fd is held for
   * the process lifetime and the kernel auto-releases it on exit (clean or crash),
   * so a killed/crashed run never leaves a stale lock. */
  if(getenv("MIPS_AXI_LOCK_HELD") == nullptr) {
    int lockfd = open("/tmp/mips-axi.lock", O_CREAT | O_RDWR, 0666);
    if(lockfd < 0 || flock(lockfd, LOCK_EX | LOCK_NB) != 0) {
      fprintf(stderr, "mips-axi: BOARD BUSY -- another instance holds the board lock "
                      "(/tmp/mips-axi.lock); aborting.\n");
      return -1;
    }
  }

  initCapstone();
  fd = open("/dev/rv64core_fpga", O_RDWR | O_SYNC);
  assert(fd != -1);  
  if (ioctl(fd, 0, &phys_addr) < 0) {
    printf("error with fpga memory ioctl\n");
    close(fd);
    exit(-1);
  }
  printf("fpga memory starts at %lx\n", phys_addr);
  close(fd);

  d = new Driver(control);

  /* check mips */
  assert(d->read32(8) == 0x7370696d);
  
  fd = open("/dev/mem", O_RDWR | O_SYNC);
  assert(fd != -1);

  printf("opened phys memory\n");
  
  vaddr = mmap(0,
	       memsize,
	       PROT_READ|PROT_WRITE,
	       MAP_SHARED,
	       fd,
	       phys_addr);
  assert(vaddr != MAP_FAILED);
  c_addr = reinterpret_cast<uint8_t*>(vaddr);

  printf("mmap'd phys memory\n");
  memset(vaddr, 0, memsize);

  d->write32(2, max_fetches);

  if(not(cimg_name.empty())) {
    // checkpoint resume: memory from the .cimg, state from the --arcs 'preamble' blob.
    load_cimg(cimg_name.c_str(), c_addr);
    sgi_mode = true;        // need the IP22 MC (System Memory Alias + device map)
    pc = 0xbfc00000;        // reset vector -> the preamble installs regs/CP0/TLB, ERETs
  } else {
    pc = loadelf(chpt_name.c_str(), c_addr, sgi_mode);
  }
  printf("starting pc %x\n", pc);
  uint32_t *cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);

  /* arcs_image is set from --arcs (default empty = no firmware loaded;
   * use arcs_fw.bin for Linux, arcs_irix.bin for IRIX) */
  if(not(arcs_image.empty())) {
    struct stat ast;
    int afd = open(arcs_image.c_str(), O_RDONLY);
    if(afd < 0) {
      std::cerr << "could not open arcs image " << arcs_image << "\n";
      exit(-1);
    }
    if(fstat(afd, &ast) < 0) {
        std::cerr << "fstat failed on arcs image " << arcs_image << "\n";
        exit(-1);
    }
    char *abuf = (char*)mmap(nullptr, ast.st_size, PROT_READ, MAP_PRIVATE, afd, 0);
      /* The ARCS firmware is now an IP22-faithful first-stage boot loader (FSBL)
       * living in the Boot PROM @ kseg1 0xBFC00000 (phys 0x1fc00000), which the
       * sgi_mode address map shadows into DRAM @ 0x10C00000.  The FSBL copies the
       * SPB to 0xA0001000 itself at reset; we just drop the blob in the PROM. */
    memcpy(c_addr  + 0x10C00000ULL, abuf, ast.st_size);
    munmap(abuf, ast.st_size);
    close(afd);
    std::cout << "loaded ARCS FSBL (" << ast.st_size
	      << " bytes) at phys 0x1fc00000 (dram 0x10C00000)\n";
    /* Patch the FSBL kernel-entry slot @ phys 0x1fc00008 (-> dram 0x10C00008)
     * with the loaded kernel's ELF e_entry, big-endian (the MIPS core reads it
     * with lw).  'pc' still holds loadelf's e_entry here, before the --start-pc
     * override below.  Without this the FSBL jumps to a baked-in default that
     * goes stale whenever a kernel rebuild shifts the entry. */
    c_addr[0x10C00008ULL] = (uint8_t)(pc >> 24);
    c_addr[0x10C00009ULL] = (uint8_t)(pc >> 16);
    c_addr[0x10C0000AULL] = (uint8_t)(pc >>  8);
    c_addr[0x10C0000BULL] = (uint8_t)(pc      );
    std::cout << "patched FSBL kentry slot @0x1fc00008 (dram 0x10C00008) = 0x" << std::hex << pc << std::dec << "\n";
    
  }
  //for(int i = 0; i < 32; i++) {
  //std::cout << std::hex << pc+4*i << " : " << std::dec;
  //disassemble(std::cout, bswap<false>(*cptr), pc + 4*i); std::cout << "\n";
  //++cptr;
  //}  

   __builtin___clear_cache((char*)vaddr, ((char*)vaddr) + memsize);
  
  d->write32(6, phys_addr);

  printf("set phys addr\n");
  d->write32(8, memsize-1);

  printf("set mem size\n");  

  d->write32(CONTROL_REG, 0);
  d->write32(4, 1);
  d->write32(4, 0);
  if(sgi_mode) {
    d->write32(0xc,1);
    pc = 0xbfc00000;
  }
  /* --start-pc overrides the ELF entry / sgi default (e.g. arcs_boot 0xa0003000) */
  if(not(start_pc.empty())) pc = (uint32_t)strtoul(start_pc.c_str(), nullptr, 0);
  printf("start pc (final) = %x\n", pc);
  d->write32(PC_REG, pc);
  
  while(true) {
    __sync_synchronize();
    rs.u = d->read32(0xa);
    if(rs.s.ready) {
      printf("ready!, state %u\n", rs.s.state);
      break;
    }
  }


  
  g_l2nc = getenv("L2_NOCACHE") ? (1u<<20) : 0u;
  g_deeptrace = getenv("TRACE_DEEP") ? (1u<<21) : 0u;
  if(g_deeptrace) printf("### TRACE_DEEP=1: DRAM control-flow deep trace armed (ctrl bit21, ring @0x18000000)\n"), std::fflush(nullptr);
  printf("### RTL build-rev = 0x%08x (expect 0x20260727 = deep-trace)\n", d->read32(0x3f)), std::fflush(nullptr);
  if(g_l2nc) printf("### L2_NOCACHE=1: L2 behaves as no-cache (ctrl bit20 set before go)\n"), std::fflush(nullptr);
  uint32_t cr = (single_step ? (8 | 2 | STEP) : (8 | 2)) | g_l2nc | g_deeptrace;
  d->write32(4, cr);


  
  uint64_t zz= 0 , total_us = 0;
  uint64_t c = 0;
  volatile uint32_t *halt_flag = (volatile uint32_t*)(c_addr + 0x10D00000ULL); /* sgi: 0xBFD00000 -> c_addr[0x10D00000] */
  *halt_flag = 0;
  uint32_t magic_flag = 0, core_halt_u = 0; bool core_halted = false;
  
  
  g_scsi_disk.open_image(disk_name.c_str(), O_RDWR);
  
  { const char* e = getenv("SELDELAY"); int sd = e ? atoi(e) : 65535;
    d->write32(SCSI_W_SELDELAY, (uint32_t)sd);
    printf("[rtl] SELDELAY set to %d\n", sd); }
  printf("[rtl] revision = %08x (expect 20260721)\n", d->read32(SCSI_R_RTLREV));

  /* --- ENET tap service (PS side); host end is tap0 = 192.168.7.1/24 --- */

  { const char* et = getenv("ENETTAP"); g_enet_tap.open_tap(et ? et : "tap0"); }   /* absent tap => ENET idle (poll no-ops) */

  { const char* mp = getenv("MONPORT"); mon_init(d, mp ? atoi(mp) : 2323); }

  /* --- PS<->PL ping-pong producer (experiment): write DATA then SEQ (1MB apart,
   *     different DRAM pages); the MIPS consumer (pingpong.elf) reads-once-after-SEQ
   *     and checks DATA==SEQ.  c_addr is O_SYNC (uncached). --- */
  xpath_mode = (getenv("XPATH") != nullptr);
  const bool pingpong_mode = (getenv("PINGPONG") != nullptr);
  const bool xpath2_mode   = (getenv("XPATH2") != nullptr);   /* completion direction: ARM produces, signals via SCC Rx */
  volatile uint32_t *pp_data = (volatile uint32_t*)(c_addr + 0x09000000);
  volatile uint32_t *pp_seq  = (volatile uint32_t*)(c_addr + 0x09100000);
  volatile uint32_t *xc_ring = (volatile uint32_t*)(c_addr + 0x09000000);
  uint32_t pp_n = 0, xc_k = 0;
  if(pingpong_mode) { *pp_data = 0; *pp_seq = 0; __sync_synchronize(); printf("[pp] ARM producer armed\n"); }
  if(xpath2_mode)   { printf("[xc] ARM producer armed (completion direction)\n"); }

  FILE* g_trace=nullptr; int g_cdb_count=0; unsigned long g_nlog=0;
  FILE* g_rt_file=nullptr; unsigned long g_rt_fills=0, g_rt_entries=0, g_rt_rearm=0;
  while(c < max_iters && !done) {
    mon_poll();
    if(g_deeptrace && (c & 0x3ffff)==0){
      uint32_t wb=d->read32(0x1c); uint32_t cr4=d->read32(0x04); bool ov=(d->read32(0x26)>>11)&1;
      fprintf(stderr,"### deep-trace: wptr=%u (%u recs) ovf=%d ctrl=0x%08x bit21=%d\n", wb, wb/8, (int)ov, cr4, (cr4>>21)&1), fflush(stderr);
    }

    if(getenv("PAUSE_TEST") && g_armed==7){
      static int g_pev=0, g_pdur=0, g_paused=0; static time_t g_next_pause=0, g_pause_end=0;
      if(!g_pev){ const char*e=getenv("PAUSE_EVERY"); g_pev=e?atoi(e):10; const char*u=getenv("PAUSE_DUR"); g_pdur=u?atoi(u):2; }
      time_t now=time(0); uint32_t acr=(8u|2u)|(1u<<17)|g_l2nc;
      if(!g_next_pause) g_next_pause=now+g_pev;
      if(!g_paused && now>=g_next_pause){
        d->write32(4, acr|STEP);   /* HALT (single_step) -- do NOT sleep; loop keeps servicing */
        g_paused=1; g_pause_end=now+g_pdur;
        fprintf(stderr,"### HALT %ds (non-blocking; ARM keeps servicing)\n", g_pdur); fflush(stderr);
      } else if(g_paused && now>=g_pause_end){
        d->write32(4, acr);        /* RESUME */
        g_paused=0; g_next_pause=now+g_pev;
        fprintf(stderr,"### RESUME\n"); fflush(stderr);
      }
    }
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMFATAL", F_OK)==0){
      g_armed = 1;
      d->write32(4, (8u|2u) | (1u<<17));   /* arm RTL fault-trap: bp_enable = ctrl bit17 */
      fprintf(stderr, "### ARMFATAL armed (RTL fault-trap via bp_enable)\n");
    }
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMWP", F_OK)==0){
      g_armed = 2;   /* watchpoint mode */
      d->write32(10, 0x1006e984u);          /* bp_wp_addr = head slot s1+0x14 */
      d->write32(4, (8u|2u) | (1u<<17));     /* arm bp_enable (watchpoint + fault-trap) */
      fprintf(stderr, "### ARMWP armed: watch store EA==0x1006e984\n");
    }
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMWPV", F_OK)==0){
      g_armed = 3;   /* value+addr watchpoint mode */
      d->write32(10, 0x1006e984u);          /* bp_wp_addr = head slot s1+0x14 */
      d->write32(11, 0x00000007u);          /* bp_wp_val = 7 (the corrupt value) */
      d->write32(4, (8u|2u) | (1u<<17));     /* arm bp_enable */
      fprintf(stderr, "### ARMWPV armed: freeze on store 7 -> 0x1006e984\n");
    }
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMSC", F_OK)==0){
      g_armed = 4;
      d->write32(9, 0x0fa59f1cu);            /* bp_pc = libc lock-free SC */
      d->write32(4, (8u|2u) | (1u<<17));      /* arm bp_enable */
      fprintf(stderr, "### ARMSC armed: break at malloc SC 0x0fa59f1c\n");
    }
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMHDR", F_OK)==0){
      g_armed = 5;
      d->write32(10, 0x100400c4u);           /* bp_wp_addr = mem[s5+4] = CAPACITY slot */
      d->write32(11, 0xffffffffu);           /* wildcard: any store value */
      d->write32(4, (8u|2u) | (1u<<17));       /* arm */
      fprintf(stderr, "### ARMHDR armed: watch ANY store -> 0x1006e600 (array chunk header)\n");
    }
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMCAP", F_OK)==0){
      g_armed = 6;
      d->write32(9, 0x0e774158u);            /* bp_pc = array-insert grow-test (idx=gpr6, cap=gpr8 live) */
      d->write32(4, (8u|2u) | (1u<<17));      /* arm bp_enable */
      fprintf(stderr, "### ARMCAP armed: break @0e774158, watch idx(gpr6) vs cap(gpr8)\n");
    }
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMRT", F_OK)==0){
      g_armed = 7;
      const char* rtp = getenv("RT_TRACE_FILE"); if(!rtp) rtp = "/mnt/rttrace/trace.bin";
      g_rt_file = fopen(rtp, "wb");
      const char* bpe = getenv("RT_BP_PC"); uint32_t bppc = bpe ? (uint32_t)strtoul(bpe,0,0) : 0x0fafd878u;
      /* PC+THRESHOLD store watchpoint (be small-int-in-pointer-field BIRTH catch): with the new RTL,
       * bp_wp_addr(reg10)=the STORE'S PC to watch, and the WP freezes when that store's DATA < bp_wp_val(reg11).
       * RT_WP_PC unset -> reg10=0xffffffff (no store pc matches => WP off). */
      const char* wpe = getenv("RT_WP_PC");     uint32_t wppc = wpe ? (uint32_t)strtoul(wpe,0,0) : 0xffffffffu;
      const char* wte = getenv("RT_WP_THRESH"); uint32_t wpth = wte ? (uint32_t)strtoul(wte,0,0) : 0x01000000u;
      d->write32(9, bppc);                    /* bp_pc = retire-match freeze fallback (RT_BP_PC; 0 disables) */
      d->write32(10, wppc);                   /* bp_wp_addr = store-WP PC (RT_WP_PC); 0xffffffff => WP off */
      d->write32(11, wpth);                   /* bp_wp_val = small-data threshold: freeze if store DATA < this */
      d->write32(4, (8u|2u) | (1u<<17) | g_l2nc | g_deeptrace);   /* arm bp_enable (+L2_NOCACHE +deep-trace) -> flight recorder */
      fprintf(stderr, "### ARMRT armed: bp_pc=%08x wp_pc=%08x thresh=%08x (l2nc=%u) -> %s (%s)\n", bppc, wppc, wpth, g_l2nc?1:0, rtp, g_rt_file?"open":"OPEN-FAIL");
    }
    /* ARMFAULT: fault-only freeze at bp_pc (ctrl bit19 = bp_fault_only).  Deterministic -- the
     * pipe + ring freeze ONLY when bp_pc FAULTS, so a HOT bp_pc (0e788bc0, usually a clean load)
     * doesn't churn on every normal pass.  No wild-deref classifier / re-arm; freeze == the crash. */
    if(!g_armed && (c & 0xFFFF)==0 && access("/tmp/ARMFAULT", F_OK)==0){
      g_armed = 8;
      const char* rtp = getenv("RT_TRACE_FILE"); if(!rtp) rtp = "/mnt/rttrace/trace.bin";
      g_rt_file = fopen(rtp, "wb");
      const char* bpe = getenv("RT_BP_PC"); uint32_t bppc = bpe ? (uint32_t)strtoul(bpe,0,0) : 0x0e788bc0u;
      d->write32(9, bppc);                                  /* bp_pc = the faulting load */
      d->write32(10, 0xffffffffu);                          /* bp_wp_addr = never-matched: DISABLE the store
                                                             * watchpoint (else reset-0 bp_wp_addr freezes on
                                                             * a store of 0 to addr 0 during boot -> r_wp_hit) */
      d->write32(11, 0x00000001u);                          /* bp_wp_val != wildcard, != 0 */
      d->write32(4, (8u|2u) | (1u<<17) | (1u<<19));         /* arm bp_enable + bp_fault_only */
      fprintf(stderr, "### ARMFAULT armed: fault-only freeze bp_pc=%08x (wp disabled) -> %s (%s)\n", bppc, rtp, g_rt_file?"open":"OPEN-FAIL");
    }
    if(g_armed==7 && g_rt_file){
      const uint32_t CR=(8u|2u)|(1u<<17)|g_l2nc|g_deeptrace;
      d->write32(0x17, 0x800);                /* select retire-trace region (dbg_trace_index[11]=1) */
      uint32_t wp = d->read32(0x19);
      uint32_t st26 = d->read32(0x26);        /* global frozen bits: [8]=fault [9]=wp [10]=bp-match */
      static int g_dbgfrz=0;
      if(((st26>>8)&0x7u) && g_dbgfrz<4){ fprintf(stderr,"### DBGFRZ 0x19=%08x 0x26=%08x rt=%d flt=%d wp=%d bp=%d\n",wp,st26,(int)((wp>>8)&1),(int)((st26>>8)&1),(int)((st26>>9)&1),(int)((st26>>10)&1)); g_dbgfrz++; fflush(stderr); }
      if(((wp>>8)&1) || ((st26>>8)&0x7u)){                          /* r_rt_frozen -> ring froze on a fault OR a bp_pc match */
        /* fault context from the exc-ring (index[10]=1: w0=cause w1=epc w2=badvaddr w3=cycle w4=uop w5=fetched).
         * KEY: the exc-ring records ONLY t_arch_fault with cause 4/5/6/7/10 (w_exc_we in core.sv); a bp_pc
         * match does NOT write it.  A re-arm drops bp_enable which RESETS r_exc_wptr to 0.  Therefore, at a
         * freeze: ewp==0  => no recorded fault since the last re-arm => the freeze IS the bp_pc match (the
         * SIGSEGV-post @0x880f3284 = the real be crash).  ewp!=0 => a real fault entry (cause 4/5/6/7/10) was
         * written => read its cause.  In IRIX cause 4/5 (AdEL/AdES) are ROUTINE (the kernel EMULATES misaligned
         * accesses via the AdEL handler) and cause 10 (RI) is FP-emul -- re-arm past them; only cause 6/7
         * (a genuine bus error) is fatal. */
        /* WP-BIRTH: the PC+threshold store watchpoint froze on a store at the producer PC (RT_WP_PC,
         * e.g. 0f377124) whose DATA < threshold.  dbg_frozen(reg0x26 bits[10:8])={bp,wp,fault};
         * dbg_wp_data(reg0x27)=the store value (r21).  A small NONZERO value = a small int written where
         * a heap pointer belongs = THE CORRUPTION BIRTH -> dump (window rooted AT the bad store).  A NULL
         * (0) or large value at that pc is a legit store -> re-arm past it. */
        uint32_t frz3 = (d->read32(0x26) >> 8) & 0x7;
        bool wp_hit   = (frz3 >> 1) & 1u;
        uint32_t wpdata = wp_hit ? d->read32(0x27) : 0u;
        bool wp_birth = wp_hit && (wpdata != 0u) && (wpdata < 0x01000000u);
        d->write32(0x17, 0x400); uint32_t ewp = d->read32(0x19) & 0x1f;
        uint32_t ee = (ewp - 1) & 0xf; uint32_t ex[6];
        for(int k=0;k<6;k++){ d->write32(0x17, 0x400u|(ee<<3)|k); ex[k]=d->read32(0x18); }
        uint32_t cause=ex[0], epc=ex[1], badv=ex[2], fetched=ex[5];
        /* TWO triggers:
         * (A) WILD-BADVADDR cause-4 (the primary: roots the window AT the faulting deref).  A cause-4 (AdEL)
         *     froze the ring.  Routine unaligned-emulation targets the process's REAL data (badv ~0x200000,
         *     0x0c..0x12M heap/text, 0x7fffxxxx stack) -> re-arm.  A CRASH deref of a corrupt pointer hits a
         *     WILD addr: tiny (<64K, e.g. crash-6's 0x17) or kernel-range (>=0x80000000 from user).  Stop &
         *     dump -> the 16K window is rooted at the deref, holding the corrupt value's origin.  epc must be
         *     userspace (a real user fault, not a kernel-internal AdEL).  RT_WILD_MAX overrides the 0x10000.
         * (B) bp_pc = kernel exit() entry 0x8814a360: fallback for cause-2 (aligned unmapped) crashes that
         *     don't freeze the ring.  a0(r4)=sig (sigtramp 0x0fafd878); stop on crash signals (4/10/11), re-arm past normal exits. */
        bool bpmatch = (ewp==0u);
        bool bpdump = bpmatch && g_bp_dump;   /* off-golden trigger pc retired -> capture the 32K window ENDING here */
        uint32_t sig_arg = 0;
        if(bpmatch){ d->write32(14, 4); sig_arg = d->read32(0xe); }
        /* WILD = below the lowest valid mapping (0x200000) or kernel-range from user.  cause 4/5 (AdEL/AdES,
         * misaligned = SIGBUS) freeze the ring UNCONDITIONALLY -> filter them here by badv (catches SIGBUS-face
         * wild derefs of any magnitude below 0x200000).  cause 2/3 (aligned = SIGSEGV) only reach us when the
         * NEW bit already gated them to <64KB.  Nothing routine lives below 0x200000. */
        const char* wm = getenv("RT_WILD_MAX"); uint32_t wild_max = wm ? (uint32_t)strtoul(wm,0,0) : 0x200000u;
        bool wild_deref = (!bpmatch) && (cause==2u||cause==3u||cause==4u||cause==5u) && (epc < 0x80000000u) && (badv < wild_max || badv >= 0x80000000u);
        /* cause-10 RI = wild JUMP to garbage (SIGILL face).  The jump lands on a MAPPED data/heap
         * page (valid addr, garbage decode) -> EPC is the wild target.  be text ~0x0e, libc/rld
         * ~0x0f (routine FP-emul RIs live there -> re-arm); a wild jump lands in the data/heap/stack
         * (>=0x10000000) or below text -> stop.  (A jump to a TINY addr faults as cause-2 IFetch, caught above.) */
        bool wild_jump  = (!bpmatch) && (cause==10u) && (epc < 0x80000000u) && (epc >= 0x10000000u || epc < 0x0c000000u);
        bool bpkill     = bpmatch && (sig_arg==4u || sig_arg==10u || sig_arg==11u);
        bool real       = wild_deref || wild_jump || bpkill || (cause==6u) || (cause==7u) || wp_birth || bpdump;
        if(wp_hit && !wp_birth) real = false;   /* NULL/large store at the producer pc -> re-arm past it */
        if(!real){
          g_rt_rearm++;
          if(bpmatch)
            fprintf(stderr,"### re-arm #%lu past sigtramp sig=%u\n",(unsigned long)g_rt_rearm,sig_arg), fflush(stderr);
          else if((g_rt_rearm % 64)==0)
            fprintf(stderr,"### re-arm #%lu past routine fault cause=%u epc=%08x badv=%08x\n",
                    (unsigned long)g_rt_rearm, cause, epc, badv), fflush(stderr);
          d->write32(4, (8u|2u)|g_l2nc); d->write32(4, CR);   /* re-arm (drop+raise bp_enable, keep l2nc): reset+resume */
        } else {
          const char* why = bpdump ? "BP-DUMP @off-golden trigger pc" : wp_birth ? "WP-BIRTH (small int stored @ producer pc)" : (wild_deref ? "WILD-DEREF @fault" : (wild_jump ? "WILD-JUMP @RI (SIGILL face)" : (bpkill ? "SIGNAL-KILL @sigtramp" : "BUS ERROR")));
          if(wp_birth) fprintf(stderr,"### *** WP-BIRTH CAUGHT: producer store DATA=0x%08x (a small int in a pointer field) -- window rooted AT the bad store ***\n", wpdata);
          if(g_characterize){   /* lightweight: log the crash's VA/sp + re-arm+continue (many samples/boot) */
            d->write32(14, 29); uint32_t sp = d->read32(0xe);
            g_rt_fills++;
            fprintf(stderr,"### CHAR #%lu: %s cause=%u epc=%08x badv=%08x sp=%08x poison_pg=%08x fetched=%08x\n",
                    (unsigned long)g_rt_fills, why, cause, epc, badv, sp, sp & ~0xfffu, fetched);
            fflush(stderr);
            d->write32(4, (8u|2u)|g_l2nc); d->write32(4, CR);   /* re-arm + resume -> catch the next crash */
          } else {
          fprintf(stderr,"### FRZ #%lu (REAL, %s sig=%u): ewp=%u exc[cause=%u epc=%08x badvaddr=%08x fetched=%08x]  (%lu re-arms)\n",
                  (unsigned long)g_rt_fills, why, sig_arg, ewp, cause, epc, badv, fetched, (unsigned long)g_rt_rearm);
          fprintf(stderr,"### GPRs:"); for(int i=0;i<32;i++){ d->write32(14,i); fprintf(stderr," r%d=%08x",i,d->read32(0xe)); } fprintf(stderr,"\n");
          /* TLB SHADOW dump: 48 entries x 4 words via dbg_trace_index 0x200|(entry<<3)|word.
           * w0={r,vpn} w1={asid,pagemask} w2=EntryLo0 w3=EntryLo1.  Compared offline vs the
           * interp_mips checkpoint TLB (golden_tlb.py) to catch a mistranslation. */
          fprintf(stderr,"### TLB48:");
          for(int e=0;e<48;e++){
            for(int w4=0;w4<4;w4++){ d->write32(0x17, 0x200u|((uint32_t)e<<3)|(uint32_t)w4); fprintf(stderr," %08x", d->read32(0x18)); }
          }
          fprintf(stderr,"\n");
          fflush(stderr);
          /* STACK DRAM read: does memory actually HOLD the spilled small-int, or did the LOAD
           * fabricate it?  Translate sp(r29) via the TLB shadow -> PA, read a range of DRAM.
           * Offline: the crash's ld was `ld rX,off(r29)` -> check DRAM[sp+off] == the value. */
          {
            d->write32(14, 29); uint32_t sp = d->read32(0xe);
            uint32_t vpn2 = (sp >> 13) & 0x7ffffffu, odd = (sp >> 12) & 1u, pfn = 0xffffffffu;
            for(int e=0;e<48;e++){
              d->write32(0x17, 0x200u|((uint32_t)e<<3)|0u); uint32_t w0 = d->read32(0x18);
              if((w0 & 0x7ffffffu) != vpn2) continue;
              d->write32(0x17, 0x200u|((uint32_t)e<<3)|(odd?3u:2u)); uint32_t lo = d->read32(0x18);
              if(lo & 0x2u) pfn = (lo>>6)&0xffffffu;   /* v bit set */
              break;
            }
            if(pfn != 0xffffffffu){
              uint32_t pab = pfn*0x1000u + (sp & 0xfffu);
              fprintf(stderr,"### STACK DRAM sp=%08x -> pa=%08x:", sp, pab);
              for(uint32_t off=0; off<0x120u; off+=4){ fprintf(stderr," +%x=%08x", off, rdbe(pab+off)); }
              fprintf(stderr,"\n");
              /* CROSS-CHECK (complete history via per-page bitmap): was the poison's physical page
               * EVER a SCSI DMA (disk->DRAM) target? (page-reuse + stale-L1 hypothesis).  Check the
               * stack page +- 1 neighbor. */
              int hitpg = -2;
              for(int pg = -1; pg <= 1 && hitpg < -1; pg++){
                uint32_t ppage = (pab & ~0xfffu) + (uint32_t)(pg*0x1000);
                if(scsi_dma_page_seen(ppage)) hitpg = pg;
              }
              if(hitpg >= -1)
                fprintf(stderr,"### *** POISON-PAGE WAS SCSI-DMA'd (page %+d of stack pa=%08x) -- page-reuse+stale-cache CANDIDATE (of %u total DMA ranges) ***\n",
                        hitpg, pab, g_scsi_dma_n);
              else
                fprintf(stderr,"### poison page %08x + neighbors NEVER SCSI-DMA'd (of %u total ranges) -- DMA-reuse hypothesis RULED OUT for this crash\n",
                        pab & ~0xfffu, g_scsi_dma_n);
            } else fprintf(stderr,"### STACK: no valid TLB entry for sp=%08x vpn2=%x\n", sp, vpn2);
            fflush(stderr);
          }
          /* walk the 32768-entry pre-fault window (oldest..newest) via auto-increment:
           * read a row's 6 words, then pulse `step` to advance the internal read ptr. */
          for(int i=0;i<32768;i++){
            uint32_t w[6];
            for(int k=0;k<6;k++){ d->write32(0x17, 0x800u|k); w[k]=d->read32(0x18); }
            struct __attribute__((packed)) { uint64_t pc; uint32_t inst; uint8_t valid,reg; uint64_t val; } rec;
            rec.pc=((uint64_t)w[1]<<32)|w[0]; rec.inst=w[2];
            rec.valid=(uint8_t)((w[5]>>5)&1); rec.reg=(uint8_t)(w[5]&0x1f); rec.val=((uint64_t)w[4]<<32)|w[3];
            fwrite(&rec,sizeof(rec),1,g_rt_file); g_rt_entries++;
            d->write32(4, CR|(1u<<30)); d->write32(4, CR);   /* step: advance read ptr to next entry */
          }
          fflush(g_rt_file); g_rt_fills++;
          fprintf(stderr,"### REAL CRASH (%s) -- 16K window dumped (%lu entries), STOPPING.\n",why,(unsigned long)g_rt_entries);
          if(g_deeptrace) dump_dram_trace(d);
          done = true;
          }   /* end !g_characterize (full dump) */
        }
      }
    }
    if(g_armed==8){
      /* fault-only: r_bp_hit (status bit10) latches ONLY on the fault at bp_pc.  Freeze == crash. */
      const uint32_t CR = (8u|2u)|(1u<<17)|(1u<<19);
      uint32_t st = d->read32(0x26);
      uint32_t flt_frz = (st>>8)&1, wp_frz = (st>>9)&1, bp_frz = (st>>10)&1;
      static int g_reported = 0;
      if((flt_frz||wp_frz) && !bp_frz && !g_reported){
        g_reported = 1;
        fprintf(stderr,"### UNEXPECTED FREEZE (not bp): fault=%u wp=%u  cause=%u epc=%08x badv=%08x pc=%08x\n",
                flt_frz, wp_frz, st&31, d->read32(0xb), d->read32(0xc), d->read32(7));
        fflush(stderr);
      }
      if(bp_frz){
        uint32_t cz=st&31, epc=d->read32(0xb), badv=d->read32(0xc), lastpc=d->read32(7);
        fprintf(stderr,"### FAULT-FREEZE @bp_pc: cause=%u epc=%08x badvaddr=%08x lastpc=%08x\n", cz, epc, badv, lastpc);
        fprintf(stderr,"### GPRs:"); for(int i=0;i<32;i++){ d->write32(14,i); fprintf(stderr," r%d=%08x",i,d->read32(0xe)); } fprintf(stderr,"\n");
        /* TLB SHADOW: 48 entries x 4 words via dbg_trace_index 0x200|(entry<<3)|word.
         * w0={r,vpn} w1={asid,pagemask} w2=EntryLo0 w3=EntryLo1 -- diff offline vs golden_tlb.py. */
        fprintf(stderr,"### TLB48:");
        for(int e=0;e<48;e++){
          for(int w4=0;w4<4;w4++){ d->write32(0x17, 0x200u|((uint32_t)e<<3)|(uint32_t)w4); fprintf(stderr," %08x", d->read32(0x18)); }
        }
        fprintf(stderr,"\n"); fflush(stderr);
        /* dump the 32768-entry pre-fault window (oldest..newest) via step auto-increment */
        if(g_rt_file){
          for(int i=0;i<32768;i++){
            uint32_t w[6];
            for(int k=0;k<6;k++){ d->write32(0x17, 0x800u|k); w[k]=d->read32(0x18); }
            struct __attribute__((packed)) { uint64_t pc; uint32_t inst; uint8_t valid,reg; uint64_t val; } rec;
            rec.pc=((uint64_t)w[1]<<32)|w[0]; rec.inst=w[2];
            rec.valid=(uint8_t)((w[5]>>5)&1); rec.reg=(uint8_t)(w[5]&0x1f); rec.val=((uint64_t)w[4]<<32)|w[3];
            fwrite(&rec,sizeof(rec),1,g_rt_file); g_rt_entries++;
            d->write32(4, CR|(1u<<30)); d->write32(4, CR);
          }
          fflush(g_rt_file);
        }
        fprintf(stderr,"### REAL CRASH (fault-only @bp_pc) -- 16K window dumped (%lu entries), STOPPING.\n",(unsigned long)g_rt_entries);
        done = true;
      }
    }
    if(g_armed==5){
      const uint32_t CR = (8u|2u)|(1u<<17);
      static uint32_t hpc[16], g_fcv[16]; static uint32_t hn=0;
      uint32_t st = d->read32(0x26);
      uint32_t cz = st & 31, wp_frz = (st>>9)&1, flt_frz = (st>>8)&1;
      if(flt_frz || cz==4||cz==5||cz==6||cz==7||cz==10){
        fprintf(stderr, "### HDR-FAULT cause=%u EPC=%08x BadVAddr=%08x (%u writes to 0x1006e600)\n",
                cz, d->read32(0xb), d->read32(0xc), hn);
        uint32_t base=(hn>16)?(hn-16):0;
        for(uint32_t j=base;j<hn;j++) fprintf(stderr, "###   fc[%u] pc=%08x val=%08x\n", j, hpc[j&15], g_fcv[j&15]);
        fflush(stderr); done=true;
      } else if(wp_frz){
        uint32_t p1 = d->read32(7); uint32_t val = d->read32(0x27);
        hpc[hn&15]=p1; g_fcv[hn&15]=val; hn++;
        if(val==0x1006e608u){   /* the grow that overlaps -- full context (realloc size in a0/regs) */
          fprintf(stderr, "### BASE-GROW #%u frozen_pc=%08x val=%08x GPRs:", hn, p1, val);
          for(int i=0;i<32;i++){ d->write32(14,i); fprintf(stderr, " r%d=%08x", i, d->read32(0xe)); }
          fprintf(stderr, "\n"); fflush(stderr);
        } else fprintf(stderr, "### CAP-WRITE #%u frozen_pc=%08x cap=%u (0x%08x)\n", hn, p1, val, val), fflush(stderr);
        d->write32(4, CR|(1u<<18)); d->write32(4, CR);
      }
    }
    if(g_armed==6){
      const uint32_t CR = (8u|2u)|(1u<<17);
      static uint32_t g_cap_n=0, g_cap_oob=0, g_cap_min=0xffffffff, g_cap_max=0;
      uint32_t st = d->read32(0x26);
      uint32_t cz = st & 31, bp_frz = (st>>10)&1, flt_frz = (st>>8)&1;
      if(flt_frz || cz==4||cz==5||cz==6||cz==7||cz==10){
        fprintf(stderr, "### CAP-FAULT cause=%u EPC=%08x BadVAddr=%08x (%u inserts, %u OOB; cap range %u..%u)\n",
                cz, d->read32(0xb), d->read32(0xc), g_cap_n, g_cap_oob, g_cap_min, g_cap_max);
        fflush(stderr); done=true;
      } else if(bp_frz){
        d->write32(14,6); uint32_t idx=d->read32(0xe);
        d->write32(14,8); uint32_t cap=d->read32(0xe);
        g_cap_n++;
        if(cap<g_cap_min) g_cap_min=cap;
        if(cap>g_cap_max && cap<0x10000) g_cap_max=cap;
        if(idx>cap){ g_cap_oob++;
          fprintf(stderr, "### OOB #%u: idx=%u > cap=%u  (insert #%u)\n", g_cap_oob, idx, cap, g_cap_n); fflush(stderr); }
        else if((g_cap_n & 511)==0){ fprintf(stderr, "### insert #%u idx=%u cap=%u ok\n", g_cap_n, idx, cap); fflush(stderr); }
        d->write32(4, CR|(1u<<30)); d->write32(4, CR);   /* step over the bp insn */
        d->write32(4, CR|(1u<<18)); d->write32(4, CR);   /* resume to next insert */
      }
    }
    if(g_armed==4){
      const uint32_t CR = (8u|2u)|(1u<<17);
      uint32_t st = d->read32(0x26);
      uint32_t cz = st & 31, bp_frz = (st>>10)&1, flt_frz = (st>>8)&1;
      if(flt_frz || cz==4||cz==5||cz==6||cz==7||cz==10){
        fprintf(stderr, "### SC-FAULT cause=%u EPC=%08x BadVAddr=%08x (%u SCs seen)\n",
                cz, d->read32(0xb), d->read32(0xc), g_sc_n);
        uint32_t base=(g_sc_n>24)?(g_sc_n-24):0;
        for(uint32_t j=base;j<g_sc_n;j++)
          fprintf(stderr, "###   SC[%u] addr=%08x val=%08x result=%u\n", j, g_sc_addr[j&31], g_sc_val[j&31], g_sc_res[j&31]);
        fflush(stderr); done=true;
      } else if(bp_frz){
        d->write32(14,1);  uint32_t val  = d->read32(0xe);   /* at = value to store (pre-SC) */
        d->write32(14,3);  uint32_t addr = d->read32(0xe);   /* v1 = lock addr */
        d->write32(4, CR|(1u<<30)); d->write32(4, CR);       /* step: execute the SC */
        d->write32(14,1);  uint32_t res  = d->read32(0xe);   /* at = SC result (1 succ / 0 fail) */
        g_sc_val[g_sc_n&31]=val; g_sc_res[g_sc_n&31]=res; g_sc_addr[g_sc_n&31]=addr; g_sc_n++;
        if((g_sc_n & 1023)==0){ fprintf(stderr, "### SCs so far: %u (last val=%08x res=%u)\n", g_sc_n, val, res); fflush(stderr); }
        d->write32(4, CR|(1u<<18)); d->write32(4, CR);       /* resume to next SC */
      }
    }
    if(g_armed==3){
      const uint32_t CR = (8u|2u)|(1u<<17);
      uint32_t st = d->read32(0x26);
      uint32_t cz = st & 31, wp_frz = (st>>9)&1, flt_frz = (st>>8)&1;
      if(flt_frz || cz==4||cz==5||cz==6||cz==7||cz==10){
        /* the be fault -- dump the writer ring (last 16 real 7-writes). */
        uint32_t p1 = d->read32(7);
        fprintf(stderr, "### WPV-FAULT cause=%u EPC=%08x BadVAddr=%08x pc=%08x (%u real 7-writes to slot)\n",
                cz, d->read32(0xb), d->read32(0xc), p1, g_wp_n);
        uint32_t base=(g_wp_n>16)?(g_wp_n-16):0;
        for(uint32_t j=base;j<g_wp_n;j++) fprintf(stderr, "###   7-write[%u] frozen_pc=%08x\n", j, g_wp_pc[j&15]);
        fflush(stderr); done=true;
      } else if(wp_frz){
        /* REAL 7-write freeze (r_wp_hit set) -- no stall false-positive.  Compact ring-log
         * the frozen (retire-trailing) PC + a couple GPRs; the last one before the fault
         * names the writer region.  (Follow-up run full-dumps a specific one if needed.) */
        uint32_t p1 = d->read32(7);
        g_wp_pc[g_wp_n & 15] = p1; g_wp_n++;
        fprintf(stderr, "### WPV-HIT #%u frozen_pc=%08x GPRs:", g_wp_n, p1);
        for(int i=0;i<32;i++){ d->write32(14,i); fprintf(stderr, " r%d=%08x", i, d->read32(0xe)); }
        fprintf(stderr, "\n"); fflush(stderr);
        d->write32(4, CR|(1u<<18)); d->write32(4, CR);   /* resume to next real 7-write */
      }
    }
    if(g_armed==2){
      const uint32_t CR = (8u|2u)|(1u<<17);
      uint32_t p1 = d->read32(7), p2 = d->read32(7);
      if(p1==p2){            /* frozen (running core would retire thousands between reads) */
        uint32_t cz = d->read32(0x26) & 31;
        if(cz==4||cz==5||cz==6||cz==7||cz==10){
          fprintf(stderr, "### WP-FAULT cause=%u EPC=%08x BadVAddr=%08x pc=%08x  (%u wp fires)\n",
                  cz, d->read32(0xb), d->read32(0xc), p1, g_wp_n);
          uint32_t base = (g_wp_n>16)?(g_wp_n-16):0;
          for(uint32_t j=base;j<g_wp_n;j++)
            fprintf(stderr, "###   wp[%u] frozen_pc=%08x\n", j, g_wp_pc[j&15]);
          fflush(stderr); done = true;
        } else {
          g_wp_pc[g_wp_n & 15] = p1; g_wp_n++;
          d->write32(14,16); uint32_t r16 = d->read32(0xe);   /* s0 = head-store data */
          d->write32(14,8);  uint32_t r8  = d->read32(0xe);   /* t0 = count-store data */
          int head_bad  = (r16!=0 && r16 < 0x1000u);
          int cnt_store = (p1 >= 0x0e6818b8u && p1 <= 0x0e6818dcu);
          if(head_bad || cnt_store){
            fprintf(stderr, "### WP-ANOM fire#%u frozen_pc=%08x s0(r16)=%08x t0(r8)=%08x %s%s\n",
                    g_wp_n, p1, r16, r8, head_bad?"[HEAD-VAL-BAD]":"", cnt_store?"[COUNT-STORE-MISADDR]":"");
            fflush(stderr);
          }
          if((g_wp_n & 4095)==0){ fprintf(stderr, "### wp fires so far: %u (last pc=%08x s0=%08x)\n", g_wp_n, p1, r16); fflush(stderr); }
          d->write32(4, CR|(1u<<18)); d->write32(4, CR);   /* resume: clear wp_hit, run to next */
        }
      }
    }
    if(g_armed){
      static int n_trap = 0; const int STEP_N = 8;
      uint32_t cz = d->read32(0x26) & 31;
      /* RTL froze the core at a fatal userspace fault -> {epc,cause,badvaddr,GPRs} STABLE */
      if(cz==4||cz==5||cz==6||cz==7||cz==10){
        uint32_t epc = d->read32(0xb), bv = d->read32(0xc), pc = d->read32(7);
        fprintf(stderr, "### BEFAULT[%d] cause=%u EPC=%08x BadVAddr=%08x pc=%08x\n", n_trap, cz, epc, bv, pc);
        fprintf(stderr, "### GPRs[%d]:", n_trap);
        for(int i=0;i<32;i++){ d->write32(14,i); fprintf(stderr, " r%d=%08x", i, d->read32(0xe)); }
        fprintf(stderr, "\n"); fflush(stderr);
        fprintf(stderr, "### MEM node@1003f820(PA9473820)= %08x %08x %08x %08x | head[s1+20]@PA92f596c= %08x\n",
                rdbe(0x9473820), rdbe(0x9473824), rdbe(0x9473828), rdbe(0x947382c), rdbe(0x92f596c));
        fflush(stderr);
        n_trap++;
        if(n_trap >= STEP_N){ done = true; }
        else {
          /* step to the next fault: pulse fault_clear (bit18), keep armed (bit17) */
          d->write32(4, (8u|2u) | (1u<<17) | (1u<<18));
          d->write32(4, (8u|2u) | (1u<<17));
        }
      }
    }
    if(g_dumpreq){
      d->write32(4, 8|2|(1u<<31));            /* halt: single-step mode = freeze */
      fprintf(stderr, "\n### HALT-DUMP: cause=%u pc=%08x sr=%08x\n",
              d->read32(0x26)&31, d->read32(7), d->read32(0x16));
      dump_registers(d);
      dump_trace(d);
      unsigned long long base = 0x08000000ULL, len = 0x04000000ULL; /* PA 0x08M..0x0CM (64MB) */
      FILE *df = fopen("/tmp/dram_dump.img", "wb");
      if(df){ size_t w = fwrite(c_addr + base, 1, len, df); fclose(df);
              fprintf(stderr, "[dump] wrote %zu bytes (PA base 0x%llx) to /tmp/dram_dump.img (core halted)\n", w, base); }
      g_dumpreq = 0;
    }
#ifdef CLAUDE_DEBUG
    if(g_trace){
      d->write32(4, cr | (1u<<30));   /* rising edge -> retire ~1 */
      d->write32(4, cr);              /* back to 0, ready for next edge */
      uint32_t lpc=d->read32(7);
      fwrite(&lpc,4,1,g_trace);
      static uint32_t plpc=0; long dd=(long)lpc-(long)plpc; plpc=lpc;
      if((++g_nlog % 100000)==0){ fprintf(stderr,"[trace] %lu pcs pc=%08x d=%ld\\n",(unsigned long)g_nlog,lpc,dd); fflush(stderr); }
      if((g_nlog & 0x7f)==0) scsi_arm_poll(d,&g_scsi_disk,c_addr);   /* keep the probe fed */
      if(lpc>=0x88007790u && lpc<=0x880077c0u){ fprintf(stderr,"[trace] PARK after %lu\\n",(unsigned long)g_nlog); fflush(g_trace); fclose(g_trace); g_trace=nullptr; done=true; }
      else if(g_nlog>4000000UL){ fprintf(stderr,"[trace] cap 4M\\n"); fflush(g_trace); fclose(g_trace); g_trace=nullptr; done=true; }
      c++; continue;
    }

    
    if(pingpong_mode) {
      pp_n++;
      *pp_data = pp_n;          /* DATA first */
      __sync_synchronize();     /* DSB: data write ordered before the seq write */
      *pp_seq = pp_n;           /* then SEQ (the signal), a DRAM page away */
    }
#endif
    
    if(xpath2_mode && !scc_rx_full()) {     /* completion direction: DATA->DDR, then SIGNAL via SCC Rx (S00 leg) */
      xc_ring[xc_k & 0xFFFFu] = __builtin_bswap32(xc_k);  /* MIPS reads big-endian -> k */
      __sync_synchronize();                              /* DSB: DATA write before the signal */
      d->write32(0x3b, (1u << 8) | (xc_k & 0xff));        /* push SCC Rx byte (the signal) */
      d->write32(0x3b, 0);
      xc_k++;
    }
    uint32_t s = cr | 1U<<30;
    if(single_step) {
      d->write32(4, s);
    }
    if((zz&POLL_FREQ) == 0) {
      total_us += us_amt;
      pump_stdin_to_rx();
      
      const bool scsi_serviced = scsi_arm_poll(d, &g_scsi_disk, c_addr);
      const bool enet_serviced = enet_arm_poll(d, &g_enet_tap, c_addr);   /* bridge the ENET mailbox <-> tap0 */
#ifdef CLAUDE_DEBUG
      if(scsi_serviced){
        static const bool g_pctrace = getenv("PCTRACE") != nullptr;
        if(g_pctrace && ++g_cdb_count==1 && !g_trace){
	  g_trace=fopen("/tmp/pctrace.bin","wb");
	  g_nlog=0;
	  cr=(8|2|STEP);
	  d->write32(4,cr);
	  fprintf(stderr,"[trace] CDB#1 -> tight single-step to park\n");
	  fflush(stderr);
	}
      }

      {
	static uint32_t last_dbg = 0xffffffffu;
	uint32_t dbg = d->read32(SCSI_R_DBG);
        if(dbg != last_dbg) {
	  last_dbg = dbg;     /* print-on-change: low spam during the boot */
          static const bool g_scsidbg = getenv("SCSIDBG") != nullptr;
          if(g_scsidbg)printf("[scsidbg] 0x38=%08x #rst=%u #rd=%u #scmdwr=%u #sasrwr=%u ph=%u CIP=%u BSY=%u INTRQ=%u SASR=%02x\n",
				      dbg, (dbg>>28)&0xf, (dbg>>22)&0x3f, (dbg>>16)&0x3f, (dbg>>10)&0x3f,
				      (dbg>>8)&3, (dbg>>7)&1, (dbg>>6)&1, (dbg>>5)&1, dbg&0x1f); }
      }
#endif
      bool new_c = read_char_fifo();
      if(*halt_flag != 0) {
	magic_flag = *halt_flag;
	done = true;
      }
      rs.u = d->read32(0xa);
      if(cpu_stopped(rs)) {
	core_halt_u = rs.u;
	core_halted = true;
	done = true;
      }
      /* debug PC-breakpoint (RTL freezes core after retiring BP_PC=0x880023c0):
       * detect the freeze (last pc stuck at BP_PC, insn cnt stable) and exit so
       * the post-run dump snapshots the frozen (un-reset) register state. */
#ifdef CLAUDE_DEBUG
      {
	static uint32_t bp_ic=0xffffffffu;
	static int bp_st=0;
        uint32_t ic=d->read32(0), lpc=d->read32(7);
        if(ic==bp_ic){
	  if(++bp_st>60){
	    printf("[WP] core FROZEN: last pc=%08x insn=%u\n", lpc, ic);
	    done=true;
	  }
	}
        else {
	  bp_st=0;
	}
	bp_ic=ic;
      }
      {
	static time_t _lt=0;
	time_t _nw=time(0);
	if(_nw!=_lt) {
	  _lt=_nw;
	  printf("[stat] insn=%u pc=%08x\n", d->read32(0), d->read32(7)); fflush(stdout);
	  }
      }
#endif
      if(not(new_c) && not(scsi_serviced) && not(enet_serviced)) {
	usleep(us_amt);
	us_amt = std::min(us_amt+1, 1000);
	///printf("last pc = %x, insn cnt %u\n", d->read32(7), d->read32(0));
      }
      else {
	us_amt = 1;
      }
    }
    c++;
    if(single_step) {
      dump_registers(d);
      d->write32(4, cr);
    }
  }

  /* the magic-halt flag (DRAM) can become visible before the last putchars
   * reach the console FIFO; settle, then drain to N consecutive empties so
   * the full checksum is captured (not a truncated prefix). */
  usleep(5000);
  for(int e=0; e<200; ) { if(read_char_fifo()) e=0; else { e++; usleep(50); } }
  printf("\n");
  if(magic_flag) printf("MAGIC HALT: flag=0x%x\n", magic_flag);
  if(xpath_mode) printf("XPATH RESULT ok=%u bad=%u (signals=%u)\n", x_ok, x_bad, xk);
  if(core_halted) { rvstatus hr(core_halt_u); printf("CORE HALTED: break=%u ud=%u bad_addr=%u monitor=%u\n", hr.s.break_, hr.s.ud, hr.s.bad_addr, hr.s.monitor); }


  if(not(silent)) {
    printf("last pc = %x, insn cnt %u\n", d->read32(7), d->read32(0));
    dump_trace(d);
    printf("axi reads  %d\n", d->read32(18));
    printf("axi writes %d\n", d->read32(20));  

    pc = d->read32(7);
    cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);



    printf("cycles since last retired %u\n", d->read32(0x27));
    uint32_t epc = d->read32(0xb);
    printf("[CP0] EPC=%08x cause=%08x(ExcCode=%u) badvaddr=%08x\n", epc, d->read32(0x26), d->read32(0x26)&31, d->read32(0xc));
    printf("%d register writes\n", d->read32(0x16));
    rs.u = d->read32(0xa);
    printf("bad addr %u\n", rs.s.bad_addr);
    printf("monitor %u\n", rs.s.monitor);
    printf("ud %u\n", rs.s.ud);
    printf("break %u\n", rs.s.break_);  
    printf("epc %x\n", epc);
    printf("badvaddr %x\n", d->read32(0xc));  
    printf("cause %u\n", (d->read32(0x26)&31));  
    printf("last addr %x\n", d->read32(0x9));
  
    uint32_t states = d->read32(0xd);    
    std::cout << "core state   = " << (states & 31) << "\n";
    std::cout << "l2 state     = " << ((states>>5) & 15) << "\n";
    std::cout << "l1i state    = " << ((states>>9) & 7) << "\n";
    std::cout << "l1d state    = " << ((states>>12) & 15) << "\n";
    std::cout << "axi state    = " << ((states>>16) & 15) << "\n";
    std::cout << "inflight     = " << ((states>>20) & 63) << "\n";
    std::cout << "l2 rsp state = " << ((states>>26) & 15) << "\n";          

    for(int i = 0; i < 32; i++) {
      d->write32(14, i);
      printf("reg %s : %x\n", getGPRName(i).c_str(),
	     d->read32(0xe));
    }

    if(0) {
      uint32_t n = d->read32(0);
      printf("%u instructions retired\n", d->read32(0));
      for(uint32_t i = 0; i <= n; i++) {
	d->write32(0x16, i);
	printf("%u : %x\n", i, d->read32(0x17));
      }
    }
  }
 
  munmap(c_addr, memsize);
  stopCapstone();
  return 0;
}
