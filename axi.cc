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
#include <fcntl.h>

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <capstone/capstone.h>

#include "helper.hh"
#include "driver.hh"
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


static const uint64_t memsize = 496*MB;
static uint64_t phys_addr = ~0UL;

static uint8_t *c_addr = nullptr;
static bool done = false;

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
  int c = d->read32(0x3b);
  int cc = (c==0 ? '\n' : c);
  printf("%c", c==0 ? '\n' : c);
  std::fflush(nullptr);
  d->write32(0x3a, 1);
  d->write32(0x3a, 0);
  return true;
}

void sigintHandler(int id) {
  done = true;
}

uint32_t loadelf(const char* fn, uint8_t *mem, bool sgi_mode);

bool cmdline(int argc,
	     char *argv[],
	     bool &initialize,
	     std::string &chpt_name,
	     uint32_t &max_fetches,
	     uint64_t &max_iters,
	     bool &sgi_mode,
	     bool &single_step);

static void dump_registers(Driver *d) {
  printf("pc=%x cause=%u, sr %x |", d->read32(7), d->read32(0x26)&31, d->read32(0x16));
  for(int i=0;i<32;i++){ d->write32(14,i); printf(" %s=%x", getGPRName(i).c_str(), d->read32(0xe)); }
  printf("\n");
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
  bool initialize = true, sgi_mode = false, single_step = false;
  int fd, steps = 0, us_amt = 1;
  uint32_t pc = 0x0, max_fetches = 0;
  uint64_t max_iters;
  void *vaddr = nullptr;
  std::string chpt_name;
  rvstatus rs(0);  

  if(not(cmdline(argc, argv, initialize, chpt_name, max_fetches, max_iters, sgi_mode, single_step))) {
    return -1;
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
  
  pc = loadelf(chpt_name.c_str(), c_addr, sgi_mode);
  printf("starting pc %x\n", pc);
  uint32_t *cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);

  std::string arcs_image = "arcs_fw.bin";
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
      /* ARCS SPB lives at kseg1 0xA0001000 -> physical 0x1000 */
    memcpy(c_addr  + 0x1000, abuf, ast.st_size);
    munmap(abuf, ast.st_size);
    close(afd);
    std::cout << "loaded ARCS firmware (" << ast.st_size
	      << " bytes) at physical 0x1000\n";
    
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
  d->write32(PC_REG, pc);
  
  while(true) {
    __sync_synchronize();
    rs.u = d->read32(0xa);
    if(rs.s.ready) {
      printf("ready!, state %u\n", rs.s.state);
      break;
    }
  }


  
  uint32_t cr = single_step ? (8 | 2 | STEP) : (8 | 2);
  d->write32(4, cr);


  
  uint64_t zz= 0 , total_us = 0;
  uint64_t c = 0;
  volatile uint32_t *halt_flag = (volatile uint32_t*)(c_addr + 0x10D00000ULL); /* sgi: 0xBFD00000 -> c_addr[0x10D00000] */
  *halt_flag = 0;
  uint32_t magic_flag = 0, core_halt_u = 0; bool core_halted = false;
  
  while(c < max_iters && !done) {
    uint32_t s = cr | 1U<<30;
    if(single_step) {
      d->write32(4, s);
    }
    if((zz&POLL_FREQ) == 0) {
      total_us += us_amt;
      bool new_c = read_char_fifo();
      if(*halt_flag != 0) { magic_flag = *halt_flag; done = true; }
      rs.u = d->read32(0xa);
      if(cpu_stopped(rs)) { core_halt_u = rs.u; core_halted = true; done = true; }
      if(not(new_c)) {
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
  if(core_halted) { rvstatus hr(core_halt_u); printf("CORE HALTED: break=%u ud=%u bad_addr=%u monitor=%u\n", hr.s.break_, hr.s.ud, hr.s.bad_addr, hr.s.monitor); }
  printf("last pc = %x, insn cnt %u\n", d->read32(7), d->read32(0));
  dump_trace(d);
  printf("axi reads  %d\n", d->read32(18));
  printf("axi writes %d\n", d->read32(20));  

  pc = d->read32(7);
  cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);



  printf("cycles since last retired %u\n", d->read32(0x27));
  uint32_t epc = d->read32(0xb);
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

 if(1) {
   uint32_t n = d->read32(0);
   printf("%u instructions retired\n", d->read32(0));
   for(uint32_t i = 0; i <= n; i++) {
     d->write32(0x16, i);
     printf("%u : %x\n", i, d->read32(0x17));
   }
 }
	      
 
  munmap(c_addr, memsize);
  stopCapstone();
  return 0;
}
