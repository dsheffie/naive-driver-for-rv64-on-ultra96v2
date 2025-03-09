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
#include <unistd.h>

#include <regex>
#include <iostream>
#include <fstream>
#include <string>
#include <bitset>
#include <map>

#include "helper.hh"
#include "driver.hh"
#include "loadelf.hh"
#include "globals.hh"
#include "helper.hh"
#include "saveState.hh"

#include "syscall.h"

uint32_t globals::tohost_addr;
uint32_t globals::fromhost_addr;

int globals::sysArgc;
char **globals::sysArgv;
bool globals::silent;
bool globals::log;
std::map<std::string, uint32_t> globals::symtab;

static const uint32_t control = 0xA0050000;
static const uint64_t disk_addr = (384+32)*1024UL*1024UL;

#define PHYS_ADDR 0x60100000
#define CONTROL_REG 0
#define STATUS_REG 1
#define RAM_REG 2
#define PC_REG 5

#define STEP_MASK (1UL<<16)
#define STEP_ACK (1UL<<31)

struct rvstatus_ {
  uint32_t ready : 1;
  uint32_t flush : 1;
  uint32_t break_: 1;
  uint32_t ud : 1;
  uint32_t bad_addr : 1;
  uint32_t monitor : 1;
  uint32_t state : 5;
  uint32_t l1d_flushed : 1;
  uint32_t l1i_flushed : 1;
  uint32_t l2_flushed : 1;
  uint32_t reset_out : 1;
  uint32_t mem_req : 1;
  uint32_t mem_req_opcode : 4;
  uint32_t l1d_state : 4;
  uint32_t mem_rsp : 1;
  uint32_t l1i_state : 3;
  uint32_t l2_state : 2;
  uint32_t memq_empty : 1;
};

static_assert(sizeof(rvstatus_) == 4, "rvstatus bad size");

union rvstatus {
  uint32_t u;
  rvstatus_ s;
  rvstatus(uint32_t u) : u(u) {}
};

inline bool cpu_stopped(const rvstatus &rs) {
  return rs.s.break_ or rs.s.ud or rs.s.bad_addr or rs.s.monitor;
}

std::ostream &operator<<(std::ostream &out, const rvstatus &rs) {
  out << "ready           : " << rs.s.ready << "\n";
  out << "flush           : " << rs.s.flush << "\n";
  out << "break           : " << rs.s.break_ << "\n";
  out << "undef inst      : " << rs.s.ud << "\n";
  out << "bad address     : " << rs.s.bad_addr << "\n";
  out << "monitor         : " << rs.s.monitor << "\n";
  out << "state           : " << rs.s.state << "\n";
  out << "l1d_flushed     : " << rs.s.l1d_flushed << "\n";
  out << "l1i_flushed     : " << rs.s.l1i_flushed << "\n";
  out << "l2_flushed      : " << rs.s.l2_flushed << "\n";
  out << "reset           : " << rs.s.reset_out << "\n";
  out << "mem_req         : " << rs.s.mem_req << "\n";
  out << "mem_req_opcode  : " << rs.s.mem_req_opcode << "\n";
  out << "l1d state       : " << rs.s.l1d_state << "\n";
  out << "l1i state       : " << rs.s.l1i_state << "\n";
  out << "l2  state       : " << rs.s.l2_state << "\n";  
  out << "mem_rsp         : " << rs.s.mem_rsp << "\n";
  out << "memq_empty      : " << rs.s.memq_empty << "\n";
  return out;
}

static inline uint8_t *mmap4G() {
  void* mempt = mmap(nullptr, 1UL<<32, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  assert(mempt != reinterpret_cast<void*>(-1));
  assert(madvise(mempt, 1UL<<32, MADV_DONTNEED)==0);

  return reinterpret_cast<uint8_t*>(mempt);
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
static uint64_t zz = 0, mem_ops = 0;

static inline int get_axi_state() {
  uint32_t states = d->read32(0xd);
  return ((states>>18) & 7);
}

static inline void report_status() {
  if(d) {
    uint64_t txns = read64(d, 0x3c);
    uint64_t lat = read64(d, 0x3e);
    uint32_t states = d->read32(0xd);    
    rvstatus rs(d->read32(0xa));
    std::cout << "core state = " << (states & 31) << "\n";
    std::cout << "l2 state = " << ((states>>5) & 31) << "\n";
    std::cout << "l1i state = " << ((states>>10) & 15) << "\n";
    std::cout << "l1d state = " << ((states>>14) & 15) << "\n";
    std::cout << "axi state = " << ((states>>18) & 7) << "\n";
    std::cout << rs << "\n";
    std::cout << "txns = " << txns << "\n";
    std::cout << "lat = " << lat << "\n";
    std::cout << "avg lat = " << static_cast<double>(lat)/txns << "\n";
  }
}
#define POLL_FREQ ((1UL<<12)-1)
#define MAX_LOG (1UL<<22)
static uint64_t char_pos = 0, char_buf_sz = 0, char_line_start = 0;
static char *log_buf = nullptr;

void dumplog() {
  if(log_buf != nullptr) {
    FILE *fp = fopen("output.txt", "w");
    assert(fp);
    fwrite(log_buf, 1, char_pos, fp);
    fclose(fp);
  }
}

void sigintHandler(int id) {
  report_status();
  dumplog();
  exit(-1);
}


static inline bool read_char_fifo(bool &done) {
  int v = d->read32(0x3a) & 255;
  int wptr =v&0xf, rptr = (v>>4)&0xf;
  if(wptr == rptr) {
    return false;
  }
  int c = d->read32(0x3b);	
  printf("%c", c==0 ? '\n' : c);

  if(char_pos == char_buf_sz) {
    if(char_buf_sz == MAX_LOG) {
      char_pos = 0;
    }
    else {
      size_t n_sz = std::max(1024UL, 2*char_buf_sz);
      char *t = new char[n_sz];
      if(log_buf != nullptr) {
	memset(t, 0, n_sz);
	memcpy(t, log_buf, char_buf_sz);
      }
      char_buf_sz = n_sz;
      free(log_buf);
      log_buf = t;
    }
  }
  //printf("char_pos = %lu, char_buf_sz = %lu\n", char_pos, char_buf_sz);
  log_buf[char_pos++] = c==0 ? '\n' : c;
  if(c==0 or c == '\n') {
    char *l = log_buf+char_line_start;
    size_t len = strlen(l);
    int m = strncmp("fpga_done", l, 9);
    //printf("len = %zu, m = %d, %s \n", len, m, l);
    if(m == 0) {
      done = true;
    }
    char_line_start = char_pos;
  }
  std::fflush(nullptr);
  d->write32(0x3a, 1);
  d->write32(0x3a, 0);
  return true;
}

#define WRITE_WORD(EA,WORD) { *reinterpret_cast<uint32_t*>(c_addr + EA) = WORD; }

int main(int argc, char *argv[]) {
  if(argc != 2)
    return -1;
  uintptr_t pgsize = sysconf(_SC_PAGESIZE);
  size_t memsize = 448*1024*1024;
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  assert(fd != -1);
  
  int prot = PROT_READ | PROT_WRITE;
  int flags = MAP_SHARED;
  void* vaddr = mmap(0,
		     memsize,
		     prot,
		     flags,
		     fd,
		     PHYS_ADDR);
  assert(vaddr != MAP_FAILED);
  uint8_t *c_addr = reinterpret_cast<uint8_t*>(vaddr);
  memset(vaddr, 0x00, memsize);
  
  uint64_t i_pc = loadState(c_addr, argv[1]);
  d = new Driver(control);
  d->write32(CONTROL_REG, 0);
  d->write32(4, 1);
  d->write32(4, 0);
  
  d->write32(6,PHYS_ADDR);
  d->write32(8, memsize-1);
  
  d->write32(PC_REG, i_pc);
  __builtin___clear_cache((char*)vaddr, ((char*)vaddr) + memsize);

  //usleep(1000);
  
  rvstatus rs(0);
  while(true) {
    __sync_synchronize();
    rs.u = d->read32(0xa);
    if(rs.s.ready) {
      break;
    }
  }

  signal(SIGINT, sigintHandler);
  
  //#define DO_STEP
  uint32_t cr = 8 | 2;

  if(getenv("STEP") != nullptr) {
    cr |= STEP_MASK;
  }
  __sync_synchronize();
  
  /* let the games begin */
   d->write32(4, cr);

  int steps = 0;
  uint64_t ss = 0, zz = 0;
  bool done = false;
  int us_amt = 1;
  while(not(done)) {
    ss++;
    zz++;
    
    if((zz&POLL_FREQ) == 0) {
      bool new_c = read_char_fifo(done);
      if(not(new_c)) {
	usleep(us_amt);
	us_amt = std::min(us_amt+1, 100);
      }
      else {
	us_amt = 1;
      }
    }
    
#if 0
    if(cr & STEP_MASK) {
      int state = get_axi_state();
      if(not(state == 5 || state == 6)) {
	ss++;
	continue;
      }
      ss = 0;
      uint32_t cr_ = cr | STEP_ACK;
      d->write32(4, cr_);
#if 1
      uint64_t disp = (d->read32(0x8) - PHYS_ADDR);
      std::cout
	<< "state " << state
	<< " addr " << std::hex << (disp)
	<< " data " << d->read32(0x9)
	<< " pc " << d->read32(7)
	<< std::dec 
	<< " txns " << d->read32(1)
	<< " step " << steps
	<< "\n";
#endif
      //cr &= (~STEP_MASK);
      d->write32(4, cr);
      steps++;
    }
#endif
    
    //printf("steps = %d\n", steps);
  }


  // {
  //   uint8_t *buf = c_addr+disk_addr;    
  //   int fd = ::open("disk.img", O_RDWR|O_CREAT|O_TRUNC, 0600);
  //   write(fd, buf, 16*1024*1024);
  //   close(fd);
  // }
  

  printf("last pc %x\n", d->read32(7));    

  report_status();

  std::cout << "axi txns " << d->read32(0x1) << "\n";
  std::cout << std::hex << "epc : "
	    << d->read32(0xb) << std::dec << "\n";


  std::cout << "last addr " << std::hex << d->read32(0x8) << std::dec << "\n";
  std::cout << "last data " << std::hex << d->read32(0x9) << std::dec << "\n";  
  //exit(-1);
  //signal(SIGINT, sigintHandler);

  dumplog();
  //printf("fdt_magic = %x\n", *fdt_magic);
  
  munmap(c_addr, memsize);
  return 0;
}
