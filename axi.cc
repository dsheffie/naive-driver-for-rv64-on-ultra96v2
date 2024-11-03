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
#include <arm_neon.h>

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
//static const uint32_t control = 0xA0000000;
static const uint32_t ram = 0x40e00000;

#define CONTROL_REG 0
#define STATUS_REG 1
#define RAM_REG 2

struct status_ {
  uint32_t state : 3;
  uint32_t hist : 8;
  uint32_t bresp : 2;
  uint32_t hash : 15;  
  uint32_t read_resp_error : 1;
  uint32_t write_resp_error : 1;
  uint32_t read_mismatch : 1;
  uint32_t rnext : 1;
};

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
  uint32_t l2_state : 4;
};

static_assert(sizeof(rvstatus_) == 4, "rvstatus bad size");

union status {
  uint32_t u;
  status_ s;
  status(uint32_t u) :u(u) {}
};

std::ostream &operator<<(std::ostream &out, const status &s) {
  out << "state       : " <<  s.s.state << "\n";
  out << "hist        : " << std::hex << s.s.hist << std::dec << "\n";
  out << "bresp       : " << s.s.bresp << "\n";
  out << "read error  : " << s.s.read_resp_error << "\n";
  out << "write error : " << s.s.write_resp_error << "\n";
  
  return out;
}


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
  
  return out;
}


static_assert(sizeof(status) == 4, "wrong size for status");

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

void sigintHandler(int id) {
  if(d) {
    printf("bad addr %x\n", d->read32(7));    
    uint64_t txns = read64(d, 0x3c);
    uint64_t mem_cycles = read64(d, 0x3e);
    double avg = static_cast<double>(mem_cycles) / txns;
    std::cout << "avg = " << avg << "\n";
  }
  std::cout << "mem_ops = " << mem_ops << "\n";
  std::cout << "zz = " << zz << "\n";  
  exit(-1);
}


int main(int argc, char *argv[]) {
  
  d = new Driver(control);
  d->write32(CONTROL_REG, 0);
  d->write32(4, 1);
  d->write32(4,0);
  status s(0);

  d->write32(6,ram);

  Driver dd(ram);

  uint8_t *mem_ = mmap4G();
  char *bin_name = nullptr;
    
  if(argc == 2) {
    bin_name = argv[1];
  }
  uint64_t pc = 0;
  bool isElf = true;
  pc = load_elf(bin_name, mem_);
  if(pc == (~0U)) {
    isElf = false;
    pc = loadState(mem_, bin_name);
  }
  
  //make addresses 0 based
  d->write32(6,0);
  uint32_t *mem_w32 = reinterpret_cast<uint32_t*>(mem_);
  
  rvstatus rs(d->read32(0xa));


  //resume pc
  d->write32(5, pc);
  printf("initial pc = %x\n", d->read32(5));

  //1 -> bit[0]
  //2 -> bit[1]
  //4 -> bit[2]
  //8 -> bit[3]
  
  //syscall emulation is 8
  d->write32(4, 8 | 2);
  
  bool run = true;
  uint64_t  mem_reads = 0, mem_writes = 0;
  uint64_t silent_wbs = 0;
  uint64_t z = 0;

  while(true) {
    rs.u = d->read32(0xa);
    if(not(rs.s.ready)) {
      break;
    }
    std::cout << rs;
  }
  
  //std::cout << std::hex << d.read32(1) << std::dec << "\n";
  //exit(-1);
  signal(SIGINT, sigintHandler);
  volatile uint32_t *rptr = d->get_ptr(32);
  volatile uint32_t *wptr = d->get_ptr(36);    

  uint32_t la = ~0;
  uint32_t las[8];
  int ia = 0;
  int op = ~0;
  uint64_t l = 0;
  double start_time = timestamp();
  while(run) {
    rs.u = d->read32(0xa);

    if((zz & 63) == 0) {
      while(d->read32(0x3a) == 0) {
	int c = d->read32(0x3b);
	printf("%c", c==0 ? '\n' : c);
	std::fflush(nullptr);
	d->write32(0x3a, 1);
	d->write32(0x3a, 0);
      }
    }

    //if(l > (1UL<<24)) {
    //printf("timeout\n");
    //break;
    //}

    //if(rs.s.bad_addr) {
    //printf("boned!\n");
    //break;
    //}

    //l++;
    if(rs.s.mem_req) {
      //l = 0;
      mem_ops++;
      uint32_t addr = d->read32(15);
      //la = addr;
      //las[ia & 7] = addr;
      //ia++;
      //op = rs.s.mem_req_opcode;
      //printf("got req for addr %x, type %d\n",
      //addr, rs.s.mem_req_opcode);
      
      if(rs.s.mem_req_opcode == 4) {
	++mem_reads;
	volatile uint32_t *dptr = &mem_w32[(addr/4)];
	int32x4_t t = vld1q_s32((const int32_t*)(dptr));
	vst1q_s32((int32_t*)(wptr), t);
      }
      else {
	assert((addr >> 12) != 0);
	++mem_writes;
	int32x4_t t = vld1q_s32((const int32_t*)(rptr));
	volatile uint32_t *dptr = &mem_w32[(addr/4)];
	vst1q_s32((int32_t*)(dptr), t);
      }
      d->write32(CONTROL_REG, 1U<<31);
      //ack
      //d->write32(CONTROL_REG, 0);
    }

    zz++;
  }

  printf("mem_ops = %lu in %g seconds\n", mem_ops, start_time);

  return 0;
  
  uint64_t insns = read64(d, 0x28);
  uint64_t cycle = read64(d, 0x2a);
  uint64_t l1i_accesses = read64(d, 0x2c);
  uint64_t l1i_hits = read64(d, 0x2e);
  uint64_t l1d_accesses = read64(d, 0x30);
  uint64_t l1d_hits = read64(d, 0x32);
  uint64_t l2_accesses = read64(d, 0x34);
  uint64_t l2_hits = read64(d, 0x36);
  uint64_t mispredicts = read64(d, 0x38);    
  std::cout << "insns        = " << insns << "\n";
  std::cout << "cycle        = " << cycle << "\n";
  std::cout << "l1i accesses = " << l1i_accesses << "\n";
  std::cout << "l1i hits     = " << l1i_hits << "\n";
  std::cout << "l1d accesses = " << l1d_accesses << "\n";
  std::cout << "l1d hits     = " << l1d_hits << "\n";
  std::cout << "l2 accesses  = " << l2_accesses << "\n";
  std::cout << "l2 hits      = " << l2_hits << "\n";    
  std::cout << "mispredicts  = " << mispredicts << "\n";
  
  double ipc = static_cast<double>(insns) / cycle;
  double mpki = 1000.0 * (static_cast<double>(mispredicts) / insns);
  std::cout << "ipc   = " << ipc << "\n";
  std::cout << "mpki  = " << mpki << "\n";
  std::cout << "mem_writes = " << mem_writes << "\n";
  std::cout << "mem_reads  = " << mem_reads << "\n";  
  //std::cout << "silent_wbs = " << silent_wbs << "\n";  
  munmap(mem_, 1UL<<32);
  delete d;
  return 0;
  
  while(true) {
    sleep(1);
    s.u = d->read32(STATUS_REG);
    std::cout << "state : " << s.s.state << "\n";
    std::cout << "hist : " << std::hex << s.s.hist << std::dec << "\n";
    std::cout << "bresp : " << s.s.bresp << "\n";
    std::cout << "read_resp_error : " << s.s.read_resp_error << "\n";
    std::cout << "write_resp_error : " << s.s.write_resp_error << "\n";
    std::cout << "read_mismatch : " << s.s.read_mismatch << "\n";
    std::cout << "rnext         : " << s.s.rnext << "\n";
    std::cout << "zero        : " << s.s.hash << "\n";
    
    rs.u = d->read32(0xa);
    std::cout << rs << "\n";
    
    std::cout << std::hex << "memory txns : " << d->read32(14) << std::dec << "\n";
    std::cout << std::hex << "last addr : " << d->read32(15) << std::dec << "\n";

    if(rs.s.mem_req) {
      std::cout << "see mem req, force rsp_valid\n";
      d->write32(CONTROL_REG, 1U<<31);
      d->write32(CONTROL_REG, 0);
    }
    
    for(int i = 0; i < 4; i++) {
      std::cout << std::hex << d->read32(16+i) << std::dec << "\n";
    }
    
    if(cpu_stopped(rs)) {
      std::cout << "EPC : " << std::hex << d->read32(7) << std::dec << "\n";
      std::cout << "insn retired " << d->read32(0) << "\n";
      std::cout << std::hex << "memory txns : " << d->read32(14) << std::dec << "\n";
      std::cout << std::hex << "last addr : " << d->read32(15) << std::dec << "\n";
      break;
    }
    sleep(1);
  }
  
  for(int i = 0; i < 4; i++) {
    std::cout << std::hex << d->read32(16+i) << std::dec << "\n";
  }
  


  
#if 0
  Driver dd(ram);
  std::cout << "dram values interface:\n";  
  for(int i = 0; i < 1024; i++) {
    std::cout << dd->read32(i) << "\n";
  }
#endif

  //std::cout << "last read interface:\n";
  //d->write32(CONTROL_REG, 1);
  //d->write32(CONTROL_REG, 0);
  
  //for(int i = 0;i < 4; i++) {
  //std::cout << std::hex << d->read32(4+i) << std::dec << "\n";
  //}
  
  //for(int i = 0; i < 4; i++) {
  //std::cout << std::hex << dd->read32(i) << std::dec << "\n";
  //}


  
  return 0;
}
