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
#include <boost/program_options.hpp>
#include <capstone/capstone.h>

#include "helper.hh"
#include "driver.hh"
#include "helper.hh"
#include "saveState.hh"
#include "disassemble.hh"

#define POLL_FREQ ((1UL<<12)-1)

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

uint32_t loadelf(const char* fn, uint8_t *mem);

int main(int argc, char *argv[]) {
  namespace po = boost::program_options; 
  bool initialize = true;
  int fd, steps = 0, us_amt = 1;
  uint32_t pc = 0x0, max_fetches = 0;  
  void *vaddr = nullptr;
  std::string chpt_name;
  po::options_description desc("Options");
  rvstatus rs(0);  
  desc.add_options() 
    ("help,h", "Print help messages") 
    ("initialize,i", po::value<bool>(&initialize)->default_value(true), "initialize") 
    ("file,f", po::value<std::string>(&chpt_name), "checkpoint filename")
    ("fetches", po::value<uint32_t>(&max_fetches)->default_value(0), "max fetches")
    ;  
  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm); 
  }
  catch(po::error &e) {
    std::cerr << "command-line error : " << e.what() << "\n";
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

  printf("start - open device driver\n");
  d = new Driver(control);
  printf("complete - open device driver\n");

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
  
  pc = loadelf(chpt_name.c_str(), c_addr);
  uint32_t *cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);
  for(int i = 0; i < 32; i++) {
    std::cout << std::hex << pc+4*i << " : " << std::dec;
    disassemble(std::cout, bswap<false>(*cptr), pc + 4*i); std::cout << "\n";
    ++cptr;
  }  

#if 0
#define INSN(XX) {*cptr = bswap<false>(XX); cptr++; }
  INSN(0x24030061);
  INSN(0x40023800);
  INSN(0x00000000);
  INSN(0x1440fffd);
  INSN(0x00000000);
  INSN(0x40833800);
  INSN(0x24630001);
  INSN(0x40023800);
  INSN(0x00000000);
  INSN(0x1440fff7);
  INSN(0x00000000);
  INSN(0x1000fff9);
  INSN(0x00000000);
  printf("pc = %x\n", pc);
  cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);
#endif
   
   __builtin___clear_cache((char*)vaddr, ((char*)vaddr) + memsize);
  
  d->write32(6, phys_addr);

  printf("set phys addr\n");
  d->write32(8, memsize-1);

  printf("set mem size\n");  

  d->write32(CONTROL_REG, 0);
  d->write32(4, 1);
  d->write32(4, 0);

  printf("cleared control reg and reset board\n");  
  d->write32(PC_REG, pc);
  
  while(true) {
    __sync_synchronize();
    rs.u = d->read32(0xa);
    if(rs.s.ready) {
      printf("ready!, state %u\n", rs.s.state);
      break;
    }
  }


  
  uint32_t cr = 8 | 2 /*| (1<<16)*/;
  d->write32(4, cr);

  //  while(1) {

  // printf("axi reads  %d\n", d->read32(18));
  //printf("axi writes %d\n", d->read32(20));
  uint64_t zz= 0 , total_us = 0;
  double last_time = timestamp(), now;
  while(1) {

#if 0
    printf("last pc = %x, insn cnt %u\n", d->read32(7), d->read32(0));

    for(int i = 0; i < 32; i++) {
      d->write32(14, i);
      uint32_t reg = d->read32(0xe);
      if(reg == 0) continue;
      printf("reg %s : %x\n", getGPRName(i).c_str(), reg);
    }
    
    for(int i = 0; i < 16; i++) {
      d->write32(12, i);
      printf("%d: %x, op %u, branch target %x, dest reg %s\n", i,
	     d->read32(12),
	     d->read32(0x17),
	     d->read32(0x19), 
	     getGPRName(d->read32(0x18)).c_str());
    }
    rs.u = d->read32(0xa);
    printf("core state %u\n", rs.s.state);
  #endif    

    if((zz&POLL_FREQ) == 0) {
      now = timestamp();
      if((now-last_time) > 1.0) {
	break;
      }      
      total_us += us_amt;
      bool new_c = read_char_fifo();
      if(not(new_c)) {
	usleep(us_amt);
       	// printf("last pc %x\n", d->read32(0x7));
	// for(int i = 0; i <31; i++) {
	//   d->write32(19, i);
	//   printf("%d : %x\n", i, d->read32(0x1a));
	// }
	us_amt = std::min(us_amt+1, 1000);
      }
      else {
	last_time = timestamp();
	us_amt = 1;
      }
    }
    
  }

#if 0  
  for(int i = 0; i < 32; i++) {
    d->write32(14, i);
    uint32_t reg = d->read32(0xe);
    if(reg == 0) continue;
    printf("reg %s : %x\n", getGPRName(i).c_str(), reg);
  }
  
  for(int i = 0; i < 16; i++) {
    d->write32(12, i);
    printf("%d: %x, op %u, branch target %x, dest reg %s\n", i,
	   d->read32(12),
	   d->read32(0x17),
	   d->read32(0x19), 
	   getGPRName(d->read32(0x18)).c_str());
  }
#endif
  printf("last pc = %x, insn cnt %u\n", d->read32(7), d->read32(0));
  printf("axi reads  %d\n", d->read32(18));
  printf("axi writes %d\n", d->read32(20));  

  printf("%d register writes\n", d->read32(0x16));
  rs.u = d->read32(0xa);
  printf("core state %u\n", rs.s.state);
  printf("l1d state %u\n", rs.s.l1d_state);
  printf("l1i state %u\n", rs.s.l1i_state);  
  printf("l2 state %u\n", rs.s.l2_state);  

  uint32_t states = d->read32(0xd);    
  std::cout << "core state = " << (states & 31) << "\n";
  std::cout << "l2 state = " << ((states>>5) & 15) << "\n";
  std::cout << "l1i state = " << ((states>>9) & 7) << "\n";
  std::cout << "l1d state = " << ((states>>12) & 15) << "\n";
  std::cout << "axi state = " << ((states>>16) & 15) << "\n";      
  

  munmap(c_addr, memsize);
  stopCapstone();
  return 0;
}
