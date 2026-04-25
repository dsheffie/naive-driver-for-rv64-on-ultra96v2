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

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <boost/program_options.hpp>

#include "helper.hh"
#include "driver.hh"
#include "helper.hh"
#include "saveState.hh"

#define CONTROL_REG 0
#define STATUS_REG 1
#define RAM_REG 2
#define PC_REG 5
#define STEP_MASK (1UL<<16)
#define STEP_ACK (1UL<<31)
#define POLL_FREQ ((1UL<<12)-1)
#define MAX_LOG (1UL<<22)

#define MB ((1UL<<20))

static const uint32_t control = 0xA0050000;
static const uint64_t disk_addr = (384+32)*1024UL*1024UL;
static const uint64_t memsize = 496*MB;
static uint64_t phys_addr = ~0UL;
static uint64_t char_pos = 0, char_line_start = 0;
static char line_buf[1024] = {0};

static bool dump_mem = false;
static uint8_t *c_addr = nullptr;
static bool done = false;

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
  if(d == nullptr) {
    return;
  }
  uint64_t txns = read64(d, 0x3c);
  uint64_t lat = read64(d, 0x3e);
  uint32_t states = d->read32(0xd);    
  rvstatus rs(d->read32(0xa));
  uint64_t cycles = read64(d, 0x2a);
  uint64_t icnt = read64(d, 0x28);  
  std::cout << "core state = " << (states & 31) << "\n";
  std::cout << "l2 state = " << ((states>>5) & 31) << "\n";
  std::cout << "l1i state = " << ((states>>10) & 15) << "\n";
  std::cout << "l1d state = " << ((states>>14) & 15) << "\n";
  std::cout << "axi state = " << ((states>>18) & 7) << "\n";
  std::cout << rs << "\n";
  std::cout << "txns = " << txns << "\n";
  std::cout << "lat = " << lat << "\n";
  std::cout << "avg lat = " << static_cast<double>(lat)/txns << "\n";
  std::cout << "axi busy cycles = " << read64(d, 16) << "\n";
  uint64_t rd_txns = read64(d, 18);
  uint64_t wr_txns = read64(d, 20);
  std::cout << "cycles = " << cycles << "\n";
  std::cout << "icnt = " << icnt << "\n";
  std::cout << "ipc = " << static_cast<double>(icnt)/cycles << "\n";
  std::cout << "axi rds = " << rd_txns << "\n";
  std::cout << "axi wrs = " << wr_txns << "\n";
  uint64_t e_ld = read64(d, 0x16);
  uint64_t a_l2 = read64(d, 0x34);
  uint64_t h_l2 = read64(d, 0x36);
  std::cout << "early l1d loads   = " << e_ld << "\n";
  std::cout << "l2 accesses       = " << a_l2 << "\n";
  std::cout << "frac early = " << (100.0 * (static_cast<double>(e_ld)/a_l2)) << "\n";
  std::cout << "l2 hits           = " << read64(d, 0x36) << "\n";
  std::cout << "l2 hit ratio = " << (100.0 * (static_cast<double>(h_l2)/a_l2)) << "\n";
  double axi_bytes_per_cycle = static_cast<double>((rd_txns + wr_txns)*16UL) / cycles;
  std::cout << "axi bw = " << (axi_bytes_per_cycle*1e2) << " mbytes/sec\n";
}

static FILE *log_fp = nullptr;

void dumplog() {
  if(log_fp == nullptr) {
    return;
  }
  fclose(log_fp);
}


typedef unsigned char Rgb[3];


void sigintHandler(int id) {
  done = true;
}


static uint32_t xorshift32(uint32_t &x) {
  uint32_t t = x;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return t;
}

int main(int argc, char *argv[]) {
  namespace po = boost::program_options; 
  bool initialize = true;
  int fd, steps = 0, us_amt = 1;
  uint64_t i_pc = 0, ss = 0, zz = 0;
  void *vaddr = nullptr;
  std::string chpt_name;
  po::options_description desc("Options");
  rvstatus rs(0);  
  desc.add_options() 
    ("help,h", "Print help messages") 
    ("initialize,i", po::value<bool>(&initialize)->default_value(true), "initialize") 
    ("file,f", po::value<std::string>(&chpt_name), "checkpoint filename")
    ("dump,d", po::value<bool>(&dump_mem)->default_value(false), "dump phys mem on exit")
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

  fd = open("/dev/rv64core_fpga", O_RDWR | O_SYNC);
  assert(fd != -1);  
  if (ioctl(fd, 0, &phys_addr) < 0) {
    printf("error with fpga memory ioctl\n");
    close(fd);
    exit(-1);
  }
  printf("fpga memory starts at %lx\n", phys_addr);
  close(fd);
  
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
  
  d = new Driver(control);

  printf("open device driver\n");
  
  d->write32(6, phys_addr);

  printf("set phys addr\n");
  d->write32(8, memsize-1);

  printf("set mem size\n");  

  d->write32(CONTROL_REG, 0);
  d->write32(4, 1);
  d->write32(4, 0);

  printf("cleared control reg and reset board\n");  
  
  while(true) {
    __sync_synchronize();
    rs.u = d->read32(0xa);
    if(rs.s.ready) {
      printf("ready!\n");
      break;
    }
  }
  
  uint32_t cr = 8 | 2;
  d->write32(4, cr);

  while(true) {
    printf("last pc = %x\n", d->read32(7));
  }
  
  munmap(c_addr, memsize);
  return 0;
}
