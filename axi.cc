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
#include <capstone/capstone.h>

#include "helper.hh"
#include "driver.hh"
#include "helper.hh"
#include "saveState.hh"
#include "disassemble.hh"

union itype {
  struct {
    uint32_t imm : 16;
    uint32_t rs : 5;
    uint32_t rt : 5;
    uint32_t op : 6;
  } uu;
  uint32_t u;
};



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

static inline int get_axi_state() {
  uint32_t states = d->read32(0xd);
  return ((states>>18) & 7);
}

static inline void report_status() {
  if(d == nullptr) {
    return;
  }
  uint32_t states = d->read32(0xd);    
  std::cout << "core state = " << (states & 31) << "\n";
  std::cout << "l2 state = " << ((states>>5) & 31) << "\n";
  std::cout << "l1i state = " << ((states>>10) & 15) << "\n";
  std::cout << "l1d state = " << ((states>>14) & 15) << "\n";
  std::cout << "axi state = " << ((states>>18) & 7) << "\n";
}



void sigintHandler(int id) {
  done = true;
}


static const  uint32_t code[] = {
    0x24090001,
    0x3c070010, 
    0x3c080010, 
    0x01201021, 
    0x00001821, 
    0x00022340, 
    0x00822026, 
    0x00042c42, 
    0x00a42026, 
    0x00673021, 
    0x00042940, 
    0x24630004, 
    0xacc20000, 
    0x1468fff7, 
    0x00a41026, 
    0x08000013, 
    0x00001821, 
    0x1068000d, 
    0x00c44826, 
    0x00e32821, 
    0x00092340, 
    0x00892026, 
    0x8ca50000, 
    0x00043442, 
    0x00c42026, 
    0x00043140, 
    0x10a9fff6,
    0x24630004, 
    0x24090001, 
    0x08000003, 
    0x3c070010, 
    0x00e83821,
    0x08000003,
    0x00404821
  };


uint32_t loadelf(const char* fn, uint8_t *mem);

int main(int argc, char *argv[]) {
  namespace po = boost::program_options; 
  bool initialize = true;
  int fd, steps = 0, us_amt = 1;
  uint32_t pc = 0x20000, max_fetches = 0;  
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

  //0:	08000000 	j	0 <foo>
  //4:	00000000 	nop
  //*reinterpret_cast<uint32_t*>(&c_addr[0x20000]) = 0x08000000;

  //printf("code buffer has %zu instructions\n", sizeof(code)/sizeof(code[0]));
  //pc = 0x20714;
  pc = 0x0;    
   uint32_t *cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);
#if 0
   for(uint32_t r = 1; r < 32; r++) {
     itype y;
     y.uu.op = 13;
     y.uu.rt = 0;
     y.uu.rs = r;
     y.uu.imm = r & 0xffff;
     *cptr = bswap<false>(y.u);
     cptr++;
   }
#endif
   *cptr = bswap<false>(0x08000000); cptr++;   
   //*cptr = bswap<false>(0x3c020003); cptr++;
   //*cptr = bswap<false>(0x8c430000); cptr++;
   //*cptr = bswap<false>(0x24630001); cptr++;
   //*cptr = bswap<false>(0xac430000); cptr++;
   //*cptr = bswap<false>(0x080081c6); cptr++;
   //*cptr = bswap<false>(0x00000000); cptr++;


   

   cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);
   for(int i = 0; i < 10; i++) {
     std::cout << std::hex << pc+4*i << " : " << std::dec;
     disassemble(std::cout, bswap<false>(*cptr), pc + 4*i); std::cout << "\n";
     ++cptr;
   }

   d->write32(2, max_fetches);
   
  //pc = loadelf("spin.mips", c_addr);
  //*cptr = (0x08000000);
  
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

  sleep(1);
  printf("last pc = %x, insn cnt %u\n",
	 d->read32(7), d->read32(0));
  
  //d->write32(CONTROL_REG, cr|1U<<31);
  //d->write32(CONTROL_REG, cr);  
  //sleep(1);



  printf("last pc = %x, insn cnt %u\n",
	 d->read32(7), d->read32(0));  

  for(int i = 0; i < 32; i++) {
    d->write32(14, i);
    printf("reg %s : %x\n", getGPRName(i).c_str(),
	   d->read32(0xe));
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

  printf("axi reads  %d\n", d->read32(18));
  printf("axi writes %d\n", d->read32(20));  

  printf("%d register writes\n", d->read32(0x16));

  
   cptr = reinterpret_cast<uint32_t*>(&c_addr[pc]);
   for(int i = 0; i < 10; i++) {
     std::cout << std::hex << pc+4*i << " : " << std::dec;
     disassemble(std::cout, bswap<false>(*cptr), pc + 4*i); std::cout << "\n";
     ++cptr;
   }


  munmap(c_addr, memsize);
  stopCapstone();
  return 0;
}
