#ifndef _DRIVER_HH
#define _DRIVER_HH

#include <iostream>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <cassert>
#include <cstdio>
#include <sys/mman.h>

class Driver
{
 private:
  uintptr_t vaddr;
  uintptr_t paddr;
  volatile uint32_t *ptr32;
  static const int regs = 8;

  volatile uintptr_t* reg_addr(int port) {
    return (volatile uintptr_t*)vaddr + (regs - port - 1);
  }
    
 public:
  Driver(uintptr_t paddr)
  {
    this->paddr = paddr;
    uintptr_t pgsize = sysconf(_SC_PAGESIZE);
    size_t memsize = 16*1024*1024;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    assert(fd != -1);

    int prot = PROT_READ | PROT_WRITE;
    int flags = MAP_SHARED;
    vaddr = (uintptr_t)mmap(0,
			    memsize,
			    prot,
			    flags,
			    fd,
			    paddr & ~(pgsize-1));
    assert((void*)vaddr != MAP_FAILED);
    vaddr = vaddr + (paddr & (pgsize-1));
    ptr32 = reinterpret_cast<volatile uint32_t*>(vaddr);
  }
  
  
  uint32_t read32(int port) {
    return ptr32[port];
  }
  void write32(int port, uint32_t value) {
    ptr32[port] = value;
  }
  volatile uint32_t *get_ptr(int port) {
    return ptr32+port;
  }
  int num_ports() {
    return regs;
  }
  
  uint8_t *get_vaddr() const {
    return reinterpret_cast<uint8_t*>(vaddr);
  }
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
  uint32_t l2_state : 2;
  uint32_t memq_empty : 1;
};

static_assert(sizeof(rvstatus_) == 4, "rvstatus bad size");

union rvstatus {
  uint32_t u;
  rvstatus_ s;
  rvstatus(uint32_t u) : u(u) {}
};



#endif
