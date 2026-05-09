#include "driver.hh"
#include <fcntl.h>
#include <unistd.h>
#include <cassert>
#include <cstdio>
#include <sys/mman.h>

Driver::Driver(uintptr_t paddr)
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
    printf("vaddr = %p\n", vaddr);
    assert((void*)vaddr != MAP_FAILED);
    vaddr = vaddr + (paddr & (pgsize-1));
    ptr32 = reinterpret_cast<volatile uint32_t*>(vaddr);
    printf("magic key %x\n", *ptr32);
  }
  
  
uint32_t Driver::read32(int port) {
  return ptr32[port];
}

void Driver::write32(int port, uint32_t value) {
  ptr32[port] = value;
}

volatile uint32_t *Driver::get_ptr(int port) {
    return ptr32+port;
}

int Driver::num_ports() {
  return regs;
}

uint8_t *Driver::get_vaddr() const {
  return reinterpret_cast<uint8_t*>(vaddr);
}
