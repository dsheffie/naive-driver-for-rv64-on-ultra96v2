#include <boost/dynamic_bitset.hpp>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include "interpret.hh"

struct page {
  uint32_t va;
  uint8_t data[4096];
} __attribute__((packed));

static const uint64_t MAGICNUM = 0x64646464beefd005UL;

struct header {
  uint64_t magic;
  uint64_t pc;
  int64_t gpr[32];
  uint64_t icnt;
  uint32_t num_nz_pages;
  uint64_t tohost_addr;
  uint64_t fromhost_addr;
  int64_t priv;
  int64_t mstatus;
  int64_t misa;
  int64_t mideleg;
  int64_t medeleg;
  int64_t mscratch;
  int64_t mhartid;
  int64_t mtvec;
  int64_t mcounteren;
  int64_t mie;
  int64_t mip;
  int64_t mcause;
  int64_t mepc;
  int64_t mtval;
  int64_t sscratch;
  int64_t scause;
  int64_t stvec;
  int64_t sepc;
  int64_t sip;
  int64_t stval;
  int64_t satp;
  int64_t scounteren;
  int64_t pmpaddr0;
  int64_t pmpaddr1;
  int64_t pmpaddr2;
  int64_t pmpaddr3;
  int64_t pmpcfg0;
  int64_t mtimecmp;  
  header() {}
} __attribute__((packed));


uint64_t loadState(uint8_t *mem, const std::string &filename) {
  header h;
  int fd = ::open(filename.c_str(), O_RDONLY, 0600);
  assert(fd != -1);
  size_t sz = read(fd, &h, sizeof(h));
  assert(sz == sizeof(h));
  //assert(h.magic == MAGICNUM);
  printf("header has %lu pages\n", static_cast<uint64_t>(h.num_nz_pages));
  for(uint32_t i = 0; i < h.num_nz_pages; i++) {
    page p;
    sz = read(fd, &p, sizeof(p));
    //std::cout << "sz = " << sz << "\n";
    assert(sz == sizeof(p));
    memcpy(mem+p.va, p.data, 4096);
  }
  close(fd);
  return h.pc;
}

