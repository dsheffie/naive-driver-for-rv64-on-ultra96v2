#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <utility>
#include <cstdint>
#include <list>
#include <map>
#include <elf.h>

#include "helper.hh"
#include "interpret.hh"
#include "loadelf.hh"

static const uint8_t magicArr[4] = {0x7f, 'E', 'L', 'F'};

bool checkElf(const Elf64_Ehdr *eh32) {
  uint8_t *identArr = (uint8_t*)eh32->e_ident;
  return memcmp((void*)magicArr, identArr, 4)==0;
}

bool check32Bit(const Elf64_Ehdr *eh32) {
  return (eh32->e_ident[EI_CLASS] == ELFCLASS64);
}

bool checkBigEndian(const Elf64_Ehdr *eh32) {
  return (eh32->e_ident[EI_DATA] == ELFDATA2MSB);
}

bool checkLittleEndian(const Elf64_Ehdr *eh32) {
  return (eh32->e_ident[EI_DATA] == ELFDATA2LSB);
}

uint32_t load_elf(const char* fn, Driver &d) {
  uint8_t *mem = d.get_vaddr();  
  return load_elf(fn, mem);
}

uint32_t load_elf(const char* fn, uint8_t *mem) {
  struct stat s;
  Elf64_Ehdr *eh32 = nullptr;
  Elf64_Phdr* ph32 = nullptr;
  Elf64_Shdr* sh32 = nullptr;
  int32_t e_phnum=-1,e_shnum=-1;
  size_t pgSize = getpagesize();
  int fd,rc;
  char *buf = nullptr;
  
  /* symbol code shamelessly stolen from
   * elfloader.cc - need for to/from host
   * symbols */
  int32_t strtabidx = 0, symtabidx = 0;

  fd = open(fn, O_RDONLY);
  if(fd<0) {
    return ~0U;
  }
  rc = fstat(fd,&s);
  if(rc<0) {
    return ~0U;    
  }
  buf = (char*)mmap(nullptr, s.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
  eh32 = (Elf64_Ehdr *)buf;
  close(fd);
    
  if(!checkElf(eh32)) {
    return ~0U;    
  }

  if(!check32Bit(eh32)) {
    return ~0U;    
  }
  assert(checkLittleEndian(eh32));

  if((eh32->e_machine) != 243) {
    return ~0U;        
  }


  e_phnum = (eh32->e_phnum);
  ph32 = reinterpret_cast<Elf64_Phdr*>(buf + eh32->e_phoff);
  e_shnum = eh32->e_shnum;
  sh32 = reinterpret_cast<Elf64_Shdr*>(buf + eh32->e_shoff);
  char* shstrtab = buf + sh32[eh32->e_shstrndx].sh_offset;

  uint32_t pc = eh32->e_entry;
  //assert(pc==0);
  //std::cout << "pc = " << std::hex << pc << std::dec << "\n";
  
  /* Find instruction segments and copy to
   * the memory buffer */
  for(int32_t i = 0; i < e_phnum; i++, ph32++) {
    auto p_memsz = (ph32->p_memsz);
    auto p_offset = (ph32->p_offset);
    auto p_filesz = (ph32->p_filesz);
    auto p_type = ph32->p_type;
    auto p_vaddr = ph32->p_vaddr;

#if 0
    std::cout << "p_type = "
	      << std::hex
	      << p_type
	      << " size = "
	      << p_memsz
	      << std::dec << "\n";
#endif
    
    if(p_type == SHT_PROGBITS && p_memsz) {

      std::cout << "copying progbits VA :  "
		<< std::hex << p_vaddr
		<< " to "
		<< (p_vaddr + p_memsz)
		<< std::dec << "\n";

      
      std::cout << "mem = "<< std::hex
		<< reinterpret_cast<uint64_t>(mem)
		<< std::dec << "\n";
      
      memset(mem+p_vaddr, 0, sizeof(uint8_t)*p_memsz);
      
      memcpy(mem+p_vaddr, (uint8_t*)(buf + p_offset),
	     sizeof(uint8_t)*p_filesz);
    }
  }
  /* this code is all basically from elfloader.cc */
  munmap(buf, s.st_size);
  return pc;
}
