#include <list>
#include "interpret.hh"
#include "driver.hh"

#ifndef __LOAD_ELF_H__
#define __LOAD_ELF_H__

uint32_t load_elf(const char* fn, Driver &d);
uint32_t load_elf(const char* fn, uint8_t *mem);

#endif 

