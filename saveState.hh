#ifndef __SAVE_STATE_HH__
#define __SAVE_STATE_HH__

#include <string>  // for string
#include <cstdint>

uint64_t loadState(uint8_t *mem, const std::string &filename);

#endif
