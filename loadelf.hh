#include <list>
#include "interpret.hh"

#ifndef __LOAD_ELF_H__
#define __LOAD_ELF_H__

bool load_elf(const char* fn, state_t *ms);
bool is_rv64_elf(const char* fn);

#endif 

