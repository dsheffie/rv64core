#ifndef __SAVE_STATE_HH__
#define __SAVE_STATE_HH__

#include <string>  // for string
#define ELIDE_STATE_IMPL
#include "interpret.hh"
#undef ELIDE_STATE_IMPL

void dumpState(const state_t &s, const std::string &filename);
void loadState(state_t &s, const std::string &filename);

#endif
