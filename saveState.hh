#ifndef __SAVE_STATE_HH__
#define __SAVE_STATE_HH__

#include <string>  // for string
#include "interpret.hh"

void dumpState(const state_t &s, const std::string &filename);
void loadState(state_t &s, const std::string &filename);

#endif
