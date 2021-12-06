#include <boost/dynamic_bitset.hpp>
#include <iostream>
#include <cstdint>
#include <cassert>
#include <fcntl.h>

#include "sparse_mem.hh"
#include "saveState.hh"

struct page {
  uint32_t va;
  uint8_t data[4096];
} __attribute__((packed));

struct header {
  static const uint64_t magic = 0xbeefcafefacebabe;
  uint32_t pc;
  int32_t gpr[32];
  int32_t lo;
  int32_t hi;
  uint32_t cpr0[32];
  uint32_t cpr1[32];
  uint32_t fcr1[5];
  uint64_t icnt;
  uint32_t num_nz_pages;
  header() {}
} __attribute__((packed));

void loadState(state_t &s, const std::string &filename) {
  int fd = ::open(filename.c_str(), O_RDONLY, 0600);
  assert(fd != -1);
  header h;
  size_t sz = read(fd, &h, sizeof(h));
  //std::cerr << std::hex << "magic = " << h.magic << std::dec << "\n";
  assert(sz == sizeof(h));
  assert(h.pc != 0);
  
  s.pc = h.pc;
  memcpy(&s.gpr,&h.gpr,sizeof(s.gpr));
  s.lo = h.lo;
  s.hi = h.hi;
  memcpy(&s.cpr0,&h.cpr0,sizeof(s.cpr0));
  memcpy(&s.cpr1,&h.cpr1,sizeof(s.cpr1));
  memcpy(&s.fcr1,&h.fcr1,sizeof(s.fcr1));
  s.icnt = 0;

  s.mem.clear();
  
  for(uint32_t i = 0; i < h.num_nz_pages; i++) {
    page p;
    sz = read(fd, &p, sizeof(p));
    assert(sz == sizeof(p));
    for(uint32_t j = 0; j < 4096; j++) {
      s.mem.set(p.va+j, p.data[j]);
    }
  }
  close(fd);
}
