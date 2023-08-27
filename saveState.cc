#include <boost/dynamic_bitset.hpp>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include "interpret.hh"

struct page {
  uint32_t va;
  uint8_t data[4096];
} __attribute__((packed));

static const uint64_t MAGICNUM = 0xbeefd00d12345670UL;

struct header {
  uint64_t magic;
  uint32_t pc;
  int32_t gpr[32];
  uint64_t icnt;
  uint32_t num_nz_pages;
  header() {}
} __attribute__((packed));

void dumpState(const state_t &s, const std::string &filename) {
  static const int n_pages = 1<<20;
  header h;
  boost::dynamic_bitset<> nz_pages(n_pages,false);
  uint64_t *mem64 = reinterpret_cast<uint64_t*>(s.mem);
  static_assert(sizeof(page)==4100, "struct page has weird size");
  
  /* mark non-zero pages */
  for(int p = 0; p < n_pages; p++) {
    for(int pp = 0; pp < 512; pp++) {
      if(mem64[p*512+pp]) {
	nz_pages[p] = true;
	break;
      }
    }
  }
  int fd = ::open(filename.c_str(), O_RDWR|O_CREAT|O_TRUNC, 0600);
  assert(fd != -1);
  h.magic = MAGICNUM;
  h.pc = s.pc;
  memcpy(&h.gpr,&s.gpr,sizeof(s.gpr));
  h.icnt = s.icnt;
  h.num_nz_pages = nz_pages.count();
  ssize_t wb = write(fd, &h, sizeof(h));
  assert(wb == sizeof(h));

  for(size_t i = nz_pages.find_first(); i != boost::dynamic_bitset<>::npos;
      i = nz_pages.find_next(i)) {
    page p;
    p.va = i*4096;
    memcpy(p.data, s.mem+p.va, 4096);
    wb = write(fd, &p, sizeof(p));
    assert(wb == sizeof(p));
  }
  close(fd);
}

void loadState(state_t &s, const std::string &filename) {
  header h;
  int fd = ::open(filename.c_str(), O_RDONLY, 0600);
  assert(fd != -1);
  size_t sz = read(fd, &h, sizeof(h));
  assert(sz == sizeof(h));
  assert(h.magic == MAGICNUM);
  s.pc = h.pc;
  memcpy(&s.gpr,&h.gpr,sizeof(s.gpr));
  s.icnt = h.icnt;
  
  for(uint32_t i = 0; i < h.num_nz_pages; i++) {
    page p;
    sz = read(fd, &p, sizeof(p));
    assert(sz == sizeof(p));
    memcpy(s.mem+p.va, p.data, 4096);
  }
  close(fd);
}
