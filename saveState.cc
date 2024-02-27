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

static const uint64_t MAGICNUM = 0x64646464beefd00dUL;

struct header {
  uint64_t magic;
  uint64_t pc;
  int64_t gpr[32];
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
  //std::cout << "got magic number of " << std::hex << h.magic << std::dec << "\n";
  //std::cout << "got pc of " << std::hex << h.pc << std::dec << "\n";
  //assert(h.magic == MAGICNUM);
  s.pc = h.pc;
  memcpy(&s.gpr,&h.gpr,sizeof(s.gpr));
  s.icnt = h.icnt;
  
  for(uint32_t i = 0; i < h.num_nz_pages; i++) {
    page p;
    sz = read(fd, &p, sizeof(p));
    //std::cout << "sz = " << sz << "\n";
    assert(sz == sizeof(p));
    memcpy(s.mem+p.va, p.data, 4096);
  }
  close(fd);
}



void emitCodeForInitialRegisterValues(state_t &s, uint64_t pc) {

  int addi = 0, slli = 0;
  for(int i = 1; i < 32; i++) {
    //std::cout << std::hex << s.gpr[i] << std::dec << "\n";
    slli = ((8) << 20) | (i<<15) | 1 << 12 | (i<<7) | 0x13;
    
    uint64_t u = *reinterpret_cast<uint64_t*>(&s.gpr[i]);
    
    for(int j = 56; j >= 0; j-=8) {
      uint8_t v = (u>>j) & 0x0ff;
      if(v) {
	addi = ((v) << 20) | (i<<15) | 0 << 12 | (i<<7) | 0x13;
	*reinterpret_cast<int*>(&s.mem[pc]) = addi;
	pc += 4;	
      }

      if(j != 0) {
	*reinterpret_cast<int*>(&s.mem[pc]) = slli;
	pc += 4;
      }
    }
  }
  *reinterpret_cast<int*>(&s.mem[pc]) = 0x73;
  pc += 4;
  for(int i = 0; i < 128; i++) {
    *reinterpret_cast<int*>(&s.mem[pc]) = 0x13;
    pc += 4;
  }
}
