#include <boost/dynamic_bitset.hpp>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include "interpret.hh"
#include "globals.hh"

struct page {
  uint32_t va;
  uint8_t data[4096];
} __attribute__((packed));


static const uint64_t MAGIC_NUM = 0x6464f5f5beefd005UL;

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
  
  header() : magic(MAGIC_NUM) {}
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
  h.pc = s.pc;
  

  static_assert(sizeof(s.gpr)==sizeof(h.gpr), "mistakes were made");
  memcpy(&h.gpr,&s.gpr,sizeof(s.gpr));

  
  h.icnt = s.icnt;
  h.num_nz_pages = nz_pages.count();
  h.tohost_addr = globals::tohost_addr;
  h.fromhost_addr = globals::fromhost_addr;

  h.priv = s.priv;
  h.mstatus = s.mstatus;
  h.misa = s.misa;
  h.mideleg = s.mideleg;
  h.medeleg = s.medeleg;
  h.mscratch = s.mscratch;
  h.mhartid = s.mhartid;
  h.mtvec = s.mtvec;
  h.mcounteren = s.mcounteren;
  h.mie = s.mie;
  h.mip = s.mip;
  h.mcause = s.mcause;
  h.mepc = s.mepc;
  h.mtval = s.mtval;
  h.sscratch = s.sscratch;
  h.scause = s.scause;
  h.stvec = s.stvec;
  h.sepc = s.sepc;
  h.sip = s.sip;
  h.stval = s.stval;
  h.satp = s.satp;
  h.scounteren = s.scounteren; 
  h.pmpaddr0 = s.pmpaddr0;
  h.pmpaddr1 = s.pmpaddr1;
  h.pmpaddr2 = s.pmpaddr2;
  h.pmpaddr3 = s.pmpaddr3;
  h.pmpcfg0 = s.pmpcfg0;
  h.mtimecmp = s.mtimecmp;
  

  
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
  int fd = ::open(filename.c_str(), O_RDONLY, 0600);
  assert(fd != -1);
  header h;
  size_t sz = read(fd, &h, sizeof(h));
  assert(sz == sizeof(h));
  
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
