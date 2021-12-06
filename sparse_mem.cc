#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>

#ifdef __amd64__
#include <x86intrin.h>
#endif

#include "sparse_mem.hh"


#define PROT (PROT_READ | PROT_WRITE)
#define MAP (MAP_ANONYMOUS|MAP_PRIVATE|MAP_POPULATE)

sparse_mem::sparse_mem() {
  mem = reinterpret_cast<uint8_t*>(mmap(nullptr, sparse_mem::sz, PROT, MAP, -1, 0));
  assert(mem != reinterpret_cast<uint8_t*>(~0UL));
  memset(mem, 0, sparse_mem::sz);
}

sparse_mem::~sparse_mem() {
  munmap(mem, sz);
  mem = nullptr;
}

void sparse_mem::clear() {
  
}

#if 0
uint8_t *sparse_mem::add_page(uint64_t addr) {
  uint64_t paddr = addr / pgsize;
  uint8_t *pg = nullptr;
  //std::cerr << "ADDING PAGE " << std::hex << (paddr * pgsize) << std::dec << "\n";
  pg = reinterpret_cast<uint8_t*>(mmap(nullptr, pgsize, PROT, MAP, -1, 0));
  assert(pg != reinterpret_cast<uint8_t*>(~0UL));
  allocations[pg] = pgsize;
  memset(pg,0x0,pgsize);    
  smem[paddr] = pg;
  return pg;
}  

uint8_t* sparse_mem::get_page(uint64_t addr) {
  uint64_t paddr = addr / pgsize;
  //std::cerr << "trying to find page " << std::hex << addr << std::dec << "\n";
  auto it = smem.find(paddr);
  if(it == smem.end()) {
    return add_page(addr);
  }
  return it->second;
}

void sparse_mem::coalesce(uint64_t addr, uint64_t sz) {
  uint64_t pstart = addr / pgsize, eaddr = (addr + sz);
  uint64_t pstop = (eaddr + pgsize - 1) / pgsize;
  bool need_coalesce = false;
  //std::cout << "start = " << pstart*pgsize << "\n";
  //std::cout << "stop = " << pstop*pgsize << "\n";
  for(uint64_t paddr = pstart+1; paddr < pstop; ++paddr) {
    if(smem[paddr] != (smem[paddr-1] + pgsize)) {
      need_coalesce = true;
      std::cout << "need to coalesce\n";
      break;
    }
  }
  if(!need_coalesce) {
    return;
  }
}

void sparse_mem::prefault(uint64_t addr, uint64_t alloc_size) {
  uint64_t paddr = addr / pgsize, eaddr = (addr + alloc_size);
  bool any_present = false;
  
  uint64_t pstart = paddr, pstop = (eaddr + pgsize - 1) / pgsize;
  //std::cout << "pstart = " << pstart << ", pstop = " << pstop << "\n";

  for(uint64_t paddr = pstart; paddr < pstop; ++paddr) {
    if(smem.find(paddr) != smem.end()) {
      any_present = true;
      break;
    }
  }
  //any_present = true;
  /* create one big mmap */
  if(not(any_present)) {
    uint8_t *pg = nullptr;
    uint64_t allocsz = (pstop-pstart)*pgsize;
    //if(not(isPow2(allocsz))) {
    // allocsz = nextPow2(allocsz);
    //}
    //std::cout << "creating large allocation of "
    ///<< allocsz
    //	      << " bytes "
    //<< " from "
    //<< std::hex
    //<< pstart*pgsize
    //<< " to "
    //<< pstop*pgsize
    //<< "\n";
    
    pg = reinterpret_cast<uint8_t*>(mmap(nullptr, allocsz, PROT, MAP, -1, 0));
    assert(pg != reinterpret_cast<uint8_t*>(~0UL));
    allocations[pg] = allocsz;
    memset(pg,0x0,allocsz);
    for(uint64_t paddr = pstart, c = 0; paddr < pstop; ++paddr, ++c) {
      smem[paddr] = pg + (c*pgsize);
    }
  }
  else {
    /* add each page */
    for(uint64_t paddr = pstart; paddr < pstop; ++paddr) {
      if(smem.find(paddr) == smem.end()) {
	add_page(paddr * pgsize);
      }
    }
  }
}


uint32_t sparse_mem::crc32() const {
  uint32_t c = ~0x0;
#if 0
  //std::cout << present_bitvec.popcount()
  //<< " non-zero pages\n";
  for(size_t i = 0; i < npages; i++) {
#ifdef __amd64__
    if(present_bitvec[i]==false) {
      for(size_t n=0;n<4096;n++) {
	c = _mm_crc32_u8(c, 0);
      }
    }
    else {
      //uint8_t x = 0;
      for(size_t n=0;n<4096;n++) {
	c = _mm_crc32_u8(c, mem[i][n]);
	//x ^= mem[i][n];
      }
      //std::cout << "page " << i << " is non-zero, x = "
      //<< std::hex << static_cast<uint32_t>(x) << std::dec << "\n";
    }
#else
    static const uint32_t POLY = 0x82f63b78;
    for(size_t n=0;n<4096;n++) {
      uint8_t b = present_bitvec[i] ? mem[i][n] : 0;
      c ^= b;
      for(int k = 0; k < 8; k++) {
	c = c & 1 ? (c>>1) ^ POLY : c>>1;
      }
    }
#endif
  }
#endif
  return c ^ (~0x0);
}

void sparse_mem::clear() {
  for(auto o : allocations) {
    munmap(o.first, o.second);
  }
  allocations.clear();
  smem.clear();
}

sparse_mem::~sparse_mem() {
  for(auto o : allocations) {
    munmap(o.first, o.second);
  }
}

#endif


