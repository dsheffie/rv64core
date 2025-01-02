#ifndef __tophh__
#define __tophh__

#include <cstdint>
#include <cstdlib>
#include <cstdint>
#include <vector>
#include <cmath>
#include <tuple>
#include <map>

#include <sys/time.h>
#include <boost/version.hpp>

#if BOOST_VERSION >= 107400
#include <boost/serialization/library_version_type.hpp>
#endif

#include <boost/program_options.hpp>
#include <boost/dynamic_bitset.hpp>

#include <sys/mman.h>
#include <unistd.h>
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>
#include <fenv.h>
#include <verilated.h>
#include "Vcore_l1d_l1i.h"
#include "loadelf.hh"
#include "helper.hh"
#include "interpret.hh"
#include "globals.hh"
#include "disassemble.hh"
#include "saveState.hh"
#include "pipeline_record.hh"

#include "Vcore_l1d_l1i__Dpi.h"
#include "svdpi.h"

template <typename A, typename B>
inline double histo_mean_median(const std::map<A,B> &histo, A &median) {
  double acc = 0.0;
  B count = 0, x = 0;
  if(histo.size() == 0) {
    median = 0;
    return 0.0;
  }
  
  for(const auto &p : histo) {
    acc += (p.first * p.second);
    count += p.second;
  }

  acc /= count;
  for(const auto &p : histo) {
    x += p.second;
    if(x >= (count/2)) {
      median = p.first;
      break;
    }
  }
  return acc;
}



union itype {
  struct {
    uint32_t imm : 16;
    uint32_t rs : 5;
    uint32_t rt : 5;
    uint32_t op : 6;
  } uu;
  uint32_t u;
};

union rtype {
  struct {
    uint32_t subop : 6;
    uint32_t z : 5;
    uint32_t rd : 5;
    uint32_t rt : 5;
    uint32_t rs : 5;
    uint32_t op : 6;
  } uu;
  uint32_t u;
};

union ceqs {
  struct {
    uint32_t fpop : 6;
    uint32_t zeros : 2;
    uint32_t cc : 3;
    uint32_t fs : 5;
    uint32_t ft : 5;
    uint32_t fmt : 5;
    uint32_t opcode : 6;
  } uu;
  uint32_t u;
  ceqs(uint32_t fs, uint32_t ft, uint32_t cc) {
    uu.fpop = 50;
    uu.zeros = 0;
    uu.cc = cc;
    uu.fs = fs;
    uu.ft = ft;
    uu.fmt = 16;
    uu.opcode = 17;
  }  
};

union mtc1 {
  struct {
    uint32_t zeros : 11;
    uint32_t fs : 5;
    uint32_t rt : 5;
    uint32_t mt : 5;
    uint32_t opcode : 6;
  } uu;
  uint32_t u;
  mtc1(uint32_t fs, uint32_t rt) {
    uu.opcode = 17;
    uu.mt = 4;
    uu.zeros = 0;
    uu.fs = fs;
    uu.rt = rt;
  }
};


union mthi {
  struct {
    uint32_t secondary_opcode : 6;
    uint32_t zeros : 15;
    uint32_t rs : 5;
    uint32_t primary_opcode : 6;
  } uu;
  uint32_t u;
  mthi(uint32_t rs) {
    uu.primary_opcode = 0;
    uu.rs = rs;
    uu.zeros = 0;
    uu.secondary_opcode = 17;
  }
};

union mtlo {
  struct {
    uint32_t secondary_opcode : 6;
    uint32_t zeros : 15;
    uint32_t rs : 5;
    uint32_t primary_opcode : 6;
  } uu;
  uint32_t u;
  mtlo(uint32_t rs) {
    uu.primary_opcode = 0;
    uu.rs = rs;
    uu.zeros = 0;
    uu.secondary_opcode = 19;
  }
};



struct dbl {
  uint64_t f : 52;
  uint64_t e : 11;
  uint64_t s : 1;
} __attribute__((packed));

union double_ {
  static const uint32_t bias = 1023;
  dbl dd;
  double d;
  double_(double x) : d(x) {
    static_assert(sizeof(dbl)==sizeof(uint64_t), "bad size");
  };
};

template <typename T>
static inline T round_to_alignment(T x, T m) {
  return ((x+m-1) / m) * m;
}

static inline uint32_t get_insn(uint64_t pc, const state_t *s) {
  return *reinterpret_cast<uint32_t*>(&s->mem[pc]);
}


template<typename X, typename Y>
static inline void dump_histo(const std::string &fname,
			      const std::map<X,Y> &histo,
			      const state_t *s) {
  std::vector<std::pair<Y,X>> sorted_by_cnt;
  for(auto &p : histo) {
    sorted_by_cnt.emplace_back(p.second, p.first);
  }
  std::ofstream out(fname);
  std::sort(sorted_by_cnt.begin(), sorted_by_cnt.end());
  for(auto it = sorted_by_cnt.rbegin(), E = sorted_by_cnt.rend(); it != E; ++it) {
    auto pc = it->second;
    if(pc >= (1UL<<32))
      continue;
    uint32_t r_inst = *reinterpret_cast<uint32_t*>(&s->mem[pc]);
    auto s = getAsmString(r_inst, it->second);
    out << std::hex << it->second << ":"
  	      << s << ","
  	      << std::dec << it->first << "\n";
  }
  out.close();
}

static inline void dump_tip(const std::string &fname,
			    const std::map<int64_t, double> &tip,
			    const std::map<int64_t, uint64_t> &icnt,
			    const state_t *s) {
  std::vector<std::pair<double,int64_t>> sorted_by_cnt;
  for(auto &p : tip) {
    sorted_by_cnt.emplace_back(p.second, p.first);
  }
  std::cout << "tip.size() = " << tip.size() << "\n";
  std::cout << "icnt.size() = " << icnt.size() << "\n";
  std::ofstream out(fname);
  std::sort(sorted_by_cnt.begin(), sorted_by_cnt.end());
  for(auto it = sorted_by_cnt.rbegin(), E = sorted_by_cnt.rend(); it != E; ++it) {
    auto pc = it->second;
    if(pc >= (1UL<<32))
      continue;
    uint32_t r_inst = *reinterpret_cast<uint32_t*>(&s->mem[pc]);
    auto s = getAsmString(r_inst, it->second);
    double avg = 0.0;
    if(icnt.find(it->second) != icnt.end()) {
      avg = (it->first) / icnt.at(it->second);
    }
    out << std::hex << it->second << ":"
	<< s << ","
	<< std::dec 
	<< (it->first) << ","
	<< avg
	<< "\n";
  }
  out.close();
}

template<typename X, typename Y>
static inline void dump_histo(const std::string &fname,
			      const std::map<X,Y> &histo) {
  std::vector<std::pair<Y,X>> sorted_by_cnt;
  for(auto &p : histo) {
    sorted_by_cnt.emplace_back(p.second, p.first);
  }
  std::ofstream out(fname);
  std::sort(sorted_by_cnt.begin(), sorted_by_cnt.end());
  for(auto it = sorted_by_cnt.rbegin(), E = sorted_by_cnt.rend(); it != E; ++it) {
    out << std::hex << it->second << "," << std::dec << it->first << "\n";
  }
  out.close();
}

static inline uint32_t to_uint32(float f) {
  return *reinterpret_cast<uint32_t*>(&f);
}

static inline uint64_t to_uint64(double d) {
  return *reinterpret_cast<uint64_t*>(&d);
}

static inline float to_float(uint32_t u) {
  return *reinterpret_cast<float*>(&u);
}

static inline double to_double(uint64_t u) {
  return *reinterpret_cast<double*>(&u);
}

static inline uint32_t mem_r32(const state_t*s, uint64_t ea) {
  assert(ea < (1UL<<32));
  return *reinterpret_cast<uint32_t*>(&s->mem[ea]);
}

static inline uint64_t mem_r64(const state_t*s, uint64_t ea) {
  assert(ea < (1UL<<32));
  return *reinterpret_cast<uint64_t*>(&s->mem[ea]);
}

static inline void mem_w32(state_t*s, uint64_t ea, uint32_t x) {
  assert(ea < (1UL<<32));  
  *reinterpret_cast<uint32_t*>(&s->mem[ea]) = x;
}

static inline void mem_w64(state_t*s, uint64_t ea, uint64_t x) {
  assert(ea < (1UL<<32));
  *reinterpret_cast<uint64_t*>(&s->mem[ea]) = x;
}

static inline uint8_t *mmap4G() {
  #ifdef __linux__
  void* mempt = mmap(nullptr, 1UL<<32, PROT_READ | PROT_WRITE,
#ifdef __amd64__
		     (21 << MAP_HUGE_SHIFT) |
#endif
		     MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
#else
  void* mempt = mmap(nullptr, 1UL<<32, PROT_READ | PROT_WRITE,
		     MAP_PRIVATE | MAP_ANONYMOUS , -1, 0);
#endif
  assert(mempt != reinterpret_cast<void*>(-1));
  assert(madvise(mempt, 1UL<<32, MADV_DONTNEED)==0);
  
  return reinterpret_cast<uint8_t*>(mempt);
}

static inline
void reset_core(Vcore_l1d_l1i *tb, uint64_t &cycle,
		uint64_t init_pc) {
  for(; (cycle < 4) && !Verilated::gotFinish(); ++cycle) {
    tb->mem_rsp_valid = 0;
    tb->monitor_ack = 0;
    tb->reset = 1;
    tb->extern_irq = 0;
    tb->clk = 1;
    tb->eval();
    tb->clk = 0;
    tb->eval();
    ++cycle;
  }
  //deassert reset
  tb->reset = 0;
  tb->clk = 1;
  tb->eval();
  tb->clk = 0;
  tb->eval();

  tb->resume_pc = init_pc;
  while(!tb->ready_for_resume) {
    ++cycle;  
    tb->clk = 1;
    tb->eval();
    tb->clk = 0;
    tb->eval();
  }
  
  ++cycle;
  tb->resume = 1;

  tb->clk = 1;
  tb->eval();
  tb->clk = 0;
  tb->eval();
  
  ++cycle;  
  tb->resume = 0;
  tb->clk = 1;
  tb->eval();
  tb->clk = 0;
  tb->eval();
}

static state_t *s = nullptr;

static inline long long translate(long long va, long long root, bool iside, bool store) {
  uint64_t a = 0, u = 0;
  int mask_bits = -1;
  a = root + (((va >> 30) & 511)*8);
  u = *reinterpret_cast<int64_t*>(s->mem + a);
  //printf("1st level entry %lx\n", a);
  if((u & 1) == 0) {
    return (~0UL);
  }
  if((u>>1)&7) {
    mask_bits = 30;
    goto translation_complete;
  }

  //2nd level walk
  root = ((u >> 10) & ((1UL<<44)-1)) * 4096;
  a = root + (((va >> 21) & 511)*8);
  u = *reinterpret_cast<int64_t*>(s->mem + a);
  //printf("2nd level entry %lx\n", a);  
  if((u & 1) == 0) {
    return (~0UL);
  }
  if((u>>1)&7) {
    mask_bits = 21;
    goto translation_complete;
  }
  
  //3rd level walk
  root = ((u >> 10) & ((1UL<<44)-1)) * 4096;  
  a = root + (((va >> 12) & 511)*8);
  //printf("3rd level entry %lx\n", a);
  u = *reinterpret_cast<int64_t*>(s->mem + a);
  if((u & 1) == 0) {
    return (~0UL);
  }
  assert((u>>1)&7);
  mask_bits = 12;

 translation_complete:
  int64_t m = ((1L << mask_bits) - 1);

  /* accessed bit */
  bool accessed = ((u >> 6) & 1);
  bool dirty = ((u >> 7) & 1);
  if(!accessed) {
    u |= 1 << 6;
    *reinterpret_cast<int64_t*>(s->mem + a) = u;
  }

  if(store and not(dirty)) {
    u |= 1<<7;
    *reinterpret_cast<int64_t*>(s->mem + a) = u;    
  }
  
  u = ((u >> 10) & ((1UL<<44)-1)) * 4096;
  uint64_t pa = (u&(~m)) | (va & m);
  // printf("translation complete, va %llx -> pa %llx!\n", va, pa);
  //exit(-1);
  return (pa & ((1UL<<32)-1));
}

static const char* l1d_stall_str[7] = {
  "wasnt idle or hit", //0
  "full memory queue", //1
  "read retry", //2
  "store to same set", //3
  "tlb miss", //4
  "cm block", //5
  "rob ptr inflight"
};


#endif
