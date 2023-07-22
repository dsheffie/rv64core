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

static inline uint32_t get_insn(uint32_t pc, const state_t *s) {
  return *reinterpret_cast<uint32_t*>(&s->mem[pc]);
}


template<typename X, typename Y>
static inline void dump_histo(const std::string &fname,
			      const std::map<X,Y> &histo,
			      const state_t *s) {
  std::vector<std::pair<X,Y>> sorted_by_cnt;
  for(auto &p : histo) {
    sorted_by_cnt.emplace_back(p.second, p.first);
  }
  std::ofstream out(fname);
  std::sort(sorted_by_cnt.begin(), sorted_by_cnt.end());
  for(auto it = sorted_by_cnt.rbegin(), E = sorted_by_cnt.rend(); it != E; ++it) {
    uint32_t r_inst = *reinterpret_cast<uint32_t*>(&s->mem[it->second]);
    auto s = getAsmString(r_inst, it->second);
    out << std::hex << it->second << ":"
  	      << s << ","
  	      << std::dec << it->first << "\n";
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

static inline uint32_t mem_r32(const state_t*s, uint32_t a) {
  return *reinterpret_cast<uint32_t*>(&s->mem[a]);
}

static inline void mem_w32(state_t*s, uint32_t a, uint32_t x) {
  *reinterpret_cast<uint32_t*>(&s->mem[a]) = x;
}



#endif
