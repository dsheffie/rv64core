#ifndef __ghbhh__
#define __ghbhh__

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>

class state_t;

struct linked_list_prefetcher {
  struct idx_entry {
    uint64_t pc;
    uint64_t last_addr;
    uint64_t last_data[2];
    int64_t confidence;
  };
  size_t lg_itbl_sz, itbl_sz;
  idx_entry *idx_tbl;
  linked_list_prefetcher(size_t lg_itbl_sz) :
    lg_itbl_sz(lg_itbl_sz), itbl_sz(1UL<<lg_itbl_sz) {
    idx_tbl = new idx_entry[itbl_sz];
  }
  ~linked_list_prefetcher() {
    delete [] idx_tbl;
  }
  void add(uint64_t pc, uint64_t addr, uint64_t data[2], uint64_t page_table_root, state_t *s);
};

struct global_history_buffer {
  struct ghb_entry {
    uint64_t addr, pc;
    ghb_entry *next;
  };
  struct idx_entry {
    uint64_t pc;
    ghb_entry *ptr;
  };
  uint64_t head_idx;
  size_t lg_itbl_sz, lg_ghb_sz, itbl_sz, ghb_sz;
  
  idx_entry *idx_tbl;
  ghb_entry *ghb;
  
  
  global_history_buffer(size_t lg_itbl_sz, size_t lg_ghb_sz) : 
    head_idx(0), lg_itbl_sz(lg_itbl_sz), lg_ghb_sz(lg_ghb_sz),
    itbl_sz(1UL<<lg_itbl_sz), ghb_sz(1UL<<lg_ghb_sz) {
    idx_tbl = new idx_entry[itbl_sz];
    ghb = new ghb_entry[ghb_sz];
    memset(idx_tbl, 0, sizeof(idx_entry)*itbl_sz);
    memset(ghb, 0, sizeof(ghb_entry)*ghb_sz);
  }
  
  ~global_history_buffer() {
    delete [] idx_tbl;
    delete [] ghb;
  }

  void add(uint64_t pc, uint64_t addr);
  bool lookup_stride(uint64_t pc, uint64_t addr) const;

  
};


#endif
