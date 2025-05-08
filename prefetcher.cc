#include "prefetcher.hh"
#include "interpret.hh"
#include <cassert>

template<typename T>
static T min(const T&a, const T&b) {
  return (a<b) ? a : b;
}

template<typename T>
static T max(const T&a, const T&b) {
  return (a>b) ? a : b;
}


static inline long long translate(long long va, long long root, state_t *s) {
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
  mask_bits = ((u>>63)&1) ? 16 : 12;

 translation_complete:
  int64_t m = ((1L << mask_bits) - 1);
  
  u = ((u >> 10) & ((1UL<<44)-1)) * 4096;
  uint64_t pa = (u&(~m)) | (va & m);
  // printf("translation complete, va %llx -> pa %llx!\n", va, pa);
  //exit(-1);
  return (pa & ((1UL<<32)-1));
}


void global_history_buffer::add(uint64_t pc, uint64_t addr) {
  uint64_t e_idx = (pc>>2) & (itbl_sz-1);
  idx_entry &i = idx_tbl[e_idx];
  ghb_entry &e = ghb[head_idx & (ghb_sz-1)];
  bool h = i.pc == pc;
  
    //printf("pc %lx maps to idx table entry %ld, hit %d, ptr %p, e %p, addr %lx\n",
    //pc, e_idx, static_cast<int>(h), i.ptr, &e, addr);
    
    ++head_idx;

    e.next = h ? i.ptr : nullptr;
    i.pc = pc;
    i.ptr = &e;
    e.pc = pc;
    e.addr = addr;
  }

bool global_history_buffer::lookup_stride(uint64_t pc, uint64_t addr) const {
  uint64_t e_idx = (pc>>2) & (itbl_sz-1);
  idx_entry &i = idx_tbl[e_idx];
  ghb_entry *e = i.ptr;
  bool h = i.pc == pc;
  //printf("lookup pc %lx, hit %d\n", pc, static_cast<int>(h));
  
  if(h == false) {
    return false;
  }
  
  uint64_t last[4] = {0};
  int lp = 0;
  for(int i = 0; i < 4 and (e != nullptr); i++) {
    if(e->pc != pc) {
      break;
    }
    last[lp++] = e->addr;
      //printf("pc %lx, addr %lx\n", e->pc, e->addr);
    e = e->next;
  }
  int64_t s0 = static_cast<int64_t>(last[0]) - static_cast<int64_t>(last[1]);
#if 0
  int64_t s1 = static_cast<int64_t>(last[1]) - static_cast<int64_t>(last[2]);
  if(s0 != s1) {
    printf("pc %lx, s0 = %ld, s1 = %ld\n", pc, s0, s1);
    return false;
  }
#endif
  return ((last[0] + s0) == addr);
}

void linked_list_prefetcher::add(uint64_t pc, uint64_t addr, uint64_t data[2],
				 uint64_t page_table_root, state_t *s) {
  static const uint64_t m = (1UL<<16) - 1;  
  uint64_t e_idx = (pc>>2) & (itbl_sz-1);
  idx_entry &i = idx_tbl[e_idx];
  uint64_t tc[2] = {0};
  int hit_last = -1;

  if(page_table_root) {
    tc[0] = translate(data[0], page_table_root, s);
    tc[1] = translate(data[1], page_table_root, s);
  }
  else {
    tc[0] = data[0];
    tc[1] = data[1];
  }
  tc[0] &= ~15UL;
  tc[1] &= ~15UL;
  
  for(int j = 0; (i.pc == pc) and (j < 2); j++) {
    if(addr == i.last_data[j] ) {
      hit_last = j;
      break;
    }
  }

  if(hit_last != -1) {
    i.confidence++;
    i.confidence = min(10L, i.confidence);
    printf("pc %lx linked list walk with confidence %ld\n",
	   pc, i.confidence);
  }
  else if(i.pc == pc){
    //printf("was %ld\n", i.confidence);
    i.confidence--; 
    i.confidence = (i.confidence < 0) ? 0 : i.confidence;
    //printf("pc %lx linked list miss with confidence %ld\n",
    //pc, i.confidence);    
  }

  if(i.pc != pc) {
    i.confidence = 0;
  }
  i.pc = pc;
  i.last_addr = addr;
  //these need to be translated from virtual addresses to physical addresses
  i.last_data[0] = tc[0];
  i.last_data[1] = tc[1];

}
