
#include "top.hh"
#include <queue>
#include <signal.h>
#include <setjmp.h>
#include "inst_record.hh"

#define BRANCH_DEBUG 1
#define CACHE_STATS 1

#define ROB_ENTRIES 64

bool globals::syscall_emu = true;
uint32_t globals::tohost_addr = 0;
uint32_t globals::fromhost_addr = 0;
bool globals::log = false;
bool globals::checker_enable_irqs = false;
std::map<std::string, uint32_t> globals::symtab;

char **globals::sysArgv = nullptr;
int globals::sysArgc = 0;

static uint64_t cycle = 0;
static uint64_t fetch_slots = 0;
static bool trace_retirement = false;
static uint64_t mem_reqs = 0;
static state_t *ss = nullptr;
static uint64_t insns_retired = 0, insns_allocated = 0;
static uint64_t cycles_in_faulted = 0, fetch_stalls = 0, mispred_to_restart_cycles = 0;

static uint64_t pipestart = 0, pipeend = ~(0UL), pipeip = 0, pipeipcnt = 0;



static pipeline_logger *pl = nullptr;

static uint64_t l1d_misses = 0, l1d_insns = 0;

static uint64_t last_retire_cycle = 0, last_retire_pc  = 0;

static std::map<uint64_t, uint64_t> retire_map;

static uint64_t n_fetch[5] = {0};
static uint64_t n_resteer_bubble = 0;
static uint64_t n_fq_full = 0;

static uint64_t n_uq_full[3] = {0};
static uint64_t n_alloc[3] = {0};
static uint64_t n_rdy[3] = {0};

static uint64_t n_int_exec[2] = {0};
static uint64_t n_int2_exec[2] = {0};
static uint64_t n_mem_exec[2] = {0};

static uint64_t q_full[2] = {0};
static uint64_t dq_empty =  0;
static uint64_t uq_full = 0;
static uint64_t n_active = 0;
static uint64_t rob_full = 0;

static uint64_t l1d_reqs = 0;
static uint64_t l1d_acks = 0;
static uint64_t l1d_new_reqs = 0;
static uint64_t l1d_accept = 0;

static uint64_t l1d_stores = 0;

static std::map<int,uint64_t> block_distribution;
static std::map<int,uint64_t> restart_distribution;
static std::map<int,uint64_t> restart_ds_distribution;
static std::map<int,uint64_t> fault_distribution;
static std::map<int,uint64_t> branch_distribution;
static std::map<int,uint64_t> fault_to_restart_distribution;
static std::map<int,uint64_t> mmu_walk_lat;

static uint64_t l1d_stall_reasons[8] = {0};
static bool enable_checker = true, use_checkpoint = false;
static bool pending_fault = false;
static uint64_t fault_start_cycle = 0;

static uint64_t fault_counts[32] = {0};

void record_exception_type(int fault_type) {
  fault_counts[fault_type & 31]++;
}

void record_branches(int n_branches) {
  branch_distribution[n_branches]++;
}

void record_faults(int n_faults) {
  fault_distribution[n_faults]++;
  if(n_faults && not(pending_fault)) {
    pending_fault = true;
    fault_start_cycle = cycle;
  }
}

static long long mispred_cycle = -1L;
void record_exec_mispred(long long curr_cycle) {
  if(mispred_cycle == -1) {
    mispred_cycle = curr_cycle;
  }
}

void record_restart(int cycles, long long curr_cycle) {
  restart_distribution[cycles]++;
  pending_fault = false;

  fault_to_restart_distribution[(cycle - fault_start_cycle)]++;
  fault_start_cycle = 0;
  mispred_to_restart_cycles += static_cast<uint64_t>( curr_cycle -  mispred_cycle );
  mispred_cycle = -1UL;
  //std::cout << "clearing fault took "
  //<< (cycle - fault_start_cycle)
  //<< " cycles\n";
}

void record_ds_restart(int cycles) {
  restart_ds_distribution[cycles]++;
}


uint64_t l1d_block_reason[7] = {0};
  
void record_l1d(int req, int ack, int new_req, int accept, int block) {
  l1d_reqs += req;
  l1d_acks += ack;
  l1d_new_reqs += new_req;
  l1d_accept += accept;
  for(int i = 0; (i < 7) & req & (!new_req); i++) {
    if((block & (1<<i)) == 0) {
      l1d_block_reason[i]++;
      break;
    }
  }
}

uint64_t mem_table[32] = {0};
uint64_t mem_pc_table[32] = {0};
uint64_t mem_addr_table[32] = {0};
bool is_load[ROB_ENTRIES] = {false};
uint64_t n_logged_loads = 0;
uint64_t total_load_lat = 0;

std::map<uint64_t, uint64_t> last_store;

void log_mem_begin(int r, int l, long long c, long long pc, long long vaddr) {
  mem_table[r] = c;
  is_load[r] = l;
  mem_pc_table[r] = pc;
  mem_addr_table[r] = vaddr;
  if(l==0) {
    last_store[vaddr & (~15UL)] = pc;
  }
}

static std::map<uint64_t, uint64_t> store_latency_map;
static std::map<int, uint64_t> mem_lat_map, fp_lat_map, non_mem_lat_map, mispred_lat_map;

void log_store_release(int r, long long c) {
  //assert(is_load[r] == false);
  if(not(is_load[r])) {
    uint64_t cc = c - mem_table[r];
    //printf("store released after %lu cycles\n", cc);
    store_latency_map[cc]++;
  }
}

void log_mem_end(int r, long long c) {
  uint64_t cc = c - mem_table[r];
  mem_lat_map[cc]++;
  if(is_load[r]) {
    ++n_logged_loads;
    total_load_lat += cc;
    // if(cc > 5) {
    //   std::cout << "load at " << std::hex << mem_pc_table[r] << std::dec
    // 		<< " pushes out " << cc << " cycles , last store from "
    // 		<< std::hex << last_store[mem_addr_table[r] & (~15UL)]
    // 		<< std::dec
    // 		<<"\n";
    // }
  }
}


long long loadgpr(int gprid) {
  return s->gpr[gprid];
}

int load_priv() {
  /*printf("initial priv = %d\n", s->priv); */
  return s->priv;
}

int is_satp_armed() {
  return not(s->unpaged_mode());
}

#define LOAD(x) long long load_##x() {/*printf("loading %s with %lx\n", #x, s->x);*/ return s->x;}

LOAD(scounteren);
LOAD(satp);
LOAD(stval);
LOAD(scause);
LOAD(sepc);
LOAD(mcause);
LOAD(mie);
LOAD(mip);
LOAD(mstatus);
LOAD(mtvec);
LOAD(stvec);
LOAD(mcounteren);
LOAD(mideleg);
LOAD(medeleg);
LOAD(mscratch);
LOAD(sscratch);
LOAD(mepc);
LOAD(mtval);
LOAD(icnt);
#undef LOAD

uint64_t schedules[2] = {0};
uint64_t schedules_alloc[2] = {0};

void record_sched(int p) {
  ++schedules[p&1];
}
void record_sched_alloc(int p) {
  ++schedules_alloc[p&1];
}


void csr_putchar(char c) {
  if(c==0) std::cout << "\n";
  else std::cout << c;
}
void mark_accessed_checker(long long pa) {
  if(not(enable_checker)) {
    return;
  }
  uint64_t pte = ss->load64(pa);
  pte |= (1UL<<6);
  ss->store64(pa, pte);
}

void check_translation(long long addr, int paddr) {
#if 0
  if(not(enable_checker)) {
    return;
  }
  int fault = 0;
  uint64_t pa = ss->page_lookup(addr, fault, 1, false);
  uint32_t upa = paddr;
  if(!fault) {
    pa &= ((1UL<<32)-1);
    if(pa != upa) {
      printf("---> %d va %lx, sw %lx, hw %lx, delta %lx\n",
             (pa==upa),
             addr,
             pa, upa,
             (pa^upa));
    }
  }
#endif
}


long long dc_ld_translate(long long va, long long root) {
  return translate(va,root, false, false);
}

long long ic_translate(long long va, long long root) {
  return translate(va,root,true, false);
}

uint64_t page_table_root = ~0UL;
std::list<store_rec> store_queue;
std::list<store_rec> atomic_queue;

int checker_irq_enabled() {
  return globals::checker_enable_irqs;
}

void start_log(int l) {
  trace_retirement |= (l!=0);
}

static std::map<int, uint64_t> l1_to_l2_dist;
static std::map<int, uint64_t> l2_state;

void l1_to_l2_queue_occupancy(int e) {
  l1_to_l2_dist[e]++;
}
void record_l2_state(int s) {
  l2_state[s]++;
}

static std::list<long long> inflight_uuids;

void alloc_uuid(long long uuid) {
  inflight_uuids.push_back(uuid);
}

void retire_uuid(long long uuid) {
  assert(not(inflight_uuids.empty()));

  do {
    uint64_t head = inflight_uuids.front();
    inflight_uuids.pop_front();
    if(head == uuid) {
      break;
    }
  } while(true);
}

void log_mmu_walk_lat(long long walk_lat, char hit_lvl) {
  mmu_walk_lat[walk_lat]++;
  //  std::cout << "mmu walk took " << walk_lat
  //<< " cycles with hit at "
  //<< static_cast<int>(hit_lvl)
  //<< " in the page table\n";
}

static uint64_t log_l1d_accesses = 0;
static uint64_t log_l1d_push_miss = 0;
static uint64_t log_l1d_push_miss_hit_inflight = 0;
static uint64_t log_l1d_early_reqs = 0;
static uint64_t log_l1d_is_ld_hit = 0;
static uint64_t log_l1d_is_st_hit = 0;
static uint64_t log_l1d_is_hit_under_miss = 0;

static uint64_t log_l1d_misses = 0;
static uint64_t log_l1d_dirty_misses = 0;

void log_l1d_miss(int is_dirty) {
  ++log_l1d_misses;
  if(is_dirty) {
    ++log_l1d_dirty_misses;
  }
}


void log_l1d(int is_early_req,
	     int is_push_miss,
	     int is_push_miss_hit_inflight,
	     int is_st_hit,
	     int is_ld_hit,
	     int is_hit_under_miss) {
  log_l1d_accesses++;
  if(is_push_miss) {
    log_l1d_push_miss++;
  }
  if(is_push_miss_hit_inflight) {
    log_l1d_push_miss_hit_inflight++;
  }
  if(is_early_req) {
    log_l1d_early_reqs++;
  }
  if(is_st_hit) {
    log_l1d_is_st_hit++;
  }
  if(is_ld_hit) {
    log_l1d_is_ld_hit++;
  }
  if(is_hit_under_miss) {
    log_l1d_is_hit_under_miss++;
  }
  
}

void wr_log(long long pc,
	    int robptr,
	    unsigned long long addr,
	    unsigned long long data,
	    int is_atomic) {
  if(not(enable_checker))
    return;

  
  if(globals::log) {
    printf("pc %llx, addr %llx, data %llx, atomic %d, store queue entries %d\n",
	   pc, addr, data, is_atomic,
	   static_cast<int>(store_queue.size()));
  }
  
  if(is_atomic) {
    atomic_queue.emplace_back(pc, addr, data);
    return;
  }
  assert(not(store_queue.empty()));

  auto &t = store_queue.front();
  if(globals::log) {
    std::cout << "check store : sim pc " << std::hex << t.pc << ", rtl pc " << pc
	      << t.addr << ", " << t.data << std::dec << "\n";
  }
  if(not(t.pc == pc and t.addr == addr and t.data == data)) {
    printf("you have a store error! for an atomic %d, pc match %d, addr match %d, data match %d, rob ptr %d\n",
	   is_atomic, t.pc==pc, t.addr==addr, t.data == data, robptr);
    printf("sim pc %lx, rtl pc %llx sim addr %lx, sim data %lx, rtl addr %llx, rtl data %llx, addr xor %llx\n",
	   t.pc, pc, t.addr, t.data, addr, data, t.addr ^ addr);
    exit(-1);
  }
  store_queue.pop_front();
}

int check_bad_fetch(long long pc, long long rtl_pa, int insn) {
  assert(page_table_root != (~0UL));
  uint64_t pa = translate(pc, page_table_root, false, false);
  if (pa==(~0UL)) return 0;
  rtl_pa &= ~4095UL;
  rtl_pa |= (pc & 4095);
  uint32_t u = *reinterpret_cast<uint32_t*>(s->mem + pa);
  bool match = u == *reinterpret_cast<uint32_t*>(&insn);

  std::cout << "check : pc " << std::hex
	    << pc << ", pa " << pa
	    << ", rtl pa " << rtl_pa
	    << match << ", sim "
	    << u <<", rtl "
	    << *reinterpret_cast<uint32_t*>(&insn)
	    << std::dec << "\n";
    
  return not(match);
}


long long ld_translate(long long va) {
  return translate(va, page_table_root, false, false);
}

uint64_t csr_time = 0;

void csr_puttime(long long t) {
  csr_time = t;
}

bool kernel_panicd = false;
void term_sim() {
  kernel_panicd = true;
}


std::string getAsmString(uint64_t addr, uint64_t root, bool paging_enabled) {
  int64_t pa = addr;
  if(paging_enabled) {
    pa = translate(addr, root, true, false);
    if(pa == -1) {
      return "code page not present";
    }
  }
  return getAsmString(mem_r32(s,pa), addr);
}

bool verbose = false;

long long read_dword(long long addr) {
  int64_t pa = addr;
  pa &= ((1UL<<32)-1);
  long long x = *reinterpret_cast<long long*>(s->mem + pa);
  //std::cout << std::hex << addr << " -> " << x << std::hex << "\n";
  return x;
}

long long ic_read_dword(long long addr) {
  return read_dword(addr);
}

int read_word(long long addr) {
  int64_t pa = addr;
  pa &= ((1UL<<32)-1);
  return *reinterpret_cast<int*>(s->mem + pa);
}

void write_byte(long long addr, char data, long long root) {
  int64_t pa = addr;
  //printf("%s:%lx:%lx\n", __PRETTY_FUNCTION__, addr, root);  
  if(root) {
    pa = translate(addr, root, false, true);
    //printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }  
  uint8_t d = *reinterpret_cast<uint8_t*>(&data);
  *reinterpret_cast<uint8_t*>(s->mem + pa) = d;  
}

void write_half(long long addr, short data, long long root) {
  int64_t pa = addr;
  //printf("%s:%lx:%lx\n", __PRETTY_FUNCTION__, addr, root);  
  if(root) {
    pa = translate(addr, root, false, true);
    ///printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }  
  uint16_t d = *reinterpret_cast<uint16_t*>(&data);
  *reinterpret_cast<uint16_t*>(s->mem + pa) = d;  

}

void write_word(long long addr, int data, long long root, int id) {
  int64_t pa = addr;
  //printf("%s:%lx:%lx\n", __PRETTY_FUNCTION__, addr, root);  
  if(root) {
    pa = translate(addr, root, false, true);
    //printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }  
  uint32_t d = *reinterpret_cast<uint32_t*>(&data);
  *reinterpret_cast<uint32_t*>(s->mem + pa) = d;
}

void write_dword(long long addr, long long data, long long root, int id) {
  int64_t pa = addr;

  if(root) {
    pa = translate(addr, root, false, true);
    //printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }
  uint64_t d = *reinterpret_cast<uint64_t*>(&data);
  *reinterpret_cast<uint64_t*>(s->mem + pa) = d;
}

static std::map<int, uint64_t> int_sched_rdy_map;

void report_exec(int int_valid, int int_ready,
		 int int2_valid, int int2_ready,		 
		 int mem_valid, int mem_ready,
		 int fp_valid,  int fp_ready,
		 int intq_full, int memq_full,
		 int fpq_full,
		 int blocked_by_store,
		 int ready_int,
		 int ready_int2) {
  n_int_exec[0] += int_valid;
  n_int_exec[1] += int_ready;
  n_int2_exec[0] += int2_valid;
  n_int2_exec[1] += int2_ready;  
  n_mem_exec[0] += mem_valid;
  n_mem_exec[1] += mem_ready;
    
  q_full[0] += intq_full;
  q_full[1] += memq_full;


  int total_ready = __builtin_popcount(ready_int) +
    __builtin_popcount(ready_int2);
  int_sched_rdy_map[total_ready]++;
}


void record_alloc(int rf,
		  int a1, int a2, int de,
		  int f1, int f2,
		  int r1, int r2, int active) {

  rob_full += rf;
  dq_empty += de;
  uq_full += f1;
  n_active += active;
  
  if(a2)
    ++n_alloc[2];
  else if(a1)
    ++n_alloc[1];
  else
    ++n_alloc[0];

  if(f2)
    ++n_uq_full[2];
  else if(f1)
    ++n_uq_full[1];
  else
    ++n_uq_full[0];
  
  if(r2) {
    ++n_rdy[2];
    fetch_slots += 2;
  }
  else if(r1) {
    ++n_rdy[1];
    fetch_slots += 1;
  }
  else {
    ++n_rdy[0];
  }
  
}


void record_fetch(int p1, int p2, int p3, int p4, 
		  long long pc1, long long pc2, long long pc3, long long pc4,
		  int bubble, int fq_full) {
  n_resteer_bubble += bubble;
  n_fq_full += fq_full;
  
  if(p1)
    ++n_fetch[1];
  else if(p2)
    ++n_fetch[2];
  else if(p3)
    ++n_fetch[3];
  else if(p4)
    ++n_fetch[4];
  else
    ++n_fetch[0];
}



int check_insn_bytes(long long pc, int data) {
  uint32_t insn = get_insn(pc, s);
  return (*reinterpret_cast<uint32_t*>(&data)) == insn;
}

static long long lrc = -1;
static uint64_t record_insns_retired = 0;

static int pl_regs[32] = {0};

struct pt_ {
  static const uint64_t NI = (~0UL);
  uint64_t pc;
  uint64_t fetch;
  uint64_t alloc;
  uint64_t sched;
  uint64_t complete;
  uint64_t retire;
  uint64_t l1d_p1_hit;
  uint64_t l1d_p1_miss;
  uint64_t l1d_replay;
  bool maybe_alias;
  std::list<uint64_t> l1d_blocks;
  std::list<uint64_t> l1d_sd;
  void clear() {
    fetch = alloc = sched = NI;
    complete = retire = NI;
    maybe_alias = false;
    l1d_p1_hit = l1d_p1_miss = l1d_replay = NI;
    l1d_blocks.clear();
    l1d_sd.clear();
  }
};

static pt_ records[64];

static std::ostream &operator<<(std::ostream &out, const pt_ &r) {
  out << std::hex << r.pc << std::dec 
      << r.fetch << "," << r.alloc << "," << r.sched << "," << r.complete << ","
      << r.retire;
  return out;
}

void pt_alloc(long long pc, long long fetch_cycle, long long alloc_cycle, int rob_id) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.clear();
    r.pc = pc;
    r.fetch = fetch_cycle;
    r.alloc = r.sched = r.complete = alloc_cycle;
  }
}

void pt_sched(long long cycle, int rob_id) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.sched = cycle;
  }
}

void pt_l1d_pass1_hit(long long cycle, int rob_id) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.l1d_p1_hit = cycle;
  }
}

void pt_l1d_pass1_miss(long long cycle, int rob_id, int maybe_alias,
		       unsigned long long va,
		       unsigned long long pa) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.l1d_p1_miss = cycle;
    r.maybe_alias = maybe_alias;
    // if(maybe_alias) {
    //   std::cout << "op at " << std::hex << r.pc
    // 		<< std::dec
    // 		<<" could alias\n";
    //   uint64_t m = (1UL<<14)-1;
    //   std::cout << "va = " << std::hex << (va & m) << " , pa "
    // 		<<  (pa & m) << std::dec << "\n";
    // }
  }
}

void pt_l1d_blocked(long long cycle, int rob_id) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.l1d_blocks.push_back(cycle);
  }
}

void pt_l1d_replay(long long cycle, int rob_id) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.l1d_replay = cycle;
  }
}

void pt_l1d_store_data_ready(long long cycle, int rob_id) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.l1d_sd.push_back(cycle);
  }
}

void pt_complete(long long cycle, int rob_id) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.complete = cycle;
  }
}


void pt_retire(long long cycle,
	       int rob_id,
	       int paging_active,
	       long long page_table_root	       
	       ) {
  if(pl != nullptr) {
    auto &r = records[rob_id & (ROB_ENTRIES-1)];
    r.retire = cycle;
    if((pipeip != 0)) {
      if(r.pc ==  pipeip) {
	--pipeipcnt;
	//printf("hit token ip, count %lu\n", pipeipcnt);      
	if(pipeipcnt == 0) {
	  pipestart = record_insns_retired;
	  pipeend = record_insns_retired + 4096;
	  std::cout << "trace " << std::hex << pipeip << " hit enough times : starts at " 
		    << std::dec << pipestart << ", will end at "
		    << pipeend << "\n";
	  pipeip = 0;
	}
      }
    }
    if((record_insns_retired >= pipestart) and (record_insns_retired < pipeend)) {
      uint32_t insn = get_insn(r.pc, s);
      uint32_t opcode = insn & 127;
      auto disasm = getAsmString(r.pc, page_table_root, paging_active);
      riscv_t m(insn);
      if(opcode == 0x3 ) {
	std::stringstream ss;
	int32_t disp = m.l.imm11_0;
	if((insn>>31)&1) {
	  disp |= 0xfffff000;
	}
	uint32_t ea = disp + pl_regs[m.l.rs1];
	ss << std::hex << ea << std::dec;
	disasm += " EA :  " + ss.str();
      }
      else if(opcode == 0x23) {
	std::stringstream ss;
	int32_t disp = m.s.imm4_0 | (m.s.imm11_5 << 5);
	disp |= ((insn>>31)&1) ? 0xfffff000 : 0x0;
	uint32_t ea = disp + pl_regs[m.s.rs1];
	ss << std::hex << ea << std::dec;
	disasm += " EA :  " + ss.str();
      }
      pl->append(record_insns_retired,
		 disasm,
		 r.pc,
		 r.fetch,
		 r.alloc,
		 r.sched,
		 r.complete,
		 r.retire,
		 r.l1d_p1_hit,
		 r.l1d_p1_miss,
		 r.l1d_replay,
		 r.l1d_blocks,
		 r.l1d_sd,
		 false);
    }
  }
  ++record_insns_retired;  
}
		       


void record_retirement(long long pc,
		       long long fetch_cycle,
		       long long alloc_cycle,
		       long long complete_cycle,
		       long long retire_cycle,
		       int retire_reg_val,
		       int retire_reg_ptr,
		       long long retire_reg_data,
		       int faulted ,
		       int br_mispredict,
		       int paging_active,
		       long long page_table_root
		       ) {

  uint32_t insn = get_insn(pc, s);
  uint64_t delta = retire_cycle - last_retire_cycle;
  
  if(retire_reg_val) {
    pl_regs[retire_reg_ptr & 31] = retire_reg_data;
  }
  
  if(retire_cycle < lrc) {
    std::cout << "retirement cycle out-of-order\n";
    std::cout << "lrc = " << lrc << "\n";
    std::cout << "retire_cycle = " << retire_cycle << "\n";
    exit(-1);
  }
  lrc = retire_cycle;

  if(br_mispredict) {
    //long long t = retire_cycle - alloc_cycle;
    //long long tt = retire_cycle - fetch_cycle;
    //std::cout << "mispredict at " << std::hex << pc << std::dec << " took " << t
    //<< " cycles from alloc to retire and "
    //<< tt << " cycles from fetch to retire\n";
    mispred_lat_map[complete_cycle-alloc_cycle]++;
  }
  
  retire_map[delta]++;
  
  last_retire_cycle = retire_cycle;
  last_retire_pc = pc;
  

}


static int buildArgcArgv(const char *filename, const char *sysArgs, char ***argv) {
  int cnt = 0;
  std::vector<std::string> args;
  char **largs = 0;
  args.push_back(std::string(filename));

  char *ptr = nullptr, *sa = nullptr;
  if(sysArgs) {
    sa = strdup(sysArgs);
    ptr = strtok(sa, " ");
  }

  while(ptr && (cnt<MARGS)) {
    args.push_back(std::string(ptr));
    ptr = strtok(nullptr, " ");
    cnt++;
  }
  largs = new char*[args.size()];
  for(size_t i = 0; i < args.size(); i++) {
    std::string s = args[i];
    size_t l = strlen(s.c_str());
    largs[i] = new char[l+1];
    memset(largs[i],0,sizeof(char)*(l+1));
    memcpy(largs[i],s.c_str(),sizeof(char)*l);
  }
  *argv = largs;
  if(sysArgs) {
    free(sa);
  }
  return (int)args.size();
}

static uint64_t port1_active = 0, port2_active = 0;

void l1d_port_util(int port1, int port2) {
  if(port1) {
    ++port1_active;
  }
  if(port2) {
    ++port2_active;
  }
}

static uint64_t last_retired_pc = 0;
static uint64_t last_insns_retired = 0, last_cycle = 0;
static Vcore_l1d_l1i *tb = nullptr;
void catchUnixSignal(int n) {
  std::cout << std::hex << "last_retired_pc = " << last_retired_pc
	    << std::dec << ", last_insns_retired = " << last_insns_retired
	    << ", last_cycle = " << last_cycle << "\n";
  if(tb) {
    printf("core_state = %d\n", (int)tb->core_state);
    printf("l1i_state  = %d\n", (int)tb->l1i_state);
    printf("l1d_state  = %d\n", (int)tb->l1d_state);
    printf("l2_state   = %d\n", (int)tb->l2_state);
    printf("mmu_state  = %d\n", (int)tb->mmu_state);        
    
  }
  exit(-1);
}

struct mem_req_t {
  bool is_store;
  uint32_t addr;
  uint32_t tag;
  uint64_t reply_cycle;
  uint32_t data[4] = {0};
  mem_req_t(bool is_store, uint32_t addr,
	    uint32_t tag, uint64_t reply_cycle) :
    is_store(is_store), addr(addr), tag(tag),
    reply_cycle(reply_cycle) {}
};

std::queue<mem_req_t> mem_queue;

int main(int argc, char **argv) {
  static_assert(sizeof(itype) == 4, "itype must be 4 bytes");
  //std::fesetround(FE_TOWARDZERO);
  namespace po = boost::program_options; 
  // Initialize Verilators variables
  std::string sysArgs, pipelog;
  std::string rv32_binary = "dhrystone3";
  std::string retire_name;
  bool window, retiretrace = false, verbose = false;
  uint64_t heartbeat = 1UL<<36, start_trace_at = ~0UL;
  uint64_t max_cycle = 0, max_icnt = 0, mem_lat = 1;
  uint64_t last_store_addr = 0, last_load_addr = 0, last_addr = 0;
  int misses_inflight = 0;
  int64_t mem_reply_cycle = -1L;
  retire_trace rt;
  std::map<int64_t, double> &tip_map = rt.tip;
  std::map<int64_t, uint64_t> insn_cnts;
  uint64_t priv[4] = {0};
  
  try {
    po::options_description desc("Options");
    desc.add_options() 
      ("help", "Print help messages")
      ("args,a", po::value<std::string>(&sysArgs), "arguments to mips binary")
      ("checker,c", po::value<bool>(&enable_checker)->default_value(true), "use checker")
      ("isdump,d", po::value<bool>(&use_checkpoint)->default_value(false), "is a dump")
      ("file,f", po::value<std::string>(&rv32_binary), "mips binary")
      ("heartbeat,h", po::value<uint64_t>(&heartbeat)->default_value(1<<24), "heartbeat for stats")
      ("memlat,m", po::value<uint64_t>(&mem_lat)->default_value(4), "memory latency")
      ("pipelog,p", po::value<std::string>(&pipelog), "log for pipeline tracing")
      ("pipestart", po::value<uint64_t>(&pipestart)->default_value(0), "when to start logging")
      ("pipeend", po::value<uint64_t>(&pipeend)->default_value(~0UL), "when to stop logging")
      ("pipeip", po::value<uint64_t>(&pipeip)->default_value(0), "when to start logging")
      ("pipeipcnt", po::value<uint64_t>(&pipeipcnt)->default_value(0), "when to start logging")      
      ("maxcycle", po::value<uint64_t>(&max_cycle)->default_value(1UL<<34), "maximum cycles")
      ("maxicnt", po::value<uint64_t>(&max_icnt)->default_value(1UL<<50), "maximum icnt")
      ("trace,t", po::value<bool>(&trace_retirement)->default_value(false), "trace retired instruction stream")
      ("starttrace,s", po::value<uint64_t>(&start_trace_at)->default_value(~0UL), "start tracing retired instructions")
      ("window,w", po::value<bool>(&window)->default_value(false), "report windowed ipc")
      ("retiretrace", po::value<std::string>(&retire_name), "retire trace filename")
      ("verbose,v", po::value<bool>(&verbose)->default_value(false), "verbose")
      ("irq", po::value<bool>(&globals::checker_enable_irqs)->default_value(true), "enable irqs")
      ; 
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm); 
  }
  catch(po::error &e) {
    std::cerr <<"command-line error : " << e.what() << "\n";
    return -1;
  }
  uint32_t max_insns_per_cycle = 4;
  uint32_t max_insns_per_cycle_hist_sz = 2*max_insns_per_cycle;
  globals::log = trace_retirement;
  use_checkpoint = not(is_rv64_elf(rv32_binary.c_str()));
  retiretrace = retire_name.size() != -0;

  std::map<uint32_t, uint64_t> mispredicts;
  uint64_t inflight[32] = {0};
  uint64_t *insns_delivered = new uint64_t[max_insns_per_cycle_hist_sz];
  memset(insns_delivered, 0, sizeof(uint64_t)*max_insns_per_cycle_hist_sz);
  
  uint32_t max_inflight = 0;


  const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
  contextp->commandArgs(argc, argv);  
  s = new state_t;
  ss = new state_t;
  
  initState(s);
  initState(ss);

  
  s->mem = mmap4G();
  ss->mem = mmap4G();

  
  globals::sysArgc = buildArgcArgv(rv32_binary.c_str(),sysArgs.c_str(),&globals::sysArgv);
  std::string log_name = rv32_binary + "-log.txt";
  std::string tip_name = rv32_binary + "-tip.txt";
  std::string branch_name = rv32_binary + "-branch_info.txt";
  
  initCapstone();
  tb = new Vcore_l1d_l1i;
  uint64_t last_match_pc = 0;
  uint64_t last_retire = 0, last_check = 0, last_restart = 0;
  uint64_t mismatches = 0, n_stores = 0, n_loads = 0;
  uint64_t last_n_logged_loads = 0, last_total_load_lat = 0;
  uint64_t n_branches = 0, n_mispredicts = 0, n_checks = 0, n_flush_cycles = 0;
  bool got_mem_rsp = false, got_monitor = false, incorrect = false;
  bool got_putchar = false;
  bool was_in_flush_mode = false;
  bool pending_irq = false;

  globals::syscall_emu = not(use_checkpoint);
  tb->syscall_emu = globals::syscall_emu;

  
  if(use_checkpoint) {
    loadState(*s, rv32_binary.c_str());
    loadState(*ss, rv32_binary.c_str());
  }
  else {
    load_elf(rv32_binary.c_str(), s);
    load_elf(rv32_binary.c_str(), ss);
  }
  reset_core(tb, cycle, s->pc);

  
  if(not(pipelog.empty())) {
    pl = new pipeline_logger(pipelog);
  }
  s->pc = ss->pc;
  signal(SIGINT, catchUnixSignal);


  double t0 = timestamp();

  while(!Verilated::gotFinish() && (cycle < max_cycle) && (insns_retired < max_icnt)) {
    contextp->timeInc(1);  // 1 timeprecision periodd passes...    

    tb->clk = 1;
    tb->eval();

    if(kernel_panicd) {
      break;
    }
    //std::cout << "got to host " << std::hex << to_host << std::dec << ", flush = " << static_cast<int>(tb->in_flush_mode) << "\n";

    page_table_root = tb->page_table_root;

    if(not(tb->putchar_fifo_empty)) {
      std::cout << tb->putchar_fifo_out;
      got_putchar = true;
      tb->putchar_fifo_pop = 1;
    }
    
    if(tb->got_monitor) {
      uint32_t to_host = mem_r32(s, globals::tohost_addr);
      if(to_host) {
	if(to_host & 1) {
	  break;
	}
	handle_syscall(s, to_host);
	tb->monitor_ack = 1;
	got_monitor = true;      
      }
    }
    

    if(tb->retire_reg_valid) {
      s->gpr[tb->retire_reg_ptr] = tb->retire_reg_data;
      //if(tb->retire_reg_ptr == R_a0) {      
      //std::cout << std::hex << "insn with pc " << tb->retire_pc << " updates a0 \n"
      //<< std::dec;
      //}      
    }

    // if(tb->retire_valid) {
    //   std::cout << std::hex
    // 		<< tb->retire_pc
    // 		<< std::dec
    // 		<< "\n";
    // }
    
    
#ifdef BRANCH_DEBUG
    if(tb->branch_pc_valid) {
      ++n_branches;
    }
    if(tb->branch_fault) {
      uint64_t pa = tb->branch_pc;
      if(tb->paging_active) {
	pa = translate(tb->branch_pc, tb->page_table_root, true, false);
      }
      //auto disasm = getAsmString(tb->branch_pc, tb->page_table_root, tb->paging_active); 
      //printf("mispredict at %lx, translated to %lx, %s\n",
      //tb->branch_pc, pa, disasm.c_str());
      mispredicts[pa]++; 
    }
    
    if(tb->branch_fault) {
      ++n_mispredicts;
    }
#endif
    if(tb->in_flush_mode) {
      ++n_flush_cycles;
    }
    
    if(tb->alloc_two_valid) {
      insns_allocated+=2;
    }
    else if(tb->alloc_valid) {
      insns_allocated++;
    }

    if(tb->iq_one_valid) {
      fetch_stalls++;
    }
    else if(tb->iq_none_valid) {
      fetch_stalls+=2;
    }
    
    if(tb->in_branch_recovery) {
      cycles_in_faulted++;
    }
    if(tb->retire_valid) {
      priv[tb->priv & 3] += tb->retire_valid + tb->retire_two_valid;
    }
    
    if(tb->retire_valid and retiretrace) {
      uint64_t va = tb->retire_pc;
      uint64_t pa = va;
      if(tb->paging_active) {
	pa = translate(va, tb->page_table_root, true, false);
      }      
      if(pa < (1UL<<32)) {
	rt.get_records().emplace_back(pa,va,*reinterpret_cast<uint32_t*>(&s->mem[pa]));
      }
      
    }
    
    if(tb->retire_two_valid and retiretrace) {
      uint64_t va = tb->retire_two_pc;
      uint64_t pa = va;
      if(tb->paging_active) {
	pa = translate(va, tb->page_table_root, true, false);
      }      
      rt.get_records().emplace_back(pa,va,*reinterpret_cast<uint32_t*>(&s->mem[pa]));
    }

    
    if(tb->rob_empty) {
      uint64_t pa = last_retired_pc;
      if(tb->paging_active) {
	pa = translate(last_retired_pc, tb->page_table_root, true, false);
      }
      tip_map[pa]+= 1.0;
    }
    else if(!(tb->retire_valid or tb->retire_two_valid)) {
      uint64_t pa = tb->retire_pc;
      if(tb->paging_active) {
	pa = translate(tb->retire_pc, tb->page_table_root, true, false);
      }	
      tip_map[pa]+= 1.0;
    }
    else {
      assert(tb->retire_valid or tb->retire_two_valid);
      
      double total = static_cast<double>(tb->retire_valid) +
	static_cast<double>(tb->retire_two_valid);
      
      uint64_t pa = tb->retire_pc;
      if(tb->paging_active) {
	pa = translate(tb->retire_pc, tb->page_table_root, true, false);
      }
      tip_map[pa]+= 1.0 / total;
      insn_cnts[pa]++;
      if(tb->retire_two_valid) {
	pa = tb->retire_two_pc;
	if(tb->paging_active) {
	  pa = translate(tb->retire_two_pc, tb->page_table_root, true, false);
	}	
	tip_map[pa]+= 1.0 / total;
	insn_cnts[pa]++;
      }
    }
    //printf("rt.get_records().size() = %zu\n", rt.get_records().size());
    
    if(tb->took_irq) {
      pending_irq = true;
    }

    
    
    if(tb->retire_valid) {
      ++insns_retired;
      last_retire = 0;

      last_retired_pc = tb->retire_pc;

      if(insns_retired >= start_trace_at) {
	globals::log = trace_retirement = true;
      }

      if(((insns_retired % heartbeat) == 0) or trace_retirement ) {
	double w_ipc = static_cast<double>(insns_retired - last_insns_retired) / (cycle - last_cycle);
	double w_lat = static_cast<double>(total_load_lat-last_total_load_lat)/(n_logged_loads-last_n_logged_loads);	
	if(window) {
	  last_insns_retired = insns_retired;
	  last_cycle = cycle;
	  last_n_logged_loads = n_logged_loads;
	  last_total_load_lat = total_load_lat;
	}
	std::cout << "port a "
		  << " cycle " << cycle
		  << " "
		  << std::hex
		  << tb->retire_pc
		  << std::dec
		  << " "
		  << getAsmString(tb->retire_pc, tb->page_table_root, tb->paging_active)	  
		  << std::fixed
		  << ", " << w_ipc << " IPC "
		  << ", " << w_lat << " cyc "
		  << ", insns_retired "
		  << insns_retired
		  << ", mem pki "
		  << ((static_cast<double>(mem_reqs)/insns_retired)*100.0)
		  << ", mispredict pki "
		  << (static_cast<double>(n_mispredicts) / insns_retired) * 1000.0
		  << std::defaultfloat
		  << std::hex
		  << " "
		  << tb->retire_reg_data
		  << std::dec	  
		  <<" \n";
      }
      if(tb->retire_two_valid) {
	++insns_retired;
	last_retired_pc = tb->retire_pc;	
	if(((insns_retired % heartbeat) == 0) or trace_retirement ) {
	  double w_ipc = static_cast<double>(insns_retired - last_insns_retired) / (cycle - last_cycle);
	double w_lat = static_cast<double>(total_load_lat-last_total_load_lat)/(n_logged_loads-last_n_logged_loads);		  
	  if(window) {
	    last_insns_retired = insns_retired;
	    last_cycle = cycle;
	    last_n_logged_loads = n_logged_loads;
	    last_total_load_lat = total_load_lat;
	  }
	  std::cout << "port b "
		    << " cycle " << cycle
		    << " "
		    << std::hex
		    << tb->retire_two_pc
		    << std::dec
		    << " "
		    << getAsmString(tb->retire_two_pc, tb->page_table_root, tb->paging_active)	  	    
		    << std::fixed
		    << ", " << w_ipc << " IPC "
		    << ", " << w_lat << " cyc "	    
		    << ", insns_retired "
		    << insns_retired
		    << ", mem pki "
		    << ((static_cast<double>(mem_reqs)/insns_retired)*100.0)
		    << ", mispredict pki "
		    << (static_cast<double>(n_mispredicts) / insns_retired) * 1000.0
		    << std::defaultfloat
		    << std::hex
		    << " "
		    << tb->retire_reg_two_data
		    << std::dec
		    <<" \n";
	}
      }

      
      if(tb->got_bad_addr) {
	std::cout << "fatal - unaligned address\n";
	break;
      }
       
      
      if( enable_checker) {
	//printf("checking rtl %lx, sim %lx\n", tb->retire_pc, ss->pc);	
	int cnt = 0;
	bool mismatch = (tb->retire_pc != ss->pc), exception = false;
	uint64_t initial_pc = ss->pc;
	
	if((tb->retire_pc != ss->pc) and pending_irq) {
	  printf("divergence, rtl took interrupt\n");
	  exit(-1);
	}
	
	while( (tb->retire_pc != ss->pc) and (cnt < 3)) {
	  std::cout << "did not match, moving checker : RTL "
		    << std::hex << tb->retire_pc
		    << ", sim " << ss->pc
		    << std::dec
		    << " "
		    << " pending irq "
		    << static_cast<int>(tb->pending_irq)
		    << " "
		    << insns_retired
		    << " insns retired\n";
	  
	  execRiscv(ss);
	  exception |= ss->took_exception;
	  cnt++;
	}

	if(mismatch and not(exception)) {
	  std::cout << "mismatch without an exception at icnt " << insns_retired << "\n";
	  std::cout << std::hex << "last match " << std::hex << last_match_pc << std::dec << "\n";
	  std::cout << std::hex << "rtl : " << tb->retire_pc << ", sim : " << initial_pc << std::dec << "\n";
	  std::cout << "irq : " << pending_irq << "\n";
	  break;
	}
	
	if(tb->retire_pc == ss->pc) {
	  execRiscv(ss);
	  
	  bool diverged = false;
	  if(ss->pc == (tb->retire_pc + 4)) {

	    
	    for(int i = 0; i < 32; i++) {
	      if((ss->gpr[i] != s->gpr[i])) {
		/* not really a bug? */
		//if(ss->did_system) {
		//ss->gpr[i] = s->gpr[i];
		//break;
		//}
		
		int wrong_bits = __builtin_popcountll(ss->gpr[i] ^ s->gpr[i]);
		++mismatches;
		std::cout << "register " << getGPRName(i)
			  << " does not match : rtl "
			  << std::hex
			  << s->gpr[i]
			  << " simulator "
			  << ss->gpr[i]
			  << std::dec
			  << " bits in difference "
			  << wrong_bits
			  << "\n";
		
		if(wrong_bits == 1) {
		  int b = __builtin_ffsll(ss->gpr[i] ^ s->gpr[i])-1;
		  std::cout << "bit " << b << " differs\n";
		}
		//trace_retirement |= (wrong_bits != 0);
		diverged = true;

		
		//(wrong_bits > 16);
		std::cout << "incorrect "
			  << std::hex
			  << ss->pc 
			  << std::dec
			  << " : "
			  << getAsmString(ss->pc, tb->page_table_root, tb->paging_active)	  	    		  
			  << "\n";
		
	      }
	    }

	    
	  }
	  
	  if(diverged) {
	    incorrect = true;
	    std::cout << "incorrect "
		      << std::hex
		      << tb->retire_pc
		      << std::dec
		      << " : "
		      << getAsmString(tb->retire_pc, tb->page_table_root, tb->paging_active)	  	    		  	      
		      << "\n";
	    for(int i = 0; i < 32; i+=4) {
	      std::cout << "reg "
			<< getGPRName(i)
			<< " = "
			<< std::hex
			<< s->gpr[i]
			<< " reg "
			<< getGPRName(i+1)
			<< " = "
			<< s->gpr[i+1]
			<< " reg "
			<< getGPRName(i+2)
			<< " = "
			<< s->gpr[i+2]
			<< " reg "
			<< getGPRName(i+3)
			<< " = "
			<< s->gpr[i+3]
			<< std::dec <<"\n";
	    }
	    break;
	  }


	  
	  ++n_checks;
	  last_check = 0;
	  last_match_pc =  tb->retire_pc; 
	}
	else {
	  ++last_check;
	  if(last_check > 0) {
	    std::cerr << "no match in " << last_check << " insts, last match : "
		      << std::hex
		      << last_match_pc
		      << " "
		      << getAsmString(last_match_pc, tb->page_table_root, tb->paging_active)	  	    		  	      	      
		      << ", rtl pc =" << tb->retire_pc
		      << ", sim pc =" << ss->pc
		      << std::dec
		      <<"\n";
	    for(int i = 0; i < 32; i+=4) {
	      std::cout << "reg "
			<< getGPRName(i)
			<< " = "
			<< std::hex
			<< s->gpr[i]
			<< " reg "
			<< getGPRName(i+1)
			<< " = "
			<< s->gpr[i+1]
			<< " reg "
			<< getGPRName(i+2)
			<< " = "
			<< s->gpr[i+2]
			<< " reg "
			<< getGPRName(i+3)
			<< " = "
			<< s->gpr[i+3]
			<< std::dec <<"\n";
	    }
	    break;
	  }
	}
      }
      //do       
    }
    
    if(tb->retire_reg_two_valid) {
      s->gpr[tb->retire_reg_two_ptr] = tb->retire_reg_two_data;
      //if(tb->retire_reg_two_ptr == R_a0) {
      //std::cout << std::hex << "insn two with pc " << tb->retire_two_pc << " updates a0 \n"
      //<< std::dec;
      //}
    }
    

    if(enable_checker && tb->retire_two_valid) {
      if(tb->retire_two_pc == ss->pc) {
	execRiscv(ss);
	++n_checks;
	last_check = 0;
	last_match_pc =  tb->retire_two_pc; 
      }
    }

    if(not(tb->in_flush_mode) and was_in_flush_mode) {
      //int mem_eq = memcmp(ss->mem, s->mem, 1UL<<32);
      //printf("%lx, %lx\n", *(uint64_t*)&s->mem[0xffffe028],  *(uint64_t*)&ss->mem[0xffffe028]);
      //std::cout << "flush completes, mem eq = " << mem_eq << ", cycle " << cycle << "\n";
      //assert(mem_eq == 0);
    }
    was_in_flush_mode = tb->in_flush_mode;
    
    ++last_retire;
    if(last_retire > (1U<<21) && not(tb->in_flush_mode)) {
      std::cout << "in flush mode = " << static_cast<int>(tb->in_flush_mode) << "\n";
      std::cout << "no retire in " << last_retire << " cycles, last retired "
    		<< std::hex
    		<< last_retired_pc + 0
    		<< std::dec
    		<< " "
    		<< getAsmString(get_insn(last_retired_pc+0, s), last_retired_pc+0)
    		<< "\n";
      std::cout << "l1d  state = " << static_cast<int>(tb->l1d_state) << "\n";
      std::cout << "l1i  state = " << static_cast<int>(tb->l1i_state) << "\n";
      std::cout << "l2   state = " << static_cast<int>(tb->l2_state) << "\n";
      std::cout << "mmu  state = " << static_cast<int>(tb->mmu_state) << "\n";      
      std::cout << "core state = " << static_cast<int>(tb->core_state) << "\n";
      std::cout << "rob  ptr   = " << static_cast<int>(tb->rob_ptr) << "\n";
      break;
    }
    if(tb->got_break) {
      std::cout << "got break, epc = " << std::hex << tb->epc << std::dec << "\n";      
      break;
    }

    
    if(tb->got_ud) {
      uint32_t insn = get_insn(tb->epc, s);
      std::cerr << "GOT UD for "
		<< std::hex
		<< tb->epc
		<< " opcode " 
		<< (insn & 127)
		<< std::dec
		<< " "
		<< getAsmString(insn, tb->epc)
		<< "\n";
      break;
    }
    else if(tb->got_bad_addr) {
      uint32_t insn = get_insn(tb->epc, s);
      std::cerr << "GOT VA for "
		<< std::hex
		<< tb->epc
		<< " opcode " 
		<< (insn & 127)
		<< std::dec
		<< " "
		<< getAsmString(insn, tb->epc)
		<< "\n";
      break;
    }
    inflight[tb->inflight & 31]++;
    max_inflight = std::max(max_inflight, static_cast<uint32_t>(tb->inflight));

    //negedge
    tb->mem_rsp_valid = 0;
    tb->mem_req_gnt = 0;
    
    if(tb->mem_req_valid) {
      ++mem_reqs;
      int lat = mem_lat + 1;
      mem_reply_cycle = cycle + lat;
      tb->mem_req_gnt = 1;
      mem_req_t req(tb->mem_req_opcode==7,
		    tb->mem_req_addr,
		    tb->mem_req_tag,
		    cycle+lat);
      memcpy(req.data, tb->mem_req_store_data, sizeof(int)*4);

      //printf("got memory request for address %x of type %d, tag %d, will reply at cycle %lu, now %lu\n",
      //tb->mem_req_addr, tb->mem_req_opcode, tb->mem_req_tag,
      //cycle+lat,
      //cycle);
      
      mem_queue.push(req);
    }
    
    if(not(mem_queue.empty()) and mem_queue.front().reply_cycle ==cycle) {
      last_retire = 0;
      mem_req_t &req = mem_queue.front();
      //printf("reply memory request for address %x tag %d\n",
      //req.addr, req.tag);

      if(not(req.is_store)) {/*load word */
	for(int i = 0; i < 4; i++) {
	  uint64_t ea = (req.addr + 4*i) & ((1UL<<32)-1);
	  tb->mem_rsp_load_data[i] = mem_r32(s,ea);
	}
	last_load_addr = req.addr;
	assert((req.addr & 0xf) == 0);
	++n_loads;
      }
      else { /* store word */
	for(int i = 0; i < 4; i++) {
	  uint64_t ea = (req.addr + 4*i) & ((1UL<<32)-1);	  
	  mem_w32(s, ea, req.data[i]);
	}
	last_store_addr = req.addr;
	++n_stores;
      }
      last_addr = req.addr;
      tb->mem_rsp_valid = 1;
      tb->mem_rsp_tag = req.tag;
      mem_queue.pop();
    }

    
    tb->clk = 0;
    tb->eval();

    if(got_mem_rsp) {
      tb->mem_rsp_valid = 0;
      got_mem_rsp = false;
    }
    
    if(got_monitor) {
      tb->monitor_ack = 0;
      got_monitor = false;
    }
    if(got_putchar) {
      tb->putchar_fifo_pop = 0;
      got_putchar = false;
    }
    ++cycle;
  }


  tb->final();
  t0 = timestamp() - t0;

  if(enable_checker) {
    int mem_eq = memcmp(ss->mem, s->mem, 1UL<<32);
    if(mem_eq == 0) {
      std::cout << "checker mem equal rtl mem\n";
    }
    else {
      std::cout << "checker mem does not equal rtl mem\n";
      for(uint64_t p = 0; (p < (1UL<<32)) and verbose; p+=8) {
	uint64_t t0 = *reinterpret_cast<uint64_t*>(ss->mem + p);
	uint64_t t1 = *reinterpret_cast<uint64_t*>(s->mem + p);
	if(t0 != t1) {
	  std::cout << "qword at " << std::hex << p << " does not match SIM "
		    << t0 << " vs RTL " << t1
		    << std::dec << "\n";
	}
      }
    }  
  }
  
  if(!incorrect) {
    std::ofstream out(log_name);
    out << "n_mispredicts = " << n_mispredicts
	<<  ", cycles = " << cycle
	<< ", insns = " << insns_retired
	<< ", n_checks = " << n_checks
	<< "\n";
    out << static_cast<double>(insns_retired) / cycle << " insn per cycle\n";
    double avg_inflight = 0, sum = 0;
    for(int i = 0; i < 32; i++) {
      if(inflight[i] == 0) continue;
      avg_inflight += i * inflight[i];
      sum += inflight[i];
      //printf("inflight[%d] = %lu\n", i, inflight[i]);
    }
    avg_inflight /= sum;
    out << insns_retired << " insns retired\n";
    out << insns_allocated << " insns allocated\n";
    out << cycles_in_faulted*2 << " slots in faulted\n";
    out << fetch_stalls << " fetch stalls\n";
    out << mispred_to_restart_cycles << " cycles from mispred detect to restart\n";
    
    uint64_t totalSlots = 2*cycle;
    uint64_t badSpecSlots = insns_allocated - insns_retired + (2*cycles_in_faulted);
    double rr = static_cast<double>(insns_retired)/totalSlots;
    double bs = static_cast<double>(badSpecSlots) / totalSlots;
    double fe = static_cast<double>(fetch_stalls) / totalSlots;
    out << "top-down bb = " << (1.0 - (rr+bs+fe)) << "\n";     
    out << "top-down rr = " << rr << "\n";
    out << "top-down bs = " << bs << "\n";
    out << "top-down fe = " << fe << "\n";
										
		              
    //(SlotsIssued – SlotsRetired + RecoveryBubbles) / TotalSlots
    
    out << "avg insns in ROB = " << avg_inflight
	      << ", max inflight = " << max_inflight << "\n";
  

    out << "l1d cache hits = " << tb->l1d_cache_hits << "\n";
    out << "l1d cache accesses = " << tb->l1d_cache_accesses << "\n";
    out << "l1d hit rate = "
	      << 100.0 *(static_cast<double>(tb->l1d_cache_hits) / tb->l1d_cache_accesses)
	      << "\n";
    out << "l1i cache hits = " << tb->l1i_cache_hits << "\n";
    out << "l1i cache accesses = " << tb->l1i_cache_accesses << "\n";
    out << "l1i hit rate = "
	      << 100.0 *(static_cast<double>(tb->l1i_cache_hits) / tb->l1i_cache_accesses)
	      << "\n";
    out << "l1d tlb hits = " << tb->l1d_tlb_hits << "\n";
    out << "l1d tlb accesses = " << tb->l1d_tlb_accesses << "\n";
    out << "l1d tlb hit rate = "
	<<  100.0 *(static_cast<double>(tb->l1d_tlb_hits) /
		    tb->l1d_tlb_accesses) << "\n";
    out << "l1i tlb hits = " << tb->l1i_tlb_hits << "\n";
    out << "l1i tlb accesses = " << tb->l1i_tlb_accesses << "\n";        
    out << "l1i tlb hit rate = "
	<<  100.0 *(static_cast<double>(tb->l1i_tlb_hits) /
		    tb->l1i_tlb_accesses) << "\n";
    
    out << "l2 cache hits = " << tb->l2_cache_hits << "\n";
    out << "l2 cache accesses = " << tb->l2_cache_accesses << "\n";


    out << "branch mispredict rate = "
	      << (static_cast<double>(n_mispredicts)/n_branches)*100.0
	      << "\n";

    out << "mispredicts per kiloinsn = "
	      << (static_cast<double>(n_mispredicts) / insns_retired) * 1000.0
	      << "\n";
    out << n_flush_cycles << " cycles spent flushing caches\n";
    out << n_loads << " cache line loads\n";
    out << n_stores << " cache line stores\n";
    out << l1d_misses << " l1d misses\n";
    out << l1d_insns << " insns access the l1d\n";

    uint64_t total_fetch = 0, total_fetch_cycles = 0;
    for(int i = 0; i < 5; i++) {
      //out << "n_fetch[" << i << "] = " << n_fetch[i] << "\n";
      total_fetch_cycles += n_fetch[i];
      total_fetch += n_fetch[i] * i;
    }
    out << "avg fetch = " << static_cast<double>(total_fetch) / total_fetch_cycles << "\n";
    out << "resteer bubble = " << n_resteer_bubble << "\n";
    out << "front-end queues full = " << n_fq_full << "\n";
    out << "fetch_slots = " << fetch_slots << "\n";
    out << "total_slots = " << (cycle*2) << "\n";
    out << "retire_slots = " << insns_retired << "\n";

    out << "priv[0] = " << priv[0] << "\n";
    out << "priv[1] = " << priv[1] << "\n";
    out << "priv[2] = " << priv[2] << "\n";
    out << "priv[3] = " << priv[3] << "\n";

    out << "avg load lat = "
	<< static_cast<double>(total_load_lat)/n_logged_loads
	<< " cycles\n";

    
    for(int i = 0; i < 32; i++) {
      if(fault_counts[i] != 0) {
	out << "fault_counts[" << i << "] = "
	    << fault_counts[i] << "\n";
      }
    }
    
    double total_fetch_cap = 0.0;

  
    // for(int i = 0; i < 3; i++) {
    //   out << "uq_full[" << i << "] = " << n_uq_full[i] << "\n";
    // }
    uint64_t total_alloc = 0;
    for(int i = 0; i < 3; i++) {
      out << "alloc[" << i << "] = " << n_alloc[i] << "\n";
      total_alloc += i*n_alloc[i];
    }
    out << total_alloc << " total allocated uops\n";
    out << n_int_exec[0] << " cycles where int exec queue is not empty\n";
    out << n_int_exec[1] << " cycles where int exec queue dispatches\n";
    out << n_int2_exec[0] << " cycles where int2 exec queue is not empty\n";
    out << n_int2_exec[1] << " cycles where int2 exec queue dispatches\n";    
    out << n_mem_exec[0] << " cycles where mem exec queue is not empty\n";
    out << n_mem_exec[1] << " cycles where mem exec queue dispatches\n";

    std::cout << "mem queue tput = " << static_cast<double>(n_mem_exec[1]) /  n_mem_exec[0] << "\n";
    
    out << q_full[0] << " cycles with int queue full\n";
    out << q_full[1] << " cycles with mem queue full\n";
    out << dq_empty  << " cycles with an empty decode queue\n";
    out << uq_full   << " cycles with a  full uop queue\n";
    out << n_active << " cycles where the machine is in active state\n";
    out << rob_full << " cycles where the rob is full\n";
  
    double avg_restart = 0.0;
    uint64_t total_restart = 0, accum_restart = 0;
    for(auto &p : restart_distribution) {
      avg_restart += (p.first * p.second);
      total_restart += p.second;
    }
    for(auto &p : restart_distribution) {
      accum_restart += p.second;
      if(accum_restart >= (total_restart/2)) {
	out << p.first << " median flush cycles\n";
	break;
      }
    }
    if(total_restart != 0) {
      out << avg_restart << " cycles spent in pipeline flush\n";
      avg_restart /= total_restart;
      out << total_restart << " times pipeline was flushed\n";
      out << avg_restart << " cycles to flush on avg\n";
      out << restart_distribution.begin()->first << " min cycles to flush\n";
      out << restart_distribution.rbegin()->first << " max cycles to flush\n";
    }
    
    double avg_ds_restart = 0.0;
    uint64_t total_ds_restart = 0, accum_ds_restart = 0;
    for(auto &p : restart_ds_distribution) {
      avg_ds_restart += (p.first * p.second);
      total_ds_restart += p.second;
    }
    for(auto &p : restart_ds_distribution) {
      accum_ds_restart += p.second;
      if(accum_ds_restart >= (total_ds_restart/2)) {
	out << p.first << " median delay slot flush cycles\n";
	break;
      }
    }
    if(total_ds_restart != 0) {
      out << avg_ds_restart << " cycles spent waiting for delay slot in flush\n";
      avg_ds_restart /= total_ds_restart;
      out << avg_ds_restart << " cycles waiting on delay slot on avg\n";
      out << restart_ds_distribution.begin()->first << " min cycles for delay slot\n";
      out << restart_ds_distribution.rbegin()->first << " max cycles for delay slot\n";
    }
    //for(auto &p : fault_distribution) {
    //out << p.first << " faults inflight, " << p.second << " times\n";
    //}
    //for(auto &p : branch_distribution) {
    //out << p.first << " branches inflight, " << p.second << " times\n";
    //}
    //for(auto &p : fault_to_restart_distribution) {
    //out << p.first << " cycles before restart, " << p.second << " times\n";
    //}
    dump_histo(branch_name, mispredicts, s);
    
    uint64_t total_retire = 0, total_cycle = 0;
    for(auto &p : retire_map) {
      total_retire += p.second;
    }
    for(auto &p : retire_map) {
      total_cycle += (p.first * p.second);
    }

    int median_int_rdy;
    double avg_int_rdy = histo_mean_median(int_sched_rdy_map, median_int_rdy);
    out << "avg int rdy insn = " << avg_int_rdy << "\n";
    out << "median int rdy insn = " << median_int_rdy << "\n";
    
    int median_mem_lat = 0;
    double avg_mem_lat = histo_mean_median(mem_lat_map, median_mem_lat);
    out << "avg mem alloc to complete = " << avg_mem_lat << "\n";
    out << "median mem alloc to complete = " << median_mem_lat << "\n";

    avg_mem_lat = histo_mean_median(non_mem_lat_map, median_mem_lat);
    out << "avg non-mem alloc to complete = " << avg_mem_lat << "\n";
    out << "median non-mem alloc to complete = " << median_mem_lat << "\n";

    avg_mem_lat = histo_mean_median(mispred_lat_map, median_mem_lat);
    out << "avg mispred branch alloc to complete = " << avg_mem_lat << "\n";
    out << "median mispred branch alloc to complete = " << median_mem_lat << "\n";
    
    out << "l1d_reqs = " << l1d_reqs << "\n";
    out << "l1d_acks = " << l1d_acks << "\n";
    out << "l1d_new_reqs = " << l1d_new_reqs << "\n";
    out << "l1d_accept   = " << l1d_accept << "\n";

    for(size_t i = 0; i < (sizeof(l1d_block_reason)/sizeof(l1d_block_reason[0])); i++) {
      std::cout << l1d_stall_str[i] << " = " << l1d_block_reason[i] << "\n";
    }
    
    out << "l1d_stores = " << l1d_stores << "\n";
    out << "l1d tput = " << (static_cast<double>(l1d_acks) /l1d_reqs) << "\n";
    std::cout << "l1d tput = " << (static_cast<double>(l1d_acks) /l1d_reqs) << "\n";
    std::cout << "l1d_reqs = " << l1d_reqs << "\n";
    std::cout << "l1d_acks = " << l1d_acks << "\n";
    std::cout << "l1d_new_reqs = " << l1d_new_reqs << "\n";
    std::cout << "l1d_accept   = " << l1d_accept << "\n";
    
    
    std::cout << "total_retire = " << total_retire << "\n";
    std::cout << "total_cycle  = " << total_cycle << "\n";
    std::cout << "total ipc    = " << static_cast<double>(total_retire) / total_cycle << "\n";

    std::cout << "mmu walks    = " << count_histo(mmu_walk_lat) << "\n";
    std::cout << "avg mmu lat  = " << avg_histo(mmu_walk_lat) << "\n";

    uint64_t l1i_misses = tb->l1i_cache_accesses -
      tb->l1i_cache_hits;
    uint64_t l1d_misses = tb->l1d_cache_accesses -
      tb->l1d_cache_hits;
    uint64_t l2_misses = tb->l2_cache_accesses -
      tb->l2_cache_hits;    
    
    double l1i_mpki = static_cast<double>(l1i_misses) / (total_retire/1000);
    double l1d_mpki = static_cast<double>(l1d_misses) / (total_retire/1000);
    double l2_mpki = static_cast<double>(l2_misses) / (total_retire/1000);
    double jpki = static_cast<double>(n_mispredicts) / (total_retire/1000);
    std::cout << "jpki         = " << jpki << "\n";
    std::cout << "l1i mpki     = " << l1i_mpki << "\n";
    std::cout << "l1d mpki     = " << l1d_mpki << "\n";
    std::cout << "l2  mpki     = " << l2_mpki << "\n";        
    std::cout << "l1d accesses = " << tb->l1d_cache_accesses << "\n";

    double port1_util = static_cast<double>(port1_active) / total_cycle;
    double port2_util = static_cast<double>(port2_active) / total_cycle;    
    
    std::cout << "l1d port1 util = " << port1_util << "\n";
    std::cout << "l1d port2 util = " << port2_util << "\n";

    std::cout << "log_l1d_misses                 = " << log_l1d_misses << "\n";
    std::cout << "log_l1d_dirty_misses           = " << log_l1d_dirty_misses <<"\n";
    std::cout << "log_l1d_accesses               = " << log_l1d_accesses << "\n";
    std::cout << "log_l1d_push_miss              = " <<log_l1d_push_miss << "\n";
    std::cout << "log_l1d_push_miss_hit_inflight = " << log_l1d_push_miss_hit_inflight << "\n";
    std::cout << "log_l1d_early_reqs             = " << log_l1d_early_reqs << "\n";
    std::cout << "log_l1d_is_ld_hit              = " << log_l1d_is_ld_hit << "\n";
    std::cout << "log_l1d_is_st_hit              = " << log_l1d_is_st_hit << "\n";
    std::cout << "log_l1d_is_hit_under_miss      = " << log_l1d_is_hit_under_miss << "\n";

    double tip_cycles = 0.0;
    for(auto &p : tip_map) {
      tip_cycles += p.second;
    }
    std::cout << "tip cycles  = " << tip_cycles << "\n";
    dump_tip(tip_name, tip_map, insn_cnts, s);    
    out.close();

    uint64_t total_ticks = 0;
    for(auto &p : l1_to_l2_dist) {
      std::cout << p.first << "," << p.second << "\n";
      total_ticks += p.second;
    }
    std::cout << "total_ticks = " << total_ticks << "\n";
    const char* l2_state_names[] = {
      "INITIALIZE",
      "IDLE",
      "CHECK_VALID_AND_TAG",
      "CLEAN_RELOAD",
      "DIRTY_STORE",
      "STORE_TURNAROUND",
      "WAIT_CLEAN_RELOAD",
      "WAIT_STORE_IDLE",
      "FLUSH_STORE",
      "FLUSH_STORE_WAY2",
      "FLUSH_WAIT",
      "FLUSH_TRIAGE",
      "UPDATE_PTE"
    };
    
    for(auto &p : l2_state) {
      std::cout << l2_state_names[p.first] << "," << p.second << "\n";
    }
    std::cout << "port0 sched       " << schedules[0] << "\n";
    std::cout << "port1 sched       " << schedules[1] << "\n";
    std::cout << "port0 sched alloc " << schedules_alloc[0] << "\n";
    std::cout << "port1 sched alloc " << schedules_alloc[1] << "\n";
    
    if(not(rt.empty())) {
      std::ofstream ofs(retire_name);
      boost::archive::binary_oarchive oa(ofs);
      oa << rt;
      std::cout << "rt.get_records().size() = " <<
	rt.get_records().size() << "\n";
    }
  }
  else {
    std::cout << "instructions retired = " << insns_retired << "\n";
  }
  
  std::cout << "simulation took " << t0 << " seconds, " << (insns_retired/t0)
	    << " insns per second\n";

  uint64_t store_median, total_stores = 0;
  double avg_store_latency = 0.0;
  histo_mean_median(store_latency_map, store_median);
  std::cout << "median store latency = " << store_median << "\n";
  for(auto &p : store_latency_map) {
    total_stores += p.second;
    avg_store_latency += p.first * p.second;
  }
  avg_store_latency /= total_stores;
  std::cout << "avg store latency = " << (avg_store_latency) << "\n";
  
  munmap(s->mem, 1UL<<32);
  munmap(ss->mem, 1UL<<32);
  delete s;
  delete ss;
  delete [] insns_delivered;
  if(pl) {
    delete pl;
  }
  //delete tb;
  stopCapstone();
  delete tb;
  exit(EXIT_SUCCESS);
}
