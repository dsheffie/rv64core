#include "top.hh"

#ifdef USE_SDL
#include <SDL2/SDL.h>
#endif

#define BRANCH_DEBUG 1
#define CACHE_STATS 1

bool globals::syscall_emu = true;
uint32_t globals::tohost_addr = 0;
uint32_t globals::fromhost_addr = 0;
bool globals::log = false;
std::map<std::string, uint32_t> globals::symtab;


char **globals::sysArgv = nullptr;
int globals::sysArgc = 0;

SDL_Window *globals::sdlwin = nullptr;
SDL_Surface *globals::sdlscr = nullptr;

static uint64_t cycle = 0;
static uint64_t fetch_slots = 0;
static bool trace_retirement = false;
static uint64_t mem_reqs = 0;
static state_t *s = nullptr;
static state_t *ss = nullptr;
static uint64_t insns_retired = 0, insns_allocated = 0;
static uint64_t cycles_in_faulted = 0, fetch_stalls = 0;

static uint64_t pipestart = 0, pipeend = ~(0UL);

static boost::dynamic_bitset<> touched_lines(1UL<<28);

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
static uint64_t n_mem_exec[3] = {0};

static uint64_t q_full[3] = {0};
static uint64_t dq_empty =  0;
static uint64_t uq_full = 0;
static uint64_t n_active = 0;
static uint64_t rob_full = 0;

static uint64_t l1d_reqs = 0;
static uint64_t l1d_acks = 0;
static uint64_t l1d_stores = 0;

static std::map<int,uint64_t> block_distribution;
static std::map<int,uint64_t> restart_distribution;
static std::map<int,uint64_t> restart_ds_distribution;
static std::map<int,uint64_t> fault_distribution;
static std::map<int,uint64_t> branch_distribution;
static std::map<int,uint64_t> fault_to_restart_distribution;

static const char* l1d_stall_str[8] =
  {
   "no stall", //0
   "got miss", //1 
   "full memory queue", //2
   "not possible", //3
   "load retry", //4
   "store to same set", //5
   "cm block stall", //6
   "inflight rob ptr", //7
};
static uint64_t l1d_stall_reasons[8] = {0};
static bool enable_checker = true, use_checkpoint = false;
static bool pending_fault = false;
static uint64_t fault_start_cycle = 0;

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

void record_restart(int cycles) {
  restart_distribution[cycles]++;
  pending_fault = false;

  fault_to_restart_distribution[(cycle - fault_start_cycle)]++;
  fault_start_cycle = 0;
  //std::cout << "clearing fault took "
  //<< (cycle - fault_start_cycle)
  //<< " cycles\n";
}

void record_ds_restart(int cycles) {
  restart_ds_distribution[cycles]++;
}



void record_l1d(int req, int ack, int ack_st, int blocked, int stall_reason) {
  l1d_reqs += req;
  l1d_acks += ack;
  l1d_stores += ack_st;
  block_distribution[__builtin_popcount(blocked)]++;
  l1d_stall_reasons[stall_reason&15]++;
}

static bool verbose_ic_translate = false;

void csr_putchar(char c) {
  if(c==0) std::cout << "\n";
  else std::cout << c;
}

long long translate(long long va, long long root, bool iside, bool store) {
  uint64_t a = 0, u = 0;
  int mask_bits = -1;
  a = root + (((va >> 30) & 511)*8);
  u = *reinterpret_cast<int64_t*>(s->mem + a);
  if((u & 1) == 0) {
    if(verbose_ic_translate)
      printf("failed translation for %llx at level 3, u %lx r %llx\n", va, u, root);
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
  if((u & 1) == 0) {
    if(verbose_ic_translate)
      printf("failed translation for %llx at level 2\n", va);
    return (~0UL);
  }
  if((u>>1)&7) {
    mask_bits = 21;
    goto translation_complete;
  }
  
  //3rd level walk
  root = ((u >> 10) & ((1UL<<44)-1)) * 4096;  
  a = root + (((va >> 12) & 511)*8);
  u = *reinterpret_cast<int64_t*>(s->mem + a);
  if((u & 1) == 0) {
    if(verbose_ic_translate)
      printf("failed translation for %llx at level 1\n", va);
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
  //printf("translation complete, pa %lx!\n", pa);
  //exit(-1);
  return pa;
}

uint64_t page_table_root = ~0UL;
std::list<store_rec> store_queue;
std::list<store_rec> atomic_queue;

void wr_log(long long pc, long long addr, long long data, int is_atomic) {
  if(not(enable_checker))
    return;
  //  if(is_atomic) {
  //printf("pc %llx, addr %llx, data %llx, atomic %d\n",
  //	   pc, addr, data, is_atomic);
  //}

  if(is_atomic) {
    atomic_queue.emplace_back(pc, addr, data);
    return;
  }
  assert(not(store_queue.empty()));

  auto &t = store_queue.front();
  if(not(t.pc == pc and t.addr == addr and t.data == data)) {
    printf("you have a store error! for an atomic %d, pc mismatch %d, addr mismatch %d, data mismatch %d\n",
	   is_atomic, t.pc==pc, t.addr==addr, t.data == data);
    printf("%lx, %lx, %lx\n", t.pc, t.addr, t.data);
    //exit(-1);
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

  printf("check : pc %llx, pa %lx, rtl pa %llx, match %d, sim %x, rtl %x\n",
	 pc,
	 pa,
	 rtl_pa,
	 match,
	 u,
	 *reinterpret_cast<uint32_t*>(&insn));
    
  return not(match);
}


long long ld_translate(long long va) {
  return translate(va, page_table_root, false, false);
}

uint64_t csr_time = 0;

long long csr_gettime() {
  return csr_time;
}

bool kernel_panicd = false;
void term_sim() {
  kernel_panicd = true;
}


std::string getAsmString(uint64_t addr, uint64_t root, bool paging_enabled) {
  int64_t pa = addr;
  if(paging_enabled) {
    verbose_ic_translate = true;
    pa = translate(addr, root, true, false);
    verbose_ic_translate = false;
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
  n_mem_exec[2] += blocked_by_store;
    
  q_full[0] += intq_full;
  q_full[1] += memq_full;
  q_full[2] += fpq_full;

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

static std::map<int, uint64_t> mem_lat_map, fp_lat_map, non_mem_lat_map, mispred_lat_map;
static std::map<int64_t, double> tip_map;

int check_insn_bytes(long long pc, int data) {
  uint32_t insn = get_insn(pc, s);
  return (*reinterpret_cast<uint32_t*>(&data)) == insn;
}

static long long lrc = -1;
static uint64_t record_insns_retired = 0;

static int pl_regs[32] = {0};


void record_retirement(long long pc,
		       long long fetch_cycle,
		       long long alloc_cycle,
		       long long complete_cycle,
		       long long retire_cycle,
		       int retire_reg_val,
		       int retire_reg_ptr,
		       long long retire_reg_data,
		       int faulted ,
		       int br_mispredict) {

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
  
  
  if((pl != nullptr) and (record_insns_retired >= pipestart) and (record_insns_retired < pipeend)) {
    uint32_t insn = get_insn(pc, s);
    uint32_t opcode = insn & 127;
    auto disasm = getAsmString(insn, pc);
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

    pl->append(record_insns_retired, disasm, pc, fetch_cycle, alloc_cycle, complete_cycle, retire_cycle, faulted);
  }
  ++record_insns_retired;
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


int main(int argc, char **argv) {
  static_assert(sizeof(itype) == 4, "itype must be 4 bytes");
  //std::fesetround(FE_TOWARDZERO);
  namespace po = boost::program_options; 
  // Initialize Verilators variables
  std::string sysArgs, pipelog;
  std::string rv32_binary = "dhrystone3";
  std::string log_name = "log.txt";
  std::string pushout_name = "pushout.txt";
  std::string branch_name = "branch_info.txt";
  bool use_fb = false;
  uint64_t heartbeat = 1UL<<36, start_trace_at = ~0UL;
  uint64_t max_cycle = 0, max_icnt = 0, mem_lat = 2;
  uint64_t last_store_addr = 0, last_load_addr = 0, last_addr = 0;
  int misses_inflight = 0;
  std::map<uint64_t, uint64_t> pushout_histo;
  int64_t mem_reply_cycle = -1L;
  try {
    po::options_description desc("Options");
    desc.add_options() 
      ("help", "Print help messages")
      ("args,a", po::value<std::string>(&sysArgs), "arguments to mips binary")
      ("checker,c", po::value<bool>(&enable_checker)->default_value(true), "use checker")
      ("isdump,d", po::value<bool>(&use_checkpoint)->default_value(false), "is a dump")
      ("file,f", po::value<std::string>(&rv32_binary), "mips binary")
      ("heartbeat,h", po::value<uint64_t>(&heartbeat)->default_value(1<<24), "heartbeat for stats")
      ("log,l", po::value<std::string>(&log_name), "stats log filename")
      ("pushout", po::value<std::string>(&pushout_name), "pushout log filename")
      ("branch", po::value<std::string>(&branch_name), "branch log filename")
      ("memlat,m", po::value<uint64_t>(&mem_lat)->default_value(4), "memory latency")
      ("pipelog,p", po::value<std::string>(&pipelog), "log for pipeline tracing")
      ("pipestart", po::value<uint64_t>(&pipestart)->default_value(0), "when to start logging")
      ("pipeend", po::value<uint64_t>(&pipeend)->default_value(~0UL), "when to stop logging")      
      ("maxcycle", po::value<uint64_t>(&max_cycle)->default_value(1UL<<34), "maximum cycles")
      ("maxicnt", po::value<uint64_t>(&max_icnt)->default_value(1UL<<50), "maximum icnt")
      ("trace,t", po::value<bool>(&trace_retirement)->default_value(false), "trace retired instruction stream")
      ("starttrace,s", po::value<uint64_t>(&start_trace_at)->default_value(~0UL), "start tracing retired instructions")
      ("fb", po::value<bool>(&use_fb)->default_value(false), "use an SDL framebuffer")
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

  std::map<uint32_t, uint64_t> mispredicts;

  uint64_t hist = 0, spec_hist = 0;
  static const int TBL_SIZE = (1<<24);
  static const int BTB_SIZE = (1<<6);
  
  
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
  initCapstone();
  std::unique_ptr<Vcore_l1d_l1i> tb(new Vcore_l1d_l1i);
  uint64_t last_match_pc = 0;
  uint64_t last_retire = 0, last_check = 0, last_restart = 0;
  uint64_t last_retired_pc = 0, last_retired_fp_pc = 0;
  uint64_t mismatches = 0, n_stores = 0, n_loads = 0;
  uint64_t n_branches = 0, n_mispredicts = 0, n_checks = 0, n_flush_cycles = 0;
  bool got_mem_req = false, got_mem_rsp = false, got_monitor = false, incorrect = false;
  bool got_putchar = false;
  bool was_in_flush_mode = false;
#ifdef USE_SDL
  if(use_fb) {
    assert(SDL_Init(SDL_INIT_VIDEO) == 0);
    globals::sdlwin = SDL_CreateWindow("FRAMEBUFFER",
				       SDL_WINDOWPOS_UNDEFINED,
				       SDL_WINDOWPOS_UNDEFINED,
				       FB_WIDTH,
				       FB_HEIGHT,
				       SDL_WINDOW_SHOWN);
    assert(globals::sdlwin != nullptr);
    globals::sdlscr = SDL_GetWindowSurface(globals::sdlwin);
    assert(globals::sdlscr);
  }
#endif
  
  globals::syscall_emu = not(use_checkpoint);
  tb->syscall_emu = globals::syscall_emu;
  
  if(use_checkpoint) {
    loadState(*s, rv32_binary.c_str());
    for(int i = 0; i < 32; i++) {
      assert(s->gpr[i] == 0);
    }
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
      mispredicts[tb->branch_pc]++; 
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

    /* virtual addresses */
    if(globals::syscall_emu) {
      if(tb->rob_empty) {
	tip_map[last_retired_pc]+= 1.0;
      }
      else if(!(tb->retire_valid or tb->retire_two_valid)) {
	tip_map[tb->retire_pc]+= 1.0;
      }
      else {
	assert(tb->retire_valid or tb->retire_two_valid);      
	double total = static_cast<double>(tb->retire_valid) +
	  static_cast<double>(tb->retire_two_valid);
	tip_map[tb->retire_pc]+= 1.0 / total;
	if(tb->retire_two_valid) {
	  tip_map[tb->retire_two_pc]+= 1.0 / total;
	}
      }
    }
    
    if(tb->retire_valid) {
      ++insns_retired;
      if(last_retire > 1) {
	pushout_histo[tb->retire_pc] += last_retire;
      }
      last_retire = 0;

      last_retired_pc = tb->retire_pc;

      if(insns_retired >= start_trace_at) {
	trace_retirement = true;
      }

      if(((insns_retired % (1<<20)) == 0)) {
	++csr_time;
      }
      
      if(((insns_retired % heartbeat) == 0) or trace_retirement ) {

	std::cout << "port a "
		  << " cycle " << cycle
		  << " "
		  << std::hex
		  << tb->retire_pc
		  << std::dec
		  << " "
		  << getAsmString(tb->retire_pc, tb->page_table_root, tb->paging_active)	  
		  << std::fixed
		  << ", " << static_cast<double>(insns_retired) / cycle << " IPC "
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
	  std::cout << "port b "
		    << " cycle " << cycle
		    << " "
		    << std::hex
		    << tb->retire_two_pc
		    << std::dec
		    << " "
		    << getAsmString(tb->retire_two_pc, tb->page_table_root, tb->paging_active)	  	    
		    << std::fixed
		    << ", " << static_cast<double>(insns_retired) / cycle << " IPC "	    
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
	
	int cnt = 0;
	bool mismatch = (tb->retire_pc != ss->pc), exception = false;
	uint64_t initial_pc = ss->pc;
	while( (tb->retire_pc != ss->pc) and (cnt < 3)) {
	  execRiscv(ss);
	  exception |= ss->took_exception;
	  cnt++;
	}

	if(mismatch and not(exception)) {
	  std::cout << "mismatch without an exception at icnt " << insns_retired << "\n";
	  std::cout << std::hex << "last match " << std::hex << last_match_pc << std::dec << "\n";
	  std::cout << std::hex << tb->retire_pc << "," << initial_pc << std::dec << "\n";
	  break;
	}
	
	if(tb->retire_pc == ss->pc) {
	  //std::cout << std::hex << tb->retire_pc << "," << ss->pc << std::dec << "\n";	  	
	  execRiscv(ss);
	  // if(static_cast<uint32_t>(ss->mem.at(0x4cadc)) == 3) {
	  //   std::cout << "changed memory at " << std::hex << ss->pc << std::dec << "\n";
	  //   exit(-1);
	  // }
	  
	  bool diverged = false;
	  if(ss->pc == (tb->retire_pc + 4)) {
	    for(int i = 0; i < 32; i++) {
	      if((ss->gpr[i] != s->gpr[i])) {
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
		//trace_retirement |= (wrong_bits != 0);
		diverged = true;//(wrong_bits > 16);
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
    if(last_retire > (1U<<15) && not(tb->in_flush_mode)) {
      std::cout << "in flush mode = " << static_cast<int>(tb->in_flush_mode) << "\n";
      std::cerr << "no retire in " << last_retire << " cycles, last retired "
    		<< std::hex
    		<< last_retired_pc + 0
    		<< std::dec
    		<< " "
    		<< getAsmString(get_insn(last_retired_pc+0, s), last_retired_pc+0)
    		<< "\n";
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

    if(tb->mem_req_valid && (mem_reply_cycle == -1)) {
      ++mem_reqs;
      mem_reply_cycle = cycle + (tb->mem_req_opcode == 4 ? 1 : 2)*mem_lat;
      
    }
    
    if(/*tb->mem_req_valid*/mem_reply_cycle ==cycle) {
      //std::cout << "got memory request for address "
      //<< std::hex << tb->mem_req_addr << std::dec <<"\n";
      last_retire = 0;
      mem_reply_cycle = -1;
      assert(tb->mem_req_valid);

      
      if(tb->mem_req_opcode == 4) {/*load word */
	for(int i = 0; i < 4; i++) {
	  uint64_t ea = (tb->mem_req_addr + 4*i) & ((1UL<<32)-1);
	  //if(tb->paging_active) {
	  //printf("got dram read for %lx, data %x\n", ea, mem_r32(s,ea));
	  //}
	  tb->mem_rsp_load_data[i] = mem_r32(s,ea);
	}
	last_load_addr = tb->mem_req_addr;
	assert((tb->mem_req_addr & 0xf) == 0);
	touched_lines[(tb->mem_req_addr & ((1UL<<32) - 1))>>4] = 1;
	++n_loads;
      }
      else if(tb->mem_req_opcode == 7) { /* store word */
	for(int i = 0; i < 4; i++) {
	  uint64_t ea = (tb->mem_req_addr + 4*i) & ((1UL<<32)-1);
	  //if(tb->paging_active) {
	  //printf("got dram write for %lx, data %x\n", ea, tb->mem_req_store_data[i]);
	  //}
	  mem_w32(s, ea, tb->mem_req_store_data[i]);
	}
	last_store_addr = tb->mem_req_addr;
	++n_stores;
      }
      last_addr = tb->mem_req_addr;
      tb->mem_rsp_valid = 1;
    }

    
    tb->clk = 0;
    tb->eval();
    if(got_mem_req) {
      got_mem_req = false;
    }
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
      // for(uint64_t p = 0; p < (1UL<<32); p+=8) {
      // 	uint64_t t0 = *reinterpret_cast<uint64_t*>(ss->mem + p);
      // 	uint64_t t1 = *reinterpret_cast<uint64_t*>(s->mem + p);
      // 	if(t0 != t1) {
      // 	  printf("qword at %lx does not match SIM %lx vs RTL %lx\n",
      // 		 p, t0, t1);
      // 	}
	  
      // }
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
    out << n_mem_exec[2] << " cycles where mem exec queue is blocked by a store\n";

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
    for(auto &p : fault_distribution) {
      out << p.first << " faults inflight, " << p.second << " times\n";
    }
    for(auto &p : branch_distribution) {
      out << p.first << " branches inflight, " << p.second << " times\n";
    }
    for(auto &p : fault_to_restart_distribution) {
      out << p.first << " cycles before restart, " << p.second << " times\n";
    }
    dump_histo(branch_name, mispredicts, s);
    uint64_t total_pushout = 0;
    for(auto &p : pushout_histo) {
      total_pushout += p.second;
    }
    out << total_pushout << " cycles of pushout\n";
    dump_histo(pushout_name, pushout_histo, s);
    
    //std::ofstream branch_info("retire_info.csv");
    uint64_t total_retire = 0, total_cycle = 0;
    for(auto &p : retire_map) {
      total_retire += p.second;
    }
    for(auto &p : retire_map) {
      //branch_info << p.first << "," << p.second << "," << static_cast<double>(p.second) / total_retire << "\n";
      total_cycle += (p.first * p.second);
    }
    //branch_info.close();
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
    out << "l1d_stores = " << l1d_stores << "\n";
    out << "l1d tput = " << (static_cast<double>(l1d_acks) /l1d_reqs) << "\n";
    
    //for(auto &p :block_distribution) {
    //out << p.first << "," << p.second << "\n";
    //}
    for(int i = 1; i < 8; i++) {
      if(l1d_stall_reasons[i] != 0) {
	out << l1d_stall_reasons[i] << " " << l1d_stall_str[i] << "\n";
      }
    }
    std::cout << "total_retire = " << total_retire << "\n";
    std::cout << "total_cycle  = " << total_cycle << "\n";
    std::cout << "total ipc    = " << static_cast<double>(total_retire) / total_cycle << "\n";
    if(globals::syscall_emu) {
      double tip_cycles = 0.0;
      for(auto &p : tip_map) {
	tip_cycles += p.second;
      }
      std::cout << "tip cycles  = " << tip_cycles << "\n";
      dump_histo("tip.txt", tip_map, s);    
      out.close();
    }
  }
  else {
    std::cout << "instructions retired = " << insns_retired << "\n";
  }
  
  std::cout << "simulation took " << t0 << " seconds, " << (insns_retired/t0)
	    << " insns per second\n";


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
#ifdef USE_SDL
  if(globals::sdlwin) {
    SDL_DestroyWindow(globals::sdlwin);
    SDL_Quit();
  }
#endif
  exit(EXIT_SUCCESS);
}
