#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>
#include <list>
#include <unordered_map>

#include "interpret.hh"
#include "temu_code.hh"
#include "disassemble.hh"
#include "helper.hh"
#include "globals.hh"

#include <stack>
extern std::list<store_rec> store_queue;
extern std::list<store_rec> atomic_queue;

static uint64_t last_tval = 0;
static std::stack<int64_t> calls;

static void dump_calls() {
  int cnt = 0;
  while(!calls.empty() && (cnt < 5)) {
    int64_t ip = calls.top();
    std::cout << std::hex << ip << std::dec << "\n";
    calls.pop();
    cnt++;
  }
}

void initState(state_t *s) {
  memset(s, 0, sizeof(state_t));
  s->misa = 0x8000000000141101L;
  s->priv = priv_machine;
  s->mstatus = ((uint64_t)2 << MSTATUS_UXL_SHIFT) |((uint64_t)2 << MSTATUS_SXL_SHIFT);

}

bool state_t::memory_map_check(uint64_t pa, bool store, int64_t x) {
  if(pa >= VIRTIO_BASE_ADDR and (pa < (VIRTIO_BASE_ADDR + VIRTIO_SIZE))) {

  }
  if(pa >= UART_BASE_ADDR and (pa < (UART_BASE_ADDR + UART_SIZE))) {
  }
  if(pa >= PLIC_BASE_ADDR and (pa < (PLIC_BASE_ADDR + PLIC_SIZE))) {
    printf(">> %s plic range at pc %lx, offset %ld bytes\n", store ? "write" : "read", pc, pa-PLIC_BASE_ADDR);
    //exit(-1);
    return true;
  }
  if(pa >= CLINT_BASE_ADDR and (pa < (CLINT_BASE_ADDR + CLINT_SIZE))) {
    assert(store);
    switch(pa-CLINT_BASE_ADDR)
      {
      case 0x0:
	printf("msip access\n");
	break;
      case 0x4000:	
	if(store) {
	  mtimecmp = x;
	  printf(">> mtimecmp = %ld at icnt %ld\n", mtimecmp, icnt);
	  csr_t cc(mip);
	  cc.mie.mtie = 0;
	  printf("mip %lx -> %lx\n", mip, cc.raw);
	  mip = cc.raw;
	}
	break;
      case 0xbff8:
	assert(not(store));
	break;
      default:
	break;
      }
    //printf(">> %s clint range at pc %lx, offset %ld bytes, st value %lx\n",
    //store ? "write" : "read", pc, pa-CLINT_BASE_ADDR, x);
    //exit(-1);
    return true;
  }  
  return false;
}

int8_t state_t::load8(uint64_t pa) {
  memory_map_check(pa);
  return *reinterpret_cast<int8_t*>(mem + pa);
}

int64_t state_t::load8u(uint64_t pa) {
  uint64_t z = 0;
  memory_map_check(pa);  
  *reinterpret_cast<uint64_t*>(&z) = *reinterpret_cast<uint8_t*>(mem + pa);
  return z;
}  

int16_t state_t::load16(uint64_t pa) {
  memory_map_check(pa);
  return *reinterpret_cast<int16_t*>(mem + pa);
}

int64_t state_t::load16u(uint64_t pa) {
  uint64_t z = 0;
  memory_map_check(pa);
  *reinterpret_cast<uint64_t*>(&z) = *reinterpret_cast<uint16_t*>(mem + pa);
  return z;
}  

int32_t state_t::load32(uint64_t pa) {
  memory_map_check(pa);  
  return *reinterpret_cast<int32_t*>(mem + pa);
}

int64_t state_t::load32u(uint64_t pa) {
  uint64_t z = 0;
  memory_map_check(pa);
  *reinterpret_cast<uint64_t*>(&z) = *reinterpret_cast<uint32_t*>(mem + pa);
  return z;
}  

int64_t state_t::load64(uint64_t pa) {
  memory_map_check(pa);
  return *reinterpret_cast<int64_t*>(mem + pa);
}

void state_t::store8(uint64_t pa,  int8_t x) {
  memory_map_check(pa,true);
  *reinterpret_cast<int8_t*>(mem + pa) = x;
}

void state_t::store16(uint64_t pa, int16_t x) {
  memory_map_check(pa,true);
  *reinterpret_cast<int16_t*>(mem + pa) = x;
}

void state_t::store32(uint64_t pa, int32_t x) {
  memory_map_check(pa,true);
  *reinterpret_cast<int32_t*>(mem + pa) = x;
}

void state_t::store64(uint64_t pa, int64_t x) {
  memory_map_check(pa,true);
  *reinterpret_cast<int64_t*>(mem + pa) = x;
}

extern uint64_t csr_time;

void state_t::set_time(int64_t t) {
  csr_time = t;
}

int64_t state_t::get_time() const {
  return icnt;
}

//static std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> tlb;

uint64_t state_t::translate(uint64_t ea, int &fault, int sz, bool store, bool fetch,
			    bool force) const {
  fault = false;
  if(unpaged_mode() and not(force)) {
    return ea;
  }
  csr_t c(satp);
  pte_t r(0);
  uint64_t ea0 = ea & (~4095L);
  uint64_t ea1 = ((ea + sz - 1) & (~4095L));
  bool same_page = (ea0 == ea1);
  uint64_t a = 0, u = 0;
  int mask_bits = -1;
  uint64_t tlb_pa = 0;
  
  //if we are unaligned assert out (for now)
  if(!same_page) {
    fault = 1;
    return 2;
  }

  assert(c.satp.mode == 8);
  a = (c.satp.ppn * 4096) + (((ea >> 30) & 511)*8);
  
  //printf("level 0 : %x\n", a);  
  u = *reinterpret_cast<uint64_t*>(mem + a);

  if((u&1) == 0) {
    fault = 1;
    return 2;
  }  
  r.r = u;
  if(r.sv39.x || r.sv39.w || r.sv39.r) {
    mask_bits = 30;
    goto translation_complete;
  }
  
  a = (r.sv39.ppn * 4096) + (((ea >> 21) & 511)*8);
  //printf("level 1 : %x\n", a);  
  u = *reinterpret_cast<uint64_t*>(mem + a);

  if((u&1) == 0) {
    fault = 1;
    return 1;
  }
  
  r.r = u;
  if(r.sv39.x || r.sv39.w || r.sv39.r) {
    mask_bits = 21;
    goto translation_complete;
  }
  a = (r.sv39.ppn * 4096) + (((ea >> 12) & 511)*8);
  //printf("level 2 : %x\n", a);
  u = *reinterpret_cast<uint64_t*>(mem + a);  
  if((u&1) == 0) {
    fault = 1;
    return 0;
  }
  r.r = u;  
  if(not(r.sv39.x || r.sv39.w || r.sv39.r)) {
    std::cout << "huh no translation for " << std::hex << pc << std::dec << "\n";
    std::cout << "huh no translation for " << std::hex << ea << std::dec << "\n";
    std::cout << "u = " << std::hex << u << std::dec << "\n";
  }

  assert(r.sv39.x || r.sv39.w || r.sv39.r);
  mask_bits = 12;
  
 translation_complete:

  //* permission checks */
  if(fetch && (r.sv39.x == 0)) {
    fault = 1;
    return 0;
  }
  if(store && (r.sv39.w == 0)) {
    fault = 1;
    return 0;
  }
  if(r.sv39.w && (r.sv39.r == 0)) {
    fault = 1;
    return 0;
  }
  if(not(store or fetch) && (r.sv39.r == 0)) {
    fault = 1;
    return 0;
  }
  if(r.sv39.u == 0 && (priv == priv_user)) {
    fault = 1;
    return 0;
  }
  if(r.sv39.u == 1 && fetch && (priv != priv_user)) {
    fault = 1;
    return 0;
  }
  assert(mask_bits != -1);

  
  if(r.sv39.a == 0) {
    r.sv39.a = 1;
    printf("simulator marking page at %lx accessed for pc %lx\n", ea, pc);
    *reinterpret_cast<uint64_t*>(mem + a) = r.r;
  }
  if((r.sv39.d == 0) && store) {
    r.sv39.d = 1;
    printf("simulator marking page at %lx dirty for pc %lx\n", ea, pc);
    *reinterpret_cast<uint64_t*>(mem + a) = r.r;    
  }
  int64_t m = ((1L << mask_bits) - 1);
  int64_t pa = ((r.sv39.ppn * 4096) & (~m)) | (ea & m);
  
  //tlb[ea >>  12] = std::pair<uint64_t, uint64_t>(r.r, (r.sv39.ppn << 12) | mask_bits );
  // if(tlb_pa != 0 && (pa != tlb_pa)) {
  //   std::cout << "m = " << std::hex << m << std::dec << "\n";
  //   printf("ea     = %lx\n", ea);
  //   printf("pa     = %lx\n", pa);
  //   printf("tlb pa = %lx\n", tlb_pa);
  //   assert(pa == tlb_pa);
  // }
  return pa;
}

uint64_t state_t::page_lookup(uint64_t ea, int &fault, int sz, bool verbose) const {
  fault = false;
  if(unpaged_mode()) {
    return ea;
  }
  csr_t c(satp);
  pte_t r(0);
  uint64_t ea0 = ea & (~4095L);
  uint64_t ea1 = ((ea + sz - 1) & (~4095L));
  bool same_page = (ea0 == ea1);
  uint64_t a = 0, u = 0;
  int mask_bits = -1;
  uint64_t tlb_pa = 0;
  
  //if we are unaligned assert out (for now)
  if(!same_page) {
    fault = 1;
    return 2;
  }

  assert(c.satp.mode == 8);
  a = (c.satp.ppn * 4096) + (((ea >> 30) & 511)*8);
  
  u = *reinterpret_cast<uint64_t*>(mem + a);
  if(verbose) {
    printf("level 0 : va %lx, addr %lx, data %lx\n", ea, a, u);
  }

  if((u&1) == 0) {
    fault = 1;
    return 2;
  }  
  r.r = u;
  if(r.sv39.x || r.sv39.w || r.sv39.r) {
    mask_bits = 30;
    goto translation_complete;
  }
  
  a = (r.sv39.ppn * 4096) + (((ea >> 21) & 511)*8);
  //printf("level 1 : %x\n", a);  
  u = *reinterpret_cast<uint64_t*>(mem + a);

  if(verbose) {
    printf("level 1 : va %lx, addr %lx, data %lx\n", ea, a, u);
  }
  
  if((u&1) == 0) {
    fault = 1;
    return 1;
  }
  
  r.r = u;
  if(r.sv39.x || r.sv39.w || r.sv39.r) {
    mask_bits = 21;
    goto translation_complete;
  }
  a = (r.sv39.ppn * 4096) + (((ea >> 12) & 511)*8);
  u = *reinterpret_cast<uint64_t*>(mem + a);

  if(verbose) {
    printf("level 2 : addr %lx, data %lx\n", a, u);
  }

  
  if((u&1) == 0) {
    fault = 1;
    return 0;
  }
  r.r = u;  
  if(not(r.sv39.x || r.sv39.w || r.sv39.r)) {
    std::cout << "huh no translation for " << std::hex << pc << std::dec << "\n";
    std::cout << "huh no translation for " << std::hex << ea << std::dec << "\n";
    std::cout << "u = " << std::hex << u << std::dec << "\n";
  }

  assert(r.sv39.x || r.sv39.w || r.sv39.r);
  mask_bits = 12;
  
 translation_complete:
  assert(mask_bits != -1);
  int64_t m = ((1L << mask_bits) - 1);
  ea &= (~4095UL);
  int64_t pa = ((r.sv39.ppn * 4096) & (~m)) | (ea & m);
  
  //tlb[ea >>  12] = std::pair<uint64_t, uint64_t>(r.r, (r.sv39.ppn << 12) | mask_bits );
  // if(tlb_pa != 0 && (pa != tlb_pa)) {
  //   std::cout << "m = " << std::hex << m << std::dec << "\n";
  //   printf("ea     = %lx\n", ea);
  //   printf("pa     = %lx\n", pa);
  //   printf("tlb pa = %lx\n", tlb_pa);
  //   assert(pa == tlb_pa);
  // }
  return pa;
}

static void set_priv(state_t *s, int priv) {
  if (s->priv != priv) {
    //printf("tlb had %lu entries\n", tlb.size());
    //tlb.clear();
    int mxl;
    if (priv == priv_supervisor) {
      mxl = (s->mstatus >> MSTATUS_SXL_SHIFT) & 3;
      assert(mxl == 2);
    }
    else if (priv == priv_user) {
      mxl = (s->mstatus >> MSTATUS_UXL_SHIFT) & 3;
      assert(mxl == 2);
    }
  }
  s->priv = static_cast<riscv_priv>(priv);
}

static void set_mstatus(state_t *s, int64_t v) {
  int64_t mask = MSTATUS_MASK;
  s->mstatus = (s->mstatus & ~mask) | (v & mask);
  //csr_t c(s->mstatus);
  //std::cout << c.mstatus << "\n";
  //std::cout << "write mstatus = " << std::hex << s->mstatus << " @ " << s->pc << std::dec << "\n";
  
}
    


/* shitty dsheffie code */
std::ostream &operator<<(std::ostream &out, const state_t & s) {
  using namespace std;
  out << "PC : " << hex << s.last_pc << dec << "\n";
  for(int i = 0; i < 32; i++) {
    out << getGPRName(i) << " : 0x"
	<< hex << s.gpr[i] << dec
	<< "(" << s.gpr[i] << ")\n";
  }
  out << "icnt : " << s.icnt << "\n";
  return out;
}

static int64_t read_csr(int csr_id, state_t *s, bool &undef) {
  undef = false;
  switch(csr_id)
    {
    case 0x100:
      return s->mstatus & 0x3000de133UL;
    case 0x104:
      return s->mie & s->mideleg;
    case 0x105:
      return s->stvec;
    case 0x140:
      return s->sscratch;
    case 0x141:
      return s->sepc;
    case 0x142:
      return s->scause;
    case 0x143:
      return s->stval;
    case 0x144:
      return s->mip & s->mideleg;
    case 0x180:
      return s->satp;
    case 0x300:
      return s->mstatus;
    case 0x301: /* misa */
      return s->misa;
    case 0x302:
      return s->medeleg;
    case 0x303:
      return s->mideleg;
    case 0x304:
      return s->mie;
    case 0x305:
      return s->mtvec;
    case 0x306:
      return s->mcounteren;
    case 0x340:
      return s->mscratch;
    case 0x341:
      return s->mepc;
    case 0x342:
      return s->mcause;
    case 0x343:
      return s->mtvec;
    case 0x344:
      return s->mip;
    case 0x3a0:
      return s->pmpcfg0;
    case 0x3b0:
      return s->pmpaddr0;
    case 0x3b1:
      return s->pmpaddr1;      
    case 0x3b2:
      return s->pmpaddr2;
    case 0x3b3:
      return s->pmpaddr3;      
    case 0xc00:
      return s->icnt;
    case 0xc01:
      s->did_rdtime = 1;
      return s->get_time();
    case 0xc03:
      return 0;
    case 0xf11:
    case 0xf12:
    case 0xf13:
      return 0;
    case 0xf14:
      return s->mhartid;      
    default:
      //printf("rd csr id 0x%x unimplemented, pc %lx\n", csr_id, s->pc);
      undef = true;
      break;
    }
  return 0;
}

static void write_csr(int csr_id, state_t *s, int64_t v, bool &undef) {
  undef = false;
  csr_t c(v);
  switch(csr_id)
    {
    case 0x100:
      //printf("%lx writes %lx, old %lx\n", s->pc, v, s->mstatus);
      s->mstatus = (v & 0x000de133UL) | ((s->mstatus & (~0x000de133UL)));
      break;
    case 0x104:
      s->mie = (s->mie & ~(s->mideleg)) | (v & s->mideleg);
      break;
    case 0x105:
      s->stvec = v;
      break;
    case 0x106:
      s->scounteren = v;
      break;
    case 0x140:
      s->sscratch = v;
      break;
    case 0x141:
      s->sepc = v;
      break;
    case 0x142:
      s->scause = v;
      break;
    case 0x143:
      s->stvec = v;
      break;
    case 0x144:
      s->mip = (s->mip & ~(s->mideleg)) | (v & s->mideleg);      
      break;
    case 0x180:
      if(c.satp.mode == 8 &&
	 c.satp.asid == 0) {
	s->satp = v;
	////printf("tlb had %lu entries\n", tlb.size());
	//tlb.clear();	
      }
      break;
    case 0x300:
      set_mstatus(s,v);
      break;
    case 0x301:
      s->misa = v;
      break;
    case 0x302:
      s->medeleg = v;
      break;
    case 0x303:
      s->mideleg = v;
      break;
    case 0x304:
      s->mie = v;
      break;
    case 0x305:
      s->mtvec = v;
      break;
    case 0x306:
      s->mcounteren = v;
      break;
    case 0x340:
      s->mscratch = v;
      break;
    case 0x341:
      s->mepc = v;
      break;
    case 0x344:
      s->mip = v;
      break;
    case 0x3a0:
      s->pmpcfg0 = v;
      break;
    case 0x3b0:
      s->pmpaddr0 = v;
      break;
    case 0x3b1:
      s->pmpaddr1 = v;
      break;
    case 0x3b2:
      s->pmpaddr2 = v;
      break;
    case 0x3b3:
      s->pmpaddr3 = v;
      break;

      /* linux hacking */
    case 0xc03:
      std::cout << (char)(v&0xff);
      break;
    case 0xc04:
      //s->brk = v&1;
      //if(s->brk) {
      //std::cout << "you have panicd linux, game over\n";
      //}
      break;
    default:
      //printf("wr csr id 0x%x unimplemented\n", csr_id);
      undef = true;
      break;
    }
}


void execRiscv(state_t *s) {
  //printf("%s:%lx\n", __PRETTY_FUNCTION__, s->pc);
  uint8_t *mem = s->mem;
  int fetch_fault = 0, except_cause = -1;
  uint64_t tval = 0, tohost = 0,phys_pc = 0;
  uint32_t inst = 0, opcode = 0, rd = 0, lop = 0;
  int64_t irq = 0;
  riscv_t m(0);
  s->took_exception = false;
  s->did_rdtime = s->did_system = false;

  csr_t c(s->mie);
  //if(c.mie.mtie) {
  //printf("s->icnt = %lu\n", s->icnt);
  //exit(-1);
  //}
  if(s->get_time() >= s->mtimecmp) {
    csr_t cc(0);
    cc.mip.mtip = 1;
    //printf("setting interrupt pending at icnt %lu\n", s->icnt);
    s->mip |= cc.raw;
  }
  
  irq = take_interrupt(s);
  if(irq) {
    except_cause = CAUSE_INTERRUPT | irq;
    goto handle_exception;
  }
  

  phys_pc = s->translate(s->pc, fetch_fault, 4, false, true);
  //printf("translating %lx to %lx, fetch fault %d\n",
  //s->pc, phys_pc, fetch_fault);
  //if(s->pc == 0xffffffff80ba287cL) {
  //printf("linux panic, last call %lx\n", s->last_call);
  //dump_calls();
  //s->brk = 1;
  //}


  /* lightly modified from tinyemu */
  
  if(fetch_fault) {
    except_cause = CAUSE_FETCH_PAGE_FAULT;
    tval = s->pc;

    if(tval == last_tval) {
      abort();
    }
    last_tval = tval;
    //std::cout << csr_t(s->mstatus).mstatus << "\n";
    goto handle_exception;
  }
  assert(phys_pc < (1UL << 32));
  // assert(!fetch_fault);
  
  inst = s->load32(phys_pc);
  m.raw = inst;
  opcode = inst & 127;

  
  if(s->priv == priv_machine) {
    tohost = *reinterpret_cast<uint64_t*>(mem + globals::tohost_addr);
    if(tohost) {
      if(not(globals::syscall_emu)) {
	uint64_t dev = tohost >> 56;
	uint64_t cmd = (tohost >> 48) & 255;
	uint64_t payload = tohost & ((1UL<<48)-1);
	if(tohost == 1) { /* shutdown */
	  s->brk = 1;
	  return;
	}
	if(dev != 1) {
	  dump_calls();
	  abort();
	}
	
	if(cmd == 1) {
	  std::cout << static_cast<char>(payload & 0xff);
	  *reinterpret_cast<uint64_t*>(mem + globals::tohost_addr) = 0;
	  *reinterpret_cast<uint64_t*>(mem + globals::fromhost_addr) = (dev << 56) | (cmd << 48);
	}
	else {
	  abort();
	}
      }
      else {
	tohost &= ((1UL<<32)-1);      
	handle_syscall(s, tohost);
      }
    }
  }
  
  lop = (opcode & 3);
  rd = (inst>>7) & 31;

  if(s->gpr[0] != 0) {
    std::cout << "you broke the zero reg : last_pc = "
	      << std::hex << s->last_pc
	      << std::dec << "\n";
    abort();
  }


  if(globals::log) {
    std::cout << std::hex << s->pc << std::dec
	      << " : " << getAsmString(inst, s->pc)
	      << " , raw " << std::hex
	      << inst
	      << std::dec
	      << " , icnt " << s->icnt
	      << "\n";
  }
  s->last_pc = s->pc;  


  if(lop != 3) { /* compressed instructions generate exceptions */
    except_cause = CAUSE_ILLEGAL_INSTRUCTION;
    tval = s->pc;
    goto handle_exception;
  }

  switch(opcode)
    {
    case 0x3: {
      if(m.l.rd != 0) {
	int32_t disp = m.l.imm11_0;
	if((inst>>31)&1) {
	  disp |= 0xfffff000;
	}
	int64_t disp64 = disp;
	int64_t ea = ((disp64 << 32) >> 32) + s->gpr[m.l.rs1];
	int sz = 1<<(m.s.sel & 3);
	int page_fault = 0;
	
	bool unaligned = (ea & (sz-1)) != 0;
	if(unaligned) {
	  printf("unaligned load fault for ea %lx, sz = %d\n", ea, sz);
	  except_cause = CAUSE_MISALIGNED_LOAD;
	  tval = ea;
	  goto handle_exception;
	}

	int64_t pa = s->translate(ea, page_fault, sz);
	if(page_fault) {
	  except_cause = CAUSE_LOAD_PAGE_FAULT;
	  tval = ea;
	  goto handle_exception;
	}

	//if(s->pc == 0xffffffff8030db54UL) {
	// printf("ld for phys addr %lx, virt addr %lx\n", pa, ea);
	//}
	
	switch(m.s.sel)
	  {
	  case 0x0: /* lb */
	    s->gpr[m.l.rd] = s->load8(pa);
	    break;
	  case 0x1: /* lh */
	    s->gpr[m.l.rd] = s->load16(pa);
	    break;
	  case 0x2: /* lw */
	    s->sext_xlen( s->load32(pa), m.l.rd);
	    break;
	  case 0x3: /* ld */
	    s->gpr[m.l.rd] = s->load64(pa);
	    break;
	  case 0x4:/* lbu */
	    s->gpr[m.l.rd] = s->load8u(pa);
	    break;
	  case 0x5: /* lhu */
	    s->gpr[m.l.rd] = s->load16u(pa);	    
	    break;
	  case 0x6: /* lwu */
	    s->gpr[m.l.rd] = s->load32u(pa);	    	    
	    break;
	  default:
	    goto report_unimplemented;
	    assert(0);
	  }
	s->pc += 4;
	break;
      }
    }
    case 0xf:/* fence - there's a bunch of 'em */
      s->pc += 4;
      break;
    case 0x13: {
      int32_t simm32 = (inst >> 20);

      simm32 |= ((inst>>31)&1) ? 0xfffff000 : 0x0;
      int64_t simm64 = simm32;
      //sign extend!
      simm64 = (simm64 <<32) >> 32;
      uint32_t subop =(inst>>12)&7;
      uint32_t shamt = (inst>>20) & (s->xlen()-1);

      if(rd != 0) {
	switch(m.i.sel)
	  {
	  case 0: {/* addi */
	    s->sext_xlen(s->gpr[m.i.rs1] + simm64, rd);
	    break;
	  }
	  case 1: /* slli */
	    s->sext_xlen((*reinterpret_cast<uint64_t*>(&s->gpr[m.i.rs1])) << shamt, rd);
	    break;
	  case 2: /* slti */
	    s->gpr[rd] = (s->gpr[m.i.rs1] < simm64);
	    break;
	  case 3: { /* sltiu */
	    uint64_t uimm64 = static_cast<uint64_t>(simm64);
	    uint64_t u_rs1 = *reinterpret_cast<uint64_t*>(&s->gpr[m.i.rs1]);
	    s->gpr[rd] = (u_rs1 < uimm64);
	    break;
	  }
	  case 4: /* xori */
	    s->sext_xlen((s->gpr[m.i.rs1] ^ simm64), rd);
	    break;
	  case 5: { /* srli & srai */
	    uint32_t sel =  (inst >> 26) & 127;	    
	    if(sel == 0) { /* srli */
	      s->gpr[rd] = (*reinterpret_cast<uint64_t*>(&s->gpr[m.i.rs1]) >> shamt);
	    }
	    else if(sel == 16) { /* srai */
	      s->gpr[rd] = s->gpr[m.i.rs1] >> shamt;
	    }
	    else {
	      assert(0);
	    }
	    break;
	  }
	  case 6: /* ori */
	    s->sext_xlen((s->gpr[m.i.rs1] | simm64), rd);	    
	    break;
	  case 7: /* andi */
	    s->sext_xlen((s->gpr[m.i.rs1] & simm64), rd);
	    break;
	    
	  default:
	    std::cout << "implement case " << subop << "\n";
	    goto report_unimplemented;
	    assert(false);
	  }
      }
      s->pc += 4;
      break;
    }
    case 0x1b: {      
      if(rd != 0) {
	int32_t simm32 = (inst >> 20);
	simm32 |= ((inst>>31)&1) ? 0xfffff000 : 0x0;
	uint32_t shamt = (inst>>20) & 31;
	switch(m.i.sel)
	  {
	  case 0: {
	    int32_t r = simm32 + *reinterpret_cast<int32_t*>(&s->gpr[m.i.rs1]);
	    //std::cout << std::hex << simm32 << std::dec << "\n";
	    //std::cout << std::hex << s->gpr[m.i.rs1] << std::dec << "\n";
	    //std::cout << std::hex << r << std::dec << "\n";
	    s->sext_xlen(r, rd);
	    break;
	  }
	  case 1: { /*SLLIW*/
	    int32_t r = *reinterpret_cast<int32_t*>(&s->gpr[m.i.rs1]) << shamt;
	    s->sext_xlen(r, rd);
	    break;
	  }
	  case 5: { 
	    uint32_t sel =  (inst >> 25) & 127;
	    if(sel == 0) { /* SRLIW */
	      uint32_t r = *reinterpret_cast<uint32_t*>(&s->gpr[m.i.rs1]) >> shamt;
	      int32_t rr =  *reinterpret_cast<int32_t*>(&r);
	      //std::cout << std::hex << *reinterpret_cast<uint32_t*>(&s->gpr[m.i.rs1])
	      //<< std::dec << "\n";
	      //std::cout << "rr = " << std::hex << rr << std::dec << "\n";
	      s->sext_xlen(rr, rd);
	    }
	    else if(sel == 32){ /* SRAIW */
	      int32_t r = *reinterpret_cast<int32_t*>(&s->gpr[m.i.rs1]) >> shamt;
	      s->sext_xlen(r, rd);	      
	    }
	    else {
	      assert(0);
	    }
	    break;	    
	  }
	  default:
	    std::cout << m.i.sel << "\n";
	    goto report_unimplemented;
	    break;
	  }
      }
      s->pc += 4;
      break;
    }
    case 0x2f: {
      int page_fault = 0;
      uint64_t pa = 0;
      
      if(m.a.sel == 2) {
	switch(m.a.hiop)
	  {
	  case 0x0: {/* amoadd.w */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 4, true);
	    assert(!page_fault);
	    int32_t x = s->load32(pa);
	    s->store32(pa, s->gpr[m.a.rs2] + x);

	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]+x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]+x);
	      exit(-1);
	    }
	    atomic_queue.pop_front();

	    
	    if(m.a.rd != 0) { 
	      s->sext_xlen(x, m.a.rd);
	    }
	    break;
	  }
	  case 0x1: {/* amoswap.w */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 4,  true);
	    assert(!page_fault);
	    int32_t x = s->load32(pa);

	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]);
	      exit(-1);
	    }
	    atomic_queue.pop_front();

	    
	    s->store32(pa, s->gpr[m.a.rs2]);
	    
	    if(m.a.rd != 0) {
	      s->sext_xlen(x, m.a.rd);
	    }
	    break;
	  }
	  case 0x2: { /* lr.w */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 4);
	    assert(!page_fault);
	    if(m.a.rd != 0) {
	      s->sext_xlen(s->load32(pa), m.a.rd);
	      s->link = pa & (~15UL);
	    }
	    break;
	  }
	  case 0x3 : { /* sc.w */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 4, true);
	    assert(!page_fault);

	    assert(s->link == (pa & (~15UL)));
	    
	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == s->gpr[m.a.rs2])) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]);
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    
	    s->store32(pa, s->gpr[m.a.rs2]);
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = 0;
	    }
	    break;	    
	  }
	  case 0x4: { /* amoxor.w */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 4, true);
	    assert(!page_fault);
	    int64_t x = s->load32(pa);
	    s->store32(pa, s->gpr[m.a.rs2] ^ x);
	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]+x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]+x);
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    
	    if(m.a.rd != 0) {
	      s->sext_xlen(x, m.a.rd);
	    }
	    break;
	  }
	  case 0x8: {/* amoor.w */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 4, true);
	    assert(!page_fault);
	    int64_t x = s->load32(pa);
	    s->store32(pa, s->gpr[m.a.rs2] | x);
	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]+x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]+x);
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    
	    if(m.a.rd != 0) {
	      s->sext_xlen(x, m.a.rd);
	    }
	    break;
	  }	    
	  case 0x1c: {/* amomaxu.w */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 4, true);
	    assert(!page_fault);
	    uint32_t x = s->load32(pa);
	    uint32_t mm = *reinterpret_cast<uint32_t*>(&s->gpr[m.a.rs2]);
	    s->store32(pa, std::max(x,mm));
	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == std::max(mm,x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %x\n", s->pc, pa, std::max(mm,x));
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    
	    if(m.a.rd != 0) {
	      s->sext_xlen(x, m.a.rd);
	    }
	    break;
	  }	    

	    
	    
	  default:
	    std::cout << "m.a.hiop " << m.a.hiop << "\n";
	    assert(false);
	  }
      }
      else if(m.a.sel == 3) {
	switch(m.a.hiop)
	  {
	  case 0x0: {/* amoadd.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8, true);
	    assert(!page_fault);
	    int64_t x = s->load64(pa);

	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]+x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]+x);
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    
	    s->store64(pa, s->gpr[m.a.rs2] +x);
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = x;
	    }
	    break;
	  }
	  case 0x1: {/* amoswap.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8, true);
	    assert(!page_fault);
	    int64_t x = s->load64(pa);

	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]);
	      exit(-1);
	    }
	    atomic_queue.pop_front();

	    
	    s->store64(pa, s->gpr[m.a.rs2]);
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = x;
	    }
	    break;
	  }
	  case 0x2: { /* lr.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8);
	    assert(!page_fault);
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = s->load64(pa);
	      s->link = pa & (~15UL);
	    }
	    break;
	  }
	  case 0x3 : { /* sc.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8,  true);
	    if(page_fault) {
	      except_cause = CAUSE_STORE_PAGE_FAULT;
	      tval = s->gpr[m.a.rs1];
	      goto handle_exception;
	    }	    
	    bool succ = false;
	    
	    if(s->link == (pa & (~15UL))) {
	      succ = true;
	      assert(not(atomic_queue.empty()));
	      auto &t = atomic_queue.front();
	      if(not(t.pc == s->pc and t.addr == pa and t.data == s->gpr[m.a.rs2])) {
		printf("you have an atomic error\n");
		printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
		printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]);
		exit(-1);
	      }
	      atomic_queue.pop_front();
	      
	      s->store64( pa, s->gpr[m.a.rs2]);
	    }
	    
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = succ ? 0 : 1;
	    }
	    break;
	  }
	  case 0x4: {/* amoxor.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8, true);
	    assert(!page_fault);
	    int64_t x = s->load64(pa);

	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]|x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]|x);
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    s->store64(pa, s->gpr[m.a.rs2] ^ x);	    
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = x;
	    }
	    break;
	  }	    
	  case 0x8: {/* amoor.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8, true);
	    assert(!page_fault);
	    int64_t x = s->load64(pa);

	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]|x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]|x);
	      exit(-1);
	    }
	    atomic_queue.pop_front();

	    
	    s->store64(pa, s->gpr[m.a.rs2] | x);	    
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = x;
	    }
	    break;
	  }
	  case 0xc: {/* amoand.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8, true);
	    assert(!page_fault);
	    int64_t x = s->load64(pa);

	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == (s->gpr[m.a.rs2]&x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, s->gpr[m.a.rs2]&x);
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    
	    s->store64(pa, s->gpr[m.a.rs2] & x);
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = x;
	    }
	    break;
	  }
	  case 0x1c: {/* amomaxu.d */
	    pa = s->translate(s->gpr[m.a.rs1], page_fault, 8, true);
	    assert(!page_fault);
	    uint64_t x = s->load64(pa);
	    uint64_t mm = *reinterpret_cast<uint64_t*>(&s->gpr[m.a.rs2]);
	    
	    assert(not(atomic_queue.empty()));
	    auto &t = atomic_queue.front();
	    if(not(t.pc == s->pc and t.addr == pa and t.data == std::max(mm,x))) {
	      printf("you have an atomic error\n");
	      printf("rtl %lx, %lx, %lx\n", t.pc, t.addr, t.data);
	      printf("sim %lx, %lx, %lx\n", s->pc, pa, std::max(mm,x));
	      exit(-1);
	    }
	    atomic_queue.pop_front();
	    
	    s->store64(pa,  std::max(mm, x));
	    if(m.a.rd != 0) {
	      s->gpr[m.a.rd] = x;
	    }
	    break;
	  }
	    
	  default:
	    std::cout << "m.a.hiop " << std::hex << m.a.hiop << std::dec <<  "\n";
	    goto report_unimplemented;
	  }
      }
      else {	
	assert(false);
      }
      
      s->pc += 4;
      break;
    }
    case 0x3b: {
      if(m.r.rd != 0) {
	int32_t a = s->get_reg_i32(m.r.rs1), b = s->get_reg_i32(m.r.rs2);
	
	if((m.r.sel == 0) & (m.r.special == 0)) { /* addw */
	  int32_t c = a+b;
	  s->sext_xlen(c, m.r.rd);	 
	}
	else if((m.r.sel == 0) & (m.r.special == 1)) { /* mulw */
	  int32_t c = a*b;
	  s->sext_xlen(c, m.r.rd);
	}	
	else if((m.r.sel == 0) & (m.r.special == 32)) { /* subw */
	  int32_t c = a-b;
	  s->sext_xlen(c, m.r.rd);
	} 
	else if((m.r.sel == 1) & (m.r.special == 0)) { /* sllw */
	  int32_t c = a << (s->gpr[m.r.rs2]&31);
	  s->sext_xlen(c, m.r.rd);
	}
	else if((m.r.sel == 4) & (m.r.special == 1)) { /* divw */
	  int32_t c = b ? a/b : ~0;
	  s->sext_xlen(c, m.r.rd);
	}
	else if((m.r.sel == 5) & (m.r.special == 0)) { /* srlw */
	  uint32_t c = s->get_reg_u32(m.r.rs1) >> (s->gpr[m.r.rs2]&31);
	  int32_t rr =  *reinterpret_cast<int32_t*>(&c);	  
	  s->sext_xlen(rr, m.r.rd);	  
	}
	else if((m.r.sel == 5) & (m.r.special == 1)) { /* divuw */
	  uint32_t aa = s->get_reg_u32(m.r.rs1);
	  uint32_t bb = s->get_reg_u32(m.r.rs2);
	  uint32_t c = bb ? aa/bb : ~(0U);
	  int32_t rr =  *reinterpret_cast<int32_t*>(&c);
	  s->sext_xlen(rr, m.r.rd);
	}
	else if((m.r.sel == 5) & (m.r.special == 32)) { /* sraw */
	  int32_t c = a >> (s->gpr[m.r.rs2]&31);
	  s->sext_xlen(c, m.r.rd);	  
	}
	else if((m.r.sel == 6) & (m.r.special == 1)) { /* remw */
	  int32_t c = b ? a % b : ~0;
	  s->sext_xlen(c, m.r.rd);
	}
	else if((m.r.sel == 7) & (m.r.special == 1)) { /* remuw */
	  uint32_t aa = s->get_reg_u32(m.r.rs1);
	  uint32_t bb = s->get_reg_u32(m.r.rs2);
	  uint32_t c = bb ? aa%bb : ~(0U);
	  int32_t rr =  *reinterpret_cast<int32_t*>(&c);
	  s->sext_xlen(rr, m.r.rd);
	}
	else {
	  std::cout << "special = " << m.r.special << "\n";
	  std::cout << "sel = " << m.r.sel << "\n";
	  goto report_unimplemented;
	}
	
      }
      s->pc += 4;
      break;
    }      
    case 0x23: {
      int32_t disp = m.s.imm4_0 | (m.s.imm11_5 << 5);
      disp |= ((inst>>31)&1) ? 0xfffff000 : 0x0;
      int64_t disp64 = disp;
      int64_t ea = ((disp64 << 32) >> 32) + s->gpr[m.s.rs1];
      int fault;

      int sz = 1<<(m.s.sel);
      int64_t pa = s->translate(ea, fault, sz, true);

      bool unaligned = (ea & (sz-1)) != 0;
      if(unaligned) {
	printf("unaligned store fault for ea %lx, sz = %d\n", ea, sz);
	except_cause = CAUSE_MISALIGNED_STORE;
	tval = ea;
	goto handle_exception;
      }

      
      if(fault) {
	except_cause = CAUSE_STORE_PAGE_FAULT;
	tval = ea;
	goto handle_exception;
      }

      //if(s->pc == 0xffffffff8030db50UL) {
      //printf("storing %lx to phys addr %lx\n", s->gpr[m.s.rs2], pa);
      //}

      switch(m.s.sel)
	{
	case 0x0: /* sb */
	  s->store8(pa, s->gpr[m.s.rs2]);
	  break;
	case 0x1: /* sh */
	  s->store16(pa, s->gpr[m.s.rs2]);
	  break;
	case 0x2: /* sw */
	  s->store32(pa, s->gpr[m.s.rs2]);
	  break;
	case 0x3: /* sd */
	  s->store64(pa, s->gpr[m.s.rs2]);
	  break;
	default:
	  assert(0);
	}

      //printf("%lx, %lx, %lx\n", s->pc, pa, s->gpr[m.s.rs2]);
      if(not(pa >= UC_START_ADDR and pa < UC_END_ADDR and false)) {
	store_queue.emplace_back(s->pc, pa, s->gpr[m.s.rs2]);
      }
      s->pc += 4;
      break;
    }

      
      //imm[31:12] rd 011 0111 LUI
    case 0x37:
      if(rd != 0) {
	int32_t imm32 = inst & 0xfffff000;
	s->sext_xlen(imm32, rd);
      }
      s->pc += 4;
      break;
      //imm[31:12] rd 0010111 AUIPC
    case 0x17: /* is this sign extended */
      if(rd != 0) {
	int64_t imm = inst & (~4095);
	imm = (imm << 32) >> 32;
	int64_t y = s->pc + imm;
	s->sext_xlen(y, rd);
      }
      s->pc += 4;
      break;
      
      //imm[11:0] rs1 000 rd 1100111 JALR
    case 0x67: {
      int32_t tgt = m.jj.imm11_0;
      tgt |= ((inst>>31)&1) ? 0xfffff000 : 0x0;
      int64_t tgt64 = tgt;
      tgt64 = (tgt64<<32)>>32;
      tgt64 += s->gpr[m.jj.rs1];
      tgt64 &= ~(1UL);
      if(m.jj.rd != 0) {
	s->gpr[m.jj.rd] = s->pc + 4;
      }
      //std::cout << "target = " << std::hex << tgt64 << std::dec << "\n";
      s->last_call = s->pc;
      bool rs1_is_link = m.jj.rs1==1 or m.jj.rs1==5;
      bool rd_is_link = m.jj.rd==1 or m.jj.rd==5;
      
      if((m.jj.rd == 0) and rs1_is_link) {
	if(!calls.empty())
	  calls.pop();
      }
      if(rd_is_link) {
	calls.push(s->pc);
      }
      
      s->pc = tgt64;
      break;
    }

      
      //imm[20|10:1|11|19:12] rd 1101111 JAL
    case 0x6f: {
      int32_t jaddr32 =
	(m.j.imm10_1 << 1)   |
	(m.j.imm11 << 11)    |
	(m.j.imm19_12 << 12) |
	(m.j.imm20 << 20);
      jaddr32 |= ((inst>>31)&1) ? 0xffe00000 : 0x0;
      int64_t jaddr = jaddr32;
      jaddr = (jaddr << 32) >> 32;
      if(rd != 0) {
	s->gpr[rd] = s->pc + 4;
      }
      s->last_call = s->pc;
      bool rd_is_link = rd==1 or rd==5;      
      if(rd_is_link) {
	calls.push(s->pc);
      }
      s->pc += jaddr;
      break;
    }
    case 0x33: {      
      if(m.r.rd != 0) {
	uint64_t u_rs1 = *reinterpret_cast<uint64_t*>(&s->gpr[m.r.rs1]);
	uint64_t u_rs2 = *reinterpret_cast<uint64_t*>(&s->gpr[m.r.rs2]);
	switch(m.r.sel)
	  {
	  case 0x0: /* add & sub */
	    switch(m.r.special)
	      {
	      case 0x0: /* add */
		s->sext_xlen(s->gpr[m.r.rs1] + s->gpr[m.r.rs2], m.r.rd);
		break;
	      case 0x1: /* mul */
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] * s->gpr[m.r.rs2];
		break;
	      case 0x20: /* sub */
		s->sext_xlen(s->gpr[m.r.rs1] - s->gpr[m.r.rs2], m.r.rd);		
		break;
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		assert(0);
	      }
	    break;
	  case 0x1: /* sll */
	    switch(m.r.special)
	      {
	      case 0x0:
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] << (s->gpr[m.r.rs2] & (s->xlen()-1));
		break;
	      case 0x1: { /* MULH */
		__int128 t = static_cast<__int128>(s->gpr[m.r.rs1]) * static_cast<__int128>(s->gpr[m.r.rs2]);
		s->gpr[m.r.rd] = (t>>64);
		break;
	      }
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		std::cout << std::hex << s->pc << std::dec << "\n";
		assert(0);
	      }
	    break;
	  case 0x2: /* slt */
	    switch(m.r.special)
	      {
	      case 0x0:
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] < s->gpr[m.r.rs2];
		break;
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		assert(0);		
	      }
	    break;
	  case 0x3: /* sltu */
	    switch(m.r.special)
	      {
	      case 0x0:
		s->gpr[m.r.rd] = u_rs1 < u_rs2;
		break;
	      case 0x1: {/* MULHU */
		__uint128_t t = static_cast<__uint128_t>(u_rs1) * static_cast<__uint128_t>(u_rs2);
		*reinterpret_cast<uint64_t*>(&s->gpr[m.r.rd]) = (t>>64);
		break;
	      }
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		std::cout << "pc = " << std::hex << s->pc << std::dec << "\n";
		assert(0);		
	      }
	    break;
	  case 0x4:
	    switch(m.r.special)
	      {
	      case 0x0:
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] ^ s->gpr[m.r.rs2];
		break;
	      case 0x1:
		s->gpr[m.r.rd] = s->gpr[m.r.rs2] ? s->gpr[m.r.rs1] / s->gpr[m.r.rs2] : ~(0L);
		break;
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		assert(0);		
	      }
	    break;		
	  case 0x5: /* srl & sra */
	    switch(m.r.special)
	      {
	      case 0x0: /* srl */
		s->gpr[rd] = (*reinterpret_cast<uint64_t*>(&s->gpr[m.r.rs1]) >> (s->gpr[m.r.rs2] & (s->xlen()-1)));
		break;
	      case 0x1: {
		*reinterpret_cast<uint64_t*>(&s->gpr[m.r.rd]) = u_rs2 ? u_rs1 / u_rs2 : ~(0UL);
		break;
	      }
	      case 0x7: {
		s->gpr[m.r.rd] = s->gpr[m.r.rs2]==0 ? 0 : s->gpr[m.r.rs1];
		break;
	      }		
	      case 0x20: /* sra */
		s->gpr[rd] = s->gpr[m.r.rs1] >> (s->gpr[m.r.rs2] & (s->xlen()-1));
		break;
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		assert(0);				
	      }
	    break;
	  case 0x6:
	    switch(m.r.special)
	      {
	      case 0x0:
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] | s->gpr[m.r.rs2];
		break;
	      case 0x1:
		s->gpr[m.r.rd] = s->gpr[m.r.rs2] ? s->gpr[m.r.rs1] % s->gpr[m.r.rs2] : ~(0L);
		break;		
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		assert(0);
	      }
	    break;
	  case 0x7:
	    switch(m.r.special)
	      {
	      case 0x0:
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] & s->gpr[m.r.rs2];
		break;
	      case 0x1: { /* remu */
		*reinterpret_cast<uint64_t*>(&s->gpr[m.r.rd]) = u_rs2 ? u_rs1 % u_rs2 : ~(0UL);
		//std::cout << std::hex << u_rs1 << std::dec << "\n";
		//std::cout << std::hex << u_rs2 << std::dec << "\n";
		//std::cout << std::hex << (u_rs1 % u_rs2) << std::dec << "\n";
		break;
	      }
	      case 0x7: {
		s->gpr[m.r.rd] = s->gpr[m.r.rs2]!=0 ? 0 : s->gpr[m.r.rs1];
		break;
	      }
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
		assert(0);
	      }
	    break;
	  default:
	    std::cout << "implement = " << m.r.sel << "\n";
	    assert(0);
	  }
      }
      s->pc += 4;
      break;
    }
#if 0
    imm[12|10:5] rs2 rs1 000 imm[4:1|11] 1100011 BEQ
    imm[12|10:5] rs2 rs1 001 imm[4:1|11] 1100011 BNE
    imm[12|10:5] rs2 rs1 100 imm[4:1|11] 1100011 BLT
    imm[12|10:5] rs2 rs1 101 imm[4:1|11] 1100011 BGE
    imm[12|10:5] rs2 rs1 110 imm[4:1|11] 1100011 BLTU
    imm[12|10:5] rs2 rs1 111 imm[4:1|11] 1100011 BGEU
#endif
    case 0x63: {
      int32_t disp32 =
	(m.b.imm4_1 << 1)  |
	(m.b.imm10_5 << 5) |	
        (m.b.imm11 << 11)  |
        (m.b.imm12 << 12);
      disp32 |= m.b.imm12 ? 0xffffe000 : 0x0;
      int64_t disp = disp32;
      disp = (disp << 32) >> 32;
      bool takeBranch = false;
      uint64_t u_rs1 = *reinterpret_cast<uint64_t*>(&s->gpr[m.b.rs1]);
      uint64_t u_rs2 = *reinterpret_cast<uint64_t*>(&s->gpr[m.b.rs2]);
      switch(m.b.sel)
	{
	case 0: /* beq */
	  takeBranch = s->gpr[m.b.rs1] == s->gpr[m.b.rs2];
	  break;
	case 1: /* bne */
	  takeBranch = s->gpr[m.b.rs1] != s->gpr[m.b.rs2];
	  break;
	case 4: /* blt */
	  takeBranch = s->gpr[m.b.rs1] < s->gpr[m.b.rs2];
	  break;
	case 5: /* bge */
	  takeBranch = s->gpr[m.b.rs1] >= s->gpr[m.b.rs2];	  
	  break;
	case 6: /* bltu */
	  takeBranch = u_rs1 < u_rs2;
	  break;
	case 7: /* bgeu */
	  takeBranch = u_rs1 >= u_rs2;
	  //std::cout << "s->pc " << std::hex << s->pc << ", rs1 " << u_rs1 << ", rs2 "
	  //<< u_rs2 << std::dec
	  //	    << ", takeBranch " << takeBranch
	  //<< "\n";

	  break;
	default:
	  std::cout << "implement case " << m.b.sel << "\n";
	  assert(0);
	}
      //assert(not(takeBranch));
      s->pc = takeBranch ? disp + s->pc : s->pc + 4;
      break;
    }

    case 0x73: {
      uint32_t csr_id = (inst>>20);
      bool is_ecall = ((inst >> 7) == 0);
      bool is_ebreak = ((inst>>7) == 0x2000);
      bool bits19to7z = (((inst >> 7) & 8191) == 0);
      uint64_t upper7 = (inst>>25);
      s->did_system = 1;
      if(is_ecall) { /* ecall and ebreak dont increment the retired instruction count */
	if(globals::syscall_emu) {
	  s->brk = 1;
	}
	else {
	  except_cause = CAUSE_USER_ECALL + static_cast<int>(s->priv);
	  goto handle_exception;
	}
      }
      else if(upper7 == 9 && ((inst & (16384-1)) == 0x73 )) {
	//std::cout << "warn : got sfence\n";
	//printf("tlb had %lu entries\n", tlb.size());	
	//tlb.clear();
      }      
      else if(bits19to7z and (csr_id == 0x002)) {  /* uret */
	assert(false);
      }
      else if(bits19to7z and (csr_id == 0x102)) {  /* sret */
	/* stolen from tinyemu */
	int spp = (s->mstatus >> MSTATUS_SPP_SHIFT) & 1;
	/* set the IE state to previous IE state */
	int spie = (s->mstatus >> MSTATUS_SPIE_SHIFT) & 1;
	s->mstatus = (s->mstatus & ~(1 << spp)) | (spie << spp);
	/* set SPIE to 1 */
	s->mstatus |= MSTATUS_SPIE;
	/* set SPP to U */
	s->mstatus &= ~MSTATUS_SPP;
	set_priv(s, spp);
	s->pc = s->sepc;
	break;
      }
      else if(bits19to7z and (csr_id == 0x105)) {  /* wfi */
	s->pc += 4;
	break;
      }
      else if(bits19to7z and (csr_id == 0x202)) {  /* hret */
	assert(false);
      }            
      else if(bits19to7z and (csr_id == 0x302)) {  /* mret */
	/* stolen from tinyemu */
	int mpp = (s->mstatus >> MSTATUS_MPP_SHIFT) & 3;
	/* set the IE state to previous IE state */
	int mpie = (s->mstatus >> MSTATUS_MPIE_SHIFT) & 1;
	//printf("mpp = %d, mpie = %d\n", mpp, mpie);
	int64_t old = s->mstatus;
	s->mstatus = (s->mstatus & ~(1 << mpp)) |(mpie << mpp);
	/* set MPIE to 1 */
	s->mstatus |= MSTATUS_MPIE;
	/* set MPP to U */
	s->mstatus &= ~MSTATUS_MPP;
	//printf("mret mstatus %lx, old %lx\n", s->mstatus, old);
	//exit(-1);
	set_priv(s, mpp);
	s->pc = s->mepc;
	break;
      }
      else if(is_ebreak) {
	if(not(globals::syscall_emu)) {
	  except_cause = CAUSE_BREAKPOINT;
	  goto handle_exception;
	}
      }
      else {
	int rd = (inst>>7) & 31;
        int rs = (inst >> 15) & 31;
	bool undef=false;
	switch((inst>>12) & 7)
	  {
	  case 1: { /* CSRRW */
	    int64_t v = 0;
	    if(rd != 0) {
	      v = read_csr(csr_id, s, undef);
	      if(undef) goto report_unimplemented;
	    }
	    write_csr(csr_id, s, s->gpr[rs], undef);
	    if(undef) goto report_unimplemented;
	    if(rd != 0) {
	      s->gpr[rd] = v;
	    }
	    break;
	  }
	  case 2: {/* CSRRS */
	    int64_t t = read_csr(csr_id, s, undef);
	    if(undef) goto report_unimplemented;
	    if(rs != 0) {
	      write_csr(csr_id, s, t | s->gpr[rs], undef);
	      if(undef) goto report_unimplemented;
	    }
	    if(rd != 0) {
	      s->gpr[rd] = t;
	    }
	    break;
	  }
	  case 3: {/* CSRRC */
	    int64_t t = read_csr(csr_id, s,undef);
	    if(undef) goto report_unimplemented;	    
	    if(rs != 0) {
	      write_csr(csr_id, s, t & (~s->gpr[rs]), undef);
	      if(undef) goto report_unimplemented;	      
	    }
	    if(rd != 0) {
	      s->gpr[rd] = t;
	    }
	    break;
	  }
	  case 5: {/* CSRRWI */
	    int64_t t = 0;
	    if(rd != 0) {
	      t = read_csr(csr_id, s, undef);
	      if(undef) goto report_unimplemented;
	    }
	    write_csr(csr_id, s, rs, undef);
	    if(undef) goto report_unimplemented;
	    if(rd != 0) {
	      s->gpr[rd] = t;
	    }
	    break;
	  }
	  case 6:{ /* CSRRSI */
	    int64_t t = read_csr(csr_id,s,undef);
	    if(undef) goto report_unimplemented;
	    if(rs != 0) {
	      write_csr(csr_id, s, t | rs, undef);
	      if(undef) goto report_unimplemented;
	    }
	    if(rd != 0) {
	      s->gpr[rd] = t;
	    }
	    break;
	  }
	  case 7: {/* CSRRCI */
	    int64_t t = read_csr(csr_id, s, undef);
	    if(undef) goto report_unimplemented;	    
	    if(rs != 0) {
	      write_csr(csr_id, s, t & (~rs), undef);
	      if(undef) goto report_unimplemented;	      
	    }
	    if(rd != 0) {
	      s->gpr[rd] = t;
	    }
	    break;
	  }
	  default:
	    goto report_unimplemented;	    
	  }
      }
      s->pc += 4;
      break;
    }
      
    default:
      std::cout << std::hex << s->pc << std::dec
		<< " : " << getAsmString(inst, s->pc)
		<< " , opcode "
		<< std::hex
		<< opcode
		<< " , insn "
		<< inst
		<< std::dec
		<< " , icnt " << s->icnt
		<< "\n";
      std::cout << *s << "\n";
      exit(-1);
      break;
    }

  s->icnt++;
  return;

 report_unimplemented:
  printf("taking csr exception at pc %lx\n", s->pc);
  except_cause = CAUSE_ILLEGAL_INSTRUCTION;
  tval = s->pc;
  
 handle_exception: {
    s->took_exception = true;
    bool delegate = false;
    if(s->priv == priv_user || s->priv == priv_supervisor) {
      if(except_cause & CAUSE_INTERRUPT) {
	uint32_t cc = (except_cause & 0x7fffffffUL);
	delegate = ((s->mideleg) >> cc) & 1;	
      }
      else {
	delegate = (s->medeleg >> except_cause) & 1;
	
      }
    }
    auto oldpc = s->pc;
    if(delegate) {
      s->scause = except_cause & 0x7fffffff;
      s->sepc = s->pc;
      s->stval = tval;
      s->mstatus = (s->mstatus & ~MSTATUS_SPIE) |
	(((s->mstatus >> s->priv) & 1) << MSTATUS_SPIE_SHIFT);
      s->mstatus = (s->mstatus & ~MSTATUS_SPP) |
	(s->priv << MSTATUS_SPP_SHIFT);
      s->mstatus &= ~MSTATUS_SIE;
      set_priv(s, priv_supervisor);
      s->pc = s->stvec;
    }
    else {
      auto old = s->mstatus;
      s->mcause = except_cause & 0x7fffffff;
      s->mepc = s->pc;
      s->mtval = tval;
      s->mstatus = (s->mstatus & ~MSTATUS_MPIE) |
	(((s->mstatus >> s->priv) & 1) << MSTATUS_MPIE_SHIFT);
      s->mstatus = (s->mstatus & ~MSTATUS_MPP) |
	(s->priv << MSTATUS_MPP_SHIFT);
      s->mstatus &= ~MSTATUS_MIE;
      set_priv(s, priv_machine);
      s->pc = s->mtvec;
      //printf("new mstatus %lx, old %lx\n", s->mstatus, old);
      //exit(-1);
    }
    //printf("CHECKER: exception at %lx, cause %d, new pc %lx\n",
    //oldpc, except_cause, s->pc);
    //exit(-1);
    //printf("after exception, new pc will be %lx\n", s->pc);
  }
  return;
  
}

void runRiscv(state_t *s, uint64_t dumpIcnt) {
  while(s->brk==0 and (s->icnt < s->maxicnt) and (s->icnt < dumpIcnt)) {
    execRiscv(s);
  }
}

