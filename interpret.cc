#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/utsname.h>


#include "interpret.hh"
#include "disassemble.hh"
#include "helper.hh"
#include "globals.hh"

static fpMode currFpMode = fpMode::mipsii;

state_t::~state_t() {
  //std::cout << mem.bytes_allocated() << " bytes present in memory image\n";
  delete &mem;
}

static void execSpecial2(uint32_t inst, state_t *s);
static void execSpecial3(uint32_t inst, state_t *s);
static void execCoproc0(uint32_t inst, state_t *s);
static void execCoproc2(uint32_t inst, state_t *s);

template <bool EL> void execMips(state_t *s);

void execMips(state_t *s) {
  execMips<IS_LITTLE_ENDIAN>(s);
}

#if 1
std::ostream &operator<<(std::ostream &out, const state_t & s) {
  using namespace std;
  for(int i = 0; i < 32; i++) {
    out << getGPRName(i) << " : 0x"
	<< hex << s.gpr[i] << dec
	<< "(" << s.gpr[i] << ")\n";
  }
#if 0
  for(int i = 0; i < 32; i++) {
    out << "cpr0_" << i << " : 0x"
	<< hex << s.cpr0[i] << dec
	<< "\n";
  }
  for(int i = 0; i < 32; i++) {
    out << "cpr1_" << i << " : 0x"
	<< hex << s.cpr1[i] << dec
	<< "\n";
  }
  for(int i = 0; i < 5; i++) {
    out << "fcr" << i << " : 0x"
	<< hex << s.fcr1[i] << dec
	<< "\n";
  }
#endif
  out << "icnt : " << s.icnt << "\n";
  return out;
}
#endif

static uint32_t getConditionCode(state_t *s, uint32_t cc);
static void setConditionCode(state_t *s, uint32_t v, uint32_t cc);


/* IType instructions */
static void _lb(uint32_t inst, state_t *s);
static void _lbu(uint32_t inst, state_t *s);
static void _sb(uint32_t inst, state_t *s);


static void _mtc1(uint32_t inst, state_t *s);
static void _mfc1(uint32_t inst, state_t *s);

static void _sc(uint32_t inst, state_t *s);

/* FLOATING-POINT */
static void _c(uint32_t inst, state_t *s);

static void _cvts(uint32_t inst, state_t *s);
static void _cvtd(uint32_t inst, state_t *s);

static void _truncw(uint32_t inst, state_t *s);
static void _truncl(uint32_t inst, state_t *s);

static void _movci(uint32_t inst, state_t *s);

static void _fmovc(uint32_t inst, state_t *s);
static void _fmovn(uint32_t inst, state_t *s);
static void _fmovz(uint32_t inst, state_t *s);


static void _movcs(uint32_t inst, state_t *s);
static void _movcd(uint32_t inst, state_t *s);

static void _movnd(uint32_t inst, state_t *s);
static void _movns(uint32_t inst, state_t *s);
static void _movzd(uint32_t inst, state_t *s);
static void _movzs(uint32_t inst, state_t *s);

void initState(state_t *s) {
  /* setup the status register */
  s->cpr0[12] |= 1<<2;
  s->cpr0[12] |= 1<<22;
}

static uint32_t getConditionCode(state_t *s, uint32_t cc) {
  return ((s->fcr1[CP1_CR25] & (1U<<cc)) >> cc) & 0x1;
}

static void setConditionCode(state_t *s, uint32_t v, uint32_t cc) {
  uint32_t m0,m1,m2;
  m0 = 1U<<cc;
  m1 = ~m0;
  m2 = ~(v-1);
  s->fcr1[CP1_CR25] = (s->fcr1[CP1_CR25] & m1) | ((1U<<cc) & m2);
}


void mkMonitorVectors(state_t *s) {
  for (uint32_t loop = 0; (loop < IDT_MONITOR_SIZE); loop += 4) {
      uint32_t vaddr = IDT_MONITOR_BASE + loop;
      uint32_t insn = (RSVD_INSTRUCTION |
		       (((loop >> 2) & RSVD_INSTRUCTION_ARG_MASK)
			<< RSVD_INSTRUCTION_ARG_SHIFT));
      s->mem.set<uint32_t>(vaddr, globals::isMipsEL ? bswap<true,uint32_t>(insn) : bswap<false,uint32_t>(insn));

  }
}




static void execSpecial2(uint32_t inst,state_t *s) {
  uint32_t funct = inst & 63; 
  uint32_t rs = (inst >> 21) & 31;
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rd = (inst >> 11) & 31;

  switch(funct)
    {
    case(0x0): /* madd */ {
      int64_t y,acc;
      acc = ((int64_t)s->hi) << 32;
      acc |= ((int64_t)s->lo);
      y = (int64_t)s->gpr[rs] * (int64_t)s->gpr[rt];
      y += acc;
      s->lo = (int32_t)(y & 0xffffffff);
      s->hi = (int32_t)(y >> 32);
      s->insn_histo[mipsInsn::MADD]++;
      break;
    }
    case 0x1: /* maddu */ {
      uint64_t y,acc;
      uint32_t u0 = *((uint32_t*)&s->gpr[rs]);
      uint32_t u1 = *((uint32_t*)&s->gpr[rt]);
      uint64_t uk0 = (uint64_t)u0;
      uint64_t uk1 = (uint64_t)u1;
      y = uk0*uk1;
      acc = ((uint64_t)s->hi) << 32;
      acc |= ((uint64_t)s->lo);
      y += acc;
      s->lo = (uint32_t)(y & 0xffffffff);
      s->hi = (uint32_t)(y >> 32);
      s->insn_histo[mipsInsn::MADDU]++;
      break;
    }
    case(0x2): /* mul */{
      int64_t y = ((int64_t)s->gpr[rs]) * ((int64_t)s->gpr[rt]);
      //printf("multiply: %x x %x -> %x\n", s->gpr[rs], s->gpr[rt], y);
      s->gpr[rd] = (int32_t)y;
      s->insn_histo[mipsInsn::MUL]++;
      break;
    }
    case(0x4): /* msub */ {
      int64_t y,acc;
      acc = ((int64_t)s->hi) << 32;
      acc |= ((int64_t)s->lo);
      y = (int64_t)s->gpr[rs] * (int64_t)s->gpr[rt];
      y = acc - y;
      s->lo = (int32_t)(y & 0xffffffff);
      s->hi = (int32_t)(y >> 32);
      s->insn_histo[mipsInsn::MSUB]++;
      break;
    }
    case(0x20): /* clz */
      s->gpr[rd] = (s->gpr[rs]==0) ? 32 : __builtin_clz(s->gpr[rs]);
      s->insn_histo[mipsInsn::CLZ]++;
      break;
    default:
      printf("unhandled special2 instruction @ 0x%08x\n", s->pc); 
      exit(-1);
      break;
    }
  s->pc += 4;
}

static void execSpecial3(uint32_t inst,state_t *s) {
  uint32_t funct = inst & 63;
  uint32_t op = (inst>>6) & 31;
  uint32_t rt = (inst >> 16) & 31; 
  uint32_t rs = (inst >> 21) & 31;
  uint32_t rd = (inst >> 11) & 31;
  if(funct == 32) {
    switch(op)
      {
      case 0x10: /* seb */
	s->gpr[rd] = (int32_t)((int8_t)s->gpr[rt]);
	s->insn_histo[mipsInsn::SEB]++;
	break;
      case 0x18: /* seh */
	s->gpr[rd] = (int32_t)((int16_t)s->gpr[rt]);
	s->insn_histo[mipsInsn::SEH]++;
	break;
      default:
	printf("unhandled special3 instruction @ 0x%08x, opcode = %x\n", s->pc, funct); 
	exit(-1);    
	break;
      }
  }
  else if(funct == 0) { /* ext */  
    uint32_t pos = (inst >> 6) & 31;
    uint32_t size = ((inst >> 11) & 31) + 1;
    s->gpr[rt] = (s->gpr[rs] >> pos) & ((1<<size)-1);
    s->insn_histo[mipsInsn::EXT]++;
  }
  else if(funct == 0x4) {/* ins */
    uint32_t size = rd-op+1;
    uint32_t mask = (1U<<size) -1;
    uint32_t cmask = ~(mask << op);
    uint32_t v = (s->gpr[rs] & mask) << op;
    uint32_t c = (s->gpr[rt] & cmask) | v;
    s->gpr[rt] = c;
    s->insn_histo[mipsInsn::INS]++;    
  }
  else if(funct == 0x3b) { /* rdhwr */
    switch(rd)
      {
      case 29:
	s->gpr[rt] = s->cpr0[29];
	s->insn_histo[mipsInsn::RDHWR]++;
	break;
      default:
	abort();
      }
  }
  else {
    printf("unhandled special3 instruction @ 0x%08x\n", s->pc); 
    exit(-1);    
  }
  s->pc += 4;
}


template <typename T>
struct c1xExec {
  void operator()(const coproc1x_t& insn, state_t *s) {
    T _fr = *reinterpret_cast<T*>(s->cpr1+insn.fr);
    T _fs = *reinterpret_cast<T*>(s->cpr1+insn.fs);
    T _ft = *reinterpret_cast<T*>(s->cpr1+insn.ft);
    T &_fd = *reinterpret_cast<T*>(s->cpr1+insn.fd);  
    switch(insn.id)
      {
      case 4:
	_fd = _fs*_ft + _fr;
	break;
      case 5:
	_fd = _fs*_ft - _fr;
	break;
      default:
	std::cerr << "unhandled coproc1x insn @ 0x"
		  << std::hex << s->pc << std::dec
		  << ", id = " << insn.id
		  <<"\n";
	exit(-1);
      }
    s->pc += 4;
  }
};


template <bool EL, typename T>
void lxc1(uint32_t inst, state_t *s) {
  mips_t mi(inst);
  uint32_t ea = s->gpr[mi.lc1x.base] + s->gpr[mi.lc1x.index];
  *reinterpret_cast<T*>(s->cpr1 + mi.lc1x.fd) = bswap<EL>(s->mem.get<T>(ea));
  s->pc += 4;
}

template <bool EL>
static void execCoproc1x(uint32_t inst, state_t *s) {
  mips_t mi(inst);

  switch(mi.lc1x.id)
    {
    case 0:
      //lwxc1
      lxc1<EL,int32_t>(inst, s);
      return;
    case 1:
      //ldxc1
      lxc1<EL,int64_t>(inst, s);
      return;
    default:
      break;
    }
  
  switch(mi.c1x.fmt)
   {
   case 0: {
     c1xExec<float> e;
     e(mi.c1x, s);
     return;
   }
   case 1: {
     c1xExec<double> e;
     e(mi.c1x, s);
     return;
   }
   default:
     std::cerr << "weird type in do_c1x_op @ 0x"
	       << std::hex << s->pc << std::dec
	       <<"\n";
     exit(-1);
   }
}



template <bool EL, branch_type bt>
void branch(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = ((int32_t)himm) << 2;
  uint32_t npc = s->pc+4; 
  bool isLikely = false, takeBranch = false, saveReturn = false;
  switch(bt)
    {
    case branch_type::beql:
      takeBranch = (s->gpr[rt] == s->gpr[rs]);
      s->insn_histo[mipsInsn::BEQL]++;
      isLikely = true;
      break;
    case branch_type::beq:
      takeBranch = (s->gpr[rt] == s->gpr[rs]);
      s->insn_histo[mipsInsn::BEQ]++;
      break;
    case branch_type::bnel:
      isLikely = true;
      takeBranch = (s->gpr[rt] != s->gpr[rs]);
      s->insn_histo[mipsInsn::BNEL]++;
      break;
    case branch_type::bne:
      takeBranch = (s->gpr[rt] != s->gpr[rs]);
      s->insn_histo[mipsInsn::BNE]++;
      break;
    case branch_type::blezl:
      isLikely = true;
      takeBranch = (s->gpr[rs] <= 0);
      s->insn_histo[mipsInsn::BLEZL]++;
      break;
    case branch_type::blez:
      takeBranch = (s->gpr[rs] <= 0);
      s->insn_histo[mipsInsn::BLEZ]++;
      break;
    case branch_type::bgtzl:
      isLikely = true;
      takeBranch = (s->gpr[rs] > 0);
      s->insn_histo[mipsInsn::BGTZL]++;
      break;
    case branch_type::bgtz:
      takeBranch = (s->gpr[rs] > 0);
      s->insn_histo[mipsInsn::BGTZ]++;
      break;
    case branch_type::bgezl:
      isLikely = true;
      takeBranch = (s->gpr[rs] >= 0);
      s->insn_histo[mipsInsn::BGEZL]++;
      break;      
    case branch_type::bgez:
      takeBranch = (s->gpr[rs] >= 0);
      s->insn_histo[mipsInsn::BGEZ]++;
      break;
    case branch_type::bltzl:
      isLikely = true;
      takeBranch = (s->gpr[rs] < 0);
      s->insn_histo[mipsInsn::BLTZL]++;
      break;
    case branch_type::bltz:
      takeBranch = (s->gpr[rs] < 0);
      s->insn_histo[mipsInsn::BLTZ]++;
      break;
    case branch_type::bgezal:
      takeBranch = (s->gpr[rs] >= 0);
      s->insn_histo[mipsInsn::BGEZAL]++;
      saveReturn = true;
      break;
    case branch_type::bc1tl:
      isLikely = true;
      takeBranch = getConditionCode(s,((inst>>18)&7))==1;
      s->insn_histo[mipsInsn::BC1TL]++;
      break;
    case branch_type::bc1t:
      takeBranch = getConditionCode(s,((inst>>18)&7))==1;
      s->insn_histo[mipsInsn::BC1T]++;
      break;
    case branch_type::bc1fl:
      isLikely = true;
      takeBranch = getConditionCode(s,((inst>>18)&7))==0;
      s->insn_histo[mipsInsn::BC1FL]++;
      break;
    case branch_type::bc1f:
      takeBranch = getConditionCode(s,((inst>>18)&7))==0;
      s->insn_histo[mipsInsn::BC1F]++;
      break;
    default:
      UNREACHABLE();
    }

  s->pc += 4;
  if(isLikely) {
    if(takeBranch) {
      execMips<EL>(s);
      s->pc = (imm+npc);
    }
    else {
      s->pc += 4;
    }
  }
  else {
    execMips<EL>(s);
    if(takeBranch){
      if(saveReturn) {
	s->gpr[31] = npc + 4;
      }
      s->pc = (imm+npc);
    }
  }
}

template <bool EL>
void _bgez_bltz(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  switch(rt)
    {
    case 0:
      branch<EL,branch_type::bltz>(inst, s);
      break;
    case 1:
      branch<EL,branch_type::bgez>(inst, s);
      break;
    case 2:
      branch<EL,branch_type::bltzl>(inst, s);
      break;
    case 3:
      branch<EL,branch_type::bgezl>(inst, s);
      break;
    case 17:
      branch<EL,branch_type::bgezal>(inst, s);
      break;
    default:      
      std::cerr << "case " << rt << " not handled!\n";
      exit(-1);
    }
}


template <bool EL>
void _lw(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  uint32_t ea = (uint32_t)s->gpr[rs] + imm;
  s->gpr[rt] = bswap<EL>(s->mem.get<int32_t>(ea));
  //#define TRACE_MEM
#ifdef TRACE_MEM
  printf("_lw pc %x from ea %x = %x\n", s->pc, ea, s->gpr[rt]);
#endif
  //#undef TRACE_MEM
  s->pc += 4;
  s->insn_histo[mipsInsn::LW]++;
}

template <bool EL>
void _lh(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  
  uint32_t ea = s->gpr[rs] + imm;
  int16_t mem = bswap<EL>(s->mem.get<int16_t>(ea));
  
  s->gpr[rt] = static_cast<int32_t>(mem);
#ifdef TRACE_MEM
  printf("_lh from %x = %x\n", ea, s->gpr[rt]);
#endif
  s->pc +=4;
  s->insn_histo[mipsInsn::LH]++;  
}


static void _lb(uint32_t inst, state_t *s){
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  
  uint32_t ea = s->gpr[rs] + imm;
  s->gpr[rt] = static_cast<int32_t>(s->mem.get<int8_t>(ea));
#ifdef TRACE_MEM
  printf("_lb from %x = %x\n", ea, s->gpr[rt]);
#endif  
  s->pc += 4;
  s->insn_histo[mipsInsn::LB]++;  
}

static void _lbu(uint32_t inst, state_t *s){
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;

  uint32_t ea = s->gpr[rs] + imm;
  uint32_t zExt = s->mem.get<uint8_t>(ea);
  *((uint32_t*)&(s->gpr[rt])) = zExt;
  s->pc += 4;
  s->insn_histo[mipsInsn::LBU]++;
}


template <bool EL>
void _lhu(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  
  uint32_t ea = s->gpr[rs] + imm;
  uint32_t zExt = bswap<EL>(s->mem.get<uint16_t>(ea));
  *((uint32_t*)&(s->gpr[rt])) = zExt;
  //printf("_lhu from %x = %x\n", ea, s->gpr[rt]);  
  s->pc += 4;
  s->insn_histo[mipsInsn::LHU]++;  
}


template <bool EL>
void _sw(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  uint32_t ea = s->gpr[rs] + imm;
  s->mem.set<int32_t>(ea,  bswap<EL>(s->gpr[rt]));
  s->pc += 4;
  s->insn_histo[mipsInsn::SW]++;
}

template <bool EL>
void _sc(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  uint32_t ea = s->gpr[rs] + imm;
  s->mem.set<int32_t>(ea,  bswap<EL>(s->gpr[rt]));
  s->gpr[rt] = 1;
  s->pc += 4;
  s->insn_histo[mipsInsn::SC]++;
}


template <bool EL>
void _sh(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
    
  uint32_t ea = s->gpr[rs] + imm;
  s->mem.set<int16_t>(ea,  bswap<EL>(((int16_t)s->gpr[rt])));
  s->pc += 4;
  s->insn_histo[mipsInsn::SH]++;  
}

static void _sb(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
    
  uint32_t ea = s->gpr[rs] + imm;
  s->mem.set<uint8_t>(ea, static_cast<uint8_t>(s->gpr[rt]));
  
  s->pc +=4;
  s->insn_histo[mipsInsn::SB]++;
}

static void _mtc1(uint32_t inst, state_t *s) {
  uint32_t rd = (inst>>11) & 31;
  uint32_t rt = (inst>>16) & 31;
  s->cpr1[rd] = s->gpr[rt];
  s->pc += 4;
  s->insn_histo[mipsInsn::MTC1]++;  
}

static void _mfc1(uint32_t inst, state_t *s) {
  uint32_t rd = (inst>>11) & 31;
  uint32_t rt = (inst>>16) & 31;
  s->gpr[rt] = s->cpr1[rd];
  s->pc +=4;
  s->insn_histo[mipsInsn::MFC1]++;
}


template <bool EL>
void _swl(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;

  uint32_t ea = s->gpr[rs] + imm;
  uint32_t ma = ea & 3;
  ea &= 0xfffffffc;
  if(EL)
    ma = 3 - ma;
  uint32_t r = bswap<EL>(s->mem.get<uint32_t>(ea));   
  uint32_t xx=0,x = s->gpr[rt];
  
  uint32_t xs = x >> (8*ma);
  uint32_t m = ~((1U << (8*(4 - ma))) - 1);
  xx = (r & m) | xs;
  // std::cout << "SIM SWL EA " << std::hex << ea
  // 	    << ", MA = " << ma
  //   	    << ", X = " << x
  // 	    << ", R = " << r
  // 	    << ", XX = " << xx << std::dec << "\n";

  s->mem.set<uint32_t>(ea, bswap<EL>(xx));
  s->pc += 4;
  s->insn_histo[mipsInsn::SWL]++;  
}

template <bool EL>
void _swr(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
   
  uint32_t ea = s->gpr[rs] + imm;
  uint32_t ma = ea & 3;
  if(EL)
    ma = 3 - ma;
  ea &= ~(3U);
  uint32_t r = bswap<EL>(s->mem.get<uint32_t>(ea));   
  uint32_t xx=0,x = s->gpr[rt];
  
  uint32_t xs = 8*(3-ma);
  uint32_t rm = (1U << xs) - 1;

  xx = (x << xs) | (rm & r);
  s->mem.set<uint32_t>(ea, bswap<EL>(xx));
  s->pc += 4;
  s->insn_histo[mipsInsn::SWR]++;
}

template <bool EL>
void _lwl(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  
  uint32_t ea = ((uint32_t)s->gpr[rs] + imm);
  uint32_t u_ea = ea;
  uint32_t ma = ea & 3;
  ea &= 0xfffffffc;
  if(EL)
    ma = 3 - ma;
  uint32_t r = bswap<EL>(s->mem.get<uint32_t>(ea));
  int32_t x =  s->gpr[rt];
  
  switch(ma)
    {
    case 0:
      s->gpr[rt] = r;
      break;
    case 1:
      s->gpr[rt] = ((r & 0x00ffffff) << 8) | (x & 0x000000ff) ;
      break;
    case 2:
      s->gpr[rt] = ((r & 0x0000ffff) << 16)  | (x & 0x0000ffff) ;
      break;
    case 3:
      s->gpr[rt] = ((r & 0x00ffffff) << 24)  | (x & 0x00ffffff);
      break;
    }
#ifdef TRACE_MEM
  printf("_lwl from %x = %x\n", u_ea, s->gpr[rt]);
#endif  
  s->pc += 4;
  s->insn_histo[mipsInsn::LWL]++;  
}

template<bool EL>
void _lwr(uint32_t inst, state_t *s) {
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
 
  uint32_t ea = ((uint32_t)s->gpr[rs] + imm);
  uint32_t u_ea = ea;
  uint32_t ma = ea & 3;
  ea &= 0xfffffffc;
  if(EL)
    ma = 3-ma;

  uint32_t r = bswap<EL>(s->mem.get<uint32_t>(ea));
  uint32_t x =  s->gpr[rt];

  switch(ma)
    {
    case 0:
      s->gpr[rt] = (x & 0xffffff00) | (r>>24);
      break;
    case 1:
      s->gpr[rt] = (x & 0xffff0000) | (r>>16);
      break;
    case 2:
      s->gpr[rt] = (x & 0xff000000) | (r>>8);
      break;
    case 3:
      s->gpr[rt] = r;
      break;
    }

#ifdef TRACE_MEM
  printf("_lwr from %x = %x (x=%x, r = %x)\n", u_ea, s->gpr[rt], x, r);
#endif  
  
  s->pc += 4;
  s->insn_histo[mipsInsn::LWR]++;
}

static inline char* get_open_string(sparse_mem &mem, uint32_t offset) {
  size_t len = 0;
  char *ptr = reinterpret_cast<char*>(mem.get_raw_ptr(offset));
  char *buf = nullptr;
  while(*ptr != '\0') {
    ptr++;
    len++;
  }
  buf = new char[len+1];
  memset(buf, 0, len+1);
  ptr = reinterpret_cast<char*>(mem.get_raw_ptr(offset));
  for(size_t i = 0; i < len; i++) {
    buf[i] = *ptr;
    ptr++;
  }
  return buf;
}

template <bool EL>
void _monitorBody(uint32_t inst, state_t *s) {
 uint32_t reason = (inst >> RSVD_INSTRUCTION_ARG_SHIFT) & RSVD_INSTRUCTION_ARG_MASK;
  reason >>= 1;
  int32_t fd=-1,nr=-1,flags=-1;
  char *path;
  struct timeval tp;
  timeval32_t tp32;
  struct tms tms_buf;
  tms32_t tms32_buf;
  struct stat native_stat;
  stat32_t *host_stat = nullptr;
  if(globals::report_syscalls) {
    printf("monitor reason %d\n", reason);
  }
  switch(reason)
    {
    case 6: /* int open(char *path, int flags) */
      path = reinterpret_cast<char*>(s->mem.get_raw_ptr(s->gpr[R_a0]));
      flags = remapIOFlags(s->gpr[R_a1]);
      fd = open(path, flags, S_IRUSR|S_IWUSR);
      s->gpr[R_v0] = fd;
      break;
    case 7: /* int read(int file,char *ptr,int len) */
      fd = s->gpr[R_a0];
      nr = s->gpr[R_a2];
      s->mem.coalesce(s->gpr[R_a1], nr);
      if(fd == STDIN_FILENO) {
	const char* in = getenv("STDIN_FILE");
	assert(in);
	fd = open(in, O_RDONLY, S_IRUSR|S_IWUSR);
	s->gpr[R_v0] = per_page_rdwr<false>(s->mem, fd, s->gpr[R_a1], nr);
	close(fd);
      }
      else {
	s->gpr[R_v0] = per_page_rdwr<false>(s->mem, fd, s->gpr[R_a1], nr);
      }
      break;
    case 8: 
      /* int write(int file, char *ptr, int len) */
      fd = s->gpr[R_a0];
      nr = s->gpr[R_a2];
      s->gpr[R_v0] = per_page_rdwr<true>(s->mem, fd, s->gpr[R_a1], nr);
      //s->gpr[R_v0] = (int32_t)write(fd, (void*)(s->mem + (uint32_t)s->gpr[R_a1]), nr);
      if(fd==1)
	fflush(stdout);
      else if(fd==2)
	fflush(stderr);
      break;
    case 9:
      s->gpr[R_v0] = lseek(s->gpr[R_a0], s->gpr[R_a1], s->gpr[R_a2]);
      break;
    case 10:
      fd = s->gpr[R_a0];
      if(fd>2)
	s->gpr[R_v0] = (int32_t)close(fd);
      else
	s->gpr[R_v0] = 0;
      break;
    case 33: {
      uint32_t uptr = *(uint32_t*)(s->gpr + R_a0);
      s->mem.set<uint32_t>(uptr, 0);
      s->mem.set<uint32_t>(uptr+4, 0);
      s->gpr[R_v0] = 0;
      break;
    }
    case 34: {
      uint32_t uptr = *(uint32_t*)(s->gpr + R_a0);
      //if(globals::enClockFuncts) {
      //	 = times(&tms_buf);
      // }
      //else {
      //uint64_t mips = globals::icountMIPS*1000000;
      // tms_buf.tms_utime = (s->icnt/mips)*100;
      //tms_buf.tms_stime = 0;
      //tms_buf.tms_cutime = 0;
      //tms_buf.tms_cstime = 0;	
      //}
      //memset(&tms32_buf, 0, sizeof(tms32_buf));
      //tms32_buf.tms_utime = bswap<EL>((uint32_t)tms_buf.tms_utime);
      //tms32_buf.tms_stime = bswap<EL>((uint32_t)tms_buf.tms_stime);
      //tms32_buf.tms_cutime = bswap<EL>((uint32_t)tms_buf.tms_cutime);
      //tms32_buf.tms_cstime = bswap<EL>((uint32_t)tms_buf.tms_cstime);      
      //*((tms32_t*)(s->mem + uptr)) = tms32_buf;
      for(int i = 0; i < 4; i++) {
	s->mem.set<uint32_t>(uptr+i,0);
      }
      *((uint32_t*)(&s->gpr[R_v0])) = 0;
      break;
    }
    case 35:
      /* int getargs(char **argv) */
      for(int i = 0; i < std::min(MARGS, globals::sysArgc); i++) {
	  uint32_t arrayAddr = ((uint32_t)s->gpr[R_a0])+4*i;
	  uint32_t ptr = bswap<EL>(*((uint32_t*)(s->mem + arrayAddr)));
	  strcpy((char*)(s->mem + ptr), globals::sysArgv[i]);
	}
      s->gpr[R_v0] = globals::sysArgc;
      break;
    case 37:
      /*char *getcwd(char *buf, uint32_t size) */
      path = (char*)(s->mem + (uint32_t)s->gpr[R_a0]);
      getcwd(path, (uint32_t)s->gpr[R_a1]);
      s->gpr[R_v0] = s->gpr[R_a0];
      break;
    case 38:
      /* int chdir(const char *path); */
      path = reinterpret_cast<char*>(s->mem.get_raw_ptr(s->gpr[R_a0]));
      s->gpr[R_v0] = chdir(path);
      break;
    case 50:
      s->gpr[R_v0] = globals::cycle;
      break;
    case 55: 
      /* void get_mem_info(unsigned int *ptr) */
      /* in:  A0 = pointer to three word memory location */
      /* out: [A0 + 0] = size */
      /*      [A0 + 4] = instruction cache size */
      /*      [A0 + 8] = data cache size */
      /* 256 MBytes of DRAM */
      //printf("monitor writing to words %x, %x, and %x\n", s->gpr[R_a0], s->gpr[R_a0] + 4, s->gpr[R_a0] + 8);
      s->mem.set<uint32_t>(static_cast<uint32_t>(s->gpr[R_a0] + 0),
			   bswap<EL>(K1SIZE));
      /* No Icache */
      s->mem.set<uint32_t>(static_cast<uint32_t>(s->gpr[R_a0] + 4), 0);
      /* No Dcache */
      s->mem.set<uint32_t>(static_cast<uint32_t>(s->gpr[R_a0] + 8), 0);
      
      s->gpr[R_v0] = 0;
      break;
    default:
      printf("unhandled monitor instruction (reason = %d)\n", reason);
      exit(-1);
      break;
    }
  s->pc = s->gpr[31];
  s->insn_histo[mipsInsn::MONITOR]++;
}



template <bool EL>
void _ldc1(uint32_t inst, state_t *s) {
  uint32_t ft = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  uint32_t ea = s->gpr[rs] + imm;
  *reinterpret_cast<int64_t*>(s->cpr1 + ft) = bswap<EL>(s->mem.get<int64_t>(ea));
  s->pc += 4;
  s->insn_histo[mipsInsn::LDC1]++;
}

template <bool EL>
void _sdc1(uint32_t inst, state_t *s) {
  uint32_t ft = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  uint32_t ea = s->gpr[rs] + imm;
  s->mem.set<int64_t>(ea,  bswap<EL>((*(int64_t*)(s->cpr1 + ft))));
  s->pc += 4;
  s->insn_histo[mipsInsn::SDC1]++;  
}

template <bool EL>
void _lwc1(uint32_t inst, state_t *s) {
  uint32_t ft = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  uint32_t ea = s->gpr[rs] + imm;
  uint32_t v = bswap<EL>(s->mem.get<uint32_t>(ea)); 
  *((float*)(s->cpr1 + ft)) = *((float*)&v);
  s->pc += 4;
  s->insn_histo[mipsInsn::LWC1]++;
}

template <bool EL>
void _swc1(uint32_t inst, state_t *s) {
  uint32_t ft = (inst >> 16) & 31;
  uint32_t rs = (inst >> 21) & 31;
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = (int32_t)himm;
  uint32_t ea = s->gpr[rs] + imm;
  uint32_t v = *((uint32_t*)(s->cpr1+ft));
  s->mem.set<uint32_t>(ea, bswap<EL>(v));
  s->pc += 4;
  s->insn_histo[mipsInsn::SWC1]++;
}

static void _truncl(uint32_t inst, state_t *s) {
  printf("%s\n",__func__);
  exit(-1);
}

static void _truncw(uint32_t inst, state_t *s) {
  uint32_t fmt = (inst >> 21) & 31;
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  float f;
  double d;
  int32_t *ptr = ((int32_t*)(s->cpr1 + fd));
  if(currFpMode != fpMode::mips32) {
    assert((fd & 1) == 0);
    assert((fs & 1) == 0);
  }  
  switch(fmt)
    {
    case FMT_S:
      f = (*((float*)(s->cpr1 + fs)));
      //printf("f=%g\n", f);
      *ptr = (int32_t)f;
      s->insn_histo[mipsInsn::TRUNC_SP_W]++;
      break;
    case FMT_D:
      d = (*((double*)(s->cpr1 + fs)));
      *ptr = (int32_t)d;
      s->insn_histo[mipsInsn::TRUNC_DP_W]++;      
      //printf("id=%d\n", *ptr);
      break;
    default:
      printf("unknown trunc for fmt %d\n", fmt);
      exit(-1);
      break;
    }
  if(currFpMode != fpMode::mips32) {
    s->cpr1[fd + 1] = 0;
  }      
  s->pc += 4;
}

static void _movnd(uint32_t inst, state_t *s) {
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  uint32_t rt = (inst>>16) & 31;
  bool notZero = (s->gpr[rt] != 0);
  s->cpr1[fd+0] = notZero ? s->cpr1[fs+0] : s->cpr1[fd+0];
  s->cpr1[fd+1] = notZero ? s->cpr1[fs+1] : s->cpr1[fd+1];
  s->pc += 4;
}

static void _movns(uint32_t inst, state_t *s) {
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  uint32_t rt = (inst>>16) & 31;
  bool notZero = (s->gpr[rt] != 0);
  s->cpr1[fd+0] = notZero ? s->cpr1[fs+0] : s->cpr1[fd+0];
  s->pc += 4;
}

static void _movzd(uint32_t inst, state_t *s) {
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  uint32_t rt = (inst>>16) & 31;
 
  s->cpr1[fd+0] = (s->gpr[rt] == 0) ? s->cpr1[fs+0] : s->cpr1[fd+0];
  s->cpr1[fd+1] = (s->gpr[rt] == 0) ? s->cpr1[fs+1] : s->cpr1[fd+1];
  s->pc += 4;
}

static void _movzs(uint32_t inst, state_t *s) {
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  uint32_t rt = (inst>>16) & 31;

  s->cpr1[fd+0] = (s->gpr[rt] == 0) ? s->cpr1[fs+0] : s->cpr1[fd+0];
  s->pc += 4;
}

static void _movcd(uint32_t inst, state_t *s) {
  uint32_t cc = (inst >> 18) & 7;
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  uint32_t tf = (inst>>16) & 1;

  if(tf==0) {
    if(getConditionCode(s,cc)==0) {
      s->cpr1[fd+0] = s->cpr1[fs+0];
      s->cpr1[fd+1] = s->cpr1[fs+1];
    }
    s->insn_histo[mipsInsn::FP_MOVF];    
  }
  else {
    if(getConditionCode(s,cc)==1) {
      s->cpr1[fd+0] = s->cpr1[fs+0];
      s->cpr1[fd+1] = s->cpr1[fs+1];
    }
    s->insn_histo[mipsInsn::FP_MOVT];    
  }
  s->pc += 4;
}

static void _movcs(uint32_t inst, state_t *s) {
  uint32_t cc = (inst >> 18) & 7;
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  uint32_t tf = (inst>>16) & 1;
  if(tf==0) {
    s->cpr1[fd+0] = getConditionCode(s, cc) ? s->cpr1[fd+0] : s->cpr1[fs+0];
    s->insn_histo[mipsInsn::FP_MOVF];
  }
  else {
    s->cpr1[fd+0] = getConditionCode(s, cc) ? s->cpr1[fs+0] : s->cpr1[fd+0];
    s->insn_histo[mipsInsn::FP_MOVT];
  }
  s->pc += 4;
}


static void _movci(uint32_t inst, state_t *s) {
  uint32_t cc = (inst >> 18) & 7;
  uint32_t tf = (inst>>16) & 1;
  uint32_t rd = (inst>>11) & 31;
  uint32_t rs = (inst >> 21) & 31;
  if(tf==0) {
    /* movf */
    s->gpr[rd] = getConditionCode(s, cc) ? s->gpr[rd] : s->gpr[rs];
    s->insn_histo[mipsInsn::MOVF]++;
  }
  else {
    /* movt */
    s->gpr[rd] = getConditionCode(s, cc) ? s->gpr[rs] : s->gpr[rd];
    s->insn_histo[mipsInsn::MOVT]++;    
  }
  s->pc += 4;
}

static void _cvts(uint32_t inst, state_t *s) {
  uint32_t fmt = (inst >> 21) & 31;
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  if(currFpMode != fpMode::mips32) {
    assert((fd & 1) == 0);
    assert((fs & 1) == 0);
  }
  switch(fmt)
    {
    case FMT_D:
      *((float*)(s->cpr1 + fd)) = (float)(*((double*)(s->cpr1 + fs)));
      if(currFpMode != fpMode::mips32) {
	s->cpr1[fd+1] = 0;
      }
      s->cpr1_state[fd] = fp_reg_state::sp;      
      break;
    case FMT_W:
      *((float*)(s->cpr1 + fd)) = (float)(*((int32_t*)(s->cpr1 + fs)));
      if(currFpMode != fpMode::mips32) {
	*((float*)(s->cpr1 + fd + 1)) = 0;
      }
      break;
    default:
      printf("%s @ %d\n", __func__, __LINE__);
      exit(-1);
      break;
    }
  s->pc += 4;
}

static void _cvtd(uint32_t inst, state_t *s) {
  uint32_t fmt = (inst >> 21) & 31;
  uint32_t fd = (inst>>6) & 31;
  uint32_t fs = (inst>>11) & 31;
  switch(fmt)
    {
    case FMT_S:
      *((double*)(s->cpr1 + fd)) = (double)(*((float*)(s->cpr1 + fs)));
      s->cpr1_state[fd] = fp_reg_state::dp;
      break;
    case FMT_W:
     *((double*)(s->cpr1 + fd)) = (double)(*((int32_t*)(s->cpr1 + fs)));
      break;
    default:
      printf("%s @ %d\n", __func__, __LINE__);
      exit(-1);
      break;
    }
  s->pc += 4;
}

static void _fmovn(uint32_t inst, state_t *s) {
  uint32_t fmt = (inst >> 21) & 31;
  switch(fmt)
    {
    case FMT_S:
      _movns(inst, s);
      break;
    case FMT_D:
      _movnd(inst, s);
      break;
    default:
      printf("unsupported %s\n", __func__);
      exit(-1);
      break;
    }
}


static void _fmovz(uint32_t inst, state_t *s) {
  uint32_t fmt = (inst >> 21) & 31;
  switch(fmt)
    {
    case FMT_S:
      _movzs(inst, s);
      break;
    case FMT_D:
      _movzd(inst, s);
      break;
    default:
      printf("unsupported %s\n", __func__);
      exit(-1);
      break;
    }
}

static void _fmovc(uint32_t inst, state_t *s) {
  uint32_t fmt = (inst >> 21) & 31;
  switch(fmt)
    {
    case FMT_S:
      _movcs(inst, s);
      break;
    case FMT_D:
      _movcd(inst, s);
      break;
    default:
      printf("unsupported %s\n", __func__);
      exit(-1);
      break;
    }
}



template <typename T>
static void fpCmp(uint32_t inst, state_t *s) {
  uint32_t cond = inst & 15;
  uint32_t cc = (inst >> 8) & 7;
  uint32_t ft = (inst >> 16) & 31;
  uint32_t fs = (inst >> 11) & 31;
  T Tfs = *((T*)(s->cpr1+fs));
  T Tft = *((T*)(s->cpr1+ft));
  uint32_t v = 0;

  switch(cond)
    {
    case COND_UN:
      v = (Tfs == Tft);
      s->fcr1[CP1_CR25] = setBit(s->fcr1[CP1_CR25],v,cc);
      break;
    case COND_EQ:
      v = (Tfs == Tft);
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_CMP_EQ, mipsInsn::SP_CMP_EQ)]++;            
      s->fcr1[CP1_CR25] = setBit(s->fcr1[CP1_CR25],v,cc);
      break;
    case COND_LT:
      v = (Tfs < Tft);
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_CMP_LT, mipsInsn::SP_CMP_LT)]++;      
      s->fcr1[CP1_CR25] = setBit(s->fcr1[CP1_CR25],v,cc);
      break;
    case COND_LE:
      v = (Tfs <= Tft);
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_CMP_LE, mipsInsn::SP_CMP_LE)]++;            
      s->fcr1[CP1_CR25] = setBit(s->fcr1[CP1_CR25],v,cc);
      break;
    default:
      printf("unimplemented %s = %s\n", __func__, getCondName(cond).c_str());
      exit(-1);
      break;
    }
  if(globals::trace_retirement) {
    std::cout << std::hex
	      << s->pc
	      << std::dec
	      << " c. "
	      << Tfs
	      << " "
	      << getCondName(cond)
	      << " "
	      << Tft
	      << " = "
	      << v
	      << "\n";
  }
  
  s->pc += 4;
}

static void _c(uint32_t inst, state_t *s) {
  uint32_t fmt = (inst >> 21) & 31;
  switch(fmt)
    {
    case FMT_S:
      fpCmp<float>(inst,s);
      break;
    case FMT_D:
      fpCmp<double>(inst,s);
      break;
    default:
      printf("unsupported comparison\n");
      exit(-1);
      break;
    }
}

template< typename T, fpOperation op>
static void execFP(uint32_t inst, state_t *s) {
  uint32_t ft = (inst>>16)&31, fs=(inst>>11)&31, fd=(inst>>6)&31;
  T _fs = *reinterpret_cast<T*>(s->cpr1+fs);
  T _ft = *reinterpret_cast<T*>(s->cpr1+ft);
  T &_fd = *reinterpret_cast<T*>(s->cpr1+fd);

  switch(op)
    {
    case fpOperation::abs:
      _fd = std::abs(_fs);
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_ABS, mipsInsn::SP_ABS)]++;      
      break;
    case fpOperation::neg:
      _fd = -_fs;
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_NEG, mipsInsn::SP_NEG)]++;
      break;
    case fpOperation::mov:
      _fd = _fs;
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_MOV, mipsInsn::SP_MOV)]++;            
      break;
    case fpOperation::add:
      _fd = _fs + _ft;
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_ADD, mipsInsn::SP_ADD)]++;            
      break;
    case fpOperation::sub:
      _fd = _fs - _ft;
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_SUB, mipsInsn::SP_SUB)]++;                  
      break;
    case fpOperation::mul:
      _fd = _fs * _ft;
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_MUL, mipsInsn::SP_MUL)]++;      
      break;
    case fpOperation::div:
      if(_ft==0.0) {
	_fd = std::numeric_limits<T>::max();
      }
      else {
	_fd = _fs / _ft;
      }
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_DIV, mipsInsn::SP_DIV)]++;       
      break;
    case fpOperation::sqrt:
      _fd = std::sqrt(_fs);
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_SQRT, mipsInsn::SP_SQRT)]++;      
      break;
    case fpOperation::rsqrt:
      _fd = static_cast<T>(1.0) / std::sqrt(_fs);
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_RSQRT, mipsInsn::SP_RSQRT)]++;
      break;
    case fpOperation::recip:
      _fd = static_cast<T>(1.0) / _fs;
      s->insn_histo[select_fp_insn<T>(mipsInsn::DP_RECIP, mipsInsn::SP_RECIP)]++;
      break;
    default:
      UNREACHABLE();
    }
  s->pc+=4;
}

template <fpOperation op>
void do_fp_op(uint32_t inst, state_t *s) {
  int fd=(inst>>6)&31;
  switch((inst>>21)&31) {
  case FMT_S: 
    assert((fd&1) == 0);
    execFP<float,op>(inst,s);
    s->cpr1[fd+1] = 0;
    s->cpr1_state[fd] = fp_reg_state::sp;
    s->cpr1_state[fd+1] = fp_reg_state::unknown;
    break;
  case FMT_D:
    execFP<double,op>(inst,s);
    s->cpr1_state[fd] = fp_reg_state::dp;
    break;
  default:
    UNREACHABLE();
  }
}


template <bool EL>
static void execCoproc1(uint32_t inst, state_t *s) {
  uint32_t opcode = inst>>26;
  uint32_t functField = (inst>>21) & 31;
  uint32_t lowop = inst & 63;  
  uint32_t fmt = (inst >> 21) & 31;
  uint32_t nd_tf = (inst>>16) & 3;
  
  uint32_t lowbits = inst & ((1<<11)-1);
  opcode &= 0x3;

  if(fmt == 0x8)
    {
      switch(nd_tf)
	{
	case 0x0:
	  branch<EL,branch_type::bc1f>(inst, s);
	  break;
	case 0x1:
	  branch<EL,branch_type::bc1t>(inst, s);
	  break;
	case 0x2:
	  branch<EL,branch_type::bc1fl>(inst, s);
	  break;
	case 0x3:
	  branch<EL,branch_type::bc1tl>(inst, s);
	  break;
	}
      /*BRANCH*/
    }
  else if((lowbits == 0) && ((functField==0x0) || (functField==0x4)))
    {
      if(functField == 0x0)
	{
	  /* move from coprocessor */
	  _mfc1(inst,s);
	}
      else if(functField == 0x4)
	{
	  /* move to coprocessor */
	  _mtc1(inst,s);
	}
    }
  else
    {
      if((lowop >> 4) == 3)
	{
	  _c(inst, s);
	}
      else{
	switch(lowop)
	  {
	  case 0x0:
	    do_fp_op<fpOperation::add>(inst, s);
	    break;
	  case 0x1:
	    do_fp_op<fpOperation::sub>(inst, s);
	    break;
	  case 0x2:
	    do_fp_op<fpOperation::mul>(inst, s);
	    break;
	  case 0x3:
	    do_fp_op<fpOperation::div>(inst, s);
	    break;
	  case 0x4:
	    do_fp_op<fpOperation::sqrt>(inst, s);
	    break;
	  case 0x5:
	    do_fp_op<fpOperation::abs>(inst, s);
	    break;
	  case 0x6:
	    do_fp_op<fpOperation::mov>(inst, s);
	    break;
	  case 0x7:
	    do_fp_op<fpOperation::neg>(inst, s);
	    break;
	  case 0x9:
	    _truncl(inst, s);
	    break;
	  case 0xd:
	    _truncw(inst, s);
	    break;
	  case 0x11:
	    _fmovc(inst, s);
	    break;
	  case 0x12:
	    _fmovz(inst, s);
	    break;
	  case 0x13:
	    _fmovn(inst, s);
	    break;
	  case 0x15:
	    do_fp_op<fpOperation::recip>(inst, s);
	    break;
	  case 0x16:
	    do_fp_op<fpOperation::rsqrt>(inst, s);
	    break;
	  case 0x20:
	    /* cvt.s */
	    _cvts(inst, s);
	    break;
	  case 0x21:
	    _cvtd(inst, s);
	    break;
	  default:
	    printf("unhandled coproc1 instruction (%x) @ %08x\n",
		   inst, s->pc);
	    exit(-1);
	    break;
	  }
      }
    }
}

template <bool EL>
bool is_store_insn(state_t *s) {
  sparse_mem &mem = s->mem;
  uint32_t inst = bswap<EL>(mem.get<uint32_t>(s->pc));
  uint32_t opcode = inst>>26;
  switch(opcode)
    {
    case 0x28: //_sb(inst, s); 
    case 0x29: //_sh<EL>(inst, s); 
    case 0x2a: //_swl<EL>(inst, s); 
    case 0x2B: //_sw<EL>(inst, s); 
    case 0x2e: //_swr<EL>(inst, s);
    case 0x39: //_swc1<EL>(inst, s);
    case 0x38: //_sc
    case 0x3D: //_sdc1<EL>(inst, s);
      return true;
    default:
      break;
    }
  return false;
}


bool is_store_insn(state_t *s) {
  return is_store_insn<false>(s);
}


template <bool EL>
void execMips(state_t *s) {
  sparse_mem &mem = s->mem;
  uint32_t inst = bswap<EL>(mem.get<uint32_t>(s->pc));
  if(globals::trace_retirement and false) {
    std::cout << std::hex
	      << "cosim "
	      << s->pc << ","
	      << std::dec << " : "
	      << getAsmString(inst, s->pc) << "\n";
  }
  //std::cout << std::hex << s->pc << std::dec << " : "
  //<< getAsmString(inst, s->pc) << "\n";
  uint32_t opcode = inst>>26;
  bool isRType = (opcode==0);
  bool isJType = ((opcode>>1)==1);
  bool isCoproc0 = (opcode == 0x10);
  bool isCoproc1 = (opcode == 0x11);
  bool isCoproc1x = (opcode == 0x13);
  bool isCoproc2 = (opcode == 0x12);
  bool isSpecial2 = (opcode == 0x1c); 
  bool isSpecial3 = (opcode == 0x1f);
  bool isLoadLinked = (opcode == 0x30);
  bool isStoreCond = (opcode == 0x38);
  uint32_t rs = (inst >> 21) & 31;
  uint32_t rt = (inst >> 16) & 31;
  uint32_t rd = (inst >> 11) & 31;
  s->icnt++;
    
  if(isRType) {
    uint32_t funct = inst & 63;
    uint32_t sa = (inst >> 6) & 31;
    switch(funct) 
      {
      case 0x00: /*sll*/
	s->gpr[rd] = s->gpr[rt] << sa;
	s->pc += 4;
	if(inst == 0) {
	  s->insn_histo[mipsInsn::NOP]++;
	}
	else {
	  s->insn_histo[mipsInsn::SLL]++;
	}
	break;
      case 0x01: /* movci */
	_movci(inst,s);
	break;
      case 0x02: /* srl */
	s->gpr[rd] = ((uint32_t)s->gpr[rt] >> sa);
	s->pc += 4;
	s->insn_histo[mipsInsn::SRL]++;
	break;
      case 0x03: /* sra */
	s->gpr[rd] = s->gpr[rt] >> sa;
	s->pc += 4;
	s->insn_histo[mipsInsn::SRA]++;
	break;	
      case 0x04: /* sllv */
	s->gpr[rd] = s->gpr[rt] << (s->gpr[rs] & 0x1f);
	s->pc += 4;
	s->insn_histo[mipsInsn::SLLV]++;
	break;
      case 0x05:
	_monitorBody<EL>(inst, s);
	break;
      case 0x06:  
	s->gpr[rd] = ((uint32_t)s->gpr[rt]) >> (s->gpr[rs] & 0x1f);
	s->pc += 4;
	s->insn_histo[mipsInsn::SRLV]++;
	break;
      case 0x07:  
	s->gpr[rd] = s->gpr[rt] >> (s->gpr[rs] & 0x1f);
	s->pc += 4;
	s->insn_histo[mipsInsn::SRAV]++;
	break;
      case 0x08: { /* jr */
	uint32_t jaddr = s->gpr[rs];
	s->pc += 4;
	execMips<EL>(s);
	s->pc = jaddr;
	s->insn_histo[mipsInsn::JR]++;	
	break;
      }
      case 0x09: { /* jalr */
	uint32_t jaddr = s->gpr[rs];
	s->gpr[31] = s->pc+8;
	s->pc += 4;
	execMips<EL>(s);
	s->pc = jaddr;
	s->insn_histo[mipsInsn::JALR]++;	
	break;
      }
      case 0x0C: /* syscall */
      case 0x0D: /* break */
	s->brk = 1;
	s->insn_histo[mipsInsn::BREAK]++;
	break;
      case 0x0f: /* sync */
	s->pc += 4;
	s->insn_histo[mipsInsn::SYNC]++;
	break;
      case 0x10: /* mfhi */
	s->gpr[rd] = s->hi;
	s->pc += 4;
	s->insn_histo[mipsInsn::MFHI]++;
	break;
      case 0x11: /* mthi */ 
	s->hi = s->gpr[rs];
	s->pc += 4;
	s->insn_histo[mipsInsn::MTHI]++;
	break;
      case 0x12: /* mflo */
	s->gpr[rd] = s->lo;
	s->pc += 4;
	s->insn_histo[mipsInsn::MFLO]++;	
	break;
      case 0x13: /* mtlo */
	s->lo = s->gpr[rs];
	s->pc += 4;
	s->insn_histo[mipsInsn::MTLO]++;		
	break;
      case 0x18: { /* mult */
	int64_t y;
	y = (int64_t)s->gpr[rs] * (int64_t)s->gpr[rt];
	s->lo = (int32_t)(y & 0xffffffff);
	s->hi = (int32_t)(y >> 32);
	s->pc += 4;
	s->insn_histo[mipsInsn::MULT]++;			
	break;
      }
      case 0x19: { /* multu */
	uint64_t y;
	uint64_t u0 = (uint64_t)*((uint32_t*)&s->gpr[rs]);
	uint64_t u1 = (uint64_t)*((uint32_t*)&s->gpr[rt]);
	y = u0*u1;
	*((uint32_t*)&(s->lo)) = (uint32_t)y;
	*((uint32_t*)&(s->hi)) = (uint32_t)(y>>32);
	s->pc += 4;
	s->insn_histo[mipsInsn::MULTU]++;				
	break;
      }
      case 0x1A: /* div */
	if(s->gpr[rt] != 0) {
	  s->lo = s->gpr[rs] / s->gpr[rt];
	  s->hi = s->gpr[rs] % s->gpr[rt];
	}
	s->pc += 4;
	s->insn_histo[mipsInsn::DIV]++;
	break;
      case 0x1B: /* divu */
	if(s->gpr[rt] != 0) {
	  s->lo = (uint32_t)s->gpr[rs] / (uint32_t)s->gpr[rt];
	  s->hi = (uint32_t)s->gpr[rs] % (uint32_t)s->gpr[rt];
	}
	s->pc += 4;
	s->insn_histo[mipsInsn::DIVU]++;
	break;
      case 0x20: /* add */
	s->gpr[rd] = s->gpr[rs] + s->gpr[rt];
	s->pc += 4;
	s->insn_histo[mipsInsn::ADD]++;
	break;
      case 0x21: { /* addu */
	uint32_t u_rs = (uint32_t)s->gpr[rs];
	uint32_t u_rt = (uint32_t)s->gpr[rt];
	s->gpr[rd] = u_rs + u_rt;
	s->pc += 4;
	s->insn_histo[mipsInsn::ADDU]++;
	break;
      }
      case 0x22: /* sub */
	printf("sub()\n");
	exit(-1);
	break;
      case 0x23:{ /*subu*/  
	uint32_t u_rs = (uint32_t)s->gpr[rs];
	uint32_t u_rt = (uint32_t)s->gpr[rt];
	uint32_t y = u_rs - u_rt;
	s->gpr[rd] = y;
	s->pc += 4;
	s->insn_histo[mipsInsn::SUBU]++;
	break;
      }
      case 0x24: /* and */
	s->gpr[rd] = s->gpr[rs] & s->gpr[rt];
	s->pc += 4;
	s->insn_histo[mipsInsn::AND]++;
	break;
      case 0x25: /* or */
	if(rd != 0) {
	  s->gpr[rd] = s->gpr[rs] | s->gpr[rt];
	}
	s->pc += 4;
	s->insn_histo[mipsInsn::OR]++;
	break;
      case 0x26: /* xor */
	s->gpr[rd] = s->gpr[rs] ^ s->gpr[rt];
	s->pc += 4;
	s->insn_histo[mipsInsn::XOR]++;	
	break;
      case 0x27: /* nor */
	s->gpr[rd] = ~(s->gpr[rs] | s->gpr[rt]);
	s->pc += 4;
	s->insn_histo[mipsInsn::NOR]++;
	break;
      case 0x2A: /* slt */
	s->gpr[rd] = s->gpr[rs] < s->gpr[rt];
	s->pc += 4;
	s->insn_histo[mipsInsn::SLT]++;	
	break;
      case 0x2B: { /* sltu */
	uint32_t urs = (uint32_t)s->gpr[rs];
	uint32_t urt = (uint32_t)s->gpr[rt];
	s->gpr[rd] = (urs < urt);
	s->pc += 4;
	s->insn_histo[mipsInsn::SLTU]++;	
	break;
      }
      case 0x0B: /* movn */
	s->gpr[rd] = (s->gpr[rt] != 0) ? s->gpr[rs] : s->gpr[rd];
	s->pc +=4;
	s->insn_histo[mipsInsn::MOVN]++;
	break;
      case 0x0A: /* movz */
	s->gpr[rd] = (s->gpr[rt] == 0) ? s->gpr[rs] : s->gpr[rd];
	s->pc += 4;
	s->insn_histo[mipsInsn::MOVZ]++;	
	break;
      case 0x34: /* teq */
	if(s->gpr[rs] == s->gpr[rt]) {
	  printf("teq trap!!!!!\n");
	  exit(-1);
	}
	s->pc += 4;
	s->insn_histo[mipsInsn::TEQ]++;
	break;
      default:
	printf("%sunknown RType instruction %x, funct = %d%s\n", 
	       KRED, s->pc, funct, KNRM);
	exit(-1);
	break;
      }
  }
  else if(isSpecial2)
    execSpecial2(inst,s);
  else if(isSpecial3)
    execSpecial3(inst,s);
  else if(isJType) {
    uint32_t jaddr = inst & ((1<<26)-1);
    jaddr <<= 2;
    if(opcode==0x2) { /* j */
      s->pc += 4;
      s->insn_histo[mipsInsn::J]++;
    }
    else if(opcode==0x3) { /* jal */
      s->gpr[31] = s->pc+8;
      s->pc += 4;
      s->insn_histo[mipsInsn::JAL]++;
    }
    else {
      printf("Unknown JType instruction\n");
      exit(-1);
    }
    jaddr |= (s->pc & (~((1<<28)-1)));
    execMips<EL>(s);
    s->pc = jaddr;
  }
  else if(isCoproc0) {
    if( ((inst >> 25)&1) ) {
      switch(inst & 63)
	{
	case 24: //ERETU
	  exit(-1);
	case 32: //WAIT
	  if((s->cpr0[CPR0_SR] & 1) == 0) {
	    printf("attempting to wait with interrupts disabled @ VA %x, PA %x\n",
		   s->pc, VA2PA(s->pc));
	    exit(-1);
	  }
	  s->insn_histo[mipsInsn::WAIT]++;
	  break;
	default:
	  exit(-1);
	}
    }
    else if( (((inst >> 21) & 31) == 11 ) &&
	     ((inst & 65535) == 0x6000) ) {
      //DI
      if(rt != 0) {
	s->gpr[rt] = s->cpr0[CPR0_SR];
      }
      s->cpr0[CPR0_SR] &= (~1U);
      s->insn_histo[mipsInsn::DI]++;
    }
    else if( (((inst >> 21) & 31) == 11 ) &&
	     ((inst & 65535) == 0x6020) ) {
      //EI
      if(rt != 0) {
	s->gpr[rt] = s->cpr0[CPR0_SR];
      }
      s->cpr0[CPR0_SR] |= 1U;
      s->insn_histo[mipsInsn::EI]++;
    }

    else {
      switch(rs) 
	{
	case 0x0: /*mfc0*/
	  s->gpr[rt] = s->cpr0[rd];
	  s->insn_histo[mipsInsn::MFC0]++;
	  break;
	case 0x4: /*mtc0*/
	  s->cpr0[rd] = s->gpr[rt];
	  //std::cout << "writing cpr0[" << rd << "] = " << std::hex << s->gpr[rt] << std::dec << "\n";
	  s->insn_histo[mipsInsn::MTC0]++;
	  break;
	default:
	  std::cerr << "unhandled cpr0 instruction @ "
		    << std::hex << s->pc << std::dec << "\n";
	  exit(-1);
	  break;
	}
      }
    s->pc += 4;
  }
  else if(isCoproc1) 
    execCoproc1<EL>(inst,s);
  else if(isCoproc1x)
    execCoproc1x<EL>(inst,s);
  else if(isCoproc2) {
    printf("coproc2 unimplemented\n");  exit(-1);
  }
  else if(isLoadLinked)
    _lw<EL>(inst, s);
  else if(isStoreCond)
    _sc<EL>(inst, s);
  else { /* itype */
    uint32_t uimm32 = inst & ((1<<16) - 1);
    int16_t simm16 = (int16_t)uimm32;
    int32_t simm32 = (int32_t)simm16;
    int32_t tmp;
    switch(opcode) 
      {
      case 0x01:
	_bgez_bltz<EL>(inst, s); 
	break;
      case 0x04:
	branch<EL,branch_type::beq>(inst, s);
	break;
      case 0x05:
	branch<EL,branch_type::bne>(inst, s); 
	break;
      case 0x06:
	branch<EL,branch_type::blez>(inst, s); 
	break;
      case 0x07:
	branch<EL,branch_type::bgtz>(inst, s); 
	break;
      case 0x08: /* addi */
	s->gpr[rt] = s->gpr[rs] + simm32;  
	s->pc+=4;
	s->insn_histo[mipsInsn::ADDI]++;
	break;
      case 0x09: /* addiu */
	tmp = s->gpr[rs] + simm32;
	s->gpr[rt] = tmp;
	s->pc+=4;
	s->insn_histo[mipsInsn::ADDIU]++;
	break;
      case 0x0A: /* slti */
	s->gpr[rt] = (s->gpr[rs] < simm32);
	s->pc += 4;
	s->insn_histo[mipsInsn::SLTI]++;
	break;
      case 0x0B:/* sltiu */
	s->gpr[rt] = ((uint32_t)s->gpr[rs] < (uint32_t)simm32);
	s->pc += 4;
	s->insn_histo[mipsInsn::SLTIU]++;
	break;
      case 0x0c: /* andi */
	s->gpr[rt] = s->gpr[rs] & uimm32;
	s->pc += 4;
	s->insn_histo[mipsInsn::ANDI]++;
	break;
      case 0x0d: /* ori */
	s->gpr[rt] = s->gpr[rs] | uimm32;
	s->pc += 4;
	s->insn_histo[mipsInsn::ORI]++;
	break;
      case 0x0e: /* xori */
	s->gpr[rt] = s->gpr[rs] ^ uimm32;
	s->pc += 4;
	s->insn_histo[mipsInsn::XORI]++;
	break;
      case 0x0F: /* lui */
	uimm32 <<= 16;
	s->gpr[rt] = uimm32;
	s->pc += 4;
	s->insn_histo[mipsInsn::LUI]++;
	break;
      case 0x14:
	branch<EL,branch_type::beql>(inst, s); 
	break;
      case 0x16:
	branch<EL,branch_type::blezl>(inst, s); 
	break;
      case 0x15:
	branch<EL,branch_type::bnel>(inst, s); 
	break;
      case 0x17:
	branch<EL,branch_type::bgtzl>(inst, s); 
	break;
      case 0x20:
	_lb(inst, s);
	break;
      case 0x21:
	_lh<EL>(inst, s);
	break;
      case 0x22: 
	_lwl<EL>(inst, s);
	break;
      case 0x23:
	_lw<EL>(inst, s); 
	break;
      case 0x24:
	_lbu(inst, s);
	break;
      case 0x25:
	_lhu<EL>(inst, s);
	break;
      case 0x26:
	_lwr<EL>(inst, s);
	break;
      case 0x28:
	_sb(inst, s); 
	break;
      case 0x29:
	_sh<EL>(inst, s); 
	break;
      case 0x2a:
	_swl<EL>(inst, s); 
	break;
      case 0x2B:
	_sw<EL>(inst, s); 
	break;
      case 0x2e:
	_swr<EL>(inst, s); 
	break;
      case 0x31:
	_lwc1<EL>(inst, s);
	break;
      case 0x33: /* prefetch */
	s->pc += 4;
	break;
      case 0x35:
	_ldc1<EL>(inst, s);
	break;
      case 0x39:
	_swc1<EL>(inst, s);
	break;
      case 0x3D:
	_sdc1<EL>(inst, s);
	break;
      default:
	printf("%s: Unknown IType instruction (bits=%x) @ pc=0x%08x\n", 
	       __func__, inst, s ? s->pc : 0);
	exit(-1);
	break;
      }
  }
}
