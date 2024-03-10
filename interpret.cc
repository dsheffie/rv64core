#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "interpret.hh"
#include "disassemble.hh"
#include "helper.hh"
#include "globals.hh"



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

void execRiscv(state_t *s) {
  uint8_t *mem = s->mem;

  uint32_t inst = *reinterpret_cast<uint32_t*>(mem + s->pc);
  uint32_t opcode = inst & 127;
  
  
  
  uint64_t tohost = *reinterpret_cast<uint64_t*>(mem + globals::tohost_addr);
  tohost &= ((1UL<<32)-1);
  if(tohost) {
    handle_syscall(s, tohost);
  }


  if(globals::log) {
    std::cout << std::hex << s->pc << std::dec
	      << " : " << getAsmString(inst, s->pc)
	      << " , opcode " << std::hex
	      << opcode
	      << std::dec
	      << " , icnt " << s->icnt
	      << "\n";
  }
  s->last_pc = s->pc;  

  uint32_t rd = (inst>>7) & 31;
  riscv_t m(inst);

  //#define OLD_GPR
  
#ifdef OLD_GPR
  int64_t old_gpr[32];
  uint64_t old_pc = s->pc;
  memcpy(old_gpr, s->gpr, sizeof(old_gpr));
#endif

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
	switch(m.s.sel)
	  {
	  case 0x0: /* lb */
	    s->gpr[m.l.rd] = static_cast<int32_t>(*(reinterpret_cast<int8_t*>(s->mem + ea)));	 
	    break;
	  case 0x1: /* lh */
	    s->gpr[m.l.rd] = static_cast<int32_t>(*(reinterpret_cast<int16_t*>(s->mem + ea)));	 
	    break;
	  case 0x2: /* lw */
	    s->sext_xlen( *reinterpret_cast<int32_t*>(s->mem + ea), m.l.rd);
	    break;
	  case 0x3: /* ld */
	    s->sext_xlen( *reinterpret_cast<int64_t*>(s->mem + ea), m.l.rd);	    
	    break;
	  case 0x4: {/* lbu */
	    uint32_t b = s->mem[ea];
	    *reinterpret_cast<uint64_t*>(&s->gpr[m.l.rd]) = b;
	    break;
	  }
	  case 0x5: { /* lhu */
	    uint16_t b = *reinterpret_cast<uint16_t*>(s->mem + ea);
	    *reinterpret_cast<uint64_t*>(&s->gpr[m.l.rd]) = b;
	    break;
	  }
	  case 0x6: { /* lwu */
	    uint32_t b = *reinterpret_cast<uint32_t*>(s->mem + ea);
	    *reinterpret_cast<uint64_t*>(&s->gpr[m.l.rd]) = b;
	    break;
	  }	    
	  default:
	    assert(0);
	  }
	s->pc += 4;
	break;
      }
    }
    case 0xf: { /* fence - there's a bunch of 'em */
      s->pc += 4;
      break;
    }
#if 0
    imm[11:0] rs1 000 rd 0010011 ADDI
    imm[11:0] rs1 010 rd 0010011 SLTI
    imm[11:0] rs1 011 rd 0010011 SLTIU
    imm[11:0] rs1 100 rd 0010011 XORI
    imm[11:0] rs1 110 rd 0010011 ORI
    imm[11:0] rs1 111 rd 0010011 ANDI
    0000000 shamt rs1 001 rd 0010011 SLLI
    0000000 shamt rs1 101 rd 0010011 SRLI
    0100000 shamt rs1 101 rd 0010011 SRAI
#endif
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
	    //std::cout << std::hex << "pc     = " << s->pc << std::dec << "\n";
	    //std::cout << std::hex << "simm64 = " << simm64 << std::dec << "\n";
	    //std::cout << std::hex << "srcA   = " << s->gpr[m.i.rs1] << std::dec << "\n";
	    s->sext_xlen((s->gpr[m.i.rs1] & simm64), rd);
	    break;
	    
	  default:
	    std::cout << "implement case " << subop << "\n";
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
	    assert(0);
	    break;
	  }
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
	  int32_t c = a/b;
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
	  uint32_t c = aa/bb;
	  int32_t rr =  *reinterpret_cast<int32_t*>(&c);
	  s->sext_xlen(rr, m.r.rd);
	}
	else if((m.r.sel == 5) & (m.r.special == 32)) { /* sraw */
	  int32_t c = a >> (s->gpr[m.r.rs2]&31);
	  s->sext_xlen(c, m.r.rd);	  
	}
	else if((m.r.sel == 6) & (m.r.special == 1)) { /* remw */
	  int32_t c = a % b;
	  s->sext_xlen(c, m.r.rd);
	}
	else if((m.r.sel == 7) & (m.r.special == 1)) { /* remuw */
	  uint32_t aa = s->get_reg_u32(m.r.rs1);
	  uint32_t bb = s->get_reg_u32(m.r.rs2);
	  uint32_t c = aa%bb;
	  int32_t rr =  *reinterpret_cast<int32_t*>(&c);
	  s->sext_xlen(rr, m.r.rd);
	}
	else {
	  std::cout << "special = " << m.r.special << "\n";
	  std::cout << "sel = " << m.r.sel << "\n";
	  assert(0);
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
      switch(m.s.sel)
	{
	case 0x0: /* sb */
	  s->mem[ea] = *reinterpret_cast<uint8_t*>(&s->gpr[m.s.rs2]);
	  break;
	case 0x1: /* sh */
	  *(reinterpret_cast<uint16_t*>(s->mem + ea)) = *reinterpret_cast<uint16_t*>(&s->gpr[m.s.rs2]);
	  break;
	case 0x2: /* sw */
	  *(reinterpret_cast<int32_t*>(s->mem + ea)) = s->gpr[m.s.rs2];
	  break;
	case 0x3: /* sd */
	  *(reinterpret_cast<int64_t*>(s->mem + ea)) = s->gpr[m.s.rs2];
	  break;
	default:
	  assert(0);
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
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] / s->gpr[m.r.rs2];
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
		*reinterpret_cast<uint64_t*>(&s->gpr[m.r.rd]) = u_rs1 / u_rs2;
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
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] % s->gpr[m.r.rs2];
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
		*reinterpret_cast<uint64_t*>(&s->gpr[m.r.rd]) = u_rs1 % u_rs2;
		//std::cout << std::hex << u_rs1 << std::dec << "\n";
		//std::cout << std::hex << u_rs2 << std::dec << "\n";
		//std::cout << std::hex << (u_rs1 % u_rs2) << std::dec << "\n";
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

    case 0x73:
      if((inst >> 7) == 0) {
	s->brk = 1;
      }
      else if( ((inst >> 12) & 3) == 2) {
	switch(inst>>20)
	  {
	  case 0xc00:
	    s->gpr[(inst>>7) & 31] = s->icnt;
	    break;
	  default:
	    break;
	  }
	    
	s->pc += 4;
      }
      else {
	s->pc += 4;
      }
      break;
    
    default:
      std::cout << std::hex << s->pc << std::dec
		<< " : " << getAsmString(inst, s->pc)
		<< " , opcode " << std::hex
		<< opcode
		<< std::dec
		<< " , icnt " << s->icnt
		<< "\n";
      std::cout << *s << "\n";
      exit(-1);
      break;
    }

  s->icnt++;

#ifdef OLD_GPR
  for(int i = 0; i < 32; i++){
    if(old_gpr[i] != s->gpr[i]) {
      std::cout << "\t" << getGPRName(i) << " changed from "
		<< std::hex
		<< old_gpr[i]
		<< " to "
		<< s->gpr[i]	
		<< " at pc "
		<< old_pc
		<< std::dec
		<< "\n";
    }
  }
#endif
  
}

void runRiscv(state_t *s, uint64_t dumpIcnt) {
  while(s->brk==0 and (s->icnt < s->maxicnt) and (s->icnt < dumpIcnt)) {
    execRiscv(s);
  }
}

