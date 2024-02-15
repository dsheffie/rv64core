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
#include <map>
#include <stack>

#ifdef USE_SDL
#include <SDL2/SDL.h>
#endif

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

void initState(state_t *s, int xlen) {
  memset(s, 0, sizeof(state_t));
  s->xlen = xlen;
}


void execRiscv(state_t *s) {
  uint8_t *mem = s->mem;

  uint32_t inst = *reinterpret_cast<uint32_t*>(mem + s->pc);
  uint32_t opcode = inst & 127;
  
#if 0
  std::cout << std::hex << s->pc << "\n";
  for(int r = 0; r < 32; r++) {
    std::cout << "\t" << s->gpr[r] << "\n";
  }
  std::cout << std::dec;
#endif				    
  
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

#if OLD_GPR
  int32_t old_gpr[32];
  memcpy(old_gpr, s->gpr, 4*32);
#endif
  
  switch(opcode)
    {
    case 0x3: {
      if(m.l.rd != 0) {
	int32_t disp = m.l.imm11_0;
	if((inst>>31)&1) {
	  disp |= 0xfffff000;
	}
	uint32_t ea = disp + s->gpr[m.l.rs1];
	switch(m.s.sel)
	  {
	  case 0x0: /* lb */
	    s->gpr[m.l.rd] = static_cast<int32_t>(*(reinterpret_cast<int8_t*>(s->mem + ea)));	 
	    break;
	  case 0x1: /* lh */
	    s->gpr[m.l.rd] = static_cast<int32_t>(*(reinterpret_cast<int16_t*>(s->mem + ea)));	 
	    break;
	  case 0x2: /* lw */
	    s->gpr[m.l.rd] = *(reinterpret_cast<int32_t*>(s->mem + ea));
	    break;
	  case 0x4: {/* lbu */
	    uint32_t b = s->mem[ea];
	    *reinterpret_cast<uint32_t*>(&s->gpr[m.l.rd]) = b;
	    break;
	  }
	  case 0x5: { /* lhu */
	    uint16_t b = *reinterpret_cast<uint16_t*>(s->mem + ea);
	    *reinterpret_cast<uint32_t*>(&s->gpr[m.l.rd]) = b;
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
      uint32_t subop =(inst>>12)&7;
      uint32_t shamt = (inst>>20) & 31;

      if(rd != 0) {
	switch(m.i.sel)
	  {
	  case 0: /* addi */
	    s->gpr[rd] = sext_xlen(s->gpr[m.i.rs1] + simm32, s->xlen);
	    break;
	  case 1: /* slli */
	    s->gpr[rd] = (*reinterpret_cast<uint32_t*>(&s->gpr[m.i.rs1])) << shamt;
	    break;
	  case 2: /* slti */
	    s->gpr[rd] = (s->gpr[m.i.rs1] < simm32);
	    break;
	  case 3: { /* sltiu */
	    uint32_t uimm32 = static_cast<uint32_t>(simm32);
	    uint32_t u_rs1 = *reinterpret_cast<uint32_t*>(&s->gpr[m.i.rs1]);
	    s->gpr[rd] = (u_rs1 < uimm32);
	    break;
	  }
	  case 4: /* xori */
	    s->gpr[rd] = s->gpr[m.i.rs1] ^ simm32;
	    break;
	  case 5: { /* srli & srai */
	    uint32_t sel =  (inst >> 25) & 127;	    
	    if(sel == 0) { /* srli */
	      s->gpr[rd] = (*reinterpret_cast<uint32_t*>(&s->gpr[m.i.rs1]) >> shamt);
	    }
	    else if(sel == 32) { /* srai */
	      s->gpr[rd] = s->gpr[m.i.rs1] >> shamt;
	    }
	    else {
	      std::cout << "sel = " << sel << "\n";
	      assert(0);
	    }
	    break;
	  }
	  case 6: /* ori */
	    s->gpr[rd] = s->gpr[m.i.rs1] | simm32;
	    break;
	  case 7: /* andi */
	    s->gpr[rd] = s->gpr[m.i.rs1] & simm32;
	    break;
	    
	  default:
	    std::cout << "implement case " << subop << "\n";
	    assert(false);
	  }
      }
      s->pc += 4;
      break;
    }
    case 0x23: {
      int32_t disp = m.s.imm4_0 | (m.s.imm11_5 << 5);
      disp |= ((inst>>31)&1) ? 0xfffff000 : 0x0;
      uint32_t ea = disp + s->gpr[m.s.rs1];
      //std::cout << "STORE EA " << std::hex << ea << std::dec << "\n";      
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
	default:
	  assert(0);
	}
      s->pc += 4;
      break;
    }
      
      //imm[31:12] rd 011 0111 LUI
    case 0x37:
      if(rd != 0) {
	s->gpr[rd] = inst & 0xfffff000;
      }
      s->pc += 4;
      break;
      //imm[31:12] rd 0010111 AUIPC
    case 0x17: /* is this sign extended */
      if(rd != 0) {
	uint32_t imm = inst & (~4095U);
	uint32_t u = static_cast<uint32_t>(s->pc) + imm;
	*reinterpret_cast<uint32_t*>(&s->gpr[rd]) = u;
	//std::cout << "u = " << std::hex << u << std::dec << "\n";
	//if(s->pc == 0x80000084) exit(-1);
      }
      s->pc += 4;
      break;
      
      //imm[11:0] rs1 000 rd 1100111 JALR
    case 0x67: {
      int32_t tgt = m.jj.imm11_0;
      tgt |= ((inst>>31)&1) ? 0xfffff000 : 0x0;
      tgt += s->gpr[m.jj.rs1];
      tgt &= ~(1U);
      if(m.jj.rd != 0) {
	s->gpr[m.jj.rd] = s->pc + 4;
      }
      s->pc = tgt;
      break;
    }

      
      //imm[20|10:1|11|19:12] rd 1101111 JAL
    case 0x6f: {
      int32_t jaddr =
	(m.j.imm10_1 << 1)   |
	(m.j.imm11 << 11)    |
	(m.j.imm19_12 << 12) |
	(m.j.imm20 << 20);
      jaddr |= ((inst>>31)&1) ? 0xffe00000 : 0x0;
      if(rd != 0) {
	s->gpr[rd] = s->pc + 4;
      }
      s->pc += jaddr;
      break;
    }
    case 0x33: {      
      if(m.r.rd != 0) {
	uint32_t u_rs1 = *reinterpret_cast<uint32_t*>(&s->gpr[m.r.rs1]);
	uint32_t u_rs2 = *reinterpret_cast<uint32_t*>(&s->gpr[m.r.rs2]);
	switch(m.r.sel)
	  {
	  case 0x0: /* add & sub */
	    switch(m.r.special)
	      {
	      case 0x0: /* add */
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] + s->gpr[m.r.rs2];
		break;
	      case 0x1: /* mul */
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] * s->gpr[m.r.rs2];
		break;
	      case 0x20: /* sub */
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] - s->gpr[m.r.rs2];
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
		s->gpr[m.r.rd] = s->gpr[m.r.rs1] << (s->gpr[m.r.rs2] & 31);
		break;
	      case 0x1: { /* MULH */
		int64_t t = static_cast<int64_t>(s->gpr[m.r.rs1]) * static_cast<int64_t>(s->gpr[m.r.rs2]);
		s->gpr[m.r.rd] = (t>>32);
		break;
	      }		
	      default:
		std::cout << "sel = " << m.r.sel << ", special = " << m.r.special << "\n";
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
		uint64_t t = static_cast<uint64_t>(u_rs1) * static_cast<uint64_t>(u_rs2);
		*reinterpret_cast<uint32_t*>(&s->gpr[m.r.rd]) = (t>>32);
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
		s->gpr[m.r.rd] = (s->gpr[m.r.rs2]==0) ? ~0 : s->gpr[m.r.rs1] / s->gpr[m.r.rs2];
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
		s->gpr[rd] = (*reinterpret_cast<uint32_t*>(&s->gpr[m.r.rs1]) >> (s->gpr[m.r.rs2] & 31));
		break;
	      case 0x1: {
		*reinterpret_cast<uint32_t*>(&s->gpr[m.r.rd]) = u_rs2 ? (u_rs1 / u_rs2) : ~0U;
		break;
	      }
	      case 0x20: /* sra */
		s->gpr[rd] = s->gpr[m.r.rs1] >> (s->gpr[m.r.rs2] & 31);
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
		s->gpr[m.r.rd] = s->gpr[m.r.rs2] ? (s->gpr[m.r.rs1] % s->gpr[m.r.rs2]) : ~0;
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
		*reinterpret_cast<uint32_t*>(&s->gpr[m.r.rd]) = u_rs2 ? (u_rs1 % u_rs2) : ~0U;
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
      int32_t disp =
	(m.b.imm4_1 << 1)  |
	(m.b.imm10_5 << 5) |	
        (m.b.imm11 << 11)  |
        (m.b.imm12 << 12);
      disp |= m.b.imm12 ? 0xffffe000 : 0x0;
      bool takeBranch = false;
      uint32_t u_rs1 = *reinterpret_cast<uint32_t*>(&s->gpr[m.b.rs1]);
      uint32_t u_rs2 = *reinterpret_cast<uint32_t*>(&s->gpr[m.b.rs2]);
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
      else {
	s->pc += 4;
      }
      break;
    
    default:
      std::cout << "opcode " << std::hex << opcode << std::dec << "\n";
      std::cout << *s << "\n";
      exit(-1);
      break;
    }

  s->icnt++;
#if OLD_GPR
  for(int i = 0; i < 32; i++){
    if(old_gpr[i] != s->gpr[i]) {
      std::cout << "\t" << getGPRName(i) << " changed from "
		<< std::hex
		<< old_gpr[i]
		<< " to "
		<< s->gpr[i]
		<< std::dec
		<< "\n";
    }
  }
#endif
}



void handle_syscall(state_t *s, uint64_t tohost) {
  uint8_t *mem = s->mem;  
  if(tohost & 1) {
    /* exit */
    s->brk = 1;
    return;
  }
  uint64_t *buf = reinterpret_cast<uint64_t*>(mem + tohost);
  //std::cout << "syscall id " << buf[0] << "\n";
  switch(buf[0])
    {
    case SYS_write: /* int write(int file, char *ptr, int len) */
      buf[0] = write(buf[1], (void*)(s->mem + buf[2]), buf[3]);
      if(buf[1]==1)
	fflush(stdout);
      else if(buf[1]==2)
	fflush(stderr);
      break;
    case SYS_open: {
      const char *path = reinterpret_cast<const char*>(s->mem + buf[1]);
      buf[0] = open(path, remapIOFlags(buf[2]), S_IRUSR|S_IWUSR);
      break;
    }
    case SYS_close: {
      if(buf[1] > 2) {
	buf[0] = close(buf[1]);
      }
      else {
	buf[0] = 0;
      }
      break;
    }
    case SYS_read: {
      buf[0] = read(buf[1], reinterpret_cast<char*>(s->mem + buf[2]), buf[3]); 
      break;
    }
    case SYS_lseek: {
      buf[0] = lseek(buf[1], buf[2], buf[3]);
      break;
    }
    case SYS_fstat : {
      struct stat native_stat;
      stat32_t *host_stat = reinterpret_cast<stat32_t*>(s->mem + buf[2]);
      int rc = fstat(buf[1], &native_stat);
      host_stat->st_dev = native_stat.st_dev;
      host_stat->st_ino = native_stat.st_ino;
      host_stat->st_mode = native_stat.st_mode;
      host_stat->st_nlink = native_stat.st_nlink;
      host_stat->st_uid = native_stat.st_uid;
      host_stat->st_gid = native_stat.st_gid;
      host_stat->st_size = native_stat.st_size;
      host_stat->_st_atime = native_stat.st_atime;
      host_stat->_st_mtime = 0;
      host_stat->_st_ctime = 0;
      host_stat->st_blksize = native_stat.st_blksize;
      host_stat->st_blocks = native_stat.st_blocks;
            buf[0] = rc;
      break;
    }
    case SYS_stat : {
      buf[0] = 0;
      break;
    }
    case SYS_gettimeofday: {
      static_assert(sizeof(struct timeval)==16, "timeval has a weird size");
      struct timeval *tp = reinterpret_cast<struct timeval*>(s->mem + buf[1]);
      struct timezone *tzp = reinterpret_cast<struct timezone*>(s->mem + buf[2]);
      buf[0] = gettimeofday(tp, tzp);
      break;
    }
    case 0x1337: {
#ifdef USE_SDL
      printf("draw frame syscall\n");
      SDL_LockSurface(globals::sdlscr);
      uint8_t *px = reinterpret_cast<uint8_t*>(globals::sdlscr->pixels);
      memcpy(px, (s->mem + buf[1]), sizeof(uint32_t)*FB_WIDTH*FB_HEIGHT);
      SDL_UnlockSurface(globals::sdlscr);
      SDL_UpdateWindowSurface(globals::sdlwin);
      SDL_PumpEvents();
#endif
      break;
    }
    default:
      std::cout << "syscall " << buf[0] << " unsupported\n";
      exit(-1);
    }
  //ack
  *reinterpret_cast<uint64_t*>(mem + globals::tohost_addr) = 0;
  *reinterpret_cast<uint64_t*>(mem + globals::fromhost_addr) = 1;
}
