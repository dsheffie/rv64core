#ifndef __INTERPRET_HH__
#define __INTERPRET_HH__

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <ostream>
#include <cassert>
#include <unordered_map>

#include "sparse_mem.hh"
#include "mips_insns.hh"

#define IS_LITTLE_ENDIAN false

/* from gdb simulator */
#define RSVD_INSTRUCTION           (0x00000005)
#define RSVD_INSTRUCTION_MASK      (0xFC00003F)
#define RSVD_INSTRUCTION_ARG_SHIFT 6
#define RSVD_INSTRUCTION_ARG_MASK  0xFFFFF  
#define IDT_MONITOR_BASE           0xBFC00000
#define IDT_MONITOR_SIZE           2048
#define MARGS 20

#define K1SIZE  (0x80000000)

enum class fpMode {
  mipsii, mipsiv, mips32
};

enum class fpOperation {
  abs,neg,mov,add,
  sub,mul,div,sqrt,
  rsqrt,recip,truncl,
  truncw, cvts, cvtd,
  unknown
};

static inline fpOperation decode_fp(uint32_t inst) {
  uint32_t opcode = inst>>26;
  uint32_t functField = (inst>>21) & 31;
  uint32_t lowop = inst & 63;  
  uint32_t fmt = (inst >> 21) & 31;
  uint32_t nd_tf = (inst>>16) & 3;
  uint32_t lowbits = inst & ((1<<11)-1);
  
  if(opcode != 0x11) {
    return fpOperation::unknown;
  }
  opcode &= 0x3;
  if(fmt == 0x8) {
    return fpOperation::unknown;
    }
  else if((lowbits == 0) && ((functField==0x0) || (functField==0x4))) {
    return fpOperation::unknown;    
  }
  if((lowop >> 4) == 3) {
    return fpOperation::unknown;	  
  }
  switch(lowop)
    {
    case 0x0:
      return fpOperation::add;
    case 0x1:
      return fpOperation::sub;
    case 0x2:
      return fpOperation::mul;
    case 0x3:
      return fpOperation::div;
    case 0x4:
      return fpOperation::sqrt;
    case 0x5:
      return fpOperation::abs;
    case 0x6:
      return fpOperation::mov;
    case 0x7:
      return fpOperation::neg;
      //case 0x9:
      //return fpOperation::truncl;
      //case 0xd:
      //return fpOperation::truncw;
    // case 0x11:
    //   _fmovc(inst, s);
    //   break;
    // case 0x12:
    //   _fmovz(inst, s);
    //   break;
    // case 0x13:
    //   _fmovn(inst, s);
    //   break;
    case 0x15:
      return fpOperation::recip;
    case 0x16:
      return fpOperation::rsqrt;
    // case 0x20:
    //   return fpOperation::cvts;
    // case 0x21:
    //   return fpOperation::cvtd;
    default:
      break;
    }
  return fpOperation::unknown;
}

enum class branch_type {
  beq, bne, blez, bgtz,
  beql, bnel, blezl, bgtzl,
  bgez, bgezl, bltz, bltzl,
  bgezal,
  bc1f, bc1t, bc1fl, bc1tl
};

/* stolen from QEMU */
struct image_info32 {
  uint32_t       load_bias;
  uint32_t       load_addr;
  uint32_t       start_code;
  uint32_t       end_code;
  uint32_t       start_data;
  uint32_t       end_data;
  uint32_t       start_brk;
  uint32_t       brk;
  uint32_t       reserve_brk;
  uint32_t       start_mmap;
  uint32_t       start_stack;
  uint32_t       stack_limit;
  uint32_t       entry;
  uint32_t       code_offset;
  uint32_t       data_offset;
  uint32_t       saved_auxv;
  uint32_t       auxv_len;
  uint32_t       arg_start;
  uint32_t       arg_end;
  uint32_t       arg_strings;
  uint32_t       env_strings;
  uint32_t       file_string;
  uint32_t       elf_flags;
  int		 personality;
  uint32_t       alignment;
  
  /* The fields below are used in FDPIC mode.  */
  uint32_t       loadmap_addr;
  uint16_t        nsegs;
  void           *loadsegs;
  uint32_t       pt_dynamic_addr;
  uint32_t       interpreter_loadmap_addr;
  uint32_t       interpreter_pt_dynamic_addr;
  struct image_info32 *other_info;
  /* For target-specific processing of NT_GNU_PROPERTY_TYPE_0. */
  uint32_t        note_flags;
  int             fp_abi;
  int             interp_fp_abi;

  uint32_t target_brk;
  uint32_t target_original_brk;
  uint32_t brk_page;
  uint32_t pgsz;
};


struct image_info64 {
  uint64_t       load_bias;
  uint64_t       load_addr;
  uint64_t       start_code;
  uint64_t       end_code;
  uint64_t       start_data;
  uint64_t       end_data;
  uint64_t       start_brk;
  uint64_t       brk;
  uint64_t       reserve_brk;
  uint64_t       start_mmap;
  uint64_t       start_stack;
  uint64_t       stack_limit;
  uint64_t       entry;
  uint64_t       code_offset;
  uint64_t       data_offset;
  uint64_t       saved_auxv;
  uint64_t       auxv_len;
  uint64_t       arg_start;
  uint64_t       arg_end;
  uint64_t       arg_strings;
  uint64_t       env_strings;
  uint64_t       file_string;
  uint64_t        elf_flags;
  int		personality;
  uint64_t       alignment;
  
  /* The fields below are used in FDPIC mode.  */
  uint64_t       loadmap_addr;
  uint16_t        nsegs;
  void           *loadsegs;
  uint64_t       pt_dynamic_addr;
  uint64_t       interpreter_loadmap_addr;
  uint64_t       interpreter_pt_dynamic_addr;
  struct image_info32 *other_info;
  /* For target-specific processing of NT_GNU_PROPERTY_TYPE_0. */
  uint64_t        note_flags;
  int             fp_abi;
  int             interp_fp_abi;
};

struct timeval32_t {
  uint32_t tv_sec;
  uint32_t tv_usec;
  timeval32_t() : tv_sec(0), tv_usec(0) {}
};

struct tms32_t { 
  uint32_t tms_utime;
  uint32_t tms_stime;
  uint32_t tms_cutime;
  uint32_t tms_cstime;
};

struct stat32_t {
  uint16_t st_dev;
  uint16_t st_ino;
  uint32_t st_mode;
  uint16_t st_nlink;
  uint16_t st_uid;
  uint16_t st_gid;
  uint16_t st_rdev;
  uint32_t st_size;
  uint32_t _st_atime;
  uint32_t st_spare1;
  uint32_t _st_mtime;
  uint32_t st_spare2;
  uint32_t _st_ctime;
  uint32_t st_spare3;
  uint32_t st_blksize;
  uint32_t st_blocks;
  uint32_t st_spare4[2];
};

enum class fp_reg_state { unknown, sp, dp };

class state_t{
public:
  uint32_t pc = 0;
  int32_t gpr[32] = {0};
  int32_t lo = 0;
  int32_t hi = 0;
  uint32_t cpr0[32] = {0};
  uint32_t cpr1[32] = {0};
  uint32_t fcr1[5] = {0};
  uint64_t icnt = 0;
  uint8_t brk = 0;
  uint64_t maxicnt = 0;
  sparse_mem &mem;
  image_info32 linux_image;
  fp_reg_state cpr1_state[32] = {fp_reg_state::unknown};
  std::unordered_map<mipsInsn,uint64_t> insn_histo;
  state_t(sparse_mem &mem) : mem(mem) {
    memset(&linux_image, 0, sizeof(linux_image));
  }
  ~state_t();
};

struct rtype_t {
  uint32_t opcode : 6;
  uint32_t sa : 5;
  uint32_t rd : 5;
  uint32_t rt : 5;
  uint32_t rs : 5;
  uint32_t special : 6;
};

struct itype_t {
  uint32_t imm : 16;
  uint32_t rt : 5;
  uint32_t rs : 5;
  uint32_t opcode : 6;
};

struct coproc1x_t {
  uint32_t fmt : 3;
  uint32_t id : 3;
  uint32_t fd : 5;
  uint32_t fs : 5;
  uint32_t ft : 5;
  uint32_t fr : 5;
  uint32_t opcode : 6;
};

struct lwxc1_t {
  uint32_t id : 6;
  uint32_t fd : 5;
  uint32_t pad : 5;
  uint32_t index : 5;
  uint32_t base : 5;
  uint32_t opcode : 6;
};


union mips_t {
  rtype_t r;
  itype_t i;
  coproc1x_t c1x;
  lwxc1_t lc1x;
  uint32_t raw;
  mips_t(uint32_t x) : raw(x) {}
};


static inline bool is_jr(uint32_t inst, bool r31 = false) {
  uint32_t opcode = inst>>26;
  uint32_t funct = inst & 63;
  uint32_t rs = (inst >> 21) & 31;
  bool jr = (opcode==0) and (funct == 0x08);
  if(jr) {
    return r31 ? (rs==31) : true;
  }
  return false;
}

static inline bool is_jal(uint32_t inst) {
  uint32_t opcode = inst>>26;
  return (opcode == 3);
}

inline bool is_jalr(uint32_t inst) {
  uint32_t opcode = inst>>26;
  return (opcode==0) && ((inst&63) == 0x9);
}


static inline bool is_j(uint32_t inst) {
  uint32_t opcode = inst>>26;
  return (opcode == 2);
}

static inline uint32_t get_jump_target(uint32_t pc, uint32_t inst) {
  assert(is_jal(inst) or is_j(inst));
  static const uint32_t pc_mask = (~((1U<<28)-1));
  uint32_t jaddr = (inst & ((1<<26)-1)) << 2;
  return ((pc + 4)&pc_mask) | jaddr;
}

static inline bool is_memory(uint32_t inst) {
  uint32_t opcode = inst>>26;
  switch(opcode)
    {
    case 0x20: //_lb(inst, s);
    case 0x21: //_lh<EL>(inst, s);
    case 0x22: //_lwl<EL>(inst, s);
    case 0x23: //_lw<EL>(inst, s);
    case 0x24: //_lbu(inst, s);
    case 0x25: //_lhu<EL>(inst, s);
    case 0x26: //_lwr<EL>(inst, s);
    case 0x28: //_sb(inst, s);
    case 0x29: //_sh<EL>(inst, s);
    case 0x2a: //_swl<EL>(inst, s);
    case 0x2b: //_sw<EL>(inst, s);
    case 0x2e: //_swr<EL>(inst, s); 
    case 0x31: //_lwc1<EL>(inst, s);
    case 0x35: //_ldc1<EL>(inst, s);
    case 0x39: //_swc1<EL>(inst, s);
    case 0x3d: //_sdc1<EL>(inst, s);
      return true;
    default:
      break;
    }
  return false;
}

static inline bool is_branch(uint32_t inst) {
  uint32_t opcode = inst>>26;
  uint32_t rt = ((inst>>16) & 31);
  switch(opcode)
    {
    case 0x01:
      return (rt == 0) || (rt == 1);
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
      return true;
    default:
      break;
    }
  return false;
}

static inline bool is_branch_likely(uint32_t inst) {
  uint32_t opcode = inst>>26;
  uint32_t rt = ((inst>>16) & 31);
  switch(opcode)
    {
    case 0x01:
      return (rt == 2) || (rt == 3);
    case 0x14: /* BEQL */
    case 0x15: /* BNEL */
    case 0x16: /* BNEZL */
    case 0x17: /* BGTZL */
      return true;
    default:
      break;
    }
  return false;
}

static inline uint32_t get_branch_target(uint32_t pc, uint32_t inst) {
  int16_t himm = (int16_t)(inst & ((1<<16) - 1));
  int32_t imm = ((int32_t)himm) << 2;
  return  pc+4+imm; 
}

void initState(state_t *s);
void execMips(state_t *s);

void mkMonitorVectors(state_t *s);

std::ostream &operator<<(std::ostream &out, const state_t & s);

struct new_utsname;

int sys_uname(struct new_utsname *buf);

bool is_store_insn(state_t *s);

#define CPR0_SR 12

#define VA2PA(x) ((x & 0x1fffffff))

#endif
