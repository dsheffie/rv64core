#ifndef __mips_insnshh__
#define __mips_insnshh__

#include <type_traits>


#define INSN_LIST(X)				\
  X(SLL)					\
  X(SRL)					\
  X(SRA)					\
  X(SLLV)					\
  X(SRLV)					\
  X(SRAV)					\
  X(JR)						\
  X(JALR)					\
  X(SYSCALL)					\
  X(MFHI)					\
  X(MTHI)					\
  X(MULT)					\
  X(MULTU)					\
  X(DIV)					\
  X(DIVU)					\
  X(ADD)					\
  X(ADDU)					\
  X(SUB)					\
  X(SUBU)					\
  X(AND)					\
  X(OR)						\
  X(XOR)					\
  X(NOR)					\
  X(SLT)					\
  X(SLTU)					\
  X(MFLO)					\
  X(MTLO)					\
  X(BEQ)					\
  X(BNE)					\
  X(BLEZ)					\
  X(BGTZ)					\
  X(ADDI)					\
  X(ADDIU)					\
  X(SLTI)					\
  X(SLTIU)					\
  X(ANDI)					\
  X(ORI)					\
  X(XORI)					\
  X(LUI)					\
  X(J)						\
  X(JAL)					\
  X(MFC0)					\
  X(MTC0)					\
  X(MFC1)					\
  X(MTC1)					\
  X(LW)						\
  X(LB)						\
  X(LBU)					\
  X(LH)						\
  X(LHU)					\
  X(SB)						\
  X(SH)						\
  X(SW)						\
  X(BEQL)					\
  X(BNEL)					\
  X(BLTZ)					\
  X(BGEZ)					\
  X(BLTZL)					\
  X(BGEZL)					\
  X(BGTZL)					\
  X(BLEZL)					\
  X(MOVN)					\
  X(MOVZ)					\
  X(TEQ)					\
  X(EXT)					\
  X(INS)					\
  X(MADD)					\
  X(MADDU)					\
  X(MUL)					\
  X(MSUB)					\
  X(CLZ)					\
  X(LWL)					\
  X(LWR)					\
  X(SWL)					\
  X(SWR)					\
  X(SEB)					\
  X(SEH)					\
  X(DADDU)					\
  X(DADDIU)					\
  X(DSUBU)					\
  X(DSLL)					\
  X(DSRL)					\
  X(DSRA)					\
  X(DSLL32)					\
  X(DSRL32)					\
  X(DSRA32)					\
  X(DSLLV)					\
  X(DSRLV)					\
  X(DSRAV)					\
  X(LD)						\
  X(SD)						\
  X(LDL)					\
  X(LDR)					\
  X(SDL)					\
  X(SDR)					\
  X(BAL)					\
  X(BGEZAL)					\
  X(BGEZALL)					\
  X(RDHWR)					\
  X(SC)						\
  X(SYNC)					\
  X(BREAK)					\
  X(SDC1)					\
  X(LDC1)					\
  X(SWC1)					\
  X(LWC1)					\
  X(SP_ABS)					\
  X(DP_ABS)					\
  X(SP_NEG)					\
  X(DP_NEG)					\
  X(SP_SQRT)					\
  X(DP_SQRT)					\
  X(SP_RSQRT)					\
  X(DP_RSQRT)					\
  X(SP_RECIP)					\
  X(DP_RECIP)					\
  X(SP_ADD)					\
  X(DP_ADD)					\
  X(SP_SUB)					\
  X(DP_SUB)					\
  X(SP_MUL)					\
  X(DP_MUL)					\
  X(SP_DIV)					\
  X(DP_DIV)					\
  X(SP_MOV)					\
  X(DP_MOV)					\
  X(BC1TL)					\
  X(BC1T)					\
  X(BC1FL)					\
  X(BC1F)					\
  X(SP_CMP_LT)					\
  X(DP_CMP_LT)					\
  X(SP_CMP_EQ)					\
  X(DP_CMP_EQ)					\
  X(SP_CMP_LE)					\
  X(DP_CMP_LE)					\
  X(TRUNC_DP_W)					\
  X(TRUNC_SP_W)					\
  X(CVT_W_DP)					\
  X(CVT_W_SP)					\
  X(CVT_SP_DP)					\
  X(CVT_DP_SP)					\
  X(MOVF)					\
  X(MOVT)					\
  X(FP_MOVN)					\
  X(FP_MOVZ)					\
  X(FP_MOVF)					\
  X(FP_MOVT)					\
  X(ERET)					\
  X(DI)						\
  X(EI)						\
  X(WAIT)					\
  X(MOVI)					\
  X(MOV)					\
  X(MONITOR)					\
  X(NOP)					\
  X(II) //illegal instruction

#define INSN(x) x,
#define INSN_PAIR(x) {mipsInsn::x, #x},

enum class mipsInsn { INSN_LIST(INSN) };

static inline std::ostream &operator<<(std::ostream &out, mipsInsn i) {
  static const std::unordered_map<mipsInsn, std::string> m =
    {
     INSN_LIST(INSN_PAIR)
    };
  out << m.at(i);
  return out;
}
#undef INSN
#undef INSN_PAIR
#undef INSN_LIST

#define FLOAT_ENABLE_IF(SZ,T) typename std::enable_if<std::is_floating_point<T>::value and (sizeof(T)==SZ),T>::type* = nullptr

template<typename T, FLOAT_ENABLE_IF(4, T)>
mipsInsn select_fp_insn(mipsInsn dp, mipsInsn sp) {
  return sp;
}

template<typename T, FLOAT_ENABLE_IF(8, T)>
mipsInsn select_fp_insn(mipsInsn dp, mipsInsn sp) {
  return dp;
}

#undef FLOAT_ENABLE_IF
	 
#endif
