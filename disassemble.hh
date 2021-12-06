#include <string>
#include <cstdint>

#ifndef __DISASSEMBLE_HH__
#define __DISASSEMBLE_HH__

#define R_zero 0
#define R_at 1
#define R_v0 2
#define R_v1 3
#define R_a0 4
#define R_a1 5
#define R_a2 6
#define R_a3 7
#define R_t0 8
#define R_t1 9
#define R_t2 10
#define R_t3 11
#define R_t4 12
#define R_t5 13
#define R_t6 14
#define R_t7 15
#define R_s0 16
#define R_s1 17
#define R_s2 18
#define R_s3 19
#define R_s4 20
#define R_s5 21
#define R_s6 22
#define R_s7 23
#define R_t8 24
#define R_t9 25
#define R_k0 26
#define R_k1 27
#define R_gp 28
#define R_sp 29
#define R_s8 30
#define R_ra 31

#define FMT_S 16 /* single precision */
#define FMT_D 17 /* double precision */
#define FMT_E 18 /* extended precision */
#define FMT_Q 19 /* quad precision */
#define FMT_W 20 /* 32-bit fixed */
#define FMT_L 21 /* 64-bit fixed */

#define COND_F 0
#define COND_UN 1
#define COND_EQ 2
#define COND_UEQ 3
#define COND_OLT 4
#define COND_ULT 5
#define COND_OLE 6
#define COND_ULE 7
#define COND_SF 8
#define COND_NGLE 9
#define COND_SEQ 10
#define COND_NGL 11
#define COND_LT 12
#define COND_NGE 13
#define COND_LE 14
#define COND_NGT 15

#define CP1_CR0 0
#define CP1_CR31 1
#define CP1_CR25 2
#define CP1_CR26 3
#define CP1_CR28 4

const std::string &getCondName(uint32_t c);
const std::string &getGPRName(uint32_t r);

void initCapstone();
void stopCapstone();

std::string getAsmString(uint32_t inst,uint32_t addr);
void disassemble(std::ostream &out, uint32_t inst, uint32_t addr);


#endif
