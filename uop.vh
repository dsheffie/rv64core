`ifndef __uop_hdr__
`define __uop_hdr__

`include "machine.vh"

typedef enum logic [5:0]
{
 SSTATUS, //0
 SIE, //1
 STVEC, //2
 SSCRATCH, //3
 SEPC, //4
 SCAUSE, //5
 SCOUNTEREN, //6
 STVAL, //7
 SIP, //8
 SATP, //9
 MSTATUS, //10
 MIE, //11 
 MCAUSE, //12
 MCOUNTEREN, //13
 MISA, //14
 MEDELEG, //15
 MIDELEG, //16
 MTVEC, //17
 MEPC, //18
 MIP, //19
 MSCRATCH, //20
 PMPADDR0, //21
 PMPADDR1, //22
 PMPADDR2, //23
 PMPADDR3, //24
 PMPCFG0, //25
 RDCYCLE_CSR, //26
 RDTIME_CSR, //27
 RDINSTRET_CSR, //28
 RDBRANCH_CSR, //29
 RDFAULTEDBRANCH_CSR, //30
 MHARTID,
 BADCSR
} csr_t;


typedef enum logic [6:0] 
  {
   SRL, //0
   SRA, //1
   SRLV, //2
   SRAV, //3
   SLT, //4
   SLTU, //5
   ADDIU, //6
   LRW, //7
   LRD, //8
   SCW, //9
   SCD, //10
   AMOW,//11
   AMOD, //12
   MONITOR, //13
   RDCYCLE, //14
   RDINSTRET, //15
   RDBRANCH, //16
   RDFAULTEDBRANCH,
   ECALL,
   EBREAK,
   MRET,
   SRET,
   CSRRW,
   CSRRS,
   CSRRC,
   CSRRWI,
   CSRRSI,
   CSRRCI,   
   SFENCEVMA,
   //known used in riscv design
   MUL,
   MULH,
   MULHU,
   DIV,
   DIVU,
   REM,
   REMU,
   SLTI,
   SLTIU,   
   ADDU,
   SUBU,
   ANDI,   
   BEQ,
   BGE,
   BGEU,
   BLT, 
   BLTU,  
   BNE,
   SLL,
   SLLI,
   SRAI,
   SRLI,   
   LB,
   LH,
   LW,
   LWU,
   LD,
   LBU,
   LHU,
   SB,
   SH,
   SW,
   SD,
   ORI,
   XORI,
   J,
   JAL,
   JR,
   RET,
   JALR,
   BREAK,
   ADDI,
   AUIPC,
   LUI,   
   NOP,
   AND,
   OR,
   XOR,
   ADDW,
   SUBW,
   ADDIW,
   SLLIW,
   SRLIW,
   SRAIW,
   SRAW,
   MULW,
   DIVW,
   DIVUW,
   REMW,
   REMUW,
   SLLW,
   SRLW,
   FETCH_PF,
   II //illegal instruction
   } opcode_t;

function logic uses_mul(opcode_t op);
   logic     x;
   case(op)
     MUL:
       x = 1'b1;
     MULHU:
       x = 1'b1;
     MULH:
       x = 1'b1;
     MULW:
       x = 1'b1;
     default:
       x = 1'b0;
   endcase
   return x;
endfunction // is_mult

function logic uses_div(opcode_t op);
   logic     x;
   case(op)
     DIV:
       x = 1'b1;
     DIVU:
       x = 1'b1;
     REM:
       x = 1'b1;
     REMU:
       x = 1'b1;
     DIVW:
       x = 1'b1;
     DIVUW:
       x = 1'b1;
     REMW:
       x = 1'b1;
     REMUW:
       x = 1'b1;
     default:
       x = 1'b0;
   endcase
   return x;
endfunction // is_div




typedef struct packed {
   opcode_t op;
   
   logic [`LG_PRF_ENTRIES-1:0] srcA;
   logic		       srcA_valid;
   logic [`LG_PRF_ENTRIES-1:0] srcB;
   logic 		       srcB_valid;
   
   logic [`LG_PRF_ENTRIES-1:0] dst;
   logic 		       dst_valid;


   logic [`M_WIDTH-1:0]        rvimm;
   logic [15:0]		       imm;
   logic [`M_WIDTH-17:0]       jmp_imm;
   
   logic [`M_WIDTH-1:0]        pc;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic 		       serializing_op;
   logic 		       must_restart;
   logic 		       br_pred;
   logic 		       is_int;
   logic 		       is_br;
   logic 		       is_mem;
   logic 		       is_store;
   logic [`LG_PHT_SZ-1:0]      pht_idx;
   logic 		       is_cheap_int;
`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
   logic [31:0]		    raw_insn;   
`endif   
} uop_t;



`endif
