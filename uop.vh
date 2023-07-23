`ifndef __uop_hdr__
`define __uop_hdr__

`include "machine.vh"

typedef enum logic [6:0] 
  {
   SRL,
   SRA,
   SRLV,
   SRAV,
   MFHI,
   MTHI,
   MULT,
   MULTU,
   DIV,
   DIVU,
   ADD,
   SUB,
   SUBU,
   AND,
   OR,
   XOR,
   NOR,
   SLT,
   SLTU,
   MFLO,
   MTLO,
   BLEZ,
   BGTZ,
   ADDIU,
   SLTI,
   SLTIU,
   ANDI,
   MFC0,
   MTC0,
   MFC1,
   MTC1,
   SB,
   SH,
   SW,

   BGEZ,
   TEQ,
   SC,
   MTC1_MERGE,
   MFC1_MERGE,
   MOVI,
   MOV,
   MONITOR,
   
   //known used in riscv design
   ADDU,
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
   LBU,
   LHU,
   ORI,
   XORI,
   J,
   JAL,
   JR,
   JALR,
   BREAK,
   ADDI,
   AUIPC,
   LUI,   
   NOP,
   II //illegal instruction
   } opcode_t;

function logic is_mult(opcode_t op);
   logic     x;
   case(op)
     MULT:
       x = 1'b1;
     MULTU:
       x = 1'b1;
     default:
       x = 1'b0;
   endcase
   return x;
endfunction // is_mult

function logic is_div(opcode_t op);
   logic     x;
   case(op)
     DIV:
       x = 1'b1;
     DIVU:
       x = 1'b1;
     default:
       x = 1'b0;
   endcase
   return x;
endfunction // is_div

function logic is_store(opcode_t op);
   logic     x;
   case(op)
     SB:
       x = 1'b1;
     SH:
       x = 1'b1;
     SW:
       x = 1'b1;
     SC:
       x = 1'b1;
     default:
       x = 1'b0;
   endcase // case (op)
   return x;
endfunction // is_store



typedef struct packed {
   opcode_t op;
   
   logic [`LG_PRF_ENTRIES-1:0] srcA;
   logic 		       srcA_valid;
   logic 		       fp_srcA_valid;
   logic [`LG_PRF_ENTRIES-1:0] srcB;
   logic 		       srcB_valid;
   logic 		       fp_srcB_valid;   
   logic [`LG_PRF_ENTRIES-1:0] dst;
   logic 		       dst_valid;
   logic 		       fp_dst_valid;

   logic 		       hilo_dst_valid;
   logic [`LG_HILO_PRF_ENTRIES-1:0] hilo_dst;

   logic 			    hilo_src_valid;
   logic [`LG_HILO_PRF_ENTRIES-1:0] hilo_src;
     
   logic [15:0] 		    imm;
   logic [`M_WIDTH-17:0] 	    jmp_imm;

   logic [31:0] 		    rvimm;
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
`ifdef VERILATOR
   logic [31:0] 	       clear_id;
`endif
`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
`endif   
} uop_t;



`endif
