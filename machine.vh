`ifndef __machine_hdr__
`define __machine_hdr__

`ifdef VERILATOR
 `define ENABLE_CYCLE_ACCOUNTING 1
 //`define PERFECT_L1D
`endif

`define FPGA 1

//`define SECOND_EXEC_PORT 1

`define LG_M_WIDTH 6

`define MAX_VA 39

`define LG_INT_SCHED_ENTRIES 3

//gshare branch predictor
`define LG_PHT_SZ 16

`define GBL_HIST_LEN 16

//page size
`define LG_PG_SZ 12

`define LG_PRF_ENTRIES 6

//queue between decode and alloc
`define LG_DQ_ENTRIES 2

//queue between fetch and decode
`define LG_FQ_ENTRIES 3

//rob size
`define LG_ROB_ENTRIES 5

//bob size
`define LG_BOB_ENTRIES 2

`define LG_RET_STACK_ENTRIES 3

/* non-uop queue */
`define LG_UQ_ENTRIES 3
/* mem uop queue */
`define LG_MEM_UQ_ENTRIES 3
/* mem data queue */
`define LG_MEM_DQ_ENTRIES 4
/* mem uop queue */
`define LG_MQ_ENTRIES 2

/* mem retry queue */
`define LG_MRQ_ENTRIES 3

`define MUL_LAT 3

`define DIV64_LAT 65
`define MAX_LAT (`DIV64_LAT)


// l1 cacheline length (in bytes)
`define LG_L1D_CL_LEN 4

// l2 cacheline length (in bytes)
`define LG_L2_CL_LEN 4

//number of sets in direct mapped cache
`define LG_L1D_NUM_SETS 8

`define LG_L1I_NUM_SETS 8

`define LG_L2_NUM_SETS 14

`define M_WIDTH (1 << `LG_M_WIDTH)

`define LG_BTB_SZ 7

typedef enum logic [3:0] {
   MEM_LB  = 4'd0,
   MEM_LBU = 4'd1,
   MEM_LH  = 4'd2,
   MEM_LHU = 4'd3,
   MEM_LW  = 4'd4,
   MEM_SB  = 4'd5,
   MEM_SH  = 4'd6,
   MEM_SW  = 4'd7,
   MEM_SCW  = 4'd8,
   MEM_SCD  = 4'd9,
   MEM_NOP = 4'd10,
   MEM_LWU = 4'd11,
   MEM_LD  = 4'd12,
   MEM_SD  = 4'd13,
   MEM_AMOW = 4'd14,
   MEM_AMOD = 4'd15			  			  
} mem_op_t;

function logic [31:0] bswap32(logic [31:0] in);
   return in;
endfunction

function logic [15:0] bswap16(logic [15:0] in);
   return in;
endfunction

function logic sext16(logic [15:0] in);
   return in[15];
endfunction

`endif
