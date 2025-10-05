`ifndef __machine_hdr__
`define __machine_hdr__

`ifdef VERILATOR
 `define ENABLE_CYCLE_ACCOUNTING 1
/* `define PERFECT_L1_CACHES 1 */
`endif

`define FPGA 1

`define SECOND_EXEC_PORT 1
`define TWO_SRC_CHEAP 1

//`define FOUR_CYCLE_L1D 1

`define LG_M_WIDTH 6

`define MAX_VA 39

`define NUM_DTLB_ENTRIES 32
`define NUM_ITLB_ENTRIES 16

`define LG_INT_SCHED0_ENTRIES 3
`define LG_INT_SCHED1_ENTRIES 2
`define LG_MEM_SCHED_ENTRIES 2


//gshare branch predictor
`define LG_PHT_SZ 16
`define LG_BPU_TBL_SZ 16

`define GBL_HIST_LEN 16

//page size
`define LG_PG_SZ 12

`define LG_PRF_ENTRIES 7

//queue between decode and alloc
`define LG_DQ_ENTRIES 2

//queue between fetch and decode
`define LG_FQ_ENTRIES 3

//rob size
`define LG_ROB_ENTRIES 5

`define LG_RET_STACK_ENTRIES 3

/* non-uop queue */
`define LG_UQ_ENTRIES 3
/* mem uop queue */
`define LG_MEM_UQ_ENTRIES 3

/* mem data queue */
`define LG_MEM_DQ_ENTRIES 3

/* mem uop queue */
`define LG_MQ_ENTRIES 2

`define LG_MDQ_ENTRIES 3

`define LG_EB_ENTRIES 1

/* mem retry queue */
`define LG_MRQ_ENTRIES 3

`define MUL_LAT 3

`define DIV64_LAT 66
`define MAX_LAT (`DIV64_LAT)


// l1 cacheline length (in bytes)
`define LG_L1D_CL_LEN 4

// l2 cacheline length (in bytes)
`define LG_L2_CL_LEN 4

//number of sets in direct mapped cache
`define LG_L1D_NUM_SETS 12
`define LG_L1I_NUM_SETS 11

//size 13 -> 2-way assoc * 16 byte lines * 2^13 -> 2^5 * 2^13 -> 2^18 (256k cache)
//size 14 -> 2-way assoc * 16 byte lines * 2^14 -> 2^5 * 2^14 -> 2^19 (512k cache)
`define LG_L2_NUM_SETS 14

`define LG_L2_REQ_TAGS 2

`define M_WIDTH (1 << `LG_M_WIDTH)

`define PA_WIDTH 32

`define LG_BTB_SZ 7

`define UC_START (64'h40500000)
`define UC_END   (64'h40510000)

`define MTIMECMP_ADDR (64'h40004000)

typedef enum logic [4:0] {
   MEM_LB  = 'd0,
   MEM_LBU = 'd1,
   MEM_LH  = 'd2,
   MEM_LHU = 'd3,
   MEM_LW  = 'd4,
   MEM_SB  = 'd5,
   MEM_SH  = 'd6,
   MEM_SW  = 'd7,
   MEM_SCW  = 'd8,
   MEM_SCD  = 'd9,
   MEM_NOP = 'd10,
   MEM_LWU = 'd11,
   MEM_LD  = 'd12,
   MEM_SD  = 'd13,
   MEM_AMOW = 'd14,
   MEM_AMOD = 'd15,
   MEM_PREFETCH = 'd16			  			  
} mem_op_t;

`endif
