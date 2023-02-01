`ifndef __rob_hdr__
`define __rob_hdr__

`include "machine.vh"

typedef struct packed {
   //logic       complete;
   logic       faulted;
   logic       is_ii;
   logic       take_trap;
   logic       is_store;
   logic       is_ret;
   logic       is_call;
   logic       is_eret;
   logic       is_wait;
   logic       valid_dst;
   logic       valid_hilo_dst;
   logic       valid_fcr_dst;
   logic       valid_fp_dst;
   logic       has_delay_slot;
   logic       has_nullifying_delay_slot;
   logic       in_delay_slot;
   logic [4:0] ldst;

   logic [(`LG_PRF_ENTRIES-1):0] pdst;
   logic [(`LG_PRF_ENTRIES-1):0] old_pdst;
   logic [(`M_WIDTH-1):0] 	 pc;
   logic [(`M_WIDTH-1):0] 	 target_pc;
   logic 			 is_br;
   logic 			 is_indirect;
   logic 			 take_br;
   logic 			 is_break;
   logic 			 is_syscall;
   logic [63:0] 		 data;
   logic [`LG_PHT_SZ-1:0] 	 pht_idx;

`ifdef ENABLE_CYCLE_ACCOUNTING
   logic 			 missed_l1d;
   logic 			 is_mem;
   logic 			 is_fp;
   logic [63:0] 	    fetch_cycle;
   logic [63:0] 	    alloc_cycle;
   logic [63:0] 	    complete_cycle;
`endif
   
} rob_entry_t;

typedef struct packed {
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic 		       complete;
   logic 		       faulted;
   logic [`M_WIDTH-1:0]        restart_pc;
   logic 		       take_br;
   logic 		       is_ii;
   logic 		       take_trap;
   logic [63:0] 	       data;
} complete_t;

typedef struct packed {
   logic [31:0] data;
   logic [(`M_WIDTH-1):0] pc;
   logic [(`M_WIDTH-1):0] pred_target;
   logic 		  pred;
   logic [(`LG_PHT_SZ-1):0] pht_idx;
`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
`endif
} insn_fetch_t;

typedef struct packed {
   logic [(`M_WIDTH-1):0] addr;
   logic 		  in_storebuf;
   logic 		  is_store;
   logic 		  is_fp;
   /* for merging */
   logic 		  lwc1_lo;
   mem_op_t op;
   logic [63:0] 	  data;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] dst_ptr;
   logic 		       dst_valid;
   logic 		       fp_dst_valid;
   logic [`M_WIDTH-1:0]        pc;
   logic [31:0] 	       uuid;
} mem_req_t;

typedef struct packed {
   mem_op_t op;
   logic [63:0] data;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] dst_ptr;
   logic 		       dst_valid;
   logic 		       fp_dst_valid;
   logic 		       faulted;
   logic 		       was_mem;
   logic 		       missed_l1d;
   logic [`M_WIDTH-1:0]        pc;
} mem_rsp_t;


`endif
