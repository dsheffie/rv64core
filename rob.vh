`ifndef __rob_hdr__
`define __rob_hdr__

`include "machine.vh"

typedef struct packed {
   logic       faulted;
   logic       is_ii;
   logic       is_bad_addr;
   logic       is_ret;
   logic       is_call;
   logic       valid_dst;
   logic [4:0] ldst;

   logic [(`LG_PRF_ENTRIES-1):0] pdst;
   logic [(`LG_PRF_ENTRIES-1):0] old_pdst;
   logic [(`M_WIDTH-1):0] 	 pc;
   logic [(`M_WIDTH-1):0] 	 target_pc;
   logic 			 is_br;
   logic 			 is_indirect;
   logic 			 take_br;
   logic 			 is_break;
   logic [`M_WIDTH-1:0] 	 data;
   logic [`LG_PHT_SZ-1:0] 	 pht_idx;

`ifdef ENABLE_CYCLE_ACCOUNTING
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
   logic [`M_WIDTH-1:0]        data;
} complete_t;

typedef struct packed {
   logic [31:0] insn_bytes;
   logic [(`M_WIDTH-1):0] pc;
   logic [(`M_WIDTH-1):0] pred_target;
   logic 		  pred;
   logic [(`LG_PHT_SZ-1):0] pht_idx;
`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
`endif
} insn_fetch_t;

typedef struct packed {
   logic [`M_WIDTH-1:0] addr;
   logic 	is_store;
   logic 	is_load;
   mem_op_t op;
   logic 	spans_cacheline;
   logic 	unaligned;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] dst_ptr;
   logic 		       dst_valid;
   logic [`M_WIDTH-1:0]        data;
   logic [`M_WIDTH-1:0]        pc;
} mem_req_t;

typedef struct packed {
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] src_ptr;
} dq_t;

typedef struct packed {
   logic [`M_WIDTH-1:0] data;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
} mem_data_t;

typedef struct packed {
   logic [`M_WIDTH-1:0] data;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] dst_ptr;
   logic 		       dst_valid;
   logic 		       bad_addr;
   logic [`M_WIDTH-1:0]        pc;
} mem_rsp_t;

typedef struct packed {
   logic       valid;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
} bob_entry_t;



`endif
