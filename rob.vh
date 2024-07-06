`ifndef __rob_hdr__
`define __rob_hdr__

`include "machine.vh"


typedef enum logic [4:0] {
   MISALIGNED_FETCH = 'd0,
   FAULT_FETCH = 'd1,
   ILLEGAL_INSTRUCTION = 'd2,			  
   BREAKPOINT = 'd3,
   MISALIGNED_LOAD = 'd4,
   FAULT_LOAD = 'd5,
   MISALIGNED_STORE = 'd6,
   FAULT_STORE = 'd7,
   USER_ECALL = 'd8,
   SUPERVISOR_ECALL = 'd9,
   HYPERVISOR_ECALL = 'd10,
   MACHINE_ECALL = 'd11,
   FETCH_PAGE_FAULT = 'd12,
   LOAD_PAGE_FAULT = 'd13,
/* note - 14 is reserved */			  
   STORE_PAGE_FAULT = 'd15			  
} cause_t;


typedef struct packed {
   logic       faulted;
   logic       has_cause;
   logic [4:0] cause;
   logic       mark_page_dirty;   
   logic       is_ret;
   logic       is_call;
   logic       is_irq;
   logic       valid_dst;
   logic [4:0] ldst;

   logic [(`LG_PRF_ENTRIES-1):0] pdst;
   logic [(`LG_PRF_ENTRIES-1):0] old_pdst;
   logic [(`M_WIDTH-1):0] 	 pc;
   logic [(`M_WIDTH-1):0] 	 target_pc;
   logic 			 is_br;
   logic 			 is_indirect;
   logic 			 take_br;
   logic [`M_WIDTH-1:0] 	 data;
   logic [`LG_PHT_SZ-1:0] 	 pht_idx;

`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
   logic [63:0] 	    alloc_cycle;
   logic [63:0] 	    complete_cycle;
   logic [31:0]		    raw_insn;
`endif
   
} rob_entry_t;

typedef struct packed {
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic 		       complete;
   logic 		       faulted;
   logic [`M_WIDTH-1:0]        restart_pc;
   logic 		       take_br;
   cause_t		       cause;
   logic		       has_cause;
   logic [`M_WIDTH-1:0]        data;
} complete_t;

typedef struct packed {
   logic [31:0] insn_bytes;
   logic	page_fault;
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
   logic	is_atomic;
   logic	is_ll;
   mem_op_t op;
   logic [4:0]	amo_op;
   logic 	spans_cacheline;
   logic 	unaligned;
   logic	has_cause;
   cause_t 	cause;
   logic	uncachable;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] dst_ptr;
   logic 		       dst_valid;
   logic [`M_WIDTH-1:0]        data;
   logic [`M_WIDTH-1:0]        pc;
`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
`endif

`ifdef VERILATOR
   logic [63:0]		    vaddr;
`endif
   logic [3:0] 		    restart_id;   
} mem_req_t;

typedef struct packed {
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] src_ptr;
`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
`endif   
} dq_t;

typedef struct packed {
   logic [`M_WIDTH-1:0] data;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
`ifdef ENABLE_CYCLE_ACCOUNTING
   logic [63:0] 	    fetch_cycle;
`endif      
} mem_data_t;

typedef struct packed {
   logic [`M_WIDTH-1:0]	data;
   logic [`M_WIDTH-1:0]	addr;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] dst_ptr;
   logic 		       dst_valid;
   cause_t		       cause;
   logic		       has_cause;
   logic		       mark_page_dirty;
} mem_rsp_t;

typedef struct packed {
   logic       valid;
   logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
} bob_entry_t;

typedef struct packed {
   logic       fault;
   logic       dirty;
   logic       readable;
   logic       writable;
   logic       executable;
   logic       user;
   logic [63:0] paddr;
   logic [1:0]	pgsize;
} page_walk_rsp_t;

typedef struct packed {
   logic [63:0]	itlb_hits;
   logic [63:0]	itlb_accesses;
   logic [63:0]	dtlb_hits;
   logic [63:0]	dtlb_accesses;
   logic [63:0]	l1d_hits;
   logic [63:0]	l1d_accesses;
   logic [63:0]	l1i_hits;
   logic [63:0]	l1i_accesses;   
   logic [63:0]	l2_hits;
   logic [63:0]	l2_accesses;   
} counters_t;



`endif
