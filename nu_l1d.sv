`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

`ifdef VERILATOR
import "DPI-C" function void pt_l1d_pass1_hit(input longint cycle,
					      input int	rob_id);

import "DPI-C" function void pt_l1d_replay(input longint cycle,
					   input int rob_id);

import "DPI-C" function void pt_l1d_blocked(input longint cycle,
					    input int rob_id);

import "DPI-C" function void pt_l1d_store_data_ready(input longint cycle,
						     input int rob_id);

import "DPI-C" function void pt_l1d_pass1_miss(input longint cycle,
					       input int	      rob_id,
					       input int	      maybe_alias,
					       input longint unsigned va,
					       input longint unsigned pa);


import "DPI-C" function void l1d_port_util(input int port1, input int port2);
import "DPI-C" function void record_l1d(input int req, 
					input int ack, 
					input int new_req,
					input int accept,
					input int block);


import "DPI-C" function void log_store_release(input int r,
					       input longint c);

import "DPI-C" function void log_l1d(input int gen_early_req,
				     input int push_miss,
				     input int push_miss_hit_inflight,
				     input int is_st_hit,
				     input int is_ld_hit,
				     input int is_hit_under_miss);
   
import "DPI-C" function void log_l1d_miss(input int dirty);

   
import "DPI-C" function void wr_log(input longint pc,
				    input int 		   rob_ptr,
				    input longint unsigned addr, 
				    input longint unsigned data, 
				    int 		   is_atomic);
`endif

//`define DEBUG
//`define VERBOSE_L1D 1

module nu_l1d(clk, 
	   reset,
	   l2_empty,
	   priv,
	   page_table_root,
	   l2_probe_addr,
	   l2_probe_val,
	   l2_probe_ack,	   
	   l1d_state,
	   restart_complete,
	   paging_active,
	   clear_tlb,
	   page_walk_req_valid,
	   page_walk_req_va,
	   page_walk_rsp_gnt,
	   page_walk_rsp_valid,	   
	   page_walk_rsp,
	   head_of_rob_ptr,
	   head_of_rob_ptr_valid,
	   retired_rob_ptr_valid,
	   retired_rob_ptr_two_valid,
	   retired_rob_ptr,
	   retired_rob_ptr_two,
	   memq_empty,
	   drain_ds_complete,
	   dead_rob_mask,
	   flush_req,
	   flush_complete,
	   flush_cl_req,
	   flush_cl_addr,
	   //inputs from core
	   core_mem_va_req_valid,
	   core_mem_va_req,
	   //store data
	   core_store_data_valid,
	   core_store_data,
	   core_store_data_ack,
	   //outputs to core
	   core_mem_va_req_ack,
	   core_mem_rsp,
	   core_mem_rsp_valid,
	   mem_rdy,
	   //output to the memory system
	   mem_req_valid,
	   mem_req,
	   //reply from memory system
	   l2_rsp_valid,
	   l2_rsp_load_data,
	   l2_rsp_tag,
	   l2_rsp_writeback,
	   l2_rsp_addr,
           l2_rsp_uc,	      
	   mtimecmp,
	   mtimecmp_val,
	   cache_accesses,
	   cache_hits,
	   tlb_accesses,
	   tlb_hits
	   );

   localparam L1D_NUM_SETS = 1 << `LG_L1D_NUM_SETS;
   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);   
   input logic clk;
   input logic reset;
   input logic l2_empty;
   input logic [1:0] priv;
   input logic [63:0] page_table_root;
   input logic l2_probe_val;
   input logic [(`PA_WIDTH-1):0] l2_probe_addr;
   output logic			 l2_probe_ack;
   
   output logic [3:0] l1d_state;
   input logic 	      restart_complete;
   input logic paging_active;
   input logic clear_tlb;
   output logic	page_walk_req_valid;
   output logic [63:0] page_walk_req_va;
   input logic 	       page_walk_rsp_gnt;
   input logic 	       page_walk_rsp_valid;
   input 	       page_walk_rsp_t page_walk_rsp;
      
   input logic [`LG_ROB_ENTRIES-1:0] head_of_rob_ptr;
   input logic 			     head_of_rob_ptr_valid;
   input logic retired_rob_ptr_valid;
   input logic retired_rob_ptr_two_valid;
   input logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr;
   input logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr_two;
   output logic			     memq_empty;
   input logic 			     drain_ds_complete;
   input logic [(1<<`LG_ROB_ENTRIES)-1:0] dead_rob_mask;
   
   
   input logic flush_cl_req;
   input logic [`M_WIDTH-1:0] flush_cl_addr;
   input logic 		      flush_req;
   output logic 	      flush_complete;

   input logic		      core_mem_va_req_valid;
   input		      mem_req_t core_mem_va_req;

   input logic core_store_data_valid;
   input       mem_data_t core_store_data;
   output logic core_store_data_ack;
   
   output logic core_mem_va_req_ack;
   output 	mem_rsp_t core_mem_rsp;
   output logic core_mem_rsp_valid;
   input logic mem_rdy;
   
   output logic mem_req_valid;
   output	l1d_req_t mem_req;
   

   input logic 				  l2_rsp_valid;
   input logic [L1D_CL_LEN_BITS-1:0] 	  l2_rsp_load_data;
   input logic [`LG_MRQ_ENTRIES:0] 	  l2_rsp_tag;
   input logic [`PA_WIDTH-1:0] 		  l2_rsp_addr;
   input logic				  l2_rsp_writeback;
   input logic				  l2_rsp_uc;
   

   output logic [63:0]			  mtimecmp;
   output logic				  mtimecmp_val;
      
   output logic [63:0] 			 cache_accesses;
   output logic [63:0] 			 cache_hits;

   output logic [63:0]			 tlb_accesses;
   output logic [63:0] 			 tlb_hits;
   
         
   localparam LG_WORDS_PER_CL = `LG_L1D_CL_LEN - 2;
   localparam LG_DWORDS_PER_CL = `LG_L1D_CL_LEN - 3;
   
   localparam WORDS_PER_CL = 1<<(LG_WORDS_PER_CL);
   localparam BYTES_PER_CL = 1 << `LG_L1D_CL_LEN;
   
   localparam N_TAG_BITS = 20;
   localparam IDX_START = `LG_L1D_CL_LEN;
   localparam IDX_STOP  = `LG_L1D_CL_LEN + `LG_L1D_NUM_SETS;
   localparam LG_ALIAS_BITS = (`LG_L1D_CL_LEN + `LG_L1D_NUM_SETS) - `LG_PG_SZ;
   
   localparam LG_MAX_SET = `LG_PG_SZ - `LG_L1D_CL_LEN;
   
   localparam WORD_START = 2;
   localparam WORD_STOP = WORD_START+LG_WORDS_PER_CL;
   localparam DWORD_START = 3;
   localparam DWORD_STOP = DWORD_START + LG_DWORDS_PER_CL;
  
   localparam N_MQ_ENTRIES = (1<<`LG_MRQ_ENTRIES);
   
   logic 				  r_got_req, r_last_wr, n_last_wr;
   logic 				  r_wr_array;
   logic 				  r_got_req2, r_last_wr2, n_last_wr2;
   logic 				  rr_got_req, rr_last_wr, rr_is_retry, rr_did_reload;
   logic 				  r_lock_cache, n_lock_cache;
   logic 				  n_l2_probe_ack, r_l2_probe_ack;
   assign l2_probe_ack = r_l2_probe_ack;
   
   logic [`LG_MRQ_ENTRIES:0] 		  r_n_inflight;   
   
   
   
   //1st read port
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_cache_idx, r_cache_idx, rr_cache_idx;
   logic [N_TAG_BITS-1:0] 		  t_cache_tag, r_cache_tag, r_tag_out;
   logic [N_TAG_BITS-1:0] 		  rr_cache_tag;
   logic 				  r_valid_out, r_dirty_out;
   logic [L1D_CL_LEN_BITS-1:0] 		  r_array_out, t_data, t_data2;
   
   //2nd read port
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_cache_idx2, r_cache_idx2, rr_cache_idx2;
   logic [N_TAG_BITS-1:0] 		  t_cache_tag2, r_cache_tag2, r_tag_out2;
   logic 				  r_valid_out2, r_dirty_out2;
   logic [L1D_CL_LEN_BITS-1:0] 		  r_array_out2;
   
   
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_miss_idx, r_miss_idx;
   logic [`M_WIDTH-1:0] 		  t_miss_addr, r_miss_addr;

   //write port   
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_array_wr_addr;
   logic [L1D_CL_LEN_BITS-1:0] 		  t_array_wr_data, r_array_wr_data;

   logic 				  t_array_wr_en;
		  
   logic 				  r_flush_req, n_flush_req;
   logic 				  r_flush_cl_req, n_flush_cl_req;
   logic 				  r_flush_complete, n_flush_complete;
   

   logic [127:0] 		  t_shift, t_shift_2;
   logic [127:0] 		  t_store_shift, t_store_mask;

   
   wire				  w_port2_hit_cache, w_port2_vapa_mismatch;
   wire				  w_port2_missed_no_alias;
   
   logic			  t_got_rd_retry;
      
   logic 				  t_mark_invalid;
   logic 				  t_wr_array;
   logic				  t_wr_store;
   logic 				  t_hit_cache;
   logic 				  t_rsp_dst_valid;
   logic [63:0] 			  t_rsp_data;
   
   logic 				  t_hit_cache2;
   logic 				  t_rsp_dst_valid2;
   logic [63:0] 			  t_rsp_data2;


   
   logic [127:0]			  t_array_data;
   
   logic [`M_WIDTH-1:0] 		  t_addr;
   logic 				  t_got_req, t_got_req2, t_replay_req2;
   logic 				  t_tlb_xlat,t_tlb_xlat_replay;
   logic 				  n_pending_tlb_miss, r_pending_tlb_miss;
   logic				  n_pending_tlb_zero_page, r_pending_tlb_zero_page;
   logic				  t_got_miss, t_dirty_miss;
   logic				  t_pop_eb, t_push_eb;
   
   logic 				  t_push_miss;
   
   logic 				  t_mh_block, t_cm_block, t_cm_block2,
					  t_cm_block_stall;

   logic 				  r_must_forward, r_must_forward2;
      
   logic 				  n_inhibit_write, r_inhibit_write;
   logic 				  t_got_non_mem, r_got_non_mem;

   logic                                  t_incr_busy,t_force_clear_busy;
   logic 				  n_stall_store, r_stall_store;
      
   logic 				  n_is_retry, r_is_retry;
   logic 				  r_q_priority, n_q_priority;
   
   logic 				  n_core_mem_rsp_valid, r_core_mem_rsp_valid;

   typedef struct packed {
      logic [(`PA_WIDTH-1):0] addr;
      logic [127:0] 	      data;
   } eb_t;

   localparam		      N_EB_ENTRIES = 1<<(`LG_EB_ENTRIES);
   
   eb_t [(N_EB_ENTRIES-1):0] r_sb;   
   logic [`LG_EB_ENTRIES:0] r_eb_head_ptr, n_eb_head_ptr;
   logic [`LG_EB_ENTRIES:0] r_eb_tail_ptr, n_eb_tail_ptr;
   logic [(N_EB_ENTRIES-1):0] r_eb_valid;
   
   
   mem_rsp_t n_core_mem_rsp, r_core_mem_rsp;
      
   mem_req_t n_req, r_req, t_req;
   mem_req_t n_req2, r_req2, t_req2_pa;

   mem_req_t r_mem_q[N_MQ_ENTRIES-1:0];
   logic [`LG_MRQ_ENTRIES:0] r_mq_head_ptr, n_mq_head_ptr;
   logic [`LG_MRQ_ENTRIES:0] r_mq_tail_ptr, n_mq_tail_ptr;
   logic [`LG_MRQ_ENTRIES:0] t_mq_tail_ptr_plus_one;

   typedef struct 	     packed {
      logic [(`PA_WIDTH-1):0] addr;
      logic [127:0] 	      data;
      logic [`LG_MRQ_ENTRIES:0]	tag;
      logic			writeback;
      logic			uncacheable;
   } queue_t;
   
   queue_t r_l2q[N_MQ_ENTRIES-1:0];
   logic [`LG_MRQ_ENTRIES:0] r_l2q_head_ptr, n_l2q_head_ptr;
   logic [`LG_MRQ_ENTRIES:0] r_l2q_tail_ptr, n_l2q_tail_ptr;

   wire 		     w_l2q_empty = (r_l2q_tail_ptr == r_l2q_head_ptr);
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_l2q_head_ptr <= 'd0;
	     r_l2q_tail_ptr <= 'd0;
	     r_eb_head_ptr <= 'd0;
	     r_eb_tail_ptr <= 'd0;
	  end
	else
	  begin
	     r_l2q_head_ptr <= n_l2q_head_ptr;	     
	     r_l2q_tail_ptr <= n_l2q_tail_ptr;
	     r_eb_head_ptr <= n_eb_head_ptr;
	     r_eb_tail_ptr <= n_eb_tail_ptr;	     
	  end
     end // always_ff@ (posedge clk)

   wire w_eb_empty = r_eb_head_ptr==r_eb_tail_ptr;
   wire w_eb_full =  (r_eb_head_ptr!=r_eb_tail_ptr) & 
	(r_eb_head_ptr[`LG_EB_ENTRIES-1:0] == r_eb_tail_ptr[`LG_EB_ENTRIES-1:0]);


   
   wire  mem_rsp_valid = r_got_req & r_req.is_store ? 1'b0 : !w_l2q_empty;



   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_eb_valid <= 'd0;
	  end
	else
	  begin
	     if(t_push_eb)
	       begin
		  r_eb_valid[r_eb_tail_ptr[`LG_EB_ENTRIES-1:0]] <= 1'b1;
	       end
	     if(t_pop_eb)
	       begin
		  r_eb_valid[r_eb_head_ptr[`LG_EB_ENTRIES-1:0]] <= 1'b0;
	       end
	  end // else: !if(reset)
     end // always_ff@ (posedge clk)
   

   always_ff@(posedge clk)
     begin
	if(t_push_eb)
	  begin
	     r_sb[r_eb_tail_ptr[`LG_EB_ENTRIES-1:0]].data <= t_data;
	     r_sb[r_eb_tail_ptr[`LG_EB_ENTRIES-1:0]].addr <= n_port1_req_addr;
	  end
     end

   wire [N_EB_ENTRIES-1:0] w_eb_port1_hits, w_eb_port2_hits;
   
   generate
      for(genvar i = 0; i < N_EB_ENTRIES; i=i+1)
	begin
	   assign w_eb_port1_hits[i] = r_eb_valid[i] ? (r_sb[i].addr[`PA_WIDTH-1:IDX_START] == t_mem_head.addr[`PA_WIDTH-1:IDX_START]) : 1'b0;
	   assign w_eb_port2_hits[i] = r_eb_valid[i] ? (r_sb[i].addr[IDX_STOP-1:IDX_START] == t_cache_idx2) : 1'b0;
	end
   endgenerate

   wire w_eb_port1_hit = |w_eb_port1_hits;
   wire	w_eb_port2_hit = |w_eb_port2_hits;

   
   always_comb
     begin
	n_l2q_head_ptr = r_l2q_head_ptr;
	n_l2q_tail_ptr = r_l2q_tail_ptr;
	if(l2_rsp_valid)
	  begin
	     n_l2q_tail_ptr = r_l2q_tail_ptr + 'd1;
	  end
	if(mem_rsp_valid)
	  begin
	     n_l2q_head_ptr = r_l2q_head_ptr + 'd1;
	  end
     end

   always_ff@(posedge clk)
     begin
	if(l2_rsp_valid)
	  begin
	     r_l2q[r_l2q_tail_ptr[`LG_MRQ_ENTRIES-1:0]].addr <= l2_rsp_addr;
	     r_l2q[r_l2q_tail_ptr[`LG_MRQ_ENTRIES-1:0]].data <= l2_rsp_load_data;
	     r_l2q[r_l2q_tail_ptr[`LG_MRQ_ENTRIES-1:0]].tag  <= l2_rsp_tag;
	     r_l2q[r_l2q_tail_ptr[`LG_MRQ_ENTRIES-1:0]].writeback <= l2_rsp_writeback;
	     r_l2q[r_l2q_tail_ptr[`LG_MRQ_ENTRIES-1:0]].uncacheable <= l2_rsp_uc;	     
	     //$display("got l2 reply, writeback bit %b", l2_rsp_writeback);
	  end
     end


   wire [127:0] 			  mem_rsp_load_data = r_l2q[r_l2q_head_ptr[`LG_MRQ_ENTRIES-1:0]].data;
   wire [`LG_MRQ_ENTRIES:0] 		  mem_rsp_tag =  r_l2q[r_l2q_head_ptr[`LG_MRQ_ENTRIES-1:0]].tag;
   wire [`PA_WIDTH-1:0] 		  mem_rsp_addr =  r_l2q[r_l2q_head_ptr[`LG_MRQ_ENTRIES-1:0]].addr;		     

   wire					  mem_rsp_reload = mem_rsp_valid & 
					  (r_l2q[r_l2q_head_ptr[`LG_MRQ_ENTRIES-1:0]].writeback == 1'b0);
   wire					  mem_rsp_cacheable = r_l2q[r_l2q_head_ptr[`LG_MRQ_ENTRIES-1:0]].uncacheable==1'b0;
   

   //always_ff@(negedge clk)
   //begin
   //if(mem_rsp_valid & !mem_rsp_reload)
   //	  begin
   //$stop();
   //end
   //end
      
   function logic [15:0] make_mask(mem_req_t r);
      logic [15:0]		  t_m, m;
      logic			  b,s,w,d;
      
      b = 	(r.op == MEM_SB || r.op == MEM_LB || r.op == MEM_LBU);
      s = 	(r.op == MEM_SH || r.op == MEM_LH || r.op == MEM_LHU);
      w = 	(r.op == MEM_SW || r.op == MEM_LW );
      d	= 	(r.op == MEM_SD || r.op == MEM_LD );         
      t_m = b ? 16'h0001 :
	    s ? 16'h0003 :
	    w ? 16'h000f :
	    d ? 16'h00ff :
	    16'hffff;
      m = t_m << r.addr[3:0];      
      return m;
   endfunction
   
   logic [15:0]			  t_mq_mask, t_req_mask;
   
   always_comb
     begin
	t_mq_mask = make_mask(r_req2);
	t_req_mask = make_mask(core_mem_va_req);
     end
   
   
   logic [N_MQ_ENTRIES-1:0] r_mq_addr_valid;
   logic [N_MQ_ENTRIES-1:0] r_mq_inflight;
   logic [27:0]		    r_last_early;
   logic		    r_last_early_valid;
   
   logic [IDX_STOP-IDX_START-1:0] r_mq_addr[N_MQ_ENTRIES-1:0];
   logic [31:0]			  r_mq_dbg_addr[N_MQ_ENTRIES-1:0];
   
   logic [15:0]			  r_mq_mask[N_MQ_ENTRIES-1:0];
   
   logic [`M_WIDTH-1:0] 	  r_mq_full_addr[N_MQ_ENTRIES-1:0];
   logic 			  r_mq_is_load[N_MQ_ENTRIES-1:0];
   logic 			  r_mq_is_unaligned[N_MQ_ENTRIES-1:0];

   mem_op_t r_mq_op[N_MQ_ENTRIES-1:0];
   
   logic [`M_WIDTH-3:0] 	  r_mq_word_addr[N_MQ_ENTRIES-1:0];
   wire [BYTES_PER_CL-1:0] 	  w_store_byte_en;  
   
   mem_req_t t_mem_tail, t_mem_head;
   logic 	mem_q_full, mem_q_empty, mem_q_almost_full;
   
   typedef enum logic [3:0] {INITIALIZE, //0
			     INIT_CACHE, //1
			     ACTIVE, //2
                             INJECT_RELOAD, //3
			     WAIT_INJECT_RELOAD, //4
                             CLEAR_DIRTY, //3			     			     
                             FLUSH_CACHE, //5
                             FLUSH_CACHE_WAIT, //6
			     FLUSH_CACHE_LAST_WAIT, //7
                             FLUSH_CL, //8
                             FLUSH_CL_WAIT, //9			     
                             HANDLE_RELOAD, //10
			     TLB_RELOAD, //11
			     TLB_TURNAROUND, //12
			     UC_LOAD
                             } state_t;

   
   state_t r_state, n_state;
   assign l1d_state = r_state;
   logic 	t_pop_mq;
   logic 	n_did_reload, r_did_reload;
   logic r_got_rd_retry;
   
   
   
   logic r_mem_req_valid, n_mem_req_valid;
   logic r_mem_req_uc, n_mem_req_uc;
   logic [(`PA_WIDTH-1):0] r_mem_req_addr, n_mem_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] r_mem_req_store_data, n_mem_req_store_data;
   logic [3:0] 		       r_mem_req_opcode, n_mem_req_opcode;
   logic [`LG_MRQ_ENTRIES:0]   r_mem_req_tag, n_mem_req_tag;


   logic 		       n_port1_req_valid;
   logic 		       n_port1_req_uc;
   logic [(`PA_WIDTH-1):0]     n_port1_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] n_port1_req_store_data;
   logic [3:0] 		       n_port1_req_opcode;
   logic [`LG_MRQ_ENTRIES:0]   n_port1_req_tag;

   logic 		       n_port2_req_valid;
   logic 		       n_port2_req_uc;
   logic [(`PA_WIDTH-1):0]     n_port2_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] n_port2_req_store_data;
   logic [3:0] 		       n_port2_req_opcode;
   logic [`LG_MRQ_ENTRIES:0]   n_port2_req_tag;


   logic [63:0] r_cache_accesses, r_cache_hits,
		n_cache_accesses, n_cache_hits;
   
   
   wire 		       w_tlb_hit, w_tlb_dirty, w_tlb_writable, w_tlb_readable, 
			       w_tlb_user, w_zero_page;


   
   
   wire [`PA_WIDTH-1:0] w_tlb_pa;
   logic [63:0] r_tlb_addr, n_tlb_addr;
   logic 	t_reload_tlb;
   logic	n_page_walk_req_valid, r_page_walk_req_valid;
   logic 	r_page_walk_gnt, n_page_walk_gnt;
   logic 	n_flush_was_active, r_flush_was_active;
   
   logic [63:0] 	       r_store_stalls, n_store_stalls;
   
   
   logic [63:0] 			 r_cycle;
   assign flush_complete = r_flush_complete;

   assign mem_req_valid = r_mem_req_valid;
   

   always_comb
     begin
	mem_req.addr = r_mem_req_addr[31:0];
	mem_req.data = r_mem_req_store_data;
	mem_req.opcode = r_mem_req_opcode;
	mem_req.tag = r_mem_req_tag;
	mem_req.uncacheable = r_mem_req_uc;
     end

   

`ifdef FOUR_CYCLE_L1D
   assign core_mem_rsp_valid = r_core_mem_rsp_valid;
   assign core_mem_rsp = r_core_mem_rsp;
`else
   assign core_mem_rsp_valid = n_core_mem_rsp_valid;
   assign core_mem_rsp = n_core_mem_rsp;
`endif
   assign cache_accesses = r_cache_accesses;
   assign cache_hits = r_cache_hits;
   
   
   assign page_walk_req_valid = r_page_walk_req_valid;
   assign page_walk_req_va = r_tlb_addr;
   
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end
   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mq_head_ptr <= 'd0;
	     r_mq_tail_ptr <= 'd0;
	  end
	else
	  begin
	     r_mq_head_ptr <= n_mq_head_ptr;
	     r_mq_tail_ptr <= n_mq_tail_ptr;
	  end
     end // always_ff@ (posedge clk)

   localparam N_ROB_ENTRIES = (1<<`LG_ROB_ENTRIES);
   logic [1:0] r_graduated [N_ROB_ENTRIES-1:0];
   logic [N_ROB_ENTRIES-1:0] r_rob_inflight;
   
   
   logic t_reset_graduated;

   always_ff@(posedge clk)
     begin
	if(r_state == INIT_CACHE)
	  begin
	     r_graduated[r_cache_idx[`LG_ROB_ENTRIES-1:0]] <= 2'b00;
	  end
	else
	  begin
	     if(retired_rob_ptr_valid && r_graduated[retired_rob_ptr] == 2'b01)
	       begin
		  r_graduated[retired_rob_ptr] <= 2'b10;
	       end
	     if(retired_rob_ptr_two_valid && r_graduated[retired_rob_ptr_two] == 2'b01) 
	       begin
		  r_graduated[retired_rob_ptr_two] <= 2'b10;
	       end
	     if(t_incr_busy)
	       begin
		  //$display("cycle %d : incr busy for ptr %d", r_cycle, r_req2.rob_ptr);
		  r_graduated[r_req2.rob_ptr] <= 2'b01;
	       end
	     if(t_reset_graduated)
               begin
		  r_graduated[r_req.rob_ptr] <= 2'b00;
	       end
	     if(t_force_clear_busy)
	       begin
		  r_graduated[t_mem_head.rob_ptr] <= 2'b00;
	       end
	  end
     end // always_ff@ (posedge clk)


   

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_n_inflight <= 'd0;
	  end
	else if(core_mem_va_req_valid && core_mem_va_req_ack && !core_mem_rsp_valid)
	  begin
	     r_n_inflight <= r_n_inflight + 'd1;
	  end
	else if(!(core_mem_va_req_valid && core_mem_va_req_ack) && core_mem_rsp_valid)
	  begin
	     r_n_inflight <= r_n_inflight - 'd1;
	  end
     end // always_ff@ (posedge clk)
   
   
   always_comb
     begin
	n_mq_head_ptr = r_mq_head_ptr;
	n_mq_tail_ptr = r_mq_tail_ptr;
	t_mq_tail_ptr_plus_one = r_mq_tail_ptr + 'd1;
	
	if(t_push_miss)
	  begin
	     n_mq_tail_ptr = r_mq_tail_ptr + 'd1;
	  end
	
	if(t_pop_mq)
	  begin
	     n_mq_head_ptr = r_mq_head_ptr + 'd1;
	  end
	
	t_mem_head = r_mem_q[r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0]];
	
	mem_q_empty = (r_mq_head_ptr == r_mq_tail_ptr);
	
	mem_q_full = (r_mq_head_ptr != r_mq_tail_ptr) &
		     (r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0] == r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]);
	
	mem_q_almost_full = (r_mq_head_ptr != t_mq_tail_ptr_plus_one) &
			    (r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0] == t_mq_tail_ptr_plus_one[`LG_MRQ_ENTRIES-1:0]);
	
	
     end // always_comb



   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_rob_inflight <= 'd0;
	  end
	else
	  begin
	     if(r_got_req2 && !drain_ds_complete && t_push_miss)
	       begin
		  //$display("rob entry %d enters at cycle %d", r_req2.rob_ptr, r_cycle);
		  
		  if(r_rob_inflight[r_req2.rob_ptr] == 1'b1)
		    $display("entry %d should not be inflight\n", r_req2.rob_ptr);
		  
		  r_rob_inflight[r_req2.rob_ptr] <= 1'b1;
	       end
	     if(r_got_req & ( (r_valid_out & (r_tag_out == r_cache_tag)) | r_req.uncacheable) )
	       begin
		  //$display("rob entry %d leaves at cycle %d", r_req.rob_ptr, r_cycle);
		  if(r_rob_inflight[r_req.rob_ptr] == 1'b0) 
		    $display("huh %d should be inflight....\n", r_req.rob_ptr);
		  
		  r_rob_inflight[r_req.rob_ptr] <= 1'b0;
	       end
	     if(t_force_clear_busy)
	       begin
		  r_rob_inflight[t_mem_head.rob_ptr] <= 1'b0;
	       end
	  end
     end


   /* logic to check if memory request port is free */
   wire w_cache_port1_hit = r_valid_out & (r_tag_out == r_cache_tag);
   wire	w_cache_port1_clean_miss = !r_valid_out;
   
   wire w_req_port_free = r_got_req ? w_cache_port1_hit : 1'b1;

   wire	w_port2_dirty_miss = r_valid_out2 & r_dirty_out2 && (r_tag_out2 != w_tlb_pa[31:`LG_PG_SZ]);
   assign w_port2_hit_cache = r_valid_out2 & (r_tag_out2 == w_tlb_pa[`PA_WIDTH-1:`LG_PG_SZ]);

   assign w_port2_vapa_mismatch = r_req2.addr[LG_ALIAS_BITS+`LG_PG_SZ-1:`LG_PG_SZ] != w_tlb_pa[LG_ALIAS_BITS+`LG_PG_SZ-1:`LG_PG_SZ];
   
   wire	w_port2_almost_match = w_port2_vapa_mismatch & (r_tag_out2[N_TAG_BITS-1:LG_ALIAS_BITS] == w_tlb_pa[`PA_WIDTH-1:IDX_STOP]);

   
   assign w_port2_missed_no_alias = w_port2_hit_cache ? 1'b0 : (!w_port2_vapa_mismatch);
   

   wire w_hit_pop = r_pop_busy_addr2 ? (r_cache_idx[`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0] == r_req2.addr[`LG_PG_SZ-1:IDX_START]) : 1'b0;

   logic [`LG_MRQ_ENTRIES-1:0] r_mrq_credits, n_mrq_credits;
   always_ff@(posedge clk)
     begin
	r_mrq_credits <= reset ? {`LG_MRQ_ENTRIES{1'b1}} : n_mrq_credits;
     end

   wire w_one_free_credit = (r_mrq_credits != 'd0);
   wire	w_two_free_credits = (r_mrq_credits > 'd1);
   wire	w_three_free_credits = (r_mrq_credits > 'd2);

   wire	w_queues_drained = (&r_mrq_credits) & w_eb_empty;

   wire [`PA_WIDTH-1:0] w_req2_pa = {w_tlb_pa[`PA_WIDTH-1:`LG_PG_SZ], r_req2.addr[`LG_PG_SZ-1:0]};

   wire			w_uncacheable = 1'b1;
   
   wire			w_could_early_req_any = t_push_miss & (w_uncacheable == 1'b0) & w_three_free_credits 
			& w_port2_missed_no_alias &
			(r_last_early_valid ? (r_last_early != w_req2_pa[31:4]) : 1'b1) &
			!(r_hit_busy_line2 | r_fwd_busy_addr2 | w_hit_pop ) &
			(r_req2.is_load | r_req.is_store) &
			w_tlb_hit & 
			(rr_last_wr ? (rr_cache_idx[`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0] !=  r_req2.addr[`LG_PG_SZ-1:IDX_START]) : 1'b1) &
			(r_last_wr ? (r_cache_idx[`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0] != r_req2.addr[`LG_PG_SZ-1:IDX_START]) : 1'b1) &
			(n_last_wr ? (t_cache_idx[`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0] != r_req2.addr[`LG_PG_SZ-1:IDX_START]) : 1'b1);
   
   
   //dsheffie - disable early requests
   wire	w_could_early_req = !w_port2_dirty_miss & w_could_early_req_any & 1'b1;

   wire w_gen_early_req = w_could_early_req & (r_got_req ? w_cache_port1_hit : 1'b1);
   
   wire	w_early_rsp = mem_rsp_valid ? (mem_rsp_tag != (1 << `LG_MRQ_ENTRIES)) : 1'b0;

   always_ff@(posedge clk)
     begin
	r_last_early_valid <= reset ? 1'b0 : w_gen_early_req;
	r_last_early <= w_req2_pa[31:4];
     end
   
`ifdef DEBUG
   logic r_port1_req_valid, r_port2_req_valid;
   always_ff@(posedge clk)
     begin
	r_port1_req_valid <= reset ? 1'b0 : n_port1_req_valid;
	r_port2_req_valid <= reset ? 1'b0 : n_port2_req_valid;	
     end
   
   always_ff@(negedge clk)
     begin
	if(mem_rsp_valid)
	  begin
	     $display("resp for tag %d, addr %x, data %x at cycle %d, t_array_wr_en = %b cycle %d",
		      mem_rsp_tag, mem_rsp_addr, mem_rsp_load_data, r_cycle, t_array_wr_en, r_cycle);
	  end
	if(mem_req_valid)
	  begin
	     // if(mem_req_opcode == MEM_SW && (mem_req_tag !=  {1'b1, {`LG_MRQ_ENTRIES{1'b1}}}) && (r_state == ACTIVE))
	     // begin
	     // 	$display("generating store but tag is %d, n_port2_req_valid = %b, n_port1_req_valid = %b, r_state = %d, n_state = %d, r_cycle = %d", 
	     // 		 n_mem_req_tag, n_port2_req_valid, n_port1_req_valid, r_state, n_state, r_cycle);
	     // 	$stop();
	     // end
		
	     $display("req for tag %d, line %x at cycle %d, opcode %d, r_last_wr = %b, rr_last_wr = %b, r_state = %d, req1 %b, req2 %b",
		      mem_req.tag, 
		      mem_req.addr[IDX_STOP-1:IDX_START], 
		      r_cycle, 
		      mem_req.opcode, 
		      r_last_wr, 
		      rr_last_wr,
		      r_state,
		      r_port1_req_valid,
		      r_port2_req_valid
		      );
	  end
     end
`endif
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mq_inflight <= 'd0;
	  end
	else
	  begin
	     if(w_gen_early_req)
	       begin
`ifdef DEBUG
		  $display("generating early memory request with tag %d for pc %x addr %x rob ptr %d at cycle %d, r_last_wr = %b, rr_last_wr = %b, line %x",
			   r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0], r_req2.pc, 
			   r_req2.addr[`PA_WIDTH-1:0],
			   r_req2.rob_ptr,
			   r_cycle, r_last_wr, rr_last_wr, r_req2.addr[IDX_STOP-1:IDX_START]);
		  
`endif
		  r_mq_inflight[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= 1'b1;
	       end
	     if(w_early_rsp & (mem_rsp_tag[`LG_MRQ_ENTRIES] == 1'b0))
	       begin
`ifdef DEBUG
		  $display("early mem req returns for tag %d, addr %x at cycle %d", mem_rsp_tag, mem_rsp_addr, r_cycle);
`endif
		  r_mq_inflight[mem_rsp_tag[`LG_MRQ_ENTRIES-1:0]] <= 1'b0;		  
	       end
	  end
     end // always_ff@ (posedge clk)

   always_comb
     begin
	n_port2_req_valid = w_gen_early_req;
	n_port2_req_uc = 1'b0;
	n_port2_req_addr = w_tlb_pa[`PA_WIDTH-1:0];
	n_port2_req_store_data = r_mem_req_store_data;
	n_port2_req_opcode = 4'd4;
	n_port2_req_tag = {1'b0, r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]};
     end
   
   always_ff@(posedge clk)
     begin
	if(t_push_miss)
	  begin
	     r_mem_q[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0] ] <= t_req2_pa;
	     r_mq_addr[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.addr[IDX_STOP-1:IDX_START];
	     r_mq_dbg_addr[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= w_tlb_pa[31:0];	     

	     r_mq_mask[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= t_mq_mask & {16{r_req2.is_store}};	     
	     r_mq_op[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.op;
	     r_mq_is_load[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.is_load;
	     r_mq_is_unaligned[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.unaligned;
	     
	     r_mq_full_addr[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.addr;
	     r_mq_word_addr[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.addr[`M_WIDTH-1:2];
	  end
     end

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mq_addr_valid <= 'd0;
	  end
	else 
	  begin
	     if(t_push_miss)
	       begin
		  //$display("entry %d will be valid at cycle %d", r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0], r_cycle+'d1);
		  r_mq_addr_valid[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= 1'b1;
	       end
	     if(t_pop_mq)
	       begin
		  //$display("entry %d will be clear at cycle %d", r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0], r_cycle+'d1);
		  r_mq_addr_valid[r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0]] <= 1'b0;		  
	       end
	  end
     end // always_ff@ (posedge clk)

   wire [N_MQ_ENTRIES-1:0] w_hit_busy_addrs;
   logic [N_MQ_ENTRIES-1:0] r_hit_busy_addrs;
   logic 		   r_hit_busy_addr;
   
   wire [N_MQ_ENTRIES-1:0] w_hit_busy_addrs2, w_hit_busy_line2;
   wire [N_MQ_ENTRIES-1:0] w_addr_intersect;
   
   logic [N_MQ_ENTRIES-1:0] r_hit_busy_addrs2;
   logic		    r_hit_busy_addr2, r_hit_busy_line2;

   wire [N_MQ_ENTRIES-1:0] w_unaligned_in_mq;
   logic 		   r_any_unaligned;
   logic		   r_fwd_busy_addr2, r_pop_busy_addr2;

   
   generate
      for(genvar i = 0; i < N_MQ_ENTRIES; i=i+1)
	begin
	   
	   assign w_hit_busy_addrs[i] = (t_pop_mq && r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0] == i) ? 1'b0 : 
					r_mq_addr_valid[i] ? r_mq_addr[i] == t_cache_idx : 1'b0;

	   assign w_addr_intersect[i] = (|(r_mq_mask[i] & t_req_mask));
	   
	   assign w_hit_busy_line2[i] = r_mq_addr_valid[i] ? (r_mq_addr[i][`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0] == t_cache_idx2[`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0]) : 1'b0;
	   assign w_hit_busy_addrs2[i] = w_hit_busy_line2[i] & w_addr_intersect[i];

	   assign w_unaligned_in_mq[i] = r_mq_addr_valid[i] ? r_mq_is_unaligned[i] : 1'b0;
	end
   endgenerate
   
   always_ff@(posedge clk)
     begin
	r_hit_busy_addr <= reset ? 1'b0 : |w_hit_busy_addrs;
	r_hit_busy_addr2 <= reset ? 1'b0 : |w_hit_busy_addrs2;
	r_hit_busy_line2 <= reset ? 1'b0 : |w_hit_busy_line2;
	r_fwd_busy_addr2 <=  reset ? 1'b0 : (t_push_miss & (t_cache_idx2[`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0] == r_cache_idx2[`LG_L1D_NUM_SETS-LG_ALIAS_BITS-1:0]));
	r_pop_busy_addr2 <= reset ? 1'b0 : t_pop_mq;
	r_hit_busy_addrs <= t_got_req ? w_hit_busy_addrs : {{N_MQ_ENTRIES{1'b1}}};
	r_hit_busy_addrs2 <= t_got_req2 ? w_hit_busy_addrs2 : {{N_MQ_ENTRIES{1'b1}}};

	r_any_unaligned <= reset ? 1'b0 : (|w_unaligned_in_mq) | core_mem_va_req.unaligned;
     end // always_ff@ (posedge clk)


   always_ff@(posedge clk)
     begin
	//r_array_wr_data <= t_array_wr_data;
	r_array_wr_data <= t_array_data;
     end

   always_ff@(posedge clk)
     begin
	r_cache_accesses <= reset ? 64'd0 : n_cache_accesses;
	r_cache_hits <= reset ? 64'd0 : n_cache_hits;
     end

   wire w_drained = (|r_rob_inflight) == 1'b0;
   
   wire [10:0] w_memq_empty = {
			       mem_q_empty, //0
			       w_drained,  //1
			       (&n_mrq_credits), //2 
			       !core_mem_va_req_valid, //3
			       w_eb_empty, //4
			       !t_got_req, //5
			       !t_got_req2, //6
			       !t_push_miss, //7
			       !n_mem_req_valid, //8 
			       !mem_rsp_valid, //9
			       (r_n_inflight == 'd0) //10
			       };
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_l2_probe_ack <= 1'b0;
	     r_page_walk_req_valid <= 1'b0;
	     r_page_walk_gnt <= 1'b0;
	     r_flush_was_active <= 1'b0;
	     r_pending_tlb_miss <= 1'b0;
	     r_pending_tlb_zero_page <= 1'b0;
	     r_tlb_addr <= 'd0;
	     r_did_reload <= 1'b0;
	     r_stall_store <= 1'b0;
	     r_is_retry <= 1'b0;
	     r_flush_complete <= 1'b0;
	     r_flush_req <= 1'b0;
	     r_flush_cl_req <= 1'b0;
	     r_cache_idx <= 'd0;
	     r_cache_tag <= 'd0;
	     r_cache_idx2 <= 'd0;
	     rr_cache_idx2 <= 'd0;
	     r_cache_tag2 <= 'd0;
	     rr_cache_idx <= 'd0;
	     rr_cache_tag <= 'd0;
	     r_miss_addr <= 'd0;
	     r_miss_idx <= 'd0;
	     r_got_req <= 1'b0;
	     r_got_req2 <= 1'b0;
	     
	     rr_got_req <= 1'b0;
	     r_lock_cache <= 1'b0;
	     rr_is_retry <= 1'b0;
	     rr_did_reload <= 1'b0;
	     rr_last_wr <= 1'b0;
	     r_last_wr <= 1'b0;
	     
	     r_wr_array <= 1'b0;
	     r_got_non_mem <= 1'b0;

	     r_last_wr2 <= 1'b0;
	     r_state <= INITIALIZE;
	     r_mem_req_valid <= 1'b0;
	     r_mem_req_uc <= 1'b0;
	     r_mem_req_addr <= 'd0;
	     r_mem_req_store_data <= 'd0;
	     r_mem_req_opcode <= 'd0;
	     r_mem_req_tag <= 'd0;
	     r_core_mem_rsp_valid <= 1'b0;
	     r_store_stalls <= 'd0;
	     r_inhibit_write <= 1'b0;
	     memq_empty <= 1'b1;
	     r_q_priority <= 1'b0;
	     r_must_forward <= 1'b0;
	     r_must_forward2 <= 1'b0;
	  end
	else
	  begin
	     r_l2_probe_ack <= n_l2_probe_ack;
	     r_page_walk_req_valid <= n_page_walk_req_valid;
	     r_page_walk_gnt <= n_page_walk_gnt;
	     r_flush_was_active <= n_flush_was_active;
	     r_pending_tlb_miss <= n_pending_tlb_miss;
	     r_pending_tlb_zero_page <= n_pending_tlb_zero_page;
	     r_tlb_addr <= n_tlb_addr;
	     r_did_reload <= n_did_reload;
	     r_stall_store <= n_stall_store;
	     r_is_retry <= n_is_retry;
	     r_flush_complete <= n_flush_complete;
	     r_flush_req <= n_flush_req;
	     r_flush_cl_req <= n_flush_cl_req;
	     r_cache_idx <= t_cache_idx;
	     r_cache_tag <= t_cache_tag;
	     
	     r_cache_idx2 <= t_cache_idx2;
	     rr_cache_idx2 <= r_cache_idx2;
	     r_cache_tag2 <= t_cache_tag2;
	     rr_cache_idx <= r_cache_idx;
	     rr_cache_tag <= r_cache_tag;
	     
	     r_miss_idx <= t_miss_idx;
	     r_miss_addr <= t_miss_addr;
	     r_got_req <= t_got_req;
	     r_got_req2 <= t_got_req2 | t_replay_req2;
	     
	     rr_got_req <= r_got_req;
	     r_lock_cache <= n_lock_cache;
	     rr_is_retry <= r_is_retry;
	     rr_did_reload <= r_did_reload;
	     
	     r_last_wr <= n_last_wr;	     
	     rr_last_wr <= r_last_wr;
	     r_wr_array <= t_wr_array;
	     r_got_non_mem <= t_got_non_mem;

	     r_last_wr2 <= n_last_wr2;
	     r_state <= n_state;
	     r_mem_req_valid <= n_mem_req_valid;
	     r_mem_req_uc <= n_mem_req_uc;
	     r_mem_req_addr <= n_mem_req_addr;
	     r_mem_req_store_data <= n_mem_req_store_data;
	     r_mem_req_opcode <= n_mem_req_opcode;
	     r_mem_req_tag <= n_mem_req_tag;
	     
	     r_core_mem_rsp_valid <= n_core_mem_rsp_valid;
	     r_store_stalls <= n_store_stalls;
	     r_inhibit_write <= n_inhibit_write;
	     memq_empty <= &w_memq_empty;
	     r_q_priority <= n_q_priority;
	     r_must_forward  <= t_mh_block & t_pop_mq;
	     r_must_forward2 <= t_cm_block & core_mem_va_req_ack;
	  end
     end // always_ff@ (posedge clk)
   
   always_ff@(negedge clk)
    begin
       if(!memq_empty)
	 begin
	    for(integer i = 0; i < 11; i=i+1)
	      begin
		 if(w_memq_empty[i]==1'b0)
		   begin
		      $display("%d prevents memq_empty, mem_rsp_valid = %b, n_mem_req_valid = %b, w_drained = %b, cycle %d ", 
			       i, mem_rsp_valid, n_mem_req_valid, w_drained, r_cycle);
		   end
	      end
	 end
    end

   // always_ff@(negedge clk)
   //   begin
   // 	if(memq_empty & !reset)
   // 	  begin
   // 	     if(l2_empty == 1'b0) 
   // 	       begin
   // 		  $display("memq_empty asserted but l2_empty aint empty at cycle %d", r_cycle);
   // 		  $stop();
   // 	       end
   // 	  end
   //   end
   
   
   always_ff@(posedge clk)
     begin
	r_req <= n_req;
	r_req2 <= n_req2;
	r_core_mem_rsp <= n_core_mem_rsp;
     end

   always_comb
     begin
	t_array_wr_addr = mem_rsp_reload ? mem_rsp_addr[IDX_STOP-1:IDX_START] : r_cache_idx;
	t_array_wr_data = mem_rsp_reload ? mem_rsp_load_data : t_store_shift;
	t_array_wr_en = (mem_rsp_reload & mem_rsp_cacheable) | t_wr_array;
     end

   always_ff@(negedge clk)
     begin
	if(mem_rsp_reload)
	  begin
	     //$display("got reload, addr %x, idx %d, data %x, state %d",
	     //mem_rsp_addr, mem_rsp_addr[IDX_STOP-1:IDX_START],
	     //mem_rsp_load_data,
	     //r_state);
	  end
	if(mem_rsp_reload & t_wr_array) $stop();
     end
   
 ram2r1w #(.WIDTH(N_TAG_BITS), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_tag
     (
      .clk(clk),
      .rd_addr0(t_cache_idx),
      .rd_addr1(t_cache_idx2),
      .wr_addr(mem_rsp_addr[IDX_STOP-1:IDX_START]),
      .wr_data(mem_rsp_addr[`PA_WIDTH-1:`LG_PG_SZ]),
      .wr_en(mem_rsp_reload & mem_rsp_cacheable),
      .rd_data0(r_tag_out),
      .rd_data1(r_tag_out2)
      );
     

   ram2r1w_l1d_data #(.LG_DEPTH(`LG_L1D_NUM_SETS)) dc_data
     (
      .clk(clk),
      .rd_addr0(t_cache_idx),
      .rd_addr1(t_cache_idx2),
      .wr_addr(t_array_wr_addr),
      .wr_data(t_array_wr_data),
      .wr_en(t_array_wr_en),
      .wr_byte_en(w_store_byte_en),
      .rd_data0(r_array_out),
      .rd_data1(r_array_out2)
      );

   logic t_dirty_value;
   logic t_write_dirty_en;
   logic [`LG_L1D_NUM_SETS-1:0] t_dirty_wr_addr;

  				     
   
   always_comb
     begin
	t_dirty_value = 1'b0;
	t_write_dirty_en = 1'b0;
	t_dirty_wr_addr = r_cache_idx;
	if(t_mark_invalid)
	  begin
	     t_write_dirty_en = 1'b1;	     
	  end
	else if(t_push_eb)
	  begin
	     t_dirty_wr_addr = n_port1_req_addr[IDX_STOP-1:IDX_START];	     
	     t_write_dirty_en = 1'b1;
	  end	
	else if(mem_rsp_reload & mem_rsp_cacheable)
	  begin
	     t_dirty_wr_addr = mem_rsp_addr[IDX_STOP-1:IDX_START];
	     t_write_dirty_en = 1'b1;
	  end
	else if(t_wr_array)
	  begin
	     t_dirty_value = 1'b1;
	     t_write_dirty_en = 1'b1;
	  end	
     end // always_comb

   

`ifdef DEBUG
    always_ff@(negedge clk)
      begin
	 // for(integer i = 0; i < N_MQ_ENTRIES; i=i+1)
	 //   begin
	 //      if(r_mq_addr_valid[i])
	 // 	begin
	 // 	   $display("line %d has addr %x", i, r_mq_addr[i]);
	 // 	end
	 //      if(r_mq_inflight[i])
	 // 	begin
	 // 	   $display("line %d is inflight", i);
	 // 	end
	 //   end

	 
	 
	 if(t_got_req2)
	   begin
	      $display("ingest t_cache_idx2 = %x at cycle %d", t_cache_idx2, r_cycle);
	   end
	 if(t_replay_req2)
	   begin
	      $display("replay t_cache_idx2 = %x at cycle %d", t_cache_idx2, r_cycle);
	   end	 
	 
	 if(t_mark_invalid & mem_rsp_valid | t_mark_invalid & t_wr_array | mem_rsp_valid & t_wr_array)
	   begin
	      $display("multiple actions to dirty in a cycle");
	      $stop();
	   end
	 
	if(t_wr_array)
	  begin
	     $display("store to line %x with value %x t_dirty_value %b t_write_dirty_en %b at cycle %d",
		      t_array_wr_addr, t_array_wr_data, t_dirty_value, t_write_dirty_en, r_cycle);
	  end

	 if(mem_rsp_valid)
	  begin
	     $display("reload to line %x with value %x t_dirty_value %b t_write_dirty_en %b at cycle %d",
		      t_array_wr_addr, t_array_wr_data, t_dirty_value, t_write_dirty_en, mem_rsp_valid, mem_rsp_tag, r_cycle);
	  end

	 
	if(mem_req_valid & (mem_req.opcode == 4'd4))
	  begin
	     $display("mem req opcode %d addr %x, tag %d, cycle %d", 
		      mem_req.opcode, mem_req.addr, mem_req.tag, r_cycle);
	  end


	if(mem_req_valid & (mem_req.opcode == 4'd7))
	  begin
	     $display("mem req opcode %d addr %x data %x, tag %d, cycle %d", 
		      mem_req.opcode, mem_req.addr, mem_req.data, mem_req.tag, r_cycle);
	  end

	 	 
	
	if(mem_rsp_reload && t_wr_array)
	  begin
	     $display("write port conflict, state %d, n_state %d, r_req.pc %x, resp tag %d",
		      r_state, n_state, r_req.pc, mem_rsp_tag);
	     $stop();
	  end
	// if(t_wr_array)
	//   begin
	//      $display("pc %x op %d rob ptr %d: t_array_data = %x, t_data = %x, t_store_shift = %x, mem_rsp_valid = %b, fwd = %b",
	// 	      r_req.pc,
	// 	      r_req.op,
	// 	      r_req.rob_ptr,
	// 	      t_array_data,
	// 	      t_data,
	// 	      t_store_shift,
	// 	      mem_rsp_valid, 
	// 	      (r_got_req && r_must_forward));
	//   end
     end
`endif
   
   
   ram2r1w #(.WIDTH(1), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_dirty
     (
      .clk(clk),
      .rd_addr0(t_cache_idx),
      .rd_addr1(t_cache_idx2),
      .wr_addr(t_dirty_wr_addr),
      .wr_data(t_dirty_value),
      .wr_en(t_write_dirty_en),
      .rd_data0(r_dirty_out),
      .rd_data1(r_dirty_out2)
      );

   
   logic t_valid_value;
   logic t_write_valid_en;
   logic [`LG_L1D_NUM_SETS-1:0] t_valid_wr_addr;

   always_comb
     begin
	t_valid_value = 1'b0;
	t_write_valid_en = 1'b0;
	t_valid_wr_addr = r_cache_idx;
	if(t_mark_invalid)
	  begin
	     t_write_valid_en = 1'b1;
	  end
	else if(t_push_eb)
	  begin
	     t_write_valid_en = 1'b1;
	     t_valid_wr_addr = n_port1_req_addr[IDX_STOP-1:IDX_START];
	     //$display("marking line %x as not valid at cycle %d", 
	     //t_valid_wr_addr, r_cycle);
	  end
	else if(mem_rsp_reload)
	  begin
	     t_valid_wr_addr = mem_rsp_addr[IDX_STOP-1:IDX_START];
	     t_valid_value = !r_inhibit_write;
	     t_write_valid_en = 1'b1;
	     // $display("mem_rsp_valid, addr %x, tag %d, line %x as %b at cycle %d",
	     // 	      mem_rsp_addr,
	     // 	      mem_rsp_tag,		      
	     // 	      t_valid_wr_addr,
	     // 	      t_valid_value,
	     // 	      r_cycle);	     
	  end
     end // always_comb
      
   ram2r1w #(.WIDTH(1), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_valid
     (
      .clk(clk),
      .rd_addr0(t_cache_idx),
      .rd_addr1(t_cache_idx2),
      .wr_addr(t_valid_wr_addr),
      .wr_data(t_valid_value),
      .wr_en(t_write_valid_en),
      .rd_data0(r_valid_out),
      .rd_data1(r_valid_out2)
      );




   tlb #(.LG_N(5)) dtlb(
    	    .clk(clk), 
    	    .reset(reset),
	    .priv(priv),
    	    .clear(clear_tlb),
    	    .active(paging_active),
    	    .req(t_tlb_xlat|t_tlb_xlat_replay),
	    .va(n_tlb_addr),
	    .pa(w_tlb_pa),
	    .hit(w_tlb_hit),
	    .dirty(w_tlb_dirty),
	    .readable(w_tlb_readable),
	    .writable(w_tlb_writable),
	    .user(w_tlb_user),
            .zero_page(w_zero_page),			
            .tlb_hits(tlb_hits),
            .tlb_accesses(tlb_accesses),			
            .replace_va(r_tlb_addr),
	    .replace(t_reload_tlb),
	    .page_walk_rsp(page_walk_rsp)
	    );


   logic t_wr_link_reg;
   logic r_paging_active;   
   logic [63:0]	n_link_reg, r_link_reg;
   logic	n_link_reg_val, r_link_reg_val;
   logic [1:0]	r_priv;
   
   always_ff@(posedge clk)
     begin
	r_paging_active <= reset ? 1'b0 : paging_active;
	r_priv <= reset ? 2'd0 : priv;
     end

   wire w_paging_toggle = r_paging_active ^ paging_active;
   wire	w_priv_toggle = (priv  != r_priv);
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_link_reg_val <= 1'b0;
	  end
	else
	  begin
	     r_link_reg_val <= n_link_reg_val;
	  end
     end
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_link_reg <= 64'd0;
	  end
	else if(w_paging_toggle | w_priv_toggle)
	  begin
	     //$display("reset link reg on priv or paging toggle");
	     r_link_reg <= 'd0;
	  end
	else if(t_wr_link_reg)
	  begin
	     //$display("writing link reg to %x at pc %x",
	     //n_link_reg, r_req.pc);
	     r_link_reg <= n_link_reg;
	  end
     end
   



   wire [`LG_L1D_CL_LEN+2:0] w_shift_amt2 = {r_req2.addr[`LG_L1D_CL_LEN-1:0], 3'd0};
   
   always_comb
     begin
	t_data2 = r_got_req2 && r_must_forward2 ? r_array_wr_data : r_array_out2;
	t_hit_cache2 = w_port2_hit_cache && r_got_req2;
	t_rsp_dst_valid2 = 1'b0;
	t_rsp_data2 = 'd0;

	t_shift_2 = t_data2 >> w_shift_amt2;

	
	case(r_req2.op)
	  MEM_LB:
	    begin
	       t_rsp_data2 = {{56{t_shift_2[7]}}, t_shift_2[7:0]};
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end
	  MEM_LBU:
	    begin
	       t_rsp_data2 = {56'd0, t_shift_2[7:0]};	       
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;	       
	    end
	  MEM_LH:
	    begin
	       t_rsp_data2 = {{48{t_shift_2[15]}}, t_shift_2[15:0]};	       
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end
	  MEM_LHU:
	    begin
	       t_rsp_data2 = {48'd0, t_shift_2[15:0]};
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;	       
	    end
	  MEM_LW:
	    begin
	       t_rsp_data2 = {{32{t_shift_2[31]}}, t_shift_2[31:0]};
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end
	  MEM_LWU:
	    begin
	       t_rsp_data2 = {32'd0, t_shift_2[31:0]};
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end	  
	  MEM_LD:
	    begin
	       t_rsp_data2 = t_shift_2[63:0];
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end	  
	   default:
	     begin
	  //      $stop();
	     end
	endcase
     end

   wire        w_store32 = (r_req.op == MEM_SW || r_req.op == MEM_AMOW || r_req.op == MEM_SCW);
   wire        w_store64 = (r_req.op == MEM_SD || r_req.op == MEM_AMOD || r_req.op == MEM_SCD);
   
   wire [63:0] w_store_mask = 
	       r_req.op == MEM_SB ? 64'hff :
	       r_req.op == MEM_SH ? 64'hffff :
	       w_store32 ? 64'hffffffff :
	       w_store64 ? 64'hffffffffffffffff :
	       'd0;   

   logic [31:0] t_amo32_data;
   logic [63:0]	t_amo64_data;

   logic [63:0]	r_mtimecmp;
   logic	r_mtimecmp_val;
   assign mtimecmp = r_mtimecmp;
   assign mtimecmp_val = r_mtimecmp_val;
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mtimecmp <= 64'd0;
	     r_mtimecmp_val <= 1'b0;
	  end
	else
	  begin
	     r_mtimecmp_val <= t_wr_store && r_req.addr == `MTIMECMP_ADDR;
	     r_mtimecmp <= r_req.data;
	  end
     end // always_ff@ (posedge clk)

   //always_ff@(negedge clk)
   //begin
   //if(t_wr_store && r_req.addr == `MTIMECMP_ADDR)
   //begin
   //$display("pc %x sets mtimecmp to %d at cycle %d", r_req.pc, r_req.data, r_cycle);
   //end
   //end
	     
   wire w_match_link = ({r_req.addr[63:4], 4'd0} == r_link_reg) & r_link_reg_val;

`ifdef VERILATOR
   wire	w_aborted_sc = t_hit_cache & ((r_req.op == MEM_SCD) | (r_req.op == MEM_SCW)) & (!w_match_link);
   wire	w_succces_sc = t_hit_cache & ((r_req.op == MEM_SCD) | (r_req.op == MEM_SCW)) & (w_match_link);   
   always_ff@(negedge clk)
     begin
	// if(n_link_reg_val & (r_link_reg_val==1'b0))	
	//   begin
	//      $display("set link reg at cycle %d", r_cycle);
	//   end
	// else if((n_link_reg_val==1'b0) & r_link_reg_val)
	//   begin
	//      $display("clear link reg at cycle %d, op %d", r_cycle,r_req.op);
	//   end
	
	// if(w_aborted_sc)
	//   begin
	//      $display("r_link_reg_val = %b, r_link_reg = %x, r_req.addr = %x, link addr = %x, pc = %x",
	// 	      r_link_reg_val, r_link_reg, {r_req.addr[63:4], 4'd0}, r_link_reg, r_req.pc);
	//      $stop();
	//   end
	
	//if(w_succces_sc)
	//begin
	// $display("r_link_reg_val = %b, r_link_reg = %x, r_req.addr = %x, link addr = %x",
	//	      r_link_reg_val, r_link_reg, {r_req.addr[63:4], 4'd0}, r_link_reg);
	//end
	
	
	l1d_port_util({31'd0,r_got_req}, {31'd0, r_got_req2});
	record_l1d({31'd0,core_mem_va_req_valid},
		   {31'd0,core_mem_va_req_ack},
		   {31'd0, t_new_req},
		   {31'd0, t_accept},
		   {25'd0, t_new_req_c}
		   );


	// $display("----->  r_got_req = %b, w_got_hit = %b, r_req.is_store = %b, r_req.is_atomic = %b, r_req.pc = %x, t_wr_store = %b, t_hit_cache = %b, retry = %b reload = %b, cause %b, op = %d", 
	// 	 r_got_req,w_got_hit,r_req.is_store,r_req.is_atomic,r_req.pc,t_wr_store,
	// 	 t_hit_cache,  
	// 	 r_is_retry,
	// 	 r_did_reload,
	// 	 r_req.has_cause,
	// 	 r_req.op
	// 	 );
	
	if(t_wr_store)
	  begin	     
	     wr_log(r_req.pc,
		    { {(32-`LG_ROB_ENTRIES){1'b0}}, r_req.rob_ptr},
		    r_req.addr, 
		    r_req.op == MEM_AMOD ? t_amo64_data : (r_req.op == MEM_AMOW ? {{32{t_amo32_data[31]}},t_amo32_data} : r_req.data), 
		    r_req.is_atomic ? 32'd1 : 32'd0);
`ifdef VERBOSE_L1D			    
		  if(r_req.is_atomic)
	            $display("firing atomic for pc %x addr %x with data %x t_shift %x, at cycle %d for rob ptr %d, r_cache_idx %d, match link %b", 
	     		     r_req.pc, r_req.addr, r_req.data, t_shift, r_cycle, r_req.rob_ptr, r_cache_idx, w_match_link);
`endif	     
	  end // if (t_wr_store)
	
     end // always_ff@ (negedge clk)
`endif



   always_comb
     begin
	t_data = /* mem_rsp_valid ? mem_rsp_load_data : */
		 (r_got_req && r_must_forward) ? r_array_wr_data : 
		 r_array_out;
	
	t_hit_cache =  r_got_req & w_cache_port1_hit & (r_state == ACTIVE);
	t_array_data = 'd0;
	t_wr_array = 1'b0;
	t_wr_store = 1'b0;
	
	t_rsp_dst_valid = 1'b0;
	t_rsp_data = 'd0;

	t_shift = t_data >> {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};
	t_store_shift = {64'd0, r_req.data} << {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};
	t_store_mask = {64'd0, w_store_mask} << {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};

	t_amo32_data = 32'hdeadbeef;
	t_amo64_data = 64'hd0debabefacebeef;

	t_wr_link_reg = 1'b0;
	n_link_reg = r_link_reg;
	n_link_reg_val = r_link_reg_val;
	
	case(r_req.amo_op)
	  5'd0: /* amoadd */
	    begin
	       t_amo32_data = t_shift[31:0] + r_req.data[31:0];
	       t_amo64_data = t_shift[63:0] + r_req.data[63:0];
	       //$display("amo add data %x", r_req.data);
	    end
	  5'd1: /* amoswap */
	    begin
	       t_amo32_data = r_req.data[31:0];
	       t_amo64_data = r_req.data[63:0];
	    end
	  5'd4:
	    begin
	       t_amo32_data = t_shift[31:0] ^ r_req.data[31:0];
	       t_amo64_data = t_shift[63:0] ^ r_req.data[63:0];
	    end
	  5'd8: /* amoor */
	    begin
	       t_amo32_data = t_shift[31:0] | r_req.data[31:0];
	       t_amo64_data = t_shift[63:0] | r_req.data[63:0];
	    end
	  5'd12: /* amoand */
	    begin
	       t_amo32_data = t_shift[31:0] & r_req.data[31:0];
	       t_amo64_data = t_shift[63:0] & r_req.data[63:0];
	    end
	  5'd28: /* amomax */
	    begin
	       t_amo32_data = t_shift[31:0] < r_req.data[31:0] ? r_req.data[31:0] : t_shift[31:0];
	       t_amo64_data = t_shift[63:0] < r_req.data[63:0] ? r_req.data[63:0] : t_shift[63:0];
	    end
	  
	  default:
	    begin
	    end
	endcase // case (r_req.amo_op)
	
	case(r_req.op)
	  MEM_LB:
	    begin
	       t_rsp_data = {{56{t_shift[7]}}, t_shift[7:0]};	       
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end
	  MEM_LBU:
	    begin
	       t_rsp_data = {56'd0, t_shift[7:0]};	       	       
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;	       
	    end
	  MEM_LH:
	    begin
	       t_rsp_data = {{48{t_shift[15]}}, t_shift[15:0]};	       	       
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end
	  MEM_LHU:
	    begin
	       t_rsp_data = {48'd0, t_shift[15:0]};	       
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;	       
	    end
	  MEM_LW:
	    begin
	       t_rsp_data = {{32{t_shift[31]}}, t_shift[31:0]};	       
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       t_wr_link_reg = r_req.is_ll;
	       n_link_reg = {r_req.addr[63:4], 4'd0};
	       n_link_reg_val = r_req.is_ll ? 1'b1 : r_link_reg_val;
	    end
	  MEM_LWU:
	    begin
	       t_rsp_data = {32'd0, t_shift[31:0]};	       
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end	  
	  MEM_LD:
	    begin
	       t_rsp_data = t_shift[63:0];	       
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       t_wr_link_reg = r_req.is_ll;
	       n_link_reg = {r_req.addr[63:4], 4'd0};
	       n_link_reg_val = r_req.is_ll ? 1'b1 : r_link_reg_val;	       
	    end	  
	  MEM_SB:
	    begin
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);	       
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload);
	    end
	  MEM_SH:
	    begin
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);	       
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload);
	    end
	  MEM_SW:
	    begin
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       //t_array_data = t_store_shift;
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload);
	    end
	  MEM_SD:
	    begin
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload);
	    end
	  MEM_SCD:
	    begin
	       t_rsp_data = {63'd0, ~w_match_link};
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = w_match_link && t_hit_cache && 
			    (r_is_retry || r_did_reload) & (!r_req.has_cause);
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       //n_link_reg_val = t_wr_store ? 1'b0 : r_link_reg_val;
	    end
	  MEM_SCW:
	    begin
	       t_rsp_data = {63'd0, ~w_match_link};	       
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = w_match_link & t_hit_cache & 
			    (r_is_retry | r_did_reload) & (!r_req.has_cause);
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       //n_link_reg_val = t_wr_store ? 1'b0 : r_link_reg_val;	       
	    end
	  MEM_AMOW:
	    begin
	       //return old data
	       t_rsp_data = {{32{t_shift[31]}}, t_shift[31:0]};
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       t_store_shift = {96'd0, t_amo32_data} << {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};	       
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = t_hit_cache & (r_is_retry | r_did_reload) & (!r_req.has_cause);
	    end // case: MEM_AMOW
	  MEM_AMOD:
	    begin
	       t_rsp_data = t_shift[63:0];
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       t_store_shift = {64'd0, t_amo64_data} << {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = t_hit_cache & (r_is_retry | r_did_reload) & (!r_req.has_cause);
	    end
	  
	  default:
	    begin
	    end
	endcase // case r_req.op
	t_wr_array = t_wr_store;
     end


   generate
       for(genvar i = 0; i < BYTES_PER_CL; i=i+1)
	 begin
	    assign w_store_byte_en[i] = mem_rsp_valid ? 1'b1 : (t_wr_array & t_store_mask[i*8]);
	 end
   endgenerate


   wire w_st_amo_grad = t_mem_head.is_store ? 
			r_graduated[t_mem_head.rob_ptr] == 2'b10 : 1'b1;	       

   wire w_tlb_st_exc = w_tlb_hit & paging_active & (r_req2.is_store | r_req2.is_atomic) & 
	!w_tlb_writable;

   wire	w_tlb_st_not_dirty = w_tlb_hit & paging_active & (r_req2.is_store | r_req2.is_atomic) & w_tlb_writable & !w_tlb_dirty;   

   wire w_flush_hit = (r_tag_out == l2_probe_addr[`PA_WIDTH-1:`LG_PG_SZ]) & r_valid_out;


   mem_rsp_t t_core_mem_rsp;
   logic t_core_mem_rsp_valid;
   wire	 w_got_reload_pf = page_walk_rsp_valid & page_walk_rsp.fault;
   wire  w_port2_rd_hit = w_port2_hit_cache && (!r_hit_busy_addr2) & (!r_pending_tlb_miss);

   always_comb
     begin
`ifdef COUNT_DCACHE_HITS
	n_cache_hits = r_cache_hits;
	n_cache_accesses = r_cache_accesses;
	if(r_got_req2)
	  begin
	     n_cache_accesses = r_cache_accesses + 64'd1;
	  end
	if(t_hit_cache2 & !r_pending_tlb_miss)
	  begin
	     n_cache_hits = r_cache_hits + 64'd1;
	  end
`else // !`ifdef COUNT_DCACHE_HITS
	n_cache_hits ='d0;
	n_cache_accesses = 'd0;
`endif
     end // always_comb
   
   always_comb
     begin
	t_core_mem_rsp.data = r_req.addr;
	t_core_mem_rsp.addr = r_req.addr;
	t_core_mem_rsp.rob_ptr = r_req.rob_ptr;
	t_core_mem_rsp.dst_ptr = r_req.dst_ptr;
	t_core_mem_rsp.dst_valid = 1'b0;
	t_core_mem_rsp.has_cause = 1'b0;
	t_core_mem_rsp.mark_page_dirty = 1'b0;
	t_core_mem_rsp.cause = MISALIGNED_FETCH;
	t_core_mem_rsp_valid = 1'b0;

	t_incr_busy = 1'b0;
	n_stall_store = 1'b0;
	
	t_push_miss = 1'b0;
	t_req2_pa = r_req2;
	n_pending_tlb_miss  = r_pending_tlb_miss;
	n_pending_tlb_zero_page = r_pending_tlb_zero_page;

	if(r_got_req2)
	  begin
	     t_core_mem_rsp.data = r_req2.addr;
	     t_core_mem_rsp.rob_ptr = r_req2.rob_ptr;
	     t_core_mem_rsp.dst_ptr = r_req2.dst_ptr;
	     t_req2_pa.addr = {32'd0, w_tlb_pa};
	     t_req2_pa.uncacheable = w_uncacheable;
	     if(r_pending_tlb_miss)
	       begin
		  n_pending_tlb_miss = 1'b0;
		  n_pending_tlb_zero_page = 1'b0;
	       end
	     
	     if(drain_ds_complete)
	       begin
		  t_core_mem_rsp.dst_valid = r_req2.dst_valid;
		  t_core_mem_rsp.has_cause = r_req2.has_cause;
		  t_core_mem_rsp.cause = r_req2.cause;
		  t_core_mem_rsp.addr = r_req2.addr;
		  t_core_mem_rsp_valid = 1'b1;
	       end
	     else if(r_req2.op == MEM_NOP)
	       begin
		  if(r_req2.spans_cacheline)
		    begin
		       t_core_mem_rsp.cause = MISALIGNED_FETCH;
		    end
		  else
		    begin
		       t_core_mem_rsp.cause = r_req2.cause;
		    end
		  t_core_mem_rsp.dst_valid = r_req2.dst_valid;
		  t_core_mem_rsp.has_cause = 1'b1;
		  t_core_mem_rsp.addr = r_req2.addr;
		  t_core_mem_rsp_valid = 1'b1;		  
	       end // if (r_req2.op == MEM_NOP)
	     else if(!w_tlb_hit & w_zero_page)
	       begin
		  t_core_mem_rsp.dst_valid = r_req2.dst_valid;
		  t_core_mem_rsp.has_cause = 1'b1;
		  t_core_mem_rsp.cause = LOAD_PAGE_FAULT;
		  t_core_mem_rsp.addr = r_req2.addr;
		  t_core_mem_rsp_valid = 1'b1;			 
	       end
	     else if(!w_tlb_hit)
	       begin
		  //$display("TLB miss! for address %x", r_req2.addr);
		  n_pending_tlb_miss = 1'b1;
		  n_pending_tlb_zero_page = w_zero_page;
		  if(r_pending_tlb_miss) $stop();
	       end
	     else if(w_tlb_st_exc)
	       begin
		  t_core_mem_rsp.dst_valid = r_req2.dst_valid;
		  t_core_mem_rsp.has_cause = 1'b1;
		  t_core_mem_rsp.cause = STORE_PAGE_FAULT;
		  t_core_mem_rsp.addr = r_req2.addr;
		  t_core_mem_rsp_valid = 1'b1;			 
	       end
	     else if(r_req2.is_atomic | r_req2.is_ll | w_uncacheable)
	       begin
		  t_push_miss = 1'b1;
	       end
	     else if(r_req2.is_store)
	       begin
		  //$display("store miss at cycle %d for rob pointer %d", r_cycle, r_req2.rob_ptr);
		  t_push_miss = 1'b1;
		  t_incr_busy = 1'b1;
		  n_stall_store = 1'b1;
		  //ack early
		  t_core_mem_rsp.dst_valid = 1'b0;
		  t_core_mem_rsp_valid = 1'b1;		 
		  t_core_mem_rsp.has_cause = r_req2.spans_cacheline;
		  t_core_mem_rsp.mark_page_dirty = w_tlb_st_not_dirty;
		  t_core_mem_rsp.addr = r_req2.addr;
	       end // if (r_req2.is_store)
	     else if(w_port2_rd_hit & !r_got_rd_retry)
	       begin
`ifdef DEBUG
		  $display("cycle %d load on port2 hit cache for pc %x, addr %x, got data %x, t_data2 = %x, t_shift_2 = %x, shift = %d", 
			   r_cycle, r_req2.pc, r_req2.addr, t_rsp_data2[`M_WIDTH-1:0], t_data2, t_shift_2, w_shift_amt2[6:3]);
		  
		  
`endif
		  t_core_mem_rsp.data = t_rsp_data2[`M_WIDTH-1:0];
                  t_core_mem_rsp.dst_valid = t_rsp_dst_valid2;
                  t_core_mem_rsp_valid = 1'b1;
		  t_core_mem_rsp.has_cause = r_req2.spans_cacheline;
`ifdef VERILATOR
		  pt_l1d_pass1_hit(r_cycle,
				   {{ (32-`LG_ROB_ENTRIES){1'b0}}, r_req2.rob_ptr});				   
`endif
	       end
	     else
	       begin
		  t_push_miss = 1'b1;
	       end
	  end // if (r_got_req2)
     end // always_comb

`ifdef VERILATOR
   always_ff@(negedge clk)
     begin
	if(core_store_data_valid)
	  begin
	     pt_l1d_store_data_ready(r_cycle,
				     {{ (32-`LG_ROB_ENTRIES){1'b0}}, 
				      core_store_data.rob_ptr});	
	  end
	
	if(!t_accept & core_mem_va_req_valid)
	  begin
	     pt_l1d_blocked(r_cycle,
			    {{ (32-`LG_ROB_ENTRIES){1'b0}}, 
			     core_mem_va_req.rob_ptr});	
	  end
	
	if(t_push_miss)
	  begin
	     pt_l1d_pass1_miss(r_cycle,
			      {{ (32-`LG_ROB_ENTRIES){1'b0}},			       
			       r_req2.rob_ptr},
			       {31'd0, w_port2_almost_match},
			       r_req2.addr,
			       {32'd0, w_tlb_pa}
			       );				   
	  end

	if(t_got_req & t_mem_head.is_load)
	  begin
	     pt_l1d_replay(r_cycle,
			   {{ (32-`LG_ROB_ENTRIES){1'b0}}, t_mem_head.rob_ptr});				   
	  end
	
	if(r_got_req2)
	  begin
 `ifdef DEBUG
	     
	     $display("triage new op for r_hit_busy_addr = %b, pc %x, addr %x at cycle %d dirty %b valid %b w_port2_rd_hit %b drain_ds %b, nop %b, has cause %b push miss %b, store %b load %b ll %b atomic %b, tlb store exec %b, pending tlb miss %b flush %b, tlb hit %b, spans cacheline %b, early push %b, could push %b, early dirty %b",
		      r_hit_busy_line2,
		      r_req2.pc,
		      r_req2.addr, 
		      r_cycle,
		      r_dirty_out2, 
		      r_valid_out2,
		      w_port2_rd_hit, drain_ds_complete , r_req2.op == MEM_NOP, 
		      r_req2.has_cause,
		      t_push_miss,
		      r_req2.is_store,
		      r_req2.is_load,
		      r_req2.is_ll,
		      r_req2.is_atomic,
		      w_tlb_st_exc,
		      r_pending_tlb_miss,
		      drain_ds_complete || r_req2.op == MEM_NOP,
		      w_tlb_hit,
		      r_req2.spans_cacheline,
		      w_gen_early_req,
		      w_could_early_req_any,
		      w_could_early_req);
 `endif
	     log_l1d(w_gen_early_req ? 32'd1 : 32'd0,
		     t_push_miss & r_req2.is_load ? 32'd1 : 32'0,
		     t_push_miss & r_req2.is_load & (r_hit_busy_line2 | r_fwd_busy_addr2 | r_pop_busy_addr2) ? 32'd1 : 32'd0,
		     r_req2.is_store & w_port2_rd_hit ? 32'd1 : 32'd0,
		     ((r_req2.is_store==1'b0) & w_port2_rd_hit) ? 32'd1 : 32'd0,
		     ((r_req2.is_store==1'b0) & w_port2_rd_hit & (r_state != ACTIVE)) ? 32'd1 : 32'd0);
	  end
     end
`endif
    
     
   wire w_got_hit_or_idle = r_got_req ? w_cache_port1_hit : 1'b1;
   wire w_got_hit = r_got_req ? w_cache_port1_hit : 1'b0;
   
   wire	w_got_clean_miss = r_got_req ? w_cache_port1_clean_miss : 1'b0;   
   
   wire	w_mh_block = r_got_req && r_last_wr && (r_cache_idx == t_mem_head.addr[IDX_STOP-1:IDX_START] );

   wire	w_got_rd_retry = (!w_mh_block & !mem_q_empty & w_got_hit & !r_lock_cache & !n_pending_tlb_miss) 
	& !(t_mem_head.is_store | t_mem_head.is_atomic);
   
   logic t_new_req;
   logic [6:0] t_new_req_c;
   
   logic t_accept;

  
   always_comb
     begin
	t_cm_block = r_got_req && r_last_wr && (r_cache_idx == core_mem_va_req.addr[IDX_STOP-1:IDX_START]);
	t_cm_block_stall = t_cm_block && !(r_did_reload||r_is_retry);

	t_new_req_c[0] = w_got_hit_or_idle;
	t_new_req_c[1] = !(mem_q_almost_full|mem_q_full);
	t_new_req_c[2] = /* !w_got_rd_retry; */ 1'b1;
	t_new_req_c[3] = !(r_last_wr2 & (r_cache_idx2 == core_mem_va_req.addr[IDX_STOP-1:IDX_START]) & !core_mem_va_req.is_store);
	t_new_req_c[4] = !(n_pending_tlb_miss | r_pending_tlb_miss);
	t_new_req_c[5] = !t_cm_block_stall;
	t_new_req_c[6] = !r_rob_inflight[core_mem_va_req.rob_ptr];
	
	t_new_req = core_mem_va_req_valid & (&t_new_req_c);
     end // always_comb

   always_ff@(negedge clk)
     begin
	if(core_mem_va_req_valid & !t_new_req)
	  begin
	     $display("can't accept new txn at cycle %d", r_cycle);
	  end
     end

       
   
   always_ff@(posedge clk)
     begin
	r_got_rd_retry <= reset ? 1'b0 : w_got_rd_retry & t_new_req;
     end

   wire [127:0] w_uc_shift =  mem_rsp_load_data >> {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};

   logic t_old_ack;
   always_comb
     begin
	t_old_ack = 1'b0;
	n_flush_was_active = r_flush_was_active;
	n_page_walk_gnt = r_page_walk_gnt | page_walk_rsp_gnt;
	n_l2_probe_ack = 1'b0;
	t_reload_tlb = 1'b0;
	n_page_walk_req_valid = 1'b0;

	t_got_rd_retry = 1'b0;
	
	n_state = r_state;
	t_miss_idx = r_miss_idx;
	t_miss_addr = r_miss_addr;
	t_cache_idx = 'd0;
	t_cache_tag = 'd0;
	

	
	t_got_req = 1'b0;
	t_replay_req2 = 1'b0;
	
	
	t_got_non_mem = 1'b0;
	n_last_wr = 1'b0;

	
	
	t_got_miss = 1'b0;
	t_dirty_miss = 1'b0;
	t_push_eb = 1'b0;
	
	n_req = r_req;

	t_tlb_xlat_replay = 1'b0;
	core_store_data_ack = 1'b0;
	
	n_port1_req_valid = 1'b0;
	n_port1_req_uc = 1'b0;
	n_port1_req_addr = r_mem_req_addr;
	n_port1_req_store_data = r_mem_req_store_data;
	n_port1_req_opcode = r_mem_req_opcode;
	n_port1_req_tag = r_mem_req_tag;
	
	t_pop_mq = 1'b0;
	
	n_core_mem_rsp_valid = t_core_mem_rsp_valid;
	n_core_mem_rsp.data = t_core_mem_rsp.data;
	n_core_mem_rsp.addr = t_core_mem_rsp.addr;
	n_core_mem_rsp.rob_ptr = t_core_mem_rsp.rob_ptr;
	n_core_mem_rsp.dst_ptr = t_core_mem_rsp.dst_ptr;
	n_core_mem_rsp.dst_valid = t_core_mem_rsp.dst_valid;
	n_core_mem_rsp.has_cause = t_core_mem_rsp.has_cause;
	n_core_mem_rsp.mark_page_dirty = t_core_mem_rsp.mark_page_dirty;
	n_core_mem_rsp.cause = t_core_mem_rsp.cause;
	
	n_store_stalls = r_store_stalls;

	n_flush_req = r_flush_req | flush_req;
	n_flush_cl_req = r_flush_cl_req |l2_probe_val;
	n_flush_complete = 1'b0;
	t_addr = 'd0;
	
	n_inhibit_write = r_inhibit_write;
	
	t_mark_invalid = 1'b0;
	n_is_retry = 1'b0;
	t_reset_graduated = 1'b0;
	t_force_clear_busy = 1'b0;
	
	n_q_priority = !r_q_priority;
	
	n_did_reload = 1'b0;
	n_lock_cache = r_lock_cache;
	
	t_mh_block = r_got_req && r_last_wr && 
		     (r_cache_idx == t_mem_head.addr[IDX_STOP-1:IDX_START] );
	
	
	case(r_state)
	  INITIALIZE:
	    begin
	       n_state = INIT_CACHE;
	       t_cache_idx = 'd0;	       
	    end
	  INIT_CACHE:
	    begin
	       t_cache_idx = r_cache_idx + 'd1;
	       t_mark_invalid = 1'b1;
	       if(r_cache_idx == (L1D_NUM_SETS-1))
		 begin
		    //$display("flush done at cycle %d", r_cycle);
		    n_state = ACTIVE;
		    n_flush_complete = 1'b1;
		 end
	       else
		 begin
		    t_cache_idx = r_cache_idx + 'd1;		    
		 end
	    end
	  ACTIVE:
	    begin
	       if(r_got_req)
		 begin
`ifdef DEBUG
		    $display("---> cycle %d port1 addr %x for pc %x, rob_ptr %d, hit %b, data %x, store %b, dirty %b, idx %d", 
			     r_cycle, r_req.addr, r_req.pc, 
			     r_req.rob_ptr, 
			     w_got_hit, 
			     t_rsp_data, 
			     r_req.is_store,
			     r_dirty_out,
			     r_cache_idx);
`endif
		    if(r_req.uncacheable)
		      begin
			 if(r_req.is_store)
			   begin
			      $stop();
			   end
			 n_port1_req_addr = {r_req.addr[`PA_WIDTH-1:`LG_L1D_CL_LEN], 4'd0};
			 $display("--> UC ADDR %x, r_req.is_load = %b", n_port1_req_addr, r_req.is_load);
			 n_port1_req_opcode = MEM_LW;
			 n_port1_req_tag = (1 << `LG_MRQ_ENTRIES);
			 n_port1_req_uc = 1'b1;
			 n_state = UC_LOAD;
			 n_port1_req_valid = 1'b1;
		      end
		    else if(w_got_hit)
		      begin /* valid cacheline - hit in cache */
			 t_reset_graduated = r_req.is_store;
`ifdef VERILATOR
			 if(r_req.is_store)
			   begin
			      log_store_release( { {(32-`LG_ROB_ENTRIES){1'b0}}, r_req.rob_ptr}, r_cycle);
			   end
`endif
`ifdef DEBUG
			 $display("p1 store %b hit cache for pc %x, addr %x, got data %x, cycle %d, data %x", 
				  r_req.is_store,
				  r_req.pc, r_req.addr, t_rsp_data[`M_WIDTH-1:0], r_cycle,
				  t_data);
`endif
			 
			 if(r_req.is_store==1'b0)
			   begin
			      n_core_mem_rsp.data = t_rsp_data[`M_WIDTH-1:0];
			      n_core_mem_rsp.dst_valid = t_rsp_dst_valid;
			      n_core_mem_rsp_valid = 1'b1;
			      if(t_core_mem_rsp_valid) $stop();
			      n_core_mem_rsp.has_cause = r_req.spans_cacheline;

			   end // else: !if(r_req.is_store)

		      end // if (w_got_hit)
		    else if(r_valid_out && r_dirty_out && (r_tag_out != r_cache_tag) )
		      begin
			 t_got_miss = 1'b1;
			 t_dirty_miss = 1'b1;			 
			 n_inhibit_write = 1'b1;
			 if(r_hit_busy_addr && r_is_retry || !r_hit_busy_addr)
			   begin
			      n_port1_req_addr = {r_tag_out,r_cache_idx[LG_MAX_SET-1:0],4'd0};
			      n_port1_req_opcode = MEM_SW;
			      n_port1_req_store_data = t_data;
			      //$display("time to writeback address %x with data %x", n_port1_req_addr, t_data);
			      n_inhibit_write = 1'b1;
			      t_miss_idx = r_cache_idx;
			      t_miss_addr = r_req.addr;
			      //$display("lock cache at cycle %d (case 1)", r_cycle);
			      n_lock_cache = 1'b1;
			      if((rr_cache_idx == r_cache_idx) && rr_last_wr)
				begin
				   //$display("inflight write to line, must wait");
				   t_cache_idx = r_cache_idx;
				   n_state = WAIT_INJECT_RELOAD;
				end
			      else
				begin
				   //$display("no wait");
				   t_push_eb = 1'b1;
				   n_state = CLEAR_DIRTY;				   
				end
			   end // if (!t_stall_for_busy)
		      end // if (r_valid_out && r_dirty_out && (r_tag_out != r_cache_tag) )
		  else
		    begin
		       t_got_miss = 1'b1;
		       n_inhibit_write = 1'b0;	
		       if(r_hit_busy_addr && r_is_retry || !r_hit_busy_addr || r_lock_cache)
			 begin
			    t_miss_idx = r_cache_idx;
			    t_miss_addr = r_req.addr;		       
			    t_cache_idx = r_cache_idx;
			    
			    if((rr_cache_idx == r_cache_idx) && rr_last_wr)
			      begin
				 n_port1_req_addr = {r_tag_out,r_cache_idx[LG_MAX_SET-1:0],4'd0};
				 //$display("lock cache at cycle %d (case 2)", r_cycle);				 
				 n_lock_cache = 1'b1;
				 n_port1_req_opcode = MEM_SW;
				 n_port1_req_tag = (1 << `LG_MRQ_ENTRIES);
				 t_dirty_miss = 1'b1;			 				 
				 n_state = WAIT_INJECT_RELOAD;
			      end                                                             
			    else
			      begin
				 //$display("unlock cache at cycle %d", r_cycle);				 
				 n_lock_cache = 1'b0;
				 n_port1_req_addr = {r_req.addr[`PA_WIDTH-1:`LG_L1D_CL_LEN], 4'd0};
				 n_port1_req_opcode = MEM_LW;
				 n_port1_req_tag = (1 << `LG_MRQ_ENTRIES);
				 n_state = INJECT_RELOAD;
				 n_port1_req_valid = 1'b1;				 
			      end // if (w_free_credit)
			 end // if (r_hit_busy_addr && r_is_retry || !r_hit_busy_addr || r_lock_cache)
		       else
			 begin
			    
			    $display("r_valid_out = %b r_dirty_out = %b r_tag_out = %x r_cache_tag = %x line %x cycle %d",
				     r_valid_out, r_dirty_out, r_tag_out, r_cache_tag, r_cache_idx, r_cycle);
			    $display("r_hit_busy_addr %b,  r_is_retry  %b r_hit_busy_addr %b r_lock_cache %b",
				     r_hit_busy_addr, r_is_retry, r_hit_busy_addr, r_lock_cache);
			    
			    $stop();
			 end
		    end // else: !if(r_valid_out && r_dirty_out && (r_tag_out != r_cache_tag) )
		    
		 end // if (r_got_req)
	       else if(n_pending_tlb_miss)
		 begin
		    n_state = TLB_RELOAD;
		    n_page_walk_gnt = 1'b0;
		    n_page_walk_req_valid = 1'b1;
		 end
	       
	       /* not qualified on r_got_req */
	       if(!mem_q_empty & !t_got_miss & !r_lock_cache & !n_pending_tlb_miss &!w_eb_port1_hit & !w_eb_full & w_two_free_credits)
		 begin		    
		    if(!t_mh_block & (r_mq_inflight[r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0]] == 1'b0)  )
		      begin
			 if((t_mem_head.is_store | t_mem_head.is_atomic))
			   begin
			      if(w_st_amo_grad && (core_store_data_valid ? (t_mem_head.rob_ptr == core_store_data.rob_ptr) : 1'b0) )
				begin
				   t_pop_mq = 1'b1;
				   core_store_data_ack = 1'b1;
				   n_req = t_mem_head;
				   n_req.data = core_store_data.data;
				   t_cache_idx = t_mem_head.addr[IDX_STOP-1:IDX_START];
				   t_cache_tag = t_mem_head.addr[`PA_WIDTH-1:`LG_PG_SZ];
				   t_addr = t_mem_head.addr;
				   t_got_req = 1'b1;
				   n_is_retry = 1'b1;
				   n_last_wr = 1'b1;
				end // if (t_mem_head.rob_ptr == head_of_rob_ptr)
			      else if(drain_ds_complete && dead_rob_mask[t_mem_head.rob_ptr])
				begin
				   t_pop_mq = 1'b1;
				   t_force_clear_busy = 1'b1;
				end
			   end // if (t_mem_head.is_store)			 
			 else
			   begin
			      t_pop_mq = 1'b1;
			      n_req = t_mem_head;
			      t_cache_idx = t_mem_head.addr[IDX_STOP-1:IDX_START];
			      t_cache_tag = t_mem_head.addr[`PA_WIDTH-1:`LG_PG_SZ];
			      t_addr = t_mem_head.addr;
			      t_got_req = 1'b1;
			      n_is_retry = 1'b1;
			      t_got_rd_retry = 1'b1;
			   end // else: !if(t_mem_head.is_store || t_mem_head.is_atomic)
		      end
		 end // if (!mem_q_empty && !t_got_miss && !r_lock_cache && !n_pending_tlb_miss)
	       
	       
	       
	     if(t_new_req)
	       begin
		  t_old_ack = 1'b1;
	       end  
	     else if(r_flush_req && mem_q_empty && !(r_got_req && r_last_wr) && !w_eb_full)
	       begin
		  n_state = FLUSH_CACHE;
		  if(!mem_q_empty) $stop();
		  if(r_got_req && r_last_wr) $stop();
		  t_cache_idx = 'd0;
		  n_flush_req = 1'b0;
	       end
	     else if(r_flush_cl_req & mem_q_empty & w_queues_drained & !(r_got_req && r_last_wr)
		     & !(n_page_walk_req_valid | t_got_miss | r_wr_array | t_wr_array))
	       begin
		  if(!mem_q_empty) $stop();
		  if(r_got_req && r_last_wr) $stop();
		  t_cache_idx = l2_probe_addr[IDX_STOP-1:IDX_START];
		  n_flush_cl_req = 1'b0;
		  n_flush_was_active = 1'b1;
		  n_state = FLUSH_CL;
	       end
	    end // case: ACTIVE
	  WAIT_INJECT_RELOAD:
	    begin
	       t_push_eb = 1'b1;
	       n_state = CLEAR_DIRTY;
	       n_port1_req_store_data = t_data;
	    end
	  CLEAR_DIRTY:
	    begin
	       //$display("now in clear dirty state..., sb empty %b sb full %b", w_eb_empty, w_eb_full);
	       t_cache_idx = r_req.addr[IDX_STOP-1:IDX_START];
	       t_cache_tag = r_req.addr[`PA_WIDTH-1:`LG_PG_SZ];
	       n_last_wr = r_req.is_store;
	       t_got_req = 1'b1;
	       t_addr  = r_req.addr;
	       n_did_reload = 1'b1;
	       n_state = ACTIVE;
	       
	    end // case: CLEAR_DIRTY
	  UC_LOAD:
	    begin
	       if(mem_rsp_reload && (mem_rsp_tag == (1 << `LG_MRQ_ENTRIES)))
		 begin		    
		    case(r_req.op)
		      MEM_LB:
			begin
			   n_core_mem_rsp.data = {{56{w_uc_shift[7]}}, w_uc_shift[7:0]};
			end
		      MEM_LBU:
			begin
			   n_core_mem_rsp.data = {56'd0, w_uc_shift[7:0]};	       
			end
		      MEM_LH:
			begin
			   n_core_mem_rsp.data = {{48{w_uc_shift[15]}}, w_uc_shift[15:0]};	       
			end
		      MEM_LHU:
			begin
			   n_core_mem_rsp.data = {48'd0, w_uc_shift[15:0]};
			end
		      MEM_LW:
			begin
			   n_core_mem_rsp.data = {{32{w_uc_shift[31]}}, w_uc_shift[31:0]};
			end
		      MEM_LWU:
			begin
			   n_core_mem_rsp.data = {32'd0, w_uc_shift[31:0]};
			end	  
		      MEM_LD:
			begin
			   n_core_mem_rsp.data = w_uc_shift[63:0];
			end
		      default:
			begin
			end
		    endcase
		    n_core_mem_rsp.dst_valid = 1'b1;
		    n_core_mem_rsp_valid = 1'b1;
		    n_state = ACTIVE;
		    $display("mem_rsp_cacheable=%b, data %x", mem_rsp_cacheable, n_core_mem_rsp.data);
		    //$stop();
		 end
	    end
	  INJECT_RELOAD:
	    begin
	       if(mem_rsp_reload && (mem_rsp_tag == (1 << `LG_MRQ_ENTRIES)))
		 begin
		    n_state = HANDLE_RELOAD;
		    n_inhibit_write = 1'b0;
		    //$display("reload, got tag %b", mem_rsp_tag);
		 end
	    end
	  HANDLE_RELOAD:
	    begin
	       t_cache_idx = r_req.addr[IDX_STOP-1:IDX_START];
	       t_cache_tag = r_req.addr[`PA_WIDTH-1:`LG_PG_SZ];
	       n_last_wr = r_req.is_store;
	       t_got_req = 1'b1;
	       t_addr  = r_req.addr;
	       n_did_reload = 1'b1;
	       n_state = ACTIVE;
	    end
	  FLUSH_CL:
	    begin
	       if(w_flush_hit & r_link_reg_val & (r_link_reg[31:0] == {r_tag_out,r_cache_idx[LG_MAX_SET-1:0],4'd0}))
		 begin
		    $stop();
		 end
	       
	       if(r_dirty_out & w_flush_hit)
		 begin
		    n_port1_req_addr = {r_tag_out,r_cache_idx[LG_MAX_SET-1:0],4'd0};
		    n_port1_req_opcode = MEM_SW;
		    n_port1_req_store_data = t_data;
		    n_state = FLUSH_CL_WAIT;
		    n_inhibit_write = 1'b1;
		    n_port1_req_valid = 1'b1;	       
		 end
	       else
		 begin
		    n_state = r_flush_was_active ? ACTIVE : TLB_RELOAD;
		    n_flush_was_active = 1'b0;
		    t_mark_invalid = w_flush_hit;		 
		    n_l2_probe_ack = 1'b1;
		 end // else: !if(r_dirty_out)
	    end
	  FLUSH_CL_WAIT:
	    begin
	       	if(w_queues_drained)
		  begin
		     n_state = n_flush_was_active ? ACTIVE : TLB_RELOAD;
		     n_flush_was_active = 1'b0;
		     n_inhibit_write = 1'b0;
		     n_l2_probe_ack = 1'b1;
		  end	       
	    end
	  FLUSH_CACHE:
	    begin
	       t_cache_idx = r_cache_idx + 'd1;
	       //$display("flush idx %x was %b",
	       //{r_tag_out,r_cache_idx[LG_MAX_SET-1:0],4'd0},
	       //r_dirty_out);
	       
	       if(!r_dirty_out)
		 begin
		    t_mark_invalid = 1'b1;
		    t_cache_idx = r_cache_idx + 'd1;
		    if(r_cache_idx == (L1D_NUM_SETS-1))
		      begin
			 n_state = ACTIVE;
			 n_flush_complete = 1'b1;
		      end
		 end
	       else
		 begin
		    n_port1_req_addr = {r_tag_out,r_cache_idx[LG_MAX_SET-1:0],4'd0};
		    n_port1_req_opcode = MEM_SW;
		    n_port1_req_store_data = t_data;
		    n_port1_req_tag = {1'b1, {`LG_MRQ_ENTRIES{1'b1}}};		    
		    n_state = (r_cache_idx == (L1D_NUM_SETS-1)) ? FLUSH_CACHE_LAST_WAIT : FLUSH_CACHE_WAIT;
		    n_inhibit_write = 1'b1;
		    //n_port1_req_valid = 1'b1;
		    t_push_eb = 1'b1;
		    
		 end // else: !if(r_valid_out && !r_dirty_out)
	    end // case: FLUSH_CACHE
	  FLUSH_CACHE_LAST_WAIT:
	    begin
	       t_cache_idx = r_cache_idx;
	       //$display("stuck in flush cache at cycle %d", r_cycle);
	       	if(mem_rsp_valid)
		  begin
		     n_state = ACTIVE;
		     n_inhibit_write = 1'b0;
		     n_flush_complete = 1'b1;
		  end
	    end	  
	  FLUSH_CACHE_WAIT:
	    begin
	       t_cache_idx = r_cache_idx;
	       //$display("stuck in flush cache at cycle %d", r_cycle);
	       	if(mem_rsp_valid)
		  begin
		     n_state = FLUSH_CACHE;
		     n_inhibit_write = 1'b0;
		  end
	    end
	  
	  TLB_RELOAD:
	    begin
	       if(page_walk_rsp_valid)
		 begin
		    t_reload_tlb = page_walk_rsp.fault==1'b0;
		    n_state = TLB_TURNAROUND;
		 end // if (page_walk_rsp_valid)
	       else if(n_flush_cl_req & w_queues_drained)
		 begin
		    n_state = FLUSH_CL;
		    n_flush_cl_req = 1'b0;
		    t_cache_idx = l2_probe_addr[IDX_STOP-1:IDX_START];
		    n_flush_was_active = 1'b0;
		 end
	    end
	  TLB_TURNAROUND:
	    begin
	       n_page_walk_gnt = 1'b0;
	       n_state = ACTIVE;
	       t_replay_req2 = 1'b1;
	       t_tlb_xlat_replay = 1'b1;
	    end
	  default:
	    begin
	    end
	endcase // case r_state
     end // always_comb


   always_ff@(negedge clk)
     begin
	
	//if(t_push_eb)
	//begin
	//$display("--> pushing addr %x, data %x to sb entry %d at cycle %d", 
	//n_port1_req_addr, t_addr, r_eb_tail_ptr[`LG_EB_ENTRIES-1:0], r_cycle);
	// end
      if(t_push_miss && mem_q_full)
	begin
	   $display("attempting to push to a full memory queue");
	   $stop();
	end
	if(t_pop_mq && mem_q_empty)
	  begin
	   $display("attempting to pop an empty memory queue");
	   $stop();
	  end
     end // always_ff@ (negedge clk)


   wire w_reload_line = ((core_mem_va_req.addr[IDX_STOP-1:IDX_START] == r_miss_idx) & 
			 (r_state != ACTIVE)) | 
	((core_mem_va_req.addr[IDX_STOP-1:IDX_START] == t_miss_idx) & t_got_miss);
   
   
   always_comb
     begin
	t_cache_idx2 = 'd0;
	t_cache_tag2 = 'd0;
	n_req2 = r_req2;
	
	core_mem_va_req_ack = 1'b0;
	t_got_req2 = 1'b0;
	n_last_wr2 = 1'b0;
	t_tlb_xlat = 1'b0;
	n_tlb_addr = r_tlb_addr;
	
	t_accept = t_new_req & (t_got_req ? n_last_wr : 1'b1) & !w_reload_line;

	if(w_got_reload_pf)
	  begin
	     n_req2.op = MEM_NOP;
	     n_req2.is_store = 1'b0;
	     n_req2.has_cause = 1'b1;
	     n_req2.cause = (r_req2.is_store | r_req2.is_atomic) ? 
			    STORE_PAGE_FAULT : LOAD_PAGE_FAULT;
	  end
	else if(t_replay_req2)
	  begin
	     t_cache_idx2 = r_req2.addr[IDX_STOP-1:IDX_START];
	     t_cache_tag2 = r_req2.addr[`PA_WIDTH-1:`LG_PG_SZ];
	     t_got_req2 = 1'b1;
	     t_tlb_xlat = 1'b1;
	     n_tlb_addr = r_req2.addr;
	     n_last_wr2 = r_req2.is_store;
	  end
        else if(t_accept)
	  begin
	     //if(r_state != ACTIVE) $stop();
	     //use 2nd read port
	     t_cache_idx2 = core_mem_va_req.addr[IDX_STOP-1:IDX_START];
	     t_cache_tag2 = core_mem_va_req.addr[`PA_WIDTH-1:`LG_PG_SZ];
	     n_req2 = core_mem_va_req;
	     core_mem_va_req_ack = 1'b1;
	     t_got_req2 = 1'b1;
	     t_tlb_xlat = 1'b1;
	     n_tlb_addr = core_mem_va_req.addr;
	     n_last_wr2 = core_mem_va_req.is_store;
	    
`ifdef DEBUG
	     $display("ingest new op at cycle %d atomic %b, ll %b pc %x rob ptr %d addr %x r_state = %d, n_state = %d, idx2 %d, miss idx = %d old ack %b, mem_rsp_valid = %b",
	      	      r_cycle,
		      core_mem_va_req.is_atomic,
		      core_mem_va_req.is_ll,
	      	      core_mem_va_req.pc,
	      	      core_mem_va_req.rob_ptr,
	      	      {core_mem_va_req.addr[63:4],4'd0},
	      	      r_state, n_state, t_cache_idx2, r_miss_idx,
	      	      t_old_ack,
		      mem_rsp_valid);
		  for(integer i = 0; i < N_MQ_ENTRIES; i=i+1)
		    begin
		       if(r_mq_addr_valid[i] & r_mq_inflight[i])
			 begin
			    $display("\tline %x already in flight for entry %d", r_mq_addr[i], i);
			 end
		    end
	     
`endif //  `ifdef DEBUG
	     if(r_state != ACTIVE && (r_miss_idx ==t_cache_idx2))
	       begin
		  $stop();
	       end

	     
	  end // if (core_mem_va_req_valid &&...
     end // always_comb
   

   always_comb
     begin
	n_mem_req_valid = n_port1_req_valid;
	n_mem_req_uc = n_port1_req_uc;
	n_mem_req_addr = n_port1_req_addr;
	n_mem_req_store_data = n_port1_req_store_data;
	n_mem_req_opcode = n_port1_req_opcode;
	n_mem_req_tag = n_port1_req_tag;
	t_pop_eb = 1'b0;
	if(n_port2_req_valid)
	  begin
	     n_mem_req_valid = n_port2_req_valid;
	     n_mem_req_uc = n_port2_req_uc;
	     n_mem_req_addr = n_port2_req_addr;
	     n_mem_req_store_data = n_port2_req_store_data;
	     n_mem_req_opcode = n_port2_req_opcode;
	     if(n_mem_req_opcode == 4'd7)
	       begin
		  $stop();
	       end
	     n_mem_req_tag = n_port2_req_tag;
	  end
	else if(!(n_port1_req_valid|n_port2_req_valid) & !w_eb_empty & w_one_free_credit)
	  begin
	     t_pop_eb = 1'b1;
	     n_mem_req_valid = 1'b1;
	     n_mem_req_uc = 1'b0;
	     n_mem_req_addr = r_sb[r_eb_head_ptr[`LG_EB_ENTRIES-1:0]].addr;
	     n_mem_req_store_data = r_sb[r_eb_head_ptr[`LG_EB_ENTRIES-1:0]].data;
	     n_mem_req_opcode = MEM_SW;
	     n_mem_req_tag = {1'b1, {`LG_MRQ_ENTRIES{1'b1}}};
	     //$display("--> write buffer using memory buffer at cycle %d with tag %x", 
	     //r_cycle, n_mem_req_tag);	     
	  end
     end // always_comb

   wire w_decr_credit = n_mem_req_valid & !mem_rsp_valid;
   wire	w_incr_credit = !n_mem_req_valid & mem_rsp_valid;
   always_comb
     begin
	n_mrq_credits = r_mrq_credits;
	if(w_decr_credit)
	  begin
	     n_mrq_credits = r_mrq_credits - 'd1;
	     if(r_mrq_credits == 'd0) 
	       begin
		  $display("trying to push with no free credits,  mem_rdy %b, w_gen_early_req %b, r_state = %d", 
			   mem_rdy, w_gen_early_req, r_state);
		  $stop();
	       end
	  end
	else if(w_incr_credit)
	  begin
	     n_mrq_credits = r_mrq_credits + 'd1;
	  end
     end // always_comb


   logic [(1<<(`LG_MRQ_ENTRIES+1))-1:0] r_credits, n_credits;
   always_comb
     begin
	n_credits = r_credits;
	if(r_mem_req_valid)
	  begin
	     //if(n_credits[mem_req.tag]) 
	       //begin
	     //$display("credit %d was already inflight", mem_req.tag);
	     //$stop();
	  //end
	     n_credits[mem_req.tag] = 1'b1;
	  end
	if(mem_rsp_valid)
	  begin
	    // if(n_credits[mem_rsp_tag] == 1'b0)
	     //begin
	     //$display("credit %d, addr %x was not inflight", mem_rsp_tag, mem_rsp_addr);
	     //$stop();
	     //end
	     n_credits[mem_rsp_tag] = 1'b0;
	  end
     end // always_comb
   always_ff@(posedge clk)
     begin
	r_credits <= reset ? 'd0 : n_credits;
     end
   


   always_comb
     begin
	n_eb_head_ptr = r_eb_head_ptr;
	n_eb_tail_ptr = r_eb_tail_ptr;	
	if(t_push_eb)
	  begin
	     n_eb_tail_ptr = r_eb_tail_ptr + 'd1;	
	  end
	if(t_pop_eb)
	  begin
	     n_eb_head_ptr = r_eb_head_ptr + 'd1;	
	  end
     end


`ifdef VERILATOR
   always_ff@(negedge clk)
     begin
	//if(w_eb_full)
	//begin
	//$display("eviction buffer full at cycle %d", r_cycle);
	//end
	//if(!w_eb_empty)
	//begin
	//$display("eviction buffer not empty at cycle %d", r_cycle);
	//end
	
	if(t_got_miss)
	  begin
	     log_l1d_miss({31'd0, t_dirty_miss});
	  end
	if(n_port1_req_valid && n_port2_req_valid)
	  begin
	     $display("two requests - main state %d, next state %d", r_state, n_state);
	     $stop();
	  end
	if(r_state == ACTIVE & r_got_req & w_got_hit & t_got_miss)
	  begin
	     $stop();
	  end
     end
`endif
   
endmodule // l1d

