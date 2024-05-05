`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

`ifdef VERILATOR
import "DPI-C" function void wr_log(input longint pc, 
				    input longint addr, 
				    input longint data, 
				    int 	  is_atomic);
`endif

//`define VERBOSE_L1D 1

module l1d(clk, 
	   reset,
	   l2_probe_addr,
	   l2_probe_val,
	   l2_probe_ack,	   
	   l1d_state,
  	   n_inflight,
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
	   restart_valid,
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
	   //output to the memory system
	   mem_req_valid, 
	   mem_req_addr, 
	   mem_req_store_data, 
	   mem_req_opcode,
	   //reply from memory system
	   mem_rsp_valid,
	   mem_rsp_load_data,
	   cache_accesses,
	   cache_hits
	   );

   localparam L1D_NUM_SETS = 1 << `LG_L1D_NUM_SETS;
   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);   
   input logic clk;
   input logic reset;
   input logic l2_probe_val;
   input logic [(`M_WIDTH-1):0] l2_probe_addr;
   output logic 		l2_probe_ack;
   
   output logic [3:0] l1d_state;
   output logic [3:0] n_inflight;
   input logic 	      restart_complete;
   input logic paging_active;
   input logic clear_tlb;
   output logic	page_walk_req_valid;
   output logic [63:0] page_walk_req_va;
   input logic 	       page_walk_rsp_gnt;
   input logic 	       page_walk_rsp_valid;
   input 	       page_walk_rsp_t page_walk_rsp;
   
   //input logic	       page_walk_rsp_valid;
   //input logic [63:0]  page_walk_rsp_pa;
   //input logic	       page_walk_rsp_fault;
   //input logic	       page_walk_rsp_dirty;   
   //input logic	       page_walk_rsp_readable;
   //input logic	       page_walk_rsp_writable;
   
   input logic [`LG_ROB_ENTRIES-1:0] head_of_rob_ptr;
   input logic 			     head_of_rob_ptr_valid;
   input logic retired_rob_ptr_valid;
   input logic retired_rob_ptr_two_valid;
   input logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr;
   input logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr_two;
   input logic 			     restart_valid;
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

   output logic mem_req_valid;
   output logic [(`M_WIDTH-1):0] mem_req_addr;
   output logic [L1D_CL_LEN_BITS-1:0] mem_req_store_data;
   output logic [3:0] 			  mem_req_opcode;

   input logic 				  mem_rsp_valid;
   input logic [L1D_CL_LEN_BITS-1:0] 	  mem_rsp_load_data;

   
   
   output logic [63:0] 			 cache_accesses;
   output logic [63:0] 			 cache_hits;
   
         
   localparam LG_WORDS_PER_CL = `LG_L1D_CL_LEN - 2;
   localparam LG_DWORDS_PER_CL = `LG_L1D_CL_LEN - 3;
   
   localparam WORDS_PER_CL = 1<<(LG_WORDS_PER_CL);
   localparam BYTES_PER_CL = 1 << `LG_L1D_CL_LEN;
   
   localparam N_TAG_BITS = `M_WIDTH - `LG_L1D_NUM_SETS - `LG_L1D_CL_LEN;
   localparam IDX_START = `LG_L1D_CL_LEN;
   localparam IDX_STOP  = `LG_L1D_CL_LEN + `LG_L1D_NUM_SETS;
   localparam WORD_START = 2;
   localparam WORD_STOP = WORD_START+LG_WORDS_PER_CL;
   localparam DWORD_START = 3;
   localparam DWORD_STOP = DWORD_START + LG_DWORDS_PER_CL;
  
   localparam N_MQ_ENTRIES = (1<<`LG_MRQ_ENTRIES);
   
   logic 				  r_got_req, r_last_wr, n_last_wr;
   logic 				  r_wr_array;
   
   logic 				  r_last_rd, n_last_rd;
   logic 				  r_got_req2, r_last_wr2, n_last_wr2;
   logic 				  r_last_rd2, n_last_rd2;
   
   logic 				  rr_got_req, rr_last_wr, rr_is_retry, rr_did_reload;

   logic 				  r_lock_cache, n_lock_cache;

   logic 				  n_l2_probe_ack, r_l2_probe_ack;
   assign l2_probe_ack = r_l2_probe_ack;
   
   logic [`LG_MRQ_ENTRIES:0] 		  r_n_inflight;   
   assign n_inflight = r_n_inflight;
   
   
   
   //1st read port
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_cache_idx, r_cache_idx, rr_cache_idx;
   logic [N_TAG_BITS-1:0] 		  t_cache_tag, r_cache_tag, r_tag_out;
   logic [N_TAG_BITS-1:0] 		  rr_cache_tag;
   logic 				  r_valid_out, r_dirty_out;
   logic [L1D_CL_LEN_BITS-1:0] 		  r_array_out, t_data, t_data2;
   
   //2nd read port
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_cache_idx2, r_cache_idx2;
   logic [N_TAG_BITS-1:0] 		  t_cache_tag2, r_cache_tag2, r_tag_out2;
   logic 				  r_valid_out2, r_dirty_out2;
   logic [L1D_CL_LEN_BITS-1:0] 		  r_array_out2;
   
   
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_miss_idx, r_miss_idx;
   logic [`M_WIDTH-1:0] 		  t_miss_addr, r_miss_addr;

   //write port   
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_array_wr_addr;
   logic [L1D_CL_LEN_BITS-1:0] 		  t_array_wr_data, r_array_wr_data;

   logic 				  t_array_wr_en;
		  
   logic 				  t_ack_ld_early, r_ack_ld_early;
   logic 				  r_flush_req, n_flush_req;
   logic 				  r_flush_cl_req, n_flush_cl_req;
   logic 				  r_flush_complete, n_flush_complete;
   

   logic [31:0] 			  t_array_out_b32[WORDS_PER_CL-1:0];
   logic [L1D_CL_LEN_BITS-1:0] 		  t_shift, t_shift_2;
   logic [L1D_CL_LEN_BITS-1:0] 		  t_store_shift, t_store_mask;

   
   
   logic 				  t_got_rd_retry, t_port2_hit_cache;
      
   logic 				  t_mark_invalid;
   logic 				  t_wr_array;
   logic				  t_wr_store;
   logic 				  t_hit_cache;
   logic 				  t_rsp_dst_valid;
   logic [63:0] 			  t_rsp_data;
   
   logic 				  t_hit_cache2;
   logic 				  t_rsp_dst_valid2;
   logic [63:0] 			  t_rsp_data2;


   
   logic [L1D_CL_LEN_BITS-1:0] 		  t_array_data;
   
   logic [`M_WIDTH-1:0] 		  t_addr;
   logic 				  t_got_req, t_got_req2, t_replay_req2;
   logic 				  t_tlb_xlat;
   logic 				  n_pending_tlb_miss, r_pending_tlb_miss;
   
   logic 				  t_got_miss;
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
   mem_rsp_t n_core_mem_rsp, r_core_mem_rsp;
      
   mem_req_t n_req, r_req, t_req;
   mem_req_t n_req2, r_req2, t_req2_pa;

   mem_req_t r_mem_q[N_MQ_ENTRIES-1:0];
   logic [`LG_MRQ_ENTRIES:0] r_mq_head_ptr, n_mq_head_ptr;
   logic [`LG_MRQ_ENTRIES:0] r_mq_tail_ptr, n_mq_tail_ptr;
   logic [`LG_MRQ_ENTRIES:0] t_mq_tail_ptr_plus_one;

   
   logic [N_MQ_ENTRIES-1:0] r_mq_addr_valid;
   logic [IDX_STOP-IDX_START-1:0] r_mq_addr[N_MQ_ENTRIES-1:0];
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
                             FLUSH_CACHE, //5
                             FLUSH_CACHE_WAIT, //6
			     FLUSH_CACHE_LAST_WAIT, //7
                             FLUSH_CL, //8
                             FLUSH_CL_WAIT, //9			     
                             HANDLE_RELOAD, //10
			     TLB_RELOAD, //11
			     TLB_TURNAROUND //12
                             } state_t;

   
   state_t r_state, n_state;
   assign l1d_state = r_state;
   logic 	t_pop_mq;
   logic 	n_did_reload, r_did_reload;
   
   
   
   logic r_mem_req_valid, n_mem_req_valid;
   logic [(`M_WIDTH-1):0] r_mem_req_addr, n_mem_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] r_mem_req_store_data, n_mem_req_store_data;
   
   logic [3:0] 		       r_mem_req_opcode, n_mem_req_opcode;
   logic [63:0] 	       n_cache_accesses, r_cache_accesses;
   logic [63:0] 	       n_cache_hits, r_cache_hits;

   wire 		       w_tlb_hit, w_tlb_dirty, w_tlb_writable, w_tlb_readable, 
			       w_tlb_user;


   
   
   wire [63:0] w_tlb_pa;
   logic [63:0] r_tlb_addr, n_tlb_addr;
   logic 	t_reload_tlb;
   logic	n_page_walk_req_valid, r_page_walk_req_valid;
   logic 	r_page_walk_gnt, n_page_walk_gnt;
   logic 	n_flush_was_active, r_flush_was_active;
   
   logic [63:0] 	       r_store_stalls, n_store_stalls;
   
   
   logic [31:0] 			 r_cycle;
   assign flush_complete = r_flush_complete;
   assign mem_req_addr = r_mem_req_addr;
   assign mem_req_store_data = r_mem_req_store_data;
   assign mem_req_opcode = r_mem_req_opcode;
   assign mem_req_valid = r_mem_req_valid;

   assign core_mem_rsp_valid = n_core_mem_rsp_valid;
   assign core_mem_rsp = n_core_mem_rsp;
   
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
	if(reset /*|| restart_valid*/)
	  begin
	     for(integer i = 0; i < N_ROB_ENTRIES; i = i+1)
	       begin
		  r_graduated[i] <= 2'b00;
	       end
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
	
	mem_q_full = (r_mq_head_ptr != r_mq_tail_ptr) &&
		     (r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0] == r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]);
	
	mem_q_almost_full = (r_mq_head_ptr != t_mq_tail_ptr_plus_one) &&
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
	     if(r_got_req && r_valid_out && (r_tag_out == r_cache_tag) || t_ack_ld_early)
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
   
   
   // always_ff@(negedge clk)
   //   begin
   // 	if(t_push_miss && !t_port2_hit_cache)
   // 	  begin
   // 	     $display("cycle %d : pushing rob ptr %d, addr %x -> was store %b",
   // 		      r_cycle,
   // 		      r_req2.rob_ptr,
   // 		      r_req2.addr,
   // 		      r_req2.is_store);
   // 	  end
   //   end
   
   always_ff@(posedge clk)
     begin
	if(t_push_miss)
	  begin
	     r_mem_q[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0] ] <= t_req2_pa;
	     r_mq_addr[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.addr[IDX_STOP-1:IDX_START];
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
		  r_mq_addr_valid[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= 1'b1;
	       end
	     if(t_pop_mq)
	       begin
		  r_mq_addr_valid[r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0]] <= 1'b0;		  
	       end
	  end
     end // always_ff@ (posedge clk)

   wire [N_MQ_ENTRIES-1:0] w_hit_busy_addrs;
   logic [N_MQ_ENTRIES-1:0] r_hit_busy_addrs;
   logic 		   r_hit_busy_addr;
   
   wire [N_MQ_ENTRIES-1:0] w_hit_busy_addrs2;
   wire [N_MQ_ENTRIES-1:0] w_hit_busy_full_addrs2;
   logic [N_MQ_ENTRIES-1:0] r_hit_busy_full_addrs2;
   
   wire [N_MQ_ENTRIES-1:0] w_hit_busy_word_addrs2;
   logic [N_MQ_ENTRIES-1:0] r_hit_busy_word_addrs2;

   logic [N_MQ_ENTRIES-1:0] r_hit_busy_addrs2;
   logic 		   r_hit_busy_addr2, r_hit_busy_word_addr2;

   wire [N_MQ_ENTRIES-1:0] w_unaligned_in_mq;
   logic 		   r_any_unaligned;
   
   generate
      for(genvar i = 0; i < N_MQ_ENTRIES; i=i+1)
	begin
	   assign w_hit_busy_addrs[i] = (t_pop_mq && r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0] == i) ? 1'b0 :
					r_mq_addr_valid[i] ? r_mq_addr[i] == t_cache_idx : 
					1'b0;
	   
	   assign w_hit_busy_addrs2[i] = r_mq_addr_valid[i] ? (core_mem_va_req.is_load && r_mq_is_load[i]) ? 1'b0 : r_mq_addr[i] == t_cache_idx2 : 1'b0;

	   assign w_hit_busy_full_addrs2[i] = r_mq_addr_valid[i] ? (r_mq_full_addr[i] ==  core_mem_va_req.addr) : 1'b0;
	   
	   assign w_hit_busy_word_addrs2[i] = r_mq_addr_valid[i] ? (r_mq_word_addr[i] ==  core_mem_va_req.addr[`M_WIDTH-1:2]) : 1'b0;

	   assign w_unaligned_in_mq[i] = r_mq_addr_valid[i] ? r_mq_is_unaligned[i] : 1'b0;
	end
   endgenerate
   

   always_ff@(posedge clk)
     begin
	r_hit_busy_addr <= reset ? 1'b0 : |w_hit_busy_addrs;
	r_hit_busy_addrs <= t_got_req ? w_hit_busy_addrs : {{N_MQ_ENTRIES{1'b1}}};
	
	r_hit_busy_addr2 <= reset ? 1'b0 : |w_hit_busy_addrs2;
	r_hit_busy_addrs2 <= t_got_req2 ? w_hit_busy_addrs2 : {{N_MQ_ENTRIES{1'b1}}};

	r_hit_busy_word_addr2 <= reset ? 1'b0 : |w_hit_busy_word_addrs2;
	
	r_hit_busy_full_addrs2 <= t_got_req2 ? w_hit_busy_full_addrs2 : {{N_MQ_ENTRIES{1'b1}}};
	r_hit_busy_word_addrs2 <= t_got_req2 ? w_hit_busy_word_addrs2 : {{N_MQ_ENTRIES{1'b1}}};

	r_any_unaligned <= reset ? 1'b0 : (|w_unaligned_in_mq) | core_mem_va_req.unaligned;
     end // always_ff@ (posedge clk)





   always_ff@(posedge clk)
     begin
	//r_array_wr_data <= t_array_wr_data;
	r_array_wr_data <= t_array_data;
     end
  
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_l2_probe_ack <= 1'b0;
	     r_page_walk_req_valid <= 1'b0;
	     r_page_walk_gnt <= 1'b0;
	     r_flush_was_active <= 1'b0;
	     r_pending_tlb_miss <= 1'b0;
	     r_tlb_addr <= 'd0;
	     r_ack_ld_early <= 1'b0;
	     r_did_reload <= 1'b0;
	     r_stall_store <= 1'b0;
	     r_is_retry <= 1'b0;
	     r_flush_complete <= 1'b0;
	     r_flush_req <= 1'b0;
	     r_flush_cl_req <= 1'b0;
	     r_cache_idx <= 'd0;
	     r_cache_tag <= 'd0;
	     r_cache_idx2 <= 'd0;
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
	     r_wr_array <= 1'b0;
	     r_got_non_mem <= 1'b0;
	     r_last_wr <= 1'b0;
	     r_last_rd <= 1'b0;
	     r_last_wr2 <= 1'b0;
	     r_last_rd2 <= 1'b0;	     
	     r_state <= INITIALIZE;
	     r_mem_req_valid <= 1'b0;
	     r_mem_req_addr <= 'd0;
	     r_mem_req_store_data <= 'd0;
	     r_mem_req_opcode <= 'd0;
	     r_core_mem_rsp_valid <= 1'b0;
	     r_cache_hits <= 'd0;
	     r_cache_accesses <= 'd0;
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
	     r_tlb_addr <= n_tlb_addr;
	     r_ack_ld_early <= t_ack_ld_early;
	     r_did_reload <= n_did_reload;
	     r_stall_store <= n_stall_store;
	     r_is_retry <= n_is_retry;
	     r_flush_complete <= n_flush_complete;
	     r_flush_req <= n_flush_req;
	     r_flush_cl_req <= n_flush_cl_req;
	     r_cache_idx <= t_cache_idx;
	     r_cache_tag <= t_cache_tag;
	     
	     r_cache_idx2 <= t_cache_idx2;
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
	     
	     rr_last_wr <= r_last_wr;
	     r_wr_array <= t_wr_array;
	     r_got_non_mem <= t_got_non_mem;
	     r_last_wr <= n_last_wr;
	     r_last_rd <= n_last_rd;
	     r_last_wr2 <= n_last_wr2;
	     r_last_rd2 <= n_last_rd2;	     
	     r_state <= n_state;
	     r_mem_req_valid <= n_mem_req_valid;
	     r_mem_req_addr <= n_mem_req_addr;
	     r_mem_req_store_data <= n_mem_req_store_data;
	     r_mem_req_opcode <= n_mem_req_opcode;
	     r_core_mem_rsp_valid <= n_core_mem_rsp_valid;
	     r_cache_hits <= n_cache_hits;
	     r_cache_accesses <= n_cache_accesses;
	     r_store_stalls <= n_store_stalls;
	     r_inhibit_write <= n_inhibit_write;
	     memq_empty <= mem_q_empty 
			   && drain_ds_complete 
			   && !core_mem_va_req_valid 
			   && !t_got_req && !t_got_req2 
			   && !t_push_miss
			   && (r_n_inflight == 'd0);
	     
	     r_q_priority <= n_q_priority;
	     r_must_forward  <= t_mh_block & t_pop_mq;
	     r_must_forward2 <= t_cm_block & core_mem_va_req_ack;
	  end
     end // always_ff@ (posedge clk)

`ifdef VERBOSE_L1D
   always_ff@(negedge clk)
     begin
	if(memq_empty)
	  begin
	     $display("MEMQ EMTPY AT CYCLE %d", r_cycle);
	  end
     end
`endif
   
   always_ff@(posedge clk)
     begin
	r_req <= n_req;
	r_req2 <= n_req2;
	r_core_mem_rsp <= n_core_mem_rsp;
     end

   always_comb
     begin
	t_array_wr_addr = mem_rsp_valid ? r_mem_req_addr[IDX_STOP-1:IDX_START] : r_cache_idx;
	t_array_wr_data = mem_rsp_valid ? mem_rsp_load_data : t_store_shift;
	t_array_wr_en = mem_rsp_valid || t_wr_array;
     end

`ifdef VERBOSE_L1D
   always_ff@(negedge clk)
     begin
   	if(t_wr_array)
   	  begin
   	     $display("cycle %d : WRITING set %d WITH data %x, addr %x, op %d ptr %d, retry %b, dirty addr %x, dirty value %b, dirty en %b", 
   		      r_cycle, 
		      r_cache_idx, 
		      t_array_data,
		      r_req.addr, 
		      r_req.op,
		      r_req.rob_ptr, 
		      r_is_retry,
		      t_dirty_wr_addr,
		      t_dirty_value,
		      t_write_dirty_en);
   	  end	
     end // always_ff@ (negedge clk)

   always_ff@(negedge clk)
     begin
	if(r_state == FLUSH_CL)
	  begin
	     $display("flush line %d at cycle %d for addr %x, was dirty %b, r_last_wr = %b, rr_last_wr = %b",
		      r_cache_idx, r_cycle, {r_tag_out,r_cache_idx,4'd0}, r_dirty_out, r_last_wr, rr_last_wr);
	  end
     end // always_ff@ (negedge clk)

`endif
   
 ram2r1w #(.WIDTH(N_TAG_BITS), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_tag
     (
      .clk(clk),
      .rd_addr0(t_cache_idx),
      .rd_addr1(t_cache_idx2),
      .wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
      .wr_data(r_mem_req_addr[`M_WIDTH-1:IDX_STOP]),
      .wr_en(mem_rsp_valid),
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
	else if(mem_rsp_valid)
	  begin
	     t_dirty_wr_addr = r_mem_req_addr[IDX_STOP-1:IDX_START];
	     t_write_dirty_en = 1'b1;
	  end
	else if(t_wr_array)
	  begin
	     t_dirty_value = 1'b1;
	     t_write_dirty_en = 1'b1;
	  end	
     end
   
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
	else if(mem_rsp_valid)
	  begin
	     t_valid_wr_addr = r_mem_req_addr[IDX_STOP-1:IDX_START];
	     t_valid_value = !r_inhibit_write;
	     t_write_valid_en = 1'b1;
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

   generate
      for(genvar i = 0; i < WORDS_PER_CL; i=i+1)
	begin
	   assign t_array_out_b32[i] = t_data[((i+1)*32)-1:i*32];
	end
   endgenerate



   tlb #(.LG_N(4)) dtlb(
    	    .clk(clk), 
    	    .reset(reset),
    	    .clear(clear_tlb),
    	    .active(paging_active),
    	    .req(t_tlb_xlat),
	    .va(n_tlb_addr),
	    .pa(w_tlb_pa),
	    .hit(w_tlb_hit),
	    .dirty(w_tlb_dirty),
	    .readable(w_tlb_readable),
	    .writable(w_tlb_writable),
	    .user(w_tlb_user),
            .replace_va(r_tlb_addr),
	    .replace(t_reload_tlb),
	    .page_walk_rsp(page_walk_rsp)
	    );

   
   
   always_comb
     begin
	t_data2 = r_got_req2 && r_must_forward2 ? r_array_wr_data : r_array_out2;
	t_hit_cache2 = r_valid_out2 && (r_tag_out2 == w_tlb_pa[`M_WIDTH-1:IDX_STOP]) && r_got_req2 && 
		      (r_state == ACTIVE);
	t_rsp_dst_valid2 = 1'b0;
	t_rsp_data2 = 'd0;

	t_shift_2 = t_data2 >> {r_req2.addr[`LG_L1D_CL_LEN-1:0], 3'd0};
	
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


`ifdef VERILATOR
   always_ff@(negedge clk)
     begin
	if(t_wr_store)
	  begin
	     wr_log(r_req.pc,
		    r_req.addr, 
		    r_req.op == MEM_AMOD ? t_amo64_data : (r_req.op == MEM_AMOW ? {{32{t_amo32_data[31]}},t_amo32_data} : r_req.data), 
		    r_req.is_atomic ? 32'd1 : 32'd0);
`ifdef VERBOSE_L1D			    
	     
	      if(r_req.is_atomic)
	        $display("firing atomic for pc %x addr %x with data %x t_shift %x, at cycle %d for rob ptr %d, r_cache_idx %d", 
	     		 r_req.pc, r_req.addr, r_req.data, t_shift, r_cycle, r_req.rob_ptr, r_cache_idx);
`endif	     
	  end
     end // always_ff@ (negedge clk)
`endif
   
   always_comb
     begin
	t_data = mem_rsp_valid ? mem_rsp_load_data : 
		 (r_got_req && r_must_forward) ? r_array_wr_data : 
		 r_array_out;
	
	t_hit_cache = r_valid_out && (r_tag_out == r_cache_tag) && r_got_req && 
		      (r_state == ACTIVE || r_state == INJECT_RELOAD);
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
	  5'd8: /* amoor */
	    begin
	       t_amo32_data = t_shift[31:0] | r_req.data[31:0];
	       t_amo64_data = t_shift[63:0] | r_req.data[63:0];
	    end
	  5'd12:
	    begin
	       t_amo32_data = t_shift[31:0] & r_req.data[31:0];
	       t_amo64_data = t_shift[63:0] & r_req.data[63:0];
	    end
	  5'd28:
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
	       t_rsp_data = 64'd0;
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload) & (!r_req.has_cause);
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end
	  MEM_SCW:
	    begin
	       t_rsp_data = 64'd0;
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload) & (!r_req.has_cause);
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end
	  MEM_AMOW:
	    begin
	       //return old data
	       t_rsp_data = {{32{t_shift[31]}}, t_shift[31:0]};
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       t_store_shift = {96'd0, t_amo32_data} << {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};	       
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload) & (!r_req.has_cause);
	    end // case: MEM_AMOW
	  MEM_AMOD:
	    begin
	       t_rsp_data = t_shift[63:0];
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       t_store_shift = {64'd0, t_amo64_data} << {r_req.addr[`LG_L1D_CL_LEN-1:0], 3'd0};
	       t_array_data = (t_store_shift & t_store_mask) | ((~t_store_mask) & t_data);
	       t_wr_store = t_hit_cache && (r_is_retry || r_did_reload) & (!r_req.has_cause);
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
   
`ifdef VERILATOR
   logic [3:0]	r_restart_counter;
   always_ff@(posedge clk)
     begin
	r_restart_counter <= reset ? 'd0 : 
			     (restart_complete ? r_restart_counter + 'd1 : r_restart_counter);
     end

   always_ff@(negedge clk)
     begin
	//$display("cycle %d, state %d", r_cycle, r_state);
	if(t_got_req2 && r_restart_counter != core_mem_req.restart_id)
	  begin
	     $display("cycle %d : current restart id is %d but ingesting %d", r_cycle, r_restart_counter, core_mem_req.restart_id);
	     $stop();
	  end
	
	//if((t_got_req2==1'b0) & core_mem_va_req_valid)
	//begin
	//$display("can't ingest new op at cycle %d, inflight %d, tlb miss %b", 
	//r_cycle,
	//r_rob_inflight[core_mem_va_req.rob_ptr], 
	//n_pending_tlb_miss);
	//end

     end
`endif //  `ifdef VERILATOR



   wire w_st_amo_grad = t_mem_head.is_store ? 
			r_graduated[t_mem_head.rob_ptr] == 2'b10 : 1'b1;	       

   wire w_tlb_st_exc = w_tlb_hit & paging_active & (r_req2.is_store | r_req2.is_atomic) & 
	!w_tlb_writable;

   wire w_flush_hit = (r_tag_out == l2_probe_addr[`M_WIDTH-1:IDX_STOP]) & r_valid_out;
   
   always_comb
     begin
	n_flush_was_active = r_flush_was_active;
	n_page_walk_gnt = r_page_walk_gnt | page_walk_rsp_gnt;
	n_l2_probe_ack = 1'b0;
	t_reload_tlb = 1'b0;
	n_page_walk_req_valid = 1'b0;
	n_tlb_addr = r_tlb_addr;
	t_ack_ld_early = 1'b0;
	t_got_rd_retry = 1'b0;
	t_port2_hit_cache = r_valid_out2 && (r_tag_out2 == w_tlb_pa[`M_WIDTH-1:IDX_STOP]);
	
	n_state = r_state;
	t_miss_idx = r_miss_idx;
	t_miss_addr = r_miss_addr;
	t_cache_idx = 'd0;
	t_cache_tag = 'd0;
	
	t_cache_idx2 = 'd0;
	t_cache_tag2 = 'd0;	
	
	t_got_req = 1'b0;
	t_got_req2 = 1'b0;
	t_replay_req2 = 1'b0;
	
	t_tlb_xlat = 1'b0;
	n_pending_tlb_miss  = r_pending_tlb_miss;
	
	t_got_non_mem = 1'b0;
	n_last_wr = 1'b0;
	n_last_rd = 1'b0;
	n_last_wr2 = 1'b0;
	n_last_rd2 = 1'b0;
	
	
	t_got_miss = 1'b0;
	t_push_miss = 1'b0;
	
	n_req = r_req;
	n_req2 = r_req2;
	t_req2_pa = r_req2;
	
	core_mem_va_req_ack = 1'b0;
	core_store_data_ack = 1'b0;
	
	n_mem_req_valid = 1'b0;
	n_mem_req_addr = r_mem_req_addr;
	n_mem_req_store_data = r_mem_req_store_data;
	n_mem_req_opcode = r_mem_req_opcode;
	t_pop_mq = 1'b0;
	n_core_mem_rsp_valid = 1'b0;
	
	n_core_mem_rsp.data = r_req.addr;
	n_core_mem_rsp.addr = r_req.addr;
	n_core_mem_rsp.rob_ptr = r_req.rob_ptr;
	n_core_mem_rsp.dst_ptr = r_req.dst_ptr;
	n_core_mem_rsp.dst_valid = 1'b0;
	n_core_mem_rsp.has_cause = 1'b0;
	n_core_mem_rsp.cause = MISALIGNED_FETCH;
	
	n_cache_accesses = r_cache_accesses;
	n_cache_hits = r_cache_hits;
	
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
	
	t_incr_busy = 1'b0;
	
	n_stall_store = 1'b0;
	n_q_priority = !r_q_priority;
	
	n_did_reload = 1'b0;
	n_lock_cache = r_lock_cache;
	
	t_mh_block = r_got_req && r_last_wr && 
		     (r_cache_idx == t_mem_head.addr[IDX_STOP-1:IDX_START] );
	
	t_cm_block = r_got_req && r_last_wr && 
		     (r_cache_idx == core_mem_va_req.addr[IDX_STOP-1:IDX_START])
	  /*&& (r_cache_tag == core_mem_va_req.addr[`M_WIDTH-1:IDX_STOP])*/;


	t_cm_block_stall = t_cm_block && !(r_did_reload||r_is_retry);//1'b0;
	
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
	       if(r_got_req2)
		 begin
		    n_core_mem_rsp.data = r_req2.addr;
		    n_core_mem_rsp.rob_ptr = r_req2.rob_ptr;
		    n_core_mem_rsp.dst_ptr = r_req2.dst_ptr;
		    t_req2_pa.addr = w_tlb_pa;

		    if(r_pending_tlb_miss)
		      begin
`ifdef VERBOSE_L1D
			 $display("pc %x, replay address %x, mapped to %x, hit %b, rob id %d  cycle %d, cache hit %b",
				  r_req2.pc, r_tlb_addr, w_tlb_pa, t_port2_hit_cache, r_req2.rob_ptr, r_cycle, t_port2_hit_cache );
`endif
			 n_pending_tlb_miss = 1'b0;
		      end
		    
		    if(drain_ds_complete || r_req2.op == MEM_NOP)
		      begin
			 n_core_mem_rsp.dst_valid = r_req2.dst_valid;
			 n_core_mem_rsp.has_cause = r_req2.has_cause;
			 n_core_mem_rsp.cause = r_req2.cause;
			 n_core_mem_rsp.addr = r_req2.addr;
			 n_core_mem_rsp_valid = 1'b1;
		      end
		    else if(!w_tlb_hit)
		      begin
			 //$display("missed address %x", r_tlb_addr);
			 n_pending_tlb_miss = 1'b1;
			 if(r_pending_tlb_miss) $stop();
		      end
		    else if(w_tlb_st_exc)
		      begin
			 $display("store exception for pc %x, addr %x", 
				  r_req2.pc, r_req2.addr);
			 n_core_mem_rsp.dst_valid = r_req2.dst_valid;
			 n_core_mem_rsp.has_cause = 1'b1;
			 n_core_mem_rsp.cause = STORE_PAGE_FAULT;
			 n_core_mem_rsp.addr = r_req2.addr;
			 n_core_mem_rsp_valid = 1'b1;			 
		      end
		    else if(r_req2.is_atomic)
		      begin
			 t_push_miss = 1'b1;
		      end
		    else if(r_req2.is_store)
		      begin
			 t_push_miss = 1'b1;
			 t_incr_busy = 1'b1;
			 n_stall_store = 1'b1;
			 //ack early
			 n_core_mem_rsp.dst_valid = 1'b0;
			 if(t_port2_hit_cache)
			   begin
			      n_cache_hits = r_cache_hits + 'd1;
			   end
			 n_core_mem_rsp_valid = 1'b1;
			 n_core_mem_rsp.has_cause = r_req2.spans_cacheline;			 
		      end // if (r_req2.is_store)
		    else if(t_port2_hit_cache && (!r_hit_busy_addr2) & (!r_pending_tlb_miss) )
		      begin
`ifdef VERBOSE_L1D
			 $display("cycle %d port2 hit for pc %x addr %x, data %x, rob %d dst %x dst valid %b", r_cycle, 
				  r_req2.pc, r_req2.addr, t_rsp_data2, r_req2.rob_ptr, r_req2.dst_ptr, r_req2.dst_valid);
`endif
			 n_core_mem_rsp.data = t_rsp_data2[`M_WIDTH-1:0];
                         n_core_mem_rsp.dst_valid = t_rsp_dst_valid2;
                         n_cache_hits = r_cache_hits + 'd1;
                         n_core_mem_rsp_valid = 1'b1;
			 n_core_mem_rsp.has_cause = r_req2.spans_cacheline;
		      end
		    else
		      begin
			 t_push_miss = 1'b1;
			 if(t_port2_hit_cache)
			   begin
			      n_cache_hits = r_cache_hits + 'd1;
			   end
		      end
		 end // if (r_got_req2)
	       

	       if(r_got_req)
		 begin
`ifdef VERBOSE_L1D
		       $display("req 1 : cycle %d, rob ptr %d, r_is_retry %b, addr %x, is store %b, r_cache_idx = %d, r_cache_tag = %d, valid %b",
				r_cycle, r_req.rob_ptr, r_is_retry, r_req.addr, r_req.is_store, r_cache_idx, r_cache_tag, r_valid_out);

`endif
		    if(r_valid_out && (r_tag_out == r_cache_tag))
		      begin /* valid cacheline - hit in cache */
			 if(r_req.is_store)
			   begin
			      t_reset_graduated = 1'b1;				   
			   end
			 else
			   begin
			      n_core_mem_rsp.data = t_rsp_data[`M_WIDTH-1:0];
			      n_core_mem_rsp.dst_valid = t_rsp_dst_valid;
			      n_core_mem_rsp_valid = 1'b1;
			      n_core_mem_rsp.has_cause = r_req.spans_cacheline;
`ifdef VERBOSE_L1D			      
			      if(r_did_reload)
				begin
				   $display("late ack at cycle %d for load with rob ptr %d, data %x, dst valid %b", 
					    r_cycle, r_req.rob_ptr, n_core_mem_rsp.data , n_core_mem_rsp.dst_valid );
			      end
`endif
			      
			   end // else: !if(r_req.is_store)
		      end // if (r_valid_out && (r_tag_out == r_cache_tag))
		    else if(r_valid_out && r_dirty_out && (r_tag_out != r_cache_tag) )
		      begin
			 t_got_miss = 1'b1;
			 n_inhibit_write = 1'b1;
			 if(r_hit_busy_addr && r_is_retry || !r_hit_busy_addr)
			   begin
			      n_mem_req_addr = {r_tag_out,r_cache_idx,4'd0};
			      n_mem_req_opcode = MEM_SW;
			      n_mem_req_store_data = t_data;
			      n_inhibit_write = 1'b1;
			      t_miss_idx = r_cache_idx;
			      t_miss_addr = r_req.addr;

			      n_lock_cache = 1'b1;
			      if((rr_cache_idx == r_cache_idx) && rr_last_wr)
				begin
				   //$display("inflight write to line, must wait");
				   t_cache_idx = r_cache_idx;
				   n_state = WAIT_INJECT_RELOAD;
				   n_mem_req_valid = 1'b0;				   
				end
			      else
				begin
				   //$display("no wait");
				   n_state = INJECT_RELOAD;				   
				   n_mem_req_valid = 1'b1;
				end
			   end // if (!t_stall_for_busy)
		      end
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
				 n_mem_req_addr = {r_tag_out,r_cache_idx,4'd0};
				 n_lock_cache = 1'b1;
				 n_mem_req_opcode = MEM_SW;
				 n_state = WAIT_INJECT_RELOAD;
				 n_mem_req_valid = 1'b0;
			      end                                                             
			    else
			      begin
				 n_lock_cache = 1'b0;
				 n_mem_req_addr = {r_req.addr[`M_WIDTH-1:`LG_L1D_CL_LEN], 4'd0};
				 n_mem_req_opcode = MEM_LW;				 
				 n_state = INJECT_RELOAD;
				 n_mem_req_valid = 1'b1;
			      end
			 end // if (r_hit_busy_addr && r_is_retry || !r_hit_busy_addr || r_lock_cache)
		       else
			 begin
			    $stop();
			 end
`ifdef VERBOSE_L1D
		       $display("at cycle %d : cache invalid miss for rob ptr %d, r_is_retry %b, addr %x, is store %b, r_cache_idx = %d, r_cache_tag = %d, valid %b, n_state %d",
				r_cycle, 
				r_req.rob_ptr, 
				r_is_retry, 
				r_req.addr, 
				r_req.is_store, 
				r_cache_idx, 
				r_cache_tag, 
				r_valid_out,
				n_state);
`endif
		       
		    end // else: !if(r_valid_out && r_dirty_out && (r_tag_out != r_cache_tag)...
		 end // if (r_got_req)
	       else if(n_pending_tlb_miss)
		 begin
		    if(n_state != ACTIVE)
		      begin
			 $stop();
		      end
		    n_state = TLB_RELOAD;
		    n_page_walk_gnt = 1'b0;
		    n_page_walk_req_valid = 1'b1;
		 end


	       
	     if(!mem_q_empty && !t_got_miss && !r_lock_cache && !n_pending_tlb_miss)
	       begin
		  // $display("t_mem_head.rob_ptr = %d, grad %b, dq ptr %d valid %b, data %x, addr %x, pc %x", 
		  // 	   t_mem_head.rob_ptr, 
		  // 	   r_graduated[t_mem_head.rob_ptr], 
		  // 	   core_store_data.rob_ptr, 
		  // 	   core_store_data_valid,
		  // 	   core_store_data.data,
		  // 	   t_mem_head.addr,
		  // 	   t_mem_head.pc
		  // 	   );
	  
		  if(!t_mh_block)
		    begin
		       if(t_mem_head.is_store || t_mem_head.is_atomic)
			 begin
			    
			    if(w_st_amo_grad && (core_store_data_valid ? (t_mem_head.rob_ptr == core_store_data.rob_ptr) : 1'b0) )
			      begin
				 //`ifdef VERBOSE_L1D
//`endif
				 t_pop_mq = 1'b1;
				 core_store_data_ack = 1'b1;
				 n_req = t_mem_head;
				 n_req.data = core_store_data.data;
				 t_cache_idx = t_mem_head.addr[IDX_STOP-1:IDX_START];
				 t_cache_tag = t_mem_head.addr[`M_WIDTH-1:IDX_STOP];
				 t_addr = t_mem_head.addr;
				 t_got_req = 1'b1;
				 n_is_retry = 1'b1;
				 n_last_wr = 1'b1;
			      end // if (t_mem_head.rob_ptr == head_of_rob_ptr)
			    else if(drain_ds_complete && dead_rob_mask[t_mem_head.rob_ptr])
			      begin
`ifdef VERBOSE_L1D
				 $display("CLEARING EVERYTHING OUT, should clear line %d for rob ptr %d, data %x", 
					  t_mem_head.addr[IDX_STOP-1:IDX_START], t_mem_head.rob_ptr, t_mem_head.data);
`endif
				 t_pop_mq = 1'b1;
				 t_force_clear_busy = 1'b1;
			      end
			 end // if (t_mem_head.is_store)
		       else
			 begin
			    t_pop_mq = 1'b1;
			    n_req = t_mem_head;
			    t_cache_idx = t_mem_head.addr[IDX_STOP-1:IDX_START];
			    t_cache_tag = t_mem_head.addr[`M_WIDTH-1:IDX_STOP];
			    t_addr = t_mem_head.addr;
			    t_got_req = 1'b1;
			    n_is_retry = 1'b1;
			    n_last_rd = 1'b1;
			    t_got_rd_retry = 1'b1;
`ifdef VERBOSE_L1D			    
			    $display("firing load for %x at cycle %d for rob ptr %d, state = %d, n_state = %d", 
				     t_mem_head.addr, r_cycle, t_mem_head.rob_ptr, r_state, n_state);
`endif
			 end
		    end
	       end // if (!mem_q_empty && !t_got_miss && !r_lock_cache)
	       
	     
	       if(core_mem_va_req_valid &&
		  !t_got_miss && 
		  !(mem_q_almost_full||mem_q_full) && 
		  !t_got_rd_retry &&
		  !(r_last_wr2 && (r_cache_idx2 == core_mem_va_req.addr[IDX_STOP-1:IDX_START]) && !core_mem_va_req.is_store) &&
		  !(n_pending_tlb_miss | r_pending_tlb_miss) &&
		  !t_cm_block_stall &&
		  (!r_rob_inflight[core_mem_va_req.rob_ptr])
		  )
	       begin
		  //use 2nd read port
		  t_cache_idx2 = core_mem_va_req.addr[IDX_STOP-1:IDX_START];
		  t_cache_tag2 = core_mem_va_req.addr[`M_WIDTH-1:IDX_STOP];
		  
		  n_req2 = core_mem_va_req;
		  core_mem_va_req_ack = 1'b1;
		  t_got_req2 = 1'b1;
		  t_tlb_xlat = 1'b1;
		  n_tlb_addr = core_mem_va_req.addr;

`ifdef VERBOSE_L1D
		  $display("accepting new op %d, pc %x addr %x for rob ptr %d at cycle %d, mem_q_empty %b", 
			   core_mem_va_req.op,
			   core_mem_va_req.pc,
			   core_mem_va_req.addr,
			   core_mem_va_req.rob_ptr,
			   r_cycle,
			   mem_q_empty);
`endif
		  
		  n_last_wr2 = core_mem_va_req.is_store;
		  n_last_rd2 = !core_mem_va_req.is_store;
		  
		  n_cache_accesses =  r_cache_accesses + 'd1;
	       end // if (core_mem_va_req_valid &&...
	       else if(r_flush_req && mem_q_empty && !(r_got_req && r_last_wr))
		 begin
		    if(n_state != r_state) $stop();
		    n_state = FLUSH_CACHE;
		    if(!mem_q_empty) $stop();
		    if(r_got_req && r_last_wr) $stop();
		    //$display("flush begins at cycle %d, mem_q_empty = %b", 
		    //r_cycle, mem_q_empty);
		    t_cache_idx = 'd0;
		    n_flush_req = 1'b0;
		 end
	       else if(r_flush_cl_req && mem_q_empty && !(r_got_req && r_last_wr)
		       && !(n_page_walk_req_valid | t_got_miss | r_wr_array | t_wr_array))
		 begin
		    //$display("t_got_miss = %b, n_state = %d", t_got_miss, n_state);
		    if(n_state != r_state) $stop();		    
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
	       n_mem_req_valid = 1'b1;
	       n_state = INJECT_RELOAD;
	       n_mem_req_store_data = t_data;
	    end
	  INJECT_RELOAD:
	    begin
	       if(mem_rsp_valid)
		 begin
		    n_state = HANDLE_RELOAD;
		    n_inhibit_write = 1'b0;
		    if(!(r_req.is_store || r_req.is_atomic || r_lock_cache))
		      begin
			 t_ack_ld_early = 1'b1;
			 n_core_mem_rsp.rob_ptr = r_req.rob_ptr;
			 n_core_mem_rsp.dst_ptr = r_req.dst_ptr;			 
			 n_core_mem_rsp.data = t_rsp_data[`M_WIDTH-1:0];
			 n_core_mem_rsp.has_cause = r_req.spans_cacheline;
			 n_core_mem_rsp_valid = 1'b1;
			 n_core_mem_rsp.dst_valid = r_req.dst_valid & n_core_mem_rsp_valid;
`ifdef VERBOSE_L1D
			 $display("early ack at cycle %d for load with rob ptr %d, data %x, dst valid %b, addr %x, r_lock_cache = %b",
				  r_cycle, r_req.rob_ptr, n_core_mem_rsp.data , n_core_mem_rsp.dst_valid, r_req.addr, r_lock_cache );
`endif
		      end
		 end
	    end
	  HANDLE_RELOAD:
	    begin
	       t_cache_idx = r_req.addr[IDX_STOP-1:IDX_START];
	       t_cache_tag = r_req.addr[`M_WIDTH-1:IDX_STOP];
	       n_last_wr = r_req.is_store;
	       t_got_req = r_req.is_store | (r_ack_ld_early == 1'b0);
	       t_addr  = r_req.addr;
	       n_did_reload = 1'b1;
	       n_state = ACTIVE;
	    end
	  FLUSH_CL:
	    if(r_dirty_out & w_flush_hit)
	      begin
		 n_mem_req_addr = {r_tag_out,r_cache_idx,4'd0};
		 n_mem_req_opcode = MEM_SW;
		 n_mem_req_store_data = t_data;
		 n_state = FLUSH_CL_WAIT;
		 n_inhibit_write = 1'b1;
		 n_mem_req_valid = 1'b1;	       
	      end
	    else
	      begin
		 n_state = r_flush_was_active ? ACTIVE : TLB_RELOAD;
		 n_flush_was_active = 1'b0;
		 t_mark_invalid = w_flush_hit;		 
		 n_l2_probe_ack = 1'b1;
	      end // else: !if(r_dirty_out)
	  FLUSH_CL_WAIT:
	    begin
	       	if(mem_rsp_valid)
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
	       //$display("flush line %x was %b", 
	       //{r_tag_out,r_cache_idx,{`LG_L1D_CL_LEN{1'b0}}},
	       //	r_dirty_out);
	       
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
		    n_mem_req_addr = {r_tag_out,r_cache_idx,4'd0};
		    n_mem_req_opcode = MEM_SW;
		    n_mem_req_store_data = t_data;
		    n_state = (r_cache_idx == (L1D_NUM_SETS-1)) ? FLUSH_CACHE_LAST_WAIT : FLUSH_CACHE_WAIT;
		    n_inhibit_write = 1'b1;
		    n_mem_req_valid = 1'b1;
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
		    if(page_walk_rsp.fault)
		      begin
			 //$display("taking page fault for pc %x", r_req2.pc);
			 n_req2.op = MEM_NOP;
			 n_req2.is_store = 1'b0;
			 n_req2.has_cause = 1'b1;
			 n_req2.cause = (r_req2.is_store | r_req2.is_atomic) ? 
					STORE_PAGE_FAULT : LOAD_PAGE_FAULT;
			 //$stop();
		      end		    
		 end // if (page_walk_rsp_valid)
	       else if(n_flush_cl_req)
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
	       t_tlb_xlat = 1'b1;
	    end
	  default:
	    begin
	    end
	endcase // case r_state
     end // always_comb

   always_ff@(negedge clk)
     begin
	// if(retired_rob_ptr_valid && (retired_rob_ptr == 'd1))
	//   begin
	//      $display("port a marking retired at cycle %d", r_cycle);
	//   end
	// if(retired_rob_ptr_two_valid && (retired_rob_ptr_two == 'd1) )
	//   begin
	//      $display("port b marking retired at cycle %d", r_cycle);
	//   end
	
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
     end


endmodule // l1d

