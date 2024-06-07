`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

//`define VERBOSE_L1D 1

`ifdef VERILATOR
import "DPI-C" function int read_word(input longint addr);
import "DPI-C" function longint read_dword(input longint addr);
import "DPI-C" function void write_byte(input longint addr, input byte data, input longint root);
import "DPI-C" function void write_half(input longint addr, input shortint  data, input longint root);
import "DPI-C" function void write_word(input longint addr, input int data, input longint root, int id);
import "DPI-C" function void write_dword(input longint addr, input longint data, input longint root, int id);
import "DPI-C" function longint dc_ld_translate(longint va, longint root );
`endif


module perfect_l1d(clk, 
	   reset,
	   priv,
           page_table_root,		   
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
	   mem_req_uc,
	   mem_req_addr, 
	   mem_req_store_data, 
	   mem_req_opcode,
	   //reply from memory system
	   mem_rsp_valid,
	   mem_rsp_load_data,
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
   input logic [1:0] priv;
   input logic [63:0] page_table_root;
   input logic l2_probe_val;
   input logic [(`M_WIDTH-1):0]	l2_probe_addr;
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

   output logic mem_req_valid;
   output logic	mem_req_uc;
   output logic [(`M_WIDTH-1):0] mem_req_addr;
   output logic [L1D_CL_LEN_BITS-1:0] mem_req_store_data;
   output logic [3:0] 			  mem_req_opcode;

   input logic 				  mem_rsp_valid;
   input logic [L1D_CL_LEN_BITS-1:0] 	  mem_rsp_load_data;

   output logic [63:0]			  mtimecmp;
   output logic				  mtimecmp_val;
      
   output logic [63:0] 			 cache_accesses;
   output logic [63:0] 			 cache_hits;

   output logic [63:0]			 tlb_accesses;
   output logic [63:0] 			 tlb_hits;


   assign page_walk_req_valid = 1'b0;
   always_ff@(posedge clk)
    begin
       l2_probe_ack <= reset ? 1'b0 : l2_probe_val;
    end

   logic [63:0]	r_mtimecmp;
   logic	r_mtimecmp_val;
   assign mtimecmp = r_mtimecmp;
   assign mtimecmp_val = r_mtimecmp_val;
   
   
      
        
`ifdef VERILATOR
         
   localparam LG_WORDS_PER_CL = `LG_L1D_CL_LEN - 2;
   localparam LG_DWORDS_PER_CL = `LG_L1D_CL_LEN - 3;
   
   localparam WORDS_PER_CL = 1<<(LG_WORDS_PER_CL);
   localparam N_TAG_BITS = `M_WIDTH - `LG_L1D_NUM_SETS - `LG_L1D_CL_LEN;
   localparam IDX_START = `LG_L1D_CL_LEN;
   localparam IDX_STOP  = `LG_L1D_CL_LEN + `LG_L1D_NUM_SETS;
   localparam WORD_START = 2;
   localparam WORD_STOP = WORD_START+LG_WORDS_PER_CL;

  
   localparam N_MQ_ENTRIES = (1<<`LG_MRQ_ENTRIES);

   
   logic 				  r_got_req, r_last_wr, n_last_wr;
   logic 				  r_last_rd, n_last_rd;
   logic 				  r_got_req2, r_last_wr2, n_last_wr2;
   logic 				  r_last_rd2, n_last_rd2;
   
   logic 				  rr_got_req, rr_last_wr, rr_is_retry;

   
   logic [`LG_MRQ_ENTRIES:0] 		  r_n_inflight;   


   
   //1st read port
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_cache_idx, r_cache_idx, rr_cache_idx;
   logic [N_TAG_BITS-1:0] 		  t_cache_tag, r_cache_tag;
   logic [N_TAG_BITS-1:0] 		  rr_cache_tag;
   logic 				  r_valid_out, r_dirty_out;
   logic [L1D_CL_LEN_BITS-1:0] 		  t_data;
   
   //2nd read port
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_cache_idx2, r_cache_idx2;
   logic [N_TAG_BITS-1:0] 		  t_cache_tag2, r_cache_tag2;
   logic 				  r_valid_out2, r_dirty_out2;
   
   
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_miss_idx, r_miss_idx;
   logic [`M_WIDTH-1:0] 		  t_miss_addr, r_miss_addr;

   //write port   
 

   logic 				  r_flush_req, n_flush_req;
   logic 				  r_flush_cl_req, n_flush_cl_req;
   logic 				  r_flush_complete, n_flush_complete;
   

   logic [63:0]				  t_w64;
   logic [31:0] 			  t_w32, t_bswap_w32;
   logic [31:0] 			  t_w32_2, t_bswap_w32_2;

   logic 				  t_got_rd_retry, t_port2_hit_cache;
      
   logic 				  t_wr_array;
   logic 				  t_hit_cache;
   logic 				  t_rsp_dst_valid;
   logic [63:0] 			  t_rsp_data;
   
   logic 				  t_hit_cache2;
   logic 				  t_rsp_dst_valid2;
   logic [63:0] 			  t_rsp_data2;


   
   logic [L1D_CL_LEN_BITS-1:0] 		  t_array_data;
   
   logic [`M_WIDTH-1:0] 		  t_addr;
   logic 				  t_got_req, t_got_req2;
   logic 				  t_got_miss;
   logic 				  t_push_miss;
   
   logic 				  t_mh_block, t_cm_block, t_cm_block2,
					  t_cm_block_stall;

   logic                                  t_incr_busy,t_force_clear_busy;
      
   logic 				  n_is_retry, r_is_retry;
   
   logic 				  n_core_mem_rsp_valid, r_core_mem_rsp_valid;
   mem_rsp_t n_core_mem_rsp, r_core_mem_rsp;
      
   mem_req_t n_req, r_req, t_req;
   mem_req_t n_req2, r_req2;

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mtimecmp <= 64'd0;
	     r_mtimecmp_val <= 1'b0;
	  end
	else
	  begin
	     r_mtimecmp_val <= t_wr_array && r_req.addr == `MTIMECMP_ADDR;
	     r_mtimecmp <= r_req.data;
	  end
     end // always_ff@ (posedge clk)


   
   mem_req_t r_mem_q[N_MQ_ENTRIES-1:0];
   logic [`LG_MRQ_ENTRIES:0] r_mq_head_ptr, n_mq_head_ptr;
   logic [`LG_MRQ_ENTRIES:0] r_mq_tail_ptr, n_mq_tail_ptr;
   logic [`LG_MRQ_ENTRIES:0] t_mq_tail_ptr_plus_one;

   
   logic [N_MQ_ENTRIES-1:0] r_mq_addr_valid;
   logic [IDX_STOP-IDX_START-1:0] r_mq_addr[N_MQ_ENTRIES-1:0];
  
   
   mem_req_t t_mem_tail, t_mem_head;
   logic 	mem_q_full, mem_q_empty, mem_q_almost_full;
   
   typedef enum logic [2:0] {INITIALIZE,
			     ACTIVE
                             } state_t;

   
   state_t r_state, n_state;
   logic 	t_pop_mq;
   
   
   
   logic r_mem_req_valid, n_mem_req_valid;
   logic [(`M_WIDTH-1):0] r_mem_req_addr, n_mem_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] r_mem_req_store_data, n_mem_req_store_data;
   
   logic [3:0] 		       r_mem_req_opcode, n_mem_req_opcode;
   logic [63:0] 	       n_cache_accesses, r_cache_accesses;
   logic [63:0] 	       n_cache_hits, r_cache_hits;
   
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
   logic [N_ROB_ENTRIES-1:0] r_missed;
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
	     r_missed <= 'd0;
	  end
	else
	  begin
	     if(t_push_miss)
	       begin
		  r_missed[r_req2.rob_ptr] <= !t_port2_hit_cache;
	       end
	  end
     end // always_ff@ (posedge clk)

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
	     if(r_got_req)
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
   // 	if(t_pop_mq && r_missed[t_mem_head.rob_ptr])
   // 	  begin
   // 	     $display("cycle %d : popping rob ptr %d, addr %x -> was store %b",
   // 		      r_cycle,
   // 		      t_mem_head.rob_ptr,
   // 		      t_mem_head.addr,
   // 		      t_mem_head.is_store);
   // 	  end
   //   end
   
   always_ff@(posedge clk)
     begin
	if(t_push_miss)
	  begin
	     r_mem_q[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0] ] <= r_req2;
	     r_mq_addr[r_mq_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= r_req2.addr[IDX_STOP-1:IDX_START];
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
   logic [N_MQ_ENTRIES-1:0] r_hit_busy_addrs2;
   logic 		   r_hit_busy_addr2;

   generate
      for(genvar i = 0; i < N_MQ_ENTRIES; i=i+1)
	begin
	   assign w_hit_busy_addrs[i] = (t_pop_mq && r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0] == i) ? 1'b0 :
					r_mq_addr_valid[i] ? r_mq_addr[i] == t_cache_idx : 
					1'b0;
	   assign w_hit_busy_addrs2[i] = //(t_pop_mq && r_mq_head_ptr[`LG_MRQ_ENTRIES-1:0] == i) ? 1'b0 :
					 r_mq_addr_valid[i] ? r_mq_addr[i] == t_cache_idx2 : 1'b0;	   
	end
   endgenerate
   

   always_ff@(posedge clk)
     begin
	r_hit_busy_addr <= reset ? 1'b0 : |w_hit_busy_addrs;
	r_hit_busy_addrs <= t_got_req ? w_hit_busy_addrs : {{N_MQ_ENTRIES{1'b1}}};
	
	r_hit_busy_addr2 <= reset ? 1'b0 : |w_hit_busy_addrs2;
	r_hit_busy_addrs2 <= t_got_req2 ? w_hit_busy_addrs2 : {{N_MQ_ENTRIES{1'b1}}};
     end


   
   
`ifdef VERBOSE_L1D
   always_ff@(negedge clk)
   begin
      if(t_pop_mq)
	begin
	   $display("popping pc %d rob ptr %d at cycle %d", 
		     t_mem_head.pc, t_mem_head.rob_ptr, r_cycle);
	end
   end
`endif

   // always_ff@(negedge clk)
   // begin
   //    if(!mem_q_empty)
   // 	begin
   // 	   $display("mem head pc %x rob ptr %d at cycle %d", 
   // 		     t_mem_head.pc, t_mem_head.rob_ptr, r_cycle);
   // 	end
   // end
   
   logic r_dead_atomic, n_dead_atomic;
  
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
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
	     rr_is_retry <= 1'b0;
	     
	     rr_last_wr <= 1'b0;
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
	     memq_empty <= 1'b1;
	     r_dead_atomic <= 1'b0;
	  end
	else
	  begin
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
	     r_got_req2 <= t_got_req2;
	     
	     rr_got_req <= r_got_req;
	     rr_is_retry <= r_is_retry;
	     
	     rr_last_wr <= r_last_wr;
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
	     memq_empty <= mem_q_empty 
			   && drain_ds_complete 
			   && !core_mem_va_req_valid 
			   && !t_got_req && !t_got_req2 
			   && !t_push_miss
			   && (r_n_inflight == 'd0);
	     
	     r_dead_atomic <= n_dead_atomic;
	  end
     end // always_ff@ (posedge clk)

   
   always_ff@(posedge clk)
     begin
	r_req <= n_req;
	r_req2 <= n_req2;
	r_core_mem_rsp <= n_core_mem_rsp;
     end



   always_ff@(posedge clk)
     begin
	r_dirty_out <= 1'b0;
	r_dirty_out2 <= 1'b0;
	r_valid_out <= 1'b1;
	r_valid_out2 <= 1'b1;
     end
   



   logic [31:0] tt_w32_2, tt_bswap_w32_2;
   logic [63:0]	tt_w64;

   logic [63:0]	t_req2_addr_pa, t_pa2;
   logic	t_pf2;



   always_ff@(posedge clk)
     begin
	t_pa2 <= dc_ld_translate({n_req2.addr[63:12], 12'd0}, page_table_root);
     end
   
   always_comb
     begin
	t_hit_cache2 =  r_got_req2 && (r_state == ACTIVE);
	
	t_rsp_dst_valid2 = 1'b0;
	t_rsp_data2 = 'd0;

	t_req2_addr_pa = paging_active ? t_pa2 : {r_req2.addr[63:12], 12'd0};
	t_pf2 = paging_active & (&t_req2_addr_pa);
	
	tt_w64 = read_dword( {t_req2_addr_pa[63:12], r_req2.addr[11:3], 3'd0});
	tt_w32_2 = read_word({t_req2_addr_pa[63:12], r_req2.addr[11:2], 2'd0});
	
 
	tt_bswap_w32_2 = bswap32(tt_w32_2);
	
	case(r_req2.op)
	  MEM_LB:
	    begin
	       case(r_req2.addr[1:0])
		 2'd0:
		   begin
		      t_rsp_data2 = {{56{tt_w32_2[7]}}, tt_w32_2[7:0]};
		   end
		 2'd1:
		   begin
		      t_rsp_data2 = {{56{tt_w32_2[15]}}, tt_w32_2[15:8]};
		   end
		 2'd2:
		   begin
		      t_rsp_data2 = {{56{tt_w32_2[23]}}, tt_w32_2[23:16]};
		   end
		 2'd3:
		   begin
		      t_rsp_data2 = {{56{tt_w32_2[31]}}, tt_w32_2[31:24]};
		   end
	       endcase
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end
	  MEM_LBU:
	    begin
	       case(r_req2.addr[1:0])
		 2'd0:
		   begin
		      t_rsp_data2 = {56'd0, tt_w32_2[7:0]};
		   end
		 2'd1:
		   begin
		      t_rsp_data2 = {56'd0, tt_w32_2[15:8]};
		   end
		 2'd2:
		   begin
		      t_rsp_data2 = {56'd0, tt_w32_2[23:16]};
		   end
		 2'd3:
		   begin
		      t_rsp_data2 = {56'd0, tt_w32_2[31:24]};
		   end
	       endcase // case (r_req2.addr[1:0])
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;	       
	    end
	  MEM_LH:
	    begin
	       case(r_req2.addr[1])
		 1'b0:
		   begin
		      t_rsp_data2 = {{48{sext16(tt_w32_2[15:0])}}, bswap16(tt_w32_2[15:0])};
		   end
		 1'b1:
		   begin
		      t_rsp_data2 = {{48{sext16(tt_w32_2[31:16])}}, bswap16(tt_w32_2[31:16])};	     
		   end
	       endcase 
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end
	  MEM_LHU:
	    begin
	       t_rsp_data2 = {48'd0, (r_req2.addr[1] ? tt_w32_2[31:16] : tt_w32_2[15:0])};
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;	       
	    end
	  MEM_LW:
	    begin
	       t_rsp_data2 = {{32{tt_bswap_w32_2[31]}}, tt_bswap_w32_2};
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end	 
	  MEM_LWU:
	    begin
	       t_rsp_data2 = {32'd0, tt_bswap_w32_2};
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end	 
	  MEM_LD:
	    begin
	       t_rsp_data2 = tt_w64;
	       t_rsp_dst_valid2 = r_req2.dst_valid & t_hit_cache2;
	    end
	  default:
	    begin
	       //$stop();
	    end
	endcase
     end // always_comb

   //always_ff@(negedge clk)
     //begin
   //if(t_rsp_dst_valid2)
   //$display("load port2 address %x, op %d", r_req2.addr, r_req2.op);
   //end
   
   logic [63:0] t_req_addr_pa, t_pa;
   logic	t_pf;
   
   logic [31:0]	t_amo32_data;
   logic [63:0]	t_amo64_data;
   
   wire		w_dead_atomic = drain_ds_complete && dead_rob_mask[r_req.rob_ptr];


   always_ff@(negedge clk)
     begin
	case(r_req.op)
	  MEM_SB:
	    begin
	       if(t_wr_array)
		 write_byte(r_req.addr, r_req.data[7:0],paging_active ? page_table_root : 64'd0);

	    end
	  MEM_SH:
	    begin
	       if(t_wr_array)
		 write_half(r_req.addr, bswap16(r_req.data[15:0]),paging_active ? page_table_root : 64'd0);
	    end
	  MEM_SW:
	    begin
	       if(t_wr_array)
		 begin
		    write_word(r_req.addr, bswap32(r_req.data[31:0]),paging_active ? page_table_root : 64'd0, 32'd0);
		 end
	    end
	  MEM_SD:
	    begin
	       if(t_wr_array)
		 write_dword(r_req.addr, r_req.data, paging_active ? page_table_root : 64'd0, r_cycle[31:0]);	       	       
	    end
	  MEM_SCD:
	    begin
	       if(t_wr_array & !(r_dead_atomic || w_dead_atomic))
		 write_dword(r_req.addr, r_req.data, paging_active ? page_table_root : 64'd0, 32'd1);
	       //if(t_wr_array) $display("execute sc.d at cycle %d", r_cycle);
	    end
	  MEM_SCW:
	    begin
	       if(t_wr_array & !(r_dead_atomic|w_dead_atomic))
		 begin
		    write_word(r_req.addr, r_req.data[31:0], paging_active ? page_table_root : 64'd0, 32'd1);
		 end
	       
	    end
	  MEM_AMOW:
	    begin
	       if(t_wr_array & !(r_dead_atomic||w_dead_atomic))
		 begin
		    //$display("AMOW for pc %x, data %x, cycle %d", r_req.pc, t_amo64_data, r_cycle);		    
		    write_word(r_req.addr, t_amo32_data, paging_active ? page_table_root : 64'd0, 32'd2);
		 end
	    end // case: MEM_AMOW
	  MEM_AMOD:
	    begin
	       if(t_wr_array  & !(r_dead_atomic||w_dead_atomic))
		 begin
		    //$display("AMOD op %d for pc %x, data %x, cycle %d", r_req.amo_op, r_req.pc, t_amo64_data, r_cycle);
		    write_dword(r_req.addr, t_amo64_data, paging_active ? page_table_root : 64'd0, 32'd2);
		 end
	    end // case: MEM_AMOD
	  default:
	    begin
	    end
	endcase // case r_req.op
     end // always_ff@ (negedge clk)
   

   always_ff@(posedge clk)
     begin
	t_pa <= dc_ld_translate({n_req.addr[63:12], 12'd0}, page_table_root);
     end
   
   always_comb
     begin
	t_data = 'd0;
	
	t_req_addr_pa = paging_active ? t_pa : {r_req.addr[63:12], 12'd0};
	t_pf = paging_active & (&t_req_addr_pa);
	
	t_w64 = read_dword({t_req_addr_pa[63:12], r_req.addr[11:3], 3'd0});
	t_w32 = read_word( {t_req_addr_pa[63:12], r_req.addr[11:2], 2'd0});
	
        t_bswap_w32 = t_w32;
	
	t_hit_cache = r_got_req && 
		      (r_state == ACTIVE);
	
	t_rsp_dst_valid = 1'b0;
	t_rsp_data = 'd0;

	case(r_req.amo_op)
	  5'd0: /* amoadd */
	    begin
	       t_amo32_data = t_w32 + r_req.data[31:0];
	       t_amo64_data = t_w64 + r_req.data[63:0];
	       //$display("amo add data %x", r_req.data);
	    end
	  5'd1: /* amoswap */
	    begin
	       t_amo32_data = r_req.data[31:0];
	       t_amo64_data = r_req.data[63:0];
	    end
	  5'd8: /* amoor */
	    begin
	       t_amo32_data = t_w32 | r_req.data[31:0];
	       t_amo64_data = t_w64 | r_req.data[63:0];
	    end
	  5'd12: /* amoand */
	    begin
	       t_amo32_data = t_w32 & r_req.data[31:0];
	       t_amo64_data = t_w64 & r_req.data[63:0];
	    end
	  5'd28: /* amomax */
	    begin
	       t_amo32_data = t_w32 < r_req.data[31:0] ? r_req.data[31:0] : t_w32;
	       t_amo64_data = t_w64 < r_req.data[63:0] ? r_req.data[63:0] : t_w64;
	    end
	  
	  default:
	    begin
	    end
	endcase // case (r_req.amo_op)

	case(r_req.op)
	  MEM_LB:
	    begin
	       case(r_req.addr[1:0])
		 2'd0:
		   begin
		      t_rsp_data = {{56{t_w32[7]}}, t_w32[7:0]};
		   end
		 2'd1:
		   begin
		      t_rsp_data = {{56{t_w32[15]}}, t_w32[15:8]};
		   end
		 2'd2:
		   begin
		      t_rsp_data = {{56{t_w32[23]}}, t_w32[23:16]};
		   end
		 2'd3:
		   begin
		      t_rsp_data = {{56{t_w32[31]}}, t_w32[31:24]};
		   end
	       endcase
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end
	  MEM_LBU:
	    begin
	       case(r_req.addr[1:0])
		 2'd0:
		   begin
		      t_rsp_data = {56'd0, t_w32[7:0]};
		   end
		 2'd1:
		   begin
		      t_rsp_data = {56'd0, t_w32[15:8]};
		   end
		 2'd2:
		   begin
		      t_rsp_data = {56'd0, t_w32[23:16]};
		   end
		 2'd3:
		   begin
		      t_rsp_data = {56'd0, t_w32[31:24]};
		   end
	       endcase // case (r_req.addr[1:0])
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;	       
	    end
	  MEM_LH:
	    begin
	       case(r_req.addr[1])
		 1'b0:
		   begin
		      t_rsp_data = {{48{sext16(t_w32[15:0])}}, bswap16(t_w32[15:0])};
		   end
		 1'b1:
		   begin
		      t_rsp_data = {{48{sext16(t_w32[31:16])}}, bswap16(t_w32[31:16])};	     
		   end
	       endcase // case (r_req.addr[1])
	       t_rsp_dst_valid = r_req.dst_valid &t_hit_cache;
	    end
	  MEM_LHU:
	    begin
	       t_rsp_data = {48'd0, (r_req.addr[1] ? t_w32[31:16] : t_w32[15:0])};
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;	       
	    end
	  MEM_LW:
	    begin
	       t_rsp_data = {{32{t_bswap_w32[31]}}, t_bswap_w32};
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end
	  MEM_LWU:
	    begin
	       t_rsp_data = {32'd0, t_bswap_w32};
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end
	  MEM_LD:
	    begin
	       t_rsp_data = t_w64;
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	    end	  
	  
	  MEM_SB:
	    begin
	       t_wr_array = r_got_req;
	       //if(t_wr_array)
	       //write_byte(r_req.addr, r_req.data[7:0],paging_active ? page_table_root : 64'd0);

	    end
	  MEM_SH:
	    begin
	       t_wr_array = r_got_req;	       
	       //if(t_wr_array)
	       //write_half(r_req.addr, bswap16(r_req.data[15:0]),paging_active ? page_table_root : 64'd0);
	    end
	  MEM_SW:
	    begin
	       t_wr_array = r_got_req;	       
	       // if(t_wr_array)
	       // 	 begin
	       // 	    write_word(r_req.addr, bswap32(r_req.data[31:0]),paging_active ? page_table_root : 64'd0, 32'd0);
	       // 	 end
	    end
	  MEM_SD:
	    begin
	       t_wr_array = r_got_req;	       
	       //if(t_wr_array)
		// write_dword(r_req.addr, r_req.data, paging_active ? page_table_root : 64'd0, 32'd0);	       	       
	    end
	  MEM_SCD:
	    begin
	       t_wr_array = r_got_req;
	       t_rsp_data = 'd0;
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       //if(t_wr_array & !(r_dead_atomic || w_dead_atomic))
		 //write_dword(r_req.addr, r_req.data, paging_active ? page_table_root : 64'd0, 32'd1);
	    end
	  MEM_SCW:
	    begin
	       t_wr_array = r_got_req;
	       t_rsp_data = 'd0;
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       //if(t_wr_array & !(r_dead_atomic|w_dead_atomic))
		 //begin
		   // write_word(r_req.addr, r_req.data[31:0], paging_active ? page_table_root : 64'd0, 32'd1);
		 //end
	       
	    end
	  MEM_AMOW:
	    begin
	       //return old data
	       t_rsp_data = {{32{t_w32[31]}}, t_w32};
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       
	       t_wr_array = r_got_req;
	       //if(t_wr_array & !(r_dead_atomic||w_dead_atomic))
	       //begin
	       // write_word(r_req.addr, t_amo32_data, paging_active ? page_table_root : 64'd0, 32'd2);
	       //end
	    end // case: MEM_AMOW
	  MEM_AMOD:
	    begin
	       t_rsp_data = t_w64;
	       t_rsp_dst_valid = r_req.dst_valid & t_hit_cache;
	       
	       t_wr_array = r_got_req;
	       //if(t_wr_array  & !(r_dead_atomic||w_dead_atomic))
	       //begin
	       // write_dword(r_req.addr, t_amo64_data, paging_active ? page_table_root : 64'd0, 32'd2);
	       //end
	    end // case: MEM_AMOD
	  MEM_NOP:
	    begin
	    end
	  default:
	    begin
	       $display("opcode %d, pc %x", r_req.op, r_req.pc);
	       $stop();
	    end
	endcase // case r_req.op
     end


	         
   always_comb
     begin
	t_got_rd_retry = 1'b0;
	t_port2_hit_cache = r_got_req2;
	
	n_state = r_state;
	t_miss_idx = r_miss_idx;
	t_miss_addr = r_miss_addr;
	t_cache_idx = 'd0;
	t_cache_tag = 'd0;
	
	t_cache_idx2 = 'd0;
	t_cache_tag2 = 'd0;	
	
	t_got_req = 1'b0;
	t_got_req2 = 1'b0;
	
	n_last_wr = 1'b0;
	n_last_rd = 1'b0;
	n_last_wr2 = 1'b0;
	n_last_rd2 = 1'b0;
	
	t_got_miss = 1'b0;
	t_push_miss = 1'b0;
	
	n_req = r_req;
	n_req2 = r_req2;
	
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
	n_core_mem_rsp.has_cause = 1'b0;
	n_core_mem_rsp.cause = 'd0;
	n_core_mem_rsp.dst_valid = 1'b0;

	n_cache_accesses = r_cache_accesses;
	n_cache_hits = r_cache_hits;
	
	n_store_stalls = r_store_stalls;
	
	n_flush_req = r_flush_req | flush_req;
	n_flush_cl_req = r_flush_cl_req | flush_cl_req;
	n_flush_complete = 1'b0;
	t_addr = 'd0;
	
	n_is_retry = 1'b0;
	t_reset_graduated = 1'b0;
	t_force_clear_busy = 1'b0;
	
	t_incr_busy = 1'b0;
	
	
	t_mh_block = r_got_req && r_last_wr && 
		     (r_cache_idx == t_mem_head.addr[IDX_STOP-1:IDX_START] );
	
	t_cm_block = r_got_req && r_last_wr && 
		     (r_cache_idx == core_mem_va_req.addr[IDX_STOP-1:IDX_START]) &&
		     (r_cache_tag == core_mem_va_req.addr[`M_WIDTH-1:IDX_STOP]);


	t_cm_block_stall = t_cm_block && !(r_is_retry);//1'b0;
	n_dead_atomic = 1'b0;
	
	case(r_state)
	  INITIALIZE:
	    begin
	       n_state = ACTIVE;
	       n_flush_complete = 1'b1;
	    end
	  ACTIVE:
	    begin
	       if(r_got_req2)
		 begin
		    n_core_mem_rsp.data = r_req2.addr;
		    n_core_mem_rsp.addr = r_req2.addr;
		    n_core_mem_rsp.rob_ptr = r_req2.rob_ptr;
		    n_core_mem_rsp.dst_ptr = r_req2.dst_ptr;
		    
		    if(drain_ds_complete)
		      begin		
			 n_core_mem_rsp.dst_valid = r_req2.dst_valid;
			 n_core_mem_rsp_valid = 1'b1;
		      end
		    else if(r_req2.is_atomic)
		      begin
			 //$display("accept atomic for pc %x, rob pointer %d, cycle %d", r_req2.pc, r_req2.rob_ptr, r_cycle);
			 if(t_pf2)
			   begin
			      //$display("using resp port for atomic page fault at cycle %d", r_cycle);
			      n_core_mem_rsp.dst_valid = 1'b1;
			      n_core_mem_rsp.has_cause = t_pf2;
			      n_core_mem_rsp.cause = STORE_PAGE_FAULT;
                              n_core_mem_rsp_valid = 1'b1;			      
			   end
			 else
			   begin
			      t_push_miss = 1'b1;
			   end
		      end
		    else if(r_req2.is_store)
		      begin
			 t_push_miss = 1'b1;
			 t_incr_busy = 1'b1;
			 //ack early
			 n_core_mem_rsp.dst_valid = 1'b0;
			 n_core_mem_rsp.has_cause = t_pf2;
			 n_core_mem_rsp.cause = STORE_PAGE_FAULT;
			 
			 if(t_port2_hit_cache)
			   begin
			      n_cache_hits = r_cache_hits + 'd1;
			   end
			 n_core_mem_rsp_valid = 1'b1;
		      end // if (r_req2.is_store)
		    else if(t_port2_hit_cache && !r_hit_busy_addr2)
		      begin
`ifdef VERBOSE_L1D
 			 $display("cycle %d port2 hit for pc %d, addr %x, data %x", r_cycle, r_req2.pc, r_req2.addr, t_rsp_data2);
`endif
			 n_core_mem_rsp.data = t_rsp_data2[63:0];
                         n_core_mem_rsp.dst_valid = t_rsp_dst_valid2;
			 n_core_mem_rsp.has_cause = t_pf2;
			 n_core_mem_rsp.cause = LOAD_PAGE_FAULT;
                         n_cache_hits = r_cache_hits + 'd1;
                         n_core_mem_rsp_valid = 1'b1;
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
		    if(r_valid_out /*&& (r_tag_out == r_cache_tag)*/)
		      begin /* valid cacheline - hit in cache */
			 if(r_req.is_store)
			   begin
			      t_reset_graduated = 1'b1;				   
			   end
			 else
			   begin
			      n_core_mem_rsp.data = t_rsp_data[63:0];
			      n_core_mem_rsp.has_cause = t_pf;
			      n_core_mem_rsp.cause = LOAD_PAGE_FAULT;
			      n_core_mem_rsp.dst_valid = t_rsp_dst_valid;
			      n_core_mem_rsp_valid = 1'b1;
			   end // else: !if(r_req.is_store)
		      end // if (r_valid_out && (r_tag_out == r_cache_tag))
		 end // if (r_got_req)

	       
	     if(!mem_q_empty)
	       begin
		   // $display("t_mem_head.rob_ptr = %d, grad %b, dq ptr %d valid %b, atomic %b, r_cycle %d", 
		   // 	    t_mem_head.rob_ptr,
		   // 	    r_graduated[t_mem_head.rob_ptr], 
		   // 	    core_store_data.rob_ptr, 
		   // 	    core_store_data_valid,
		   // 	    t_mem_head.is_atomic,
		   // 	    r_cycle);
		  
		  if(!t_mh_block)
		    begin
		       if(t_mem_head.is_store)
			 begin
			    
			    if(r_graduated[t_mem_head.rob_ptr] == 2'b10 && (core_store_data_valid ? (t_mem_head.rob_ptr == core_store_data.rob_ptr) : 1'b0) )
			      begin
				 //$display("firing store for %x with data %x at cycle %d for rob ptr %d", 
				//	  t_mem_head.addr, t_mem_head.data, r_cycle, t_mem_head.rob_ptr);
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
			      end // 
			    else if(drain_ds_complete && dead_rob_mask[t_mem_head.rob_ptr])
			      begin
				 t_pop_mq = 1'b1;
				 t_force_clear_busy = 1'b1;
			      end
			 end // if (t_mem_head.is_store)
		       else if(t_mem_head.is_atomic)
			 begin
			    if (t_mem_head.rob_ptr == head_of_rob_ptr && (core_store_data_valid ? (t_mem_head.rob_ptr == core_store_data.rob_ptr) : 1'b0))
			      begin
				 //$display("firing atomic for %x at cycle %d for rob ptr %d", 
				//	  t_mem_head.pc, r_cycle, t_mem_head.rob_ptr);	
				 t_pop_mq = 1'b1;
				 core_store_data_ack = 1'b1;
				 n_req = t_mem_head;
				 n_req.data = core_store_data.data;
				 t_cache_idx = t_mem_head.addr[IDX_STOP-1:IDX_START];
				 t_cache_tag = t_mem_head.addr[`M_WIDTH-1:IDX_STOP];
				 t_addr = t_mem_head.addr;
				 t_got_rd_retry = 1'b1;				 
				 t_got_req = 1'b1;
				 n_is_retry = 1'b1;
				 n_last_wr = 1'b1;
			      end // if (t_mem_head.rob_ptr == head_of_rob_ptr && (core_store_data_valid ? (t_mem_head.rob_ptr == core_store_data.rob_ptr) : 1'b0))
			    else if(drain_ds_complete && dead_rob_mask[t_mem_head.rob_ptr])
			      begin
				 t_pop_mq = 1'b1;
				 n_dead_atomic = 1'b1;
				 //t_force_clear_busy = 1'b1;
				 core_store_data_ack = 1'b1;
				 n_req = t_mem_head;
				 n_req.data = core_store_data.data;
				 t_cache_idx = t_mem_head.addr[IDX_STOP-1:IDX_START];
				 t_cache_tag = t_mem_head.addr[`M_WIDTH-1:IDX_STOP];
				 t_addr = t_mem_head.addr;
				 t_got_rd_retry = 1'b1;				 
				 t_got_req = 1'b1;
				 n_is_retry = 1'b1;
				 n_last_wr = 1'b1;				 
			      end			    
			 end
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
			 end
		    end
	       end
	     
	       if(core_mem_va_req_valid &&
		  !t_got_miss && 
		  !(mem_q_almost_full||mem_q_full) && 
		  !t_got_rd_retry &&
		  !(r_last_wr2 && (r_cache_idx2 == core_mem_va_req.addr[IDX_STOP-1:IDX_START]) && !core_mem_va_req.is_store) && 
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

`ifdef VERBOSE_L1D		       
		  $display("accepting new op %d, pc %x, addr %x for rob ptr %d at cycle %d, mem_q_empty %b", 
			   core_mem_req.op, core_mem_req.pc, core_mem_req.addr,
			   core_mem_req.rob_ptr, r_cycle, mem_q_empty);
`endif
		  
		  n_last_wr2 = core_mem_req.is_store;
		  n_last_rd2 = !core_mem_req.is_store;
		  
		  n_cache_accesses =  r_cache_accesses + 'd1;
	       end // if (core_mem_req_valid &&...
	       else if(r_flush_req && mem_q_empty && !(r_got_req && r_last_wr))
		 begin
		    n_state = ACTIVE;
		    t_cache_idx = 'd0;
		    n_flush_req = 1'b0;
		    n_flush_complete = 1'b1;
		 end
	       else if(r_flush_cl_req && mem_q_empty && !(r_got_req && r_last_wr))
		 begin
		    $stop();
		 end
	    end // case: ACTIVE
	  
	  default:
	    begin
	    end
	endcase // case r_state
     end // always_comb


`endif
   
endmodule // l1d

