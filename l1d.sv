`include "machine.vh"
`include "rob.vh"
`include "uop.vh"


module l1d(clk, 
	   reset,
	   head_of_rob_ptr,
	   flush_req,
	   flush_complete,
	   flush_cl_req,
	   flush_cl_addr,
	   //inputs from core
	   core_mem_req_valid,
	   core_mem_req,
	   //outputs to core
	   core_mem_req_ack,
	   core_mem_rsp,
	   core_mem_rsp_valid,
	   //output to the memory system
	   mem_req_ack,
	   mem_req_valid, 
	   mem_req_addr, 
	   mem_req_store_data, 
	   mem_req_tag,
	   mem_req_opcode,
	   //reply from memory system
	   mem_rsp_valid,
	   mem_rsp_load_data,
	   mem_rsp_tag,
	   mem_rsp_opcode,
	   utlb_miss_req,
	   utlb_miss_paddr,
	   tlb_rsp_valid,
	   tlb_rsp,
	   cache_accesses,
	   cache_hits
	   );

   localparam L1D_NUM_SETS = 1 << `LG_L1D_NUM_SETS;
   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);
   
   input logic clk;
   input logic reset;
   input logic [`LG_ROB_ENTRIES-1:0] head_of_rob_ptr;
   input logic flush_cl_req;
   input logic [`M_WIDTH-1:0] flush_cl_addr;
   input logic 		      flush_req;
   output logic 	      flush_complete;
   
   input logic core_mem_req_valid;
   input       mem_req_t core_mem_req;
   output logic core_mem_req_ack;
   output 	mem_rsp_t core_mem_rsp;
   output logic core_mem_rsp_valid;

   input logic 	mem_req_ack;
   
   output logic mem_req_valid;
   output logic [(`M_WIDTH-1):0] mem_req_addr;
   output logic [L1D_CL_LEN_BITS-1:0] mem_req_store_data;
   output logic [`LG_MEM_TAG_ENTRIES-1:0] mem_req_tag;
   output logic [4:0] 		      mem_req_opcode;

   input logic 				  mem_rsp_valid;
   input logic [L1D_CL_LEN_BITS-1:0] 	  mem_rsp_load_data;
   input logic [`LG_MEM_TAG_ENTRIES-1:0]  mem_rsp_tag;

   input logic [4:0] 			 mem_rsp_opcode;

   output logic 			 utlb_miss_req;
   output logic [`M_WIDTH-`LG_PG_SZ-1:0] utlb_miss_paddr;
   input logic 				 tlb_rsp_valid;
   input 				 utlb_entry_t tlb_rsp;
   
   output logic [63:0] 			 cache_accesses;
   output logic [63:0] 			 cache_hits;
         
   localparam LG_WORDS_PER_CL = `LG_L1D_CL_LEN - 2;
   localparam LG_DWORDS_PER_CL = `LG_L1D_CL_LEN - 3;
   
   localparam WORDS_PER_CL = 1<<(LG_WORDS_PER_CL);
   localparam N_TAG_BITS = `M_WIDTH - `LG_L1D_NUM_SETS - `LG_L1D_CL_LEN;
   localparam IDX_START = `LG_L1D_CL_LEN;
   localparam IDX_STOP  = `LG_L1D_CL_LEN + `LG_L1D_NUM_SETS;
   localparam WORD_START = 2;
   localparam WORD_STOP = WORD_START+LG_WORDS_PER_CL;
   localparam DWORD_START = 3;
   localparam DWORD_STOP = DWORD_START + LG_DWORDS_PER_CL;
  
   localparam N_MQ_ENTRIES = (1<<`LG_MQ_ENTRIES);

function logic [L1D_CL_LEN_BITS-1:0] merge_cl64(logic [L1D_CL_LEN_BITS-1:0] cl, logic [63:0] w64, logic pos);
   logic [L1D_CL_LEN_BITS-1:0] 		 cl_out;
   case(pos)
     1'b0:
       begin
	  cl_out = {cl[127:64], w64};
       end
     1'b1:
       begin
	  cl_out = {w64, cl[63:0]};
       end
   endcase // case (pos)
   return cl_out;
endfunction // merge_cl64
      
function logic [L1D_CL_LEN_BITS-1:0] merge_cl32(logic [L1D_CL_LEN_BITS-1:0] cl, logic [31:0] w32, logic[LG_WORDS_PER_CL-1:0] pos);
   logic [L1D_CL_LEN_BITS-1:0] 		 cl_out;
   case(pos)
     2'd0:
       cl_out = {cl[127:32], w32};
     2'd1:
       cl_out = {cl[127:64], w32, cl[31:0]};
     2'd2:
       cl_out = {cl[127:96], w32, cl[63:0]};
     2'd3:
       cl_out = {w32, cl[95:0]};
   endcase // case (pos)
   return cl_out;
endfunction

function logic [31:0] select_cl32(logic [L1D_CL_LEN_BITS-1:0] cl, logic[LG_WORDS_PER_CL-1:0] pos);
   logic [31:0] 			 w32;
   case(pos)
     2'd0:
       w32 = cl[31:0];
     2'd1:
       w32 = cl[63:32];
     2'd2:
       w32 = cl[95:64];
     2'd3:
       w32 = cl[127:96];
   endcase // case (pos)
   return w32;
endfunction
   
function logic [63:0] select_cl64(logic [L1D_CL_LEN_BITS-1:0] cl, logic pos);
   logic [63:0] 			 w64;
   case(pos)
     1'b0:
       w64 = cl[63:0];
     1'b1:
       w64 = cl[127:64];
   endcase // case (pos)
   return w64;
endfunction
   
   
   logic 				  r_got_req, r_last_wr, n_last_wr;
   logic 				  r_last_rd, n_last_rd;
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_cache_idx, r_cache_idx;
   
   logic [`LG_L1D_NUM_SETS-1:0] 	  t_array_wr_addr;
   logic [L1D_CL_LEN_BITS-1:0] 		  t_array_wr_data;
   logic 				  t_array_wr_en;
   
   logic [N_TAG_BITS-1:0] 		  t_cache_tag, r_cache_tag, r_tag_out;
   logic 				  r_valid_out, r_dirty_out;
   logic 				  r_flush_req, n_flush_req;
   logic 				  r_flush_cl_req, n_flush_cl_req;
   logic 				  r_flush_complete, n_flush_complete;
   
   logic [L1D_CL_LEN_BITS-1:0] 		  r_array_out;
   logic [31:0] 			  t_array_out_b32[WORDS_PER_CL-1:0];
   logic [31:0] 			  t_w32, t_bswap_w32;
   logic [63:0] 			  t_w64, t_bswap_w64;
      
   logic 				  t_mark_invalid;
   logic 				  t_wr_array;
   logic [L1D_CL_LEN_BITS-1:0] 		  t_array_data;
   
   logic [`M_WIDTH-1:0] 		  t_addr;
   logic 				  t_got_req;
   logic 				  t_got_miss;
   
   logic 				  n_core_mem_rsp_valid, r_core_mem_rsp_valid;
   mem_rsp_t n_core_mem_rsp, r_core_mem_rsp;
      
   mem_req_t n_req, r_req, t_req;

   mem_req_t r_mem_q[N_MQ_ENTRIES-1:0];
   mem_req_t n_mem_q[N_MQ_ENTRIES-1:0];
   logic [`LG_MQ_ENTRIES:0] r_mq_head_ptr, n_mq_head_ptr;
   logic [`LG_MQ_ENTRIES:0] r_mq_tail_ptr, n_mq_tail_ptr;
   logic [`LG_MQ_ENTRIES:0] t_mq_tail_ptr_plus_one;
   
   mem_req_t t_mem_tail, t_mem_head;
   logic 	mem_q_full, mem_q_empty, mem_q_almost_full;
   
`define WRITEBACK_TIMER 1
`ifdef WRITEBACK_TIMER
   logic [7:0] 	r_wb_timer, n_wb_timer;
`endif
   
   typedef enum logic [2:0] {ACTIVE = 'd0,
                            INJECT_RELOAD = 'd1,
                            INJECT_WRITEBACK = 'd2,
                            FLUSH_CACHE = 'd3,
			    FLUSH_CACHE_WAIT = 'd4,
			    FLUSH_CL = 'd5,
			    FLUSH_CL_WAIT = 'd6,
			    RELOAD_UTLB = 'd7
			    } state_t;

   state_t r_state, n_state;
   logic       t_pop_mq, t_non_speculative;
   
   logic r_mem_req_valid, n_mem_req_valid;
   logic [(`M_WIDTH-1):0] r_mem_req_addr, n_mem_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] r_mem_req_store_data, n_mem_req_store_data;
   
   logic [4:0] r_mem_req_opcode, n_mem_req_opcode;
   logic [63:0] 			 n_cache_accesses, r_cache_accesses;
   logic [63:0] 			 n_cache_hits, r_cache_hits;

   logic 				 t_utlb_hit;
   utlb_entry_t t_utlb_hit_entry;
   logic 				 n_utlb_miss_req, r_utlb_miss_req;
   logic [`M_WIDTH-`LG_PG_SZ-1:0] 	 n_utlb_miss_paddr, r_utlb_miss_paddr;
   
   logic [31:0] 			 r_cycle;
   assign flush_complete = r_flush_complete;
   assign mem_req_addr = r_mem_req_addr;
   assign mem_req_store_data = r_mem_req_store_data;
   assign mem_req_opcode = r_mem_req_opcode;
   assign mem_req_valid = r_mem_req_valid;

   //assign core_mem_rsp_valid = r_core_mem_rsp_valid;
   //assign core_mem_rsp = r_core_mem_rsp;

   assign core_mem_rsp_valid = n_core_mem_rsp_valid;
   assign core_mem_rsp = n_core_mem_rsp;
   
   assign cache_accesses = r_cache_accesses;
   assign cache_hits = r_cache_hits;

   assign utlb_miss_req = r_utlb_miss_req;
   assign utlb_miss_paddr = r_utlb_miss_paddr;
  
//   assign 

   utlb utlb0 (
	       .clk(clk),
	       .reset(reset),
	       .flush(n_flush_complete),
	       .req(t_got_req),
	       .addr(t_addr),
	       .tlb_rsp(tlb_rsp),
	       .tlb_rsp_valid(tlb_rsp_valid),
	       .hit(t_utlb_hit),
	       .hit_entry(t_utlb_hit_entry)
	       );

   
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end
   
   always_ff@(posedge clk)
     begin
	r_mem_q <= n_mem_q;
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
     end

   
   always_comb
     begin
	n_mq_head_ptr = r_mq_head_ptr;
	n_mq_tail_ptr = r_mq_tail_ptr;
	t_mq_tail_ptr_plus_one = r_mq_tail_ptr + 'd1;
	n_mem_q = r_mem_q;
	
	if(t_got_miss)
	  begin
	     n_mq_tail_ptr = r_mq_tail_ptr + 'd1;
	     n_mem_q[r_mq_tail_ptr[`LG_MQ_ENTRIES-1:0] ] = r_req;
	  end
	
	if(t_pop_mq)
	  begin
	     n_mq_head_ptr = r_mq_head_ptr + 'd1;
	  end
	
	t_mem_head = r_mem_q[r_mq_head_ptr[`LG_MQ_ENTRIES-1:0]];
	
	mem_q_empty = (r_mq_head_ptr == r_mq_tail_ptr);
	
	mem_q_full = (r_mq_head_ptr != r_mq_tail_ptr) &&
		     (r_mq_head_ptr[`LG_MQ_ENTRIES-1:0] == r_mq_tail_ptr[`LG_MQ_ENTRIES-1:0]);
	
	mem_q_almost_full = (r_mq_head_ptr != t_mq_tail_ptr_plus_one) &&
			    (r_mq_head_ptr[`LG_MQ_ENTRIES-1:0] == t_mq_tail_ptr_plus_one[`LG_MQ_ENTRIES-1:0]);
	
	
     end // always_comb

   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_flush_complete <= 1'b0;
	     r_flush_req <= 1'b0;
	     r_flush_cl_req <= 1'b0;
	     r_cache_idx <= 'd0;
	     r_cache_tag <= 'd0;
	     r_got_req <= 1'b0;
	     r_last_wr <= 1'b0;
	     r_last_rd <= 1'b0;
	     r_state <= ACTIVE;
	     r_mem_req_valid <= 1'b0;
	     r_mem_req_addr <= 'd0;
	     r_mem_req_store_data <= 'd0;
	     r_mem_req_opcode <= 'd0;
	     r_core_mem_rsp_valid <= 1'b0;
	     r_cache_hits <= 'd0;
	     r_cache_accesses <= 'd0;
	     r_utlb_miss_req <= 1'b0;
	     r_utlb_miss_paddr <= 'd0;
	  end
	else
	  begin
	     r_flush_complete <= n_flush_complete;
	     r_flush_req <= n_flush_req;
	     r_flush_cl_req <= n_flush_cl_req;
	     r_cache_idx <= t_cache_idx;
	     r_cache_tag <= t_cache_tag;
	     r_got_req <= t_got_req;
	     r_last_wr <= n_last_wr;
	     r_last_rd <= n_last_rd;
	     r_state <= n_state;
	     r_mem_req_valid <= n_mem_req_valid;
	     r_mem_req_addr <= n_mem_req_addr;
	     r_mem_req_store_data <= n_mem_req_store_data;
	     r_mem_req_opcode <= n_mem_req_opcode;
	     r_core_mem_rsp_valid <= n_core_mem_rsp_valid;
	     r_cache_hits <= n_cache_hits;
	     r_cache_accesses <= n_cache_accesses;
	     r_utlb_miss_req <= n_utlb_miss_req;
	     r_utlb_miss_paddr <= n_utlb_miss_paddr;
	  end
     end // always_ff@ (posedge clk)
   
   always_ff@(posedge clk)
     begin
	r_req <= n_req;
	r_core_mem_rsp <= n_core_mem_rsp;
     end

   always_comb
     begin
	t_array_wr_addr = mem_rsp_valid ? r_mem_req_addr[IDX_STOP-1:IDX_START] : r_cache_idx;
	t_array_wr_data = mem_rsp_valid ? mem_rsp_load_data : t_array_data;
	t_array_wr_en = mem_rsp_valid || t_wr_array;
     end

 ram1r1w #(.WIDTH(N_TAG_BITS), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_tag
     (
      .clk(clk),
      .rd_addr(t_cache_idx),
      .wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
      .wr_data(r_mem_req_addr[`M_WIDTH-1:IDX_STOP]),
      .wr_en(mem_rsp_valid),
      .rd_data(r_tag_out)
      );
     

   ram1r1w #(.WIDTH(L1D_CL_LEN_BITS), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_data
     (
      .clk(clk),
      .rd_addr(t_cache_idx),
      .wr_addr(t_array_wr_addr),
      .wr_data(t_array_wr_data),
      .wr_en(t_array_wr_en),
      .rd_data(r_array_out)
      );

   logic t_dirty_value;
   logic t_write_dirty_en;
   logic [`LG_L1D_NUM_SETS-1:0] t_dirty_wr_addr;
   
   always_comb
     begin
	t_dirty_value = 1'b0;
	t_write_dirty_en = 1'b0;
	t_dirty_wr_addr = r_cache_idx;
	if(mem_rsp_valid)
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
   
   ram1r1w #(.WIDTH(1), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_dirty
     (
      .clk(clk),
      .rd_addr(t_cache_idx),
      .wr_addr(t_dirty_wr_addr),
      .wr_data(t_dirty_value),
      .wr_en(t_write_dirty_en),
      .rd_data(r_dirty_out)
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
	     t_valid_value = !((r_state == FLUSH_CACHE_WAIT) || (r_state == FLUSH_CL_WAIT));
	     t_write_valid_en = 1'b1;
	  end
     end // always_comb
      
   ram1r1w #(.WIDTH(1), .LG_DEPTH(`LG_L1D_NUM_SETS)) dc_valid
     (
      .clk(clk),
      .rd_addr(t_cache_idx),
      .wr_addr(t_valid_wr_addr),
      .wr_data(t_valid_value),
      .wr_en(t_write_valid_en),
      .rd_data(r_valid_out)
      );

   

   generate
      genvar i;
      for(i = 0; i < WORDS_PER_CL; i=i+1)
	begin
	   assign t_array_out_b32[i] = bswap32(r_array_out[((i+1)*32)-1:i*32]);
	end
   endgenerate
`ifdef DEBUG
   always_ff@(negedge clk)
     begin
   	$display("state = %d, core_mem_req_valid = %b, op %d, head %d, rob ptr %d, dst ptr %d, rsp valid %b, rob rsp ptr %d",
   		 r_state,
   		 core_mem_req_valid,
   		 core_mem_req.op,
   		 head_of_rob_ptr, 
   		 core_mem_req.rob_ptr,
		 core_mem_req.dst_ptr,
   		 core_mem_rsp_valid,
   		 core_mem_rsp.rob_ptr
   		 );
     end
`endif
`ifdef WRITEBACK_TIMER
   always_ff@(posedge clk)
     begin
	r_wb_timer <= reset ? 'd0 : n_wb_timer;
     end
`endif

   // always_ff@(negedge clk)
   //   begin
   // 	if(core_mem_req_valid)
   // 	  begin
   // 	     $display("cycle %d : core_mem_req.dst_ptr = %d, core_mem_req.op = %d, core_mem_req_ack = %b",
   // 		      r_cycle, core_mem_req.dst_ptr, core_mem_req.op, core_mem_req_ack);
	     
   // 	  end
   // 	if(r_got_req)
   // 	  begin
   // 	     $display("cycle %d : r_req.dst_ptr = %d, r_req.op = %d, r_state = %d, hit %b",
   // 		      r_cycle, r_req.dst_ptr, r_req.op, r_state, r_valid_out && (r_tag_out == r_cache_tag));
   // 	  end
   //   end
   
   always_comb
     begin
	t_cache_idx = 'd0;
	t_cache_tag = 'd0;
	t_got_req = 1'b0;
	n_last_wr = 1'b0;
	n_last_rd = 1'b0;
	t_got_miss = 1'b0;
	n_utlb_miss_req = 1'b0;
	n_utlb_miss_paddr = r_utlb_miss_paddr;	
	n_req = r_req;
	core_mem_req_ack = 1'b0;
	n_state = r_state;
	n_mem_req_valid = 1'b0;
	n_mem_req_addr = r_mem_req_addr;
	n_mem_req_store_data = r_mem_req_store_data;
	n_mem_req_opcode = r_mem_req_opcode;
	t_pop_mq = 1'b0;
	n_core_mem_rsp_valid = 1'b0;
	n_core_mem_rsp.op = r_req.op;
	n_core_mem_rsp.data = r_req.addr;
	n_core_mem_rsp.rob_ptr = r_req.rob_ptr;
	n_core_mem_rsp.dst_ptr = r_req.dst_ptr;
	n_core_mem_rsp.dst_valid = 1'b0;
	n_core_mem_rsp.fp_dst_valid = 1'b0;
	n_core_mem_rsp.exception_tlb_refill = 1'b0;
	n_core_mem_rsp.exception_tlb_modified = 1'b0;
	n_core_mem_rsp.exception_tlb_invalid = 1'b0;
	n_core_mem_rsp.faulted = 1'b0;
	
`ifdef WRITEBACK_TIMER
	n_wb_timer = r_wb_timer;
`endif
	n_cache_accesses = r_cache_accesses;
	n_cache_hits = r_cache_hits;
	
	n_flush_req = r_flush_req | flush_req;
	n_flush_cl_req = r_flush_cl_req | flush_cl_req;
	n_flush_complete = 1'b0;
	t_addr = 'd0;
	t_wr_array = 1'b0;
	t_array_data = 'd0;
	t_mark_invalid = 1'b0;
	
	t_non_speculative = (core_mem_req.op == MEM_DEAD_LD || core_mem_req.op == MEM_DEAD_ST) ? 1'b1 : (core_mem_req.rob_ptr == head_of_rob_ptr);
		  
	t_w32 = (select_cl32(r_array_out, r_req.addr[WORD_STOP-1:WORD_START]));
	t_w64 = select_cl64(r_array_out, r_req.addr[DWORD_START]);
	t_bswap_w32 = bswap32(t_w32);
	t_bswap_w64 = bswap64(t_w64);
	
	case(r_state)
	  ACTIVE:
	    begin
	       if(r_got_req)
		 begin
		    
		    if(r_req.op == MEM_DEAD_ST)
		      begin /* dead op */
			 n_core_mem_rsp_valid = 1'b1;
		      end
		    else if(r_req.op == MEM_DEAD_LD)
		      begin
			 n_core_mem_rsp.dst_valid = r_req.dst_valid;
			 n_core_mem_rsp_valid = 1'b1;
		      end
		    else if(r_req.op == MEM_DEAD_SC)
		      begin
			 n_core_mem_rsp.dst_valid = r_req.dst_valid;
			 n_core_mem_rsp_valid = 1'b1;
		      end
		    else if(r_req.op == MEM_MTC1_MERGE)
		      begin
			 if(r_req.lwc1_lo)
			   begin
			      n_core_mem_rsp.data = {r_req.addr[31:0], r_req.data[31:0]};
			   end
			 else
			   begin
			      n_core_mem_rsp.data = {r_req.data[63:32], r_req.addr[31:0]};
			   end
			 //$display("writing lo %x, hi %x for dst prf %d, old %x, gpr %x, sel %b", 
			 //n_core_mem_rsp.data[31:0], 
			 //n_core_mem_rsp.data[63:32], 
			 //r_req.dst_ptr, r_req.data, 
			 //	  r_req.addr[31:0], 
			 //	  r_req.lwc1_lo);
			 n_core_mem_rsp.fp_dst_valid = r_req.dst_valid;
			 n_core_mem_rsp_valid = 1'b1;			 
		      end // if (r_req.op == MEM_MTC1_MERGE)
		    else if(r_req.op == MEM_MFC1_MERGE)
		      begin
			 //$display("source fp lo reg %x, hi reg %x, select %b", 
			 //r_req.data[31:0], r_req.data[63:32], r_req.lwc1_lo);
			 if(r_req.lwc1_lo)
			   begin
			      n_core_mem_rsp.data = {32'd0, r_req.data[63:32]};
			   end
			 else
			   begin
			      n_core_mem_rsp.data = {32'd0, r_req.data[31:0]};			      
			   end
			 n_core_mem_rsp.dst_valid = r_req.dst_valid;
			 n_core_mem_rsp_valid = 1'b1;
		      end // if (r_req.op == MEM_MFC1_MERGE)
		    else if(t_utlb_hit && t_utlb_hit_entry.bogus)
		      begin
			 //macro TLB miss - TLB refill exception
			 n_core_mem_rsp.exception_tlb_refill =  1'b1;
			 n_core_mem_rsp.faulted = 1'b1;
			 n_core_mem_rsp_valid = 1'b1;
		      end
		    else if(!t_utlb_hit)
		      begin
			 t_got_miss = 1'b1;
			 n_utlb_miss_req = 1'b1;
			 n_utlb_miss_paddr = r_req.addr[`M_WIDTH-1:`LG_PG_SZ];
			 //$display("at cycle %d, utlb miss for %x", r_cycle, r_req.addr);
			 n_state = RELOAD_UTLB;
		      end
		    else if(r_valid_out && (r_tag_out == r_cache_tag))
		      begin /* valid cacheline */
			 n_cache_hits = r_cache_hits + 'd1;
			 //$display("hit on %x, victory : out %x, opcode %d", 
			 //r_req.addr, r_array_out, r_req.op);
			 case(r_req.op)
			   MEM_LB:
			     begin
				case(r_req.addr[1:0])
				  2'd0:
				    begin
				       n_core_mem_rsp.data = {{56{t_w32[7]}}, t_w32[7:0]};
				    end
				  2'd1:
				    begin
				       n_core_mem_rsp.data = {{56{t_w32[15]}}, t_w32[15:8]};
				    end
				  2'd2:
				    begin
				       n_core_mem_rsp.data = {{56{t_w32[23]}}, t_w32[23:16]};
				  end
				  2'd3:
				    begin
				       n_core_mem_rsp.data = {{56{t_w32[31]}}, t_w32[31:24]};
				    end
				endcase
				n_core_mem_rsp.dst_valid = r_req.dst_valid;
			     end
			   MEM_LBU:
			     begin
				case(r_req.addr[1:0])
				  2'd0:
				    begin
				       n_core_mem_rsp.data = {56'd0, t_w32[7:0]};
				    end
				  2'd1:
				    begin
				       n_core_mem_rsp.data = {56'd0, t_w32[15:8]};
				    end
				  2'd2:
				    begin
				       n_core_mem_rsp.data = {56'd0, t_w32[23:16]};
				  end
				  2'd3:
				    begin
				       n_core_mem_rsp.data = {56'd0, t_w32[31:24]};
				    end
				endcase
				n_core_mem_rsp.dst_valid = r_req.dst_valid;
			   end
			 MEM_LH:
			   begin
			      case(r_req.addr[1])
			      	1'b0:
			      	  begin
			      	     n_core_mem_rsp.data = {{48{t_w32[7]}}, bswap16(t_w32[15:0])};
			      	  end
			      	1'b1:
			      	  begin
			      	     n_core_mem_rsp.data = {{48{t_w32[23]}}, bswap16(t_w32[31:16])};				     
			      	  end
			      	endcase
			      n_core_mem_rsp.dst_valid = r_req.dst_valid;
			   end
			 MEM_LHU:
			   begin
			      n_core_mem_rsp.data = {48'd0, bswap16(r_req.addr[1] ? t_w32[31:16] : t_w32[15:0])};
			      n_core_mem_rsp.dst_valid = r_req.dst_valid;
			   end
			   MEM_LW:
			     begin
				n_core_mem_rsp.data = {{32{t_bswap_w32[31]}}, t_bswap_w32};
				//$display("cl out = %x, sel %d, selected word = %x", 
				//r_array_out, 
				//r_req.addr[WORD_STOP-1:WORD_START], 
				//n_core_mem_rsp.data);
				//$stop();
				n_core_mem_rsp.dst_valid = r_req.dst_valid;
			     end
			   MEM_LWC1_MERGE:
			      begin
			    	if(r_req.lwc1_lo)
			    	  begin
			   	     n_core_mem_rsp.data = {t_bswap_w32, r_req.data[31:0]};
			   	  end
			   	else
			   	  begin
				     n_core_mem_rsp.data = {r_req.data[63:32], t_bswap_w32};
			   	  end
				 // $display("lwc1 data %x for prf %d at cycle %d",
				 // 	  n_core_mem_rsp.data,
				 // 	  n_core_mem_rsp.dst_ptr, 
				 // 	  r_cycle);
			   	 n_core_mem_rsp.fp_dst_valid = r_req.dst_valid;
			      end 
			 MEM_LDC1:
			   begin
			      n_core_mem_rsp.data = t_bswap_w64;
			      n_core_mem_rsp.fp_dst_valid = r_req.dst_valid;
			      //$display("LSU : LDC1 for rob slot %d, prf %d at cycle %d, dst_valid %b", r_req.rob_ptr, r_req.dst_ptr, 
			      //r_cycle, n_core_mem_rsp.fp_dst_valid);			      
			   end
			 MEM_SB:
			   begin
			      case(r_req.addr[1:0])
			      	2'd0:
			      	  begin
				     t_array_data = merge_cl32(r_array_out, {t_w32[31:8], r_req.data[7:0]}, r_req.addr[WORD_STOP-1:WORD_START]);
				  end
			      	2'd1:
				  begin
				     t_array_data = merge_cl32(r_array_out, {t_w32[31:16], r_req.data[7:0], t_w32[7:0]}, r_req.addr[WORD_STOP-1:WORD_START]);				     				     
			      	  end
				2'd2:
			      	  begin
				     t_array_data = merge_cl32(r_array_out, {t_w32[31:24], r_req.data[7:0], t_w32[15:0]}, r_req.addr[WORD_STOP-1:WORD_START]);				     
				  end
				2'd3:
				  begin
				     t_array_data = merge_cl32(r_array_out, {r_req.data[7:0], t_w32[23:0]}, r_req.addr[WORD_STOP-1:WORD_START]);
				  end
			      endcase // case (r_req.addr[1:0])

			      t_wr_array = 1'b1;			      
			   end
			 MEM_SH:
			   begin
			      case(r_req.addr[1])
			      	1'b0:
			      	  begin
			      	     t_array_data = merge_cl32(r_array_out, {t_w32[31:16], bswap16(r_req.data[15:0])}, r_req.addr[WORD_STOP-1:WORD_START]);
			      	  end
			      	1'b1:
			      	  begin
			      	     t_array_data = merge_cl32(r_array_out, {bswap16(r_req.data[15:0]), t_w32[15:0]}, r_req.addr[WORD_STOP-1:WORD_START]);				     
			      	  end
			      endcase
			      t_wr_array = 1'b1;			      
			   end
			 MEM_SW:
			   begin
			      t_array_data = merge_cl32(r_array_out, bswap32(r_req.data[31:0]), r_req.addr[WORD_STOP-1:WORD_START]);
			      //t_array_data[32*(r_req.addr[WORD_STOP-1:WORD_START]-1)-1:32*r_req.addr[WORD_STOP-1:WORD_START]] = bswap32(r_req.data);
			      //;
			      //$display("word select = %d, addr = %x, write data %x, write cache line %x", 
			      //r_req.addr[WORD_STOP-1:WORD_START], r_req.addr, r_req.data, t_array_data);
			      //$stop();
			      t_wr_array = 1'b1;
			   end
			 MEM_SWC1_MERGE:
			   begin
			      t_array_data = merge_cl32(r_array_out, bswap32(r_req.lwc1_lo ? r_req.data[63:32] : r_req.data[31:0]), r_req.addr[WORD_STOP-1:WORD_START]);
			      //$display("swc1 data %x for prf %d at cycle %d", r_req.data, n_core_mem_rsp.dst_ptr, r_cycle);
			      t_wr_array = 1'b1;
			   end
			 MEM_SDC1:
			   begin
			      //$display("SDC for rob slot %d", n_core_mem_rsp.rob_ptr);
			      t_array_data = merge_cl64(r_array_out, bswap64(r_req.data[63:0]), r_req.addr[DWORD_START]);
			      t_wr_array = 1'b1;
			   end
			 MEM_SC:
			   begin
			      t_array_data = merge_cl32(r_array_out, bswap32(r_req.data[31:0]), r_req.addr[WORD_STOP-1:WORD_START]);
			      n_core_mem_rsp.data = 64'd1;
			      n_core_mem_rsp.dst_valid = r_req.dst_valid;
			      t_wr_array = 1'b1;
			   end
			   MEM_SWR:
			     begin
				case(r_req.addr[1:0])
				  2'd0:
				    begin
				       t_array_data = merge_cl32(r_array_out, bswap32({r_req.data[7:0], t_bswap_w32[23:0]}), r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				  2'd1:
				    begin
				       t_array_data = merge_cl32(r_array_out, bswap32({r_req.data[15:0], t_bswap_w32[15:0]}), r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				  2'd2:
				    begin
				       t_array_data = merge_cl32(r_array_out, bswap32({r_req.data[23:0], t_bswap_w32[7:0]}), r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				  2'd3:
				    begin
				       t_array_data = merge_cl32(r_array_out, bswap32(r_req.data[31:0]), r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				endcase // case (r_req.addr[1:0])
				t_wr_array = 1'b1;
			     end
			   MEM_SWL:
			     begin
				case(r_req.addr[1:0])
				  2'd0:
				    begin
				       t_array_data = merge_cl32(r_array_out, t_bswap_w32, r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				  2'd1:
				    begin
				       t_array_data = merge_cl32(r_array_out, bswap32({t_bswap_w32[31:24], r_req.data[31:8]}), r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				  2'd2:
				    begin
				       t_array_data = merge_cl32(r_array_out, bswap32({t_bswap_w32[31:16], r_req.data[31:16]}), r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				  2'd3:
				    begin
				       t_array_data = merge_cl32(r_array_out, bswap32({t_bswap_w32[31:8], r_req.data[31:24]}), r_req.addr[WORD_STOP-1:WORD_START]);
				    end
				endcase // case (r_req.addr[1:0])
				t_wr_array = 1'b1;
			     end
			   MEM_LWR:
			     begin
				case(r_req.addr[1:0])
				  2'd0:
				    begin
				       n_core_mem_rsp.data = {{32{r_req.data[31]}}, r_req.data[31:8], t_bswap_w32[31:24]};
				    end
				  2'd1:
				    begin
				       n_core_mem_rsp.data = {{32{r_req.data[31]}}, r_req.data[31:16], t_bswap_w32[31:16]};
				    end
				  2'd2:
				    begin
				       n_core_mem_rsp.data = {{32{r_req.data[31]}}, r_req.data[31:24], t_bswap_w32[31:8]};				       
				    end
				  2'd3:
				    begin
				       n_core_mem_rsp.data = {{32{t_bswap_w32[31]}}, t_bswap_w32};
				    end
				endcase // case (r_req.addr[1:0])
				//$display("lwr case %d", r_req.addr[1:0]);
				n_core_mem_rsp.dst_valid = r_req.dst_valid;
			     end
			   MEM_LWL:
			     begin
				//_core_mem_rsp.data = t_bswap_w32;
				case(r_req.addr[1:0])
				  2'd0:
				    begin
				       n_core_mem_rsp.data = {{32{t_bswap_w32[31]}}, t_bswap_w32};
				    end
				  2'd1:
				    begin
				       n_core_mem_rsp.data = {{32{t_bswap_w32[23]}}, t_bswap_w32[23:0], r_req.data[7:0]};
				    end
				  2'd2:
				    begin
				       n_core_mem_rsp.data = {{32{t_bswap_w32[15]}}, t_bswap_w32[15:0], r_req.data[15:0]};
				    end
				  2'd3:
				    begin
				       n_core_mem_rsp.data = {{32{t_bswap_w32[7]}}, t_bswap_w32[7:0], r_req.data[23:0]};
				    end
				endcase // case (r_req.addr[1:0])
				n_core_mem_rsp.dst_valid = r_req.dst_valid;				
			     end // case: MEM_LWL
			 default:
			   begin
			      $stop();
			   end
		       endcase // case r_req.op
		       n_core_mem_rsp_valid = 1'b1;
		      end // if (r_valid_out && (r_tag_out == r_cache_tag))
		  else if(r_valid_out && r_dirty_out && (r_tag_out != r_cache_tag))
		    begin
		       t_got_miss = 1'b1;
		       n_mem_req_addr = {r_tag_out,r_cache_idx,{`LG_L1D_CL_LEN{1'b0}}};
		       //$display("%x conflicts with %x", r_req.addr, n_mem_req_addr);
		       //r_req.addr;
		       n_mem_req_opcode = MEM_SW;
		       n_mem_req_store_data = r_array_out;
		       n_state = INJECT_WRITEBACK;
`ifdef WRITEBACK_TIMER
		       n_wb_timer = 'd0;
`endif		       
		       n_mem_req_valid = 1'b1;	       

		    end
		  else
		    begin
		       //$display("invalid line miss at cycle %d for %x (line %d), valid = %b, read tag = %x, expected tag = %x",
		       //		r_cycle, r_req.addr, r_cache_idx, r_valid_out, r_cache_tag, r_tag_out);
		       t_got_miss = 1'b1;
		       /* todo : make parameterized */
		       n_mem_req_addr = {r_req.addr[`M_WIDTH-1:`LG_L1D_CL_LEN], {`LG_L1D_CL_LEN{1'b0}}};
		       n_mem_req_opcode = MEM_LW;
		       n_state = INJECT_RELOAD;
		       n_mem_req_valid = 1'b1;	       
		    end
	       end // if (r_got_req)
	     if(!mem_q_empty)
	       begin
		  if(!(r_got_req && (r_cache_idx == t_mem_head.addr[IDX_STOP-1:IDX_START])))
		    begin 		  
		       t_pop_mq = 1'b1;
		       n_req = t_mem_head;
		       t_cache_idx = t_mem_head.addr[IDX_STOP-1:IDX_START];
		       t_cache_tag = t_mem_head.addr[`M_WIDTH-1:IDX_STOP];
		       t_addr = t_mem_head.addr;
		       t_got_req = 1'b1;
		       n_cache_accesses = r_cache_accesses + 'd1;
		       n_last_wr = t_mem_head.is_store;
		       n_last_rd = !t_mem_head.is_store;
		       //$display("retry reload for address %x @ cycle %d", 
		       //	t_mem_head.addr, r_cycle);
		    end
	       end
	     else if(core_mem_req_valid && !t_got_miss && !mem_q_almost_full /*&& t_non_speculative*/)
	       begin
		  if(!(r_got_req && r_last_wr && (r_cache_idx == core_mem_req.addr[IDX_STOP-1:IDX_START])))
		    begin
		       t_cache_idx = core_mem_req.addr[IDX_STOP-1:IDX_START];
		       t_cache_tag = core_mem_req.addr[`M_WIDTH-1:IDX_STOP];
		       t_addr = t_mem_head.addr;
		       n_req = core_mem_req;
		       core_mem_req_ack = 1'b1;
		       t_got_req = 1'b1;
		       n_last_wr = core_mem_req.is_store;
		       n_last_rd = !core_mem_req.is_store;
		       n_cache_accesses = r_cache_accesses + 'd1;
		    end
		  //else
		  //begin
		       //$display("blocked on back to back write on cycle %d, r_last_rd = %b, r_last_wr = %b, r_cache_idx = %d, addr %x", 
		       //r_cycle, r_last_rd, r_last_wr,r_cache_idx,core_mem_req.addr);
		    //end
	       end // if (core_mem_req_valid && !t_got_miss && !mem_q_almost_full)
	     else if(r_flush_req)
	       begin
		  n_state = FLUSH_CACHE;
		  //$display("flush begins at cycle %d", r_cycle);
		  t_cache_idx = 'd0;
		  n_flush_req = 1'b0;
	       end
	     else if(r_flush_cl_req)
	       begin
		  t_cache_idx = flush_cl_addr[IDX_STOP-1:IDX_START];
		  //$display("flush addr %x, maps to cl %d at cycle", flush_cl_addr, t_cache_idx, r_cycle);
		  n_flush_cl_req = 1'b0;
		  n_state = FLUSH_CL;
	       end

	  end // case: r_state:...
	  INJECT_RELOAD:
	    begin
	       	if(mem_rsp_valid)
		  begin
		     n_state = ACTIVE;
		  end
	    end
	  INJECT_WRITEBACK:
	    begin
`ifdef WRITEBACK_TIMER
	       n_wb_timer = r_wb_timer + 'd1;
	       if(r_wb_timer == 'd255)
		 begin
		    $display("hit max writeback wait time");
		    $stop();
		 end
`endif
	       	if(mem_rsp_valid)
		  begin		     
		     n_state = ACTIVE;
		  end
	    end
	  FLUSH_CL:
	    begin
	       if(!r_dirty_out)
		 begin
		    n_state = ACTIVE;
		    t_mark_invalid = 1'b1;
		    n_flush_complete = 1'b1;
		 end
	       else
		 begin
		    n_mem_req_addr = {r_tag_out,r_cache_idx,{`LG_L1D_CL_LEN{1'b0}}};
	            n_mem_req_opcode = MEM_SW;
	            n_mem_req_store_data = r_array_out;
	            n_state = FLUSH_CL_WAIT;
	            n_mem_req_valid = 1'b1;	       
		 end
	    end // case: FLUSH_CL
	  FLUSH_CL_WAIT:
	    begin
	       	if(mem_rsp_valid)
		  begin
		     n_state = ACTIVE;		     
		     n_flush_complete = 1'b1;
		  end	       
	    end
	  FLUSH_CACHE:
	    begin
	       t_cache_idx = r_cache_idx + 'd1;
	       //$display("flushing %d : dirty %b, valid %b, data %x",  r_cache_idx, r_dirty_out, r_valid_out, r_array_out);
	       
	       if(r_cache_idx == (L1D_NUM_SETS-1))
		 begin
		    //$display("flush done at cycle %d", r_cycle);
		    n_state = ACTIVE;
		    n_flush_complete = 1'b1;
		 end
	       else
		 begin
		    if(!r_dirty_out)
		      begin
			 t_mark_invalid = 1'b1;
			 t_cache_idx = r_cache_idx + 'd1;
		      end
		    else
		      begin
			 n_mem_req_addr = {r_tag_out,r_cache_idx,{`LG_L1D_CL_LEN{1'b0}}};
	            n_mem_req_opcode = MEM_SW;
	            n_mem_req_store_data = r_array_out;
	            n_state = FLUSH_CACHE_WAIT;
	            n_mem_req_valid = 1'b1;	       
		 end // else: !if(r_valid_out && !r_dirty_out)
		 end // else: !if(r_cache_idx == (L1D_NUM_SETS-1))
	    end // case: FLUSH_CACHE
	  
	  FLUSH_CACHE_WAIT:
	    begin
	       t_cache_idx = r_cache_idx;
	       	if(mem_rsp_valid)
		  begin
		     n_state = FLUSH_CACHE;
		  end
	    end
	  RELOAD_UTLB:
	    begin
	       //$display("reload TLB for address %x @ cycle %d", 
	       //	t_mem_head.addr, r_cycle);
	       if(tlb_rsp_valid)
		 begin
		    n_state = ACTIVE;
		 end
	       //$stop();
	    end
	  default:
	    begin
	    end
	endcase // case r_state
     end // always_comb
 
endmodule // l1d

