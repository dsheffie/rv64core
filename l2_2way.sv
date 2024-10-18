`include "machine.vh"
`include "rob.vh"

`ifdef VERILATOR
import "DPI-C" function void l1_to_l2_queue_occupancy(int e);
import "DPI-C" function void record_l2_state(int s);
`endif

//`define VERBOSE_L2

module l2_2way(clk,
	  reset,
	  l2_state,
	  l1d_req_valid,
	  l1d_req,
	  l1d_rdy,     
	  l1i_req,
	  l1i_addr,
	  l1d_rsp_valid,
	  l1i_rsp_valid,
	  l1d_rsp_tag,
	  l1d_rsp_addr,
	  l1d_rsp_writeback,
	  l1i_flush_req,
	  l1d_flush_req,

	  l1i_flush_complete,
	  l1d_flush_complete,
	  
	  flush_complete,

	  //l1 -> l2
	  l1_mem_req_ack,

	  //l2 -> l1
	  l1_mem_load_data,

	  //l2 probe l1
	  l2_probe_addr,
	  l2_probe_val,
	  l2_probe_ack,
	  
	  //l2 -> mem
	  mem_req_valid, 
	  mem_req_addr,
	  mem_req_tag,
	  mem_req_store_data, 
	  mem_req_opcode,
	  
	  //mem -> l2
	  mem_rsp_valid,
	  mem_rsp_tag,
	  mem_rsp_load_data,
	  
	  //page walker signals
	  mmu_req_valid, 
	  mmu_req_addr, 
	  mmu_req_data,  
	  mmu_req_store,
	  mmu_rsp_valid,
	  mmu_rsp_data,	   

	  mem_mark_valid,
	  mem_mark_accessed,
	  mem_mark_dirty,
	  mem_mark_addr,
	  mem_mark_rsp_valid,	  
	  
	  cache_hits,
	  cache_accesses
	  
	  );

   input logic clk;
   input logic reset;
   output logic [3:0] l2_state;
   
   input logic l1d_req_valid;
   input       l1d_req_t l1d_req;
   
   output logic	l1d_rdy;
   input logic l1i_req;
   input logic [(`PA_WIDTH-1):0] l1i_addr;   
   
   output logic l1d_rsp_valid;
   output logic l1i_rsp_valid;
   output logic [`LG_MRQ_ENTRIES:0] l1d_rsp_tag;
   output logic [`PA_WIDTH-1:0]     l1d_rsp_addr;
   output logic			    l1d_rsp_writeback;
      
   input logic l1i_flush_req;
   input logic l1d_flush_req;
   input logic l1i_flush_complete;
   input logic l1d_flush_complete;
   
   output logic flush_complete;

   output logic l1_mem_req_ack;

   output logic 				  l2_probe_val;
   output logic [(`PA_WIDTH-1):0]		  l2_probe_addr;
   input logic 					  l2_probe_ack;
   
   output logic [(1 << (`LG_L1D_CL_LEN+3)) - 1 :0] l1_mem_load_data;
   
   output logic mem_req_valid;
   output logic [`PA_WIDTH-1:0] mem_req_addr;
   output logic [`LG_L2_REQ_TAGS-1:0] mem_req_tag;
   output logic [(1 << (`LG_L2_CL_LEN+3)) - 1 :0] mem_req_store_data;
   output logic [3:0] 	mem_req_opcode;
   
   input logic 		mem_rsp_valid;
   input logic [`LG_L2_REQ_TAGS-1:0] mem_rsp_tag;
   input logic [(1 << (`LG_L2_CL_LEN+3)) - 1 :0] mem_rsp_load_data;
   
   input logic					 mmu_req_valid;
   input logic [`PA_WIDTH-1:0]			 mmu_req_addr;
   input logic [63:0]				 mmu_req_data;
   input logic					 mmu_req_store;
   output logic					 mmu_rsp_valid;
   output logic [63:0]				 mmu_rsp_data;
   
   
   input logic					 mem_mark_valid;
   input logic					 mem_mark_accessed;
   input logic					 mem_mark_dirty;
   input logic [63:0]				 mem_mark_addr;
   output logic					 mem_mark_rsp_valid;
   
   output logic [63:0]				 cache_hits;
   output logic [63:0]			  cache_accesses;
   
   logic [63:0]				  r_mmu_rsp_data, n_mmu_rsp_data;
   logic				  r_mmu_rsp_valid, n_mmu_rsp_valid;
   logic				  n_mem_mark_rsp_valid, r_mem_mark_rsp_valid;
   
   
   
   assign mmu_rsp_valid = r_mmu_rsp_valid;
   assign mmu_rsp_data = r_mmu_rsp_data;
   assign mem_mark_rsp_valid = r_mem_mark_rsp_valid;
   
   localparam LG_L2_LINES = `LG_L2_NUM_SETS;
   localparam L2_LINES = 1<<LG_L2_LINES;
   
   localparam TAG_BITS = `PA_WIDTH - (LG_L2_LINES + `LG_L2_CL_LEN);

   logic 		t_wr_dirty0, t_wr_valid0;
   logic		t_wr_dirty1, t_wr_valid1;
   logic		t_wr_tag0, t_wr_tag1;
   
   logic		t_wr_d0, t_wr_d1;
   logic		t_valid, t_dirty;


   
   logic [LG_L2_LINES-1:0] t_idx, r_idx;
   logic [TAG_BITS-1:0]    n_tag, r_tag;

   logic [`PA_WIDTH-(`LG_L2_CL_LEN+1):0]	n_last_l1i_addr, r_last_l1i_addr;
   logic [`PA_WIDTH-(`LG_L2_CL_LEN+1):0]    n_last_l1d_addr, r_last_l1d_addr;
   logic 		   t_gnt_l1i, t_gnt_l1d;
   logic 		   r_l1i, r_l1d, n_l1i, n_l1d;
      

   logic		   r_wb1, n_wb1;
   logic [TAG_BITS-1:0]	   r_tag_wb1;
   logic [127:0]	   r_data_wb1;
   
   
   logic [`PA_WIDTH-1:0]    n_addr, r_addr;
   logic [`LG_L2_REQ_TAGS-1:0] n_rob_tag, r_rob_tag;
   
   logic [`PA_WIDTH-1:0]    n_wb_addr, r_wb_addr;
   logic		    n_need_wb, r_need_wb;
   
   logic [`PA_WIDTH-1:0]    n_saveaddr, r_saveaddr;
   
   logic [3:0] 		   n_opcode, r_opcode;

   logic 		   r_mem_req, n_mem_req;
   logic [3:0] 		   r_mem_opcode, n_mem_opcode;
   logic 		   r_req_ack, n_req_ack;
   
   logic 		   r_l1d_rsp_valid, n_l1d_rsp_valid;
   logic 		   r_l1i_rsp_valid, n_l1i_rsp_valid;
   logic [(1 << (`LG_L1D_CL_LEN+3)) - 1:0] 	   r_rsp_data, n_rsp_data;
   logic [(1 << (`LG_L1D_CL_LEN+3)) - 1:0] 	   r_store_data, n_store_data;
   logic [`LG_MRQ_ENTRIES:0] 			   r_l1d_rsp_tag, n_l1d_rsp_tag;
   
   
   typedef enum logic  {
			     WAIT_FOR_FLUSH,
			     WAIT_FOR_L1_FLUSH_DONE
			     } flush_state_t;

   logic 	r_need_l1i,n_need_l1i,r_need_l1d,n_need_l1d;
   logic 	t_l2_flush_req;
   
   flush_state_t n_flush_state, r_flush_state;

   typedef enum logic [2:0] {
			     WRITEBACK = 'd0,
			     FLUSH = 'd1,
			     MARK_PTE = 'd2,
			     MMU = 'd3,
			     L1D = 'd4,
			     L1I = 'd5
			     } req_t;

   req_t n_req_ty, r_req_ty;
   
   
   typedef enum 	logic [4:0] {
				     INITIALIZE = 'd0,
				     IDLE = 'd1,
				     CHECK_VALID_AND_TAG = 'd2,
				     CLEAN_RELOAD = 'd3, 
				     DIRTY_STORE = 'd4,
				     PREPARE_WRITEBACK = 'd5,
				     STORE_TURNAROUND, //5
				     WAIT_CLEAN_RELOAD, //6
				     WAIT_STORE_IDLE, //7
				     FLUSH_STORE, //8
				     FLUSH_STORE_WAY2, //9
				     FLUSH_WAIT,
				     FLUSH_TRIAGE,
				     UPDATE_PTE
				     } state_t;

   state_t n_state, r_state;
   logic r_got_req, n_got_req;
   
   assign l2_state = 4'd0;
   
   logic 		n_flush_complete, r_flush_complete;
   logic 		r_flush_req, n_flush_req;
   logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0] r_mem_req_store_data, n_mem_req_store_data;
   logic [63:0] 	r_cache_hits, n_cache_hits, r_cache_accesses, n_cache_accesses;
   logic		r_replace, n_replace;
   
   
   assign flush_complete = r_flush_complete;
   assign mem_req_addr = r_addr;
   assign mem_req_tag = r_rob_tag;
   assign mem_req_valid = r_mem_req;
   assign mem_req_opcode = r_mem_opcode;
   assign mem_req_store_data = r_mem_req_store_data;

   // always_ff@(negedge clk)
   //   begin
   // 	if(mem_req_valid)
   // 	  begin
   // 	     $display("mem op %d at addr %x cycle %d",
   // 		      mem_req_opcode,
   // 		      mem_req_addr,
   // 		      r_cycle);
   // 	  end
   //   end

   assign l1d_rsp_valid = r_l1d_rsp_valid;
   assign l1i_rsp_valid = r_l1i_rsp_valid;

   always_ff@(posedge clk)
     begin
	l1d_rsp_tag <= r_l1d_rsp_tag;
	l1d_rsp_addr <= r_saveaddr;
	l1d_rsp_writeback <= (r_opcode == MEM_SW);
     end
   assign l1_mem_load_data = r_rsp_data;

   always_ff@(negedge clk)
     begin
	//if(l1d_rsp_valid)
	//begin
	//$display("L1D RESP FOR ADDR %x TAG %d at cycle %d, data %x",
	//l1d_rsp_addr, l1d_rsp_tag, r_cycle, l1_mem_load_data);
	//end
	if(mmu_rsp_valid)
	  begin
	     $display("MMU RSP, return %x at cycle %d",
		      mmu_rsp_data, r_cycle);
	  end
     end
   
   
   assign l1_mem_req_ack = r_req_ack;
   
   assign cache_hits = r_cache_hits;
   assign cache_accesses = r_cache_accesses;
   
     
   logic [127:0] 	t_d0, t_d1;
   wire [127:0] 	w_d0, w_d1;
   wire [TAG_BITS-1:0] 	w_tag0, w_tag1;
   wire 		w_valid0, w_dirty0,w_valid1, w_dirty1;

   logic		t_last, t_wr_last;
   wire			w_last;


   
   
   wire 		w_hit0 = w_valid0 ? (r_tag == w_tag0) : 1'b0;
   wire 		w_hit1 = w_valid1 ? (r_tag == w_tag1) : 1'b0;
   wire			w_hit = (w_hit0 | w_hit1) & r_got_req;
   wire [127:0]		w_d = w_hit0 ? w_d0 : w_d1;


   localparam		N_ROB_ENTRIES = (1<<`LG_L2_REQ_TAGS);
   logic		t_alloc_rob,t_pop_rob,t_is_wb,t_is_st;
   
   logic [`LG_L2_REQ_TAGS:0] r_rob_head_ptr, n_rob_head_ptr;
   logic [`LG_L2_REQ_TAGS:0] r_rob_tail_ptr, n_rob_tail_ptr;      
   logic [N_ROB_ENTRIES-1:0] r_rob_valid, r_rob_done, r_rob_hitbusy;
   logic [N_ROB_ENTRIES-1:0] r_rob_was_wb, r_rob_was_st;
   
   logic [31:0]		     r_rob_addr [N_ROB_ENTRIES-1:0];
   logic [`LG_MRQ_ENTRIES:0] r_rob_l1tag [N_ROB_ENTRIES-1:0];
   logic		     r_rob_replace[N_ROB_ENTRIES-1:0];
   req_t	     r_rob_req_ty[N_ROB_ENTRIES-1:0];
   
   logic [127:0]	     r_rob_data [N_ROB_ENTRIES-1:0];
   logic [127:0]	     r_rob_st_data [N_ROB_ENTRIES-1:0];
   
   wire [`LG_L2_REQ_TAGS-1:0] w_rob_head_ptr = r_rob_head_ptr[`LG_L2_REQ_TAGS-1:0];
   wire [`LG_L2_REQ_TAGS-1:0] w_rob_tail_ptr = r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0];
   
   wire			      w_rob_empty = r_rob_head_ptr == r_rob_tail_ptr;
   wire			      w_rob_full = (r_rob_head_ptr != r_rob_tail_ptr) &
			      (r_rob_head_ptr[`LG_L2_REQ_TAGS-1:0] == r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0]);
   
   
   
   wire 		w_need_wb0 = w_valid0 ? w_dirty0 : 1'b0;
   wire			w_need_wb1 = w_valid1 ? w_dirty1 : 1'b0;   
   wire			w_need_wb = w_need_wb0 | w_need_wb1;

   logic		n_mmu_mark_req, r_mmu_mark_req;
   logic		n_mmu_mark_dirty, r_mmu_mark_dirty;
   logic		n_mmu_mark_accessed, r_mmu_mark_accessed;
   
   logic		r_mmu_req, n_mmu_req;
   logic		r_l1d_req, n_l1d_req;
   logic 		r_l1i_req, n_l1i_req;
   logic 		r_last_gnt, n_last_gnt;
   logic 		n_req, r_req;
   logic		r_mmu_addr3, n_mmu_addr3;
   logic		n_mmu, r_mmu;
   logic		n_mark_pte, r_mark_pte;
   logic 		r_last_idle, n_last_idle;
   logic		r_was_st, n_was_st;
   
   wire [127:0]		w_updated_pte = r_mmu_addr3 ? 
			{w_d[127:72], r_mmu_mark_dirty|w_d[71], r_mmu_mark_accessed|w_d[70], w_d[69:0]} :
			{w_d[127:8], r_mmu_mark_dirty|w_d[7], r_mmu_mark_accessed|w_d[6], w_d[5:0]};      
   wire [N_ROB_ENTRIES-1:0] w_hit_rob;
   generate
      for(genvar i = 0; i < N_ROB_ENTRIES; i=i+1)
	begin
	   assign w_hit_rob[i] = r_rob_valid[i] ? (r_rob_addr[i] == n_addr) : 1'b0;
	end
   endgenerate
   wire			    w_hit_inflight = |w_hit_rob;
   

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_rob_head_ptr <= 'd0;
	     r_rob_tail_ptr <= 'd0;	     
	  end
	else
	  begin
	     r_rob_head_ptr <= n_rob_head_ptr;
	     r_rob_tail_ptr <= n_rob_tail_ptr;
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_rob_valid <= 'd0;
	     r_rob_done <= 'd0;
	     r_rob_hitbusy <= 'd0;
	     r_rob_was_wb <= 'd0;
	     r_rob_was_st <= 'd0;
	  end
	else
	  begin
	     if(t_alloc_rob)
	       begin
		  r_rob_valid[r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0]] <= 1'b1;
		  r_rob_done[r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0]] <= 1'b0;
		  r_rob_hitbusy[r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0]] <= w_hit_inflight;
		  r_rob_was_wb[r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0]] <= t_is_wb;
		  r_rob_was_st[r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0]] <= t_is_st;
		  r_rob_st_data[r_rob_tail_ptr[`LG_L2_REQ_TAGS-1:0]] <= r_store_data;
	       end
	     
	     if(mem_rsp_valid & (r_state != FLUSH_STORE))
	       begin
		  r_rob_done[mem_rsp_tag] <= 1'b1;
		  r_rob_data[mem_rsp_tag] <= mem_rsp_load_data;
		  		     
		  if( r_rob_done[mem_rsp_tag] )
		    begin
		       $display("tag %d is already done.., valid %b", 
				mem_rsp_tag, r_rob_valid[mem_rsp_tag]);
		       $stop();
		    end
		  
		  if( r_rob_valid[mem_rsp_tag] == 1'b0)
		    begin
		       $stop();
		    end		  
	       end // if (mem_rsp_valid)
		 
	     if(t_pop_rob)
	       begin
		  r_rob_valid[w_rob_head_ptr] <= 1'b0;
	       end
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	if(t_alloc_rob)
	  begin
	     r_rob_addr[w_rob_tail_ptr] <= n_addr;
	     r_rob_l1tag[w_rob_tail_ptr] <= n_l1d_rsp_tag;
	     r_rob_replace[w_rob_tail_ptr] <= n_replace;
	     r_rob_req_ty[w_rob_tail_ptr] <= r_req_ty;
	     
	  end
     end

`ifdef VERBOSE_L2
   always_ff@(negedge clk)
     begin
	if(t_alloc_rob)
	  begin
	     $display("allocate entry %d for address %x, wb %b, tag %d, cycle %d, state = %d", 
		      w_rob_tail_ptr, 
		      n_addr,
		      t_is_wb,
		      n_l1d_rsp_tag,
		      r_cycle, 
		      r_state);
	  end

	if(t_pop_rob)
	  begin
	     $display("dealloc entry %d at cycle %d", w_rob_head_ptr, r_cycle);
	  end
     end
`endif   
   
   always_comb
     begin
	n_rob_head_ptr = r_rob_head_ptr;
	n_rob_tail_ptr = r_rob_tail_ptr;
	if(t_alloc_rob)
	  begin
	     n_rob_tail_ptr = r_rob_tail_ptr + 'd1;
	  end
	if(t_pop_rob)
	  begin
	     n_rob_head_ptr = r_rob_head_ptr + 'd1;
	  end
     end


   
   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mmu_addr3 <= 1'b0;
	     r_mmu <= 1'b0;
	     r_was_st <= 1'b0;
	     r_mark_pte <= 1'b0;
	     r_mmu_rsp_data <= 'd0;
	     r_mmu_rsp_valid <= 1'b0;
	     r_mem_mark_rsp_valid <= 1'b0;
	     r_state <= INITIALIZE;
	     r_req_ty <= WRITEBACK;
	     r_got_req <= 1'b0;
	     r_flush_state <= WAIT_FOR_FLUSH;
	     r_flush_complete <= 1'b0;
	     r_idx <= 'd0;
	     r_tag <= 'd0;
	     r_opcode <= 4'd0;
	     r_addr <= 'd0;
	     r_rob_tag <= 'd0;
	     r_wb_addr <= 'd0;
	     r_need_wb <= 1'b0;
	     r_saveaddr <= 'd0;
	     r_mem_req <= 1'b0;
	     r_mem_opcode <= 4'd0;
	     r_rsp_data <= 'd0;
	     r_l1d_rsp_valid <= 1'b0;
	     r_l1i_rsp_valid <= 1'b0;
	     r_l1d_rsp_tag <= 'd0;
	     r_req_ack <= 1'b0;
	     r_store_data <= 'd0;
	     r_flush_req <= 1'b0;
	     r_need_l1d <= 1'b0;
	     r_need_l1i <= 1'b0;
	     r_cache_hits <= 'd0;
	     r_cache_accesses <= 'd0;
	     r_l1d_req <= 1'b0;
	     r_l1i_req <= 1'b0;
	     r_mmu_req <= 1'b0;
	     r_mmu_mark_req <= 1'b0;	     
	     r_last_gnt <= 1'b0;
	     r_req <= 1'b0;
	     r_last_l1i_addr <= 'd0;
	     r_last_l1d_addr <= 'd0;
	     r_mmu_mark_dirty <= 1'b0;
	     r_mmu_mark_accessed <= 1'b0;
	     r_replace <= 1'b0;
	     r_wb1 <= 1'b0;
	     r_last_idle <= 1'b0;
	  end
	else
	  begin
	     r_mmu_addr3 <= n_mmu_addr3;
	     r_mmu <= n_mmu;
	     r_was_st <= n_was_st;
	     r_mark_pte <= n_mark_pte;
	     r_mmu_rsp_data <= n_mmu_rsp_data;
	     r_mmu_rsp_valid <= n_mmu_rsp_valid;
	     r_l1d_rsp_tag <= n_l1d_rsp_tag;
	     r_mem_mark_rsp_valid <= n_mem_mark_rsp_valid;
	     r_state <= n_state;
	     r_req_ty <= n_req_ty;
	     
	     r_got_req <= n_got_req;
	     r_flush_state <= n_flush_state;
	     r_flush_complete <= n_flush_complete;
	     r_idx <= t_idx;
	     r_tag <= n_tag;
	     r_opcode <= n_opcode;
	     r_addr <= n_addr;
	     r_rob_tag <= n_rob_tag;
	     r_wb_addr <= n_wb_addr;
	     r_need_wb <= n_need_wb;
	     r_saveaddr <= n_saveaddr;
	     r_mem_req <= n_mem_req;
	     r_mem_opcode <= n_mem_opcode;
	     r_rsp_data <= n_rsp_data;
	     r_l1d_rsp_valid <= n_l1d_rsp_valid;
	     r_l1i_rsp_valid <= n_l1i_rsp_valid;
	     r_req_ack <= n_req_ack;
	     r_store_data <= n_store_data;
	     r_flush_req <= n_flush_req;
	     r_need_l1i <= n_need_l1i;
	     r_need_l1d <= n_need_l1d;
	     r_cache_hits <= n_cache_hits;
	     r_cache_accesses <= n_cache_accesses;
	     r_l1d_req <= n_l1d_req;
	     r_l1i_req <= n_l1i_req;
	     r_mmu_req <= n_mmu_req;
	     r_mmu_mark_req <= n_mmu_mark_req;
	     r_last_gnt <= n_last_gnt;
	     r_req <= n_req;
	     r_last_l1i_addr <= n_last_l1i_addr;
	     r_last_l1d_addr <= n_last_l1d_addr;
	     r_mmu_mark_dirty <= n_mmu_mark_dirty;
	     r_mmu_mark_accessed <= n_mmu_mark_accessed;
	     r_replace <= n_replace;
	     r_wb1 <= n_wb1;
	     r_last_idle <= n_last_idle;
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	r_mem_req_store_data <= n_mem_req_store_data;
     end
   
   //always_ff@(negedge clk)
   //begin
	//$display("l1i_flush_req = %b", l1i_flush_req);
	//$display("l1d_flush_req = %b", l1d_flush_req);
	
   //if((l1d_flush_complete||l1i_flush_complete) && (r_flush_state == WAIT_FOR_FLUSH)) 
   //$stop();
   //end
   
   always_comb
     begin
	n_flush_state = r_flush_state;
	n_need_l1d = r_need_l1d | l1d_flush_req;
	n_need_l1i = r_need_l1i | l1i_flush_req;
	t_l2_flush_req = 1'b0;
	case(r_flush_state)
	  WAIT_FOR_FLUSH:
	    begin
	       if(n_need_l1i | n_need_l1d)
		 begin
		    n_flush_state = WAIT_FOR_L1_FLUSH_DONE;
		 end
	    end
	  WAIT_FOR_L1_FLUSH_DONE:
	    begin
	       if(r_need_l1d && l1d_flush_complete)
		 begin
		    //$display("-> l1d flush complete at cycle %d", r_cycle);
		    n_need_l1d = 1'b0;
		 end
	       if(r_need_l1i && l1i_flush_complete)
		 begin
		    //$display("-> l1i flush complete at cycle %d", r_cycle);
		    n_need_l1i = 1'b0;
		 end
	       
	       if((n_need_l1d==1'b0) && (n_need_l1i==1'b0))
		 begin
		    //$display("-> firing l2 flush at cycle %d", r_cycle);
		    n_flush_state = WAIT_FOR_FLUSH;
		    t_l2_flush_req = 1'b1;
		 end
	    end
	endcase
     end // always_comb

   logic t_probe_mmu_req_valid;
   logic [(`PA_WIDTH-1):0] r_l2_probe_addr, n_l2_probe_addr;
   logic 		  n_l2_probe_val, r_l2_probe_val;
   typedef enum logic  {PROBE_IDLE, PROBE_WAIT } probe_state_t;

   assign l2_probe_val = r_l2_probe_val;
   assign l2_probe_addr = r_l2_probe_addr;

   logic [63:0] r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end
  
   
   probe_state_t n_pstate, r_pstate;
   logic n_l2_probe_mmu, r_l2_probe_mmu;
   
   always_comb
     begin
	n_pstate = r_pstate;
	t_probe_mmu_req_valid = 1'b0;
	n_l2_probe_val = 1'b0;
	n_l2_probe_addr = r_l2_probe_addr;
	n_l2_probe_mmu = r_l2_probe_mmu;
	case(r_pstate)
	  PROBE_IDLE:
	    begin
	       if(mmu_req_valid)
		 begin
		    $display("got probe req at cycle %d for addr %x", r_cycle, mmu_req_addr);
		    n_pstate = PROBE_WAIT;
		    n_l2_probe_val = 1'b1;
		    n_l2_probe_addr = mmu_req_addr;
		    n_l2_probe_mmu = 1'b1;
		 end
	    end
	  PROBE_WAIT:
	    begin
	       if(l2_probe_ack)
		 begin
		    $display("got probe ack at cycle %d", r_cycle);
		    n_pstate = PROBE_IDLE;
		    t_probe_mmu_req_valid = r_l2_probe_mmu;
		    n_l2_probe_mmu = 1'b0;
		 end
	    end
	  default:
	    begin
	    end
	endcase // case (r_pstate)
     end // always_comb

   always_ff@(posedge clk)
     begin
	if(n_wb1)
	  begin
	     r_tag_wb1 <= w_tag1;
	     r_data_wb1 <= w_d1;
	  end
     end
      
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_pstate <= PROBE_IDLE;
	     r_l2_probe_val <= 1'b0;
	     r_l2_probe_addr <= 'd0;
	     r_l2_probe_mmu <= 1'b0;
	  end
	else
	  begin
	     r_pstate <= n_pstate;
	     r_l2_probe_val <= n_l2_probe_val;
	     r_l2_probe_addr <= n_l2_probe_addr;
	     r_l2_probe_mmu <= n_l2_probe_mmu;
	  end
     end

   
   localparam N_MQ_ENTRIES = (1<<`LG_MRQ_ENTRIES);
   l1d_req_t r_mem_q[N_MQ_ENTRIES-1:0];
   
   logic [`LG_MRQ_ENTRIES:0] r_l1d_head_ptr, n_l1d_head_ptr;
   logic [`LG_MRQ_ENTRIES:0] r_l1d_tail_ptr, n_l1d_tail_ptr;
   l1d_req_t t_l1dq;
   
   always_comb
     begin
	t_l1dq = r_mem_q[r_l1d_head_ptr[`LG_MRQ_ENTRIES-1:0]];
     end
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_l1d_head_ptr <= 'd0;
	     r_l1d_tail_ptr <= 'd0;
	  end
	else
	  begin
	     r_l1d_head_ptr <= n_l1d_head_ptr;
	     r_l1d_tail_ptr <= n_l1d_tail_ptr;
	  end
     end // always_ff@ (posedge clk)

   wire w_l1d_full = (r_l1d_head_ptr != r_l1d_tail_ptr) &&
		     (r_l1d_head_ptr[`LG_MRQ_ENTRIES-1:0] == r_l1d_tail_ptr[`LG_MRQ_ENTRIES-1:0]);

   wire [`LG_MRQ_ENTRIES:0] w_l1d_tail_ptr_p1 = r_l1d_tail_ptr + 'd1;
			    
   wire	w_l1d_almost_full = (r_l1d_head_ptr != w_l1d_tail_ptr_p1) &&
	(r_l1d_head_ptr[`LG_MRQ_ENTRIES-1:0] == w_l1d_tail_ptr_p1[`LG_MRQ_ENTRIES-1:0]);

   
   wire w_l1d_empty = (r_l1d_head_ptr == r_l1d_tail_ptr);
   
   assign l1d_rdy = !w_l1d_almost_full;
//!w_l1d_full;

   always_ff@(posedge clk)
     begin
	if(l1d_req_valid)
	  begin
	     r_mem_q[r_l1d_tail_ptr[`LG_MRQ_ENTRIES-1:0]] <= l1d_req;
	     if(w_l1d_full) $stop();
	  end
     end
   
`ifdef VERILATOR
   logic [31:0] r_inflight, n_inflight;
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_inflight <= 'd0;
	  end
	else
	  begin
	     r_inflight <= n_inflight;
	  end
     end // always_ff@ (negedge clk)

   always_comb
     begin
	n_inflight = r_inflight;
	if(l1d_req_valid)
	  begin
	     n_inflight = n_inflight + 'd1;
	  end
	if(t_gnt_l1d)
	  begin
	     n_inflight = n_inflight - 'd1;		  
	  end
     end // always_comb

   always_ff@(negedge clk)
     begin
	l1_to_l2_queue_occupancy(r_inflight);
	record_l2_state({27'd0, r_state});
     end
`endif
   
   always_comb
     begin
	n_l1d_head_ptr = r_l1d_head_ptr;
	n_l1d_tail_ptr = r_l1d_tail_ptr;	
	if(l1d_req_valid)
	  begin
	     n_l1d_tail_ptr = r_l1d_tail_ptr + 'd1;
	  end
	if(t_gnt_l1d)
	  begin
	     n_l1d_head_ptr = r_l1d_head_ptr + 'd1;
	  end
     end // always_comb

   always_ff@(posedge clk)
     begin
	r_l1d <= reset ? 1'b0 : n_l1d;
	r_l1i <= reset ? 1'b0 : n_l1i;	
     end
   
   wire w_l1i_req = r_l1i_req | l1i_req;
   wire w_l1d_req = !w_l1d_empty;
   //r_l1d_req | l1d_req;
   wire	w_mmu_req = r_mmu_req | t_probe_mmu_req_valid;
   wire w_mem_mark_valid = mem_mark_valid | r_mmu_mark_req;

   wire w_pick_l1i = (w_l1i_req & w_l1d_req) ? r_last_gnt : w_l1i_req;
   wire w_pick_l1d = (w_l1i_req & w_l1d_req) ? !r_last_gnt : w_l1d_req;   

   logic t_can_accept_txn;
   
   logic [127:0] r_data;
   always_ff@(posedge clk)
     begin
	r_data <= r_rob_data[w_rob_head_ptr];
     end
   
   always_comb
     begin
	n_rsp_data = w_hit ? w_d : r_rsp_data;
	n_mem_mark_rsp_valid = 1'b0;
	
	n_mmu_rsp_data = r_mmu_addr3 ? w_d[127:64] : w_d[63:0];
	n_mmu_rsp_valid = 1'b0;
	
	t_d0 = r_data;
	t_d1 = r_data;
	n_l1i_rsp_valid = 1'b0;
	t_can_accept_txn = 1'b0;
	
	//n_mmu_rsp_data = r_mmu_rsp_data;	
	if(w_hit)
	  begin
	     if(r_opcode == MEM_LW)
	       begin
		  if(r_mmu)
		    begin
		       n_mmu_rsp_valid = 1'b1;
		    end
		  else if(r_mark_pte)
		    begin
		       t_d0 = w_updated_pte;
		       t_d1 = w_updated_pte;
		       n_mem_mark_rsp_valid = 1'b1;		       
		    end
		  else if(r_last_gnt == 1'b0)
		    begin
		       n_l1i_rsp_valid  = 1'b1;
		    end
	       end // if (r_opcode == MEM_LW)
	     else
	       begin
		  t_d0 = r_store_data;
		  t_d1 = r_store_data;	
	       end
	  end
     end // always_comb

   
   // always_ff@(negedge clk)
   //   begin
   // 	if((r_state == CHECK_VALID_AND_TAG) & r_saveaddr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN] == 'd1765)
   // 	  begin
   // 	     if(w_hit)
   // 	       begin
   // 		  $display("hit  for addr %x, w_tag0 = %x, w_tag1 = %x, v0 %b, v1 %b, line %d, r_tag %x, t_last %b", 
   // 			   r_saveaddr, w_tag0, w_tag1, 
   // 			   w_valid0, w_valid1,
   // 			   r_saveaddr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN],
   // 			   r_tag,
   // 			   t_last);
   // 	       end
   // 	     else
   // 	       begin
   // 		  $display("miss %b for addr %x, w_tag0 = %x, w_tag1 = %x, v0 %b, v1 %b, line %d, r_tag %x, w_last %b", 
   // 			   n_replace, r_saveaddr, w_tag0, w_tag1, 
   // 			   w_valid0, w_valid1,
   // 			   r_saveaddr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN],
   // 			   r_tag,
   // 			   w_last);
   // 	       end
   // 	  end
   //   end
  
   wire w_head_of_rob_done = !w_rob_empty & r_rob_valid[w_rob_head_ptr] & r_rob_done[w_rob_head_ptr];
   //always_ff@(negedge clk)
   //begin
   //$display("cycle %d, state %d, valid %b", r_cycle, r_state, r_rob_valid);
   //end

   //	$display("cycle %d, rob empty %b, head done %b, head ptr %b, tail ptr %b", 
   //r_cycle, w_rob_empty, w_head_of_rob_done,
   //r_rob_head_ptr, r_rob_tail_ptr);
   //end

   logic r_pop_rob;
   always_ff@(posedge clk)
     begin
	r_pop_rob <= reset ? 1'b0 : t_pop_rob;
     end

   
   
   always_comb
     begin
	n_last_gnt = r_last_gnt;
	n_l1i_req = r_l1i_req | l1i_req;
	n_l1d_req = r_l1d_req | l1d_req_valid;
	n_mmu_req = r_mmu_req | t_probe_mmu_req_valid;
	n_mmu_mark_req = mem_mark_valid | r_mmu_mark_req;
	n_req = r_req;
	n_mmu_addr3 = r_mmu_addr3;
	n_mmu = r_mmu;
	n_mark_pte = r_mark_pte;

	n_replace = r_replace;
	n_state = r_state;
	n_got_req = 1'b0;
	n_flush_complete = 1'b0;
	
	t_wr_valid0 = 1'b0;
	t_wr_dirty0 = 1'b0;
	t_wr_tag0 = 1'b0;
	
	t_wr_valid1 = 1'b0;
	t_wr_dirty1 = 1'b0;
	t_wr_tag1 = 1'b0;	

	t_wr_last = 1'b0;
	
	t_wr_d0 = 1'b0;
	t_wr_d1 = 1'b0;
	
	t_idx = r_idx;
	n_tag = r_tag;
	n_opcode = r_opcode;
	n_addr = r_addr;
	n_rob_tag = r_rob_tag;
	n_wb_addr = r_wb_addr;
	n_need_wb = r_need_wb;
	
	n_saveaddr = r_saveaddr;
	
	n_req_ack = 1'b0;
	n_mem_req = 1'b0;
	
	n_mem_opcode = r_mem_opcode;
		
	t_valid = 1'b0;
	t_dirty = 1'b0;
	t_last = 1'b0;


	n_l1d_rsp_tag = r_l1d_rsp_tag;
	
	n_store_data = r_store_data;
	n_flush_req = r_flush_req | t_l2_flush_req;

	n_mem_req_store_data = r_mem_req_store_data;

	n_cache_hits = r_cache_hits;
	n_cache_accesses = r_cache_accesses;

	n_last_l1i_addr = r_last_l1i_addr;
	n_last_l1d_addr = r_last_l1d_addr;

	t_gnt_l1i = 1'b0;
	t_gnt_l1d = 1'b0;

	n_l1d = r_l1d;
	n_l1i = r_l1i;
	
	n_mmu_mark_dirty = r_mmu_mark_dirty;
	n_mmu_mark_accessed = r_mmu_mark_accessed;
	
	n_wb1 = r_wb1;
	//n_l1i_rsp_valid = 1'b0;
	n_l1d_rsp_valid = 1'b0;
	n_last_idle = 1'b0;
	
	t_is_wb = 1'b0;
	t_is_st = 1'b0;
	t_alloc_rob = 1'b0;
	t_pop_rob = 1'b0;
	n_req_ty = r_req_ty;
	n_was_st = r_was_st;
	
	case(r_state)
	  INITIALIZE:
	    begin
	       t_valid = 1'b0;
	       t_dirty = 1'b0;
	       t_last = 1'b1;

	       t_wr_last = 1'b1;
	       
	       t_wr_valid0 = 1'b1;
	       t_wr_dirty0 = 1'b1;
	       t_wr_tag0 = 1'b1;
	       t_wr_d0 = 1'b1;
	       
	       t_wr_valid1 = 1'b1;
	       t_wr_dirty1 = 1'b1;
	       t_wr_tag1 = 1'b1;
	       t_wr_d1 = 1'b1;
	       
	       t_idx = r_idx + 'd1;
	       if(r_idx == (L2_LINES-1))
		 begin
		    n_state = IDLE;
		    n_flush_complete = 1'b1;
		 end
	    end // case: INITIALIZE
	  IDLE:
	    begin
	       t_idx = 'd0;
	       n_tag = r_tag;
	       n_addr = r_addr;
	       
	       n_opcode = MEM_LW;
	       n_store_data = r_store_data;
	       n_last_idle = 1'b1;

	       //$display("waiting for %d entry empty %b, full %b", 
	       //r_rob_head_ptr, w_rob_empty, w_rob_full);
	       
	       if(w_head_of_rob_done)
		 begin
		    n_replace = r_rob_replace[w_rob_head_ptr];
		    n_addr = r_rob_addr[w_rob_head_ptr];
		    n_saveaddr = r_rob_addr[w_rob_head_ptr];
		    n_tag = r_rob_addr[w_rob_head_ptr][(`PA_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
		    t_idx = r_rob_addr[w_rob_head_ptr][LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];
		    n_l1d_rsp_tag = r_rob_l1tag[w_rob_head_ptr];
		    n_mmu_addr3 = r_rob_addr[w_rob_head_ptr][3];

		    n_was_st = 1'b0;
		    
		    if(r_rob_hitbusy[w_rob_head_ptr]) 
		      begin
			 $display("HIT BUSY for ADDR %x, tag entry %d", n_addr, w_rob_head_ptr);
			 //$stop();
			 n_state = CHECK_VALID_AND_TAG;
			 n_got_req = 1'b1;
		      end
		    else
		      begin
			 case(r_rob_req_ty[w_rob_head_ptr])
			   MMU:
			     begin
				n_state = CLEAN_RELOAD;
				n_opcode = MEM_LW;			   
				n_last_gnt = 1'b0;
				n_mmu = 1'b1;
			     end
			   L1I:
			     begin
				n_state = CLEAN_RELOAD;
				n_opcode = MEM_LW;			   
				n_last_gnt = 1'b0;
			     end
			   L1D:
			     begin
				n_state = CLEAN_RELOAD;				
				n_was_st = r_rob_was_st[w_rob_head_ptr];
				n_opcode =  r_rob_was_st[w_rob_head_ptr] ? MEM_SW : MEM_LW;	
				n_store_data = r_rob_st_data[w_rob_head_ptr];
				n_last_gnt = 1'b1;		
				//n_l1d_rsp_valid  = 1'b1;
			     end
			   WRITEBACK:
			     begin
			     end
			   default:
			     begin
				$display("handle req type %d", r_rob_req_ty[w_rob_head_ptr]);
				$stop();
			     end
			 endcase // case (r_rob_req_ty[w_rob_head_ptr])
		      end
		    t_pop_rob = 1'b1;
		 end
	       else if(w_rob_full==1'b0)
		 begin
		    if(r_need_wb)
		      begin
			 //$display("performing writeback at cycle %d for address %x", r_cycle, r_wb_addr);
			 n_state = PREPARE_WRITEBACK;
			 n_addr = r_wb_addr;
			 n_need_wb = 1'b0;
			 n_rob_tag = w_rob_tail_ptr;
			 n_req_ty = WRITEBACK;
		      end
		    else if(n_flush_req)
		      begin
			 t_idx = 'd0;
			 n_state = FLUSH_WAIT;
			 n_req_ty = FLUSH;			 
		      end
		    else if(w_mem_mark_valid & w_rob_empty)
		      begin
			 n_mmu_mark_req = 1'b0;
			 n_mmu_mark_dirty = mem_mark_dirty;
			 n_mmu_mark_accessed = mem_mark_accessed;
			 n_mmu_addr3 = mem_mark_addr[3];		    
			 t_idx = mem_mark_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
			 n_tag = mem_mark_addr[(`PA_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			 n_addr = {mem_mark_addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_saveaddr = {mem_mark_addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_opcode = MEM_LW;
			 n_mark_pte = 1'b1;
			 n_state = CHECK_VALID_AND_TAG;
			 n_got_req = 1'b1;
			 n_req_ty = MARK_PTE;
		      end
		    else if(w_mmu_req & w_rob_empty)
		      begin
			 n_mmu_addr3 = mmu_req_addr[3];		    
			 t_idx = mmu_req_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
			 n_tag = mmu_req_addr[(`PA_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			 n_addr = {mmu_req_addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_saveaddr = {mmu_req_addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_opcode = MEM_LW;
			 n_state = CHECK_VALID_AND_TAG;
			 n_mmu = 1'b1;
			 n_got_req = 1'b1;
			 n_req_ty = MMU;
			 //$display("r_mmu_req  = %b,  t_probe_mmu_req_valid = %b", r_mmu_req, t_probe_mmu_req_valid);
			 //$display("l2 : mmu req addr %x, w_l1d_req = %b, w_l1i_req = %b", r_addr, w_l1d_req, w_l1i_req);		    
		      end
		    else if(w_l1d_req | w_l1i_req)
		      begin
			 n_l1d = w_pick_l1d;
			 n_l1i = w_pick_l1i;
			 if(w_pick_l1i)
			   begin
			      n_last_gnt = 1'b0;			 
			      t_idx = l1i_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];
			      n_tag = l1i_addr[(`PA_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			      n_last_l1i_addr = l1i_addr[(`PA_WIDTH-1):`LG_L2_CL_LEN];
			      n_addr = {l1i_addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_saveaddr = {l1i_addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_opcode = MEM_LW;
			      n_l1i_req = 1'b0;
			      t_gnt_l1i = 1'b1;
			      n_req_ty = L1I;
			   end
			 else if(w_pick_l1d)
			   begin
			      n_last_gnt = 1'b1;
			      t_idx = t_l1dq.addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
			      n_tag = t_l1dq.addr[(`PA_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			      n_addr = {t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_last_l1d_addr = t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN];			 
			      n_saveaddr = {t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_store_data = t_l1dq.data;
			      n_opcode = t_l1dq.opcode;
			      n_l1d_req = 1'b0;
			      n_l1d_rsp_tag = t_l1dq.tag;
			      t_gnt_l1d = 1'b1;
			      n_req_ty = L1D;
			   end
			 n_req_ack = 1'b1;
			 n_got_req = 1'b1;		    
			 n_state = CHECK_VALID_AND_TAG;
			 n_cache_accesses = r_cache_accesses + 64'd1;
			 n_cache_hits = r_cache_hits + 64'd1;
		      end // if (w_l1d_req | w_l1i_req)
		 end // else: !if(w_rob_full)
	    end // case: IDLE
	  CHECK_VALID_AND_TAG:
	    begin
	       //load hit
	       if(w_hit)
		 begin
		    t_wr_last = 1'b1;
		    t_last = w_hit0 ? 1'b0 : 1'b1;
		    
		    if(r_opcode == MEM_LW)
		      begin			 
			 if(r_mmu)
			   begin
			      n_mmu_req = 1'b0;
			      n_mmu = 1'b0;
			      n_state = IDLE;			      
			   end
			 else if(r_mark_pte)
			   begin
			      n_state = WAIT_STORE_IDLE;
			      //$display("mark dirty %b, mark accessed %b", r_mmu_mark_dirty, r_mmu_mark_accessed);
			      n_mmu_mark_dirty = 1'b0;
			      n_mmu_mark_accessed = 1'b0;
			      t_wr_dirty0 = w_hit0;
			      t_wr_dirty1 = w_hit1;
			      t_dirty = 1'b1;
			      t_wr_d0 = w_hit0;
			      t_wr_d1 = w_hit1;			      
			      n_mark_pte = 1'b0;
			   end // if (r_mark_pte)
			 else if(r_last_gnt)
			   begin
			      n_l1d_rsp_valid  = 1'b1;	
			      if(w_l1d_req & !w_l1i_req & t_l1dq.opcode == MEM_LW & (r_need_wb==1'b0)&1'b0)
				begin
				   n_l1d = 1'b1;
				   n_last_idle = 1'b1;
				   n_last_gnt = 1'b1;
				   t_idx = t_l1dq.addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
				   n_tag = t_l1dq.addr[(`PA_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
				   n_addr = {t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
				   n_last_l1d_addr = t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN];			 
				   n_saveaddr = {t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
				   n_opcode = MEM_LW;
				   n_l1d_req = 1'b0;
				   n_l1d_rsp_tag = t_l1dq.tag;
				   t_gnt_l1d = 1'b1;
				   n_got_req = 1'b1;
				   //$display("early1");
				end
			      else
				begin
				   n_state = IDLE;	
				end
			   end
			 else
			   begin
			      //n_l1i_rsp_valid  = 1'b1;
			      if(w_l1d_req & !w_l1i_req & t_l1dq.opcode == MEM_LW & (r_need_wb==1'b0)&1'b0)
				begin
				   n_l1d = 1'b1;
				   n_last_idle = 1'b1;
				   n_last_gnt = 1'b1;
				   t_idx = t_l1dq.addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
				   n_tag = t_l1dq.addr[(`PA_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
				   n_addr = {t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
				   n_last_l1d_addr = t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN];			 
				   n_saveaddr = {t_l1dq.addr[(`PA_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
				   n_opcode = MEM_LW;
				   n_l1d_req = 1'b0;
				   n_l1d_rsp_tag = t_l1dq.tag;
				   t_gnt_l1d = 1'b1;
				   n_got_req = 1'b1;	
				   //$display("early2");
				end
			      else
				begin
				   n_state = IDLE;	
				end
			   end
		      end
		    else if(r_opcode == MEM_SW)
		      begin
			 t_wr_dirty0 = w_hit0;
			 t_wr_dirty1 = w_hit1;			 
			 t_dirty = 1'b1;
			 n_state = WAIT_STORE_IDLE;
			 //n_cache_hits = r_cache_hits + 64'd1;			 
			 t_wr_d0 = w_hit0;
			 t_wr_d1 = w_hit1;	
			 n_l1d_rsp_valid = 1'b1;
		      end
		 end
	       else
		 begin
		    t_alloc_rob = 1'b1;
		    t_is_st = r_opcode == MEM_SW;
		    n_rob_tag = w_rob_tail_ptr;			 		    
		    n_cache_hits = r_cache_hits - 64'd1;
		    n_replace = (w_valid0==1'b0 ? 1'b0 :
				(w_valid1==1'b0 ? 1'b1 : 
				~w_last));
		    
		    t_wr_last = 1'b1;
		    t_last = n_replace;

		    if(n_replace)
		      begin
			 if(w_dirty1)
			   begin
			      n_mem_req_store_data = w_d1;
			      n_wb_addr = {w_tag1, t_idx, {{`LG_L2_CL_LEN{1'b0}}}};
			      n_need_wb = 1'b1;
			   end
		      end // if (n_replace)
		    else
		      begin
			 if(w_dirty0)
			   begin
			      n_mem_req_store_data = w_d0;
			      n_wb_addr = {w_tag0, t_idx, {{`LG_L2_CL_LEN{1'b0}}}};
			      n_need_wb = 1'b1;
			   end
		      end // else: !if(n_replace)
		    n_state = IDLE;
		    n_mem_opcode = 4'd4; //load
		    n_mem_req = 1'b1;
		    //$display("missed l2 for address %x, pointer %d", r_addr, w_rob_tail_ptr);		    
		 end // else: !if(w_hit)
	    end // case: CHECK_VALID_AND_TAG
	  PREPARE_WRITEBACK:
	    begin
	       t_alloc_rob = 1'b1;
	       t_is_wb = 1'b1;	       
	       n_mem_opcode = MEM_SW; 	       	       
	       n_mem_req = 1'b1;	       
	       n_state = IDLE;
	    end
	  DIRTY_STORE:
	    begin
	       n_state = IDLE;
	    end // case: DIRTY_STORE
	  STORE_TURNAROUND:
	    begin
	       n_state = IDLE;
	       //n_mem_req = 1'b1;		    
	    end
	  CLEAN_RELOAD:
	    begin
	       n_mem_req = 1'b0;
	       t_valid = 1'b1;
	       t_dirty = 1'b0;
	       
	       t_wr_valid0 = r_replace == 1'b0;
	       t_wr_dirty0 = r_replace == 1'b0;
	       t_wr_tag0 = r_replace == 1'b0;
	       t_wr_d0 = r_replace ==  1'b0;
	       
	       t_wr_valid1 = r_replace == 1'b1;
	       t_wr_dirty1 = r_replace == 1'b1;		    
	       t_wr_tag1 = r_replace == 1'b1;
	       t_wr_d1 = r_replace == 1'b1;
	       
	       n_state = WAIT_CLEAN_RELOAD;
	    end // case: CLEAN_RELOAD
	  WAIT_CLEAN_RELOAD: /* need a cycle to turn around */
	    begin
	       n_state = CHECK_VALID_AND_TAG;
	       n_got_req = 1'b1;
	    end
	  WAIT_STORE_IDLE:
	    begin
	       n_state = IDLE;
	    end
	  FLUSH_WAIT:
	    begin
	       n_state = FLUSH_TRIAGE;
	       t_valid = 1'b0;
	       t_dirty = 1'b0;
	       t_wr_valid0 = 1'b1;
	       t_wr_dirty0 = 1'b1;
	       t_wr_valid1 = 1'b1;
	       t_wr_dirty1 = 1'b1;	       
	    end
	  FLUSH_TRIAGE:
	    begin
	       //if(w_need_wb)
	       //$display("address = %x needs wb", {w_tag0, t_idx, 6'd0});
	       n_wb1 = w_need_wb0 & w_need_wb1;
	       
	       if(w_need_wb)
		 begin
		    n_mem_req_store_data = w_need_wb0 ? w_d0 : w_d1;
		    n_addr = {(w_need_wb0 ? w_tag0 : w_tag1), t_idx, 4'd0};
		    n_mem_opcode = 4'd7; 
		    n_mem_req = 1'b1;
		    n_state = FLUSH_STORE;
		 end
	       else
		 begin
		    t_idx = r_idx + 'd1;
		    if(r_idx == (L2_LINES-1))
		      begin
			 n_state = IDLE;
			 //$display("L2 flush complete at cycle %d", r_cycle);
			 n_flush_complete = 1'b1;
			 n_flush_req = 1'b0;
		      end
		    else
		      begin
			 n_state = FLUSH_WAIT;
		      end
		 end
	    end // case: FLUSH_TRIAGE
	  FLUSH_STORE:
	    begin
	       if(mem_rsp_valid)
		 begin
		    if(r_wb1)
		      begin
			 n_state = FLUSH_STORE_WAY2;
		      end
		    else
		      begin
			 t_idx = r_idx + 'd1;
			 if(r_idx == (L2_LINES-1))
			   begin
			      n_state = IDLE;
			      //$display("L2 flush complete at cycle %d", r_cycle);
			      n_flush_complete = 1'b1;
			      n_flush_req = 1'b0;
			   end
			 else
			   begin
			      n_state = FLUSH_WAIT;
			   end		    
		      end
		 end // if (mem_rsp_valid)
	    end // case: FLUSH_STORE
	  FLUSH_STORE_WAY2:
	    begin
	       n_wb1 = 1'b0;
	       n_mem_req_store_data = r_data_wb1;
	       n_addr = {r_tag_wb1, t_idx, 4'd0};
	       n_mem_opcode = 4'd7; 
	       n_mem_req = 1'b1;
	       n_state = FLUSH_STORE;
	    end
	  
	  default:
	    begin
	    end
	endcase
     end

   always_ff@(negedge clk)
     begin
	if(r_state == IDLE & w_head_of_rob_done)
	  begin
	     //$display("transaction is done for %d at cycle %d, head %d, tail %d", 
	     //w_rob_head_ptr, r_cycle, r_rob_head_ptr, r_rob_tail_ptr);
	     //$display("addr = %x, ty = %d", r_rob_addr[w_rob_head_ptr], r_rob_req_ty[w_rob_head_ptr]);
	     //$display("saveaddr = %x", n_saveaddr);
	     //$display("replace = %b", r_rob_replace[w_rob_head_ptr]);
	     //$display("r_rob_l1tag = %d", r_rob_l1tag[w_rob_head_ptr]);
	     //$display("req type = %d", r_rob_req_ty[w_rob_head_ptr]);
	     if(r_rob_req_ty[w_rob_head_ptr] == MMU)
	       begin
		  $display("MMU REQ for addr %x begins at cycle %d",
			   r_rob_addr[w_rob_head_ptr],
			   r_cycle);
	       end
	     //if(w_mmu_req)
	     //begin
	     //$display("l2 : mmu req addr %x, w_l1d_req = %b, w_l1i_req = %b", r_addr, w_l1d_req, w_l1i_req);
	     //end
	  end
	
	if(r_state == CHECK_VALID_AND_TAG)
	  begin
	     
	     if(w_hit0 & w_hit1)
	       begin
		  $display("w_hit %b : hit 0 = %b, hit 1 = %b, r_addr %x, r_tag %x, cycle %d, opcode %d",
			   w_hit, w_hit0, w_hit1, r_saveaddr, r_tag, r_cycle, r_opcode);
		  
		 $display("multihit r_saveaddr = %x, r_tag = %x, cycle %d", r_saveaddr,r_tag, r_cycle);
		  $stop();
	       end
	  end // if (r_state == CHECK_VALID_AND_TAG)
     end

   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) last_ram
     (.clk(clk), .addr(t_idx), .wr_data(t_last), .wr_en(t_wr_last), .rd_data(w_last));      

   /* 1st way */
   reg_ram1rw #(.WIDTH(128), .LG_DEPTH(LG_L2_LINES)) data_ram0
     (.clk(clk), .addr(t_idx), .wr_data(t_d0), .wr_en(t_wr_d0), .rd_data(w_d0));
      
   reg_ram1rw #(.WIDTH(TAG_BITS), .LG_DEPTH(LG_L2_LINES)) tag_ram0
     (.clk(clk), .addr(t_idx), .wr_data(r_tag), .wr_en(t_wr_tag0), .rd_data(w_tag0));   
   
   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) valid_ram0
     (.clk(clk), .addr(t_idx), .wr_data(t_valid), .wr_en(t_wr_valid0), .rd_data(w_valid0));   

   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) dirty_ram0
     (.clk(clk), .addr(t_idx), .wr_data(t_dirty), .wr_en(t_wr_dirty0), .rd_data(w_dirty0));   

   /* 2nd way */
   reg_ram1rw #(.WIDTH(128), .LG_DEPTH(LG_L2_LINES)) data_ram1
     (.clk(clk), .addr(t_idx), .wr_data(t_d0), .wr_en(t_wr_d1), .rd_data(w_d1));
      
   reg_ram1rw #(.WIDTH(TAG_BITS), .LG_DEPTH(LG_L2_LINES)) tag_ram1
     (.clk(clk), .addr(t_idx), .wr_data(r_tag), .wr_en(t_wr_tag1), .rd_data(w_tag1));   
   
   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) valid_ram1
     (.clk(clk), .addr(t_idx), .wr_data(t_valid), .wr_en(t_wr_valid1), .rd_data(w_valid1));   

   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) dirty_ram1
     (.clk(clk), .addr(t_idx), .wr_data(t_dirty), .wr_en(t_wr_dirty1), .rd_data(w_dirty1));   


   //always@(posedge clk)
   //begin
   //if(r_state_cnt != 'd0)
   // begin
   //$display("in state %d at cycle %d", r_state, r_cycle);
   //end
   //if(t_alloc_rob)
   //begin
   //$display("bump rob tail pointer to %d, r_state = %d at cycle %d",
   //		      r_rob_tail_ptr + 'd1, r_state, r_cycle);	     
   //end
   //if(t_pop_rob)
   //begin
   //$display("bump rob head pointer to %d, r_state = %d, at cycle %d",
   //r_rob_head_ptr + 'd1, r_state, r_cycle);	     	     
   //end
   //end // always_ff@ (negedge clk)
   
   // always_comb
   //   begin
   // 	if(r_addr != n_addr) $display("addr change at cycle %d %x -> %x state %d", r_cycle, r_addr, n_addr, r_state);
	
   // 	if(t_wr_tag0)
   // 	  begin
   // 	     $display("----> write tag %x to line %d at cycle %d for way 0 addr %x", r_tag, t_idx, r_cycle, {r_tag, t_idx, 4'd0});
   // 	  end	
   // 	if(t_wr_tag1)
   // 	  begin
   // 	     $display("----> write tag %x to line %d at cycle %d for way 1 addr %x", r_tag, t_idx, r_cycle, , {r_tag, t_idx, 4'd0});
   // 	  end
   //   end

   // always_ff@(posedge clk)
   //   begin
   // 	if(r_state == CHECK_VALID_AND_TAG) 
   // 	  begin
   // 	     $display("w_hit = %b, r_addr = %x", w_hit, r_addr);
   // 	  end
   //   end
   
   
endmodule
