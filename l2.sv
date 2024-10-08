`include "machine.vh"

module l2(clk,
	  reset,
	  l2_state,
	  l1d_req,
	  l1i_req,
	  l1d_uc,
	  
	  l1d_addr,
	  l1i_addr,
	  l1d_opcode,
	  l1d_rsp_valid,
	  l1i_rsp_valid,
	  
	  l1i_flush_req,
	  l1d_flush_req,

	  l1i_flush_complete,
	  l1d_flush_complete,
	  
	  flush_complete,

	  //l1 -> l2
	  l1_mem_req_ack,
	  l1_mem_req_store_data,

	  //l2 -> l1
	  l1_mem_load_data,

	  //l2 probe l1
	  l2_probe_addr,
	  l2_probe_val,
	  l2_probe_ack,
	  
	  //l2 -> mem
	  mem_req_valid, 
	  mem_req_addr, 
	  mem_req_store_data, 
	  mem_req_opcode,
	  
	  //mem -> l2
	  mem_rsp_valid,
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
   
   input logic l1d_req;
   input logic l1i_req;
   input logic l1d_uc;
   input logic [(`M_WIDTH-1):0] l1d_addr;
   input logic [(`M_WIDTH-1):0] l1i_addr;   
   input logic [3:0] 		l1d_opcode;
   output logic l1d_rsp_valid;
   output logic l1i_rsp_valid;
   
   input logic l1i_flush_req;
   input logic l1d_flush_req;
   input logic l1i_flush_complete;
   input logic l1d_flush_complete;
   
   output logic flush_complete;

   output logic l1_mem_req_ack;
   input logic [(1 << (`LG_L1D_CL_LEN+3)) - 1 :0] l1_mem_req_store_data;

   output logic 				  l2_probe_val;
   output logic [(`M_WIDTH-1):0] 		  l2_probe_addr;
   input logic 					  l2_probe_ack;
   
   output logic [(1 << (`LG_L1D_CL_LEN+3)) - 1 :0] l1_mem_load_data;
   
   output logic mem_req_valid;
   output logic [`M_WIDTH-1:0] mem_req_addr;
   output logic [(1 << (`LG_L2_CL_LEN+3)) - 1 :0] mem_req_store_data;
   output logic [3:0] 	mem_req_opcode;
   
   input logic 		mem_rsp_valid;
   input logic [(1 << (`LG_L2_CL_LEN+3)) - 1 :0] mem_rsp_load_data;

   input logic				  mmu_req_valid;
   input logic [63:0]			  mmu_req_addr;
   input logic [63:0]			  mmu_req_data;
   input logic				  mmu_req_store;
   output logic				  mmu_rsp_valid;
   output logic [63:0]			  mmu_rsp_data;

   logic [63:0] r_mmu_rsp_data, n_mmu_rsp_data;
   logic	r_mmu_rsp_valid, n_mmu_rsp_valid;
   logic	n_mem_mark_rsp_valid, r_mem_mark_rsp_valid;
   
   assign mmu_rsp_valid = r_mmu_rsp_valid;
   assign mmu_rsp_data = r_mmu_rsp_data;
   assign mem_mark_rsp_valid = r_mem_mark_rsp_valid;
   
   input logic	mem_mark_valid;
   input logic	       mem_mark_accessed;
   input logic	       mem_mark_dirty;
   input logic [63:0] mem_mark_addr;
   output logic	      mem_mark_rsp_valid;
   
   output logic [63:0] cache_hits;
   output logic [63:0] cache_accesses;
   
   
   localparam LG_L2_LINES = `LG_L2_NUM_SETS-1;
   localparam L2_LINES = 1<<LG_L2_LINES;
   
   localparam TAG_BITS = `M_WIDTH - (LG_L2_LINES + `LG_L2_CL_LEN);

   logic 		t_wr_dirty, t_wr_valid;
   logic 		t_wr_d0, t_wr_d1, t_wr_d2, t_wr_d3, t_wr_tag;
   
   logic 		t_valid, t_dirty;
   logic [LG_L2_LINES-1:0] t_idx, r_idx;
   logic [TAG_BITS-1:0]    n_tag, r_tag;

   logic [`M_WIDTH-(`LG_L2_CL_LEN+1):0]    n_last_l1i_addr, r_last_l1i_addr;
   logic [`M_WIDTH-(`LG_L2_CL_LEN+1):0]    n_last_l1d_addr, r_last_l1d_addr;
   logic 		   t_gnt_l1i, t_gnt_l1d;
   
   
   logic [`M_WIDTH-1:0]    n_addr, r_addr;
   logic [`M_WIDTH-1:0] 	 n_saveaddr, r_saveaddr;
   
   logic [3:0] 		   n_opcode, r_opcode;

   logic 		   r_mem_req, n_mem_req;
   logic [3:0] 		   r_mem_opcode, n_mem_opcode;
   logic 		   r_req_ack, n_req_ack;
   
   logic 		   r_l1d_rsp_valid, n_l1d_rsp_valid;
   logic 		   r_l1i_rsp_valid, n_l1i_rsp_valid;
   logic [(1 << (`LG_L1D_CL_LEN+3)) - 1:0] 	   r_rsp_data, n_rsp_data;
   logic [(1 << (`LG_L1D_CL_LEN+3)) - 1:0] 	   r_store_data, n_store_data;
   
   logic 		   r_reload, n_reload;
   
   typedef enum logic  {
			     WAIT_FOR_FLUSH,
			     WAIT_FOR_L1_FLUSH_DONE
			     } flush_state_t;

   logic 	r_need_l1i,n_need_l1i,r_need_l1d,n_need_l1d;
   logic 	t_l2_flush_req;
   
   flush_state_t n_flush_state, r_flush_state;
   
   
   typedef enum 	logic [4:0] {
				     INITIALIZE,
				     IDLE,
				     CHECK_VALID_AND_TAG,
				     CLEAN_RELOAD,
				     DIRTY_STORE,
				     STORE_TURNAROUND,
				     WAIT_CLEAN_RELOAD,
				     WAIT_STORE_IDLE,
				     FLUSH_STORE,
				     FLUSH_WAIT,
				     FLUSH_TRIAGE,
				     UPDATE_PTE
				     } state_t;

   state_t n_state, r_state;
   assign l2_state = 4'd0;
   
   logic 		n_flush_complete, r_flush_complete;
   logic 		r_flush_req, n_flush_req;
   logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0] r_mem_req_store_data, n_mem_req_store_data;
   logic [63:0] 	r_cache_hits, n_cache_hits, r_cache_accesses, n_cache_accesses;
   
   assign flush_complete = r_flush_complete;
   assign mem_req_addr = r_addr;
   assign mem_req_valid = r_mem_req;
   assign mem_req_opcode = r_mem_opcode;
   assign mem_req_store_data = r_mem_req_store_data;
   
   assign l1d_rsp_valid = r_l1d_rsp_valid;
   assign l1i_rsp_valid = r_l1i_rsp_valid;
   
   assign l1_mem_load_data = r_rsp_data;
   assign l1_mem_req_ack = r_req_ack;
   
   assign cache_hits = r_cache_hits;
   assign cache_accesses = r_cache_accesses;
   
     
   logic [127:0] 	t_d0;
   wire [127:0] 	w_d0;
   wire [TAG_BITS-1:0] 	w_tag0;
   wire 		w_valid0, w_dirty0;

   
   reg_ram1rw #(.WIDTH(128), .LG_DEPTH(LG_L2_LINES)) data_ram0
     (.clk(clk), .addr(t_idx), .wr_data(t_d0), .wr_en(t_wr_d0), .rd_data(w_d0));
      
   reg_ram1rw #(.WIDTH(TAG_BITS), .LG_DEPTH(LG_L2_LINES)) tag_ram0
     (.clk(clk), .addr(t_idx), .wr_data(r_tag), .wr_en(t_wr_tag), .rd_data(w_tag0));   
   
   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) valid_ram0
     (.clk(clk), .addr(t_idx), .wr_data(t_valid), .wr_en(t_wr_valid), .rd_data(w_valid0));   

   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) dirty_ram0
     (.clk(clk), .addr(t_idx), .wr_data(t_dirty), .wr_en(t_wr_dirty), .rd_data(w_dirty0));   

   wire 		w_hit = w_valid0 ? (r_tag == w_tag0) : 1'b0;
   wire 		w_need_wb = w_valid0 ? w_dirty0 : 1'b0;
   
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
   
   
      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mmu_addr3 <= 1'b0;
	     r_mmu <= 1'b0;
	     r_mark_pte <= 1'b0;
	     r_mmu_rsp_data <= 'd0;
	     r_mmu_rsp_valid <= 1'b0;
	     r_mem_mark_rsp_valid <= 1'b0;
	     r_state <= INITIALIZE;
	     r_flush_state <= WAIT_FOR_FLUSH;
	     r_flush_complete <= 1'b0;
	     r_idx <= 'd0;
	     r_tag <= 'd0;
	     r_opcode <= 4'd0;
	     r_addr <= 'd0;
	     r_saveaddr <= 'd0;
	     r_mem_req <= 1'b0;
	     r_mem_opcode <= 4'd0;
	     r_rsp_data <= 'd0;
	     r_l1d_rsp_valid <= 1'b0;
	     r_l1i_rsp_valid <= 1'b0;
	     r_reload <= 1'b0;
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
	  end
	else
	  begin
	     r_mmu_addr3 <= n_mmu_addr3;
	     r_mmu <= n_mmu;
	     r_mark_pte <= n_mark_pte;
	     r_mmu_rsp_data <= n_mmu_rsp_data;
	     r_mmu_rsp_valid <= n_mmu_rsp_valid;
	     r_mem_mark_rsp_valid <= n_mem_mark_rsp_valid;
	     r_state <= n_state;
	     r_flush_state <= n_flush_state;
	     r_flush_complete <= n_flush_complete;
	     r_idx <= t_idx;
	     r_tag <= n_tag;
	     r_opcode <= n_opcode;
	     r_addr <= n_addr;
	     r_saveaddr <= n_saveaddr;
	     r_mem_req <= n_mem_req;
	     r_mem_opcode <= n_mem_opcode;
	     r_rsp_data <= n_rsp_data;
	     r_l1d_rsp_valid <= n_l1d_rsp_valid;
	     r_l1i_rsp_valid <= n_l1i_rsp_valid;
	     r_reload <= n_reload;
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
		    //$display("-> got flush req at cycle %d, n_need_l1d = %b, n_need_l1i = %b", r_cycle, n_need_l1d, n_need_l1i);
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
   logic [(`M_WIDTH-1):0] r_l2_probe_addr, n_l2_probe_addr;
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
		    //$display("got probe req at cycle %d for addr %x", r_cycle, mmu_req_addr);
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
		    //$display("got probe ack at cycle %d", r_cycle);
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

   wire w_l1i_req = r_l1i_req | l1i_req;
   wire w_l1d_req = r_l1d_req | l1d_req;
   wire	w_mmu_req = r_mmu_req | t_probe_mmu_req_valid;
   wire w_mem_mark_valid = mem_mark_valid | r_mmu_mark_req;
   
   always_comb
     begin
	n_last_gnt = r_last_gnt;
	n_l1i_req = r_l1i_req | l1i_req;
	n_l1d_req = r_l1d_req | l1d_req;
	n_mmu_req = r_mmu_req | t_probe_mmu_req_valid;
	n_mmu_mark_req = mem_mark_valid | r_mmu_mark_req;
	n_mem_mark_rsp_valid = 1'b0;
	n_req = r_req;
	n_mmu_rsp_data = r_mmu_rsp_data;
	n_mmu_rsp_valid = 1'b0;
	n_mmu_addr3 = r_mmu_addr3;
	n_mmu = r_mmu;
	n_mark_pte = r_mark_pte;
	
	n_state = r_state;
	n_flush_complete = 1'b0;
	t_wr_valid = 1'b0;
	t_wr_dirty = 1'b0;
	t_wr_tag = 1'b0;
	t_wr_d0 = 1'b0;
	
	t_idx = r_idx;
	n_tag = r_tag;
	n_opcode = r_opcode;
	n_addr = r_addr;
	n_saveaddr = r_saveaddr;
	
	n_req_ack = 1'b0;
	n_mem_req = r_mem_req;
	n_mem_opcode = r_mem_opcode;
		
	t_valid = 1'b0;
	t_dirty = 1'b0;

	t_d0 = mem_rsp_load_data[127:0];

	n_rsp_data = r_rsp_data;
	n_l1i_rsp_valid = 1'b0;
	n_l1d_rsp_valid = 1'b0;	

	n_reload = r_reload;
	n_store_data = r_store_data;
	n_flush_req = r_flush_req | t_l2_flush_req;
	n_mem_req_store_data = r_mem_req_store_data;

	n_cache_hits = r_cache_hits;
	n_cache_accesses = r_cache_accesses;

	n_last_l1i_addr = r_last_l1i_addr;
	n_last_l1d_addr = r_last_l1d_addr;

	t_gnt_l1i = 1'b0;
	t_gnt_l1d = 1'b0;
	
	n_mmu_mark_dirty = r_mmu_mark_dirty;
	n_mmu_mark_accessed = r_mmu_mark_accessed;
	
	case(r_state)
	  INITIALIZE:
	    begin
	       t_valid = 1'b0;
	       t_dirty = 1'b0;
	       
	       t_wr_valid = 1'b1;
	       t_wr_dirty = 1'b1;
	       t_wr_tag = 1'b1;
	       t_wr_d0 = 1'b1;
	       
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
	       
	       if(n_flush_req)
		 begin
		    t_idx = 'd0;
		    n_state = FLUSH_WAIT;
		 end
	       else if(w_mem_mark_valid)
		 begin
		    n_mmu_mark_req = 1'b0;
		    n_mmu_mark_dirty = mem_mark_dirty;
		    n_mmu_mark_accessed = mem_mark_accessed;
		    n_mmu_addr3 = mem_mark_addr[3];		    
		    t_idx = mem_mark_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
		    n_tag = mem_mark_addr[(`M_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
		    n_addr = {mem_mark_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
		    n_saveaddr = {mem_mark_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
		    n_opcode = MEM_LW;
		    n_mark_pte = 1'b1;
		    n_state = CHECK_VALID_AND_TAG;
		 end
	       else if(w_mmu_req)
		 begin
		    n_mmu_addr3 = mmu_req_addr[3];		    
		    t_idx = mmu_req_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
		    n_tag = mmu_req_addr[(`M_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
		    n_addr = {mmu_req_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
		    n_saveaddr = {mmu_req_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
		    n_opcode = MEM_LW;
		    n_state = CHECK_VALID_AND_TAG;
		    n_mmu = 1'b1;
		    //$display("l2 : mmu req addr %x, w_l1d_req = %b, w_l1i_req = %b", r_addr, w_l1d_req, w_l1i_req);		    
		 end
	       else if(w_l1d_req | w_l1i_req)
		 begin
		    if(w_l1i_req & (!w_l1d_req))
		      begin
			 //$display("accepting i-side, addr=%x", l1i_addr);			 
			 n_last_gnt = 1'b0;			 
			 t_idx = l1i_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];
			 n_tag = l1i_addr[(`M_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			 n_last_l1i_addr = l1i_addr[(`M_WIDTH-1):`LG_L2_CL_LEN];
			 n_addr = {l1i_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_saveaddr = {l1i_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_opcode = MEM_LW;
			 n_l1i_req = 1'b0;
			 t_gnt_l1i = 1'b1;
		      end
		    else if((!w_l1i_req) & w_l1d_req)
		      begin
			 //$display("accepting d-side, addr = %x, store=%b", l1d_addr, l1d_opcode == MEM_SW);
			 n_last_gnt = 1'b1;
			 t_idx = l1d_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 
			 n_tag = l1d_addr[(`M_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			 n_addr = {l1d_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_last_l1d_addr = l1d_addr[(`M_WIDTH-1):`LG_L2_CL_LEN];			 
			 n_saveaddr = {l1d_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			 n_store_data = l1_mem_req_store_data;
			 n_opcode = l1d_opcode;
			 n_l1d_req = 1'b0;
			 
			 if(!(l1d_opcode == MEM_SW || l1d_opcode == MEM_LW))
			   begin
			      $display("opcode is %d", l1d_opcode);
			      $stop();
			   end
			 
			 
			 if(l1d_opcode == MEM_SW)
			   begin
			      n_l1d_rsp_valid = 1'b1;
			   end
			 t_gnt_l1d = 1'b1;
		      end
		    else
		      begin
			 if(r_last_gnt)
			   begin
			      //$display("accepting i-side, addr=%x", l1i_addr);			 			      
			      n_last_gnt = 1'b0;			 
			      t_idx = l1i_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 			      
			      n_tag = l1i_addr[(`M_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			      n_last_l1i_addr = l1i_addr[(`M_WIDTH-1):`LG_L2_CL_LEN];
			      n_addr = {l1i_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_saveaddr = {l1i_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_opcode = MEM_LW;
			      n_l1i_req = 1'b0;	
			      t_gnt_l1i = 1'b1;
			   end
			 else
			   begin
			      //$display("accepting d-side, addr = %x, store=%b", l1d_addr, l1d_opcode == MEM_SW);
			      n_last_gnt = 1'b1;
			      t_idx = l1d_addr[LG_L2_LINES+(`LG_L2_CL_LEN-1):`LG_L2_CL_LEN];			 			      			      
			      n_tag = l1d_addr[(`M_WIDTH-1):LG_L2_LINES+`LG_L2_CL_LEN];
			      n_addr = {l1d_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_last_l1d_addr = l1d_addr[(`M_WIDTH-1):`LG_L2_CL_LEN];
			      n_saveaddr = {l1d_addr[(`M_WIDTH-1):`LG_L2_CL_LEN], {{`LG_L2_CL_LEN{1'b0}}}};
			      n_store_data = l1_mem_req_store_data;
			      n_opcode = l1d_opcode;
			      n_l1d_req = 1'b0;

			      if(!(l1d_opcode == MEM_SW || l1d_opcode == MEM_LW))
				$stop();	
			      
			      if(l1d_opcode == MEM_SW)
				begin
				   n_l1d_rsp_valid = 1'b1;
				end
			      t_gnt_l1d = 1'b1;
			   end
		      end
		    n_req_ack = 1'b1;
		    n_state = CHECK_VALID_AND_TAG;
		    n_cache_accesses = r_cache_accesses + 64'd1;
		    n_cache_hits = r_cache_hits + 64'd1;
		 end
	    end // case: IDLE
	  CHECK_VALID_AND_TAG:
	    begin
	       //load hit
	       if(w_hit)
		 begin
		    n_reload = 1'b0;
		    if(r_opcode == 4'd4)
		      begin			 
			 n_rsp_data = w_d0;			 
			 if(r_mmu)
			   begin
			      n_mmu_rsp_data = r_mmu_addr3 ? w_d0[127:64] : w_d0[63:0];
			      n_mmu_rsp_valid = 1'b1;
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
			      t_d0 = r_mmu_addr3 ? 
				     {w_d0[127:72], r_mmu_mark_dirty|w_d0[71], r_mmu_mark_accessed|w_d0[70], w_d0[69:0]} :
			             {w_d0[127:8], r_mmu_mark_dirty|w_d0[7], r_mmu_mark_accessed|w_d0[6], w_d0[5:0]};
			      t_wr_dirty = 1'b1;
			      t_dirty = 1'b1;
			      t_wr_d0 = 1'b1;
			      n_mark_pte = 1'b0;
			      n_mem_mark_rsp_valid = 1'b1;
			   end
			 else if(r_last_gnt == 1'b0)
			   begin
			      n_l1i_rsp_valid  = 1'b1;
			      n_state = IDLE;
			   end
			 else
			   begin
			      n_l1d_rsp_valid  = 1'b1;
			      n_state = IDLE;
			   end
			 //$display("cycle %d : ack'd for address %x, n_l1i_req = %b, n_l1d_req = %b, n_l1i_rsp_valid =%b, n_l1d_rsp_valid = %b", 
			 //r_cycle, r_addr, n_l1i_req, n_l1d_req, n_l1i_rsp_valid,n_l1d_rsp_valid);			 
		      end
		    else if(r_opcode == 4'd7)
		      begin
			 t_wr_dirty = 1'b1;
			 t_dirty = 1'b1;
			 n_state = WAIT_STORE_IDLE;
			 //n_cache_hits = r_cache_hits + 64'd1;			 
			 t_d0 = r_store_data;
			 t_wr_d0 = 1'b1;
		      end
		 end
	       else
		 begin
		    n_cache_hits = r_cache_hits - 64'd1;			 		    
		    if(w_dirty0)
		      begin
			 n_mem_req_store_data = w_d0;
			 n_addr = {w_tag0, t_idx, {{`LG_L2_CL_LEN{1'b0}}}};
			 n_mem_opcode = 4'd7; 
			 n_mem_req = 1'b1;
			 n_state = DIRTY_STORE;			 
		      end
		    else //invalid or clean
		      begin
			 if(r_reload)
			   $stop();
			 n_reload = 1'b1;
			 n_state = CLEAN_RELOAD;
			 n_mem_opcode = 4'd4; //load
			 n_mem_req = 1'b1;
		      end
		 end
	    end // case: CHECK_VALID_AND_TAG
	  DIRTY_STORE:
	    begin
	       if(mem_rsp_valid)
		 begin
		    n_addr = r_saveaddr;
		    n_mem_opcode = 4'd4; //load
		    n_state = STORE_TURNAROUND;
		    n_mem_req = 1'b0;		    
		 end
	    end // case: DIRTY_STORE
	  STORE_TURNAROUND:
	    begin
	       n_state = CLEAN_RELOAD;
	       n_reload = 1'b1;
	       n_mem_req = 1'b1;		    
	    end
	  CLEAN_RELOAD:
	    begin
	       if(mem_rsp_valid)
		 begin
		    n_mem_req = 1'b0;
		    t_valid = 1'b1;
		    t_wr_valid = 1'b1;
		    t_wr_tag = 1'b1;
		    t_wr_d0 = 1'b1;
		    n_state = WAIT_CLEAN_RELOAD;
		 end
	    end // case: CLEAN_RELOAD
	  WAIT_CLEAN_RELOAD: /* need a cycle to turn around */
	    begin
	       n_state = CHECK_VALID_AND_TAG;
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
	       t_wr_valid = 1'b1;
	       t_wr_dirty = 1'b1;
	    end
	  FLUSH_TRIAGE:
	    begin
	       //if(w_need_wb)
	       //$display("address = %x needs wb", {w_tag0, t_idx, 6'd0});
	       
	       if(w_need_wb)
		 begin
		    n_mem_req_store_data = w_d0;
		    n_addr = {w_tag0, t_idx, {{`LG_L2_CL_LEN{1'b0}}}};
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
		    n_mem_req = 1'b0;
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
	    end
	  default:
	    begin
	    end
	endcase
     end
   
endmodule
