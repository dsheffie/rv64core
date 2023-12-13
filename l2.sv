`include "machine.vh"

module l2(clk,
	  reset,

	  l1d_req,
	  l1i_req,

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

	  //l2 -> mem
	  mem_req_valid, 
	  mem_req_addr, 
	  mem_req_store_data, 
	  mem_req_opcode,
	  
	  //mem -> l2
	  mem_rsp_valid,
	  mem_rsp_load_data,

	  cache_hits,
	  cache_accesses
	  
	  );

   input logic clk;
   input logic reset;
   input logic l1d_req;
   input logic l1i_req;
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
   input logic [127:0] l1_mem_req_store_data;

   output logic [127:0] l1_mem_load_data;
   
   output logic mem_req_valid;
   output logic [31:0] mem_req_addr;
   output logic [511:0] mem_req_store_data;
   output logic [3:0] 	mem_req_opcode;
   
   input logic 		mem_rsp_valid;
   input logic [511:0] 	mem_rsp_load_data;

   output logic [63:0] cache_hits;
   output logic [63:0] cache_accesses;
   
   
   localparam LG_L2_LINES = `LG_L2_NUM_SETS;
   localparam L2_LINES = 1<<LG_L2_LINES;
   
   localparam TAG_BITS = `M_WIDTH - (LG_L2_LINES + 6);

   logic 		t_wr_dirty, t_wr_valid;
   logic 		t_wr_d0, t_wr_d1, t_wr_d2, t_wr_d3, t_wr_tag;
   
   logic 		t_valid, t_dirty;
   logic [LG_L2_LINES-1:0] t_idx, r_idx;
   logic [TAG_BITS-1:0]    n_tag, r_tag;

   logic [`M_WIDTH-7:0]    n_last_l1i_addr, r_last_l1i_addr;
   logic [`M_WIDTH-7:0]    n_last_l1d_addr, r_last_l1d_addr;
   logic 		   t_gnt_l1i, t_gnt_l1d;
   
   
   logic [1:0] 		 n_bank, r_bank;
   logic [31:0] 	 n_addr, r_addr;
   logic [31:0] 	 n_saveaddr, r_saveaddr;
   
   logic [3:0] 		   n_opcode, r_opcode;

   logic 		   r_mem_req, n_mem_req;
   logic [3:0] 		   r_mem_opcode, n_mem_opcode;
   logic 		   r_req_ack, n_req_ack;
   
   logic 		   r_l1d_rsp_valid, n_l1d_rsp_valid;
   logic 		   r_l1i_rsp_valid, n_l1i_rsp_valid;
   logic [127:0] 	   r_rsp_data, n_rsp_data;
   logic [127:0] 	   r_store_data, n_store_data;
   
   logic 		   r_reload, n_reload;
   
   typedef enum logic  {
			     WAIT_FOR_FLUSH,
			     WAIT_FOR_L1_FLUSH_DONE
			     } flush_state_t;

   logic 	r_need_l1i,n_need_l1i,r_need_l1d,n_need_l1d;
   logic 	t_l2_flush_req;
   
   flush_state_t n_flush_state, r_flush_state;
   
   
   typedef enum 	logic [3:0] {
				     INITIALIZE,
				     IDLE,
				     WAIT_FOR_RAM,
				     CHECK_VALID_AND_TAG,
				     CLEAN_RELOAD,
				     DIRTY_STORE,
				     STORE_TURNAROUND,
				     WAIT_CLEAN_RELOAD,
				     WAIT_STORE_IDLE,
				     FLUSH_STORE,
				     FLUSH_WAIT,
				     FLUSH_TRIAGE
				     } state_t;

   state_t n_state, r_state;
   logic 		n_flush_complete, r_flush_complete;
   logic 		r_flush_req, n_flush_req;
   logic [511:0] 	r_mem_req_store_data, n_mem_req_store_data;
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
   
     
   logic [127:0] 	t_d0, t_d1, t_d2, t_d3;
      
   wire [127:0] 	w_d0, w_d1, w_d2, w_d3;
   wire [TAG_BITS-1:0] 	w_tag;
   wire 		w_valid, w_dirty;

   
   reg_ram1rw #(.WIDTH(128), .LG_DEPTH(LG_L2_LINES)) data_ram0
     (.clk(clk), .addr(t_idx), .wr_data(t_d0), .wr_en(t_wr_d0), .rd_data(w_d0));

   reg_ram1rw #(.WIDTH(128), .LG_DEPTH(LG_L2_LINES)) data_ram1
     (.clk(clk), .addr(t_idx), .wr_data(t_d1), .wr_en(t_wr_d1), .rd_data(w_d1));

   reg_ram1rw #(.WIDTH(128), .LG_DEPTH(LG_L2_LINES)) data_ram2
     (.clk(clk), .addr(t_idx), .wr_data(t_d2), .wr_en(t_wr_d2), .rd_data(w_d2));

   reg_ram1rw #(.WIDTH(128), .LG_DEPTH(LG_L2_LINES)) data_ram3
     (.clk(clk), .addr(t_idx), .wr_data(t_d3), .wr_en(t_wr_d3), .rd_data(w_d3));
      
   reg_ram1rw #(.WIDTH(TAG_BITS), .LG_DEPTH(LG_L2_LINES)) tag_ram
     (.clk(clk), .addr(t_idx), .wr_data(r_tag), .wr_en(t_wr_tag), .rd_data(w_tag));   
   
   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) valid_ram
     (.clk(clk), .addr(t_idx), .wr_data(t_valid), .wr_en(t_wr_valid), .rd_data(w_valid));   

   reg_ram1rw #(.WIDTH(1), .LG_DEPTH(LG_L2_LINES)) dirty_ram
     (.clk(clk), .addr(t_idx), .wr_data(t_dirty), .wr_en(t_wr_dirty), .rd_data(w_dirty));   

   wire 		w_hit = w_valid ? (r_tag == w_tag) : 1'b0;
   wire 		w_need_wb = w_valid ? w_dirty : 1'b0;

   logic 		r_l1d_req, n_l1d_req;
   logic 		r_l1i_req, n_l1i_req;
   logic 		r_last_gnt, n_last_gnt;
   logic 		n_req, r_req;


      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= INITIALIZE;
	     r_flush_state <= WAIT_FOR_FLUSH;
	     r_flush_complete <= 1'b0;
	     r_idx <= 'd0;
	     r_tag <= 'd0;
	     r_bank <= 2'd0;
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
	     r_last_gnt <= 1'b0;
	     r_req <= 1'b0;
	     r_last_l1i_addr <= 'd0;
	     r_last_l1d_addr <= 'd0;
	  end
	else
	  begin
	     r_state <= n_state;
	     r_flush_state <= n_flush_state;
	     r_flush_complete <= n_flush_complete;
	     r_idx <= t_idx;
	     r_tag <= n_tag;
	     r_bank <= n_bank;
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
	     r_last_gnt <= n_last_gnt;
	     r_req <= n_req;
	     r_last_l1i_addr <= n_last_l1i_addr;
	     r_last_l1d_addr <= n_last_l1d_addr;	     
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


   logic [31:0] r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end

   //always_ff@(negedge clk)
   //begin
      //$display("cycle %d : L2 cache FSM in state %d", r_cycle, r_state);
      //if(l1i_req) $display("cycle %d : l1i req, state %d", r_cycle, r_state);
      //if(l1d_req) $display("cycle %d : l1d req, state %d", r_cycle, r_state);
   //end

   wire w_l1i_req = r_l1i_req | l1i_req;
   wire w_l1d_req = r_l1d_req | l1d_req;

   // always_ff@(negedge clk)
   //   begin
   // 	if(t_gnt_l1i)
   // 	  begin
   // 	     $display("l1i curr tag %x, last tag %x at cycle %d", 
   // 		      {n_addr[31:6],6'd0}, 
   // 		      {r_last_l1i_addr,6'd0},
   // 		      r_cycle);
   // 	  end
   //   end

   
   always_comb
     begin
	n_last_gnt = r_last_gnt;
	n_l1i_req = r_l1i_req | l1i_req;
	n_l1d_req = r_l1d_req | l1d_req;
	n_req = r_req;
	
	
	n_state = r_state;
	n_flush_complete = 1'b0;
	t_wr_valid = 1'b0;
	t_wr_dirty = 1'b0;
	t_wr_d0 = 1'b0;
	t_wr_d1 = 1'b0;
	t_wr_d2 = 1'b0;
	t_wr_d3 = 1'b0;
	t_wr_tag = 1'b0;
	
	t_idx = r_idx;
	n_tag = r_tag;
	n_bank = r_bank;
	n_opcode = r_opcode;
	n_addr = r_addr;
	n_saveaddr = r_saveaddr;
	
	n_req_ack = 1'b0;
	n_mem_req = r_mem_req;
	n_mem_opcode = r_mem_opcode;
		
	t_valid = 1'b0;
	t_dirty = 1'b0;

	t_d0 = mem_rsp_load_data[127:0];
	t_d1 = mem_rsp_load_data[255:128];
	t_d2 = mem_rsp_load_data[383:256];
	t_d3 = mem_rsp_load_data[511:384];

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
	
	case(r_state)
	  INITIALIZE:
	    begin
	       t_valid = 1'b0;
	       t_dirty = 1'b0;
	       
	       t_wr_valid = 1'b1;
	       t_wr_dirty = 1'b1;
	       
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
	       n_bank = r_bank;
	       n_addr = r_addr;
	       
	       n_opcode = MEM_LW;
	       n_store_data = r_store_data;
	       
	       if(n_flush_req)
		 begin
		    t_idx = 'd0;
		    n_state = FLUSH_WAIT;
		 end
	       else if(w_l1d_req | w_l1i_req)
		 begin
		    if(w_l1i_req & (!w_l1d_req))
		      begin
			 //$display("accepting i-side, addr=%x", 
			 //l1i_addr);			 
			 n_last_gnt = 1'b0;			 
			 t_idx = l1i_addr[LG_L2_LINES+5:6];
			 n_tag = l1i_addr[31:LG_L2_LINES+6];
			 n_last_l1i_addr = l1i_addr[31:6];
			 n_bank = l1i_addr[5:4];
			 n_addr = {l1i_addr[31:6], 6'd0};
			 n_saveaddr = {l1i_addr[31:6], 6'd0};
			 n_opcode = MEM_LW;
			 n_l1i_req = 1'b0;
			 t_gnt_l1i = 1'b1;
		      end
		    else if((!w_l1i_req) & w_l1d_req)
		      begin
			 //$display("accepting d-side, addr = %x, store=%b", 
			 //l1d_addr, l1d_opcode == MEM_SW);
			 n_last_gnt = 1'b1;
			 t_idx = l1d_addr[LG_L2_LINES+5:6];
			 n_tag = l1d_addr[31:LG_L2_LINES+6];
			 n_bank = l1d_addr[5:4];
			 n_addr = {l1d_addr[31:6], 6'd0};
			 n_last_l1d_addr = l1d_addr[31:6];			 
			 n_saveaddr = {l1d_addr[31:6], 6'd0};
			 n_store_data = l1_mem_req_store_data;
			 n_opcode = l1d_opcode;
			 n_l1d_req = 1'b0;			 
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
			      n_last_gnt = 1'b0;			 
			      t_idx = l1i_addr[LG_L2_LINES+5:6];
			      n_tag = l1i_addr[31:LG_L2_LINES+6];
			      n_bank = l1i_addr[5:4];
			      n_last_l1i_addr = l1i_addr[31:6];
			      n_addr = {l1i_addr[31:6], 6'd0};
			      n_saveaddr = {l1i_addr[31:6], 6'd0};
			      n_opcode = MEM_LW;
			      n_l1i_req = 1'b0;	
			      t_gnt_l1i = 1'b1;
			   end
			 else
			   begin
			      n_last_gnt = 1'b1;
			      t_idx = l1d_addr[LG_L2_LINES+5:6];
			      n_tag = l1d_addr[31:LG_L2_LINES+6];
			      n_bank = l1d_addr[5:4];
			      n_addr = {l1d_addr[31:6], 6'd0};
			      n_last_l1d_addr = l1d_addr[31:6];
			      n_saveaddr = {l1d_addr[31:6], 6'd0};
			      n_store_data = l1_mem_req_store_data;
			      n_opcode = l1d_opcode;
			      n_l1d_req = 1'b0;			 			      
			      if(l1d_opcode == MEM_SW)
				begin
				   n_l1d_rsp_valid = 1'b1;
				end
			      t_gnt_l1d = 1'b1;
			   end
		      end
		    n_req_ack = 1'b1;
		    n_state = WAIT_FOR_RAM;
		    n_cache_accesses = r_cache_accesses + 64'd1;
		    n_cache_hits = r_cache_hits + 64'd1;
		 end
	    end
	  WAIT_FOR_RAM:
	    begin
	       n_state = CHECK_VALID_AND_TAG;
	    end // case: WAIT_FOR_RAM
	
	  CHECK_VALID_AND_TAG:
	    begin
	       //load hit
	       if(w_hit)
		 begin
		    n_reload = 1'b0;
		    if(r_opcode == 4'd4)
		      begin			 
			 n_rsp_data = r_bank == 'd0 ? w_d0 :
				      r_bank == 'd1 ? w_d1 :
				      r_bank == 'd2 ? w_d2 :
				      w_d3;
			 n_state = IDLE;
			 if(r_last_gnt == 1'b0)
			   begin
			      n_l1i_rsp_valid  = 1'b1;
			   end
			 else
			   begin
			      n_l1d_rsp_valid  = 1'b1;
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
			 if(r_bank == 'd0)
			   begin
			      t_d0 = r_store_data;
			      t_wr_d0 = 1'b1;
			   end
			 else if(r_bank == 'd1)
			   begin
			      t_d1 = r_store_data;
			      t_wr_d1 = 1'b1;
			   end
			 else if(r_bank == 'd2)
			   begin
			      t_d2 = r_store_data;			      
			      t_wr_d2 = 1'b1;
			   end
			 else
			   begin
			      t_d3 = r_store_data;			      
			      t_wr_d3 = 1'b1;
			   end
		      end
		 end
	       else
		 begin
		    n_cache_hits = r_cache_hits - 64'd1;			 		    
		    if(w_dirty)
		      begin
			 n_mem_req_store_data = {w_d3, w_d2, w_d1, w_d0};
			 n_addr = {w_tag, t_idx, 6'd0};
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
		    t_wr_d1 = 1'b1;
		    t_wr_d2 = 1'b1;
		    t_wr_d3 = 1'b1;
		    n_state = WAIT_CLEAN_RELOAD;
		 end
	    end // case: CLEAN_RELOAD
	  WAIT_CLEAN_RELOAD: /* need a cycle to turn around */
	    begin
	       n_state = WAIT_FOR_RAM;
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
	       //$display("address = %x needs wb", {w_tag, t_idx, 6'd0});
	       
	       if(w_need_wb)
		 begin
		    n_mem_req_store_data = {w_d3, w_d2, w_d1, w_d0};
		    n_addr = {w_tag, t_idx, 6'd0};
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
