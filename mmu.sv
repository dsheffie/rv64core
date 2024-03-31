module mmu(clk, reset, page_table_root, 
	   l1i_req, l1i_va, l1d_req, l1d_st, l1d_va,
	   mem_req_valid, mem_req_addr, mem_req_data,  mem_req_store,
	   mem_rsp_valid, mem_rsp_data,
	   page_fault, page_dirty, page_executable, page_readable, page_writable,
	   phys_addr, l1d_rsp_valid, l1i_rsp_valid);
   input logic clk;
   input logic reset;
   input logic [63:0] page_table_root;
   input logic l1i_req;
   input logic [63:0] l1i_va;
   input logic l1d_req;
   input logic l1d_st;
   input logic [63:0] l1d_va;   

   output logic	      mem_req_valid;
   output logic [63:0] mem_req_addr;
   output logic [63:0] mem_req_data;   
   output logic	       mem_req_store;

   input logic	       mem_rsp_valid;
   input logic [63:0]  mem_rsp_data;

   output logic	       page_fault;
   output logic	       page_dirty;
   output logic	       page_executable;
   output logic	       page_readable;
   output logic	       page_writable;
   
   
   output logic [63:0] phys_addr;
   output logic	       l1d_rsp_valid;
   output logic	       l1i_rsp_valid;

      
   logic [63:0]	       n_addr, r_addr;
   logic [63:0]	       n_va, r_va, r_pa, n_pa;
   logic	       r_req, n_req;
   logic	       n_page_fault, r_page_fault;
   logic	       n_l1d_rsp_valid, r_l1d_rsp_valid;
   logic	       n_l1i_rsp_valid, r_l1i_rsp_valid;
   logic	       r_do_l1i, n_do_l1i;
   logic	       r_do_l1d, n_do_l1d;
   logic [1:0]	       n_hit_lvl, r_hit_lvl;
   logic	       r_page_dirty, n_page_dirty;
   logic	       n_page_executable, r_page_executable;
   
   assign mem_req_valid = r_req;
   assign mem_req_addr = r_addr;
   assign l1d_rsp_valid = r_l1d_rsp_valid;
   assign l1i_rsp_valid = r_l1i_rsp_valid;
   assign phys_addr = r_pa;
   assign page_fault = r_page_fault;
   assign page_dirty = r_page_dirty;
   assign page_executable = r_page_executable;
   
   typedef enum	logic [3:0] {
			     IDLE,
			     LOAD0,
			     WAIT0,
			     LOAD1,
			     WAIT1,			     
			     LOAD2,
			     WAIT2,
			     WALK_DONE
			     } state_t;
   
   state_t r_state, n_state;
   logic	n_l1i_req, r_l1i_req;
   logic	n_l1d_req, r_l1d_req;

   wire		w_lo_va = (&r_va[63:39]) & (r_va[39] == r_va[38]);
   wire		w_hi_va = (&(~r_va[63:39])) & (r_va[39] == r_va[38]);
   wire		w_bad_va = (w_lo_va | w_hi_va) == 1'b0;

   logic [63:0]	r_cycle;
   
   always_ff@(posedge clk)
     r_cycle <= reset ? 64'd0 : (r_cycle + 64'd1);
	      
	     
   always_comb
     begin
	n_l1i_req = r_l1i_req | l1i_req;
	n_l1d_req = r_l1d_req | l1d_req;
	n_l1d_rsp_valid = 1'b0;
	n_l1i_rsp_valid = 1'b0;
	n_addr = r_addr;
	n_req = 1'b0;
	n_va = r_va;
	n_pa = r_pa;
	n_state = r_state;
	n_page_fault = 1'b0;
	n_page_dirty = 1'b0;
	n_page_executable = 1'b0;
			    
	n_do_l1i = r_do_l1i;
	n_do_l1d = r_do_l1d;
	n_hit_lvl = r_hit_lvl;
	
	case(r_state)
	  IDLE:
	    begin
	       if(n_l1i_req)
		 begin
		    n_state = LOAD0;
		    n_va = l1i_va;
		    n_l1i_req = 1'b0;
		    $display("starting translation for %x", l1i_va);
		    n_do_l1i = 1'b1;
		    n_do_l1d = 1'b0;
		 end
	       else if(n_l1d_req)
		 begin
		    n_state = LOAD0;
		    n_va = l1d_va;
		    n_l1d_req = 1'b0;
		    //$display("starting translation for %x", l1d_va);
		    n_do_l1i = 1'b0;
		    n_do_l1d = 1'b1;
		 end
	    end
	  LOAD0:
	    begin
	       n_addr = page_table_root + {52'd0, r_va[38:30], 3'd0};
	       if(r_do_l1i) $display("r_va = %x, r_va[38:30] = %d, addr %x", r_va, r_va[38:30], n_addr);
	       if(w_bad_va)
		 begin
		    n_state = IDLE;
		    n_page_fault = 1'b1;
		    n_l1i_rsp_valid = r_do_l1i;
		    n_l1d_rsp_valid = r_do_l1d;		    
		 end
	       else
		 begin
		    n_req = 1'b1;
		    n_state = WAIT0;
		 end
	    end
	  WAIT0:
	    begin
	       if(mem_rsp_valid)
		 begin
		    if(r_do_l1i) $display("walker level 0 got %x, cycle %d", mem_rsp_data, r_cycle);
		    n_addr = mem_rsp_data;
		    if(mem_rsp_data[0] == 1'b0)
		      begin
			 n_state = IDLE;
			 n_page_fault = 1'b1;
			 n_l1i_rsp_valid = r_do_l1i;
			 n_l1d_rsp_valid = r_do_l1d;
		      end
		    else if(|mem_rsp_data[3:1])
		      begin
			 n_hit_lvl = 2'd0;
			 n_state = WALK_DONE;
		      end
		    else
		      begin
			 n_state = LOAD1;
		      end
		 end // if (mem_rsp_valid)
	    end // case: WAIT0
	  LOAD1:
	    begin
	       n_addr = {8'd0, r_addr[53:10], 12'd0} + {52'd0, r_va[29:21], 3'd0};
	       if(r_do_l1i) $display("walker level 1 generates address %x", n_addr);
	       n_req = 1'b1;
	       n_state = WAIT1;
	    end
	  WAIT1:
	    begin
	       if(mem_rsp_valid)
		 begin
		    n_addr = mem_rsp_data;
		    if(r_do_l1i) $display("walker level 1 got %x", mem_rsp_data);
		    if(mem_rsp_data[0] == 1'b0)
		      begin
			 n_state = IDLE;
			 n_page_fault = 1'b1;
			 n_l1i_rsp_valid = r_do_l1i;
			 n_l1d_rsp_valid = r_do_l1d;
		      end
		    else if(|mem_rsp_data[3:1])
		      begin
			 n_hit_lvl = 2'd1;			 
			 n_state = WALK_DONE;
		      end
		    else
		      begin
			 n_state = LOAD2;
		      end		    
		 end	       
	    end // case: WAIT1
	  LOAD2:
	    begin
	       n_addr = {8'd0, r_addr[53:10], 12'd0} + {52'd0, r_va[20:12], 3'd0};
	       n_req = 1'b1;
	       n_state = WAIT2;
	    end
	  WAIT2:
	    begin
	       if(mem_rsp_valid)
		 begin
		    if(r_do_l1i) $display("walker level 2 got %x",  mem_rsp_data);
		    n_addr = mem_rsp_data;
		    if(mem_rsp_data[0] == 1'b0)
		      begin
			 n_state = IDLE;
			 n_page_fault = 1'b1;
			 n_l1i_rsp_valid = r_do_l1i;
			 n_l1d_rsp_valid = r_do_l1d;
		      end
		    else
		      begin
			 n_hit_lvl = 2'd2;
			 n_state = WALK_DONE;
		      end		    
		 end	       
	    end
	  WALK_DONE:
	    begin
	       if(r_do_l1i) $display("pa root address %x, hit lvl %d", {8'd0, r_addr[53:10], 12'd0}, r_hit_lvl);
	       if(r_hit_lvl == 2'd0)
		 begin /* 4k page */
		    n_pa = {8'd0, r_addr[53:10], 12'd0};
		 end
	       else if(r_hit_lvl == 2'd1)
		 begin /* 2mbyte page */
		    n_pa = {8'd0, r_addr[53:19], r_va[20:12], 12'd0};
		 end
	       else if(r_hit_lvl == 2'd2)
		 begin /* 1gig page */
		    n_pa = {8'd0, r_addr[53:28], r_va[29:12], 12'd0};
		 end
	       /* can ack now, but need to check if accessed needs to be set */
	       n_l1i_rsp_valid = r_do_l1i;
	       n_l1d_rsp_valid = r_do_l1d;
	       n_page_dirty = r_addr[7];
	       n_page_executable = r_addr[2];
	       
	       if(r_addr[6] == 1'b0)
		 begin
		    $stop();
		 end
	       else
		 begin
		    n_state = IDLE;
		 end
	    end
	  default:
	    begin
	    end
	endcase
     end // always_comb
   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= 'd0;
	     r_addr <= 'd0;
	     r_req <= 1'b0;
	     r_va <= 'd0;
	     r_pa <= 'd0;
	     r_l1i_req <= 1'b0;
	     r_l1d_req <= 1'b0;
	     r_l1i_rsp_valid <= 1'b0;
	     r_l1d_rsp_valid <= 1'b0;
	     r_page_fault <= 1'b0;
	     r_page_dirty <= 1'b0;
	     r_page_executable <= 1'b0;
	     r_do_l1i <= 1'b0;
	     r_do_l1d <= 1'b0;
	     r_hit_lvl <= 2'd0;
	  end
	else
	  begin
	     r_state <= n_state;
	     r_addr <= n_addr;
	     r_req <= n_req;
	     r_va <= n_va;
	     r_pa <= n_pa;
	     r_l1i_req <= n_l1i_req;
	     r_l1d_req <= n_l1d_req;
	     r_l1i_rsp_valid <= n_l1i_rsp_valid;
	     r_l1d_rsp_valid <= n_l1d_rsp_valid;
	     r_page_fault <= n_page_fault;
	     r_page_dirty <= n_page_dirty;
	     r_page_executable <= n_page_executable;
	     r_do_l1i <= n_do_l1i;
	     r_do_l1d <= n_do_l1d;
	     r_hit_lvl <= n_hit_lvl;
	  end
     end
   
   
endmodule
