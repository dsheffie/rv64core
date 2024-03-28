module mmu(clk, reset, page_table_root, 
	   l1i_req, l1i_va, l1d_req, l1d_st, l1d_va,
	   mem_req_valid, mem_req_addr, mem_req_data,  mem_req_store,
	   mem_rsp_valid, mem_rsp_data,
	   page_fault, l1d_rsp_valid, l1i_rsp_valid);
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
   output logic	       l1d_rsp_valid;
   output logic	       l1i_rsp_valid;
   
   
   
      
   logic [63:0]	       n_addr, r_addr;
   logic [63:0]	       n_va, r_va;
   logic	       r_req, n_req;
   logic	       n_page_fault, r_page_fault;
   logic	       n_l1d_rsp_valid, r_l1d_rsp_valid;
   logic	       n_l1i_rsp_valid, r_l1i_rsp_valid;
   logic	       r_do_l1i, n_do_l1i;
   logic	       r_do_l1d, n_do_l1d;
   
   assign mem_req_valid = r_req;
   assign mem_req_addr = r_addr;
   assign l1d_rsp_valid = r_l1d_rsp_valid;
   assign l1i_rsp_valid = r_l1i_rsp_valid;
   
   assign page_fault = r_page_fault;
   
   typedef enum	logic [3:0] {
			     IDLE,
			     LOAD0,
			     WAIT0,
			     LOAD1,
			     WAIT1,			     
			     LOAD2,
			     WAIT2,
			     DONE
			     } state_t;
   
   state_t r_state, n_state;
   logic	n_l1i_req, r_l1i_req;
   logic	n_l1d_req, r_l1d_req;

   
   always_comb
     begin
	n_l1i_req = r_l1i_req | l1i_req;
	n_l1d_req = r_l1d_req | l1d_req;
	n_l1d_rsp_valid = 1'b0;
	n_l1i_rsp_valid = 1'b0;
	n_addr = r_addr;
	n_req = 1'b0;
	n_va = r_va;
	n_state = r_state;
	n_page_fault = 1'b0;
	n_do_l1i = r_do_l1i;
	n_do_l1d = r_do_l1d;
	
	case(r_state)
	  IDLE:
	    begin
	       if(n_l1i_req)
		 begin
		    n_state = LOAD0;
		    n_va = l1i_va;
		    n_do_l1i = 1'b1;
		    n_do_l1d = 1'b0;
		 end
	       else if(n_l1d_req)
		 begin
		    $stop();
		 end
	    end
	  LOAD0:
	    begin
	       //$display("r_va = %x, r_va[38:30] = %d", r_va, r_va[38:30]);
	       n_addr = page_table_root + {52'd0, r_va[38:30], 3'd0};
	       n_req = 1'b1;
	       n_state = WAIT0;
	    end
	  WAIT0:
	    begin
	       if(mem_rsp_valid)
		 begin
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
			 n_state = DONE;
		      end
		    else
		      begin
			 n_state = LOAD1;
		      end
		 end // if (mem_rsp_valid)
	    end // case: WAIT0
	  LOAD1:
	    begin
	       n_addr = r_addr + {52'd0, r_va[29:21], 3'd0};
	       n_req = 1'b1;
	       n_state = WAIT1;
	    end
	  WAIT1:
	    begin

	    end
	  DONE:
	    begin

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
	     r_l1i_req <= 1'b0;
	     r_l1d_req <= 1'b0;
	     r_l1i_rsp_valid <= 1'b0;
	     r_l1d_rsp_valid <= 1'b0;
	     r_page_fault <= 1'b0;
	     r_do_l1i <= 1'b0;
	     r_do_l1d <= 1'b0;
	  end
	else
	  begin
	     r_state <= n_state;
	     r_addr <= n_addr;
	     r_req <= n_req;
	     r_va <= n_va;
	     r_l1i_req <= n_l1i_req;
	     r_l1d_req <= n_l1d_req;
	     r_l1i_rsp_valid <= n_l1i_rsp_valid;
	     r_l1d_rsp_valid <= n_l1d_rsp_valid;
	     r_page_fault <= n_page_fault;
	     r_do_l1i <= n_do_l1i;
	     r_do_l1d <= n_do_l1d;
	  end
     end
   
   
endmodule
