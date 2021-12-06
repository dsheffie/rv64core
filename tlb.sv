`include "rob.vh"

module tlb(clk,
	   reset,
	   iside_req,
	   dside_req,
	   iside_paddr,
	   dside_paddr,
	   iside_rsp_valid,
	   dside_rsp_valid,
	   tlb_rsp,
	   iside_tlb_miss,
	   dside_tlb_miss,
	   tlb_hit);
   input logic clk;
   input logic reset;
   
   input logic iside_req;
   input logic dside_req;
   input logic [`M_WIDTH-`LG_PG_SZ-1:0] iside_paddr;
   input logic [`M_WIDTH-`LG_PG_SZ-1:0] dside_paddr;

   output logic 			iside_rsp_valid;
   output logic 			dside_rsp_valid;
   output 				utlb_entry_t tlb_rsp;
   output logic 			iside_tlb_miss;
   output logic 			dside_tlb_miss;
   output logic 			tlb_hit;
   
   typedef enum logic [1:0] {IDLE = 'd0,
			     ISIDE = 'd1,
			     DSIDE = 'd2
			     } state_t;

   
   state_t n_state, r_state;
   
   logic 				r_iside_rsp_valid;
   logic 				r_dside_rsp_valid;
   logic 				n_iside_rsp_valid;
   logic 				n_dside_rsp_valid;      

   logic 				r_got_iside, r_got_dside;
   logic 				n_got_iside, n_got_dside;
   logic 				r_tlb_hit, n_tlb_hit;
   logic 				r_iside_tlb_miss,n_iside_tlb_miss;
   logic 				r_dside_tlb_miss,n_dside_tlb_miss;
   
   utlb_entry_t n_tlb_rsp, r_tlb_rsp;
   
   assign iside_rsp_valid = r_iside_rsp_valid;
   assign dside_rsp_valid = r_dside_rsp_valid;
   assign tlb_rsp = r_tlb_rsp;
   assign iside_tlb_miss = r_iside_tlb_miss;
   assign dside_tlb_miss = r_dside_tlb_miss;
   
   assign tlb_hit = r_tlb_hit;
   
   always_comb
     begin
	n_iside_rsp_valid = 1'b0;
	n_dside_rsp_valid = 1'b0;
	n_got_iside = r_got_iside | iside_req;
	n_got_dside = r_got_dside | dside_req;
	n_tlb_rsp = r_tlb_rsp;
	n_tlb_hit = 1'b0;
	n_iside_tlb_miss = 1'b0;
	n_dside_tlb_miss = 1'b0;
	
	n_state = r_state;
	case(r_state)
	  IDLE:
	    begin
	       n_iside_tlb_miss = n_got_iside;
	       n_dside_tlb_miss = n_got_dside;
	       if(n_got_iside)
		 begin
		    n_state = ISIDE;
		 end
	       else if(n_got_dside)
		 begin
		    n_state = DSIDE;
		 end
	    end
	  ISIDE:
	    begin
	       n_state = IDLE;
	       n_got_iside = 1'b0;
	       n_iside_rsp_valid = 1'b1;
	       n_tlb_rsp.valid = 1'b1;
	       n_tlb_rsp.paddr = iside_paddr;
	       n_tlb_rsp.r = 1'b0;
	       n_tlb_rsp.w = 1'b0;
	       n_tlb_rsp.x = 1'b1;
	       n_tlb_rsp.bogus = 1'b0;
	    end
	  DSIDE:
	    begin
	       n_state = IDLE;
	       n_got_dside = 1'b0;
	       n_dside_rsp_valid = 1'b1;
	       n_tlb_rsp.valid = 1'b1;
	       n_tlb_rsp.paddr = dside_paddr;
	       n_tlb_rsp.r = 1'b1;
	       n_tlb_rsp.w = 1'b1;
	       n_tlb_rsp.x = 1'b0;
	       n_tlb_rsp.bogus = 1'b0;
	    end
	  default:
	    begin
	    end
	endcase // case (r_state)
     end // always_comb

   always_ff@(posedge clk)
     begin
	r_tlb_rsp <= n_tlb_rsp;
     end
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= IDLE;
	     r_iside_rsp_valid <= 1'b0;
	     r_dside_rsp_valid <= 1'b0;
	     r_got_iside <= 1'b0;
	     r_got_dside <= 1'b0;
	     r_tlb_hit <= 1'b0;
	     r_iside_tlb_miss <= 1'b0;
	     r_dside_tlb_miss <= 1'b0; 
	  end
	else
	  begin
	     r_state <= n_state;
	     r_iside_rsp_valid <= n_iside_rsp_valid;
	     r_dside_rsp_valid <= n_dside_rsp_valid;
	     r_got_iside <= n_got_iside;
	     r_got_dside <= n_got_dside;
	     r_tlb_hit <= n_tlb_hit;
	     r_iside_tlb_miss <= n_iside_tlb_miss;
	     r_dside_tlb_miss <= n_dside_tlb_miss;	     
	  end
     end
   
   logic [31:0] r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end

   // always_ff@(negedge clk)
   //   begin
   // 	if(dside_req)
   // 	  begin
   // 	     $display("dsize req for %x at cycle %d", 
   // 		      {dside_paddr, 12'd0}, r_cycle);
   // 	  end
   //   end
   
endmodule
