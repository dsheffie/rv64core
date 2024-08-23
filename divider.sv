`include "uop.vh"
`include "machine.vh"

module divider(clk,
	       reset,
	       wb_slot_used,
	       inA,
	       inB,
	       rob_ptr_in,
	       prf_ptr_in,	     
	       is_signed_div,
	       is_rem,
	       is_w,
	       start_div,
	       y,
	       rob_ptr_out,
	       prf_ptr_out,
	       ready,
	       complete
	       );

   parameter LG_W = 5;
   localparam W = 1<<LG_W;
   localparam W2 = 2*W;
   input logic clk;
   input logic reset;
   input logic wb_slot_used;
   input logic [`M_WIDTH-1:0] inA;
   input logic [`M_WIDTH-1:0] inB;
   
   input logic [`LG_ROB_ENTRIES-1:0] rob_ptr_in;
   input logic [`LG_PRF_ENTRIES-1:0] prf_ptr_in;
   
   input logic 	      is_signed_div;
   input logic 	      is_rem;
   input logic	      is_w;
   input logic 	      start_div;
   
   output logic [`M_WIDTH-1:0] y;
   
   output logic [`LG_ROB_ENTRIES-1:0] rob_ptr_out;
   output logic [`LG_PRF_ENTRIES-1:0] prf_ptr_out;

   output logic        ready;
   output logic        complete;
   
   typedef enum logic [1:0] {IDLE = 'd0,
			     DIVIDE = 'd1,
			     PACK_OUTPUT = 'd2,
			     WAIT_FOR_WB = 'd3
			     } state_t;

   
   state_t r_state, n_state;
   logic 	r_is_signed, n_is_signed;
   logic 	r_sign, n_sign;
   logic 	r_rem_sign, n_rem_sign;
   logic 	r_is_rem_op, n_is_rem_op;
   
   logic [`LG_ROB_ENTRIES-1:0] r_rob_ptr, n_rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] r_gpr_prf_ptr, n_gpr_prf_ptr;

   logic [W-1:0] 		    r_A, n_A, r_B, n_B;
   logic [W2-1:0] 		    r_Y, n_Y;
   logic [W2-1:0] 		    r_D, n_D, r_R, n_R;
   logic [W-1:0] 		    t_ss;
   logic			    r_is_w, n_is_w;
   
   logic [LG_W-1:0] 		    r_idx, n_idx;
   logic 			    t_bit,t_valid;

   wire [W-1:0] 		    srcA = inA[W-1:0];
   wire [W-1:0] 		    srcB = inB[W-1:0];
   
      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= IDLE;
	     r_rob_ptr <= 'd0;
	     r_gpr_prf_ptr <= 'd0;
	     r_is_signed <= 1'b0;
	     r_sign <= 1'b0;
	     r_rem_sign <= 1'b0;
	     r_is_rem_op <= 1'b0;
	     r_A <= 'd0;
	     r_B <= 'd0;
	     r_Y <= 'd0;
	     r_D <= 'd0;
	     r_R <= 'd0;
	     r_idx <= 'd0;
	     r_is_w <= 1'b0;
	  end
	else
	  begin
	     r_state <= n_state;
	     r_rob_ptr <= n_rob_ptr;
	     r_gpr_prf_ptr <= n_gpr_prf_ptr;
	     r_is_signed <= n_is_signed;
	     r_sign <= n_sign;
	     r_rem_sign <= n_rem_sign;
	     r_is_rem_op <= n_is_rem_op;
	     r_A <= n_A;
	     r_B <= n_B;
	     r_Y <= n_Y;
	     r_D <= n_D;
	     r_R <= n_R;
	     r_idx <= n_idx;
	     r_is_w <= n_is_w;
	  end
     end

   shiftregbit #(.W(W)) 
   ss(
      .clk(clk), 
      .reset(reset),
      .b(t_bit),
      .valid(t_valid), 
      .out(t_ss)
      );

			     
   always_comb
     begin
	n_rob_ptr = r_rob_ptr;
	n_gpr_prf_ptr = r_gpr_prf_ptr;
	n_state = r_state;
	n_is_signed = r_is_signed;
	n_sign = r_sign;
	n_rem_sign = r_rem_sign;
	n_is_rem_op = r_is_rem_op;
	n_A = r_A;
	n_B = r_B;
	n_Y = r_Y;
	n_D = r_D;
	n_R = r_R;
	n_idx = r_idx;
	t_bit = 1'b0;
	t_valid = 1'b0;
	n_is_w = r_is_w;
	
	//output signals
	ready = (r_state == IDLE) & !start_div;
	rob_ptr_out = r_rob_ptr;
	prf_ptr_out = r_gpr_prf_ptr;
	y = r_Y[W-1:0];
	complete = 1'b0;
	
	unique case (r_state)
	  IDLE:
	    begin
	       n_is_w = is_w;
	       n_rob_ptr = rob_ptr_in;
	       n_gpr_prf_ptr = prf_ptr_in;
	       n_is_rem_op = is_rem;
	       n_is_signed = is_signed_div;
	       n_state = start_div ? DIVIDE : IDLE;
	       n_idx = W-1;
	       n_sign = srcA[W-1] ^ srcB[W-1];
	       n_rem_sign = srcA[W-1];
	       n_A = is_signed_div & srcA[W-1] ? ((~srcA) + 'd1) : srcA;
	       n_B = is_signed_div & srcB[W-1] ? ((~srcB) + 'd1) : srcB;
	       n_D = {n_B, {W{1'b0}}};
	       n_R = {{W{1'b0}}, n_A};
	    end
	  DIVIDE:
	    begin
	       if({r_R[W2-2:0], 1'b0} >= r_D)
		 begin
		    n_R = {r_R[W2-2:0], 1'b0} - r_D;
		    t_bit = 1'b1;
		    t_valid = 1'b1;		    
		 end
	       else
		 begin
		    n_R = {r_R[W2-2:0], 1'b0};
		    t_bit = 1'b0;
		    t_valid = 1'b1;
		 end // else: !if({r_R[W2-2:0], 1'b0} >= r_D)
	       n_state = (r_idx == 'd0) ? PACK_OUTPUT : DIVIDE;
	       n_idx = r_idx - 'd1;
	       
	    end // case: DIVIDE
	  PACK_OUTPUT:
	    begin
	       n_state = WAIT_FOR_WB;	       
	       n_Y[W-1:0] = t_ss;
	       n_Y[W2-1:W] = n_R[W2-1:W];
	       if(r_is_signed && r_sign)
		 begin
		    n_Y[W-1:0] = (~t_ss) +'d1;
		 end
	       if(r_is_signed && r_rem_sign)
		 begin
		    n_Y[W2-1:W] = (~n_R[W2-1:W]) + 'd1;
		 end
	       if(r_is_rem_op)
		 begin
		    n_Y[W-1:0] = n_Y[W2-1:W];
		 end
	       if(r_is_w)
		 begin
		    n_Y = { {96{n_Y[31]}}, n_Y[31:0]};
		 end
	    end
	  WAIT_FOR_WB:
	    begin
	       complete =1'b1;
	       n_state = IDLE;
	    end
	  default:
	    begin
	    end
	endcase // case r_state
     end // always_comb

   

endmodule // divider


   
   
   
