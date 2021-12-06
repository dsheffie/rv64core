`include "uop.vh"

module divider(clk,
	       reset,
	       srcA,
	       srcB,
	       rob_ptr_in,
	       hilo_prf_ptr_in,	     
	       is_signed_div,
	       start_div,
	       y,
	       rob_ptr_out,
	       hilo_prf_ptr_out,
	       ready,
	       complete
	       );

   parameter LG_W = 5;
   localparam W = 1<<LG_W;
   localparam W2 = 2*W;
   input logic clk;
   input logic reset;
   input logic [W-1:0] srcA;
   input logic [W-1:0] srcB;

   input logic [`LG_ROB_ENTRIES-1:0] rob_ptr_in;
   input logic [`LG_HILO_PRF_ENTRIES-1:0] hilo_prf_ptr_in;
   
   input logic 	      is_signed_div;
   input logic 	      start_div;
   
   output logic [W2-1:0] y;
   
   output logic [`LG_ROB_ENTRIES-1:0] rob_ptr_out;
   output logic [`LG_HILO_PRF_ENTRIES-1:0] hilo_prf_ptr_out;

   output logic        ready;
   output logic        complete;
   
   typedef enum logic [2:0] {IDLE = 'd0,
			     INPUT_SIGN = 'd1,
			     DIVIDE = 'd2,
			     PACK_OUTPUT = 'd3,
			     OUTPUT_SIGN = 'd4,
			     WAIT_FOR_WB = 'd5} state_t;

   
   state_t r_state, n_state;
   logic 	r_is_signed, n_is_signed;
   logic 	r_sign, n_sign;
   
   logic [`LG_ROB_ENTRIES-1:0] r_rob_ptr, n_rob_ptr;
   logic [`LG_HILO_PRF_ENTRIES-1:0] r_hilo_prf_ptr, n_hilo_prf_ptr;

   logic [W-1:0] 		    r_A, n_A, r_B, n_B;
   logic [W2-1:0] 		    r_Y, n_Y;
   logic [W2-1:0] 		    r_D, n_D, r_R, n_R;
   logic [W-1:0] 		    t_ss;
   
   logic [LG_W-1:0] 		    r_idx, n_idx;
   logic 			    t_bit,t_valid;
   
      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= IDLE;
	     r_rob_ptr <= 'd0;
	     r_hilo_prf_ptr <= 'd0;
	     r_is_signed <= 1'b0;
	     r_sign <= 1'b0;
	     r_A <= 'd0;
	     r_B <= 'd0;
	     r_Y <= 'd0;
	     r_D <= 'd0;
	     r_R <= 'd0;
	     r_idx <= 'd0;
	  end
	else
	  begin
	     r_state <= n_state;
	     r_rob_ptr <= n_rob_ptr;
	     r_hilo_prf_ptr <= n_hilo_prf_ptr;
	     r_is_signed <= n_is_signed;
	     r_sign <= n_sign;
	     r_A <= n_A;
	     r_B <= n_B;
	     r_Y <= n_Y;
	     r_D <= n_D;
	     r_R <= n_R;
	     r_idx <= n_idx;
	  end
     end

   shiftregbit #(.W(W)) ss
     (.clk(clk), .reset(reset), .b(t_bit), .valid(t_valid), .out(t_ss));
  
			     
   always_comb
     begin
	n_rob_ptr = r_rob_ptr;
	n_hilo_prf_ptr = r_hilo_prf_ptr;
	n_state = r_state;
	n_is_signed = r_is_signed;
	n_sign = r_sign;
	n_A = r_A;
	n_B = r_B;
	n_Y = r_Y;
	n_D = r_D;
	n_R = r_R;
	n_idx = r_idx;
	t_bit = 1'b0;
	t_valid = 1'b0;
	
	//output signals
	ready = (r_state == IDLE);
	rob_ptr_out = r_rob_ptr;
	hilo_prf_ptr_out = r_hilo_prf_ptr;
	y = r_Y;
	complete = 1'b0;
	
	unique case (r_state)
	  IDLE:
	    begin
	       if(start_div)
		 begin
		    n_rob_ptr = rob_ptr_in;
		    n_hilo_prf_ptr = hilo_prf_ptr_in;
		    n_is_signed = is_signed_div;
		    n_state = INPUT_SIGN;
		    n_A = srcA;
		    n_B = srcB;
		 end
	    end
	  INPUT_SIGN:
	    begin
	       if(r_is_signed)
		 begin
		    n_sign = r_A[W-1] ^ r_B[W-1];
		    n_A = r_A[W-1] ? ((~r_A) + 'd1) : r_A;
		    n_B = r_B[W-1] ? ((~r_B) + 'd1) : r_B;
		 end
	       else
		 begin
		    n_sign = 1'b0;
		 end
	       n_D = {n_B, {W{1'b0}}};
	       n_R = {{W{1'b0}}, n_A};
	       n_idx = W-1;
	       n_state = DIVIDE;
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
		 end
	       n_state = (r_idx == 'd0) ? PACK_OUTPUT : DIVIDE;
	       n_idx = r_idx - 'd1;
	    end // case: DIVIDE
	  PACK_OUTPUT:
	    begin
	       n_state = OUTPUT_SIGN;
	       n_Y[W-1:0] = t_ss;
	       n_Y[W2-1:W] = n_R[W2-1:W];
	    end
	  OUTPUT_SIGN:
	    begin
	       if(r_is_signed && r_sign)
		 begin
		    n_Y[W-1:0] = ((~r_Y[W-1:0]) +'d1);
		    n_Y[W2-1:W] = ((~r_Y[W2-1:W]) + 'd1);
		 end
	       n_state = WAIT_FOR_WB;
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

   

endmodule // div32

   
   
   
