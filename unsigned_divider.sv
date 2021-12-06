module unsigned_divider(clk,
			reset,
			srcA,
			srcB,
			start_div,
			y,
			ready,
			complete
			);

   parameter LG_W = 5;
   parameter W = 1<<LG_W;
   localparam W2 = 2*W;
   input logic clk;
   input logic reset;
   input logic [W-1:0] srcA;
   input logic [W-1:0] srcB;

   input logic 	      start_div;
   
   output logic [W-1:0] y;
   

   output logic        ready;
   output logic        complete;
   
   typedef enum logic [2:0] {IDLE = 'd0,
			     DIVIDE = 'd2,
			     PACK_OUTPUT = 'd3,
			     DONE = 'd4} state_t;

   
   state_t r_state, n_state;

   logic [W-1:0] 		    r_A, n_A, r_B, n_B;
   logic [W-1:0] 		    r_Y, n_Y;
   logic [W2-1:0] 		    r_D, n_D, r_R, n_R;
   logic [W-1:0] 		    t_ss;
   
   logic [LG_W-1:0] 		    r_idx, n_idx;
   logic 			    t_bit,t_valid;

   logic [31:0] 		    n_bits = W-1;
      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= IDLE;
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
	n_state = r_state;
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
	y = r_Y;
	complete = 1'b0;
	
	unique case (r_state)
	  IDLE:
	    begin
	       if(start_div)
		 begin
		    n_state = DIVIDE;
		 end
	       n_A = srcA;
	       n_B = srcB;
	       n_D = {srcB, {W{1'b0}}};
	       n_R = {{W{1'b0}},srcA};
	       n_idx = n_bits[LG_W-1:0];
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
	       n_state = DONE;
	       n_Y = t_ss;
	    end
	  DONE:
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

   
   
   
