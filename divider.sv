`include "uop.vh"
`include "machine.vh"

module divider(clk,
	       reset,
	       flush,
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
   input logic flush;
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
   
   typedef enum logic [2:0] {IDLE = 'd0,
			     DIVIDE = 'd1,
			     CLZ = 'd2,
			     PACK_OUTPUT = 'd3,
			     WAIT_FOR_WB = 'd4,
			     CACHE_PACK_OUTPUT = 'd5,
			     CACHE_WAIT_FOR_WB = 'd6
			     } state_t;

   
   state_t r_state, n_state;
   logic 	r_is_signed, n_is_signed;
   logic 	r_sign, n_sign;
   logic 	r_rem_sign, n_rem_sign;
   logic 	r_is_rem_op, n_is_rem_op;
   
   logic [`LG_ROB_ENTRIES-1:0] r_rob_ptr, n_rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] r_gpr_prf_ptr, n_gpr_prf_ptr;

   logic [W-1:0] 		    r_A, n_A, r_B, n_B;
   logic [W-1:0]		    r_lastA, n_lastA, r_lastB, n_lastB;
   logic			    r_last_signed, n_last_signed;
   logic [W-1:0]		    r_last_ss, n_last_ss;
   logic [W2-1:0]		    r_last_Y, n_last_Y;
   logic [W2-1:0]		    r_last_R, n_last_R;   
   logic			    r_last_valid, n_last_valid;
   
   logic [W2-1:0] 		    r_Y, n_Y;
   logic [W2-1:0] 		    r_D, n_D, r_R, n_R;
   logic [W-1:0] 		    t_ss;
   logic			    r_is_w, n_is_w;
   
   logic [LG_W:0] 		    r_idx, n_idx;
   logic 			    t_bit,t_valid,t_clr;

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
	     r_lastA <= 64'd0;
	     r_lastB <= 64'd0;
	     r_last_Y <= 'd0;
	     r_last_R <= 'd0;
	     r_last_ss <= 'd0;
	     r_last_signed <= 1'b0;
	     r_last_valid <= 1'b0;
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
	     r_lastA <= n_lastA;
	     r_lastB <= n_lastB;
	     r_last_Y <= n_last_Y;
	     r_last_R <= n_last_R;
	     r_last_ss <= n_last_ss;	     
	     r_last_signed <= n_last_signed;
	     r_last_valid <= n_last_valid;
	     r_idx <= n_idx;
	     r_is_w <= n_is_w;
	  end
     end

   //shiftregbit #(.W(W)) 
   //ss(
      //.clk(clk), 
      //.reset(reset),
      //.clear(t_clr),
      //.b(t_bit),
      //.valid(t_valid), 
      //.out(t_ss)
      //);

   always_ff@(posedge clk)
     begin
	if(reset | t_clr)
	  begin
	     t_ss <= 'd0;
	  end
	else if(t_valid)
	  begin
	     t_ss <= t_ss | ( {{(W-1){1'b0}}, t_bit}  << r_idx);
	  end
     end

		
   wire w_match_prev = (r_lastA == r_A) &
	(r_lastB == r_B) & 
	(r_last_signed == r_is_signed) &
	r_last_valid;


   wire [LG_W:0] w_clz_A;
   count_leading_zeros #(.LG_N(LG_W)) clz0 (.in(r_A), .y(w_clz_A));
   
   
   //always_ff@(posedge clk)
   //begin
   //if(r_state == CLZ)
   //begin
   //$display("a = %x, b = %x, zero skip %d", r_A, r_B, 7'd64 - w_clz_A);
   //end
   //end

   //always_ff@(negedge clk)
     //begin
   //	if(r_state == DIVIDE & flush)
   //begin
   //$display("got flush while divider is active");
   //end
   // end
   
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
	
	n_lastA = r_lastA;
	n_lastB = r_lastB;
	n_last_signed = r_last_signed;
	n_last_Y = r_last_Y;
	n_last_R = r_last_R;
	n_last_ss = r_last_ss;
	n_last_valid = r_last_valid;
	
	n_idx = r_idx;
	t_bit = 1'b0;
	t_clr = 1'b0;
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
	       t_clr = 1'b1;
	       n_is_w = is_w;
	       n_rob_ptr = rob_ptr_in;
	       n_gpr_prf_ptr = prf_ptr_in;
	       n_is_rem_op = is_rem;
	       n_is_signed = is_signed_div;
	       n_state = start_div ? CLZ : IDLE;
	       n_idx = W-1;
	       n_sign = srcA[W-1] ^ srcB[W-1];
	       n_rem_sign = srcA[W-1];
	       n_A = is_signed_div & srcA[W-1] ? ((~srcA) + 'd1) : srcA;
	       n_B = is_signed_div & srcB[W-1] ? ((~srcB) + 'd1) : srcB;
	       n_D = {n_B, {W{1'b0}}};
	       n_R = {{W{1'b0}}, n_A};
	    end // case: IDLE
	  CLZ:
	    begin
	       n_state = DIVIDE;
	       n_idx = 7'd63 - w_clz_A;
	       n_R = r_R << w_clz_A;
	       //$display("w_clz_A = %d", w_clz_A);
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
	       n_state = (w_match_prev|flush) ? CACHE_PACK_OUTPUT : 
			 ((r_idx == 'd0) ? PACK_OUTPUT : DIVIDE);
	       n_idx = r_idx - 'd1;
	       
	    end // case: DIVIDE
	  PACK_OUTPUT:
	    begin
	       n_state = CACHE_WAIT_FOR_WB;
	       //WAIT_FOR_WB;
	       n_lastA = r_A;
	       n_lastB = r_B;
	       n_last_signed = r_is_signed;
	       n_last_valid = 1'b1;
	       n_Y[W-1:0] = t_ss;
	       n_Y[W2-1:W] = n_R[W2-1:W];
	       
	       n_last_Y = n_Y;
	       n_last_R = n_R;
	       n_last_ss = t_ss;
	       
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
	  CACHE_PACK_OUTPUT:
	    begin
	       n_Y = r_last_Y;
	       n_R = r_last_R;
	       if(r_is_signed && r_sign)
		 begin
		    n_Y[W-1:0] = (~r_last_ss) +'d1;
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
	       n_state = CACHE_WAIT_FOR_WB;
	    end // case: CACHE_PACK_OUTPUT
	  CACHE_WAIT_FOR_WB:
	    begin
	       if(wb_slot_used == 1'b0)
		 begin
		    n_state = IDLE;
		    complete = 1'b1;
		 end
	    end
	  default:
	    begin
	    end
	endcase // case r_state
     end // always_comb

   

endmodule // divider


   
   
   
