`include "rob.vh"

`ifdef DEBUG_FPU
import "DPI-C" function int fp32_div(input int a, input int b);
import "DPI-C" function longint fp64_div(input longint a, input longint b);
import "DPI-C" function int fp32_sqrt(input int a);
import "DPI-C" function longint fp64_sqrt(input longint a);

module bogo_fp32_div(input logic [31:0] a, input logic [31:0] b, output logic [31:0] y);
   always_comb
     begin
	y = fp32_div(a,b);
     end
endmodule

module bogo_fp64_div(input logic [63:0] a, input logic [63:0] b, output logic [63:0] y);
   always_comb
     begin
	y = fp64_div(a,b);
     end
endmodule // bogo_fp64_div

module bogo_fp32_sqrt(input logic [31:0] a, output logic [31:0] y);
   always_comb
     begin
	y = fp32_sqrt(a);
     end
endmodule

module bogo_fp64_sqrt(input logic [63:0] a, output logic [63:0] y);
   always_comb
     begin
	y = fp64_sqrt(a);
     end
endmodule
`endif


module fp_div(
	      // Outputs
	      y,
	      valid,
	      rob_ptr_out,
	      dst_ptr_out,
	      active,
	      // Inputs
	      clk,
	      reset, 
	      a, 
	      b, 
	      start, 	 
	      is_sqrt,
	      rob_ptr_in,
	      dst_ptr_in
	      );
   parameter LG_PRF_WIDTH = 1;
   parameter LG_ROB_WIDTH = 1;
   parameter W = 32;
   localparam FW = (W==32) ? 23 : 52;
   localparam EW = (W==32) ? 8 : 11;
   localparam LG_FW = (W==32) ? 5 : 6;
   localparam DW = 2*(FW+1);
   
   input logic clk;
   input logic reset;
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   input logic 	       start;
   input logic 	       is_sqrt;
   input logic [LG_ROB_WIDTH-1:0] rob_ptr_in;
   input logic [LG_PRF_WIDTH-1:0] dst_ptr_in;
   
   output logic [W-1:0] y;
   output logic 	valid;
   output logic [LG_ROB_WIDTH-1:0] rob_ptr_out;
   output logic [LG_PRF_WIDTH-1:0] dst_ptr_out;
   output logic 		   active;

   /* inputs to the multiplier */
   wire [FW-1:0] 		   w_pad = {(FW){1'b0}};
   wire [DW-2:0] 		   w_mant_a = {1'b1, a[FW-1:0], w_pad};
   wire [DW-2:0] 		   w_mant_b = {w_pad, 1'b1, b[FW-1:0]};
   
   //logic [EW-1:0] r_exp;
   //logic [FW-1:0]  r_mant;
   //logic [(FW+4):0] r_rnd_mant_in;
   //logic [EW:0]     r_rnd_exp_in;
   logic 			   t_div_complete;
   
   logic 	 n_valid, r_valid, n_sqrt, r_sqrt;
   
   logic [DW-2:0] y_div;

   typedef enum logic [2:0] {IDLE = 'd0,
			     COMPUTE_EXP = 'd1,
			     WAIT_FOR_DIVIDE = 'd2,
			     NORMALIZE = 'd3,
			     ROUND = 'd4,
			     DONE = 'd5} state_t;   
   state_t r_state, n_state;

   logic [LG_ROB_WIDTH-1:0] r_rob_ptr, n_rob_ptr;
   logic [LG_PRF_WIDTH-1:0] r_dst_ptr, n_dst_ptr;
   
   logic [EW:0] 	    r_exp, n_exp;
   logic [EW:0] 	    r_exp_a, n_exp_a;
   logic [EW:0] 	    r_exp_b, n_exp_b;
   logic 	  r_sign, n_sign;
   logic [DW-2:0] r_div_mant, n_div_mant;
   logic [W-1:0]  r_y, n_y;
   
   logic 	  t_start_div;
   logic 	  r_active, n_active;
   
   assign active = r_active;
   //(r_state != IDLE);
   assign valid = r_valid;
   assign y = r_y;
   assign dst_ptr_out = r_dst_ptr;
   assign rob_ptr_out = r_rob_ptr;

   wire [EW-1:0] w_bias = ((1<<(EW-1)) - 1);

   logic [31:0]  r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
	// if(start && (r_state != IDLE))
	//   begin
	//      $display("trying start fp divider while already active at cycle %d", r_cycle);
	//      $stop();
	//   end
     end

`ifdef DEBUG_FPU
   logic [W-1:0] t_dpi, t_dpi_sqrt, n_dpi, r_dpi;
   generate
      if(W == 32)
	begin
	   bogo_fp32_div bd32(.a(a), .b(b), .y(t_dpi));
	   bogo_fp32_sqrt bs32(.a(a), .y(t_dpi_sqrt));
	end
      else
	begin
	   bogo_fp64_div bd64(.a(a), .b(b), .y(t_dpi));
	   bogo_fp64_sqrt bs64(.a(a), .y(t_dpi_sqrt));	   
	end
   endgenerate
`endif
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= IDLE;
	     r_valid <= 1'b0;
	     r_sqrt <= 1'b0;
	     r_y <= 'd0;
	     r_rob_ptr <= 'd0;
	     r_dst_ptr <= 'd0;
	     r_exp <= 'd0;
	     r_exp_a <= 'd0;
	     r_exp_b <= 'd0;
	     r_sign <= 1'b0;
	     r_div_mant <= 'd0;
	     r_active <= 1'b0;
`ifdef DEBUG_FPU
	       r_dpi <= 'd0;
`endif	     
	  end
	else
	  begin
	     r_state <= n_state;
	     r_valid <= n_valid;
	     r_sqrt <= n_sqrt;
	     r_y <= n_y;
	     r_rob_ptr <= n_rob_ptr;
	     r_dst_ptr <= n_dst_ptr;
	     r_exp <= n_exp;
	     r_exp_a <= n_exp_a;
	     r_exp_b <= n_exp_b;
	     r_sign <= n_sign;
	     r_div_mant <= n_div_mant;
	     r_active <= n_active;
`ifdef DEBUG_FPU
	       r_dpi <= n_dpi;
`endif	     	     
	  end
     end // always_ff@ (posedge clk)

   always_comb
     begin
	n_state = r_state;
	n_valid = 1'b0;
	n_y = r_y;
	n_rob_ptr = r_rob_ptr;
	n_dst_ptr = r_dst_ptr;
	n_exp = r_exp;
	n_exp_a = r_exp_a;
	n_exp_b = r_exp_b;
	n_sign = r_sign;
	n_div_mant = r_div_mant;
	n_active = r_active;
	t_start_div = 1'b0;
	n_sqrt = r_sqrt;
`ifdef DEBUG_FPU
	n_dpi = r_dpi;
`endif	
	case(r_state)
	  IDLE:
	    begin
	       if(start)
		 begin
		    t_start_div = 1'b1;
		    n_dst_ptr = dst_ptr_in;
		    n_rob_ptr = rob_ptr_in;
`ifdef DEBUG_FPU		    
		    n_state = ROUND;
`else
		    n_state = COMPUTE_EXP;		    
`endif
		    n_active = 1'b1;
		    // $display("%d : fp divider for rob ptr %d, dest %d starts", 
		    // 	     r_cycle, rob_ptr_in, dst_ptr_in);		    
		 end
	       n_exp_a = {1'b0, a[W-2:FW]};
	       n_exp_b = {1'b0, b[W-2:FW]};
	       n_sign = a[W-1] ^ b[W-1];
	       n_sqrt = is_sqrt;
`ifdef DEBUG_FPU
	       n_dpi = is_sqrt ? t_dpi_sqrt : t_dpi;
`endif
	    end // case: IDLE
	  COMPUTE_EXP:
	    begin
	       n_exp = r_exp_a - r_exp_b + w_bias;
	       n_state = WAIT_FOR_DIVIDE;
	    end
	  WAIT_FOR_DIVIDE:
	    begin	       
	       if(t_div_complete)
		 begin
		    n_div_mant = y_div;
		    //$display("divider result int = %b, one = %b, exp_b = %d", 
		    //y_div[FW+1:0], y_div[FW], r_exp);
		    n_state = y_div[FW] ? ROUND : NORMALIZE;
		 end
	    end
	  NORMALIZE:
	    begin
	       n_div_mant = {r_div_mant[DW-3:0], 1'b0};
	       n_exp = r_exp - 'd1;
	       n_state = ROUND;
	    end
	  ROUND:
	    begin
	       n_state = DONE;
	       n_y = {r_sign, r_exp[EW-1:0], r_div_mant[FW-1:0]};
`ifdef DEBUG_FPU
	       n_y = r_dpi;
`endif
	       n_valid = 1'b1;
	       //$display("exp = %d, frac = %b", r_exp[EW-1:0], r_div_mant[FW-1:0]);
	    end
	  DONE:
	    begin
	       //$display("r_y %x, %x\n", r_y[W-1:W/2], r_y[(W/2)-1:0]);
	       
	       n_state = IDLE;
	       n_active = 1'b0;
	       // $display("fp divider for rob ptr %d, dest %d stops", 
	       // 		r_rob_ptr, r_dst_ptr);		    
	       
	    end
	  default:
	    begin
	    end
	endcase
     end
   
   
   unsigned_divider #(.LG_W(1+LG_FW), .W(DW-1)) ud0  
   ( 
     .clk(clk), .reset(reset), 
     .srcA(w_mant_a), .srcB(w_mant_b), 
     .start_div(t_start_div),
     .ready(), .complete(t_div_complete), 
     .y(y_div)
   );


   
   //fracmul #(.L(SP_IMUL_LATENCY), .W(FW+1)) m0 
   //(.y(w_prod), .clk(clk),.a(w_mant_a), .b(w_mant_b));
   
      

endmodule // sp_mul


   
