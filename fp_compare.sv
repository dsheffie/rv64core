`include "fp_compare.vh"

`ifdef DEBUG_FPU
import "DPI-C" function int fp32_compare_lt(input int a, input int b);
import "DPI-C" function int fp64_compare_lt(input longint a, input longint b);
import "DPI-C" function int fp32_compare_le(input int a, input int b);
import "DPI-C" function int fp64_compare_le(input longint a, input longint b);

module bogo_fp32_compare(input logic [31:0] a, input logic [31:0] b, output logic [31:0] y);
   always_comb
     begin
	y = fp32_compare_le(a,b);
     end
endmodule

module bogo_fp64_compare(input logic [63:0] a, input logic [63:0] b, output logic [31:0] y);
   always_comb
     begin
	y = fp64_compare_le(a,b);
     end
endmodule
`endif


module fp_compare(clk, pc, a, b, start, cmp_type, y);
   parameter W = 32;
   parameter D = 4;
   
   input logic clk;
   input logic [31:0] pc;
   
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   input logic 	       start;
   input 	       fp_cmp_t	       cmp_type;         
   output logic        y;

   localparam F = (W==32) ? 23 : 52;
   localparam E = (W==32) ? 8 : 11;
   logic 	       t_y;

   wire 	       w_a_is_zero = (a[W-2:0] == 'd0);
   wire 	       w_b_is_zero = (b[W-2:0] == 'd0);
   wire 	       w_sign_a = w_a_is_zero ? 1'b0 : a[W-1];
   wire 	       w_sign_b = w_b_is_zero ? 1'b0 : b[W-1];
   wire 	       w_both_neg = w_sign_a &&w_sign_b;
   wire 	       a_is_nan, a_is_inf, a_is_denorm;
   wire 	       b_is_nan, b_is_inf, b_is_denorm;
   
   fp_special_cases #(.W(W)) fsc (
				  .in_a(a), 
				  .in_b(b),
				  .a_is_nan(a_is_nan), 
				  .a_is_inf(a_is_inf), 
				  .a_is_denorm(a_is_denorm), 
				  .a_is_zero(),
				  .b_is_nan(b_is_nan), 
				  .b_is_inf(b_is_inf), 
				  .b_is_denorm(b_is_denorm), 
				  .b_is_zero()
				  );
   
   
   wire [E-1:0] w_exp_a = a[W-2:F];
   wire [E-1:0] w_exp_b = b[W-2:F];
   wire [F-1:0] w_mant_a = a[F-1:0];
   wire [F-1:0] w_mant_b = b[F-1:0];

   
   logic [D-1:0]       r_d;
   wire [D-1:0]        w_d;

   assign y = r_d[D-1];
   generate
      assign w_d[0] = t_y;
      for(genvar i = 1; i < D; i=i+1)
	begin
	   assign w_d[i] = r_d[i-1];
	end
   endgenerate

   always_ff@(posedge clk)
     begin
	r_d <= w_d;
     end
`ifdef DEBUG_FPU
   logic [31:0] t_dpi;
   generate
      if(W == 32)
	bogo_fp32_compare c(a,b,t_dpi);      
      else
	bogo_fp64_compare c(a,b,t_dpi);
   endgenerate
`endif

   
   wire w_sign_lt = ((w_sign_a ^ w_sign_b) & w_sign_a);
   wire w_sign_gt = ((w_sign_a ^ w_sign_b) & w_sign_b);
   wire w_sign_eq = (w_sign_a == w_sign_b);
   wire w_exp_lt = (w_exp_a < w_exp_b) & w_sign_eq;
   wire w_exp_gt = (w_exp_a > w_exp_b) & w_sign_eq;
   wire w_exp_eq = (w_exp_a == w_exp_b) & w_sign_eq;
   wire w_mant_lt = (w_mant_a < w_mant_b) & w_exp_eq;
   wire w_mant_gt = (w_mant_a > w_mant_b) & w_exp_eq;

   wire w_lt_t = w_sign_lt | w_exp_lt | w_mant_lt;
   wire w_gt_t = w_sign_gt | w_exp_gt | w_mant_gt;
   
   wire w_lt = w_both_neg ? w_gt_t : w_lt_t;
   wire w_eq = (a == b) || (w_a_is_zero && w_b_is_zero);
   wire w_le = w_lt | w_eq;

   
`ifdef DEBUG_FPU
   always_comb
     begin
	if(start && (w_le != t_dpi[0]) && !(a_is_nan || b_is_nan))
	  begin
	     $display("a = %x, b = %x, w_sign_lt = %b, w_exp_lt = %b, w_mant_lt = %b, dpi %x",
		      a, b, w_sign_lt, w_exp_lt, w_mant_lt, t_dpi[0]);
	     $display("w_le = %b, w_lt = %b, w_eq = %b, t_dpi[0] = %b", w_le, w_lt, w_eq, t_dpi[0]);
	     $display("a sign = %b, b sign = %b", w_sign_a, w_sign_b);
	     $display("a_exp = %x, b_exp = %x", w_exp_a, w_exp_b);
	     $display("a_mant = %x, b_mant = %x, lt %d", w_mant_a, w_mant_b, (w_mant_a < w_mant_b));
	     $display("a nan = %b, b nan = %b", a_is_nan, b_is_nan);
	     $display("a denorm = %b, b denorm = %b", a_is_denorm, b_is_denorm);
	     $display("a inf = %b, b inf = %b", a_is_inf, b_is_inf);
	     $stop();
	  end
     end
`endif

   
   always_comb
     begin
	t_y = 1'b0;
	case(cmp_type)
	  CMP_LT:
	    begin
	       t_y = w_lt;
	    end
	  CMP_LE:
	    begin
	       t_y = w_le;
	    end
	  CMP_EQ:
	    begin
	       t_y = w_eq;
	    end
	  default:
	    begin
	    end
	endcase // case (cmp_type)
     end // always_comb
   
endmodule
