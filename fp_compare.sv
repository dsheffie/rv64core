`include "fp_compare.vh"

module fp_compare(clk, pc, a, b, cmp_type, y);
   parameter W = 32;
   parameter D = 4;
   
   input logic clk;
   input logic [63:0] pc;
   
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   input 	       fp_cmp_t	       cmp_type;         
   output logic        y;

   localparam F = (W==32) ? 23 : 52;
   localparam E = (W==32) ? 8 : 11;
   logic 	       t_y;

   wire 	       w_sign_a = a[W-1];
   wire 	       w_sign_b = b[W-1];

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

   wire w_sign_lt = ((w_sign_a ^ w_sign_b) & w_sign_a);
   wire w_sign_eq = (w_sign_a == w_sign_b);
   wire w_exp_lt = (w_exp_a < w_exp_b) & w_sign_eq;
   wire w_exp_eq = (w_exp_a == w_exp_b) & w_sign_eq;
   wire w_mant_lt = (w_mant_a < w_mant_b) & w_exp_eq;
   
   wire w_lt = w_sign_lt | w_exp_lt | w_mant_lt;
   
   wire w_eq = (a == b);
   
   always_comb
     begin
	t_y = 1'b0;
	case(cmp_type)
	  CMP_LT:
	    begin
	       t_y = w_lt;
	       //$display("CMP_LT : t_y = %b, w_sign %b, w_exp %b, a sign = %b, b sign = %b, a exp %d, b exp %d",
	       //w_lt, w_sign_lt, w_exp_lt, w_sign_a, w_sign_b, w_exp_a, w_exp_b);
	    end
	  CMP_LE:
	    begin
	       t_y = w_lt | w_eq;
	       //$display("CMP_LE : t_y = %b, w_sign %b, w_exp %b, a sign = %b, b sign = %b, a exp %d, b exp %d",
	       //w_lt, w_sign_lt, w_exp_lt, w_sign_a, w_sign_b, w_exp_a, w_exp_b);	       
	    end
	  CMP_EQ:
	    begin
	       t_y = w_eq;
	       //$display("CMP_EQ : t_y = %b, w_sign %b, w_exp %b, a sign = %b, b sign = %b, a exp %d, b exp %d",
	       //w_lt, w_sign_lt, w_exp_lt, w_sign_a, w_sign_b, w_exp_a, w_exp_b);	       
	    end
	  default:
	    begin
	    end
	endcase // case (cmp_type)
     end
endmodule
