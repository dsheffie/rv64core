

module fp_compare( a, b, ueq, une, ugt, ult, oeq, one, ogt, olt);
   parameter W = 32;
   
   input logic [W-1:0] a;
   input logic [W-1:0] b;

   output logic	       ueq;
   output logic	       une;
   output logic        ugt;
   output logic	       ult;
   output logic	       oeq;
   output logic	       one;
   output logic        ogt;
   output logic	       olt;   

   localparam F = (W==32) ? 23 : 52;
   localparam E = (W==32) ? 8 : 11;
   localparam INFINITY = (1 << E) - 1;

   wire 	       w_a_is_zero = (a[W-2:0] == 'd0);
   wire 	       w_b_is_zero = (b[W-2:0] == 'd0);
   wire 	       w_sign_a = w_a_is_zero ? 1'b0 : a[W-1];
   wire 	       w_sign_b = w_b_is_zero ? 1'b0 : b[W-1];
   wire 	       w_both_neg = w_sign_a &&w_sign_b;
   
   
   wire [E-1:0] w_exp_a = a[W-2:F];
   wire [E-1:0] w_exp_b = b[W-2:F];
   wire [F-1:0] w_mant_a = a[F-1:0];
   wire [F-1:0] w_mant_b = b[F-1:0];

   wire		a_is_nan = (w_exp_a == INFINITY) & (w_mant_a != 'd0);
   wire		b_is_nan = (w_exp_b == INFINITY) & (w_mant_b != 'd0);
   
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
   wire	w_gt = w_both_neg ? w_lt_t : w_gt_t;
   
   //wire	w_eq = (a == b) || (w_a_is_zero && w_b_is_zero);
   //wire w_le = w_lt | w_eq;

   wire	w_no_nan = (a_is_nan | b_is_nan) == 1'b0;
   wire	w_is_nan = (a_is_nan | b_is_nan);
   
   always_comb
     begin
	oeq = (a==b) & w_no_nan;
	ueq = (a==b) | w_is_nan;	
	/* These functions return a nonzero value if either argument is NaN, 
	 * or if a and b are unequal. */
	one = (a!=b) & w_no_nan;
	une = (a!=b) | w_is_nan;	
	/* These functions return a value less than zero if neither argument is NaN,
	 * and a is strictly less than b. */
	olt = w_lt & w_no_nan;
	ult = w_lt | w_is_nan;	
	/* These functions return a value greater than zero if neither argument is NaN,
	 * and a is strictly greater than b. */
	ogt = w_gt & w_no_nan;
	ugt = w_gt | w_is_nan;	
     end // always_comb
   
endmodule
