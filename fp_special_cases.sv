module fp_special_cases(in_a, in_b,
			a_is_nan, a_is_inf, a_is_denorm, a_is_zero,
			b_is_nan, b_is_inf, b_is_denorm, b_is_zero);
   
   parameter W = 32;
   input logic [W-1:0] in_a;
   input logic [W-1:0] in_b;
   
   output logic        a_is_nan;
   output logic        a_is_inf;
   output logic        a_is_denorm;
   output logic        a_is_zero;
   output logic        b_is_nan;
   output logic        b_is_inf;
   output logic        b_is_denorm;
   output logic        b_is_zero;
      
   localparam FW = (W==32) ? 23 : 52;
   localparam EW = (W==32) ? 8 : 11;

   localparam INFINITY = (1 << EW) - 1;
   
   wire [FW-1:0]       w_a_mant = in_a[FW-1:0];
   wire [EW-1:0]       w_a_exp = in_a[W-2:FW];
   
   wire [FW-1:0]       w_b_mant = in_b[FW-1:0];
   wire [EW-1:0]       w_b_exp = in_b[W-2:FW];
   
   always_comb
     begin
	a_is_nan = (w_a_exp == INFINITY) && (w_a_mant != 'd0);
	a_is_inf = (w_a_exp == INFINITY) && (w_a_mant == 'd0);
	a_is_denorm = (w_a_exp == 'd0);
	a_is_zero = (w_a_exp == 'd0) && (w_a_mant == 'd0);
	
	b_is_nan = (w_b_exp == INFINITY) && (w_b_mant != 'd0);
	b_is_inf = (w_b_exp == INFINITY) && (w_b_mant == 'd0);
	b_is_denorm = (w_b_exp == 'd0);
	b_is_zero = (w_b_exp == 'd0) && (w_b_mant == 'd0);
	
     end
   
endmodule // fp_special_cases
