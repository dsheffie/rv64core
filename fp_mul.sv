`ifdef DEBUG_FPU
import "DPI-C" function int fp32_mul(input int a, input int b);
import "DPI-C" function longint fp64_mul(input longint a, input longint b);

module bogo_fp32_mul(input logic [31:0] a, input logic [31:0] b, output logic [31:0] y);
   always_comb
     begin
	y = fp32_mul(a,b);
     end
endmodule

module bogo_fp64_mul(input logic [63:0] a, input logic [63:0] b, output logic [63:0] y);
   always_comb
     begin
	y = fp64_mul(a,b);
     end
endmodule
`endif


module dff(q,d,clk);
   parameter N = 1;
   input logic [N-1:0] d;
   input logic 	       clk;
   output logic [N-1:0] q;
   always_ff@(posedge clk)
     begin
	q <= d;
     end
endmodule // dff

module shiftreg(clk,in,out);
   parameter W = 32;
   parameter D = 4;
   input logic clk;
   input logic [W-1:0] in;
   output logic [W-1:0] out;

   logic [W-1:0] 	t_delay [D-1:0];
   assign out = t_delay[D-1];
   
   dff #(.N(W) ) ff0(.clk(clk), .d(in), .q(t_delay[0]));
   generate
      for(genvar i = 1; i < D; i = i + 1)
	begin:delay
	   dff #(.N(W)) ff (.clk(clk), .d(t_delay[i-1]), .q(t_delay[i]));
	end
   endgenerate

   
endmodule

module fracmul(y, a, b);
   parameter W = 24;
   localparam W2 = W*2;
   input [W-1:0]       a;
   input [W-1:0] b;
   output [W2-1:0] y;

   wire [W2-1:0] w_comb_mul = a * b;
   assign y = w_comb_mul;
endmodule // fracmul
   

module expadd(y, a, b);
   parameter L = 3;
   parameter W = 8;
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   output logic [W:0] y;

   wire [W-1:0] w_bias = ((1<<(W-1)) - 1);
   wire [W:0] w_comb_add = ((a+b) - w_bias);
   assign y = w_comb_add;

endmodule // expadd

   

module detection(
   // Outputs
   zero, nan, infinity,
   // Inputs
   a, b
   );
   parameter E = 11;
   parameter F = 52;
   
   localparam W = E + F + 1;
      
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   
   output logic        zero;
   output logic        nan;
   output logic        infinity;

   wire 	w_nan = 1'b0;
   logic 	t_az, t_bz;
   logic [2:0] 	t_detect, t_out;

   wire [E-1:0] w_exp_a = a[W-2:F];
   wire [E-1:0] w_exp_b = b[W-2:F];
   wire [F-1:0] w_mant_a = a[F-1:0];
   wire [F-1:0] w_mant_b = b[F-1:0];

   wire 	w_infinity_a = (&w_exp_a) & (w_mant_a == 'd0);
   wire 	w_infinity_b = (&w_exp_b) & (w_mant_b == 'd0);
   
   always_comb
     begin
	t_az = (a[W-2:0] == 'd0);
	t_bz = (b[W-2:0] == 'd0);
     end

   assign zero = t_az|t_bz;
   assign nan = w_nan;
   assign infinity = w_infinity_a | w_infinity_b;
endmodule // detection

module normalize(/*AUTOARG*/
   // Outputs
   exp_out, mant_out,
   // Inputs
   exp_in, mant_in
   );
   parameter E = 8;
   parameter F = 23;
   
   input logic [E:0] exp_in;
   output logic [E:0] exp_out;

   input logic [(F+4):0] mant_in;
   output logic [(F+4):0] mant_out;


   /* Ideas from "A Fully Synthesizable Single-Precision,
    * Floating-Point Adder/Subtractor and Multiplier in VHDL 
    * for General and Educational Use by G. Marcus*/
   always_comb
     begin
	mant_out = mant_in;
	exp_out = exp_in;
	if(mant_in[F+4])
	  begin
	     // shifting right by one: collapse the two low bits into a sticky (OR)
	     mant_out = {1'b0, mant_in[(F+4):2], mant_in[1] | mant_in[0]};
	     exp_out = exp_in + 'd1;
	  end
     end
endmodule // normalize

module round(
   // Outputs
   exp_out, mant_out,
   // Inputs
   exp_in, mant_in, rm, sign
   );
   parameter E = 8;
   parameter F = 23;

   input logic [E:0] exp_in;
   output logic [E:0] exp_out;

   input logic [(F+4):0] mant_in;
   output logic [(F+4):0] mant_out;

   input logic [1:0]	 rm;        // 0=RN 1=RZ 2=RP(+inf) 3=RM(-inf)
   input logic		 sign;      // sign of the result being rounded

   logic [F+1:0] t_one = {{(F+1){1'b0}},1'b1};

   // bit2=guard, bit1=round, bit0=sticky, bit3=lsb of the kept result.
   wire inexact = mant_in[2] | mant_in[1] | mant_in[0];
   wire round_up =
	(rm == 2'd0) ? (mant_in[2] & (mant_in[3] | mant_in[1] | mant_in[0])) :  // RN ties-even
	(rm == 2'd1) ? 1'b0 :                                                    // RZ truncate
	(rm == 2'd2) ? (~sign & inexact) :                                       // RP toward +inf
	                ( sign & inexact);                                       // RM toward -inf
   always_comb
     begin
	exp_out = exp_in;
	mant_out = mant_in;
	if(round_up)
	  begin
	     mant_out = {(mant_in[(F+4):3] + t_one), 3'd0};
	  end
     end
endmodule // round

module sign(y,clk, a, b);
   input clk;
   input a;
   input b;
   output y;
   parameter L = 4;
   shiftreg #(.W(1), .D(L)) d (.clk(clk), .in(a^b), .out(y));
endmodule // sign



module fp_mul(
   // Outputs
   y, denorm, fflags,
   // Inputs
   clk, a, b, en, rm
   );
   parameter W = 32;
   parameter MUL_LAT = 4;
   localparam FW = (W==32) ? 23 : 52;
   localparam EW = (W==32) ? 8 : 11;
   localparam INFINITY = (1 << EW) - 1;
   localparam BIAS = (1 << (EW-1)) - 1;

   input logic clk;
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   input logic 	       en;
   input logic [1:0]   rm;          // 0=RN(ties-even) 1=RZ 2=RP(+inf) 3=RM(-inf)
   output logic [W-1:0] y;
   output logic		denorm;
   output logic [4:0]	fflags;     // IEEE flags {V,Z,O,U,I} (invalid,divzero,ovf,unf,inexact)

   wire [EW:0] 	 w_rnd_exp_out;
   wire [EW:0] 	 w_rnd_exp_in;

   wire [FW+4:0] w_rnd_mant_out;
   wire [FW+4:0] w_rnd_mant_in;

   wire [FW+4:0] w_nrm_mant_out;
   wire [EW:0] 	 w_nrm_exp_out;
   
   wire [(2*(FW+1))-1:0] w_prod;
   wire [EW:0] 		 w_exp;

   /* inputs to the multiplier */
   wire [FW:0] 	 w_mant_a = {1'b1, a[FW-1:0]};
   wire [FW:0] 	 w_mant_b = {1'b1, b[FW-1:0]};
   wire [EW-1:0] w_exp_a = a[W-2:FW];
   wire [EW-1:0] w_exp_b = b[W-2:FW];
   wire 	 w_sign, w_zero, w_nan, w_infinity;
   assign w_sign = a[W-1] ^ b[W-1];
   
   logic [EW-1:0]  t_exp;
   logic [FW-1:0]  t_mant;
   logic 	   t_sign;
  
   fracmul #(.W(FW+1)) m0 
     (.y(w_prod), .a(w_mant_a), .b(w_mant_b));
   
   expadd #(.W(EW)) e0 
     (.y(w_exp), .a(w_exp_a), .b(w_exp_b));

   detection #(.E(EW), .F(FW)) d0 
     (.a(a),.b(b),.zero(w_zero), .nan(w_nan), .infinity(w_infinity));
   
   // low product bits below the round bit collapse into a sticky (bit 0)
   normalize #(.E(EW), .F(FW)) n0
     (.exp_out(w_rnd_exp_in), .mant_out(w_rnd_mant_in),
      .exp_in(w_exp),.mant_in({w_prod[2*(FW+1)-1:(FW-2)], |w_prod[(FW-3):0]}));
      
   round #(.E(EW), .F(FW)) r0 (
	     .exp_out(w_rnd_exp_out),
	     .mant_out(w_rnd_mant_out),
	     .exp_in(w_rnd_exp_in),
	     .mant_in(w_rnd_mant_in),
	     .rm(rm),
	     .sign(w_sign)
	     );
   
   normalize #(.E(EW), .F(FW)) n1 (
		 // Outputs
		 .exp_out		(w_nrm_exp_out),
		 .mant_out		(w_nrm_mant_out),
		 // Inputs
		 .exp_in		(w_rnd_exp_out),
		 .mant_in		(w_rnd_mant_out)
		 );

  
   // True (unwrapped, signed) result exponent for over/underflow decisions:
   // exp_a + exp_b - BIAS, +1 if the product was in [2,4) (n0 shifted right),
   // +1 if rounding carried out of the mantissa (n1 shifted right).
   wire 		w_prod_top    = w_prod[2*(FW+1)-1];
   wire 		w_round_carry = w_rnd_mant_out[FW+4];
   wire [EW+1:0]	w_exp_sum     = {2'b0, w_exp_a} + {2'b0, w_exp_b}
				    + {{(EW+1){1'b0}}, w_prod_top} + {{(EW+1){1'b0}}, w_round_carry};
   wire signed [EW+2:0] w_exp_real = $signed({1'b0, w_exp_sum}) - $signed((EW+3)'(BIAS));

   wire 		w_overflow  = (w_exp_real >= $signed((EW+3)'(INFINITY)));
   wire 		w_underflow = (w_exp_real <= $signed((EW+3)'(0))) & ~w_zero;

   // overflow default depends on the mode (R4000 Table 7-1):
   //  RN -> inf;  RZ -> max-finite;  RP -> +inf / -max;  RM -> -inf / +max
   wire 		w_ovf_inf =
			(rm == 2'd0) ? 1'b1 :
			(rm == 2'd1) ? 1'b0 :
			(rm == 2'd2) ? ~w_sign :
			                w_sign;

   // -------- NaN / infinity inputs (IEEE special cases) --------
   // NaN2008 convention: quiet bit = frac MSB. sNaN = NaN with that bit clear.
   wire 		w_a_is_nan  = (&a[W-2:FW]) & (|a[FW-1:0]);
   wire 		w_b_is_nan  = (&b[W-2:FW]) & (|b[FW-1:0]);
   wire 		w_a_is_inf  = (&a[W-2:FW]) & ~(|a[FW-1:0]);
   wire 		w_b_is_inf  = (&b[W-2:FW]) & ~(|b[FW-1:0]);
   wire 		w_a_is_snan = w_a_is_nan & ~a[FW-1];
   wire 		w_b_is_snan = w_b_is_nan & ~b[FW-1];
   wire 		w_a_is_zero = (a[W-2:0] == 'd0);
   wire 		w_b_is_zero = (b[W-2:0] == 'd0);
   wire 		w_any_nan   = w_a_is_nan | w_b_is_nan;
   wire 		w_special   = w_any_nan | w_a_is_inf | w_b_is_inf;
   // inf * 0 is the invalid case for multiply
   wire 		w_inf_x_zero = (w_a_is_inf & w_b_is_zero) | (w_b_is_inf & w_a_is_zero);
   wire 		w_invalid    = w_a_is_snan | w_b_is_snan | w_inf_x_zero;
   localparam [W-1:0] DEF_NAN = {1'b1, {EW{1'b1}}, 1'b1, {(FW-1){1'b0}}};
   wire [W-1:0] 	w_nan_src  = w_a_is_nan ? a : b;
   wire [W-1:0] 	w_qnan     = {w_nan_src[W-1:FW], 1'b1, w_nan_src[FW-2:0]};
   wire [W-1:0] 	w_specinf  = {w_sign, {EW{1'b1}}, {FW{1'b0}}};
   wire [W-1:0] 	w_special_y = w_any_nan ? w_qnan : w_inf_x_zero ? DEF_NAN : w_specinf;

   // IEEE exception flags. Special (NaN/inf) inputs raise only V (invalid).
   // A zero operand gives an exact zero -> no flags. Inexact = product had bits
   // below the round point OR overflowed. Z (div-by-zero) never for mul.
   wire 		w_mul_inexact = |w_rnd_mant_in[2:0];
   wire 		w_f_inexact   = ~w_special & ~w_zero & (w_mul_inexact | w_overflow);
   wire 		w_f_overflow  = ~w_special & ~w_zero & w_overflow;
   wire 		w_f_underflow = ~w_special & ~w_zero & w_underflow;
   wire [4:0] 		w_fflags = {w_invalid, 1'b0, w_f_overflow, w_f_underflow, w_f_inexact};

   // Denormal/underflow: subnormal operand, or a result that is subnormal or
   // smaller. Value is don't-care (punted to software, R4000 E-trap model).
   wire 		w_a_denorm = (w_exp_a == 'd0) & (a[FW-1:0] != 'd0);
   wire 		w_b_denorm = (w_exp_b == 'd0) & (b[FW-1:0] != 'd0);
   wire 		w_denorm   = ~w_special & (w_a_denorm | w_b_denorm | w_underflow);

   always_comb
     begin
	t_exp = w_nrm_exp_out[EW-1:0];
	t_mant = w_nrm_mant_out[FW+2:3];
	t_sign = a[W-1]^b[W-1];
	if(w_zero)
	  begin
	     t_exp = 'd0;
	     t_mant = 'd0;
	  end
	else if(w_overflow)
	  begin
	     t_exp  = w_ovf_inf ? {EW{1'b1}} : {{(EW-1){1'b1}}, 1'b0};
	     t_mant = w_ovf_inf ? {FW{1'b0}} : {FW{1'b1}};
	  end
     end // always_comb

   // NaN/inf inputs override the normal datapath result.
   wire [W-1:0] w_pipe_y = w_special ? w_special_y : {t_sign, t_exp, t_mant};


`ifdef DEBUG_FPU
   logic [W-1:0] t_dpi;
   generate
      if(W == 32)
	begin
	   bogo_fp32_mul bfp32(a,b,t_dpi);
	end
      else
	begin
	   bogo_fp64_mul bfp64(a,b,t_dpi);
	end
   endgenerate
   shiftreg #(.W(W), .D(MUL_LAT))
   sr0 (
	.clk(clk),
	.in(t_dpi),
	.out(y)
	);
   assign denorm = 1'b0;
   assign fflags = 5'd0;
`else
   // carry the denorm flag and IEEE flags alongside the data so they stay
   // aligned with y at the output.
   wire [W+5:0] w_pipe_out;
   shiftreg #(.W(W+6), .D(MUL_LAT))
   sr0 (
	.clk(clk),
	.in({w_fflags, w_denorm, w_pipe_y}),
	.out(w_pipe_out)
	);
   assign y      = w_pipe_out[W-1:0];
   assign denorm = w_pipe_out[W];
   assign fflags = w_pipe_out[W+5:W+1];
`endif
   
endmodule // sp_mul


   
