
module zero_detector(
   // Outputs
   distance,
   // Inputs
   a
   );
   parameter LG_W = 5;
   parameter W = 24;
   input logic [W:0] a;
   output logic [LG_W-1:0] distance;
   localparam WW = 1 << LG_W;
   localparam ZP = WW - W - 1;
   wire [ZP-1:0] w_zp = {ZP{1'b0}};
   wire [WW-1:0] w_a_pad = {a, w_zp};
   logic [LG_W:0]    t_ffs;
   count_leading_zeros#(.LG_N(LG_W)) zffs (w_a_pad, t_ffs);
   always_comb
     begin
	distance = t_ffs[LG_W-1:0];
	if(t_ffs >= W)
	  begin
	     distance = W;
	  end
     end
endmodule // zero_detector



module fp_add(/*AUTOARG*/
   // Outputs
   y, denorm, fflags,
   // Inputs
   clk, sub, a, b, en, rm
   );
   parameter W = 32;
   parameter ADD_LAT = 2;

   input logic clk;
   input logic sub;
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   input logic	       en;
   input logic [1:0]   rm;          // 0=RN(ties-even) 1=RZ 2=RP(+inf) 3=RM(-inf)
   output logic [W-1:0] y;
   output logic		denorm;
   output logic [4:0]	fflags;     // IEEE flags {V,Z,O,U,I} (invalid,divzero,ovf,unf,inexact)

   localparam FW = (W==32) ? 23 : 52;
   localparam EW = (W==32) ? 8 : 11;
   localparam INFINITY = (1 << EW) - 1;
   
   wire 	 w_sign_toggle_b = sub ? ~b[W-1] : b[W-1];
   wire [W-1:0]  w_b = {w_sign_toggle_b, b[W-2:0]};
   wire 	 w_a_is_zero = (a[W-2:0] == 'd0);
   wire 	 w_b_is_zero = (b[W-2:0] == 'd0);

   logic [W-1:0] t_aligned_a,t_aligned_b;
   logic [EW-1:0] t_dist_a, t_dist_b;

   
   logic [FW+3:0] t_a_mant,t_b_mant;
   logic [FW+3:0] t_a_align_mant,t_b_align_mant;
   logic [EW:0]   t_align_exp;

   always_comb
     begin
	t_a_mant = {1'b1, a[FW-1:0], 3'd0};
	t_b_mant = {1'b1, b[FW-1:0], 3'd0};
	t_dist_a =  a[W-2:FW] - b[W-2:FW];
	t_dist_b =  b[W-2:FW] - a[W-2:FW];
     end
   
   // sticky = OR of the bits shifted out of the smaller operand's mantissa.
   // The shift collapses everything below the round bit into bit[0].
   wire a_shifted = |(t_a_mant & ~({(FW+4){1'b1}} << t_dist_b));
   wire b_shifted = |(t_b_mant & ~({(FW+4){1'b1}} << t_dist_a));

   //align inputs
   always_comb
     begin
	t_a_align_mant = t_a_mant;
	t_align_exp = {1'b0,a[W-2:FW]};
	t_b_align_mant = t_b_mant;
	if(a[W-2:FW] > b[W-2:FW])
	  begin
	     t_b_align_mant = (t_b_mant >> t_dist_a) | {{(FW+3){1'b0}}, b_shifted};
	  end
	else if(b[W-2:FW] > a[W-2:FW])
	  begin
	     t_a_align_mant = (t_a_mant >> t_dist_b) |{{(FW+3){1'b0}}, a_shifted};
	     t_align_exp = {1'b0,b[W-2:FW]};
	  end
     end // always_comb
   

   
   //perform add
   logic [FW+4:0] t_align_sum;
   logic 	  t_align_sign;
   always_comb
     begin
	t_align_sum = {1'b0, t_a_align_mant} + t_b_align_mant;
	t_align_sign = a[W-1];
	if(a[W-1] != w_b[W-1])
	  begin
	     if(t_a_align_mant > t_b_align_mant)
	       begin
		  t_align_sum = {1'b0, t_a_align_mant} - t_b_align_mant;
		  t_align_sign = a[W-1];
	       end
	     else
	       begin
		  t_align_sum = {1'b0, t_b_align_mant} - t_a_align_mant;
		  t_align_sign = w_b[W-1];
	       end
	  end // if (a[W-1] != w_b[W-1])
     end // always_comb

   //check add
   logic [FW:0] t_add_mant;
   logic [EW:0] t_add_exp;
   logic 	t_guard,t_round,t_sticky;

   
   always_comb
     begin
	t_add_mant = t_align_sum[FW+3:3];
	t_guard = t_align_sum[2];
	t_round = t_align_sum[1];
	t_sticky = t_align_sum[0];
	t_add_exp = t_align_exp;
	if(t_align_sum[FW+4])
	  begin
	     t_add_mant = t_align_sum[FW+4:4];
	     t_guard = t_align_sum[3];
	     t_round = t_align_sum[2];
	     t_sticky = t_align_sum[1] | t_align_sum[0];
	     t_add_exp = t_align_exp + 'd1;
	  end
     end
   //normalize
   logic [FW:0] t_norm1_add_mant;
   logic [EW:0] t_norm1_add_exp;
   logic 	t_norm1_guard,t_norm1_round,t_norm1_sticky;

   localparam LG_FW = FW==23 ? 5 : 6;
   wire [LG_FW-1:0] w_shft_lft_dist;
   localparam ZP = (EW+1) - LG_FW;
   
   zero_detector #(.LG_W(LG_FW), .W(FW)) zd 
     (.distance(w_shft_lft_dist), .a(t_add_mant));
   
   wire [EW:0] 	    w_shift_dist = {{ZP{1'b0}}, w_shft_lft_dist};   
      
   always_comb
     begin
	t_norm1_add_mant = t_add_mant;
	t_norm1_add_exp = t_add_exp;
	t_norm1_guard = t_guard;
	t_norm1_round = t_round;
	t_norm1_sticky = t_sticky;
	
	if(t_add_mant[FW] == 1'b0 && (t_add_exp != 'd0))
	  begin
	     //how to handle guard, sticky, round
	     t_norm1_add_exp = t_add_exp - w_shift_dist;
	     if(w_shift_dist == 'd1)
	       begin
		  t_norm1_guard = t_norm1_round;
		  t_norm1_round = 1'b0;		  
	       end
	     else
	       begin
		  t_norm1_guard = 1'b0;
		  t_norm1_round = 1'b0;
	       end
	     if(w_shift_dist == 'd1)
	       begin
		  t_norm1_add_mant = {t_add_mant[FW-1:0], t_guard};
	       end
	     else
	       begin
		  t_norm1_add_mant = {t_add_mant[FW-2:0], t_guard, t_round} << (w_shift_dist - 'd2);
	       end
	  end
     end

   logic [FW:0] t_norm2_add_mant;
   logic [EW:0] t_norm2_add_exp;
   logic 	t_norm2_guard,t_norm2_round,t_norm2_sticky;

   always_comb
     begin
	t_norm2_add_mant = t_norm1_add_mant;
	t_norm2_add_exp = t_norm1_add_exp;
	t_norm2_guard = t_norm1_guard;
	t_norm2_round = t_norm1_round;
	t_norm2_sticky = t_norm1_sticky;
	//if(t_norm1_add_exp == 'd0)
	//begin
	//end
     end

   logic [FW:0] t_round_add_mant;
   logic [EW:0] t_round_add_exp;

   // Round-up (increment magnitude) decision per rounding mode. inexact = any
   // discarded bit set; for RP/RM the direction depends on the result sign.
   wire 	w_add_inexact = t_norm2_guard | t_norm2_round | t_norm2_sticky;
   wire 	w_round_up =
		(rm == 2'd0) ? (t_norm2_guard & (t_norm2_round | t_norm2_sticky | t_norm2_add_mant[0])) :
		(rm == 2'd1) ? 1'b0 :
		(rm == 2'd2) ? (~t_align_sign & w_add_inexact) :
		               ( t_align_sign & w_add_inexact);

   always_comb
     begin
	t_round_add_mant = t_norm2_add_mant;
	t_round_add_exp = t_norm2_add_exp;
	if (w_round_up)
	  begin
	     if(t_norm2_add_mant == {FW+1{1'b1}})
	       begin
		  t_round_add_exp = t_norm2_add_exp + 'd1;
		  t_round_add_mant = {1'b1, {{FW{1'b0}}}};
	       end
	     else
	       begin
		  t_round_add_mant = t_norm2_add_mant + 'd1;
	       end
	  end
     end

   wire w_is_zero = (a[W-1] ^ w_b[W-1]) & (t_round_add_mant=='d0);
   //wire w_is_zero = 1'b0;

   // -------- NaN / infinity inputs (IEEE special cases) --------
   // NaN2008 convention: quiet bit = frac MSB. sNaN = NaN with that bit clear.
   wire 	w_a_is_nan  = (&a[W-2:FW]) & (|a[FW-1:0]);
   wire 	w_b_is_nan  = (&b[W-2:FW]) & (|b[FW-1:0]);
   wire 	w_a_is_inf  = (&a[W-2:FW]) & ~(|a[FW-1:0]);
   wire 	w_b_is_inf  = (&b[W-2:FW]) & ~(|b[FW-1:0]);
   wire 	w_a_is_snan = w_a_is_nan & ~a[FW-1];
   wire 	w_b_is_snan = w_b_is_nan & ~b[FW-1];
   wire 	w_any_nan   = w_a_is_nan | w_b_is_nan;
   wire 	w_special   = w_any_nan | w_a_is_inf | w_b_is_inf;
   // inf - inf (opposite effective signs) is the invalid case for add
   wire 	w_inf_sub_inf = w_a_is_inf & w_b_is_inf & (a[W-1] ^ w_b[W-1]);
   wire 	w_invalid     = w_a_is_snan | w_b_is_snan | w_inf_sub_inf;
   // NaN result: propagate (prefer a) and force quiet; default NaN otherwise
   localparam [W-1:0] DEF_NAN = {1'b1, {EW{1'b1}}, 1'b1, {(FW-1){1'b0}}};
   wire [W-1:0] w_nan_src = w_a_is_nan ? a : b;
   wire [W-1:0] w_qnan    = {w_nan_src[W-1:FW], 1'b1, w_nan_src[FW-2:0]};
   wire [W-1:0] w_specinf = w_a_is_inf ? {a[W-1], {EW{1'b1}}, {FW{1'b0}}}
			                : {w_b[W-1], {EW{1'b1}}, {FW{1'b0}}};
   wire [W-1:0] w_special_y = w_any_nan ? w_qnan : w_inf_sub_inf ? DEF_NAN : w_specinf;

   // exponent overflow: the biased exponent reaches the reserved value when
   // bit[EW] is set (>= 256, e.g. round of an all-ones mantissa) or the low EW
   // bits are all 1 (== 255). With finite operands it cannot grow beyond that.
   wire 	w_overflow = t_round_add_exp[EW] | (&t_round_add_exp[EW-1:0]);
   // overflow default result depends on the mode (R4000 Table 7-1):
   //  RN -> inf;  RZ -> max-finite;  RP -> +inf / -max;  RM -> -inf / +max
   wire 	w_ovf_inf =
		(rm == 2'd0) ? 1'b1 :
		(rm == 2'd1) ? 1'b0 :
		(rm == 2'd2) ? ~t_align_sign :
		                t_align_sign;
   wire [W-1:0] w_inf    = {t_align_sign, {EW{1'b1}}, {FW{1'b0}}};
   wire [W-1:0] w_maxfin = {t_align_sign, {(EW-1){1'b1}}, 1'b0, {FW{1'b1}}};
   wire [W-1:0] w_ovf_y  = w_ovf_inf ? w_inf : w_maxfin;

   wire [W-1:0] w_y = w_special ? w_special_y :
		w_is_zero ? 'd0 :
		w_a_is_zero ? w_b :
		w_b_is_zero ? a :
		w_overflow ? w_ovf_y :
		{t_align_sign, t_round_add_exp[EW-1:0], t_round_add_mant[FW-1:0]};

   // Denormal detection. We don't compute correct subnormal results (the R4000
   // punts these to the software emulator via the Unimplemented (E) trap); we
   // just flag that a denormal is involved so the caller can take that path.
   wire w_a_denorm = (a[W-2:FW] == 'd0) & (a[FW-1:0] != 'd0);
   wire w_b_denorm = (b[W-2:FW] == 'd0) & (b[FW-1:0] != 'd0);

   // result underflow: leading-zero normalize drives the biased exponent <= 0
   wire [EW:0] 		w_lead = (t_add_mant[FW] == 1'b0) ? w_shift_dist : {(EW+1){1'b0}};
   wire signed [EW+1:0] w_real_exp = $signed({1'b0, t_add_exp}) - $signed({1'b0, w_lead});
   // w_real_exp <= 0 : sign bit set (negative) or all-zero
   wire 		w_res_denorm = (w_real_exp[EW+1] | ~(|w_real_exp)) & (t_round_add_mant != 'd0) & ~w_is_zero;

   wire 		w_denorm = ~w_special & (w_a_denorm | w_b_denorm | w_res_denorm);

   // IEEE exception flags. Special (NaN/inf) inputs raise only V (invalid).
   // The zero / passthrough cases (a+0, 0+b, a-a) are exact, so raise nothing.
   // Inexact = rounded-away bits OR overflow. Z (div-by-zero) never for add.
   wire 		w_exact_path = w_special | w_is_zero | w_a_is_zero | w_b_is_zero;
   wire 		w_f_inexact   = ~w_exact_path & (w_add_inexact | w_overflow);
   wire 		w_f_overflow  = ~w_exact_path & w_overflow;
   wire 		w_f_underflow = ~w_exact_path & w_res_denorm;
   wire [4:0] 		w_fflags = {w_invalid, 1'b0, w_f_overflow, w_f_underflow, w_f_inexact};

   // ADD_LAT-deep output pipeline. Result, denorm flag and IEEE flags travel
   // together through the same registers so they stay aligned with y.
   logic [W+5:0] r_pipe [ADD_LAT-1:0];
   integer 	 i;
   always_ff @(posedge clk)
     begin
	r_pipe[0] <= {w_fflags, w_denorm, w_y};
	for(i = 1; i < ADD_LAT; i = i + 1)
	  r_pipe[i] <= r_pipe[i-1];
     end

   assign y      = r_pipe[ADD_LAT-1][W-1:0];
   assign denorm = r_pipe[ADD_LAT-1][W];
   assign fflags = r_pipe[ADD_LAT-1][W+5:W+1];

endmodule // sp_add
