`ifdef DEBUG_FPU
import "DPI-C" function int int32_to_fp32(input int a);
import "DPI-C" function longint int32_to_fp64(input int a);

module bogo_int32_to_fp32(input logic [31:0] a, output logic [31:0] y);
   always_comb
     begin
	y = int32_to_fp32(a);
     end
endmodule

module bogo_int32_to_fp64(input logic [31:0] a, output logic [63:0] y);
   always_comb
     begin
	y = int32_to_fp64(a);
     end
endmodule

`endif

module fp_convert(clk, in, en, out);
   parameter W = 32;
   localparam FW = (W==32) ? 23 : 52;
   localparam EW = (W==32) ? 8 : 11;
   localparam LG_W = (W==32) ? 5: 6;
   
   localparam PW = EW-(LG_W+1);

   input logic clk;
   input logic [W-1:0] in;
   input logic 	       en;
   output logic [W-1:0] out;
   
   wire [EW-1:0] 	w_bias = ((1<<(EW-1)) - 1);

   logic [W-1:0] 	t_in, t_mant_full;
   logic [FW-1:0] 	t_mant,t_round;
   logic [EW-1:0] 	t_exp;
   logic [LG_W:0] 	t_ffs;
   logic 		t_sign, t_zero;
   
   always_comb
     begin
	t_in = in;
	t_sign = 1'b0;
	t_zero = (in=='d0);
	if(in[W-1])
	  begin
	     t_in = ~in + 'd1;
	     t_sign = 1'b1;
	  end
     end
   
   find_first_set #(.LG_N(LG_W)) z0
     (.in(t_in), .y(t_ffs));

`ifdef DEBUG_FPU
   logic [W-1:0] t_dpi;
   generate
      if(W == 32)
	bogo_int32_to_fp32 c0(in, t_dpi);
      else
	bogo_int32_to_fp64 c0(in[31:0], t_dpi);
   endgenerate
`endif
   
   always_comb
     begin
	t_mant_full = t_in << (W - t_ffs);
	//dsheffie - not sure how rounding is supposed to work here
	t_round = t_mant_full[W-FW-1] && t_mant_full[W-FW] &&  (t_ffs > FW) ? 'd1 : 'd0;
	t_mant = t_mant_full[W-1:W-FW] + t_round;
	t_exp = w_bias + { {PW{1'b0}}, t_ffs};
	out = t_zero ? 'd0 : {t_sign,t_exp,t_mant};
`ifdef DEBUG_FPU
	out = t_dpi;
`endif
     end

`ifdef DEBUG_FPU
   always_ff@(negedge clk)
     begin
	if(en && (out != t_dpi) )
	  begin
	     $display("in = %x, out = %x, t_dpi = %x", in, out, t_dpi);
	     $display("t_mant_full = %x", t_mant_full[W-1:W-FW-1]);
	     $display("shift dist = %d", t_ffs);
	     $display("t_dpi frac    = %x", t_dpi[FW-1:0]);
	     $display("out   frac    = %x", out[FW-1:0]);
	     $display("out   frac    = %x", 
		      out[FW-1:0] + (t_mant_full[W-FW-1] ? 'd1 : 'd0));
	  end
     end
`endif

endmodule // fp_convert

