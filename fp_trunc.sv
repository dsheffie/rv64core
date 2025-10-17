
module fp_trunc(in, out);
   parameter W = 32;
   localparam FW = (W==32) ? 23 : 52;
   localparam EW = (W==32) ? 8 : 11;
   localparam PW = W-(FW+1);

   input logic [W-1:0] in;
   output logic [W-1:0] out;
   
   wire [EW-1:0] 	w_bias = ((1<<(EW-1)) - 1);
   
   wire [EW-1:0] 	w_exp = in[W-2:FW];
   wire [FW:0] 		w_mant = {1'b1, in[FW-1:0]};
   wire [FW:0] 		w_zp = {(FW+1){1'b0}};
   wire [PW-1:0] 	w_op = {PW{1'b0}};
 	
   wire [FW+W:0] w_mant_pad = {{(W){1'd0}}, w_mant};
   logic [FW+W:0] t_mant_shift;
   
   wire 		w_sign = in[W-1];
   logic 		t_zero;
   logic [EW-1:0] 	t_shift_dist;
   logic [W-1:0] 	t_out, t_t;

   
   always_comb
     begin
	t_zero = w_exp < w_bias;
	t_shift_dist = (w_exp - w_bias)+1;
	t_shift_dist = (t_shift_dist > (W)) ? (W) : t_shift_dist;
	t_mant_shift = w_mant_pad << t_shift_dist;
	//$display("exp = %d, bias = %d, t_shift_dist = %d",
	//w_exp, w_bias, t_shift_dist);
	t_t = 'd0;
	if(!t_zero)
	  begin
	     t_t = t_mant_shift[FW+W:FW+1];
	     if(w_sign)
	       begin
		  t_t = (~t_t + 'd1);
	       end
	  end
     end // always_comb

   generate
      if(W == 64)
	begin
	   assign t_out = {32'd0, t_t[31:0]};
	end
      else
	begin
	   assign t_out = t_t;
	end
   endgenerate

   always_comb
     begin
	out = t_out;
     end
   
endmodule
