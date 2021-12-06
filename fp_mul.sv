

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

module fracmul(y,clk, a, b);
   input clk;
   parameter L = 3;
   parameter W = 24;
   localparam W2 = W*2;
   input [W-1:0]       a;
   input [W-1:0] b;
   output [W2-1:0] y;

   wire [W2-1:0] w_comb_mul = a * b;
   shiftreg #(.W(W2), .D(L)) d (.clk(clk), .in(w_comb_mul), .out(y));
endmodule // fracmul
   

module expadd(y, clk, a, b);
   parameter L = 3;
   parameter W = 8;
   input logic clk;
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   output logic [W:0] y;

   wire [W-1:0] w_bias = ((1<<(W-1)) - 1);
   wire [W:0] w_comb_add = ((a+b) - w_bias);

   shiftreg #(.W(W+1), .D(L)) d (.clk(clk), .in(w_comb_add), .out(y));
endmodule // expadd

   

module detection(
   // Outputs
   zero, nan, infinity,
   // Inputs
   clk, a, b
   );
   parameter L = 4;
   parameter E = 11;
   parameter F = 52;
   
   localparam W = E + F + 1;

      
   input logic clk;
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
	t_az = (a == 'd0);
	t_bz = (b == 'd0);
	t_detect = {t_az|t_bz, w_nan, w_infinity_a | w_infinity_b};
     end
   
   shiftreg #(.W(3), .D(L)) d (.clk(clk), .in(t_detect), .out(t_out));
   
   assign zero = t_out[2];
   assign nan = t_out[1];
   assign infinity = t_out[0];

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
	     mant_out = {1'b0, mant_in[(F+4):2], mant_in[1] & mant_in[0]};
	     exp_out = exp_in + 'd1;
	  end
     end
endmodule // normalize

module round(
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

   logic [F+1:0] t_one = {{(F+1){1'b0}},1'b1};
   always_comb
     begin
	exp_out = exp_in;
	mant_out = mant_in;
	if(mant_in[2]) 
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
   y,
   // Inputs
   clk, a, b, en
   );
   parameter SP_IMUL_LATENCY = 2;
   parameter W = 32;
   localparam FW = (W==32) ? 23 : 52;
   localparam EW = (W==32) ? 8 : 11;
   localparam INFINITY = (1 << EW) - 1;
   
   input logic clk;
   input logic [W-1:0] a;
   input logic [W-1:0] b;
   input logic 	       en;
   output logic [W-1:0] y;

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
   
   logic [EW-1:0]  r_exp;
   logic [FW-1:0]  r_mant;
   logic [(FW+4):0] r_rnd_mant_in;
   logic [EW:0]     r_rnd_exp_in;
  
   // always_ff@(negedge clk)
   //   begin
   // 	if(en)
   // 	  begin
   // 	     $display("A %x mant = %x, exp = %d", 
   // 		      a, w_mant_a, w_exp_a);
   // 	     $display("B %x mant = %x, exp = %d", b,
   // 		      w_mant_b, w_exp_b);
   // 	  end
   //   end
    
   fracmul #(.L(SP_IMUL_LATENCY), .W(FW+1)) m0 
     (.y(w_prod), .clk(clk),.a(w_mant_a), .b(w_mant_b));
   expadd #(.L(SP_IMUL_LATENCY), .W(EW)) e0 
     (.y(w_exp), .clk(clk), .a(w_exp_a), .b(w_exp_b));

   detection #(.L(SP_IMUL_LATENCY+1), .E(EW), .F(FW)) d0 
     (.clk(clk),.a(a),.b(b),.zero(w_zero), .nan(w_nan), .infinity(w_infinity));

   normalize #(.E(EW), .F(FW)) n0 
     (.exp_out(w_rnd_exp_in), .mant_out(w_rnd_mant_in),
      .exp_in(w_exp),.mant_in(w_prod[2*(FW+1)-1:(FW-3)]));
   
   
   always_ff@(posedge clk)
     begin
	r_rnd_mant_in <= w_rnd_mant_in;
	r_rnd_exp_in <= w_rnd_exp_in;
     end
        
   round #(.E(EW), .F(FW)) r0 (
	     .exp_out(w_rnd_exp_out),
	     .mant_out(w_rnd_mant_out),
	     .exp_in(r_rnd_exp_in),
	     .mant_in(r_rnd_mant_in)
	     );
   
   normalize #(.E(EW), .F(FW)) n1 (
		 // Outputs
		 .exp_out		(w_nrm_exp_out),
		 .mant_out		(w_nrm_mant_out),
		 // Inputs
		 .exp_in		(w_rnd_exp_out),
		 .mant_in		(w_rnd_mant_out)
		 );


   always_ff@(posedge clk)
     begin
	if(w_zero)
	  begin
	     r_exp <= 'd0;
	     r_mant <= 'd0;
	  end
	else if(w_infinity)
	  begin
	     r_exp <= INFINITY;
	     r_mant <= 'd0;
	  end
	else
	  begin
	     r_exp <= w_nrm_exp_out[EW-1:0];
	     r_mant <= w_nrm_mant_out[FW+2:3];
	  end
     end // always_ff@ (posedge clk)
   	
   
   sign #(SP_IMUL_LATENCY + 2) s0 (
	    .y(w_sign),
	    .clk(clk),
	    .a(a[W-1]),
	    .b(b[W-1])
	    );
   
   assign y = {w_sign, r_exp, r_mant};
      

endmodule // sp_mul


   
