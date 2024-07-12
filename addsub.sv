
module addsub(A, B, is_sub, Y);
   parameter W = 32;
   
   input [W-1:0] A;
   input [W-1:0] B;
   input	 is_sub;
   output [W-1:0] Y;

   wire [W-1:0]	  w_s, w_c;

   wire [W-1:0]	  w_zero = {{W{1'b0}}};
   wire [W-1:0]	  w_one = {{(W-1){1'b0}}, 1'b1};
   
   csa #(.N(W)) csa0 
     (
      .a(A), 
      .b(is_sub ? ~B :B ), 
      .cin(is_sub ? w_one : w_zero), 
      .s(w_s), 
      .cout(w_c) 
      );
   
   wire [W-1:0] w_srcA = {w_c[W-2:0], 1'b0};
   wire [W-1:0] w_srcB = w_s;
   
   assign Y = w_srcA+w_srcB;

endmodule
