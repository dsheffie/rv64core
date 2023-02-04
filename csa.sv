module csa(a,b,cin,s,cout);
   parameter N = 64;
   input [N-1:0] a;
   input [N-1:0] b;
   input [N-1:0] cin;
   output [N-1:0] s;
   output [N-1:0] cout;

   wire [N-1:0]   w_xor_ab = a^b;
   assign s = w_xor_ab ^ cin;
   assign cout = a&b | (cin & w_xor_ab);
endmodule // csa
