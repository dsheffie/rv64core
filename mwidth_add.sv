`include "machine.vh"
module mwidth_add(A, B, Y);
   input [`M_WIDTH-1:0] A;
   input [`M_WIDTH-1:0] B;
   output [`M_WIDTH-1:0] Y;
   assign Y = A+B;
endmodule
