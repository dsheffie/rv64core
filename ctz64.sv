module ctz64(A, Y);
   input logic [63:0] A;
   output logic [6:0] Y;


   wire [63:0]	w_Arev;
   genvar	      i;

   generate
      for(i = 0; i < 64; i=i+1)
	begin: rev
	   assign w_Arev[i] = A[63-i];
	end
   endgenerate

   clz64 clz(.A(w_Arev), .Y(Y));

endmodule // ctz64

