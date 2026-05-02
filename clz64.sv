module clz64(A, Y);
   input logic [63:0] A;
   output logic [6:0] Y;

   wire [63:0]      nA = ~A;
   wire [63:0]      wZ;
   wire [6:0]       wy;


   genvar           i;

   assign wZ[0] = &nA[63:0];

   generate
      for(i = 1; i < 64; i=i+1)
        begin
           assign wZ[i] = (&nA[63:(63-i+1)]) & A[63-i]; //one                                                                                                                                                  
        end
   endgenerate

   find_first_set #(.LG_N(6)) ffs (.in(wZ), .y(wy));

   assign Y = (A[63]) ? 'd0 : wy;

   
endmodule
