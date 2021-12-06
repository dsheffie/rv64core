module count_leading_zeros#(parameter LG_N = 2)(in, y);
   localparam N = 1<<LG_N;
   localparam N2 = 1<<(LG_N-1);
   input logic [N-1:0] in;
   output logic [LG_N:0] y;

   logic [LG_N-1:0] 	 t0, t1;
   logic 		 lo_z = in[N2-1:0]=='d0;
   logic 		 hi_z = in[N-1:N2]=='d0;
   
   //always_comb
     //begin
   //$display("N = %d, in = %b, hi=%b, lo=%b, y = %d", N, in, hi_z, lo_z, y);
   //end
   
   generate
      if(LG_N == 2)
	begin
	   always_comb
	     begin
		y = 'd0;
		casez(in)
		  4'b0000:
		    y = 3'd4;
		  4'b0001:
		    y = 3'd3;
		  4'b001?:
		    y = 3'd2;
		  4'b01??:
		    y = 3'd1;
		  4'b1???:
		    y = 3'd0;
		  default:
		    y = 3'd0;
		endcase // casez (in)
	     end // always_comb
	end // if (LG_N == 2)
      else
	begin
	   count_leading_zeros#(LG_N-1) f0(.in(in[N2-1:0]), .y(t0));
	   count_leading_zeros#(LG_N-1) f1(.in(in[N-1:N2]), .y(t1));
	   
	   always_comb
	     begin
		y = N;
		if(hi_z)
		  y = N2 + t0;
		else
		  y = {1'b0, t1};
	     end
	end
   endgenerate
endmodule // find_first_set

