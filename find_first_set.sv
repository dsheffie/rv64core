module find_first_set#(parameter LG_N = 2)(in, y);
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
		y = 3'b111;
		casez(in)
		  4'b0001:
		    y = 3'd0;
		  4'b001?:
		    y = 3'd1;
		  4'b01??:
		    y = 3'd2;
		  4'b1???:
		    y = 3'd3;
		  default:
		    y = 3'b111;
		endcase // casez (in)
	     end // always_comb
	end // if (LG_N == 2)
      else
	begin
	  
	   find_first_set#(LG_N-1) f0(.in(in[N2-1:0]), .y(t0));
	   find_first_set#(LG_N-1) f1(.in(in[N-1:N2]), .y(t1));
	   always_comb
	     begin
		y = N;
		if(lo_z && hi_z)
		  y = N;
		else if(!hi_z)
		  y = N2 + t1;
		else if(!lo_z)
		  y = {1'b0, t0};
	     end
		  //assign y = (in[N2-1:0]=='d0) ? (t1 + N2) : {1'b0, t0};
	end
   endgenerate
endmodule // find_first_set

