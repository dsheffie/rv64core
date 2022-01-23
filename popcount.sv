module popcount#(parameter LG_N = 2)(in, out);
   localparam N = 1<<LG_N;
   localparam N2 = 1<<(LG_N-1);   
   input logic [N-1:0] in;
   output logic [LG_N:0] out;

   generate
      if(LG_N == 2)
	begin
	   always_comb
	     begin
		out = 'd0;
		case(in)
		  4'b0000:
		    out = 'd0;
		  4'b0001:
		    out = 'd1;
		  4'b0010:
		    out = 'd1;
		  4'b0011:
		    out = 'd2;
		  4'b0100:
		    out = 'd1;
		  4'b0101:
		    out = 'd2;
		  4'b0110:
		    out = 'd2;
		  4'b0111:
		    out = 'd3;
		  4'b1000:
		    out = 'd1;
		  4'b1001:
		    out = 'd2;
		  4'b1010:
		    out = 'd2;
		  4'b1011:
		    out = 'd3;
		  4'b1100:
		    out = 'd2;
		  4'b1101:
		    out = 'd3;
		  4'b1110:
		    out = 'd3;
		  4'b1111:
		    out = 'd4;
		endcase // case (in)
	     end
	end // if (LG_N == 2)
      else
	begin
	   logic [LG_N-1:0] t0, t1;
	   popcount #(LG_N-1) u0 (.in(in[N2-1:0]), .out(t0));
	   popcount #(LG_N-1) u1 (.in(in[N-1:N2]), .out(t1));
	   assign out = {1'b0, t0} + {1'b0, t1};
	end // else: !if(LG_N == 2)
      endgenerate
endmodule // popcount

		  
			  
