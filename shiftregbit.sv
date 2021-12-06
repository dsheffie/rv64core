module dffen(q,d,clk,reset,en);
   parameter N = 1;
   input logic [N-1:0] d;
   input logic 	       clk;
   input logic 	       reset;
   input logic 	       en;
   output logic [N-1:0] q;
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     q <= 1'b0;
	  end
	else
	  begin
	     q <= en ? d : q;
	  end
     end // always_ff@ (posedge clk)
endmodule // dff


module shiftregbit(clk,reset,b,valid,out);
   input logic clk;
   input logic reset;
   input logic b;
   input logic valid;
   parameter W = 32;
   output logic [W-1:0] out;

   for(genvar i = 0; i < W; i = i + 1)
     begin : sr
	if(i==0)
	  begin
	     dffen #(.N(1)) ff (.clk(clk), .reset(reset), .en(valid), .d(b), .q(out[0]));
	  end
	else
	  begin
	     dffen #(.N(1)) ff (.clk(clk), .reset(reset), .en(valid), .d(out[i-1]), .q(out[i]));	     
	  end
     end
endmodule

