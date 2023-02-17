module rf4r2w(clk,
	      rdptr0,rdptr1,rdptr2,rdptr3,
	      wrptr0,wrptr1,wen0,wen1,
	      wr0, wr1,
	      rd0, rd1, rd2, rd3);
   
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic clk;
   input logic [LG_DEPTH-1:0] rdptr0;
   input logic [LG_DEPTH-1:0] rdptr1;
   input logic [LG_DEPTH-1:0] rdptr2;
   input logic [LG_DEPTH-1:0] rdptr3;
   
   input logic [LG_DEPTH-1:0] wrptr0;
   input logic [LG_DEPTH-1:0] wrptr1;
   
   input logic 		      wen0;
   input logic 		      wen1;
   input logic [WIDTH-1:0]    wr0;
   input logic [WIDTH-1:0]    wr1;
   
   output logic [WIDTH-1:0]   rd0;
   output logic [WIDTH-1:0]   rd1;
   output logic [WIDTH-1:0]   rd2;
   output logic [WIDTH-1:0]   rd3;
   
   localparam DEPTH = 1<<LG_DEPTH;
   logic [WIDTH-1:0] 	    r_ram[DEPTH-1:0];

   always_ff@(posedge clk)
     begin
	rd0 <= rdptr0=='d0 ? 'd0 : r_ram[rdptr0];
	rd1 <= rdptr1=='d0 ? 'd0 : r_ram[rdptr1];
	rd2 <= rdptr2=='d0 ? 'd0 : r_ram[rdptr2];
	rd3 <= rdptr3=='d0 ? 'd0 : r_ram[rdptr3];
	if(wen0)
	  r_ram[wrptr0] <= wr0;
	if(wen1)
	  r_ram[wrptr1] <= wr1;
     end // always_ff@ (posedge clk)

endmodule
