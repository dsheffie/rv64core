module rf6r3w(clk,
	      rdptr0,rdptr1,rdptr2,rdptr3,rdptr4,rdptr5,
	      wrptr0,wrptr1,wrptr2,
	      wen0,wen1,wen2,
	      wr0, wr1, wr2,
	      rd0, rd1, rd2, rd3,rd4,rd5);
   
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic clk;
   input logic [LG_DEPTH-1:0] rdptr0;
   input logic [LG_DEPTH-1:0] rdptr1;
   input logic [LG_DEPTH-1:0] rdptr2;
   input logic [LG_DEPTH-1:0] rdptr3;
   input logic [LG_DEPTH-1:0] rdptr4;
   input logic [LG_DEPTH-1:0] rdptr5;   
   
   input logic [LG_DEPTH-1:0] wrptr0;
   input logic [LG_DEPTH-1:0] wrptr1;
   input logic [LG_DEPTH-1:0] wrptr2;
   
   input logic 		      wen0;
   input logic 		      wen1;
   input logic 		      wen2;
   input logic [WIDTH-1:0]    wr0;
   input logic [WIDTH-1:0]    wr1;
   input logic [WIDTH-1:0]    wr2;
   
   output logic [WIDTH-1:0]   rd0;
   output logic [WIDTH-1:0]   rd1;
   output logic [WIDTH-1:0]   rd2;
   output logic [WIDTH-1:0]   rd3;
   output logic [WIDTH-1:0]   rd4;
   output logic [WIDTH-1:0]   rd5;   
   
   localparam DEPTH = 1<<LG_DEPTH;
   logic [WIDTH-1:0] 	    r_ram[DEPTH-1:0];

   // always_ff@(negedge clk)
   //   begin
   // 	if(wen0)
   // 	  begin
   // 	     $display("writing %x to location %d on write port 0",
   // 		      wr0, wrptr0);
   // 	  end
   //   end
  
   
   always_ff@(posedge clk)
     begin
	rd0 <= rdptr0=='d0 ? 'd0 : r_ram[rdptr0];
	rd1 <= rdptr1=='d0 ? 'd0 : r_ram[rdptr1];
	rd2 <= rdptr2=='d0 ? 'd0 : r_ram[rdptr2];
	rd3 <= rdptr3=='d0 ? 'd0 : r_ram[rdptr3];
	rd4 <= rdptr4=='d0 ? 'd0 : r_ram[rdptr4];
	rd5 <= rdptr5=='d0 ? 'd0 : r_ram[rdptr5];	
	if(wen0)
	  r_ram[wrptr0] <= wr0;
	if(wen1)
	  r_ram[wrptr1] <= wr1;
	if(wen2)
	  r_ram[wrptr2] <= wr2;	
     end // always_ff@ (posedge clk)

endmodule
