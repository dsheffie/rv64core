module ram1r1w(clk, rd_addr, wr_addr, wr_data, wr_en, rd_data);
   input logic clk;
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic [LG_DEPTH-1:0] rd_addr;
   input logic [LG_DEPTH-1:0] wr_addr;
   input logic [WIDTH-1:0] wr_data;
   input logic 		      wr_en;
   output logic [WIDTH-1:0] rd_data;

   localparam DEPTH = 1<<LG_DEPTH;
   logic [WIDTH-1:0] 	    r_ram[DEPTH-1:0];
   
   
   always_ff@(posedge clk)
     begin
	rd_data <= r_ram[rd_addr];
	if(wr_en)
	  begin
	     r_ram[wr_addr] <= wr_data;
	  end
     end
   
endmodule
