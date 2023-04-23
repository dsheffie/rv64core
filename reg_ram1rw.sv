module reg_ram1rw(clk, addr, wr_data, wr_en, rd_data);
   input logic clk;
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic [LG_DEPTH-1:0] addr;
   input logic [WIDTH-1:0] wr_data;
   input logic 		      wr_en;
   output logic [WIDTH-1:0] rd_data;

   localparam DEPTH = 1<<LG_DEPTH;
   logic [WIDTH-1:0] 	    r_ram[DEPTH-1:0];
   
   logic [LG_DEPTH-1:0]     r_addr;
   logic 		    r_wr_en;
   logic [WIDTH-1:0] 	    r_wr_data;
   
   always_ff@(posedge clk)
     begin
	r_addr <= addr;
	r_wr_en <= wr_en;
	r_wr_data <= wr_data;
	
	rd_data <= r_ram[r_addr];
	if(r_wr_en)
	  begin
	     r_ram[r_addr] <= r_wr_data;
	  end
     end
   
endmodule
