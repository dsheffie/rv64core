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
   
`ifdef XILINX_FPGA
   integer 		    i;
   initial
     begin
        for (i=0; i<DEPTH; i=i+1)
          r_ram[i] = 0;
     end
`endif

   //always_ff@(negedge clk)
   //begin
   //if(wr_en & (rd_addr == wr_addr))
   //begin
   //$display("read and write to same loc (%x)", rd_addr);
   //end
   //end
   
   always_ff@(posedge clk)
     begin
	rd_data <= r_ram[rd_addr];
	if(wr_en)
	  begin
	     r_ram[wr_addr] <= wr_data;
	  end
     end
   
endmodule // ram1r1w


module ram1r1w_l1d_data(clk, rd_addr, wr_addr, wr_data, wr_en, wr_byte_en, rd_data);
   input logic clk;

   parameter LG_DEPTH = 1;
   
   localparam WIDTH = 128;
   localparam NUM_BYTES = WIDTH/8;
   
   input logic [LG_DEPTH-1:0] rd_addr;
   input logic [LG_DEPTH-1:0] wr_addr;
   input logic [WIDTH-1:0] wr_data;
   input logic 		      wr_en;
   input logic [NUM_BYTES-1:0] wr_byte_en;
   output logic [WIDTH-1:0] rd_data;

   localparam DEPTH = 1<<LG_DEPTH;
   logic [NUM_BYTES-1:0][7:0] r_ram[DEPTH-1:0];

`ifdef XILINX_FPGA
   integer 		    i;
   initial
     begin
        for (i=0; i<DEPTH; i=i+1)
          r_ram[i] = 0;
     end
`endif
   
   
   always_ff@(posedge clk)
     begin
	rd_data <= r_ram[rd_addr];
	if(wr_en)
	  begin
	     //$display("mask %b", wr_byte_en);
	     if(wr_byte_en[0])
	       begin
		  r_ram[wr_addr][0] <= wr_data[7:0];
	       end
	     if(wr_byte_en[1])
	       begin
		  r_ram[wr_addr][1] <= wr_data[15:8];
	       end
	     if(wr_byte_en[2])
	       begin
		  r_ram[wr_addr][2] <= wr_data[23:16];
	       end
	     if(wr_byte_en[3])
	       begin
		  r_ram[wr_addr][3] <= wr_data[31:24];
	       end
	     if(wr_byte_en[4])
	       begin
		  r_ram[wr_addr][4] <= wr_data[39:32];
	       end
	     if(wr_byte_en[5])
	       begin
		  r_ram[wr_addr][5] <= wr_data[47:40];
	       end
	     if(wr_byte_en[6])
	       begin
		  r_ram[wr_addr][6] <= wr_data[55:48];
	       end
	     if(wr_byte_en[7])
	       begin
		  r_ram[wr_addr][7] <= wr_data[63:56];
	       end
	     if(wr_byte_en[8])
	       begin
		  r_ram[wr_addr][8] <= wr_data[(9*8)-1:8*8];
	       end
	     if(wr_byte_en[9])
	       begin
		  r_ram[wr_addr][9] <= wr_data[(10*8)-1:9*8];
	       end
	     if(wr_byte_en[10])
	       begin
		  r_ram[wr_addr][10] <= wr_data[(11*8)-1:10*8];
	       end
	     if(wr_byte_en[11])
	       begin
		  r_ram[wr_addr][11] <= wr_data[(12*8)-1:11*8];
	       end
	     if(wr_byte_en[12])
	       begin
		  r_ram[wr_addr][12] <= wr_data[(13*8)-1:12*8];
	       end
	     if(wr_byte_en[13])
	       begin
		  r_ram[wr_addr][13] <= wr_data[(14*8)-1:13*8];
	       end
	     if(wr_byte_en[14])
	       begin
		  r_ram[wr_addr][14] <= wr_data[(15*8)-1:14*8];
	       end
	     if(wr_byte_en[15])
	       begin
		  r_ram[wr_addr][15] <= wr_data[(16*8)-1:15*8];
	       end
	  end
     end
   
endmodule
