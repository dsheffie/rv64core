module ram2r1w(clk, rd_addr0, rd_addr1, wr_addr, wr_data, wr_en, rd_data0, rd_data1);
   input logic clk;
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic [LG_DEPTH-1:0] rd_addr0;
   input logic [LG_DEPTH-1:0] rd_addr1;
   input logic [LG_DEPTH-1:0] wr_addr;
   input logic [WIDTH-1:0] wr_data;
   input logic 		      wr_en;
   output logic [WIDTH-1:0] rd_data0;
   output logic [WIDTH-1:0] rd_data1;

   ram1r1w #(.WIDTH(WIDTH), .LG_DEPTH(LG_DEPTH)) b0
     (
      .clk(clk),
      .rd_addr(rd_addr0),
      .wr_addr(wr_addr),
      .wr_data(wr_data),
      .wr_en(wr_en),
      .rd_data(rd_data0)
      );

   ram1r1w #(.WIDTH(WIDTH), .LG_DEPTH(LG_DEPTH)) b1
     (
      .clk(clk),
      .rd_addr(rd_addr1),
      .wr_addr(wr_addr),
      .wr_data(wr_data),
      .wr_en(wr_en),
      .rd_data(rd_data1)
      );
   
endmodule // ram2r1w

module ram2r1w_byte_en(clk, rd_addr0, rd_addr1, wr_addr, wr_data, wr_en, wr_byte_en, rd_data0, rd_data1);
   input logic clk;
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic [LG_DEPTH-1:0] rd_addr0;
   input logic [LG_DEPTH-1:0] rd_addr1;
   input logic [LG_DEPTH-1:0] wr_addr;
   input logic [WIDTH-1:0] wr_data;
   input logic 		      wr_en;
   input logic [(WIDTH/8)-1:0] wr_byte_en;
   
   output logic [WIDTH-1:0] rd_data0;
   output logic [WIDTH-1:0] rd_data1;

   ram1r1w_byte_en #(.WIDTH(WIDTH), .LG_DEPTH(LG_DEPTH)) b0
     (
      .clk(clk),
      .rd_addr(rd_addr0),
      .wr_addr(wr_addr),
      .wr_data(wr_data),
      .wr_en(wr_en),
      .wr_byte_en(wr_byte_en),
      .rd_data(rd_data0)
      );

   ram1r1w_byte_en #(.WIDTH(WIDTH), .LG_DEPTH(LG_DEPTH)) b1
     (
      .clk(clk),
      .rd_addr(rd_addr1),
      .wr_addr(wr_addr),
      .wr_data(wr_data),
      .wr_en(wr_en),
      .wr_byte_en(wr_byte_en),      
      .rd_data(rd_data1)
      );
   
endmodule
