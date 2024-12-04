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


module ram_banked_2r2w(clk, rd_addr0, rd_addr1, wr_addr0, wr_addr1, wr_data0, wr_data1, wr_en0, wr_en1, rd_data0, rd_data1);
   input logic clk;
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic [LG_DEPTH-1:0] rd_addr0;
   input logic [LG_DEPTH-1:0] rd_addr1;
   input logic [LG_DEPTH-1:0] wr_addr0;
   input logic [LG_DEPTH-1:0] wr_addr1;   
   input logic [WIDTH-1:0]    wr_data0;
   input logic [WIDTH-1:0]    wr_data1;   
   input logic		      wr_en0;
   input logic		      wr_en1;   
   
   output logic [WIDTH-1:0] rd_data0;
   output logic [WIDTH-1:0] rd_data1;

   logic		    r_rd0_b0, r_rd1_b0;
   wire [WIDTH-1:0]	    w_rd_data0, w_rd_data1;

   wire			    w_rd0_b0 = rd_addr0[0]==1'b0;
   wire			    w_rd1_b0 = rd_addr1[0]==1'b0;

   wire			    w_wr0_b0 = wr_addr0[0]==1'b0;
   wire			    w_wr1_b0 = wr_addr1[0]==1'b0;

   
   wire [LG_DEPTH-1:0]	    w_rd_addr0 = w_rd0_b0 ? rd_addr0 : rd_addr1;
   wire [LG_DEPTH-1:0]	    w_rd_addr1 = w_rd1_b0 ? rd_addr0 : rd_addr1;

   wire [LG_DEPTH-1:0]	    w_wr_addr0 = w_wr0_b0 ? wr_addr0 : wr_addr1;
   wire [LG_DEPTH-1:0]	    w_wr_addr1 = w_wr1_b0 ? wr_addr0 : wr_addr1;

   wire [WIDTH-1:0]	    w_wr_data0 = w_wr0_b0 ? wr_data0 : wr_data1;
   wire [WIDTH-1:0]	    w_wr_data1 = w_wr1_b0 ? wr_data0 : wr_data1;   

   wire			    w_wr_en0 = w_wr0_b0 & wr_en0 | w_wr1_b0 & wr_en1;
   wire			    w_wr_en1 = (w_wr0_b0==1'b0) & wr_en0 | (w_wr1_b0==1'b0) & wr_en1;   
   

   always_ff@(posedge clk)
     begin
	r_rd0_b0 <= w_rd0_b0;
	r_rd1_b0 <= w_rd1_b0;
     end

   always_comb
     begin
	rd_data0 = r_rd0_b0 ? w_rd_data0 : w_rd_data1;
	rd_data1 = r_rd1_b0 ? w_rd_data0 : w_rd_data1;
     end

`ifdef VERILATOR
   logic [WIDTH-1:0] r_check[(1<<LG_DEPTH)-1:0];
   logic [WIDTH-1:0] r_check_d0, r_check_d1;
   always_ff@(posedge clk)
     begin
	r_check_d0 <= r_check[rd_addr0];
	r_check_d1 <= r_check[rd_addr1];
	
	if(wr_en0)
	  r_check[wr_addr0] <= wr_data0;
	if(wr_en1)
	  r_check[wr_addr1] <= wr_data1;
     end

   
   always_ff@(negedge clk)
     begin
	//$display("port0 read bank %d, port1 read bank %d", rd_addr0, rd_addr1);
	if(rd_data0 != r_check_d0)
	  begin
	     $display("port 0 got wrong data");
	  end

	if(rd_data1 != r_check_d1)
	  begin
	     $display("port 1 got wrong data");
	  end
		
	if(rd_addr0[0] == rd_addr1[0])
	  begin
	     $display("reads to same bank");
	  end
	if(wr_en0 & wr_en1 & (wr_addr0[0] == wr_addr1[0]))
	  begin
	     $stop();
	  end
     end
`endif

   
   ram1r1w #(.WIDTH(WIDTH), .LG_DEPTH(LG_DEPTH-1)) b0
     (
      .clk(clk),
      .rd_addr(w_rd_addr0[LG_DEPTH-1:1]),
      .wr_addr(w_wr_addr0[LG_DEPTH-1:1]),
      .wr_data(w_wr_data0),
      .wr_en(w_wr_en0),
      .rd_data(w_rd_data0)
      );

   ram1r1w #(.WIDTH(WIDTH), .LG_DEPTH(LG_DEPTH-1)) b1
     (
      .clk(clk),
      .rd_addr(w_rd_addr1[LG_DEPTH-1:1]),
      .wr_addr(w_wr_addr1[LG_DEPTH-1:1]),
      .wr_data(w_wr_data1),
      .wr_en(w_wr_en1),
      .rd_data(w_rd_data1)
      );
   
endmodule // ram2r1w


module ram2r1w_l1d_data(clk, rd_addr0, rd_addr1, wr_addr, wr_data, wr_en, wr_byte_en, rd_data0, rd_data1);
   input logic clk;
   parameter LG_DEPTH = 1;
   localparam WIDTH = 128;
   input logic [LG_DEPTH-1:0] rd_addr0;
   input logic [LG_DEPTH-1:0] rd_addr1;
   input logic [LG_DEPTH-1:0] wr_addr;
   input logic [WIDTH-1:0] wr_data;
   input logic 		      wr_en;
   input logic [(WIDTH/8)-1:0] wr_byte_en;
   
   output logic [WIDTH-1:0] rd_data0;
   output logic [WIDTH-1:0] rd_data1;

   ram1r1w_l1d_data #(.LG_DEPTH(LG_DEPTH)) b0
     (
      .clk(clk),
      .rd_addr(rd_addr0),
      .wr_addr(wr_addr),
      .wr_data(wr_data),
      .wr_en(wr_en),
      .wr_byte_en(wr_byte_en),
      .rd_data(rd_data0)
      );

   ram1r1w_l1d_data #(.LG_DEPTH(LG_DEPTH)) b1
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
