`include "machine.vh"

`ifdef VERILATOR
import "DPI-C" function longint loadgpr(input int regid);
`endif

module rf6r3w(clk, reset,
	      rdptr0,rdptr1,rdptr2,rdptr3,rdptr4,rdptr5,
	      wrptr0,wrptr1,wrptr2,
	      wen0,wen1,wen2,
	      wr0, wr1, wr2,
	      rd0, rd1, rd2, rd3,rd4,rd5);
   
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic clk;
   input logic reset;
   
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
   localparam H_DEPTH = 1<<(LG_DEPTH-1);   
   
   logic [WIDTH-1:0] 	    r_ram_alu[H_DEPTH-1:0];
   logic [WIDTH-1:0]	    r_ram_mem[H_DEPTH-1:0];   
   
`ifdef SECOND_EXEC_PORT
   wire			    wen2_ = wen2;
   wire [LG_DEPTH-1:0]	    rdptr4_ = rdptr4;
`ifdef TWO_SRC_CHEAP   
   wire [LG_DEPTH-1:0]	    rdptr5_ = rdptr5;
`else
   wire [LG_DEPTH-1:0]	    rdptr5_ = 'd0;   
`endif
`else
   wire			    wen2_ = 1'b0;
   wire [LG_DEPTH-1:0]	    rdptr4_ = 'd0;   
   wire [LG_DEPTH-1:0]	    rdptr5_ = 'd0;   
`endif

   wire			    rd0_mem = rdptr0[LG_DEPTH-1];
   wire			    rd1_mem = rdptr1[LG_DEPTH-1];
   wire			    rd2_mem = rdptr2[LG_DEPTH-1];
   wire			    rd3_mem = rdptr3[LG_DEPTH-1];
   wire			    rd4_mem = rdptr4_[LG_DEPTH-1];
   wire			    rd5_mem = rdptr5_[LG_DEPTH-1];

   
   
   always_ff@(posedge clk)
     begin
`ifdef VERILATOR
	if(reset)
	  begin
	     for(integer i = 1; i < 32; i=i+1)
	       begin
		  r_ram_alu[i] <= loadgpr(i);
		  r_ram_mem[i] <= loadgpr(i);		  
	       end
	  end
	else
	  begin
`endif
	     rd0 <= rdptr0=='d0 ? 'd0 : (rd0_mem ? r_ram_mem[rdptr0[LG_DEPTH-2:0]] : r_ram_alu[rdptr0[LG_DEPTH-2:0]]);	     
	     rd1 <= rdptr1=='d0 ? 'd0 : (rd1_mem ? r_ram_mem[rdptr1[LG_DEPTH-2:0]] : r_ram_alu[rdptr1[LG_DEPTH-2:0]]);
	     rd2 <= rdptr2=='d0 ? 'd0 : (rd2_mem ? r_ram_mem[rdptr2[LG_DEPTH-2:0]] : r_ram_alu[rdptr2[LG_DEPTH-2:0]]);
	     rd3 <= rdptr3=='d0 ? 'd0 : (rd3_mem ? r_ram_mem[rdptr3[LG_DEPTH-2:0]] : r_ram_alu[rdptr3[LG_DEPTH-2:0]]);
	     rd4 <= rdptr4_=='d0 ? 'd0 : (rd4_mem ? r_ram_mem[rdptr4_[LG_DEPTH-2:0]] : r_ram_alu[rdptr4_[LG_DEPTH-2:0]]);
	     rd5 <= rdptr5_=='d0 ? 'd0 : (rd5_mem ? r_ram_mem[rdptr5_[LG_DEPTH-2:0]] : r_ram_alu[rdptr5_[LG_DEPTH-2:0]]);
	     
	     if(wen0)
	       begin
		  if(wrptr0[LG_DEPTH-1] == 1'b1) $stop();		  
		  r_ram_alu[wrptr0[LG_DEPTH-2:0]] <= wr0;
	       end
	     
	     if(wen1)
	       begin
		  if(wrptr1[LG_DEPTH-1] == 1'b0) $stop();
		  r_ram_mem[wrptr1[LG_DEPTH-2:0]] <= wr1;
	       end
	     
	     if(wen2_)
	       begin
		  if(wrptr2[LG_DEPTH-1] == 1'b1) $stop();		  		  
		  r_ram_alu[wrptr2[LG_DEPTH-2:0]] <= wr2;
	       end
	  end
     end // always_ff@ (posedge clk)   

   
endmodule
