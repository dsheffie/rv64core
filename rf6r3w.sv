`include "machine.vh"

`ifdef VERILATOR
import "DPI-C" function longint loadgpr(input int regid);
`endif

module rf6r3w(clk, reset,
	      rdptr0,rdptr1,rdptr2,rdptr3,
	      wrptr0,wrptr1,wrptr2,
	      wen0,wen1,
	      wr0, wr1,
	      rd0, rd1, rd2, rd3);
   
   parameter WIDTH = 1;
   parameter LG_DEPTH = 1;
   input logic clk;
   input logic reset;
   
   input logic [LG_DEPTH-1:0] rdptr0;
   input logic [LG_DEPTH-1:0] rdptr1;
   input logic [LG_DEPTH-1:0] rdptr2;
   input logic [LG_DEPTH-1:0] rdptr3;

   
   input logic [LG_DEPTH-1:0] wrptr0;
   input logic [LG_DEPTH-1:0] wrptr1;
   input logic [LG_DEPTH-1:0] wrptr2;
   
   input logic 		      wen0;
   input logic 		      wen1;
   input logic [WIDTH-1:0]    wr0;
   input logic [WIDTH-1:0]    wr1;

   
   output logic [WIDTH-1:0]   rd0;
   output logic [WIDTH-1:0]   rd1;
   output logic [WIDTH-1:0]   rd2;
   output logic [WIDTH-1:0]   rd3;
   
   localparam DEPTH = 1<<LG_DEPTH;
   localparam H_DEPTH = 1<<(LG_DEPTH-1);   
   
   logic [WIDTH-1:0] 	    r_ram_alu[H_DEPTH-1:0];
   logic [WIDTH-1:0]	    r_ram_mem[H_DEPTH-1:0];   
   
   wire			    rd0_mem = rdptr0[LG_DEPTH-1];
   wire			    rd1_mem = rdptr1[LG_DEPTH-1];
   wire			    rd2_mem = rdptr2[LG_DEPTH-1];
   wire			    rd3_mem = rdptr3[LG_DEPTH-1];


   logic [WIDTH-1:0]	    r_rd_mem0,r_rd_mem1,r_rd_mem2,r_rd_mem3,r_rd_mem4,r_rd_mem5;
   logic [WIDTH-1:0]	    r_rd_alu0,r_rd_alu1,r_rd_alu2,r_rd_alu3,r_rd_alu4,r_rd_alu5;   

   logic		    r_rdptr0_z, r_rdptr1_z, r_rdptr2_z, r_rdptr3_z, r_rdptr4_z, r_rdptr5_z;
   logic		    r_rdptr0_m, r_rdptr1_m, r_rdptr2_m, r_rdptr3_m, r_rdptr4_m, r_rdptr5_m;


   always_ff@(posedge clk)
     begin
	r_rdptr0_z <= rdptr0 == 'd0;
	r_rdptr1_z <= rdptr1 == 'd0;
	r_rdptr2_z <= rdptr2 == 'd0;
	r_rdptr3_z <= rdptr3 == 'd0;
	r_rdptr0_m <= rd0_mem;
	r_rdptr1_m <= rd1_mem;
	r_rdptr2_m <= rd2_mem;
	r_rdptr3_m <= rd3_mem;	
     end // always_ff@ (posedge clk)

   always_comb
     begin
	rd0 = r_rdptr0_z ? 'd0 : (r_rdptr0_m ? r_rd_mem0 : r_rd_alu0);
	rd1 = r_rdptr1_z ? 'd0 : (r_rdptr1_m ? r_rd_mem1 : r_rd_alu1);
	rd2 = r_rdptr2_z ? 'd0 : (r_rdptr2_m ? r_rd_mem2 : r_rd_alu2);
	rd3 = r_rdptr3_z ? 'd0 : (r_rdptr3_m ? r_rd_mem3 : r_rd_alu3);
     end
     
   
   always_ff@(posedge clk)
     begin
`ifdef VERILATOR
	if(reset)
	  begin
	     for(integer i = 1; i < 32; i=i+1)
	       begin
		  r_ram_alu[i] <= loadgpr(i);
	       end
	  end
	else
	  begin
`endif //  `ifdef VERILATOR

	     r_rd_alu0 <= r_ram_alu[rdptr0[LG_DEPTH-2:0]];
	     r_rd_alu1 <= r_ram_alu[rdptr1[LG_DEPTH-2:0]];
	     r_rd_alu2 <= r_ram_alu[rdptr2[LG_DEPTH-2:0]];
	     r_rd_alu3 <= r_ram_alu[rdptr3[LG_DEPTH-2:0]];
	     if(wen0)
	       begin
		  r_ram_alu[wrptr0[LG_DEPTH-2:0]] <= wr0;
	       end
	     	     
`ifdef VERILATOR
	  end // else: !if(reset)
`endif
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
`ifdef VERILATOR
	if(reset)
	  begin
	     for(integer i = 1; i < 32; i=i+1)
	       begin
		  r_ram_mem[i] <= loadgpr(i);		  
	       end
	  end
	else
	  begin
`endif //  `ifdef VERILATOR
	     r_rd_mem0 <= r_ram_mem[rdptr0[LG_DEPTH-2:0]];
	     r_rd_mem1 <= r_ram_mem[rdptr1[LG_DEPTH-2:0]];
	     r_rd_mem2 <= r_ram_mem[rdptr2[LG_DEPTH-2:0]];
	     r_rd_mem3 <= r_ram_mem[rdptr3[LG_DEPTH-2:0]];
	     	     
	     if(wen1)
	       begin
		  r_ram_mem[wrptr1[LG_DEPTH-2:0]] <= wr1;
	       end
	     
`ifdef VERILATOR
	  end // else: !if(reset)
`endif
     end // always_ff@ (posedge clk)      


   
   
endmodule
