`include "rob.vh"

module tlb(clk,
	   reset,
	   priv,
	   clear,
	   active,
	   req,
	   va,
	   pa,
	   hit,
	   dirty,
	   readable,
	   writable,
	   user,
	   replace_va,
	   replace,
	   page_walk_rsp);
   
   input logic clk;
   input logic reset;
   input logic [1:0] priv;
   input logic clear;
   input logic active;
   input logic req;
   
   input logic [63:0] va;
   output logic [63:0] pa;
   output logic	       hit;
   output logic	       dirty;
   output logic	       readable;
   output logic	       writable;
   output logic        user;
   input logic [63:0]  replace_va;
   input logic	       replace;
   input 	       page_walk_rsp_t page_walk_rsp;
   
   /* bits 39 down to 12 */

   parameter	       LG_N = 2;
   parameter	       ISIDE = 0;
   
   localparam	       N = 1<<LG_N;

   logic [N-1:0]       r_valid, r_dirty, r_readable, r_writable, r_executable, r_user;
   
   logic [1:0]	       r_pgsize[N-1:0];
   logic [27:0]	       r_va_tags[N-1:0];
   logic [51:0]	       r_pa_data[N-1:0];
   
   wire [N-1:0]	       w_hits4k, w_hits2m, w_hits1g;
   wire [N-1:0]	       w_hits;
   
   wire [LG_N:0]	       w_idx;   
   generate
      for(genvar i = 0; i < N; i=i+1)
	begin : hits
	   assign w_hits4k[i] = r_valid[i] ? (r_pgsize[i] == 2'd2) & (r_va_tags[i] == va[39:12]) : 1'b0;
	   assign w_hits2m[i] = r_valid[i] ? (r_pgsize[i] == 2'd1) & (r_va_tags[i][27:9] == va[39:21]) : 1'b0;
	   assign w_hits1g[i] = r_valid[i] ? (r_pgsize[i] == 2'd0) & (r_va_tags[i][27:18] == va[39:30]) : 1'b0;
	   assign w_hits[i] = w_hits1g[i] | w_hits2m[i] | w_hits4k[i];
	end
   endgenerate


   logic [15:0] r_lfsr, n_lfsr;
   always_ff@(posedge clk)
     begin
	r_lfsr <= reset ? 'd1 : n_lfsr;
     end
   always_comb
     begin
	n_lfsr = {r_lfsr[14:0], r_lfsr[15] ^ r_lfsr[13] ^ r_lfsr[12] ^ r_lfsr[10]};
     end

   wire [63:0] w_pa_sel = 
	       (r_pgsize[w_idx[LG_N-1:0]] == 2'd0) ? {r_pa_data[w_idx[LG_N-1:0]][51:18], va[29:0]} :
	       (r_pgsize[w_idx[LG_N-1:0]] == 2'd1) ? {r_pa_data[w_idx[LG_N-1:0]][51:9], va[20:0]} :
	       {r_pa_data[w_idx[LG_N-1:0]], va[11:0]};
	       	          
   find_first_set#(.LG_N(LG_N)) 
   ffs(.in(w_hits),
       .y(w_idx));

   
   always_ff@(posedge clk)
     begin
	hit <= reset ? 1'b0 : (active ? (req & |w_hits) : 1'b1);
	writable <= r_writable[w_idx[LG_N-1:0]];
	readable <= r_readable[w_idx[LG_N-1:0]];
	dirty <= r_dirty[w_idx[LG_N-1:0]];
	pa <= active ? w_pa_sel : va;
     end

   logic [63:0] r_cycle;
   always@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end
   
   //always@(negedge clk)
     //begin
   //if(active & req & ((|w_hits) == 1'b1) && (r_pgsize[w_idx[LG_N-1:0]] == 2'd0))
   //begin
   //$display("tlb hit for addr %x at cycle %d, translated to %x", 
   //va, r_cycle, w_pa_sel);
   //end
   //end

   always_ff@(posedge clk)
     begin   
	if(reset || clear)
	  begin
	     r_valid <= 'd0;
	  end
	else if(replace)
	  begin
	     r_valid[r_lfsr[LG_N:1]] <= 1'b1;
	  end
     end // always_ff@ (posedge clk)
   
   always_ff@(posedge clk)
     begin
	if(replace)
	  begin
	     r_dirty[r_lfsr[LG_N:1]] <= page_walk_rsp.dirty;
	     r_readable[r_lfsr[LG_N:1]] <= page_walk_rsp.readable;
	     r_writable[r_lfsr[LG_N:1]] <= page_walk_rsp.writable;
	     r_executable[r_lfsr[LG_N:1]] <= page_walk_rsp.executable;
	     r_user[r_lfsr[LG_N:1]] <= page_walk_rsp.user;
	     r_va_tags[r_lfsr[LG_N:1]] <= replace_va[39:12];
	     r_pgsize[r_lfsr[LG_N:1]] <= page_walk_rsp.pgsize;
	     r_pa_data[r_lfsr[LG_N:1]] <= page_walk_rsp.paddr[63:12];
	  end
     end // always_ff@ (posedge clk)
   

endmodule
   
   
