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
   
   
   logic [27:0]	       r_va_tags[N-1:0];
   logic [51:0]	       r_pa_data[N-1:0];
   
   wire [N-1:0]	       w_hits;
   wire [LG_N:0]	       w_idx;   
   generate
      for(genvar i = 0; i < N; i=i+1)
	begin : hits
	   assign w_hits[i] = r_valid[i] ? (r_va_tags[i] == va[39:12]) : 1'b0;
	end
   endgenerate

   logic [7:0] r_lfsr, n_lfsr;
   always_ff@(posedge clk)
     begin
	r_lfsr <= reset ? 8'd1 : n_lfsr;
     end
   always_comb
     begin
	n_lfsr = r_lfsr;
	if(replace)
	  begin
	     n_lfsr = {r_lfsr[6:0], r_lfsr[7] ^ r_lfsr[5] ^ r_lfsr[4] ^ r_lfsr[3]};
	  end
     end
   
   
   find_first_set#(.LG_N(LG_N)) 
   ffs(.in(w_hits),
       .y(w_idx));

   
   always_ff@(posedge clk)
     begin
	hit <= reset ? 1'b0 : (active ? (req & |w_hits) : 1'b1);
	writable <= r_writable[w_idx[LG_N-1:0]];
	readable <= r_readable[w_idx[LG_N-1:0]];
	dirty <= r_dirty[w_idx[LG_N-1:0]];
	pa <= active ? {r_pa_data[w_idx[LG_N-1:0]], va[11:0]} : va;
     end

   logic [63:0] r_cycle;
   always@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end
   
   always@(negedge clk)
     begin
	if(priv == 2'd0 && (ISIDE == 1'b0))
	  begin
	     if(active & req & ((|w_hits) == 1'b0))
	       begin
		  $display("tlb missed for addr %x at cycle %d", 
			   {va[39:12], 12'd0}, r_cycle);
	       end
	     if(replace)
	       begin
		  $display("replace entry %d with %x",
			   r_lfsr[LG_N:1], 
			   {replace_va[39:12],12'd0});
	       end
	  end
     end

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
	     r_pa_data[r_lfsr[LG_N:1]] <= page_walk_rsp.paddr[63:12];
	  end
     end // always_ff@ (posedge clk)
   

endmodule
   
   
