`include "rob.vh"
`include "machine.vh"

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
	   executable,
	   user,
	   zero_page,
	   tlb_hits,
	   tlb_accesses,
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
   output logic [`PA_WIDTH-1:0] pa;
   
   output logic	       hit;
   output logic	       dirty;
   output logic	       readable;
   output logic	       writable;
   output logic	       executable;
   output logic        user;

   output logic	       zero_page;
      
   output logic [63:0] tlb_hits;
   output logic [63:0] tlb_accesses;
   
   input logic [63:0]  replace_va;
   input logic	       replace;
   input 	       page_walk_rsp_t page_walk_rsp;
   
   /* bits 39 down to 12 */

   parameter	       N = 2;
   parameter	       ISIDE = 0;
   
   localparam	       LG_N = $clog2(N);
   localparam	       NN = 1 << LG_N;
   
   logic [NN-1:0]       r_valid, r_dirty, r_readable, r_writable, r_executable, r_user;
   
   logic [1:0]	       r_pgsize[NN-1:0];
   logic [27:0]	       r_va_tags[NN-1:0];
   logic [51:0]	       r_pa_data[NN-1:0];

   
   wire [NN-1:0]	       w_hits4k, w_hits64k, w_hits2m, w_hits1g;
   wire [NN-1:0]	       w_hits;


   
   
   wire [LG_N:0]	       w_idx;
   generate
      for(genvar i = N; i < NN; i=i+1)
	begin
	   assign w_hits64k[i] =  1'b0;	   
	   assign w_hits4k[i] = 1'b0;
	   assign w_hits2m[i] = 1'b0;
	   assign w_hits1g[i] = 1'b0;
	   assign w_hits[i] = 1'b0;
	end
   endgenerate
   
   
   generate
      for(genvar i = 0; i < N; i=i+1)
	begin : hits
	   assign w_hits64k[i] = r_valid[i] ? (r_pgsize[i] == 2'd3) & (r_va_tags[i][27:4] == va[39:16]) : 1'b0;	   
	   assign w_hits4k[i] = r_valid[i] ? (r_pgsize[i] == 2'd2) & (r_va_tags[i] == va[39:12]) : 1'b0;
	   assign w_hits2m[i] = r_valid[i] ? (r_pgsize[i] == 2'd1) & (r_va_tags[i][27:9] == va[39:21]) : 1'b0;
	   assign w_hits1g[i] = r_valid[i] ? (r_pgsize[i] == 2'd0) & (r_va_tags[i][27:18] == va[39:30]) : 1'b0;
	   assign w_hits[i] = w_hits1g[i] | w_hits2m[i] | w_hits4k[i] | w_hits64k[i];
	end
   endgenerate

   
   wire [63:0] w_pa_sel = 
	       (r_pgsize[w_idx[LG_N-1:0]] == 2'd0) ? {r_pa_data[w_idx[LG_N-1:0]][51:18], va[29:0]} :
	       (r_pgsize[w_idx[LG_N-1:0]] == 2'd1) ? {r_pa_data[w_idx[LG_N-1:0]][51:9], va[20:0]} :
	       (r_pgsize[w_idx[LG_N-1:0]] == 2'd2) ? {r_pa_data[w_idx[LG_N-1:0]], va[11:0]} :
	       {r_pa_data[w_idx[LG_N-1:0]][51:4], va[15:0]};
	       
	       	          
   find_first_set#(.LG_N(LG_N)) 
   ffs(.in(w_hits),
       .y(w_idx));

   always_comb
     begin
	tlb_hits = 'd0;
	tlb_accesses = 'd0;
     end
   // always_ff@(posedge clk)
   //   begin
   // 	if(reset)
   // 	  begin
   // 	     tlb_hits <= 'd0;
   // 	     tlb_accesses <= 'd0;
   // 	  end
   // 	else
   // 	  begin
   // 	     tlb_hits <= (active & req & |w_hits) ? 
   // 			 tlb_hits + 'd1 : 
   // 			 tlb_hits;
   // 	     tlb_accesses <= (active & req) ?
   // 			     tlb_accesses + 'd1 : 
   // 			     tlb_accesses;
   // 	  end
   //   end
   
   always_ff@(posedge clk)
     begin
	hit <= reset ? 1'b0 : (active ? (req & |w_hits) : 1'b1);
	writable <= r_writable[w_idx[LG_N-1:0]];
	readable <= r_readable[w_idx[LG_N-1:0]];
	executable <= r_executable[w_idx[LG_N-1:0]];
	dirty <= r_dirty[w_idx[LG_N-1:0]];
	user <= r_user[w_idx[LG_N-1:0]];
	pa <= active ? w_pa_sel[`PA_WIDTH-1:0] : va[`PA_WIDTH-1:0];
	zero_page <= reset ? 1'b0 : ((|va[39:12]) == 1'b0);
     end


   logic [NN-1:0] r_accessed;
   logic [LG_N-1:0] r_replace_idx, n_replace_idx;
   always@(posedge clk)
     begin
	r_replace_idx <= reset ? 'd0 : n_replace_idx;
     end

   always_comb
     begin
	n_replace_idx = replace ? (r_replace_idx + 'd1) : r_replace_idx;
     end
   
   wire [NN-1:0] w_rot_accessed = r_accessed << (r_replace_idx) | (r_accessed >> (NN-r_replace_idx));
   
   always@(posedge clk)
     begin
	if(reset | (&r_accessed == 1'b1))
	  begin
	     r_accessed <= 'd0;
	  end
	else if(active & req & |w_hits)
	  begin
	     r_accessed[w_idx[LG_N-1:0]] <= 1'b1;
	  end
     end // always@ (posedge clk)

   wire [LG_N:0]   w_rep_idx;
   find_first_set #(.LG_N(LG_N)) f0(.in(~w_rot_accessed), .y(w_rep_idx));
   wire [LG_N-1:0] w_replace = w_rep_idx[LG_N-1:0];
   
   logic [63:0] r_cycle;
   always@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end

     
   always_ff@(posedge clk)
     begin   
	if(reset || clear)
	  begin
	     r_valid <= 'd0;
	  end
	else if(replace)
	  begin
	     r_valid[w_replace] <= 1'b1;
	     //$display("cycle %d, ISIDE %d : replacing entry %d, accessed %b, w_rot_accessed %b, r_replace_idx %d, w_rep_idx = %d", 
	     //r_cycle, ISIDE, w_replace, r_accessed, w_rot_accessed, r_replace_idx, w_rep_idx);
	  end
     end // always_ff@ (posedge clk)


   
   
   always_ff@(posedge clk)
     begin
	if(replace)
	  begin
	     r_dirty[w_replace] <= page_walk_rsp.dirty;
	     r_readable[w_replace] <= page_walk_rsp.readable;
	     r_writable[w_replace] <= page_walk_rsp.writable;
	     r_executable[w_replace] <= page_walk_rsp.executable;
	     r_user[w_replace] <= page_walk_rsp.user;
	     r_va_tags[w_replace] <= replace_va[39:12];
	     r_pgsize[w_replace] <= page_walk_rsp.pgsize;
	     r_pa_data[w_replace] <= page_walk_rsp.paddr[63:12];
	  end
     end // always_ff@ (posedge clk)
   

endmodule
   
   
