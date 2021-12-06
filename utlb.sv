`include "rob.vh"
  
module utlb(clk,
	    reset,
	    flush,
	    req,
	    addr,
	    tlb_rsp,
	    tlb_rsp_valid,
	    hit_entry,
	    hit
	    );
   input logic clk;
   input logic reset;
   input logic flush;
   input logic req;
   input       utlb_entry_t tlb_rsp;
   input       logic tlb_rsp_valid;
   
   input logic [`M_WIDTH-1:0] addr;
   output logic 	      hit;
   output 		      utlb_entry_t hit_entry;
   
   parameter ISIDE = 0;
   
   localparam N_TLB_ENTRIES = 1 << `LG_UTLB_ENTRIES;
   utlb_entry_t entries[N_TLB_ENTRIES-1:0];
   
   logic [N_TLB_ENTRIES-1:0]  t_hit_vec;
   logic 		      r_hit;
   
   logic [`LG_UTLB_ENTRIES:0] t_hit_pos,t_repl_pos;
   logic [`LG_UTLB_ENTRIES-1:0] r_repl;
   
   logic [N_TLB_ENTRIES-1:0] 	r_p_mru, n_p_mru;

   logic [`M_WIDTH-1:0] 	r_addr;
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_repl <= 'd0;
	     r_addr <= addr;
	  end
	else
	  begin
	     r_repl <= (req & (t_hit_vec == 'd0)) ? r_repl + 'd1 : r_repl;
	     r_addr <= req ? addr : r_addr;
	  end
     end
   
   generate
      for(genvar i = 0; i < N_TLB_ENTRIES; i=i+1)
	begin
	   assign t_hit_vec[i] = entries[i].valid ? (entries[i].paddr == addr[`M_WIDTH-1:`LG_PG_SZ]) : 1'b0;
	end
   endgenerate

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_p_mru <= 'd0;
	  end
	else
	  begin
	     r_p_mru <= ((&n_p_mru) == 1'b1) ? 'd0 : n_p_mru;
	  end
     end // always_ff@ (posedge clk)

   
   always_comb
     begin
	n_p_mru = r_p_mru;
	if(req)
	  begin
	     n_p_mru |= t_hit_vec;
	  end
     end
   
   find_first_set#(`LG_UTLB_ENTRIES) hit0 (.in(t_hit_vec), .y(t_hit_pos));
   find_first_set#(`LG_UTLB_ENTRIES) repl0 (.in(~r_p_mru), .y(t_repl_pos));   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_hit <= 1'b0;
	     for(integer i = 0; i < N_TLB_ENTRIES; i=i+1)
	       begin
		  entries[i].paddr = 'd0;
		  entries[i].valid = 1'b0;
	       end
	  end
	else
	  begin
	     r_hit <= req & (t_hit_vec != 'd0);
	     if(flush)
	       begin
		  for(integer i = 0; i < N_TLB_ENTRIES; i=i+1)
		    begin
		       entries[i].valid = 1'b0;
		    end		  
	       end
	     else if(/*req & (t_hit_vec == 'd0)*/tlb_rsp_valid)
	       begin
		  entries[t_repl_pos[`LG_UTLB_ENTRIES-1:0]] <= tlb_rsp;
		  //entries[t_repl_pos[`LG_UTLB_ENTRIES-1:0]].valid <= 1'b1;
		  //entries[t_repl_pos[`LG_UTLB_ENTRIES-1:0]].paddr <= addr[`M_WIDTH-1:`LG_PG_SZ];
	       end
	  end // else: !if(reset)
     end // always_ff@ (posedge clk)


   logic [31:0] r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : (r_cycle + 'd1);
     end
       

   always_ff@(posedge clk)
     begin
	hit_entry <= entries[t_hit_pos[`LG_UTLB_ENTRIES-1:0]];
     end

   always_comb
     begin
	hit = r_hit;
     end
   
endmodule // utlb

