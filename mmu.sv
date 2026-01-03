`include "rob.vh"
//`define VERBOSE_MMU

`ifdef VERILATOR
import "DPI-C" function void mark_accessed_checker(input longint addr);
import "DPI-C" function void log_mmu_walk_lat(input longint cycle, input byte hit_lvl);
`endif

module mmu(clk, reset, clear_tlb, page_table_root, 
	   l1i_req, l1i_va, l1d_req, l1d_st, l1d_va,
	   mem_req_valid, mem_req_addr, mem_req_data,  mem_req_store,
	   mem_rsp_valid, 
	   mem_rsp_data,
	   mem_rsp_cacheline,
	   page_walk_rsp,
	   l1d_rsp_valid, 
	   l1i_rsp_valid,
	   l1i_gnt, 
	   l1d_gnt,
	   core_mark_dirty_valid,
	   core_mark_dirty_addr,
	   core_mark_dirty_rsp_valid,
	   mem_mark_valid,
	   mem_mark_accessed,
	   mem_mark_dirty,
	   mem_mark_addr,
	   mem_mark_rsp_valid,
	   mmu_state
	   );
   input logic clk;
   input logic reset;
   input logic clear_tlb;
   input logic [63:0] page_table_root;
   input logic l1i_req;
   input logic [63:0] l1i_va;
   input logic l1d_req;
   input logic l1d_st;
   input logic [63:0] l1d_va;   

   output logic	      mem_req_valid;
   output logic [`PA_WIDTH-1:0] mem_req_addr;
   output logic [63:0] mem_req_data;   
   output logic	       mem_req_store;

   
   output logic	       mem_mark_valid;
   output logic	       mem_mark_accessed;
   output logic	       mem_mark_dirty;
   output logic [63:0] mem_mark_addr;
   input logic	       mem_mark_rsp_valid;
      
   input logic	       mem_rsp_valid;
   input logic [63:0]  mem_rsp_data;
   input logic [127:0] mem_rsp_cacheline;

   output 	       page_walk_rsp_t page_walk_rsp;
   output logic	       l1d_rsp_valid;
   output logic	       l1i_rsp_valid;

   output logic        l1i_gnt;
   output logic        l1d_gnt;
   
   input logic	       core_mark_dirty_valid;
   input logic [63:0]  core_mark_dirty_addr;
   output logic	       core_mark_dirty_rsp_valid;
   output logic [3:0]  mmu_state;
   
      
   logic [63:0]	       n_addr, r_addr;
   logic [63:0]	       n_last_addr, r_last_addr;
   logic [63:0]	       n_va, r_va, r_pa, n_pa;
   logic	       r_req, n_req;
   logic	       n_page_fault, r_page_fault;
   logic	       n_l1d_rsp_valid, r_l1d_rsp_valid;
   logic	       n_l1i_rsp_valid, r_l1i_rsp_valid;
   logic	       r_do_l1i, n_do_l1i;
   logic	       r_do_l1d, n_do_l1d;
   logic	       r_do_dirty, n_do_dirty;
   
   logic [1:0]	       n_hit_lvl, r_hit_lvl;
   logic	       r_page_dirty, n_page_dirty;
   logic 	       r_page_read, n_page_read;
   logic 	       r_page_write, n_page_write;
   logic 	       r_page_user, n_page_user;
   logic	       r_page_gbl, n_page_gbl;
   
   
   logic	       n_page_executable, r_page_executable;

   logic	       r_mem_mark_valid, n_mem_mark_valid;
   logic	       r_mem_mark_accessed, n_mem_mark_accessed;
   logic	       r_mem_mark_dirty, n_mem_mark_dirty;
   
   logic	       r_core_mark_dirty_rsp_valid, n_core_mark_dirty_rsp_valid;

   
   assign mem_req_valid = r_req;
   assign mem_req_addr = r_addr[31:0];
   assign l1d_rsp_valid = r_l1d_rsp_valid;
   assign l1i_rsp_valid = r_l1i_rsp_valid;

   	     
   assign mem_mark_addr = r_last_addr;
   assign mem_mark_valid = r_mem_mark_valid;
   assign mem_mark_accessed = r_mem_mark_accessed;
   assign mem_mark_dirty = r_mem_mark_dirty;


   assign core_mark_dirty_rsp_valid = r_core_mark_dirty_rsp_valid;
   
   logic [127:0]       r_cl;
   wire		       w_make_64k = (r_hit_lvl == 2'd2) & r_addr[63];
   
   // always_ff@(negedge clk)
   //   begin
   // 	if(w_make_8k & r_state == WALK_DONE)
   // 	  begin
   // 	     $display("ppn0 = %x, ppn1 = %x", r_cl[53:10], r_cl[64+53:64+10]);
   // 	  end
   //   end
   
   always_comb
     begin
	page_walk_rsp.paddr = r_pa;
	page_walk_rsp.fault = r_page_fault;
	page_walk_rsp.dirty = r_page_dirty;
	page_walk_rsp.readable = r_page_read;
	page_walk_rsp.writable = r_page_write;
	page_walk_rsp.executable = r_page_executable;
	page_walk_rsp.user = r_page_user;
	page_walk_rsp.gbl = r_page_gbl;
	page_walk_rsp.pgsize = w_make_64k ? 2'd3 : r_hit_lvl;
     end

   assign mem_req_data = 'd0;
   
   typedef enum	logic [3:0] {
			     IDLE,
			     LOAD0,
			     WAIT0,
			     LOAD1,
			     WAIT1,			     
			     LOAD2,
			     WAIT2,
			     WALK_DONE,
			     MARK_ACCESS,
			     MARK_DIRTY
			     } state_t;
   
   state_t r_state, n_state;
   logic	n_l1i_req, r_l1i_req;
   logic	n_l1d_req, r_l1d_req;
   logic	n_dirty_req, r_dirty_req;
   logic 	n_gnt_l1i, r_gnt_l1i;
   logic 	n_gnt_l1d, r_gnt_l1d;

   assign mmu_state = r_state;
   assign l1i_gnt = r_gnt_l1i;
   assign l1d_gnt = r_gnt_l1d;
   
   wire		w_lo_va = (&r_va[63:39]) & (r_va[39] == r_va[38]);
   wire		w_hi_va = (&(~r_va[63:39])) & (r_va[39] == r_va[38]);
   wire		w_bad_va = (w_lo_va | w_hi_va) == 1'b0;

   logic [63:0]	r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 64'd0 : (r_cycle + 64'd1);
     end

`ifdef VERILATOR
   logic [63:0] r_walk_cycles;
   always_ff@(posedge clk)
     begin
	if((r_state == IDLE) && (n_state != IDLE))
	  begin
	     r_walk_cycles <= 'd0;
	  end
	else
	  begin
	     r_walk_cycles <= r_walk_cycles + 'd1;
	  end
	
	if((r_state != IDLE) && (n_state == IDLE))
	  begin
	     log_mmu_walk_lat(r_walk_cycles, {6'd0, r_hit_lvl});
	  end
     end
`endif
   
`ifdef VERBOSE_MMU   
   always_ff@(negedge clk)
     begin
	if(clear_tlb)
	  begin
	     $display("clear tlbs at cycle %d", r_cycle);
	  end
	
	if((n_state == LOAD0) && (r_state == IDLE))
	  begin
	     if(n_gnt_l1i)
	       $display("MMU translate iside %x at cycle %d", l1i_va[38:12], r_cycle);
	     else if(n_gnt_l1d)
	       $display("MMU translate dside %x at cycle %d", l1d_va[38:12], r_cycle);
	  end
     end
`endif	    

   always_ff@(posedge clk)
     begin
	r_cl <= mem_rsp_cacheline;
     end

  
   
   always_comb
     begin
	n_l1i_req = r_l1i_req | l1i_req;
	n_l1d_req = r_l1d_req | l1d_req;
	n_dirty_req = r_dirty_req | core_mark_dirty_valid;
	
	n_l1d_rsp_valid = 1'b0;
	n_l1i_rsp_valid = 1'b0;
	n_addr = r_addr;
	n_last_addr = r_last_addr;
	n_mem_mark_accessed = r_mem_mark_accessed;
	n_mem_mark_valid = 1'b0;
	n_mem_mark_dirty = r_mem_mark_dirty;
	
	n_req = 1'b0;
	n_va = r_va;
	n_pa = r_pa;
	n_state = r_state;
	n_page_fault = 1'b0;
	n_page_dirty = 1'b0;
	n_page_executable = 1'b0;
	n_page_write = 1'b0;
	n_page_read = 1'b0;
	n_page_user = 1'b0;
	n_page_gbl = 1'b0;
	
	n_do_l1i = r_do_l1i;
	n_do_l1d = r_do_l1d;
	n_do_dirty = r_do_dirty;
	n_hit_lvl = r_hit_lvl;
	n_gnt_l1i = 1'b0;
	n_gnt_l1d = 1'b0;

	n_core_mark_dirty_rsp_valid = 1'b0;

	
	
	case(r_state)
	  IDLE:
	    begin
	       if(n_l1i_req)
		 begin
		    n_state = LOAD0;
		    n_va = l1i_va;
		    n_l1i_req = 1'b0;
`ifdef VERBOSE_MMU
		    $display("starting translation for l1i %x", l1i_va);
`endif
		    n_do_l1i = 1'b1;
		    n_do_l1d = 1'b0;
		    n_do_dirty = 1'b0;
		    n_gnt_l1i = 1'b1;
		 end
	       else if(n_l1d_req)
		 begin
		    n_state = LOAD0;
		    n_va = l1d_va;
		    n_l1d_req = 1'b0;
`ifdef VERBOSE_MMU
		    $display("starting translation for l1d %x", l1d_va);
`endif
		    n_do_l1i = 1'b0;
		    n_do_l1d = 1'b1;
		    n_do_dirty = 1'b0;
		    n_gnt_l1d = 1'b1;
		 end // if (n_l1d_req)
	       else if(n_dirty_req)
		 begin
		    n_do_l1i = 1'b0;
		    n_do_l1d = 1'b0;
		    n_dirty_req = 1'b0;
		    n_do_dirty = 1'b1;
		    n_state = LOAD0;
		    n_va = core_mark_dirty_addr;
`ifdef VERBOSE_MMU
		    $display("starting dirty walk for %x", core_mark_dirty_addr);
`endif
		 end
	    end
	  LOAD0:
	    begin
	       n_addr = page_table_root + {52'd0, r_va[38:30], 3'd0};
//`ifdef VERBOSE_MMU
//	       $display("walker level 0 generates address %x", n_addr);	       
//`endif
	       
	       if(w_bad_va)
		 begin
		    n_state = IDLE;
		    n_page_fault = 1'b1;
		    n_l1i_rsp_valid = r_do_l1i;
		    n_l1d_rsp_valid = r_do_l1d;		    
		 end
	       else
		 begin
		    n_req = 1'b1;
		    n_state = WAIT0;
		 end
	    end
	  WAIT0:
	    begin
	       if(mem_rsp_valid)
		 begin
		    //$display("walker level 0 got %x, cycle %d", mem_rsp_data, r_cycle);
		    n_addr = mem_rsp_data;
		    n_last_addr = r_addr;

		    if(mem_rsp_data[0] == 1'b0)
		      begin
			 n_state = IDLE;
			 n_page_fault = 1'b1;
			 n_l1i_rsp_valid = r_do_l1i;
			 n_l1d_rsp_valid = r_do_l1d;
		      end
		    else if(|mem_rsp_data[3:1])
		      begin
			 n_hit_lvl = 2'd0;
			 n_state = WALK_DONE;
		      end
		    else
		      begin
			 n_state = LOAD1;
		      end
		 end // if (mem_rsp_valid)
	    end // case: WAIT0
	  LOAD1:
	    begin
	       n_addr = {8'd0, r_addr[53:10], 12'd0} + {52'd0, r_va[29:21], 3'd0};
//`ifdef VERBOSE_MMU
//	       $display("walker level 1 generates address %x", n_addr);
//`endif
	       n_req = 1'b1;
	       n_state = WAIT1;
	    end
	  WAIT1:
	    begin
	       if(mem_rsp_valid)
		 begin
		    n_addr = mem_rsp_data;
		    n_last_addr = r_addr;
		    //$display("walker level 1 got %x", mem_rsp_data);
		    if(mem_rsp_data[0] == 1'b0)
		      begin
			 n_state = IDLE;
			 n_page_fault = 1'b1;
			 n_l1i_rsp_valid = r_do_l1i;
			 n_l1d_rsp_valid = r_do_l1d;
		      end
		    else if(|mem_rsp_data[3:1])
		      begin
			 n_hit_lvl = 2'd1;			 
			 n_state = WALK_DONE;
		      end
		    else
		      begin
			 n_state = LOAD2;
		      end		    
		 end	       
	    end // case: WAIT1
	  LOAD2:
	    begin
	       n_addr = {8'd0, r_addr[53:10], 12'd0} + {52'd0, r_va[20:12], 3'd0};
//`ifdef VERBOSE_MMU
//	       $display("walker level 2 generates address %x", n_addr);
//`endif
	       n_req = 1'b1;
	       n_state = WAIT2;
	    end
	  WAIT2:
	    begin
	       if(mem_rsp_valid)
		 begin
		    //$display("walker level 2 got %x",  mem_rsp_data);
		    n_addr = mem_rsp_data;
		    n_last_addr = r_addr;

		    if(mem_rsp_data[0] == 1'b0)
		      begin
			 n_state = IDLE;
			 n_page_fault = 1'b1;
			 n_l1i_rsp_valid = r_do_l1i;
			 n_l1d_rsp_valid = r_do_l1d;
		      end
		    else
		      begin
			 n_hit_lvl = 2'd2;
			 n_state = WALK_DONE;
		      end		    
		 end	       
	    end
	  WALK_DONE:
	    begin
	       if(w_make_64k)
		 begin
		    n_pa = {8'd0, r_addr[53:14], 16'd0};
		 end
	       else if(r_hit_lvl == 2'd2)
		 begin /* 4k page */
		    n_pa = {8'd0, r_addr[53:10], 12'd0};		    
		 end
	       else if(r_hit_lvl == 2'd1)
		 begin /* 2mbyte page */
		    //18 17 16 15 14 13 12 11 10
		    n_pa = {8'd0, r_addr[53:19], r_va[20:12], 12'd0};
		    //n_pa = {8'd0, r_addr[53:19], 21'd0};		    
		 end
	       else if(r_hit_lvl == 2'd0)
		 begin /* 1gig page */
		    n_pa = {8'd0, r_addr[53:28], r_va[29:12], 12'd0};
		 end

	       
`ifdef VERBOSE_MMU
	       $display("va %x translated to pa %x, hit lvl %d, pte %x", r_va, n_pa, r_hit_lvl, r_addr);
`endif
	       /* can ack now, but need to check if accessed needs to be set */
	       n_l1i_rsp_valid = r_do_l1i;
	       n_l1d_rsp_valid = r_do_l1d;
	       n_page_dirty = r_addr[7];
	       
	       n_page_read = r_addr[1];
	       n_page_write = r_addr[2];
	       n_page_executable = r_addr[3];	       
	       n_page_user = r_addr[4];
	       n_page_gbl = r_addr[5];

	       //$display("va %x translated to pa %x, n bit %b, hit lvl %d, pte %x", 
	       //r_va, n_pa, r_addr[63], r_hit_lvl, r_addr);
	       //if(r_addr[63]) $stop();
	       
	       if(r_do_dirty)
		 begin
		    if(r_addr[7] == 1'b0)
		      begin
			 n_mem_mark_valid = 1'b1;
			 n_mem_mark_dirty = 1'b1;
			 n_state = MARK_ACCESS;
		      end
		    else
		      begin
			 n_core_mark_dirty_rsp_valid = 1'b1;
			 n_state = IDLE;
		      end
		 end
	       else if(r_addr[6] == 1'b0)
		 begin
`ifdef VERILATOR
		    mark_accessed_checker(r_last_addr);
`endif
		    n_mem_mark_valid = 1'b1;
		    n_mem_mark_accessed = 1'b1;
		    n_state = MARK_ACCESS;
		 end
	       else
		 begin
		    n_state = IDLE;
		 end
	    end // case: WALK_DONE
	  MARK_ACCESS:
	    begin
	       if(mem_mark_rsp_valid)
		 begin
		    n_state = IDLE;
		    n_core_mark_dirty_rsp_valid = r_do_dirty;
		    n_mem_mark_valid = 1'b0;
		    n_mem_mark_dirty = 1'b0;
		    n_mem_mark_accessed = 1'b0;		    
		 end
	    end
	  default:
	    begin
	    end
	endcase
     end // always_comb
   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= IDLE;
	     r_addr <= 'd0;
	     r_mem_mark_valid <= 1'b0;
	     r_mem_mark_accessed <= 1'b0;
	     r_mem_mark_dirty <= 1'b0;
	     r_last_addr <= 'd0;
	     r_req <= 1'b0;
	     r_va <= 'd0;
	     r_pa <= 'd0;
	     r_l1i_req <= 1'b0;
	     r_l1d_req <= 1'b0;
	     r_dirty_req <= 1'b0;
	     r_l1i_rsp_valid <= 1'b0;
	     r_l1d_rsp_valid <= 1'b0;
	     r_page_fault <= 1'b0;
	     r_page_dirty <= 1'b0;
	     r_page_executable <= 1'b0;
	     r_page_read <= 1'b0;
	     r_page_write <= 1'b0;
	     r_page_user <= 1'b0;
	     r_page_gbl <= 1'b0;
	     r_do_l1i <= 1'b0;
	     r_do_l1d <= 1'b0;
	     r_do_dirty <= 1'b0;
	     r_hit_lvl <= 2'd0;
	     r_gnt_l1i <= 1'b0;
	     r_gnt_l1d <= 1'b0;
	     r_core_mark_dirty_rsp_valid <= 1'b0;
	  end
	else
	  begin
	     r_state <= n_state;	     
	     r_addr <= n_addr;
	     r_mem_mark_valid <= n_mem_mark_valid;
	     r_mem_mark_accessed <= n_mem_mark_accessed;
	     r_mem_mark_dirty <= n_mem_mark_dirty;	     
	     r_last_addr <= n_last_addr;
	     r_req <= n_req;
	     r_va <= n_va;
	     r_pa <= n_pa;
	     r_l1i_req <= n_l1i_req;
	     r_l1d_req <= n_l1d_req;
	     r_dirty_req <= n_dirty_req;
	     r_l1i_rsp_valid <= n_l1i_rsp_valid;
	     r_l1d_rsp_valid <= n_l1d_rsp_valid;
	     r_page_fault <= n_page_fault;
	     r_page_dirty <= n_page_dirty;
	     r_page_executable <= n_page_executable;
	     r_page_read <= n_page_read;
	     r_page_write <= n_page_write;
	     r_page_user <= n_page_user;
	     r_page_gbl <= n_page_gbl;
	     
	     r_do_l1i <= n_do_l1i;
	     r_do_l1d <= n_do_l1d;
	     r_do_dirty <= n_do_dirty;
	     r_hit_lvl <= n_hit_lvl;
	     r_gnt_l1i <= n_gnt_l1i;
	     r_gnt_l1d <= n_gnt_l1d;
	     r_core_mark_dirty_rsp_valid <= n_core_mark_dirty_rsp_valid;
	  end
     end
endmodule // mmu

