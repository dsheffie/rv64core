`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

`ifdef VERILATOR
import "DPI-C" function void log_mem_begin(input int r, input int l, input longint c);
import "DPI-C" function void log_mem_end(input int r, input longint c);
`endif

//`define FPGA64_32

module
`ifndef FPGA64_32
   core_l1d_l1i
`else
  core_l1d_l1i_64
`endif
    (		   
		   clk, 
		   reset,
		   syscall_emu,
		   core_state,
		   l1i_state,
		   l1d_state,
		   l2_state,
		   mmu_state,
		   rob_ptr,
		   memq_empty,
		   putchar_fifo_out,
		   putchar_fifo_empty,
		   putchar_fifo_pop,
		   took_exc,
		   paging_active,
		   page_table_root,
		   extern_irq,
		   in_flush_mode,
		   resume,
		   resume_pc,
		   ready_for_resume,
		   mem_req_valid, 
		   mem_req_addr, 
		   mem_req_store_data,
		   mem_req_opcode,
		   mem_rsp_valid,
		   mem_rsp_load_data,
		   alloc_valid,
		   alloc_two_valid,
		   iq_one_valid,
		   iq_none_valid,
		   in_branch_recovery,
		   retire_reg_ptr,
		   retire_reg_data,
		   retire_reg_valid,
		   retire_reg_two_ptr,
		   retire_reg_two_data,
		   retire_reg_two_valid,
		   retire_valid,
		   retire_two_valid,
		   rob_empty,
		   retire_pc,
		   retire_two_pc,
		   branch_pc,
		   branch_pc_valid,		    
		   branch_fault,
		   l1i_cache_accesses,
		   l1i_cache_hits,
		   l1d_cache_accesses,
		   l1d_cache_hits,
		   l2_cache_accesses,
		   l2_cache_hits,
		   l1i_tlb_accesses,
		   l1i_tlb_hits,		   
		   l1d_tlb_accesses,
		   l1d_tlb_hits,		   		   
		   monitor_ack,
		   took_irq,
		   got_break,
		   got_ud,
		   got_bad_addr,
		   got_monitor,
		   inflight,
		   epc,
		   restart_ack,
		   priv);

   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);
   
   input logic clk;
   input logic reset;
   input logic syscall_emu;
   output logic [4:0] core_state;
   output logic [3:0] l1i_state;
   output logic [3:0] l1d_state;
   output logic [3:0] l2_state;
   output logic [3:0] mmu_state;
   output logic [`LG_ROB_ENTRIES-1:0] rob_ptr;
   
   output logic       memq_empty;
   output logic [7:0] putchar_fifo_out;
   output logic       putchar_fifo_empty;
   input logic 	      putchar_fifo_pop;
   
   output logic	took_exc;
   output logic	paging_active;
   output logic	[63:0] page_table_root;
   
   input logic extern_irq;
   input logic resume;
   input logic [(`M_WIDTH-1):0] resume_pc;
   output logic 		in_flush_mode;
   output logic 		ready_for_resume;
   
   output logic [(`M_WIDTH-1):0] branch_pc;
   
   output logic 		 branch_pc_valid;
   output logic 		 branch_fault;

   output logic [63:0] 			l1i_cache_accesses;
   output logic [63:0] 			l1i_cache_hits;
   output logic [63:0] 			l1d_cache_accesses;
   output logic [63:0] 			l1d_cache_hits;
   output logic [63:0] 			l2_cache_accesses;
   output logic [63:0] 			l2_cache_hits;   

   output logic [63:0] l1i_tlb_accesses;
   output logic [63:0] l1i_tlb_hits;
   output logic [63:0] l1d_tlb_accesses;
   output logic [63:0] l1d_tlb_hits;
   
   
   /* mem port */
   output logic 			mem_req_valid;
   output logic [`PA_WIDTH-1:0]		mem_req_addr;
   output logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0] mem_req_store_data;
   output logic [3:0] 				 mem_req_opcode;
   
   input logic 					 mem_rsp_valid;
   input logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0]  mem_rsp_load_data;
   
   output logic					 alloc_valid;
   output logic					 alloc_two_valid;
   
   output logic					 iq_one_valid;
   output logic					 iq_none_valid;
   output logic					 in_branch_recovery;
   output logic [4:0] 			  retire_reg_ptr;
   output logic [`M_WIDTH-1:0] 		  retire_reg_data;
   output logic 			  retire_reg_valid;
   output logic [4:0]			  retire_reg_two_ptr;
   output logic [`M_WIDTH-1:0] 		  retire_reg_two_data;
   output logic 			  retire_reg_two_valid;
   output logic 			  retire_valid;
   output logic 			  retire_two_valid;
   output logic				  rob_empty;   
   output logic [(`M_WIDTH-1):0] 	  retire_pc;
   output logic [(`M_WIDTH-1):0] 	  retire_two_pc;
   input logic 				  monitor_ack;
   output logic				  took_irq;
   output logic 			  got_break;
   output logic 			  got_ud;
   output logic 			  got_bad_addr;
   output logic 			  got_monitor;
   
   output logic [`LG_ROB_ENTRIES:0] 	  inflight;
   output logic [`M_WIDTH-1:0] 		  epc;
   output logic				  restart_ack;
   output logic [1:0]			  priv;
   
   logic [(`M_WIDTH-1):0] 	restart_pc;
   logic [(`M_WIDTH-1):0] 	restart_src_pc;
   logic 			restart_src_is_indirect;
   logic 			restart_valid;

   logic [`LG_PHT_SZ-1:0] 	branch_pht_idx;
   logic 			took_branch;

   logic [(`M_WIDTH-1):0] 	t_branch_pc, t_target_pc;
   logic 			t_branch_pc_valid;
   logic			t_branch_pc_is_indirect;
   logic 			t_branch_fault;
   
   assign branch_pc = t_branch_pc;
   assign branch_pc_valid = t_branch_pc_valid;   
   assign branch_fault = t_branch_fault;

   logic 				  retired_call;
   logic 				  retired_ret;

   logic 				  retired_rob_ptr_valid;
   logic 				  retired_rob_ptr_two_valid;
   logic [`LG_ROB_ENTRIES-1:0] 		  retired_rob_ptr;
   logic [`LG_ROB_ENTRIES-1:0] 		  retired_rob_ptr_two;
   

   logic				  head_of_rob_ptr_valid;   
   wire [`LG_ROB_ENTRIES-1:0]		  w_head_of_rob_ptr;
   
   assign rob_ptr = w_head_of_rob_ptr;


   wire 				  flush_req_l1i, flush_req_l1d;
   logic 				  flush_cl_req;
   logic [`M_WIDTH-1:0] 		  flush_cl_addr;
   wire 				  l1d_flush_complete;
   wire 				  l1i_flush_complete;

   
   mem_req_t core_mem_req;
   mem_rsp_t core_mem_rsp;
   mem_data_t core_store_data;
   
   logic 				  core_mem_req_valid;
   logic 				  core_mem_req_ack;
   logic 				  core_mem_rsp_valid;
   logic 				  core_store_data_valid;
   logic 				  core_store_data_ack;
   

   wire [63:0]				  w_mtimecmp;
   wire					  w_mtimecmp_val;
   
   
   typedef enum logic [2:0] {
			     FLUSH_IDLE = 'd0,
			     WAIT_FOR_L1D_L1I = 'd1,
			     GOT_L1D = 'd2,
			     GOT_L1I = 'd3,
			     FLUSH_L2 = 'd4
   } flush_state_t;
   flush_state_t n_flush_state, r_flush_state;
   logic 	r_flush, n_flush;
   logic 	r_flush_l2, n_flush_l2;
   wire 	w_l2_flush_complete;
   assign in_flush_mode = r_flush;


 
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_flush_state <= FLUSH_IDLE;
	     r_flush <= 1'b0;
	     r_flush_l2 <= 1'b0;
	  end
	else
	  begin
	     r_flush_state <= n_flush_state;
	     r_flush <= n_flush;
	     r_flush_l2 <= n_flush_l2;
	  end
     end // always_ff@ (posedge clk)

   always_comb
     begin
	n_flush_state = r_flush_state;
	n_flush = r_flush;
	n_flush_l2 = 1'b0;
	
	case(r_flush_state)
	  FLUSH_IDLE:
	    begin
	       if(flush_req_l1i && flush_req_l1d)
		 begin
		    n_flush_state = WAIT_FOR_L1D_L1I;
		    n_flush = 1'b1;
		 end
	       else if(flush_req_l1i && !flush_req_l1d)
		 begin
		    n_flush_state = GOT_L1D;
		    n_flush = 1'b1;
		 end
	       else if(!flush_req_l1i && flush_req_l1d)
		 begin
		    n_flush_state = GOT_L1I;
		    n_flush = 1'b1;
		 end
	    end
	  WAIT_FOR_L1D_L1I:
	    begin
	       if(l1d_flush_complete && !l1i_flush_complete)
		 begin
		    n_flush_state = GOT_L1D;		    
		 end
	       else if(!l1d_flush_complete && l1i_flush_complete)
		 begin
		    n_flush_state = GOT_L1I;
		 end
	       else if(l1d_flush_complete && l1i_flush_complete)
		 begin
		    $display("flush l2");
		    n_flush_state = FLUSH_L2;
		    n_flush_l2 = 1'b1;
		 end
	    end
	  GOT_L1D:
	    begin
	       if(l1i_flush_complete)
		 begin
		    $display("flush l2");
		    n_flush_state = FLUSH_L2;
		    n_flush_l2 = 1'b1;
		 end
	    end
	  GOT_L1I:
	    begin
	       if(l1d_flush_complete)
		 begin
		    $display("flush l2");
		    n_flush_state = FLUSH_L2;
		    n_flush_l2 = 1'b1;
		 end
	    end
	  FLUSH_L2:
	    begin
	       if(w_l2_flush_complete)
		 begin
		    $display("L2 FLUSH COMPLETE");
		    n_flush = 1'b0;
		    n_flush_state = FLUSH_IDLE;
		 end
	    end
	  default:
	    begin
	    end
	endcase // case (r_flush_state)
     end // always_comb
   

   wire 	l1d_mem_rsp_valid, l1i_mem_rsp_valid;
   
   
   logic 				  l1d_mem_req_ack;
   logic 				  l1d_mem_req_valid;
   logic				  l1d_mem_req_uc;

   l1d_req_t l1d_mem_req;

   wire [`LG_MRQ_ENTRIES:0] 		  w_l1d_mem_rsp_tag;
   wire					  w_l1d_mem_rsp_writeback;
   wire [`PA_WIDTH-1:0] 		  w_l1d_mem_rsp_addr;
   
   logic 				  l1i_mem_req_valid;
   logic [(`PA_WIDTH-1):0] 		  l1i_mem_req_addr;
   logic [3:0] 				  l1i_mem_req_opcode;


   logic 				  insn_valid,insn_valid2;
   logic 				  insn_ack, insn_ack2;
   insn_fetch_t insn, insn2;

   wire w_l1_mem_req_ack;
   
   wire [127:0] w_l1_mem_load_data;
   wire		w_mode64, w_paging_active;
   wire [1:0]	w_priv;
   wire [63:0]	w_page_table_root;
   assign priv = w_priv;
   
   wire				    w_mmu_req_valid;
   wire				    w_mmu_req_store;
   wire [`PA_WIDTH-1:0]		    w_mmu_req_addr;
   wire [63:0]			    w_mmu_req_data;
   wire [63:0]			    w_mmu_rsp_data;
   wire				    w_mmu_rsp_valid;
   wire 			    w_mmu_gnt_l1i, 
				    w_mmu_gnt_l1d;
   
   wire [63:0]			    w_l1d_page_walk_req_va;
   wire				    w_l1d_page_walk_req_valid;

   wire				    w_page_fault;
   wire	w_l1d_rsp_valid;
   wire	w_l1i_rsp_valid;

   wire	w_mem_mark_valid, w_mem_mark_accessed, w_mem_mark_dirty;
   wire [63:0] w_mem_mark_addr;
   wire	       w_mem_mark_rsp_valid;
   
   
   page_walk_rsp_t page_walk_rsp;
   
   wire	       w_restart_complete;
   wire	       w_l2_l1d_rdy;
   
   logic	drain_ds_complete;
   logic [(1<<`LG_ROB_ENTRIES)-1:0] dead_rob_mask;
   
   assign page_table_root = w_page_table_root;
   assign paging_active = w_paging_active;
   
   wire		w_clear_tlb;

   wire 	w_l2_probe_val, w_l2_probe_ack;
   wire [(`PA_WIDTH-1):0] w_l2_probe_addr;
   
   wire [63:0]		 w_l1i_tlb_accesses,
			 w_l1i_tlb_hits, 
			 w_l1d_tlb_accesses, 
			 w_l1d_tlb_hits;

   assign l1i_tlb_accesses = w_l1i_tlb_accesses;
   assign l1i_tlb_hits = w_l1i_tlb_hits;   
   assign l1d_tlb_accesses = w_l1d_tlb_accesses;
   assign l1d_tlb_hits = w_l1d_tlb_hits;   
   
   
   counters_t t_counters;
   always_comb
     begin
	t_counters.itlb_hits = w_l1i_tlb_hits;
	t_counters.itlb_accesses = w_l1i_tlb_accesses;
	t_counters.dtlb_hits = w_l1d_tlb_hits;
	t_counters.dtlb_accesses = w_l1d_tlb_accesses;
	t_counters.l1d_hits = l1d_cache_hits;
	t_counters.l1d_accesses = l1d_cache_accesses;
	t_counters.l1i_hits = l1i_cache_hits;
	t_counters.l1i_accesses = l1i_cache_accesses;
	t_counters.l2_hits = l2_cache_hits;
	t_counters.l2_accesses = l2_cache_accesses;
     end
   
   l2_2way l2cache (
	       .clk(clk),
	       .reset(reset),
	       .l2_state(l2_state),
	       .l2_probe_val(w_l2_probe_val),
	       .l2_probe_addr(w_l2_probe_addr),
	       .l2_probe_ack(w_l2_probe_ack),
	       .l1d_rdy(w_l2_l1d_rdy),
	       .l1d_req_valid(l1d_mem_req_valid),
	       .l1d_req(l1d_mem_req),
	       .l1i_req(l1i_mem_req_valid),
	       .l1i_addr(l1i_mem_req_addr),

	       .l1d_rsp_valid(l1d_mem_rsp_valid),
	       .l1d_rsp_tag(w_l1d_mem_rsp_tag),
	       .l1d_rsp_writeback(w_l1d_mem_rsp_writeback),	       		    
	       .l1d_rsp_addr(w_l1d_mem_rsp_addr),
	       .l1i_rsp_valid(l1i_mem_rsp_valid),

	       .l1i_flush_req(flush_req_l1i),
	       .l1d_flush_req(flush_req_l1d),
	       .l1i_flush_complete(l1i_flush_complete),
	       .l1d_flush_complete(l1d_flush_complete),
	       
	       .flush_complete(w_l2_flush_complete),
	       
	       .l1_mem_req_ack(w_l1_mem_req_ack),
	       .l1_mem_load_data(w_l1_mem_load_data),
	       
	       .mem_req_valid(mem_req_valid),
	       .mem_req_addr(mem_req_addr),
	       .mem_req_store_data(mem_req_store_data),
	       .mem_req_opcode(mem_req_opcode),

	       .mem_rsp_valid(mem_rsp_valid),
	       .mem_rsp_load_data(mem_rsp_load_data),
		    
	       .mmu_req_valid(w_mmu_req_valid),
	       .mmu_req_addr(w_mmu_req_addr),
	       .mmu_req_data(w_mmu_req_data),
	       .mmu_req_store(w_mmu_req_store),
	       .mmu_rsp_valid(w_mmu_rsp_valid),
	       .mmu_rsp_data(w_mmu_rsp_data),

	       .mem_mark_valid(w_mem_mark_valid),
	       .mem_mark_accessed(w_mem_mark_accessed),
	       .mem_mark_dirty(w_mem_mark_dirty),
	       .mem_mark_addr(w_mem_mark_addr),
	       .mem_mark_rsp_valid(w_mem_mark_rsp_valid),
	       
	       .cache_accesses(l2_cache_accesses),
	       .cache_hits(l2_cache_hits)

	       );
   
   

   perfect_l1d dcache (
	       .clk(clk),
	       .reset(reset),
	       .priv(w_priv),
	       .page_table_root(w_page_table_root),
	       .l2_probe_val(w_l2_probe_val),
	       .l2_probe_addr(w_l2_probe_addr),
	       .l2_probe_ack(w_l2_probe_ack),		 
	       .l1d_state(l1d_state),
	       .restart_complete(w_restart_complete),
	       .paging_active(w_paging_active),
	       .clear_tlb(w_clear_tlb),
	       .page_walk_req_va(w_l1d_page_walk_req_va),
	       .page_walk_req_valid(w_l1d_page_walk_req_valid),
	       .page_walk_rsp_gnt(w_mmu_gnt_l1d),
	       .page_walk_rsp_valid(w_l1d_rsp_valid),
	       .page_walk_rsp(page_walk_rsp),
		 
	       
	       .head_of_rob_ptr_valid(head_of_rob_ptr_valid),
	       .head_of_rob_ptr(w_head_of_rob_ptr),
	       .retired_rob_ptr_valid(retired_rob_ptr_valid),
	       .retired_rob_ptr_two_valid(retired_rob_ptr_two_valid),
	       .retired_rob_ptr(retired_rob_ptr),
	       .retired_rob_ptr_two(retired_rob_ptr_two),
	       .memq_empty(memq_empty),
	       .drain_ds_complete(drain_ds_complete),
	       .dead_rob_mask(dead_rob_mask),
	       .flush_req(flush_req_l1d),
	       .flush_cl_req(flush_cl_req),
	       .flush_cl_addr(flush_cl_addr),
	       .flush_complete(l1d_flush_complete),
	       .core_mem_va_req_valid(core_mem_req_valid),
	       .core_mem_va_req(core_mem_req),
	       .core_mem_va_req_ack(core_mem_req_ack),

	       .core_store_data_valid(core_store_data_valid),
	       .core_store_data(core_store_data),
	       .core_store_data_ack(core_store_data_ack),
	       
	       .core_mem_rsp_valid(core_mem_rsp_valid),
	       .core_mem_rsp(core_mem_rsp),
	       .mem_rdy(w_l2_l1d_rdy),
	       .mem_req_valid(l1d_mem_req_valid),
	       .mem_req(l1d_mem_req),
	       .l2_rsp_valid(l1d_mem_rsp_valid),
	       .l2_rsp_load_data(w_l1_mem_load_data),
	       .l2_rsp_tag(w_l1d_mem_rsp_tag),
	       .l2_rsp_addr(w_l1d_mem_rsp_addr),
	       .l2_rsp_writeback(w_l1d_mem_rsp_writeback),
		  
	        .mtimecmp(w_mtimecmp),
		.mtimecmp_val(w_mtimecmp_val),
		 
	       .cache_accesses(l1d_cache_accesses),
	       .cache_hits(l1d_cache_hits),
	       .tlb_hits(w_l1d_tlb_hits),
	       .tlb_accesses(w_l1d_tlb_accesses)
	       );

`ifdef VERILATOR
   logic [63:0] r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : r_cycle + 'd1;
     end
   always_ff@(negedge clk)
     begin
	if(core_mem_req_valid)
	  begin
	     log_mem_begin({27'd0,core_mem_req.rob_ptr}, {31'd0, core_mem_req.is_load}, r_cycle);
	  end
	if(core_mem_rsp_valid)
	  begin
	     log_mem_end({27'd0, core_mem_rsp.rob_ptr}, r_cycle);
	  end
     end
`endif
   
   wire [63:0]			    w_l1i_page_walk_req_va;
   wire				    w_l1i_page_walk_req_valid;
   
   logic			    r_l1i_page_walk_rsp_valid;
   logic			    r_l1i_page_walk_rsp_fault;
   logic [63:0]			    r_l1i_page_walk_rsp_pa;
   logic [63:0]			    t_l1i_pa;


   wire				    w_core_mark_dirty_valid;
   wire [63:0]			    w_core_mark_dirty_addr;
   wire				    w_core_mark_dirty_rsp_valid;
	    
      
   
   mmu mmu0(
	    .clk(clk),
	    .reset(reset),
	    .clear_tlb(w_clear_tlb),
	    .page_table_root(w_page_table_root), 
	    .l1i_req(w_l1i_page_walk_req_valid), 
	    .l1i_va(w_l1i_page_walk_req_va), 
	    .l1d_req(w_l1d_page_walk_req_valid),
	    .l1d_st(1'b0), 
	    .l1d_va(w_l1d_page_walk_req_va),
	    .mem_req_valid(w_mmu_req_valid), 
	    .mem_req_addr(w_mmu_req_addr),
	    .mem_req_data(w_mmu_req_data),
	    .mem_req_store(w_mmu_req_store),
	    .mem_rsp_valid(w_mmu_rsp_valid),
	    .mem_rsp_data(w_mmu_rsp_data),

	    .page_walk_rsp(page_walk_rsp),
	    
	    .l1d_rsp_valid(w_l1d_rsp_valid),
	    .l1i_rsp_valid(w_l1i_rsp_valid),
	    .l1i_gnt(w_mmu_gnt_l1i),
	    .l1d_gnt(w_mmu_gnt_l1d),

	    .core_mark_dirty_valid(w_core_mark_dirty_valid),
	    .core_mark_dirty_addr(w_core_mark_dirty_addr),
	    .core_mark_dirty_rsp_valid(w_core_mark_dirty_rsp_valid),
	    
	    .mem_mark_valid(w_mem_mark_valid),
	    .mem_mark_accessed(w_mem_mark_accessed),
	    .mem_mark_dirty(w_mem_mark_dirty),
	    .mem_mark_addr(w_mem_mark_addr),
	    .mem_mark_rsp_valid(w_mem_mark_rsp_valid),
	    .mmu_state(mmu_state)
	    );
   
   
   
   perfect_l1i icache(
	      .clk(clk),
	      .reset(reset),
	      .l1i_state(l1i_state),
	      .mode64(w_mode64),
	      .priv(w_priv),
	      .page_table_root(w_page_table_root),
	      .paging_active(w_paging_active),
	      .clear_tlb(w_clear_tlb),

	      .page_walk_req_va(w_l1i_page_walk_req_va),
	      .page_walk_req_valid(w_l1i_page_walk_req_valid),
	      .page_walk_rsp_valid(w_l1i_rsp_valid),
	      .page_walk_rsp(page_walk_rsp),

	      
	      .flush_req(flush_req_l1i),
	      .flush_complete(l1i_flush_complete),
	      .restart_pc(restart_pc),
	      .restart_src_pc(restart_src_pc),
	      .restart_src_is_indirect(restart_src_is_indirect),
	      .restart_valid(restart_valid),
	      .restart_ack(restart_ack),
	      .retire_reg_ptr(retire_reg_ptr),
	      .retire_reg_data(retire_reg_data),
	      .retire_reg_valid(retire_reg_valid),	      
	      .branch_pc_valid(t_branch_pc_valid),
              .branch_pc_is_indirect(t_branch_pc_is_indirect),
	      .branch_pc(t_branch_pc),
              .target_pc(t_target_pc),
	      .took_branch(took_branch),
	      .branch_fault(t_branch_fault),
	      .branch_pht_idx(branch_pht_idx),
	      .retire_valid(retire_valid),
	      .retired_call(retired_call),
	      .retired_ret(retired_ret),
	      .insn(insn),
	      .insn_valid(insn_valid),
	      .insn_ack(insn_ack),
	      .insn_two(insn2),
	      .insn_valid_two(insn_valid2),
	      .insn_ack_two(insn_ack2),
	      .mem_req_valid(l1i_mem_req_valid),
	      .mem_req_addr(l1i_mem_req_addr),
	      .mem_req_opcode(l1i_mem_req_opcode),
	      .mem_rsp_valid(l1i_mem_rsp_valid),
	      .mem_rsp_load_data(w_l1_mem_load_data),
	      .cache_accesses(l1i_cache_accesses),
	      .cache_hits(l1i_cache_hits),
	      .tlb_hits(w_l1i_tlb_hits),
	      .tlb_accesses(w_l1i_tlb_accesses)	      
	      );
   	      
   core cpu (
	     .clk(clk),
	     .reset(reset),
	     .putchar_fifo_out(putchar_fifo_out),
	     .putchar_fifo_empty(putchar_fifo_empty),
	     .putchar_fifo_pop(putchar_fifo_pop),
	     .restart_complete(w_restart_complete),
	     .syscall_emu(syscall_emu),
	     .core_state(core_state),
	     .took_exc(took_exc),
	     .priv(w_priv),
	     .clear_tlb(w_clear_tlb),
	     .paging_active(w_paging_active),
	     .page_table_root(w_page_table_root),
	     .mode64(w_mode64),
	     .resume(resume),
	     .memq_empty(memq_empty),
	     .drain_ds_complete(drain_ds_complete),
	     .dead_rob_mask(dead_rob_mask),	     
	     .head_of_rob_ptr_valid(head_of_rob_ptr_valid),	     
	     .head_of_rob_ptr(w_head_of_rob_ptr),
	     .resume_pc(resume_pc),
	     .ready_for_resume(ready_for_resume),  
	     .flush_req_l1d(flush_req_l1d),
	     .flush_req_l1i(flush_req_l1i),	     
	     .flush_cl_req(flush_cl_req),
	     .flush_cl_addr(flush_cl_addr),	     
	     .l1d_flush_complete(l1d_flush_complete),
	     .l1i_flush_complete(l1i_flush_complete),
	     .l2_flush_complete(w_l2_flush_complete),
	     .insn(insn),
	     .insn_valid(insn_valid),
	     .insn_ack(insn_ack),
	     .insn_two(insn2),
	     .insn_valid_two(insn_valid2),
	     .insn_ack_two(insn_ack2),
	     .branch_pc(t_branch_pc),
             .target_pc(t_target_pc),	     
	     .branch_pc_valid(t_branch_pc_valid),
             .branch_pc_is_indirect(t_branch_pc_is_indirect),	     
	     .branch_fault(t_branch_fault),
	     .took_branch(took_branch),
	     .branch_pht_idx(branch_pht_idx),
	     .restart_pc(restart_pc),
	     .restart_src_pc(restart_src_pc),
	     .restart_src_is_indirect(restart_src_is_indirect),
	     .restart_valid(restart_valid),
	     .restart_ack(restart_ack),
	     
	     .core_mem_req_ack(core_mem_req_ack),
	     .core_mem_req_valid(core_mem_req_valid),
	     .core_mem_req(core_mem_req),
	     
	     .core_store_data_valid(core_store_data_valid),
	     .core_store_data(core_store_data),
	     .core_store_data_ack(core_store_data_ack),
	     
	     .core_mem_rsp_valid(core_mem_rsp_valid),
	     .core_mem_rsp(core_mem_rsp),
	     .alloc_valid(alloc_valid),
	     .alloc_two_valid(alloc_two_valid),
	     .iq_one_valid(iq_one_valid),
	     .iq_none_valid(iq_none_valid),
	     .in_branch_recovery(in_branch_recovery),
	     .retire_reg_ptr(retire_reg_ptr),
	     .retire_reg_data(retire_reg_data),
	     .retire_reg_valid(retire_reg_valid),
	     .retire_reg_two_ptr(retire_reg_two_ptr),
	     .retire_reg_two_data(retire_reg_two_data),
	     .retire_reg_two_valid(retire_reg_two_valid),
	     .retire_valid(retire_valid),
	     .retire_two_valid(retire_two_valid),
	     .retire_pc(retire_pc),
	     .retire_two_pc(retire_two_pc),
	     .rob_empty(rob_empty),
	     .retired_call(retired_call),
	     .retired_ret(retired_ret),
	     .retired_rob_ptr_valid(retired_rob_ptr_valid),
	     .retired_rob_ptr_two_valid(retired_rob_ptr_two_valid),
	     .retired_rob_ptr(retired_rob_ptr),
	     .retired_rob_ptr_two(retired_rob_ptr_two),
	     .monitor_ack(monitor_ack),
	     .mtimecmp(w_mtimecmp),
	     .mtimecmp_val(w_mtimecmp_val),
	     .took_irq(took_irq),
	     .got_break(got_break),
	     .got_ud(got_ud),
	     .got_bad_addr(got_bad_addr),
	     .got_monitor(got_monitor),
	     .inflight(inflight),
	     .epc(epc),
	     .core_mark_dirty_valid(w_core_mark_dirty_valid),
	     .core_mark_dirty_addr(w_core_mark_dirty_addr),
	     .core_mark_dirty_rsp_valid(w_core_mark_dirty_rsp_valid),
	     .counters(t_counters)
	     );
   

endmodule // core_l1d_l1i


`ifdef FPGA64_32
module core_l1d_l1i(clk, 
		    reset,
		    syscall_emu,
		    core_state,
		    l1i_state,
		    l1d_state,
		    memq_empty,
		    putchar_fifo_out,
		    putchar_fifo_empty,
		    putchar_fifo_pop,
		    took_exc,
		    paging_active,
		    page_table_root,
		    extern_irq,
		    in_flush_mode,
		    resume,
		    resume_pc,
		    ready_for_resume,
		    
		    mem_req_valid, 
		    mem_req_addr, 
		    mem_req_store_data,
		    mem_req_opcode,
		    mem_rsp_valid,
		    mem_rsp_load_data,
		    alloc_valid,
		    alloc_two_valid,
		    iq_one_valid,
		    iq_none_valid,
		    in_branch_recovery,
		    retire_reg_ptr,
		    retire_reg_data,
		    retire_reg_valid,
		    retire_reg_two_ptr,
		    retire_reg_two_data,
		    retire_reg_two_valid,
		    retire_valid,
		    retire_two_valid,
		    rob_empty,
		    retire_pc,
		    retire_two_pc,
		    branch_pc,
		    branch_pc_valid,		    
		    branch_fault,
		    l1i_cache_accesses,
		    l1i_cache_hits,
		    l1d_cache_accesses,
		    l1d_cache_hits,
		    l2_cache_accesses,
		    l2_cache_hits,
		    monitor_ack,
		    took_irq,
		    got_break,
		    got_ud,
		    got_bad_addr,
		    got_monitor,
		    inflight,
		    epc,
		    restart_ack);

   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);
   
   input logic clk;
   input logic reset;
   input logic syscall_emu;
   output logic [4:0] core_state;
   output logic [3:0] l1i_state;
   output logic [3:0] l1d_state;
   output logic       memq_empty;
   output logic [7:0] putchar_fifo_out;
   output logic       putchar_fifo_empty;
   input logic 	      putchar_fifo_pop;
   
   output logic	took_exc;
   output logic	paging_active;
   output logic	[63:0] page_table_root;
   input logic extern_irq;
   input logic resume;
   input logic [31:0] resume_pc;
   output logic 		in_flush_mode;
   output logic 		ready_for_resume;
   
   output logic [31:0] branch_pc;
   output logic 		 branch_pc_valid;
   output logic 		 branch_fault;

   output logic [63:0] 			l1i_cache_accesses;
   output logic [63:0] 			l1i_cache_hits;
   output logic [63:0] 			l1d_cache_accesses;
   output logic [63:0] 			l1d_cache_hits;
   output logic [63:0] 			l2_cache_accesses;
   output logic [63:0] 			l2_cache_hits;   

   /* mem port */
   output logic 			mem_req_valid;
   output logic [31:0] 		mem_req_addr;
   output logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0] mem_req_store_data;
   output logic [3:0] 				 mem_req_opcode;
   
   input logic 					 mem_rsp_valid;
   input logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0]  mem_rsp_load_data;
   
   output logic					 alloc_valid;
   output logic					 alloc_two_valid;
   output logic					 iq_one_valid;
   output logic					 iq_none_valid;
   output logic					 in_branch_recovery;
   output logic [4:0]				 retire_reg_ptr;
   output logic [31:0]				 retire_reg_data;
   output logic					 retire_reg_valid;
   output logic [4:0]				 retire_reg_two_ptr;
   output logic [31:0]				 retire_reg_two_data;
   output logic					 retire_reg_two_valid;
   output logic					 retire_valid;
   output logic					 retire_two_valid;
   output logic					 rob_empty;   
   output logic [31:0]				 retire_pc;
   output logic [31:0]				 retire_two_pc;
   input logic					 monitor_ack;
   output logic					 took_irq;
   output logic					 got_break;
   output logic					 got_ud;
   output logic					 got_bad_addr;
   output logic					 got_monitor;
   
   output logic [`LG_ROB_ENTRIES:0]		 inflight;
   output logic [31:0]				 epc;
   output logic 				 restart_ack;
   
   wire [63:0]					 w_resume_pc64 = {32'd0, resume_pc};
   
   wire [63:0]					 w_mem_req_addr64;
   assign mem_req_addr = w_mem_req_addr64[31:0];
   
   wire [63:0]					 w_retire_reg_data64, w_retire_reg_two_data64;
   assign retire_reg_data = w_retire_reg_data64[31:0];
   assign retire_reg_two_data = w_retire_reg_two_data64[31:0];
   
   wire [63:0]					 w_retire_pc64, w_retire_two_pc64;
   assign retire_pc = w_retire_pc64[31:0];
   assign retire_two_pc = w_retire_two_pc64[31:0];
   
   
   wire [63:0]					 w_branch_pc64;
   assign branch_pc = w_branch_pc64[31:0];
   
   wire [63:0]					 w_epc64;
   assign epc = w_epc64[31:0];
   

   
   
   core_l1d_l1i_64 c(
		     .clk(clk),
                     .reset(reset),
		     .syscall_emu(syscall_emu),
		     .core_state(core_state),
		     .l1i_state(l1i_state),
		     .l1d_state(l1d_state),
		     .putchar_fifo_out(putchar_fifo_out),
		     .putchar_fifo_empty(putchar_fifo_empty),
		     .putchar_fifo_pop(putchar_fifo_pop),
		     .took_exc(took_exc),
		     .paging_active(paging_active),
		     .page_table_root(page_table_root),
                     .extern_irq(extern_irq),
                     .in_flush_mode(in_flush_mode),
                     .resume(resume),
                     .resume_pc(w_resume_pc64),
                     .ready_for_resume(ready_for_resume),
                     .mem_req_valid(mem_req_valid),
                     .mem_req_addr(w_mem_req_addr64),
                     .mem_req_store_data(mem_req_store_data),
                     .mem_req_opcode(mem_req_opcode),
                     .mem_rsp_valid(mem_rsp_valid),
                     .mem_rsp_load_data(mem_rsp_load_data),
                     .alloc_valid(alloc_valid),
                     .alloc_two_valid(alloc_two_valid),
                     .iq_one_valid(iq_one_valid),
                     .iq_none_valid(iq_none_valid),
                     .in_branch_recovery(in_branch_recovery),
                     .retire_reg_ptr(retire_reg_ptr),
                     .retire_reg_data(w_retire_reg_data64),
                     .retire_reg_valid(retire_reg_valid),
                     .retire_reg_two_ptr(retire_reg_two_ptr),
                     .retire_reg_two_data(w_retire_reg_two_data64),
                     .retire_reg_two_valid(retire_reg_two_valid),
                     .retire_valid(retire_valid),
                     .retire_two_valid(retire_two_valid),
		     .rob_empty(rob_empty),
                     .retire_pc(w_retire_pc64),
                     .retire_two_pc(w_retire_two_pc64),
                     .branch_pc(w_branch_pc64),
                     .branch_pc_valid(branch_pc_valid),		     
                     .branch_fault(branch_fault),
                     .l1i_cache_accesses(l1i_cache_accesses),
                     .l1i_cache_hits(l1i_cache_hits),
                     .l1d_cache_accesses(l1d_cache_accesses),
                     .l1d_cache_hits(l1d_cache_hits),
                     .l2_cache_accesses(l2_cache_accesses),
                     .l2_cache_hits(l2_cache_hits),
		     .l1i_tlb_accesses(),
		     .l1i_tlb_hits(),
		     .l1d_tlb_accesses(),
		     .l1d_tlb_hits(),		     		     
                     .monitor_ack(monitor_ack),
		     .took_irq(took_irq),
                     .got_break(got_break),
                     .got_ud(got_ud),
                     .got_bad_addr(got_bad_addr),
                     .got_monitor(got_monitor),
                     .inflight(inflight),
                     .epc(w_epc64),
		     .restart_ack(restart_ack)
		     );

   
endmodule
`endif
