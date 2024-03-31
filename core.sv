`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

`ifdef VERILATOR
import "DPI-C" function void record_faults(input int n_faults);
import "DPI-C" function void record_branches(input int n_branches);


import "DPI-C" function void record_alloc(input int rob_full,
					  input int alloc_one, 
					  input int alloc_two,
					  input int dq_empty,
					  input int uq_full,
					  input int uq_next_full,
					  input int one_insn_avail,
					  input int two_insn_avail,
					  input int active);

import "DPI-C" function void record_retirement(input longint pc,
					       input longint fetch_cycle,
					       input longint alloc_cycle,
					       input longint complete_cycle,
					       input longint retire_cycle,
					       input int     retire_val,
					       input int     retire_ptr,
					       input longint retire_data,
					       input int     fault,
					       input int     br_mispredict);

import "DPI-C" function void record_restart(input int restart_cycles);
import "DPI-C" function void record_ds_restart(input int delay_cycles);
import "DPI-C" function int check_insn_bytes(input longint pc, input int data);


`endif

module core(clk, 
	    reset,
	    syscall_emu,
	    took_exc,
	    priv,
	    clear_tlb,
	    paging_active,
	    page_table_root,
	    mode64,
	    extern_irq,
	    head_of_rob_ptr_valid,
	    head_of_rob_ptr,
	    resume,
	    memq_empty,
	    drain_ds_complete,
	    dead_rob_mask,
	    resume_pc,
	    ready_for_resume,
	    flush_req_l1d,
	    flush_req_l1i,
	    flush_cl_req,
	    flush_cl_addr,
	    l1d_flush_complete,
	    l1i_flush_complete,
	    l2_flush_complete,
	    insn, 
	    insn_valid,
	    insn_ack,
	    insn_two, 
	    insn_valid_two,
	    
	    insn_ack_two,	    
	    branch_pc,
	    branch_pc_valid,
	    branch_fault,
	    took_branch,
	    branch_pht_idx,
	    restart_pc,
	    restart_src_pc,
	    restart_src_is_indirect,
	    restart_valid,
	    restart_ack,	    
	    core_mem_req_ack,
	    core_mem_req,
	    core_mem_req_valid,

	    core_store_data_valid,
	    core_store_data,
	    core_store_data_ack,
	    
	    core_mem_rsp,
	    core_mem_rsp_valid,
	    alloc_valid,
	    alloc_two_valid,
	    iq_none_valid,
	    iq_one_valid,
	    in_branch_recovery,
	    retire_reg_ptr,
	    retire_reg_data,
	    retire_reg_valid,
	    
	    retire_reg_two_ptr,
	    retire_reg_two_data,
	    retire_reg_two_valid,
	    retire_valid,
	    retire_two_valid,
	    retire_pc,
	    retire_two_pc,
	    rob_empty,
	    retired_call,
	    retired_ret,
	    retired_rob_ptr_valid,
	    retired_rob_ptr_two_valid,
	    retired_rob_ptr,
	    retired_rob_ptr_two,
	    monitor_ack,
	    got_break,
	    got_ud,
	    got_bad_addr,
	    got_monitor,
	    inflight,
	    epc);
   input logic clk;
   input logic reset;
   input logic syscall_emu;
   output logic	took_exc;
   output logic [1:0] priv;
   output	      clear_tlb;
   output logic	      paging_active;
   output logic [63:0] page_table_root;
   
   output logic	mode64;
   input logic extern_irq;
   output logic head_of_rob_ptr_valid;
   output logic [`LG_ROB_ENTRIES-1:0] head_of_rob_ptr;
   input logic resume;
   input logic memq_empty;
   output logic drain_ds_complete;
   output logic [(1<<`LG_ROB_ENTRIES)-1:0] dead_rob_mask;
   
   input logic [(`M_WIDTH-1):0] resume_pc;
   output logic 		ready_for_resume;
   output logic 		flush_req_l1d;
   output logic 		flush_req_l1i;
   
   output logic flush_cl_req;
   output logic [(`M_WIDTH-1):0] flush_cl_addr;
   input logic 	l1d_flush_complete;
   input logic 	l1i_flush_complete;
   input logic 	l2_flush_complete;
	
   input 	insn_fetch_t insn;
   input logic 	insn_valid;
   output logic insn_ack;

   input 	insn_fetch_t insn_two;
   input logic 	insn_valid_two;
   output logic insn_ack_two;
   
   
   output logic [(`M_WIDTH-1):0] restart_pc;
   output logic [(`M_WIDTH-1):0] restart_src_pc;
   output logic 		 restart_src_is_indirect;
   output logic 		 restart_valid;
   input logic 			 restart_ack;
   
   output logic [(`M_WIDTH-1):0] branch_pc;
   output logic 		 branch_pc_valid;
   output logic 		 branch_fault;
   output logic 		 took_branch;
   output logic [`LG_PHT_SZ-1:0] branch_pht_idx;
   
   /* mem port */
   input logic 	 core_mem_req_ack;
   
   output logic  core_mem_req_valid;
   output 	 mem_req_t core_mem_req;

   output logic  core_store_data_valid;
   output 	 mem_data_t core_store_data;
   input logic 	 core_store_data_ack;
  
   input 	 mem_rsp_t core_mem_rsp;
   input logic 	 core_mem_rsp_valid;

   		
   output logic [4:0] 			  retire_reg_ptr;
   output logic [`M_WIDTH-1:0] 		  retire_reg_data;
   output logic 			  retire_reg_valid;

   output logic [4:0] 			  retire_reg_two_ptr;
   output logic [`M_WIDTH-1:0] 		  retire_reg_two_data;
   output logic 			  retire_reg_two_valid;

   output logic 			  alloc_valid;
   output logic 			  alloc_two_valid;

   output logic 			  iq_one_valid;
   output logic 			  iq_none_valid;
      
   output logic 			  in_branch_recovery;
   
   output logic 			  retire_valid;
   output logic 			  retire_two_valid;
   
   output logic [(`M_WIDTH-1):0] 	  retire_pc;
   output logic [(`M_WIDTH-1):0] 	  retire_two_pc;
   output logic 			  retired_call;
   output logic 			  retired_ret;

   output logic 			  retired_rob_ptr_valid;

   output logic retired_rob_ptr_two_valid;
   output logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr;
   output logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr_two;
   
   
   input logic 			      monitor_ack;
   output logic			      rob_empty;
   
   output logic 			  got_break;
   output logic 			  got_ud;
   output logic 			  got_bad_addr;
   output logic 			  got_monitor;
   
   output logic [`LG_ROB_ENTRIES:0] 	  inflight;
   output logic [`M_WIDTH-1:0] 		  epc;
   
   
   localparam N_PRF_ENTRIES = (1<<`LG_PRF_ENTRIES);
   localparam N_ROB_ENTRIES = (1<<`LG_ROB_ENTRIES);
   localparam N_BOB_ENTRIES = (1<<`LG_BOB_ENTRIES);   
   localparam N_UQ_ENTRIES = (1<<`LG_UQ_ENTRIES);
   
   localparam N_DQ_ENTRIES = (1<<`LG_DQ_ENTRIES);
   localparam HI_EBITS = `M_WIDTH-32;

   logic 				  t_push_dq_one, t_push_dq_two;
   
   uop_t r_dq[N_DQ_ENTRIES-1:0];

   
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_head_ptr, n_dq_head_ptr;
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_next_head_ptr, n_dq_next_head_ptr;
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_next_tail_ptr, n_dq_next_tail_ptr;
   
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_cnt, n_dq_cnt;
   
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_tail_ptr, n_dq_tail_ptr;
   logic 				  t_dq_empty, t_dq_full, t_dq_next_empty, t_dq_next_full;
   
   logic 				  r_got_restart_ack, n_got_restart_ack;

   wire [63:0]				  w_exc_pc;
      
   rob_entry_t r_rob[N_ROB_ENTRIES-1:0];   
   bob_entry_t r_bob[N_BOB_ENTRIES-1:0];
   logic [63:0] r_rob_addr[N_ROB_ENTRIES-1:0];
   
   logic [N_ROB_ENTRIES-1:0] 		  r_rob_complete;
   logic [N_ROB_ENTRIES-1:0] 		  r_rob_sd_complete;

   logic 				  t_core_store_data_ptr_valid;
   logic [`LG_ROB_ENTRIES-1:0] 		  t_core_store_data_ptr;
 		  
   logic 				  t_rob_head_complete, t_rob_next_head_complete;
   
   
   logic [N_ROB_ENTRIES-1:0] 		  r_rob_inflight, r_rob_dead_insns;
   logic [N_ROB_ENTRIES-1:0] 		  t_clr_mask;
   
   rob_entry_t t_rob_head, t_rob_next_head, t_rob_tail, t_rob_next_tail;

   logic [N_PRF_ENTRIES-1:0] n_prf_free, r_prf_free;
   wire [N_PRF_ENTRIES-1:0]  w_prf_free_even, w_prf_free_odd;
   wire  w_prf_free_even_full, w_prf_free_odd_full;
   logic r_bank_sel;
   
   
   logic [N_PRF_ENTRIES-1:0] n_retire_prf_free, r_retire_prf_free;
      
   
   logic [`LG_PRF_ENTRIES-1:0]  n_prf_entry, n_prf_entry2;

   //rob
   logic [`LG_ROB_ENTRIES:0] r_rob_head_ptr, n_rob_head_ptr;
   logic [`LG_ROB_ENTRIES:0] r_rob_next_head_ptr, n_rob_next_head_ptr;
   logic [`LG_ROB_ENTRIES:0] r_rob_tail_ptr, n_rob_tail_ptr;
   logic [`LG_ROB_ENTRIES:0] r_rob_next_tail_ptr, n_rob_next_tail_ptr;
   logic 		     t_rob_empty, t_rob_full, t_rob_next_full, t_rob_next_empty;
   
   //bob
   logic [`LG_BOB_ENTRIES:0] r_bob_head_ptr, n_bob_head_ptr;
   logic [`LG_BOB_ENTRIES:0] r_bob_tail_ptr, n_bob_tail_ptr;
   logic 		     t_bob_empty, t_bob_full;
      
         
   logic [`LG_PRF_ENTRIES-1:0] r_alloc_rat[31:0];
   logic [`LG_PRF_ENTRIES-1:0] r_retire_rat[31:0];

   logic [N_ROB_ENTRIES-1:0] 	    uq_wait, mq_wait;
   

   
   
   logic 		     t_alloc, t_alloc_two, t_retire, t_retire_two,
			     t_rat_copy, t_clr_rob;

   logic 		     t_possible_to_alloc;
   
   
   logic 		     t_fold_uop, t_fold_uop2;
   
   logic 		     t_clr_dq;
   logic 		     t_enough_bob;
   logic 		     t_enough_iprfs;
   logic 		     t_enough_next_iprfs;
   
   
   
   logic 		     t_bump_rob_head;
   
   logic [(`M_WIDTH-1):0]    n_restart_pc, r_restart_pc;
   logic [(`M_WIDTH-1):0]    n_restart_src_pc, r_restart_src_pc;
   logic 		     n_restart_src_is_indirect, r_restart_src_is_indirect;
   
   logic [(`M_WIDTH-1):0]    n_branch_pc, r_branch_pc;
   logic 		     n_took_branch, r_took_branch;
   logic 		     n_branch_valid, r_branch_valid;
   logic 		     n_branch_fault,r_branch_fault;
   logic [`LG_PHT_SZ-1:0]    n_branch_pht_idx, r_branch_pht_idx;
         
   logic 		     n_restart_valid,r_restart_valid;
   logic 		     n_take_br, r_take_br;

   logic 		     n_got_break, r_got_break;
   logic 		     n_pending_break, r_pending_break;
   logic 		     n_pending_badva, r_pending_badva;
   logic 		     n_pending_ii, r_pending_ii;
   logic 		     n_got_ud, r_got_ud;
   logic 		     n_got_monitor, r_got_monitor;
   logic 		     n_got_bad_addr, r_got_bad_addr;
   

   logic 		     n_l1i_flush_complete, r_l1i_flush_complete;
   logic 		     n_l1d_flush_complete, r_l1d_flush_complete;
   logic 		     n_l2_flush_complete, r_l2_flush_complete;
   
   
   cause_t 		     n_cause, r_cause;
   logic [63:0]		     r_tval, n_tval;
   logic [63:0]		     r_epc, n_epc;
   
   
   
   complete_t t_complete_bundle_1, t_complete_bundle_2;
   logic 		     t_complete_valid_1, t_complete_valid_2;
   
   logic 		     t_any_complete;
   

   logic 		     t_free_reg;
   logic [`LG_PRF_ENTRIES-1:0] t_free_reg_ptr;
   
   logic 		       t_free_reg_two;
   logic [`LG_PRF_ENTRIES-1:0] t_free_reg_two_ptr;
   

   
   logic [`LG_PRF_ENTRIES:0] 	    t_gpr_ffs, t_gpr_ffs2;
   logic 			    t_gpr_ffs_full, t_gpr_ffs2_full;
   wire [`LG_PRF_ENTRIES:0] 	    w_gpr_ffs_even, w_gpr_ffs_odd;
   
   logic 		     t_uq_full, t_uq_next_full;
   
   logic 		     n_ready_for_resume, r_ready_for_resume;
   
   
   mem_req_t t_mem_req;
   logic 		     t_mem_req_valid;

   logic 		     n_machine_clr, r_machine_clr;
   logic 		     n_flush_req_l1d, r_flush_req_l1d;
   logic 		     n_flush_req_l1i, r_flush_req_l1i;
   
   logic 		     n_flush_cl_req, r_flush_cl_req;
   logic [(`M_WIDTH-1):0]    n_flush_cl_addr, r_flush_cl_addr;
   logic 		     r_ds_done, n_ds_done;
   
   logic 		     t_can_retire_rob_head;
   logic 		     t_arch_fault;
   
   typedef enum logic [4:0] {
			     FLUSH_FOR_HALT, //0			     
			     HALT, //1
			     ACTIVE, //2
			     DRAIN, //3
			     RAT, //4
			     ALLOC_FOR_SERIALIZE, //6
			     FLUSH_CACHE, //7
			     HANDLE_MONITOR, //8
			     ALLOC_FOR_MONITOR, //9
			     WAIT_FOR_MONITOR, //10
			     HALT_WAIT_FOR_RESTART, //11
			     WAIT_FOR_SERIALIZE_AND_RESTART, //12
			     ARCH_FAULT,
			     WRITE_CSRS
			     } state_t;
   
   state_t r_state, n_state;
   logic 	r_pending_fault, n_pending_fault;
   logic [31:0] r_restart_cycles, n_restart_cycles;
   logic t_divide_ready;
   
   
   always_comb
     begin
	core_mem_req_valid = t_mem_req_valid;
	core_mem_req = t_mem_req;
     end // always_comb
   

   assign ready_for_resume = r_ready_for_resume;
   assign in_branch_recovery = (r_state == DRAIN) || (r_state == RAT);
      
   assign head_of_rob_ptr_valid = (r_state == ACTIVE);
   assign head_of_rob_ptr = r_rob_head_ptr[`LG_ROB_ENTRIES-1:0];
   wire [63:0]	w_rob_head_addr = r_rob_addr[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]];
   
   assign flush_req_l1d = r_flush_req_l1d;
   assign flush_req_l1i = r_flush_req_l1i;
   assign flush_cl_req = r_flush_cl_req;
   assign flush_cl_addr = r_flush_cl_addr;

   
   assign got_break = r_got_break;
   assign got_ud = r_got_ud;
   assign got_bad_addr = r_got_bad_addr;
   assign got_monitor = r_got_monitor;
   assign epc = r_epc;
   
   popcount #(`LG_ROB_ENTRIES) inflight0 (.in(r_rob_inflight), 
					  .out(inflight));

   
   uop_t t_uop, t_dec_uop, t_alloc_uop;
   uop_t t_uop2, t_dec_uop2, t_alloc_uop2;
      
   assign insn_ack = !t_dq_full && insn_valid && (r_state == ACTIVE);
   assign insn_ack_two = !t_dq_full && 
			 insn_valid && 
			 !t_dq_next_full && 
			 insn_valid_two && (r_state == ACTIVE);
   
   assign restart_pc = r_restart_pc;
   assign restart_src_pc = r_restart_src_pc;
   assign restart_src_is_indirect = r_restart_src_is_indirect;

   assign dead_rob_mask = r_rob_dead_insns;   
   assign restart_valid = r_restart_valid;

   
   assign branch_pc = r_branch_pc;
   assign branch_pc_valid = r_branch_valid;
   assign branch_fault = r_branch_fault;
   assign branch_pht_idx = r_branch_pht_idx;
   

   assign took_branch = r_took_branch;
   
   logic r_update_csr_exc, n_update_csr_exc;
   logic r_mode64, n_mode64;
   assign mode64 = r_mode64;
   assign took_exc = r_update_csr_exc;
   
   logic [63:0] r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : r_cycle + 'd1;

     end


`ifdef VERILATOR
   logic [31:0] r_clear_cnt;
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_clear_cnt <= 'd0;
	  end
	else if(n_ds_done)
	  begin
	     r_clear_cnt <=  r_clear_cnt + 'd1;
	  end
     end
`endif


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_update_csr_exc <= 1'b0;
	     r_flush_req_l1i <= 1'b0;
	     r_flush_req_l1d <= 1'b0;
	     r_flush_cl_req <= 1'b0;
	     r_flush_cl_addr <= 'd0;
	     r_restart_pc <= 'd0;
	     r_restart_src_pc <= 'd0;
	     r_restart_src_is_indirect <= 1'b0;
	     r_branch_pc <= 'd0;
	     r_took_branch <= 1'b0;
	     r_branch_valid <= 1'b0;
	     r_branch_fault <= 1'b0;
	     r_branch_pht_idx <= 'd0;
	     r_restart_valid <= 1'b0;
	     r_take_br <= 1'b0;
	     r_got_break <= 1'b0;
	     r_pending_break <= 1'b0;
	     r_pending_badva <= 1'b0;
	     r_pending_ii <= 1'b0;
	     r_got_ud <= 1'b0;
	     r_got_bad_addr <= 1'b0;
	     r_got_monitor <= 1'b0;
	     r_ready_for_resume <= 1'b0;
	     r_l1i_flush_complete <= 1'b0;
	     r_l1d_flush_complete <= 1'b0;
	     r_l2_flush_complete <= 1'b0;
	     r_epc <= 'd0;
	     drain_ds_complete <= 1'b0;	     
	     r_ds_done <= 1'b0;
	  end
	else
	  begin
	     r_update_csr_exc <= n_update_csr_exc;
	     r_flush_req_l1d <= n_flush_req_l1d;
	     r_flush_req_l1i <= n_flush_req_l1i;
	     r_flush_cl_req <= n_flush_cl_req;
	     r_flush_cl_addr <= n_flush_cl_addr;
	     r_restart_pc <= n_restart_pc;
	     r_restart_src_pc <= n_restart_src_pc;
	     r_restart_src_is_indirect <= n_restart_src_is_indirect;
	     r_branch_pc <= n_branch_pc;
	     r_took_branch <= n_took_branch;
	     r_branch_valid <= n_branch_valid;
	     r_branch_fault <= n_branch_fault;
	     r_branch_pht_idx <= n_branch_pht_idx;
	     r_restart_valid <= n_restart_valid;
	     r_take_br <= n_take_br;
	     r_got_break <= n_got_break;
	     r_pending_break <= n_pending_break;
	     r_pending_badva <= n_pending_badva;
	     r_pending_ii <= n_pending_ii;	     
	     r_got_ud <= n_got_ud;
	     r_got_bad_addr <= n_got_bad_addr;
	     r_got_monitor <= n_got_monitor;
	     r_ready_for_resume <= n_ready_for_resume;
	     r_l1i_flush_complete <= n_l1i_flush_complete;
	     r_l1d_flush_complete <= n_l1d_flush_complete;
	     r_l2_flush_complete <= n_l2_flush_complete;
	     r_epc <= n_epc;
	     drain_ds_complete <= r_ds_done;	     
	     r_ds_done <= n_ds_done;
	  end
     end // always_ff@ (posedge clk)

   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mode64 <= 1'b1;	     
	     r_state <= FLUSH_FOR_HALT;
	     r_restart_cycles <= 'd0;
	     r_machine_clr <= 1'b0;
	     r_got_restart_ack <= 1'b0;
	     r_cause <= 'd0;
	     r_tval <= 'd0;
	     r_pending_fault <= 1'b0;
	  end
	else
	  begin
	     r_mode64 <= n_mode64;
	     r_state <= n_state;
	     r_restart_cycles <= n_restart_cycles;
	     r_machine_clr <= n_machine_clr;
	     r_got_restart_ack <= n_got_restart_ack;
	     r_cause <= n_cause;
	     r_tval <= n_tval;
	     r_pending_fault <= n_pending_fault;
	  end
     end

   always_ff@(posedge clk)
     begin
   	if(reset)
   	  begin
   	     retire_reg_ptr <= 'd0;
   	     retire_reg_data <= 'd0;
   	     retire_reg_valid <= 1'b0;
   	     retire_reg_two_ptr <= 'd0;
   	     retire_reg_two_data <= 'd0;
   	     retire_reg_two_valid <= 1'b0;
   	     retire_valid <= 1'b0;
	     retire_two_valid <= 1'b0;
	     alloc_valid <= 1'b0;
	     rob_empty <= 1'b0;
	     alloc_two_valid <= 1'b0;
	     iq_one_valid <= 1'b0;
	     iq_none_valid <= 1'b0;
   	     retire_pc <= 'd0;
	     retire_two_pc <= 'd0;
	     retired_call <= 1'b0;
	     retired_ret <= 1'b0;
	     retired_rob_ptr_valid <= 1'b0;
	     
	     retired_rob_ptr_two_valid <= 1'b0;
	     retired_rob_ptr <= 'd0;
	     retired_rob_ptr_two <= 'd0;
   	  end
   	else
   	  begin
   	     retire_reg_ptr <= t_rob_head.ldst;
   	     retire_reg_data <= t_rob_head.data;
   	     retire_reg_valid <= t_rob_head.valid_dst && t_retire;
   	     retire_reg_two_ptr <= t_rob_next_head.ldst;
   	     retire_reg_two_data <= t_rob_next_head.data;
   	     retire_reg_two_valid <= t_rob_next_head.valid_dst && t_retire_two;
	     
   	     retire_valid <= t_retire;
	     retire_two_valid <= t_retire_two;
	     rob_empty <= t_rob_empty;
	     alloc_valid <= t_alloc;
	     alloc_two_valid <= t_alloc_two;
	     iq_one_valid <= !t_dq_empty && t_dq_next_empty;
	     iq_none_valid <= t_dq_empty;
   	     retire_pc <= t_rob_head.pc;
	     retire_two_pc <= t_rob_next_head.pc;
	     retired_ret <= t_rob_head.is_ret && t_retire;
	     retired_call <= t_rob_head.is_call && t_retire;

	     retired_rob_ptr_valid <= t_retire;
	     
	     retired_rob_ptr_two_valid <= t_retire_two;
	     retired_rob_ptr <= r_rob_head_ptr[`LG_ROB_ENTRIES-1:0];
	     retired_rob_ptr_two <= r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0];
   	  end
     end
`ifdef ENABLE_CYCLE_ACCOUNTING
   always_ff@(negedge clk)
     begin
	record_alloc(t_rob_full ? 32'd1 : 32'd0,
		     t_alloc ? 32'd1 : 32'd0,
		     t_alloc_two ? 32'd1 : 32'd0,
		     t_dq_empty ? 32'd1 : 32'd0,
		     
		     t_uq_full ? 32'd1 : 32'd0,
		     t_uq_next_full ? 32'd1 : 32'd0,
		     
		     t_dq_empty ? 32'd0 : 32'd1,
		     !t_dq_next_empty && !t_dq_empty ? 32'd1 : 32'd0,
		     t_possible_to_alloc ? 32'd1 : 32'd0);
			    
   	if(t_retire)
   	  begin
	     record_retirement(
			       { {(64-`M_WIDTH){1'b0}},t_rob_head.pc},
   			       t_rob_head.fetch_cycle,
   			       t_rob_head.alloc_cycle,
   			       t_rob_head.complete_cycle,
   			       r_cycle,
			       t_rob_head.valid_dst ? 32'd1 : 32'd0,
			       {27'd0, t_rob_head.ldst},
			       {{(64-`M_WIDTH){1'b0}},t_rob_head.data},
			       t_rob_head.faulted ? 32'd1 : 32'd0,
			       t_rob_head.faulted ? 32'd1 : 32'd0			       
			       );
   	  end
   	if(t_retire_two)
   	  begin
	     record_retirement(
			       { {(64-`M_WIDTH){1'b0}},t_rob_next_head.pc},
   			       t_rob_next_head.fetch_cycle,
   			       t_rob_next_head.alloc_cycle,
   			       t_rob_next_head.complete_cycle,
   			       r_cycle,
			       t_rob_next_head.valid_dst ? 32'd1 : 32'd0,
			       {27'd0, t_rob_next_head.ldst},
			       {{(64-`M_WIDTH){1'b0}},t_rob_next_head.data},			       
			       t_rob_next_head.faulted ? 32'd1 : 32'd0,
			       32'd0);	     
   	  end // if (t_retire_two)
	if(r_state == RAT && n_state == ACTIVE)
	  begin
	     record_restart(r_restart_cycles);
	  end
	if(r_state == DRAIN && n_state == RAT)
	  begin
	     record_ds_restart(r_restart_cycles);
	  end
	    
     end // always_ff@ (negedge clk)
`endif
   
   
//`define DEBUG
`ifdef VERILATOR
   logic [31:0] t_faults, t_branches;
   always_comb
     begin
	t_faults = 'd0;
	t_branches = 'd0;
	for(logic [`LG_ROB_ENTRIES:0] i = r_rob_head_ptr; i != (r_rob_tail_ptr); i=i+1)
	  begin
	     if(r_rob_complete[i[`LG_ROB_ENTRIES-1:0]]  && r_rob[i[`LG_ROB_ENTRIES-1:0]].faulted)
	       begin
		  t_faults = t_faults + 'd1;
	       end
	     if(r_rob[i[`LG_ROB_ENTRIES-1:0]].is_br && r_rob_complete[i[`LG_ROB_ENTRIES-1:0]])
	       begin
		  t_branches = t_branches + 'd1;
	       end
	  end
     end // always_comb
   
   always_ff@(negedge clk)
     begin
	record_faults(t_faults);
	record_branches(t_branches);
     end
`endif
   
`define DUMP_ROB
`ifdef DUMP_ROB
   logic [15:0] r_last_cycle;
   
   always_ff@(negedge clk)
     begin
	if(retire_valid || retire_two_valid)
	  r_last_cycle <= 'd0;
	else
	  r_last_cycle <= r_last_cycle + 'd1;
	
	if(r_last_cycle >= 'd16384 || paging_active)
	  begin 
	     $display("cycle %d : state = %d, alu complete %b, mem complete %b,head_ptr %d, complete %b,  can_retire_rob_head %b, head pc %x, empty %b, full %b, bob full %b", 
		      r_cycle,
		      r_state,
		      t_complete_valid_1,
		      core_mem_rsp_valid,
		      r_rob_head_ptr,
		      t_rob_head_complete && !t_rob_empty, 
		      t_can_retire_rob_head,
		      t_rob_head.pc,
		      t_rob_empty, 
		      t_rob_full,
		      t_bob_full);
	     
	     for(logic [`LG_ROB_ENTRIES:0] i = r_rob_head_ptr; i != (r_rob_tail_ptr); i=i+1)
	       begin
		  $display("\trob entry %d, pc %x, insn %x complete %b, sd complete %b, is br %b, faulted %b, cause %d",
			   i[`LG_ROB_ENTRIES-1:0], 
			   r_rob[i[`LG_ROB_ENTRIES-1:0]].pc,
			   r_rob[i[`LG_ROB_ENTRIES-1:0]].raw_insn, 
			   r_rob_complete[i[`LG_ROB_ENTRIES-1:0]],
			   r_rob_sd_complete[i[`LG_ROB_ENTRIES-1:0]],
			   r_rob[i[`LG_ROB_ENTRIES-1:0]].is_br,
			   r_rob[i[`LG_ROB_ENTRIES-1:0]].faulted,
			   r_rob[i[`LG_ROB_ENTRIES-1:0]].cause
			   );
	       end
	  end
     end // always_ff@ (negedge clk)
`endif

   logic t_restart_complete;
   logic t_clr_extern_irq;
   logic r_extern_irq;
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_extern_irq <= 1'b0;
	  end
	else
	  begin
	     if(t_clr_extern_irq)
	       begin
		  r_extern_irq <= 1'b0;
	       end 
	     else if(extern_irq)
	       begin
		  r_extern_irq <= 1'b1;
	       end
	  end // else: !if(reset)
     end // always_ff@ (posedge clk)

   always_comb
     begin
	n_mode64 = r_mode64;
	t_clr_extern_irq = 1'b0;
	t_restart_complete = 1'b0;
	n_cause = r_cause;
	n_epc = r_epc;
	n_tval = r_tval;

	n_machine_clr = r_machine_clr;
	t_alloc = 1'b0;
	t_alloc_two = 1'b0;
	t_possible_to_alloc = 1'b0;
	
	t_retire = 1'b0;
	t_retire_two = 1'b0;
	t_rat_copy = 1'b0;
	t_clr_rob = 1'b0;
	t_clr_dq = 1'b0;
	n_state = r_state;
	n_restart_cycles = r_restart_cycles + 'd1;
	n_restart_pc = r_restart_pc;
	n_restart_src_pc = r_restart_src_pc;
	n_restart_src_is_indirect = r_restart_src_is_indirect;
	n_restart_valid = 1'b0;
	n_take_br = r_take_br;	
	t_bump_rob_head = 1'b0;
	n_pending_fault = r_pending_fault;
	n_pending_badva = r_pending_badva;
	n_pending_ii = r_pending_ii;
	
	t_enough_iprfs = !((t_uop.dst_valid) && t_gpr_ffs_full);

	t_enough_bob = t_uop.is_br ? !t_bob_full : 1'b1;
	
	t_enough_next_iprfs = !((t_uop2.dst_valid) && t_gpr_ffs2_full);

	
	t_fold_uop = (t_uop.op == NOP || t_uop.op == II || t_uop.op == FETCH_PF || t_uop.op == J );
	t_fold_uop2 = (t_uop2.op == NOP || t_uop2.op == II || t_uop2.op == FETCH_PF || t_uop2.op == J);

	n_ds_done = r_ds_done;
	n_flush_req_l1d = 1'b0;
	n_flush_req_l1i = 1'b0;
	n_flush_cl_req = 1'b0;
	n_flush_cl_addr = r_flush_cl_addr;
	n_got_break = r_got_break;
	n_pending_break = r_pending_break;
	n_got_ud = r_got_ud;
	n_got_bad_addr = r_got_bad_addr;
	n_got_restart_ack = r_got_restart_ack;
	n_got_monitor = r_got_monitor;
	n_ready_for_resume = 1'b0;
	n_update_csr_exc = 1'b0;
	
	n_l1i_flush_complete = r_l1i_flush_complete || l1i_flush_complete;
	n_l1d_flush_complete = r_l1d_flush_complete || l1d_flush_complete;
	n_l2_flush_complete = r_l2_flush_complete || l2_flush_complete;
	
	
	if(r_state == ACTIVE)
	  begin
	     n_got_restart_ack = 1'b0;
	  end
	else if(!r_got_restart_ack)
	  begin
	     n_got_restart_ack = restart_ack;
	  end	
	
	t_can_retire_rob_head = t_rob_head_complete && !t_rob_empty;

	if(t_complete_valid_1 || t_complete_valid_2)
	  begin
	     n_pending_fault = r_pending_fault | 
			       (t_complete_valid_1 ? t_complete_bundle_1.faulted : 1'b0) |
			       (t_complete_valid_2 ? t_complete_bundle_2.faulted : 1'b0);
	  end
	
	t_arch_fault = t_rob_head.faulted & t_rob_head.has_cause;
	
	unique case (r_state)
	  ACTIVE:
	    begin
	       if(r_extern_irq && !t_rob_empty)
		 begin
		    n_state = DRAIN;
		    n_restart_pc = t_rob_head.pc;
		    n_machine_clr = 1'b1;
		    n_ds_done = 1'b1;
		    t_clr_extern_irq = 1'b1;
		    n_restart_valid = 1'b1;
		 end
	       else if(t_can_retire_rob_head)
		 begin
		    if(t_rob_head.faulted)
		      begin
			 if(t_arch_fault)
			   begin
			      n_state = ARCH_FAULT;
			      n_cause = t_rob_head.cause;
			      n_epc = t_rob_head.pc;
			      n_tval = 'd0;
			   end
			 else
			   begin
			      n_state = DRAIN;
			      n_restart_cycles = 'd1;
			      n_restart_valid = 1'b1;
			      t_bump_rob_head = 1'b1;			      
			   end // else: !if(t_rob_head.is_ii)
			 n_ds_done = 1'b1;
			 n_machine_clr = 1'b1;
			 n_restart_pc = t_rob_head.target_pc;
			 n_restart_src_pc = t_rob_head.pc;
			 n_restart_src_is_indirect = t_rob_head.is_indirect && !t_rob_head.is_ret;
			 n_take_br = t_rob_head.take_br;
		      end // if (t_rob_head.faulted)
		    else if(!t_dq_empty)
		      begin
			 if(t_uop.serializing_op)
			   begin
			      if(/*r_inflight*/t_rob_empty)
				begin
				   n_state = (t_uop.op == MONITOR) ? 
					     HANDLE_MONITOR : ALLOC_FOR_SERIALIZE;
				end
			   end
			 else
			   begin
			      t_possible_to_alloc = !t_rob_full
						    && !t_uq_full
						    && !t_dq_empty;

			      t_alloc = !t_rob_full
					&& !r_pending_fault
					&& !t_uq_full
					&& !t_dq_empty					
					&& t_enough_iprfs
					&& t_enough_bob;
			      
			      t_alloc_two = t_alloc
					    && !t_uop.is_br
					    && !t_uop2.serializing_op
					    && !t_dq_next_empty 
					    && !t_rob_next_full
					    && !t_uq_next_full
					    && t_enough_next_iprfs;

			      //&& (t_uop2.op == NOP || t_uop2.op == J);
			   end // else: !if(t_uop.serializing_op && !t_dq_empty)
		      end // if (!t_dq_empty)
		    t_retire = t_rob_head_complete & !t_arch_fault;
		    t_retire_two = !t_rob_next_empty
		    		   && !t_rob_head.faulted
		    		   && !t_rob_next_head.faulted 				    
		    		   && t_rob_head_complete
		    		   && t_rob_next_head_complete				    
				   && !t_rob_head.is_br
				   && !t_rob_next_head.is_ret
				   && !t_rob_next_head.is_call;
		    
		 end // if (t_can_retire_rob_head)
	       else if(!t_dq_empty)
		 begin
		    //$display("t_uop.serializing_op = %b, rob empty %b, monitor %b", 
		    //t_uop.serializing_op, t_rob_empty,
		    //t_uop.op == MONITOR);

		    if(t_uop.serializing_op && t_rob_empty)
		      begin
			 if(t_uop.op == MONITOR || t_uop.op == SFENCEVMA)
			   begin
			      n_flush_req_l1i = 1'b1;
			      n_flush_req_l1d = 1'b1;
			      n_state = FLUSH_CACHE;
			   end // if (t_uop.op == MONITOR)
			 else
			   begin
			      n_state =  ALLOC_FOR_SERIALIZE;
			   end
		      end // if (t_uop.serializing_op)
		    else if(!t_uop.serializing_op)
		      begin
			 t_possible_to_alloc = !t_rob_full
					       && !t_uq_full
					       && !t_dq_empty;
			 
			 t_alloc = !t_rob_full
				   && !t_uop.serializing_op
				   && !t_uq_full
				   && !t_dq_empty
				   && t_enough_iprfs
				   && t_enough_bob;
			 
			 
			 t_alloc_two = t_alloc
				       && !t_uop.is_br
				       && !t_uop2.serializing_op
				       && !t_dq_next_empty 
				       && !t_rob_next_full
				       && !t_uq_next_full
				       && t_enough_bob
				       && t_enough_next_iprfs;

		      end
		 end
	    end // case: ACTIVE
	  DRAIN:	    
	    begin
	       if(r_rob_inflight == 'd0 && memq_empty && t_divide_ready)
		 begin
		    n_state = RAT;
		    //$display(">>> restarting after fault at cycle %d at priv %d, paging enabled %b", r_cycle, priv, paging_active);
		 end 
	    end // case: DRAIN
	  RAT:
	    begin
	       t_rat_copy = 1'b1;
	       t_clr_rob = 1'b1;
	       t_clr_dq = 1'b1;
	       n_machine_clr = 1'b0;
	       
	       if(n_got_restart_ack)
		 begin
		    n_state = ACTIVE;
		    n_ds_done = 1'b0;
		    n_pending_fault = 1'b0;
		    t_restart_complete = 1'b1;
		 end
	    end
	  ALLOC_FOR_SERIALIZE:
	    begin
	       t_alloc = !t_rob_full && !t_uq_full 
			 && (r_prf_free != 'd0) 
			   && !t_dq_empty;
	       n_state = t_alloc ? WAIT_FOR_SERIALIZE_AND_RESTART : ALLOC_FOR_SERIALIZE;
	    end
	  WAIT_FOR_SERIALIZE_AND_RESTART:
	    begin
	       if(t_rob_head_complete)
		 begin
		    t_clr_dq = 1'b1;
		    n_restart_pc = t_rob_head.target_pc;
		    n_restart_src_pc = t_rob_head.pc;
		    n_restart_src_is_indirect = 1'b0;
		    n_restart_valid = 1'b1;
		    n_pending_fault = 1'b0;		    
		    if(n_got_restart_ack)
		      begin
			 //$display("RESTART PIPELINE AT %d, pc %x", 
			 //r_cycle, n_restart_pc);
			 n_state = ACTIVE;			 
		      end
		 end
	    end
	  FLUSH_CACHE:
	    begin
	       //$display("monitor flush %b %b", n_l1d_flush_complete, n_l2_flush_complete);
	       if(n_l1i_flush_complete && n_l1d_flush_complete && n_l2_flush_complete)
		 begin
		    n_got_monitor = t_uop.op == MONITOR;
		    n_state = (t_uop.op == MONITOR) ? HANDLE_MONITOR : ALLOC_FOR_SERIALIZE;
		    n_l1i_flush_complete = 1'b0;
		    n_l1d_flush_complete = 1'b0;
		    n_l2_flush_complete = 1'b0;
		 end
	    end
	  HANDLE_MONITOR:
	    begin
	       if(monitor_ack)
		 begin
		    //$display("monitor ack");
		    n_got_monitor = 1'b0;
		    n_state = ALLOC_FOR_MONITOR;
		 end
	    end
	  ALLOC_FOR_MONITOR:
	    begin
	       t_alloc = !t_rob_full && !t_uq_full 
			 && (r_prf_free != 'd0) 
			   && !t_dq_empty;			 
	       n_state = WAIT_FOR_MONITOR;
	    end
	  WAIT_FOR_MONITOR:
	    begin
	       if(t_rob_head_complete)
		 begin
		    t_clr_dq = 1'b1;
		    n_restart_pc = t_rob_head.pc + 'd4;
		    n_restart_src_pc = t_rob_head.pc;
		    n_restart_src_is_indirect = 1'b0;
		    n_restart_valid = 1'b1;
		    n_pending_fault = 1'b0;
		    if(n_got_restart_ack)
		      begin
			 t_retire = 1'b1;
			 n_state = ACTIVE;
		      end
		 end
	    end // case: WAIT_FOR_MONITOR
	  FLUSH_FOR_HALT:
	    begin
	       //$display("%d : %b %b %b", r_cycle, n_l1i_flush_complete, n_l1d_flush_complete, n_l2_flush_complete);
	       if(n_l1i_flush_complete && n_l1d_flush_complete && n_l2_flush_complete)
		 begin
		    n_state = HALT;
		    n_ds_done = 1'b0;
		    n_got_break = r_pending_break;
		    n_got_ud = r_pending_ii;
		    n_got_bad_addr = r_pending_badva;
		    n_pending_break = 1'b0;
		    n_pending_badva = 1'b0;
		    n_pending_ii = 1'b0;
		    n_ready_for_resume = 1'b1;		    		    
		    n_l1i_flush_complete = 1'b0;
		    n_l1d_flush_complete = 1'b0;
		    n_l2_flush_complete = 1'b0;
		 end	       
	    end
	  HALT:
	    begin
	       //$display("HALTED");
	       if(resume)
		 begin
		    n_restart_pc = resume_pc;
		    n_restart_src_pc = t_rob_head.pc;
		    n_restart_src_is_indirect = 1'b0;
		    n_restart_valid = 1'b1;
		    n_state = HALT_WAIT_FOR_RESTART;
		    n_got_break = 1'b0;
		    n_got_ud = 1'b0;
		    t_clr_dq = 1'b1;			    
		 end
	       else
		 begin
		    n_ready_for_resume = 1'b1;		    
		 end
	    end // case: HALT
	  HALT_WAIT_FOR_RESTART:
	    begin
	       n_pending_fault = 1'b0;
	       if(n_got_restart_ack)
		 begin
		    n_state = ACTIVE;
		 end
	    end
	  ARCH_FAULT:
	    begin
	       //$display("took fault for %x with cause %d at cycle %d, priv %d", 
	       //t_rob_head.pc, t_rob_head.cause, r_cycle, priv);
	       case(t_rob_head.cause)
		 BREAKPOINT:
		   begin
		      n_pending_break = 1'b1;
		   end
		 ILLEGAL_INSTRUCTION:
		   begin
		      n_pending_ii = 1'b1;
		      //$display("took fault for %x with cause %d at cycle %d, priv %d", 
		      //t_rob_head.pc, t_rob_head.cause, r_cycle, priv);
		   end
		 FETCH_PAGE_FAULT:
		   begin
		      n_tval = t_rob_head.pc;
		   end
		 LOAD_PAGE_FAULT:
		   begin
		      n_tval = w_rob_head_addr;
		   end
		 STORE_PAGE_FAULT:
		   begin
		      n_tval = w_rob_head_addr;		      
		   end
		 default:
		   begin
		     // $display("t_rob_head.cause = %d, ", t_rob_head.cause);
		   end
	       endcase // case (t_rob_head.cause)
	       t_bump_rob_head = 1'b1;
	       if(syscall_emu)
		 begin
		    n_flush_req_l1i = 1'b1;
		    n_flush_req_l1d = 1'b1;
		    n_state = FLUSH_FOR_HALT;
		 end
	       else
		 begin
		    n_state = WRITE_CSRS;
		 end
	    end
	  WRITE_CSRS:
	    begin
	       n_update_csr_exc = 1'b1;
	       //$display("exception handler pc %x. page root %x",
	       //w_exc_pc, page_table_root);
	       if(w_exc_pc == r_epc)
		 $stop();
	       n_restart_pc = w_exc_pc;
	       n_restart_valid = 1'b1;	       
	       n_state = DRAIN;
	    end // case: WRITE_CSRS
	  default:
	    begin
	    end
	endcase // unique case (r_state)
     end // always_comb

      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_rob_head_ptr <= 'd0;
	     r_rob_tail_ptr <= 'd0;
	     r_rob_next_head_ptr <= 'd1;
	     r_rob_next_tail_ptr <= 'd1;
	     r_bob_head_ptr <= 'd0;
	     r_bob_tail_ptr <= 'd0;
	  end
	else
	  begin
	     r_rob_head_ptr <= n_rob_head_ptr;
	     r_rob_tail_ptr <= n_rob_tail_ptr;
	     r_rob_next_head_ptr <= n_rob_next_head_ptr;
	     r_rob_next_tail_ptr <= n_rob_next_tail_ptr;
	     r_bob_head_ptr <= n_bob_head_ptr;
	     r_bob_tail_ptr <= n_bob_tail_ptr;
	  end
     end // always_ff@ (posedge clk)


   wire [`LG_PRF_ENTRIES-1:0] w_rn_srcA_1 = r_alloc_rat[t_uop.srcA[4:0]];
   wire [`LG_PRF_ENTRIES-1:0] w_rn_srcB_1 = r_alloc_rat[t_uop.srcB[4:0]];
   wire [`LG_PRF_ENTRIES-1:0] w_rn_srcA_2_ = r_alloc_rat[t_uop2.srcA[4:0]];
   wire [`LG_PRF_ENTRIES-1:0] w_rn_srcB_2_ = r_alloc_rat[t_uop2.srcB[4:0]];
   wire 		      w_srcA_match = t_uop.dst_valid & (t_uop2.srcA[4:0] == t_uop.dst[4:0]);
   wire 		      w_srcB_match = t_uop.dst_valid & (t_uop2.srcB[4:0] == t_uop.dst[4:0]);
   wire [`LG_PRF_ENTRIES-1:0] w_rn_srcA_2 = w_srcA_match ? n_prf_entry : w_rn_srcA_2_;
   wire [`LG_PRF_ENTRIES-1:0] w_rn_srcB_2 = w_srcB_match ? n_prf_entry : w_rn_srcB_2_;
   
   

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(logic [`LG_PRF_ENTRIES-1:0] i_rat = 'd0; i_rat < 'd32; i_rat = i_rat + 'd1)
	       begin
		  r_alloc_rat[i_rat[4:0]] <= i_rat;
	       end
	  end
	else if(t_rat_copy)
	  begin
	     r_alloc_rat <= r_retire_rat;
	  end
	else
	  begin
	     if(t_alloc && t_uop.dst_valid)
	       begin
		  r_alloc_rat[t_uop.dst[4:0]] <= n_prf_entry;
	       end
	     if(t_alloc_two &&t_uop2.dst_valid)
	       begin
		  r_alloc_rat[t_uop2.dst[4:0]] <= n_prf_entry2;
	       end	     
	  end
     end // always_ff@ (posedge clk)
   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(logic [`LG_PRF_ENTRIES-1:0] i_rat = 'd0; i_rat < 'd32; i_rat = i_rat + 'd1)
	       begin
		  r_retire_rat[i_rat[4:0]] <= i_rat;
	       end
	  end
	else 
	  begin
	     if(t_free_reg)
	       begin
		  r_retire_rat[t_rob_head.ldst] <= t_rob_head.pdst;
	       end
	     if(t_free_reg_two)
	       begin
		  r_retire_rat[t_rob_next_head.ldst] <= t_rob_next_head.pdst;		  
	       end
	  end // always_ff@ (posedge clk)
     end // always_ff@ (posedge clk)

      
   always_comb
     begin
	t_alloc_uop = t_uop;
	t_alloc_uop2 = t_uop2;
	t_alloc_uop.srcA = w_rn_srcA_1;
	t_alloc_uop.srcB = w_rn_srcB_1;
	t_alloc_uop2.srcA = w_rn_srcA_2;
	t_alloc_uop2.srcB = w_rn_srcB_2;

	if(t_alloc)
	  begin
	     if(t_uop.dst_valid)
	       begin
		  t_alloc_uop.dst = n_prf_entry;
	       end
	     
	     t_alloc_uop.rob_ptr = r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0];
	  end // if (t_alloc)
	
	if(t_alloc_two)
	  begin
	     if(t_uop2.dst_valid)
	       begin
		  t_alloc_uop2.dst = n_prf_entry2;
	       end
	     t_alloc_uop2.rob_ptr = r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0];
	  end
     end // always_comb
  
   always_comb
     begin
	t_free_reg = 1'b0;
	t_free_reg_ptr = 'd0;
	t_free_reg_two = 1'b0;
	t_free_reg_two_ptr = 'd0;
	
	n_retire_prf_free = r_retire_prf_free;
	
	n_branch_pc = {{HI_EBITS{1'b0}}, 32'd0};
	n_took_branch = 1'b0;
	n_branch_valid = 1'b0;
	n_branch_fault = 1'b0;
	n_branch_pht_idx = 'd0;
	
	if(t_retire)
	  begin
	     if(t_rob_head.valid_dst)
	       begin
		  t_free_reg = 1'b1;
		  t_free_reg_ptr = t_rob_head.old_pdst;
		  n_retire_prf_free[t_rob_head.pdst] = 1'b0;
		  n_retire_prf_free[t_rob_head.old_pdst] = 1'b1;
	       end

	     if(t_retire_two && t_rob_next_head.valid_dst)
	       begin
		  t_free_reg_two = 1'b1;
		  t_free_reg_two_ptr = t_rob_next_head.old_pdst;
		  n_retire_prf_free[t_rob_next_head.pdst] = 1'b0;
		  n_retire_prf_free[t_rob_next_head.old_pdst] = 1'b1;
	       end
	     
	     n_branch_pc = t_retire_two ? t_rob_next_head.pc : t_rob_head.pc;
	     n_took_branch = t_retire_two ? t_rob_next_head.take_br : t_rob_head.take_br;
	     n_branch_valid = t_retire_two ? t_rob_next_head.is_br :  t_rob_head.is_br;
	     n_branch_fault = t_rob_head.faulted & (t_rob_head.has_cause==1'b0);
	     n_branch_pht_idx = t_retire_two ? t_rob_next_head.pht_idx : t_rob_head.pht_idx;
	  end // if (t_retire)
	
     end // always_comb
   
   
   always_comb
     begin
	t_rob_tail.faulted  = 1'b0;
	t_rob_tail.valid_dst  = 1'b0;
	t_rob_tail.ldst  = 'd0;
	t_rob_tail.pdst  = 'd0;
	t_rob_tail.old_pdst  = 'd0;
	t_rob_tail.pc = t_alloc_uop.pc;
	t_rob_tail.target_pc = t_alloc_uop.pc + 'd4;
	
	t_rob_tail.is_call = t_alloc_uop.op == JAL || t_alloc_uop.op == JALR;
	t_rob_tail.is_ret = (t_alloc_uop.op == RET);
	t_rob_tail.is_indirect = t_alloc_uop.op == JALR || t_alloc_uop.op == JR;

	t_rob_tail.has_cause = 1'b0;
	t_rob_tail.cause = 4'd0;
	t_rob_tail.take_br = 1'b0;
	t_rob_tail.is_br = t_alloc_uop.is_br;	
	t_rob_tail.data= 'd0;
	t_rob_tail.pht_idx = t_alloc_uop.pht_idx;

	t_rob_next_tail.faulted  = 1'b0;
	t_rob_next_tail.valid_dst  = 1'b0;
	t_rob_next_tail.ldst  = 'd0;
	t_rob_next_tail.pdst  = 'd0;
	t_rob_next_tail.old_pdst  = 'd0;
	t_rob_next_tail.pc = t_alloc_uop2.pc;
	t_rob_next_tail.target_pc = t_alloc_uop2.pc + 'd4;

	t_rob_next_tail.is_call = t_alloc_uop2.op == JAL || t_alloc_uop2.op == JALR;
	t_rob_next_tail.is_ret = (t_alloc_uop2.op == RET);
	t_rob_next_tail.is_indirect = t_alloc_uop2.op == JALR || t_alloc_uop2.op == JR;
	
	t_rob_next_tail.cause = 4'd0;
	t_rob_next_tail.has_cause = 1'b0;
	t_rob_next_tail.take_br = 1'b0;
	t_rob_next_tail.is_br = t_alloc_uop2.is_br;
	t_rob_next_tail.data = 'd0;
	t_rob_next_tail.pht_idx = t_alloc_uop2.pht_idx;
	
	
	if(t_alloc)
	  begin	     
`ifdef ENABLE_CYCLE_ACCOUNTING
	     t_rob_tail.fetch_cycle = t_alloc_uop.fetch_cycle;
	     t_rob_tail.alloc_cycle = r_cycle;
	     t_rob_tail.raw_insn = t_alloc_uop.raw_insn;
	     t_rob_tail.complete_cycle = 'd0;
`endif	     
	     if(t_uop.dst_valid)
	       begin
		  t_rob_tail.valid_dst = 1'b1;
		  /* this is correct, we do not want the renamed version */
		  t_rob_tail.ldst = t_uop.dst[4:0];
		  t_rob_tail.pdst = n_prf_entry;
		  t_rob_tail.old_pdst = r_alloc_rat[t_uop.dst[4:0]];
	       end
	     
	     if(t_fold_uop)
	       begin
`ifdef ENABLE_CYCLE_ACCOUNTING
		  t_rob_tail.complete_cycle = r_cycle;
`endif
		  if(t_uop.op == II)
                   begin
                      t_rob_tail.faulted = 1'b1;
                      t_rob_tail.has_cause = 1'b1;
                      t_rob_tail.cause = ILLEGAL_INSTRUCTION;
                   end
		  else if(t_uop.op == FETCH_PF)
		    begin
                      t_rob_tail.faulted = 1'b1;
                      t_rob_tail.has_cause = 1'b1;
                      t_rob_tail.cause = FETCH_PAGE_FAULT;
		    end
		  else if(t_uop.op == J)
		    begin
		       t_rob_tail.take_br = 1'b1;
		    end
	       end
	     
	  end // if (t_alloc)


	if(t_alloc_two)
	  begin

	     
`ifdef ENABLE_CYCLE_ACCOUNTING
	     t_rob_next_tail.fetch_cycle = t_alloc_uop2.fetch_cycle;
	     t_rob_next_tail.alloc_cycle = r_cycle;
	     t_rob_next_tail.raw_insn = t_alloc_uop2.raw_insn;
	     t_rob_next_tail.complete_cycle = 'd0;
`endif

	     if(t_uop2.dst_valid)
	       begin
		  t_rob_next_tail.valid_dst = 1'b1;
		  /* this is correct, we do not want the renamed version */
		  t_rob_next_tail.ldst = t_uop2.dst[4:0];
		  t_rob_next_tail.pdst = n_prf_entry2;
		  t_rob_next_tail.old_pdst = (t_uop.dst_valid && (t_uop.dst == t_uop2.dst)) ? t_rob_tail.pdst : r_alloc_rat[t_uop2.dst[4:0]];
	       end




	     if(t_fold_uop2)
	       begin
`ifdef ENABLE_CYCLE_ACCOUNTING
		  t_rob_next_tail.complete_cycle = r_cycle;
`endif
		  if(t_uop2.op == II)
                   begin
                      t_rob_next_tail.faulted = 1'b1;
                      t_rob_next_tail.has_cause = 1'b1;
                      t_rob_next_tail.cause = ILLEGAL_INSTRUCTION;
                   end
		  else if(t_uop2.op == FETCH_PF)
		    begin
                      t_rob_next_tail.faulted = 1'b1;
                      t_rob_next_tail.has_cause = 1'b1;
                      t_rob_next_tail.cause = FETCH_PAGE_FAULT;
		    end
		  else if(t_uop2.op == J)
		    begin
		       t_rob_next_tail.take_br = 1'b1;
		    end
	       end // if (t_fold_uop2)
	  end // if (t_alloc_two)
	
	

	

     end // always_comb
   


   always_ff@(posedge clk)
     begin
	if(reset || t_clr_rob)
	  begin
	     r_rob_complete <= 'd0;
	     r_rob_sd_complete <= 'd0;
	  end
	else
	  begin
	     if(t_alloc)
	       begin
		  r_rob_complete[r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= t_fold_uop;
		  r_rob_sd_complete[r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= !(t_uop.is_mem & t_uop.srcB_valid);
	       end
	     if(t_alloc_two)
	       begin
		  r_rob_complete[r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= t_fold_uop2;
		  r_rob_sd_complete[r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= !(t_uop2.is_mem & t_uop2.srcB_valid);				    
	       end
	     if(t_complete_valid_1)
	       begin
		  r_rob_complete[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]] <= t_complete_bundle_1.complete;
	       end
	     if(t_complete_valid_2)
	       begin
		  r_rob_complete[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]] <= t_complete_bundle_2.complete;
	       end
	     if(core_mem_rsp_valid)
	       begin
		  //$display("rob entry %d marked complete by mem port", core_mem_rsp.rob_ptr);
		  r_rob_complete[core_mem_rsp.rob_ptr] <= 1'b1;
	       end

	     if(t_core_store_data_ptr_valid)
	       begin
		  r_rob_sd_complete[t_core_store_data_ptr] <= 1'b1;
	       end
	  end
     end // always_ff@ (posedge clk)


   always_ff@(posedge clk)
     begin
	if(core_mem_rsp_valid)
	  begin
	     r_rob_addr[core_mem_rsp.rob_ptr] <= core_mem_rsp.addr;
	  end
     end
   
   always_ff@(posedge clk)
     begin
	if(reset || t_clr_rob)
	  begin
	     for(integer i = 0; i < N_ROB_ENTRIES; i=i+1)
	       begin
		  r_rob[i].faulted <= 1'b0;
	       end
	  end
	else
	  begin
	     if(t_alloc)
	       begin
		  r_rob[r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= t_rob_tail;
	       end
	     if(t_alloc_two)
	       begin
		  r_rob[r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= t_rob_next_tail;
	       end
	     if(t_complete_valid_1)
	       begin
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].faulted <= t_complete_bundle_1.faulted;
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].has_cause <= t_complete_bundle_1.has_cause;
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].cause <= t_complete_bundle_1.cause;		  
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].target_pc <= t_complete_bundle_1.restart_pc;
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_br <= t_complete_bundle_1.take_br;
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].data <= t_complete_bundle_1.data;
`ifdef ENABLE_CYCLE_ACCOUNTING
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].complete_cycle <= r_cycle;
`endif	    
	       end // if (t_complete_valid_1)
	     if(t_complete_valid_2)
	       begin
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].faulted <= t_complete_bundle_2.faulted;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].has_cause <= t_complete_bundle_2.has_cause;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].cause <= t_complete_bundle_2.cause;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].target_pc <= t_complete_bundle_2.restart_pc;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_br <= t_complete_bundle_2.take_br;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].data <= t_complete_bundle_2.data;
`ifdef ENABLE_CYCLE_ACCOUNTING
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].complete_cycle <= r_cycle;
`endif	    
	       end	     
	     if(core_mem_rsp_valid)
	       begin
		  r_rob[core_mem_rsp.rob_ptr].data <= core_mem_rsp.data;
		  r_rob[core_mem_rsp.rob_ptr].faulted <= core_mem_rsp.has_cause;
		  r_rob[core_mem_rsp.rob_ptr].cause <= core_mem_rsp.cause;
		  r_rob[core_mem_rsp.rob_ptr].has_cause <= core_mem_rsp.has_cause;
		  //$display("mem_rsp rob ; %x, rob ptr %d has cause %b %d", r_rob[core_mem_rsp.rob_ptr].pc, core_mem_rsp.rob_ptr, core_mem_rsp.has_cause, core_mem_rsp.cause);
		  
					    
`ifdef ENABLE_CYCLE_ACCOUNTING
		  r_rob[core_mem_rsp.rob_ptr].complete_cycle <= r_cycle;
`endif	    	     	     
	       end
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	if(reset || t_clr_rob)
	  begin
	     for(integer i = 0; i < N_BOB_ENTRIES; i=i+1)
	       begin
		  r_bob[i].valid <= 1'b0;
	       end
	  end
	else
	  begin
	     if(t_alloc && !t_alloc_two && t_uop.is_br)
	       begin
		  r_bob[r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]].valid <= 1'b1;
		  r_bob[r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]].rob_ptr <= r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0];
		  //$display("case 1 push rob entry %d to bob loc %d", r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0], r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]);
	       end
	     else if(t_alloc && t_alloc_two)
	       begin
		  if(t_uop.is_br && !t_uop2.is_br)
		    begin
		       r_bob[r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]].valid <= 1'b1;
		       r_bob[r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]].rob_ptr <= r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0];		       	
		       //$display("case 2 push rob entry %d to bob loc %d, bob full %b", 
		       //r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0], r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0],
		       //t_bob_full);	       
		    end
		  else if(!t_uop.is_br && t_uop2.is_br)
		    begin
		       r_bob[r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]].valid <= 1'b1;
		       r_bob[r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]].rob_ptr <= r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0];
		       //$display("case 3 push rob entry %d to bob loc %d", r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0], r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]);		       
		    end
	       end
	  end	
     end // always_ff@ (posedge clk)
   

   
   always_ff@(posedge clk)
     begin
	if(reset || t_clr_rob)
	  begin 
	     r_rob_dead_insns <= 'd0;
	  end
	else
	  begin
	     if(t_retire)
	       begin
		  r_rob_dead_insns[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]] <= 1'b0;		  
	       end
	     if(t_retire_two)
	       begin
		  r_rob_dead_insns[r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0]] <= 1'b0;
	       end
	     if(t_alloc)
	       begin
		  r_rob_dead_insns[r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= 1'b1;		  
	       end
	     if(t_alloc_two)
	       begin
		  r_rob_dead_insns[r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= 1'b1;		  
	       end
	  end // else: !if(reset || t_clr_rob)
     end // always_ff@ (posedge clk)


   always_comb
     begin
	t_clr_mask = uq_wait|mq_wait;
	if(t_complete_valid_1)
	  begin
	     t_clr_mask[t_complete_bundle_1.rob_ptr] = 1'b1;
	  end
	if(t_complete_valid_2)
	  begin
	     t_clr_mask[t_complete_bundle_2.rob_ptr] = 1'b1;
	  end
	
	if(core_mem_rsp_valid)
	  begin
	     t_clr_mask[core_mem_rsp.rob_ptr] = 1'b1;
	  end
     end
    
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_rob_inflight <= 'd0;
	  end
	else
	  begin
	     if(r_ds_done)
	       begin
		  r_rob_inflight <= r_rob_inflight & (~t_clr_mask);
	       end
	     else
	       begin
		  if(t_complete_valid_1)
		    begin
		       //$display("cycle %d, 1 rob ptr %d complete\n", r_cycle, t_complete_bundle_1.rob_ptr);		  
		       r_rob_inflight[t_complete_bundle_1.rob_ptr] <= 1'b0;		  
		    end
		  if(t_complete_valid_2)
		    begin
		       //$display("cycle %d, 1 rob ptr %d complete\n", r_cycle, t_complete_bundle_2.rob_ptr);		  
		       r_rob_inflight[t_complete_bundle_2.rob_ptr] <= 1'b0;		  
		    end
		  
		  if(core_mem_rsp_valid)
		    begin
		       //$display("cycle %d, M rob ptr %d complete\n", r_cycle, core_mem_rsp.rob_ptr);
		       r_rob_inflight[core_mem_rsp.rob_ptr] <= 1'b0;
		    end
		  if(t_alloc && !t_fold_uop)
		    begin
		       r_rob_inflight[r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= 1'b1;		  
		    end
		  if(t_alloc_two && !t_fold_uop2)
		    begin
		       r_rob_inflight[r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= 1'b1;
		    end
	       end
	  end // else: !if(reset)
     end // always_ff@ (posedge clk)
	          
   
   always_comb
     begin
	n_rob_head_ptr = r_rob_head_ptr;
	n_rob_tail_ptr = r_rob_tail_ptr;
	n_rob_next_head_ptr = r_rob_next_head_ptr;
	n_rob_next_tail_ptr = r_rob_next_tail_ptr;
	n_bob_head_ptr = r_bob_head_ptr;
	n_bob_tail_ptr = r_bob_tail_ptr;
	
	//rob control 
	if(t_clr_rob)
	  begin
	     n_rob_head_ptr = 'd0;
	     n_rob_tail_ptr = 'd0;
	     n_rob_next_head_ptr = 'd1;
	     n_rob_next_tail_ptr = 'd1;
	  end
	else
	  begin
	     if(t_alloc && !t_alloc_two)
	       begin
		  n_rob_tail_ptr = r_rob_tail_ptr + 'd1;
		  n_rob_next_tail_ptr = r_rob_next_tail_ptr + 'd1;
	       end
	     else if(t_alloc && t_alloc_two)
	       begin
		  n_rob_tail_ptr = r_rob_tail_ptr + 'd2;
		  n_rob_next_tail_ptr = r_rob_next_tail_ptr + 'd2;
	       end

	     
	     if(t_retire || t_bump_rob_head)
	       begin
		  n_rob_head_ptr = t_retire_two ? r_rob_head_ptr + 'd2 : 
				   r_rob_head_ptr + 'd1;
		  n_rob_next_head_ptr = t_retire_two ? r_rob_next_head_ptr + 'd2 : 
					r_rob_next_head_ptr + 'd1;
	       end
	  end // else: !if(t_clr_rob)

	//bob control
	if(t_clr_rob)
	  begin
	     n_bob_head_ptr = 'd0;
	     n_bob_tail_ptr = 'd0;
	  end
	else
	  begin
	     if(t_alloc && !t_alloc_two && t_uop.is_br)
	       begin
		  n_bob_tail_ptr = r_bob_tail_ptr + 'd1;
	       end
	     else if(t_alloc && t_alloc_two && (t_uop.is_br || t_uop2.is_br))
	       begin
		  n_bob_tail_ptr = r_bob_tail_ptr + 'd1;
	       end

	     //can only retire one branch per cycle
	     if((t_retire || t_bump_rob_head) && t_rob_head.is_br)
	       begin
		  //$display("cycle %d popped head of bob, r_bob ptr =%d, rob ptr = %d",
		  //r_cycle,
		  //r_bob[r_bob_head_ptr[`LG_BOB_ENTRIES-1:0]].rob_ptr,
		  //r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]);
		  n_bob_head_ptr = r_bob_head_ptr + 'd1;
	       end
	     else if(t_retire_two && t_rob_next_head.is_br)
	       begin
		  // $display("cycle %d popped head of bob, r_bob ptr =%d, rob ptr = %d",
		  //r_cycle,
		  //r_bob[r_bob_head_ptr[`LG_BOB_ENTRIES-1:0]].rob_ptr,
		  //r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]);
		  n_bob_head_ptr = r_bob_head_ptr + 'd1;
	       end
	  end

	
	
	
	t_rob_empty = (r_rob_head_ptr == r_rob_tail_ptr);
	t_rob_next_empty = (r_rob_next_head_ptr == r_rob_tail_ptr);
	t_rob_full = (r_rob_head_ptr[`LG_ROB_ENTRIES-1:0] == r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]) && (r_rob_head_ptr != r_rob_tail_ptr);
	t_rob_next_full = (r_rob_head_ptr[`LG_ROB_ENTRIES-1:0] == r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]) && (r_rob_head_ptr != r_rob_next_tail_ptr);


	t_bob_empty = (r_bob_head_ptr == r_bob_tail_ptr);
	t_bob_full = (r_bob_head_ptr[`LG_BOB_ENTRIES-1:0] == r_bob_tail_ptr[`LG_BOB_ENTRIES-1:0]) && (r_bob_head_ptr != r_bob_tail_ptr);
     end // always_comb
   

   always_comb
     begin
	t_rob_head = r_rob[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]];
	t_rob_next_head = r_rob[r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0]];
	
	t_rob_head_complete = r_rob_sd_complete[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]] &
			      r_rob_complete[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]];
	
	t_rob_next_head_complete = r_rob_sd_complete[r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0]] &
				   r_rob_complete[r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0]];
     end // always_comb

   

   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i < N_PRF_ENTRIES; i = i + 1)
	       begin
		  r_prf_free[i] <= (i < 32) ? 1'b0 : 1'b1;
		  r_retire_prf_free[i] <= (i < 32) ? 1'b0 : 1'b1;
	       end
	  end
	else
	  begin
	     r_prf_free <= t_rat_copy ? r_retire_prf_free : n_prf_free;
	     r_retire_prf_free <= n_retire_prf_free;
	  end
     end // always_ff@ (posedge clk)


   generate
      for(genvar i = 0; i < N_PRF_ENTRIES; i=i+2)
	begin
	   assign w_prf_free_even[i] = r_prf_free[i];
	   assign w_prf_free_even[i+1] = 1'b0;
	   assign w_prf_free_odd[i] = 1'b0;	   
	   assign w_prf_free_odd[i+1] = r_prf_free[i+1];
	end
   endgenerate


   assign w_prf_free_even_full = (|w_prf_free_even) == 1'b0;
   assign w_prf_free_odd_full = (|w_prf_free_odd) == 1'b0;
   
   
   find_first_set#(`LG_PRF_ENTRIES) ffs_gpr(.in(w_prf_free_even),
					    .y(w_gpr_ffs_even));

   find_first_set#(`LG_PRF_ENTRIES) ffs_gpr2(.in(w_prf_free_odd),
					     .y(w_gpr_ffs_odd));
   always_ff@(posedge clk)
     begin
	r_bank_sel <= reset ? 1'b0 : ~r_bank_sel;
     end
   
   always_comb
     begin
	t_gpr_ffs  = r_bank_sel ? w_gpr_ffs_even : w_gpr_ffs_odd;
	t_gpr_ffs2 = r_bank_sel ? w_gpr_ffs_odd : w_gpr_ffs_even;
	t_gpr_ffs_full = r_bank_sel ? w_prf_free_even_full : w_prf_free_odd_full;
	t_gpr_ffs2_full = r_bank_sel ? w_prf_free_odd_full : w_prf_free_even_full;
     end
   
   always_comb
     begin
	n_prf_free = r_prf_free;
	n_prf_entry = t_gpr_ffs[`LG_PRF_ENTRIES-1:0];
	n_prf_entry2 = t_gpr_ffs2[`LG_PRF_ENTRIES-1:0];
	
	if(t_alloc & t_uop.dst_valid)
	  begin
	     n_prf_free[n_prf_entry] = 1'b0;
	  end
	if(t_alloc_two && t_uop2.dst_valid)
	  begin
	     n_prf_free[n_prf_entry2] = 1'b0;
	  end
	if(t_free_reg)
	  begin
	     n_prf_free[t_free_reg_ptr] = 1'b1;
	  end
	if(t_free_reg_two)
	  begin
	     n_prf_free[t_free_reg_two_ptr] = 1'b1;
	  end
     end // always_comb

   
   decode_riscv dec0 
     (
      .mode64(r_mode64),
      .insn(insn.insn_bytes),
      .page_fault(insn.page_fault),
      .pc(insn.pc), 
      .insn_pred(insn.pred), 
      .pht_idx(insn.pht_idx),
      .insn_pred_target(insn.pred_target),
`ifdef ENABLE_CYCLE_ACCOUNTING
      .fetch_cycle(insn.fetch_cycle),
`endif
      .syscall_emu(syscall_emu),		      
      .uop(t_dec_uop)
      );
   
   decode_riscv dec1 
     (	
	.mode64(r_mode64),
	.insn(insn_two.insn_bytes),
	.page_fault(insn_two.page_fault), 
	.pc(insn_two.pc), 
	.insn_pred(insn_two.pred), 
	.pht_idx(insn_two.pht_idx),
	.insn_pred_target(insn_two.pred_target),
`ifdef ENABLE_CYCLE_ACCOUNTING
	.fetch_cycle(insn_two.fetch_cycle),
`endif
	.syscall_emu(syscall_emu),	
	.uop(t_dec_uop2)
	);
   

   
   logic t_push_1, t_push_2;
   
   always_comb
     begin
	t_any_complete = t_complete_valid_1 | core_mem_rsp_valid | t_complete_valid_2;
	t_push_1 = t_alloc && !t_fold_uop;
	t_push_2 = t_alloc_two && !t_fold_uop2;
     end

   exec e (
	   .clk(clk), 
	   .reset(reset),
	   .priv(priv),
	   .paging_active(paging_active),
	   .page_table_root(page_table_root),
	   .update_csr_exc(r_update_csr_exc),
	   .cause(r_cause),
	   .epc(r_epc),
	   .tval(r_tval),
	   .exc_pc(w_exc_pc),
	   .clear_tlb(clear_tlb),
	   .mode64(r_mode64),
	   .retire(t_retire),
	   .retire_two(t_retire_two),
	   .divide_ready(t_divide_ready),
`ifdef VERILATOR
	   .clear_cnt(r_clear_cnt),
`endif
	   .ds_done(r_ds_done),
	   .mem_dq_clr(t_clr_rob),
	   .restart_complete(t_restart_complete),
	   .mq_wait(mq_wait),
	   .uq_wait(uq_wait),
	   .uq_full(t_uq_full),
	   .uq_next_full(t_uq_next_full),
	   .uq_uop(t_push_1 ? t_alloc_uop : t_alloc_uop2),
	   .uq_uop_two(t_alloc_uop2),	   
	   .uq_push(t_push_1 || (!t_push_1 && t_push_2)),
	   .uq_push_two(t_push_2 && t_push_1),
	   	   
	   .complete_bundle_1(t_complete_bundle_1),
	   .complete_valid_1(t_complete_valid_1),
	   .complete_bundle_2(t_complete_bundle_2),
	   .complete_valid_2(t_complete_valid_2),
	   
	   .mem_req(t_mem_req),
	   .mem_req_valid(t_mem_req_valid),
	   .mem_req_ack(core_mem_req_ack),
	   .core_store_data_valid(core_store_data_valid),
	   .core_store_data(core_store_data),
	   .core_store_data_ack(core_store_data_ack),
	   .core_store_data_ptr_valid(t_core_store_data_ptr_valid),
	   .core_store_data_ptr(t_core_store_data_ptr),
	   .mem_rsp_dst_ptr(core_mem_rsp.dst_ptr),
	   .mem_rsp_dst_valid(core_mem_rsp.dst_valid),
	   .mem_rsp_load_data(core_mem_rsp.data),
	   .branch_valid(r_branch_valid),
	   .branch_fault(r_branch_fault)
	   );


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_dq_head_ptr <= 'd0;
	     r_dq_next_head_ptr <= 'd1;
	     r_dq_next_tail_ptr <= 'd1;
	     r_dq_tail_ptr <= 'd0;
	     r_dq_cnt <= 'd0;
	  end
	else
	  begin
	     r_dq_head_ptr <= t_clr_rob ? 'd0 :n_dq_head_ptr;
	     r_dq_tail_ptr <= t_clr_rob ? 'd0 :n_dq_tail_ptr;	     
	     r_dq_next_head_ptr <= t_clr_rob ? 'd1 : n_dq_next_head_ptr;
	     r_dq_next_tail_ptr <= t_clr_rob ? 'd1 : n_dq_next_tail_ptr;
	     r_dq_cnt <= t_clr_rob ? 'd0 : n_dq_cnt;
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	if(t_push_dq_one)
	  r_dq[r_dq_tail_ptr[`LG_DQ_ENTRIES-1:0]] <= t_dec_uop;
	if(t_push_dq_two)
	  r_dq[r_dq_next_tail_ptr[`LG_DQ_ENTRIES-1:0]] <= t_dec_uop2;
     end

   always_ff@(negedge clk)
     begin
	// if(t_push_dq_one && (t_dec_uop.op==FETCH_PF)) 
	//   begin
	//      $display("decoded %x to uop %d, is FETCH_PF=%b", 
	// 	      t_dec_uop.pc, t_dec_uop.op, t_dec_uop.op==FETCH_PF);
	//   end
	//if(t_push_dq_two && (t_dec_uop2.op==FETCH_PF)) $stop();
	//if(t_push_dq_one)
	// $display("decoded %x to uop %d, is II=%b", t_dec_uop.pc, t_dec_uop.op, t_dec_uop.op==FETCH_PF);
	//if(t_push_dq_two)
	// $display("decoded %x to uop %d, is II=%b", t_dec_uop2.pc, t_dec_uop2.op, t_dec_uop2.op==FETCH_PF);	

    	if(insn_ack && insn_ack_two && 1'b0)
    	  begin
    	     $display("ack two insns in cycle %d, valid %b, %b, pc %x %x", 
    		      r_cycle, insn_valid, insn_valid_two,
   		      insn.pc, insn_two.pc);
    	  end
    	else if(insn_ack && !insn_ack_two && 1'b0)
    	  begin
    	     $display("ack one insn in cycle %d, valid %b, pc %x ", 
    		      r_cycle, insn_valid, 
    		      insn.pc);
    	  end
     end
   
   always_comb
     begin
	t_push_dq_one = 1'b0;
	t_push_dq_two = 1'b0;
	n_dq_tail_ptr = r_dq_tail_ptr;
	n_dq_head_ptr = r_dq_head_ptr;
	n_dq_next_head_ptr = r_dq_next_head_ptr;
	n_dq_next_tail_ptr = r_dq_next_tail_ptr;
	
	t_dq_empty = (r_dq_tail_ptr == r_dq_head_ptr);
	t_dq_next_empty = (r_dq_tail_ptr == r_dq_next_head_ptr);
	
	t_dq_full = (r_dq_tail_ptr[`LG_DQ_ENTRIES-1:0] == r_dq_head_ptr[`LG_DQ_ENTRIES-1:0]) && (r_dq_tail_ptr != r_dq_head_ptr);

	t_dq_next_full = (r_dq_next_tail_ptr[`LG_DQ_ENTRIES-1:0] == r_dq_head_ptr[`LG_DQ_ENTRIES-1:0]) && (r_dq_next_tail_ptr != r_dq_head_ptr);
	
	n_dq_cnt = r_dq_cnt;
		
	t_uop = r_dq[r_dq_head_ptr[`LG_DQ_ENTRIES-1:0]];
	t_uop2 = r_dq[r_dq_next_head_ptr[`LG_DQ_ENTRIES-1:0]];
	
	if(t_clr_dq)
	  begin
	     n_dq_tail_ptr = 'd0;
	     n_dq_head_ptr = 'd0;
	     n_dq_next_head_ptr = 'd1;
	     n_dq_next_tail_ptr = 'd1;
	     n_dq_cnt = 'd0;
	  end
	else
	  begin
	     if(insn_valid && !t_dq_full && !(!t_dq_next_full && insn_valid_two))
	       begin
		  //push one instruction
		  t_push_dq_one = 1'b1;
		  n_dq_tail_ptr = r_dq_tail_ptr + 'd1;
		  n_dq_next_tail_ptr = r_dq_next_tail_ptr + 'd1;
		  n_dq_cnt = n_dq_cnt + 'd1;
	       end
	     else if(insn_valid && !t_dq_full && !t_dq_next_full && insn_valid_two)
	       begin
		  //push two instructions
		  t_push_dq_one = 1'b1;
		  t_push_dq_two = 1'b1;		  
		  n_dq_tail_ptr = r_dq_tail_ptr + 'd2;
		  n_dq_next_tail_ptr = r_dq_next_tail_ptr + 'd2;
		  n_dq_cnt = n_dq_cnt + 'd2;
	       end
	     
	     if(t_alloc && !t_alloc_two)
	       begin
		  n_dq_head_ptr = r_dq_head_ptr + 'd1;
		  n_dq_next_head_ptr = r_dq_next_head_ptr + 'd1;
		  n_dq_cnt = n_dq_cnt - 'd1;
	       end
	     else if(t_alloc && t_alloc_two)
	       begin
		  n_dq_head_ptr = r_dq_head_ptr + 'd2;
		  n_dq_next_head_ptr = r_dq_next_head_ptr + 'd2;
		  n_dq_cnt = n_dq_cnt - 'd2;
	       end
	  end
     end // always_comb
   
endmodule
