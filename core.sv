`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

`ifdef VERILATOR
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
					       input int     fault,
					       input int     is_mem,
					       input int     is_fp,
					       input int     missed_l1d);

import "DPI-C" function void record_restart(input int restart_cycles);
import "DPI-C" function void record_ds_restart(input int delay_cycles);
import "DPI-C" function int check_insn_bytes(input longint pc, input int data);


`endif

module core(clk, 
	    reset,
	    extern_irq,
	    head_of_rob_ptr_valid,
	    head_of_rob_ptr,
	    resume,
	    memq_empty,
	    drain_ds_complete,
	    dead_rob_mask,
	    resume_pc,
	    ready_for_resume,
	    flush_req,
	    flush_cl_req,
	    flush_cl_addr,
	    l1d_flush_complete,
	    l1i_flush_complete,
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
	
	    core_mem_rsp,
	    core_mem_rsp_valid,
	    
	    retire_reg_ptr,
	    retire_reg_data,
	    retire_reg_valid,
	    retire_reg_fp_valid,
	    
	    retire_reg_two_ptr,
	    retire_reg_two_data,
	    retire_reg_two_valid,
	    retire_reg_fp_two_valid,	    
	    retire_valid,
	    retire_two_valid,
	    retire_delay_slot,
	    retire_pc,
	    retire_two_pc,
	    retired_call,
	    retired_ret,
	    retired_rob_ptr_valid,
	    retired_rob_ptr_two_valid,
	    retired_rob_ptr,
	    retired_rob_ptr_two,
	    
	    monitor_req_reason,
	    monitor_req_valid,
	    monitor_rsp_valid,
	    monitor_rsp_data_valid,
	    monitor_rsp_data,
	    got_break,
	    got_ud,
	    inflight);
   input logic clk;
   input logic reset;
   input logic extern_irq;
   output logic head_of_rob_ptr_valid;
   output logic [`LG_ROB_ENTRIES-1:0] head_of_rob_ptr;
   input logic resume;
   input logic memq_empty;
   output logic drain_ds_complete;
   output logic [(1<<`LG_ROB_ENTRIES)-1:0] dead_rob_mask;
   
   input logic [(`M_WIDTH-1):0] resume_pc;
   output logic 		ready_for_resume;
   output logic flush_req;
   output logic flush_cl_req;
   output logic [(`M_WIDTH-1):0] flush_cl_addr;
   input logic 	l1d_flush_complete;
   input logic 	l1i_flush_complete;
   
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

   input 	 mem_rsp_t core_mem_rsp;
   input logic 	 core_mem_rsp_valid;

   output logic [4:0] 			  retire_reg_ptr;
   output logic [63:0] 			  retire_reg_data;
   output logic 			  retire_reg_valid;
   output logic 			  retire_reg_fp_valid;

   output logic [4:0] 			  retire_reg_two_ptr;
   output logic [63:0] 			  retire_reg_two_data;
   output logic 			  retire_reg_two_valid;
   output logic 			  retire_reg_fp_two_valid;
   
   output logic 			  retire_valid;
   output logic 			  retire_two_valid;
   
   output logic 			  retire_delay_slot;
   
   output logic [(`M_WIDTH-1):0] 	  retire_pc;
   output logic [(`M_WIDTH-1):0] 	  retire_two_pc;
   output logic 			  retired_call;
   output logic 			  retired_ret;

   output logic 			  retired_rob_ptr_valid;

   output logic retired_rob_ptr_two_valid;
   output logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr;
   output logic [`LG_ROB_ENTRIES-1:0] retired_rob_ptr_two;
   
   
   output logic [15:0] 			  monitor_req_reason;
   output logic 			  monitor_req_valid;
   input logic 				  monitor_rsp_valid;
   input logic 				  monitor_rsp_data_valid;
   input logic [(`M_WIDTH-1):0] 	  monitor_rsp_data;
   output logic 			  got_break;
   output logic 			  got_ud;
   output logic [`LG_ROB_ENTRIES:0] 	  inflight;
   
   
   
   localparam N_PRF_ENTRIES = (1<<`LG_PRF_ENTRIES);
   localparam N_ROB_ENTRIES = (1<<`LG_ROB_ENTRIES);
   localparam N_UQ_ENTRIES = (1<<`LG_UQ_ENTRIES);
   localparam N_HILO_ENTRIES = (1<<`LG_HILO_PRF_ENTRIES);
   localparam N_FCR_ENTRIES = (1<<`LG_FCR_PRF_ENTRIES);
   
   localparam N_DQ_ENTRIES = (1<<`LG_DQ_ENTRIES);
   localparam HI_EBITS = `M_WIDTH-32;
   
   uop_t r_dq[N_DQ_ENTRIES-1:0];
   uop_t n_dq[N_DQ_ENTRIES-1:0];

   logic [15:0] 			  r_monitor_reason, n_monitor_reason;
   
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_head_ptr, n_dq_head_ptr;
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_next_head_ptr, n_dq_next_head_ptr;
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_next_tail_ptr, n_dq_next_tail_ptr;
   
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_cnt, n_dq_cnt;
   
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_tail_ptr, n_dq_tail_ptr;
   logic 				  t_dq_empty, t_dq_full, t_dq_next_empty, t_dq_next_full;
   
   logic 				  r_got_restart_ack, n_got_restart_ack;
   
   rob_entry_t r_rob[N_ROB_ENTRIES-1:0];
   logic [N_ROB_ENTRIES-1:0] 		  r_rob_complete;
   logic 				  t_rob_head_complete, t_rob_next_head_complete;
   
   
   logic [N_ROB_ENTRIES-1:0] 		  r_rob_inflight, r_rob_dead_insns;
   logic [N_ROB_ENTRIES-1:0] 		  t_clr_mask;
   
   rob_entry_t t_rob_head, t_rob_next_head, t_rob_tail, t_rob_next_tail;

   logic [N_PRF_ENTRIES-1:0] n_prf_free, r_prf_free, t_prf_free;   
   logic [N_PRF_ENTRIES-1:0] n_retire_prf_free, r_retire_prf_free;
   logic [`LG_PRF_ENTRIES:0] t_prf_free_cnt;
   
   logic [N_PRF_ENTRIES-1:0] n_fp_prf_free, r_fp_prf_free, t_fp_prf_free;   
   logic [N_PRF_ENTRIES-1:0] n_retire_fp_prf_free, r_retire_fp_prf_free;
   logic [`LG_PRF_ENTRIES:0] t_fp_prf_free_cnt;
   
   logic [N_HILO_ENTRIES-1:0] n_hilo_prf_free,r_hilo_prf_free;
   logic [N_HILO_ENTRIES-1:0] n_retire_hilo_prf_free, r_retire_hilo_prf_free;
   logic [`LG_HILO_PRF_ENTRIES-1:0] n_hilo_prf_entry;
   logic [`LG_HILO_PRF_ENTRIES:0]   t_hilo_prf_idx;

   logic [N_FCR_ENTRIES-1:0] n_fcr_prf_free,r_fcr_prf_free;
   logic [N_FCR_ENTRIES-1:0] n_retire_fcr_prf_free, r_retire_fcr_prf_free;
   logic [`LG_FCR_PRF_ENTRIES-1:0] n_fcr_prf_entry;
   logic [`LG_FCR_PRF_ENTRIES:0]   t_fcr_prf_idx;
   
   logic [`LG_PRF_ENTRIES-1:0]  n_prf_entry, n_prf_entry2;
   logic [`LG_PRF_ENTRIES-1:0] 	n_fp_prf_entry, n_fp_prf_entry2;

   logic [`LG_ROB_ENTRIES:0] r_rob_head_ptr, n_rob_head_ptr;
   logic [`LG_ROB_ENTRIES:0] r_rob_next_head_ptr, n_rob_next_head_ptr;
   
   logic [`LG_ROB_ENTRIES:0] r_rob_tail_ptr, n_rob_tail_ptr;
   logic [`LG_ROB_ENTRIES:0] r_rob_next_tail_ptr, n_rob_next_tail_ptr;
         
   logic [`LG_PRF_ENTRIES-1:0] r_alloc_rat[31:0];
   logic [`LG_PRF_ENTRIES-1:0] n_alloc_rat[31:0];
   logic [`LG_PRF_ENTRIES-1:0] r_retire_rat[31:0];
   logic [`LG_PRF_ENTRIES-1:0] n_retire_rat[31:0];

   logic [`LG_PRF_ENTRIES-1:0] r_fp_alloc_rat[31:0];
   logic [`LG_PRF_ENTRIES-1:0] n_fp_alloc_rat[31:0];
   logic [`LG_PRF_ENTRIES-1:0] r_fp_retire_rat[31:0];
   logic [`LG_PRF_ENTRIES-1:0] n_fp_retire_rat[31:0];

   
   logic [`LG_HILO_PRF_ENTRIES-1:0] r_hilo_alloc_rat;
   logic [`LG_HILO_PRF_ENTRIES-1:0] n_hilo_alloc_rat;
   logic [`LG_HILO_PRF_ENTRIES-1:0] r_hilo_retire_rat;
   logic [`LG_HILO_PRF_ENTRIES-1:0] n_hilo_retire_rat;

   logic [`LG_FCR_PRF_ENTRIES-1:0] r_fcr_alloc_rat;
   logic [`LG_FCR_PRF_ENTRIES-1:0] n_fcr_alloc_rat;
   logic [`LG_FCR_PRF_ENTRIES-1:0] r_fcr_retire_rat;
   logic [`LG_FCR_PRF_ENTRIES-1:0] n_fcr_retire_rat;
   
   logic [N_ROB_ENTRIES-1:0] 	    uq_wait, mq_wait, fq_wait;
   
   logic 		     t_rob_empty, t_rob_full, t_rob_next_full, t_rob_next_empty;
   logic 		     t_alloc, t_alloc_two, t_retire, t_retire_two,
			     t_rat_copy, t_clr_rob;

   logic 		     t_possible_to_alloc;
   
   
   logic 		     t_fold_uop, t_fold_uop2;
   
   logic 		     n_in_delay_slot, r_in_delay_slot;
   
   logic 		     t_clr_dq;
   logic 		     t_enough_iprfs, t_enough_hlprfs, t_enough_fprfs, 
			     t_enough_fcrprfs;

   logic 		     t_enough_next_iprfs, t_enough_next_hlprfs, t_enough_next_fprfs, 
			     t_enough_next_fcrprfs;

   
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
   logic 		     n_has_delay_slot, r_has_delay_slot;
   logic 		     n_has_nullifying_delay_slot,r_has_nullifying_delay_slot;
   logic 		     n_take_br, r_take_br;

   logic 		     n_got_break, r_got_break;
   logic 		     n_got_ud, r_got_ud;

   logic 		     n_l1i_flush_complete, r_l1i_flush_complete;
   logic 		     n_l1d_flush_complete, r_l1d_flush_complete;
   
   logic [(`M_WIDTH-1):0]    t_cpr0_status_reg;
   
   logic [31:0] 	     r_arch_a0;

   logic [4:0] 		     n_cause, r_cause;
   
   
   complete_t t_complete_bundle_1;
   logic 		     t_complete_valid_1;
   complete_t t_complete_bundle_2;
   logic 		     t_complete_valid_2;
   
   logic 		     t_any_complete;
   

   logic 		     t_free_reg;
   logic [`LG_PRF_ENTRIES-1:0] t_free_reg_ptr;
   
   logic 		       t_free_reg_two;
   logic [`LG_PRF_ENTRIES-1:0] t_free_reg_two_ptr;

   logic 		     t_free_fp_reg;
   logic [`LG_PRF_ENTRIES-1:0] t_free_fp_reg_ptr;

   logic 		     t_free_fp_two_reg;
   logic [`LG_PRF_ENTRIES-1:0] t_free_fp_reg_two_ptr;
   
   logic 			   t_free_hilo;
   logic [`LG_HILO_PRF_ENTRIES-1:0] t_free_hilo_ptr;
   
   logic 			    t_free_fcr;
   logic [`LG_FCR_PRF_ENTRIES-1:0]  t_free_fcr_ptr;
   

   logic [`LG_HILO_PRF_ENTRIES:0]   t_hilo_ffs;
   logic [`LG_FCR_PRF_ENTRIES:0]    t_fcr_ffs;
   logic [`LG_PRF_ENTRIES:0] 	    t_fp_ffs;
   logic [`LG_PRF_ENTRIES:0] 	    t_fp_ffs2;
   logic [`LG_PRF_ENTRIES:0] 	    t_gpr_ffs;
   logic [`LG_PRF_ENTRIES:0] 	    t_gpr_ffs2;
   
   logic 		     t_uq_full, t_uq_empty, t_uq_next_full;
   
   logic 		     t_uq_read;
   logic 		     n_ready_for_resume, r_ready_for_resume;
   
   logic 		     t_exception_wr_cpr0_val;
   logic [4:0] 		     t_exception_wr_cpr0_ptr;
   logic [63:0] 	     t_exception_wr_cpr0_data;
   
   mem_req_t t_mem_req;
   logic 		     t_mem_req_valid;
   logic 		     t_monitor_req_valid;
   logic [(`M_WIDTH-1):0]    r_monitor_rsp_data, n_monitor_rsp_data;

   logic 		     n_machine_clr, r_machine_clr;
   logic 		     n_flush_req, r_flush_req;
   logic 		     n_flush_cl_req, r_flush_cl_req;
   logic [(`M_WIDTH-1):0]    n_flush_cl_addr, r_flush_cl_addr;
   logic 		     r_ds_done, n_ds_done;
   
   logic 		     t_can_retire_rob_head;
   
   logic [`LG_ROB_ENTRIES-1:0] n_delayslot_rob_ptr, r_delayslot_rob_ptr;
   
   typedef enum logic [4:0] {
			     HALT,
			     ACTIVE,
			     DRAIN,
			     RAT,
			     DELAY_SLOT,
			     ALLOC_FOR_SERIALIZE,
			     MONITOR_FLUSH_CACHE,
			     HANDLE_MONITOR,
			     ALLOC_FOR_MONITOR,
			     WAIT_FOR_MONITOR,
			     FLUSH_FOR_HALT,
			     HALT_WAIT_FOR_RESTART,
			     WAIT_FOR_SERIALIZE_AND_RESTART,
			     WRITE_EPC,
			     WRITE_CAUSE,
			     WRITE_BADVADDR,
			     EXCEPTION_DRAIN
			     } state_t;
   
   state_t r_state, n_state;
   logic [31:0] r_restart_cycles, n_restart_cycles;
   logic t_divide_ready;
   
   
   always_comb
     begin
	core_mem_req_valid = t_mem_req_valid;
	core_mem_req = t_mem_req;
     end // always_comb
   

   assign ready_for_resume = r_ready_for_resume;
   assign head_of_rob_ptr_valid = (r_state == ACTIVE) || (r_state==DRAIN) && !r_ds_done;
   assign head_of_rob_ptr = r_rob_head_ptr[`LG_ROB_ENTRIES-1:0];
   assign flush_req = r_flush_req;
   assign flush_cl_req = r_flush_cl_req;
   assign flush_cl_addr = r_flush_cl_addr;
   

   assign monitor_req_reason = r_monitor_reason;
   assign monitor_req_valid = t_monitor_req_valid;
   
   assign got_break = r_got_break;
   assign got_ud = r_got_ud;
   

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
	     r_flush_req <= 1'b0;
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
	     r_in_delay_slot <= 1'b0;
	     r_restart_valid <= 1'b0;
	     r_has_delay_slot <= 1'b0;
	     r_has_nullifying_delay_slot <= 1'b0;
	     r_take_br <= 1'b0;
	     r_monitor_rsp_data <= 'd0;
	     r_got_break <= 1'b0;
	     r_got_ud <= 1'b0;
	     r_ready_for_resume <= 1'b0;
	     r_l1i_flush_complete <= 1'b0;
	     r_l1d_flush_complete <= 1'b0;
	     r_ds_done <= 1'b0;
	     drain_ds_complete <= 1'b0;
	  end
	else
	  begin
	     r_flush_req <= n_flush_req;
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
	     r_in_delay_slot <= n_in_delay_slot;
	     r_restart_valid <= n_restart_valid;
	     r_has_delay_slot <= n_has_delay_slot;
	     r_has_nullifying_delay_slot <= n_has_nullifying_delay_slot;
	     r_take_br <= n_take_br;
	     r_monitor_rsp_data <= n_monitor_rsp_data;
	     r_got_break <= n_got_break;
	     r_got_ud <= n_got_ud;
	     r_ready_for_resume <= n_ready_for_resume;
	     r_l1i_flush_complete <= n_l1i_flush_complete;
	     r_l1d_flush_complete <= n_l1d_flush_complete;
	     r_ds_done <= n_ds_done;
	     drain_ds_complete <= r_ds_done;	     
	  end
     end // always_ff@ (posedge clk)

   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= HALT;
	     r_restart_cycles <= 'd0;
	     r_machine_clr <= 1'b0;
	     r_delayslot_rob_ptr <= 'd0;
	     r_got_restart_ack <= 1'b0;
	     r_cause <= 5'd0;
	  end
	else
	  begin
	     r_state <= n_state;
	     r_restart_cycles <= n_restart_cycles;
	     r_machine_clr <= n_machine_clr;
	     r_delayslot_rob_ptr <= n_delayslot_rob_ptr;
	     r_got_restart_ack <= n_got_restart_ack;
	     r_cause <= n_cause;
	  end
     end

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_arch_a0 <= 'd0;
	  end
	else if(t_rob_head.valid_dst && t_retire && t_rob_head.ldst == 'd4)
	  begin
	     r_arch_a0 <= t_rob_head.data[31:0];
	  end
     end
   
   always_ff@(posedge clk)
     begin
   	if(reset)
   	  begin
   	     retire_reg_ptr <= 'd0;
   	     retire_reg_data <= 'd0;
   	     retire_reg_valid <= 1'b0;
	     retire_reg_fp_valid <= 1'b0;
   	     retire_reg_two_ptr <= 'd0;
   	     retire_reg_two_data <= 'd0;
   	     retire_reg_two_valid <= 1'b0;
   	     retire_valid <= 1'b0;
	     retire_two_valid <= 1'b0;
	     
   	     retire_pc <= 'd0;
	     retire_two_pc <= 'd0;
	     retire_delay_slot <= 1'b0;
	     retired_call <= 1'b0;
	     retired_ret <= 1'b0;
	     retired_rob_ptr_valid <= 1'b0;
	     
	     retired_rob_ptr_two_valid <= 1'b0;
	     retired_rob_ptr <= 'd0;
	     retired_rob_ptr_two <= 'd0;
	     r_monitor_reason <= 16'd0;
   	  end
   	else
   	  begin
   	     retire_reg_ptr <= t_rob_head.ldst;
   	     retire_reg_data <= t_rob_head.data;
   	     retire_reg_valid <= t_rob_head.valid_dst && t_retire;
	     retire_reg_fp_valid <= t_rob_head.valid_fp_dst && t_retire;
   	     retire_reg_two_ptr <= t_rob_next_head.ldst;
   	     retire_reg_two_data <= t_rob_next_head.data;
   	     retire_reg_two_valid <= t_rob_next_head.valid_dst && t_retire_two;
	     retire_reg_fp_two_valid <= t_rob_next_head.valid_fp_dst && t_retire_two;
	     
   	     retire_valid <= t_retire;
	     retire_two_valid <= t_retire_two;
   	     retire_pc <= t_rob_head.pc;
	     retire_two_pc <= t_rob_next_head.pc;
	     retire_delay_slot <= t_rob_head.in_delay_slot && t_retire;
	     retired_ret <= t_rob_head.is_ret && t_retire;
	     retired_call <= t_rob_head.is_call && t_retire;

	     retired_rob_ptr_valid <= t_retire;
	     
	     retired_rob_ptr_two_valid <= t_retire_two;
	     retired_rob_ptr <= r_rob_head_ptr[`LG_ROB_ENTRIES-1:0];
	     retired_rob_ptr_two <= r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0];
	     r_monitor_reason <= n_monitor_reason;
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
	     record_retirement({32'd0,t_rob_head.pc}, 
   			       t_rob_head.fetch_cycle,
   			       t_rob_head.alloc_cycle,
   			       t_rob_head.complete_cycle,
   			       r_cycle,
			       t_rob_head.faulted ? 32'd1 : 32'd0,
			       t_rob_head.is_mem ? 32'd1 : 32'd0,
			       t_rob_head.is_fp ? 32'd1 : 32'd0,
			       t_rob_head.missed_l1d ? 32'd1 : 32'd0);
   	  end
   	if(t_retire_two)
   	  begin
	     record_retirement({32'd0, t_rob_next_head.pc}, 
   			       t_rob_next_head.fetch_cycle,
   			       t_rob_next_head.alloc_cycle,
   			       t_rob_next_head.complete_cycle,
   			       r_cycle,
			       t_rob_next_head.faulted ? 32'd1 : 32'd0,
			       t_rob_next_head.is_mem ? 32'd1 : 32'd0,
			       t_rob_next_head.is_fp ? 32'd1 : 32'd0,
			       t_rob_next_head.missed_l1d ? 32'd1 : 32'd0);	     
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

//`define DUMP_ROB
`ifdef DUMP_ROB
   always_ff@(negedge clk)
     begin
	if(r_cycle >= 'd520401)
	  begin
	     $display("cycle %d : state = %d, alu complete %b, mem complete %b,head_ptr %d, inflight %d, complete %b,  can_retire_rob_head %b, head pc %x, empty %b, full %b", 
		      r_cycle,
		      r_state,
		      t_complete_valid_1,
		      core_mem_rsp_valid,
		      r_rob_head_ptr,
		      r_rob_inflight,
		      t_rob_head_complete && !t_rob_empty, 
		      t_can_retire_rob_head,
		      t_rob_head.pc,
		      t_rob_empty, 
		      t_rob_full);
	     
	     for(logic [`LG_ROB_ENTRIES:0] i = r_rob_head_ptr; i != (r_rob_tail_ptr); i=i+1)
	       begin
		  $display("\trob entry %d, pc %x, complete %b, faulted %b",
			   i[`LG_ROB_ENTRIES-1:0], 
			   r_rob[i[`LG_ROB_ENTRIES-1:0]].pc, 
			   r_rob_complete[i[`LG_ROB_ENTRIES-1:0]],
			   r_rob[i[`LG_ROB_ENTRIES-1:0]].faulted);
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
	t_clr_extern_irq = 1'b0;
	t_restart_complete = 1'b0;
	
	t_exception_wr_cpr0_val = 1'b0;
	t_exception_wr_cpr0_ptr = 5'd0;
	t_exception_wr_cpr0_data = 'd0;
	n_cause = r_cause;
	
	n_machine_clr = r_machine_clr;
	n_delayslot_rob_ptr = r_delayslot_rob_ptr;
	t_alloc = 1'b0;
	t_alloc_two = 1'b0;
	t_possible_to_alloc = 1'b0;

	
	n_in_delay_slot = r_in_delay_slot;
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
	n_has_delay_slot = r_has_delay_slot;
	n_has_nullifying_delay_slot = r_has_nullifying_delay_slot;
	n_take_br = r_take_br;	
	t_bump_rob_head = 1'b0;
	t_monitor_req_valid = 1'b0;
	n_monitor_rsp_data = r_monitor_rsp_data;
	
	t_enough_iprfs = !((t_uop.dst_valid) && (r_prf_free == 'd0));
	t_enough_hlprfs = !((t_uop.hilo_dst_valid) && (r_hilo_prf_free == 'd0));
	t_enough_fprfs = !((t_uop.fp_dst_valid) && (r_fp_prf_free == 'd0));
	t_enough_fcrprfs = !((t_uop.fcr_dst_valid) && (r_fcr_prf_free == 'd0));

	t_enough_next_iprfs = !((t_uop2.dst_valid) && (t_prf_free_cnt == 'd1));
	t_enough_next_hlprfs = !((t_uop2.hilo_dst_valid) /*&& (r_hilo_prf_free == 'd0)*/);
	t_enough_next_fprfs = !((t_uop2.fp_dst_valid) && (t_fp_prf_free_cnt == 'd1));
	t_enough_next_fcrprfs = !((t_uop2.fcr_dst_valid) /*&& (r_fcr_prf_free == 'd0)*/);


	
	t_fold_uop = (t_uop.op == NOP || 
		      t_uop.op == J  ||
		      t_uop.op == II);

	t_fold_uop2 = (t_uop2.op == NOP || 
		       t_uop2.op == J  ||
		       t_uop2.op == II);
	
	n_ds_done = r_ds_done;
	n_flush_req = 1'b0;
	n_flush_cl_req = 1'b0;
	n_flush_cl_addr = r_flush_cl_addr;
	n_got_break = r_got_break;
	n_got_ud = r_got_ud;
	n_monitor_reason = r_monitor_reason;
	n_got_restart_ack = r_got_restart_ack;
	n_ready_for_resume = 1'b0;
	n_l1i_flush_complete = r_l1i_flush_complete || l1i_flush_complete;
	n_l1d_flush_complete = r_l1d_flush_complete || l1d_flush_complete;
	
	
	if(r_state == ACTIVE)
	  begin
	     n_got_restart_ack = 1'b0;
	  end
	else if(!r_got_restart_ack)
	  begin
	     n_got_restart_ack = restart_ack;
	  end	
	
	t_can_retire_rob_head = 1'b0;
	
	if(t_rob_head_complete && !t_rob_empty)
	  begin
	     t_can_retire_rob_head = (t_rob_head.has_delay_slot || t_rob_head.has_nullifying_delay_slot) && t_rob_head.faulted ? !t_rob_next_empty : 1'b1;
	  end

	       
	unique case (r_state)
	  ACTIVE:
	    begin
	       if(r_extern_irq && !t_rob_empty && !t_rob_head.in_delay_slot)
		 begin
		    n_state = EXCEPTION_DRAIN;
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
			 if(t_rob_head.is_break)
			   begin
			      n_got_break = 1'b1;
			      n_flush_req = 1'b1;
			      n_cause = 5'd9;
			      n_state = WRITE_EPC;
			   end
			 else if(t_rob_head.is_ii)
			   begin
			      n_got_ud = 1'b1;
			      n_flush_req = 1'b1;
			      n_cause = 5'd10;
			      n_state = WRITE_EPC;
			   end
			 else
			   begin
			      n_ds_done = !t_rob_head.has_delay_slot;
			      n_state = DRAIN;
			      n_restart_cycles = 'd1;
			      n_restart_valid = 1'b1;
			   end // else: !if(t_rob_head.is_ii)
			 n_machine_clr = 1'b1;
			 n_delayslot_rob_ptr = r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0];
			 n_restart_pc = t_rob_head.target_pc;
			 n_restart_src_pc = t_rob_head.pc;
			 n_restart_src_is_indirect = t_rob_head.is_indirect && !t_rob_head.is_ret;
			 
			 n_has_delay_slot = t_rob_head.has_delay_slot;
			 n_has_nullifying_delay_slot = t_rob_head.has_nullifying_delay_slot;
			 n_take_br = t_rob_head.take_br;
			 t_bump_rob_head = 1'b1;
		      end // if (t_rob_head.faulted)
		    else if(!t_dq_empty)
		      begin
			 if(t_uop.serializing_op)
			   begin
			      if(/*r_inflight*/t_rob_empty)
				begin
				   n_state = (t_uop.op == MONITOR) ? 
					     HANDLE_MONITOR : ALLOC_FOR_SERIALIZE;
				   n_monitor_reason = t_uop.imm;
				end
			   end
			 else
			   begin
			      t_possible_to_alloc = !t_rob_full
						    && !t_uq_full
						    && !t_dq_empty;

			      t_alloc = !t_rob_full
					&& !t_uq_full
					&& !t_dq_empty					
					&& t_enough_iprfs
					&& t_enough_hlprfs
					&& t_enough_fprfs
					&& t_enough_fcrprfs;
			      
					
			      
			      t_alloc_two = t_alloc
					    && !t_uop2.serializing_op
					    && !t_dq_next_empty 
					    && !t_rob_next_full
					    && !t_uq_next_full
					    && t_enough_next_iprfs
					    && t_enough_next_hlprfs
					    && t_enough_next_fprfs
					    && t_enough_next_fcrprfs
					    ;
			      //&& (t_uop2.op == NOP || t_uop2.op == J);
			   end // else: !if(t_uop.serializing_op && !t_dq_empty)
		      end // if (!t_dq_empty)
		    t_retire = t_rob_head_complete;
		    t_retire_two = !t_rob_next_empty
		    		   && !t_rob_head.faulted
		    		   && !t_rob_next_head.faulted 				    
		    		   && t_rob_head_complete
		    		   && t_rob_next_head_complete				    
				   && !t_rob_head.is_br
				   && !t_rob_next_head.is_ret
				   && !t_rob_next_head.is_call
				   && !t_rob_next_head.valid_fcr_dst
		    		   && !t_rob_next_head.valid_hilo_dst;
		 end // if (t_can_retire_rob_head)
	       else if(!t_dq_empty)
		 begin
		    if(t_uop.serializing_op && t_rob_empty)
		      begin
			 if(t_uop.op == MONITOR)
			   begin
			      n_monitor_reason = t_uop.imm;
			      case(t_uop.imm)
				'd50: /* get cycle */
				  begin
				     n_state = HANDLE_MONITOR;
				  end
				'd52: /* flush line in data cache */
				  begin
				     n_state = MONITOR_FLUSH_CACHE;
				     n_l1i_flush_complete = 1'b1;
				     n_flush_cl_addr = r_arch_a0;
				     n_flush_cl_req = 1'b1;
				  end
				'd53: /* get icnt */
				  begin
				     n_state = HANDLE_MONITOR;
				  end
				default:
				  begin
				     n_flush_req = 1'b1;
				     n_state = MONITOR_FLUSH_CACHE;
				  end
			      endcase // case (t_uop.imm)
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
				   && t_enough_hlprfs
				   && t_enough_fprfs
				   && t_enough_fcrprfs;

			 t_alloc_two = t_alloc
				       && !t_uop2.serializing_op
				       && !t_dq_next_empty 
				       && !t_rob_next_full
				       && !t_uq_next_full
				       && t_enough_next_iprfs
				       && t_enough_next_hlprfs
				       && t_enough_next_fprfs
				       && t_enough_next_fcrprfs
				       ;
			 //&& (t_uop2.op == NOP || t_uop2.op == J);
		      end
		 end
	    end // case: ACTIVE
	  DRAIN:	    
	    begin
	       //$display("cycle %d : r_rob_inflight = %b, r_ds_done = %b, t_rob_head_complete = %b", 
	       //r_cycle, r_rob_inflight, r_ds_done, t_rob_head_complete);
	       //$display("inflight is_mem %b", r_rob['d5].is_mem);

	       
	       if(r_has_nullifying_delay_slot && t_rob_head_complete && !r_ds_done)
		 begin
		    t_retire = r_take_br;
		    n_ds_done = 1'b1;
		 end
	       else if(r_has_delay_slot && t_rob_head_complete && !r_ds_done)
		 begin
		    t_retire = 1'b1;
		    n_ds_done = 1'b1;
		 end
	       if(r_rob_inflight == 'd0 && r_ds_done && memq_empty && t_divide_ready)
		 begin
		    //$display("%d : wait for drain and memq_empty  took  %d cycles",r_cycle, r_restart_cycles);		    
		    n_state = RAT;
`ifdef REPORT_FAULTS		    
		    $display("restarting after fault at cycle %d", r_cycle);
`endif
		 end // if (r_rob_inflight == 'd0 && r_ds_done && memq_empty)


	    end // case: DRAIN
	  EXCEPTION_DRAIN:
	    begin
	       if(r_rob_inflight == 'd0 && memq_empty && t_divide_ready)
		 begin
		    n_state = RAT;
		 end
	    end
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
		    if(n_got_restart_ack)
		      begin
			 //$display("RESTART PIPELINE AT %d, pc %x", 
			 //r_cycle, n_restart_pc);
			 n_state = ACTIVE;
		      end
		 end
	    end
	  MONITOR_FLUSH_CACHE:
	    begin
	       //$display("%d : %b %b", r_cycle, n_l1i_flush_complete, n_l1d_flush_complete);
	       if(n_l1i_flush_complete && n_l1d_flush_complete)
		 begin
		    n_state = HANDLE_MONITOR;
		    n_l1i_flush_complete = 1'b0;
		    n_l1d_flush_complete = 1'b0;
		 end
	    end
	  HANDLE_MONITOR:
	    begin
	       t_monitor_req_valid = 1'b1;
	       if(monitor_rsp_valid)
		 begin
		    n_state = ALLOC_FOR_MONITOR;
		    n_monitor_rsp_data = monitor_rsp_data;
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
		    n_restart_pc = t_rob_head.target_pc;
		    n_restart_src_pc = t_rob_head.pc;
		    n_restart_src_is_indirect = 1'b0;
		    n_restart_valid = 1'b1;
		    if(n_got_restart_ack)
		      begin
			 t_retire = 1'b1;
			 n_state = ACTIVE;
		      end
		 end
	    end // case: WAIT_FOR_MONITOR
	  FLUSH_FOR_HALT:
	    begin
	       if(n_l1i_flush_complete && n_l1d_flush_complete)
		 begin
		    n_state = HALT;
		    n_ready_for_resume = 1'b1;		    		    
		    n_l1i_flush_complete = 1'b0;
		    n_l1d_flush_complete = 1'b0;
		 end	       
	    end
	  HALT:
	    begin
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
	       if(n_got_restart_ack)
		 begin
		    n_state = ACTIVE;
		 end
	    end
	  WRITE_EPC:
	    begin
	       t_exception_wr_cpr0_val = 1'b1;
	       t_exception_wr_cpr0_ptr = 5'd14;
	       t_exception_wr_cpr0_data = {32'd0, (t_rob_head.in_delay_slot ? (t_rob_head.pc - 'd4) : t_rob_head.pc)};
	       n_state = WRITE_CAUSE;
	    end
	  WRITE_CAUSE:
	    begin
	       t_exception_wr_cpr0_val = 1'b1;
	       t_exception_wr_cpr0_ptr = 5'd13;
	       t_exception_wr_cpr0_data = {32'd0, t_rob_head.in_delay_slot, 15'd0, 8'd0, 1'b0, r_cause, 2'b0};
	       n_state = FLUSH_FOR_HALT; 	       
	    end
	  WRITE_BADVADDR:
	    begin
	       t_exception_wr_cpr0_val = 1'b1;
	       t_exception_wr_cpr0_ptr = 5'd8;
	       t_exception_wr_cpr0_data = t_rob_head.data;
	       n_state = WRITE_EPC;
	    end
	  default:
	    begin
	    end
	endcase // unique case (r_state)

	if(t_alloc)
	  begin
	     n_in_delay_slot = t_alloc_two ? t_uop2.has_delay_slot
			       : t_uop.has_delay_slot;
	  end
	
	else if(t_clr_dq || t_clr_rob)
	  begin
	     n_in_delay_slot = 1'b0;
	  end
	
     end // always_comb
   
      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_rob_head_ptr <= 'd0;
	     r_rob_tail_ptr <= 'd0;
	     r_rob_next_head_ptr <= 'd1;
	     r_rob_next_tail_ptr <= 'd1;
	  end
	else
	  begin
	     r_rob_head_ptr <= n_rob_head_ptr;
	     r_rob_tail_ptr <= n_rob_tail_ptr;
	     r_rob_next_head_ptr <= n_rob_next_head_ptr;
	     r_rob_next_tail_ptr <= n_rob_next_tail_ptr;
	  end
     end // always_ff@ (posedge clk)


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_hilo_alloc_rat <= 'd0;
	     r_hilo_retire_rat <= 'd0;
	     r_fcr_alloc_rat <= 'd0;
	     r_fcr_retire_rat <= 'd0;	     
	  end
	else
	  begin
	     r_hilo_alloc_rat <= t_rat_copy ? r_hilo_retire_rat : n_hilo_alloc_rat;
	     r_hilo_retire_rat <= n_hilo_retire_rat;
	     r_fcr_alloc_rat <= t_rat_copy ? r_fcr_retire_rat : n_fcr_alloc_rat;
	     r_fcr_retire_rat <= n_fcr_retire_rat;
	  end
     end

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(logic [`LG_PRF_ENTRIES-1:0] i_rat = 'd0; i_rat < 'd32; i_rat = i_rat + 'd1)
	       begin
		  r_alloc_rat[i_rat[4:0]] <= i_rat;
		  r_retire_rat[i_rat[4:0]] <= i_rat;
		  r_fp_alloc_rat[i_rat[4:0]] <= i_rat;
		  r_fp_retire_rat[i_rat[4:0]] <= i_rat;
	       end
	  end
	else
	  begin
	     r_alloc_rat <= t_rat_copy ? r_retire_rat : n_alloc_rat;
	     r_retire_rat <= n_retire_rat;
	     r_fp_alloc_rat <= t_rat_copy ? r_fp_retire_rat : n_fp_alloc_rat;
	     r_fp_retire_rat <= n_fp_retire_rat;
	  end
     end // always_ff@ (posedge clk)

   // always_ff@(negedge clk)
   //   begin
   // 	if(t_alloc) $display("alloc1 %x at cycle %d of type %d, monitor=%b", t_uop.pc, r_cycle, t_uop.op, t_uop.op == MONITOR);
   // 	if(t_alloc_two) $display("alloc2 %x at cycle %d of type %d, monitor=%b", t_uop2.pc, r_cycle, t_uop2.op, t_uop2.op == MONITOR);
   //   end
   // 	if(n_state == ACTIVE && r_state == RAT)
   // 	  begin
   // 	     $display("RESTART COMPLETE at cycle %d", r_cycle);
   // 	  end
   // 	if(t_uop.pc == 'h20d2c && t_uop.srcA_valid)
   // 	  begin
   // 	     $display("at %d, pc %x SLL with lsrcA = %d, psrcA = %d, uuid = %d", 
   // 		      r_cycle, t_alloc_uop.pc, t_uop.srcA, t_alloc_uop.srcA, t_alloc_uop.fetch_cycle);
   // 	  end
   // 	if(t_uop2.pc == 'h20d2c && t_uop2.srcA_valid)
   // 	  begin
   // 	     $display("at %d, pc %x SLL with lsrcA = %d, psrcA = %d, uuid = %d", 
   // 		      r_cycle, t_alloc_uop2.pc, t_uop2.srcA, t_alloc_uop2.srcA, t_alloc_uop.fetch_cycle);
   // 	  end
   // 	if(t_uop.pc == 'h20d28 && t_uop.srcA_valid)
   // 	  begin
   // 	     $display("at %d, pc %x dst %d with lsrcA = %d, psrcA = %d, uuid = %d", 
   // 		      r_cycle, t_alloc_uop.pc, t_alloc_uop.dst, t_uop.srcA, t_alloc_uop.srcA, t_alloc_uop.fetch_cycle);
   // 	  end
   // 	if(t_uop2.pc == 'h20d28 && t_uop2.srcA_valid)
   // 	  begin
   // 	     $display("at %d, pc %x dst %d with lsrcA = %d, psrcA = %d, uuid = %d", 
   // 		      r_cycle, t_alloc_uop2.pc, t_alloc_uop2.dst, t_uop2.srcA, t_alloc_uop2.srcA, t_alloc_uop.fetch_cycle);
   // 	  end
	
   //   end

   
   always_comb
     begin
	n_alloc_rat = r_alloc_rat;
	n_hilo_alloc_rat = r_hilo_alloc_rat;
	n_fp_alloc_rat = r_fp_alloc_rat;
	n_fcr_alloc_rat = r_fcr_alloc_rat;
	
	t_alloc_uop = t_uop;
	t_alloc_uop2 = t_uop2;
`ifdef VERILATOR
	t_alloc_uop.clear_id = r_clear_cnt;
	t_alloc_uop2.clear_id = r_clear_cnt;
`endif
	
	if(t_uop.srcA_valid || t_uop.fp_srcA_valid)
	  begin
	     t_alloc_uop.srcA = t_uop.fp_srcA_valid ?
				r_fp_alloc_rat[t_uop.srcA[4:0]] :
				r_alloc_rat[t_uop.srcA[4:0]];	     
	  end
	if(t_uop.srcB_valid || t_uop.fp_srcB_valid)
	  begin
	     t_alloc_uop.srcB = t_uop.fp_srcB_valid ?
				r_fp_alloc_rat[t_uop.srcB[4:0]] :
				r_alloc_rat[t_uop.srcB[4:0]];
	  end
	if(t_uop.fp_srcC_valid)
	  begin
	     t_alloc_uop.srcC = r_fp_alloc_rat[t_uop.srcC[4:0]];
	  end
	     
	if(t_uop.hilo_src_valid)
	  begin
	     t_alloc_uop.hilo_src = r_hilo_alloc_rat;
	  end
	else if(t_uop.fcr_src_valid)
	  begin
	     t_alloc_uop.hilo_src = r_fcr_alloc_rat;
	  end
	
	//2nd uop begins here
	if(t_uop2.srcA_valid || t_uop2.fp_srcA_valid)
	  begin
	     t_alloc_uop2.srcA = t_uop2.fp_srcA_valid ?
				 (t_uop.fp_dst_valid && (t_uop2.srcA[4:0] == t_uop.dst[4:0]) ?
				  n_fp_prf_entry : r_fp_alloc_rat[t_uop2.srcA[4:0]]) :
				 (t_uop.dst_valid && (t_uop2.srcA[4:0] == t_uop.dst[4:0]) ?
				  n_prf_entry :  r_alloc_rat[t_uop2.srcA[4:0]]);	     
	  end
	if(t_uop2.srcB_valid || t_uop2.fp_srcB_valid)
	  begin
	     t_alloc_uop2.srcB = t_uop2.fp_srcB_valid ?
				 (t_uop.fp_dst_valid && (t_uop2.srcB[4:0] == t_uop.dst[4:0]) ?
				  n_fp_prf_entry : r_fp_alloc_rat[t_uop2.srcB[4:0]]) :
				 (t_uop.dst_valid && (t_uop2.srcB[4:0] == t_uop.dst[4:0]) ?
				  n_prf_entry :  r_alloc_rat[t_uop2.srcB[4:0]]);	     	     
	  end
	if(t_uop2.fp_srcC_valid)
	  begin
	     t_alloc_uop2.srcC = (t_uop2.srcC[4:0] == t_uop.dst[4:0]) ?
				 n_fp_prf_entry : r_fp_alloc_rat[t_uop2.srcC[4:0]];
	  end
	
	if(t_uop2.hilo_src_valid)
	  begin
	     t_alloc_uop2.hilo_src = t_uop.hilo_dst_valid ? n_hilo_prf_entry : 
				     r_hilo_alloc_rat;
	  end
	else if(t_uop2.fcr_src_valid)
	  begin
	     t_alloc_uop2.hilo_src = t_uop.fcr_dst_valid ? n_fcr_prf_entry :
				     r_fcr_alloc_rat;
	  end
	

	if(t_alloc)
	  begin
	     if(t_uop.dst_valid)
	       begin
		  n_alloc_rat[t_uop.dst[4:0]] = n_prf_entry;
		  t_alloc_uop.dst = n_prf_entry;
	       end
	     else if(t_uop.hilo_dst_valid)
	       begin
		  n_hilo_alloc_rat = n_hilo_prf_entry;
		  t_alloc_uop.hilo_dst = n_hilo_prf_entry;
	       end
	     else if(t_uop.fcr_dst_valid)
	       begin
		  n_fcr_alloc_rat = n_fcr_prf_entry;
		  t_alloc_uop.hilo_dst = n_fcr_prf_entry;
	       end	     
	     else if(t_uop.fp_dst_valid)
	       begin
		  n_fp_alloc_rat[t_uop.dst[4:0]] = n_fp_prf_entry;
		  t_alloc_uop.dst = n_fp_prf_entry;
	       end
	     
	     t_alloc_uop.rob_ptr = r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0];
	  end // if (t_alloc)

	if(t_alloc_two)
	  begin
	     if(t_uop2.dst_valid)
	       begin
		  n_alloc_rat[t_uop2.dst[4:0]] = n_prf_entry2;
		  t_alloc_uop2.dst = n_prf_entry2;
	       end
	     else if(t_uop2.fp_dst_valid)
	       begin
		  n_fp_alloc_rat[t_uop2.dst[4:0]] = n_fp_prf_entry2;
		  t_alloc_uop2.dst = n_fp_prf_entry2;
	       end
	     
	     t_alloc_uop2.rob_ptr = r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0];
	  end
	
     end // always_comb


   //always_ff@(negedge clk)
   //begin
   //$display("r_cycle %d : $v1 -> %d", r_cycle, n_alloc_rat['d3]);
   //end
  
   always_comb
     begin
	n_retire_rat = r_retire_rat;
	n_hilo_retire_rat = r_hilo_retire_rat;
	n_fp_retire_rat = r_fp_retire_rat;
	n_fcr_retire_rat = r_fcr_retire_rat;
	
	t_free_reg = 1'b0;
	t_free_reg_ptr = 'd0;
	t_free_reg_two = 1'b0;
	t_free_reg_two_ptr = 'd0;
	t_free_fp_reg = 1'b0;
	t_free_fp_reg_ptr = 'd0;
	t_free_fp_two_reg = 1'b0;
	t_free_fp_reg_two_ptr = 'd0;
	
	
	t_free_hilo = 1'b0;
	t_free_hilo_ptr = 'd0;
	t_free_fcr = 1'b0;
	t_free_fcr_ptr = 'd0;
	
	n_retire_prf_free = r_retire_prf_free;
	n_retire_hilo_prf_free = r_retire_hilo_prf_free;
	n_retire_fcr_prf_free = r_retire_fcr_prf_free;
	n_retire_fp_prf_free = r_retire_fp_prf_free;
	
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
		  n_retire_rat[t_rob_head.ldst] = t_rob_head.pdst;
		  n_retire_prf_free[t_rob_head.pdst] = 1'b0;
		  n_retire_prf_free[t_rob_head.old_pdst] = 1'b1;
	       end
	     else if(t_rob_head.valid_hilo_dst)
	       begin
		  t_free_hilo = 1'b1;
		  t_free_hilo_ptr = t_rob_head.old_pdst[`LG_HILO_PRF_ENTRIES-1:0];
		  n_hilo_retire_rat = t_rob_head.pdst[`LG_HILO_PRF_ENTRIES-1:0];
		  n_retire_hilo_prf_free[t_rob_head.pdst[`LG_HILO_PRF_ENTRIES-1:0]] = 1'b0;
		  n_retire_hilo_prf_free[t_rob_head.old_pdst[`LG_HILO_PRF_ENTRIES-1:0]] = 1'b1;
	       end
	     else if(t_rob_head.valid_fcr_dst)
	       begin
		  t_free_fcr = 1'b1;
		  t_free_fcr_ptr = t_rob_head.old_pdst[`LG_FCR_PRF_ENTRIES-1:0];
		  n_fcr_retire_rat = t_rob_head.pdst[`LG_FCR_PRF_ENTRIES-1:0];
		  n_retire_fcr_prf_free[t_rob_head.pdst[`LG_FCR_PRF_ENTRIES-1:0]] = 1'b0;
		  n_retire_fcr_prf_free[t_rob_head.old_pdst[`LG_FCR_PRF_ENTRIES-1:0]] = 1'b1;
	       end
	     else if(t_rob_head.valid_fp_dst)
	       begin
		  t_free_fp_reg = 1'b1;
		  t_free_fp_reg_ptr = t_rob_head.old_pdst;
		  n_fp_retire_rat[t_rob_head.ldst] = t_rob_head.pdst;
		  n_retire_fp_prf_free[t_rob_head.pdst] = 1'b0;
		  n_retire_fp_prf_free[t_rob_head.old_pdst] = 1'b1;
	       end
	     if(t_retire_two && t_rob_next_head.valid_dst)
	       begin
		  t_free_reg_two = 1'b1;
		  t_free_reg_two_ptr = t_rob_next_head.old_pdst;
		  n_retire_rat[t_rob_next_head.ldst] = t_rob_next_head.pdst;
		  n_retire_prf_free[t_rob_next_head.pdst] = 1'b0;
		  n_retire_prf_free[t_rob_next_head.old_pdst] = 1'b1;
	       end
	     if(t_retire_two && t_rob_next_head.valid_fp_dst)
	       begin
		  t_free_fp_two_reg = 1'b1;
		  t_free_fp_reg_two_ptr = t_rob_next_head.old_pdst;
		  n_fp_retire_rat[t_rob_next_head.ldst] = t_rob_next_head.pdst;
		  n_retire_fp_prf_free[t_rob_next_head.pdst] = 1'b0;
		  n_retire_fp_prf_free[t_rob_next_head.old_pdst] = 1'b1;
	       end
	     
	     n_branch_pc = t_retire_two ? t_rob_next_head.pc : t_rob_head.pc;
	     n_took_branch = t_retire_two ? t_rob_next_head.take_br : t_rob_head.take_br;
	     n_branch_valid = t_retire_two ? t_rob_next_head.is_br :  t_rob_head.is_br;
	     n_branch_fault = t_rob_head.faulted;
	     n_branch_pht_idx = t_retire_two ? t_rob_next_head.pht_idx : t_rob_head.pht_idx;
	  end // if (t_retire)
	
     end // always_comb
   
   
   always_comb
     begin
	t_rob_tail.faulted  = 1'b0;
	t_rob_tail.valid_dst  = 1'b0;
	t_rob_tail.valid_hilo_dst = 1'b0;
	t_rob_tail.valid_fcr_dst = 1'b0;
	t_rob_tail.valid_fp_dst = 1'b0;
	t_rob_tail.ldst  = 'd0;
	t_rob_tail.pdst  = 'd0;
	t_rob_tail.old_pdst  = 'd0;
	t_rob_tail.pc = 'd0;
	t_rob_tail.target_pc = 'd0;
	t_rob_tail.is_ii = 1'b0;
	t_rob_tail.is_call = 1'b0;
	t_rob_tail.is_ret = 1'b0;
	t_rob_tail.is_break = 1'b0;
	t_rob_tail.take_br = 1'b0;
	t_rob_tail.is_br = 1'b0;
	t_rob_tail.is_indirect = 1'b0;
	t_rob_tail.in_delay_slot = r_in_delay_slot;
	t_rob_tail.data = 'd0;
	t_rob_tail.pht_idx = 'd0;


	t_rob_next_tail.faulted  = 1'b0;
	t_rob_next_tail.valid_dst  = 1'b0;
	t_rob_next_tail.valid_hilo_dst = 1'b0;
	t_rob_next_tail.valid_fcr_dst = 1'b0;
	t_rob_next_tail.valid_fp_dst = 1'b0;
	t_rob_next_tail.ldst  = 'd0;
	t_rob_next_tail.pdst  = 'd0;
	t_rob_next_tail.old_pdst  = 'd0;
	t_rob_next_tail.pc = 'd0;
	t_rob_next_tail.target_pc = 'd0;
	t_rob_next_tail.is_ii = 1'b0;
	t_rob_next_tail.is_call = 1'b0;
	t_rob_next_tail.is_ret = 1'b0;
	t_rob_next_tail.is_break = 1'b0;
	t_rob_next_tail.take_br = 1'b0;
	t_rob_next_tail.is_br = 1'b0;
	t_rob_next_tail.is_indirect = 1'b0;
	t_rob_next_tail.in_delay_slot = r_in_delay_slot;
	t_rob_next_tail.data = 'd0;
	t_rob_next_tail.pht_idx = 'd0;
	
	if(t_alloc)
	  begin
	     t_rob_tail.pc = t_alloc_uop.pc;
	     t_rob_tail.is_br = t_alloc_uop.is_br;
	     t_rob_tail.has_delay_slot = t_alloc_uop.has_delay_slot;
	     t_rob_tail.has_nullifying_delay_slot = t_alloc_uop.has_nullifying_delay_slot;
	     t_rob_tail.pht_idx = t_alloc_uop.pht_idx;
	     
	     t_rob_tail.is_call = t_alloc_uop.op == JAL || t_alloc_uop.op == JALR || t_alloc_uop.op == BAL;
	     t_rob_tail.is_ret = (t_alloc_uop.op == JR) && (t_uop.srcA == 'd31);
	     t_rob_tail.is_break  = (t_alloc_uop.op == BREAK);
	     t_rob_tail.is_indirect = t_alloc_uop.op == JALR || t_alloc_uop.op == JR;
`ifdef ENABLE_CYCLE_ACCOUNTING
	     t_rob_tail.missed_l1d = 1'b0;
	     t_rob_tail.is_mem = 1'b0;
	     t_rob_tail.is_fp = 1'b0;
	     t_rob_tail.fetch_cycle = t_alloc_uop.fetch_cycle;
	     t_rob_tail.alloc_cycle = r_cycle;
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
	     else if(t_uop.hilo_dst_valid)
	       begin
		  t_rob_tail.valid_hilo_dst = 1'b1;
		  t_rob_tail.pdst = {{(`LG_PRF_ENTRIES-`LG_HILO_PRF_ENTRIES){1'b0}}, n_hilo_prf_entry};
		  t_rob_tail.old_pdst = {{(`LG_PRF_ENTRIES-`LG_HILO_PRF_ENTRIES){1'b0}}, r_hilo_alloc_rat};
	       end
	     else if(t_uop.fcr_dst_valid)
	       begin
		  t_rob_tail.valid_fcr_dst = 1'b1;
		  t_rob_tail.pdst = {{(`LG_PRF_ENTRIES-`LG_FCR_PRF_ENTRIES){1'b0}}, n_fcr_prf_entry};
		  t_rob_tail.old_pdst = {{(`LG_PRF_ENTRIES-`LG_FCR_PRF_ENTRIES){1'b0}}, r_fcr_alloc_rat};
	       end
	     else if(t_uop.fp_dst_valid)
	       begin
		  t_rob_tail.valid_fp_dst = 1'b1;
		  t_rob_tail.ldst = t_uop.dst[4:0];
		  t_rob_tail.pdst = n_fp_prf_entry;
		  t_rob_tail.old_pdst = r_fp_alloc_rat[t_uop.dst[4:0]];
	       end
	     
	     if(t_fold_uop)
	       begin
`ifdef ENABLE_CYCLE_ACCOUNTING
		  t_rob_tail.complete_cycle = r_cycle;
`endif		  
		  if(t_uop.op == II)
		    begin
		       t_rob_tail.faulted = 1'b1;
		       t_rob_tail.is_ii = 1'b1;
		    end
		  else if(t_uop.op == J)
		    begin
		       t_rob_tail.take_br = 1'b1;
		    end
	       end
	     
	  end // if (t_alloc)


	if(t_alloc_two)
	  begin
	     t_rob_next_tail.pc = t_alloc_uop2.pc;
	     t_rob_next_tail.is_br = t_alloc_uop2.is_br;
	     t_rob_next_tail.has_delay_slot = t_uop2.has_delay_slot;
	     t_rob_next_tail.has_nullifying_delay_slot = t_uop2.has_nullifying_delay_slot;
	     t_rob_next_tail.pht_idx = t_alloc_uop2.pht_idx;

	     t_rob_next_tail.is_call = t_alloc_uop2.op == JAL || t_alloc_uop2.op == JALR || t_alloc_uop2.op == BAL;
	     t_rob_next_tail.is_ret = (t_alloc_uop2.op == JR) && (t_uop.srcA == 'd31);
	     t_rob_next_tail.is_break  = (t_alloc_uop2.op == BREAK);
	     t_rob_next_tail.is_indirect = t_alloc_uop2.op == JALR || t_alloc_uop2.op == JR;
	     
`ifdef ENABLE_CYCLE_ACCOUNTING
	     t_rob_next_tail.missed_l1d = 1'b0;
	     t_rob_next_tail.is_mem = 1'b0;
	     t_rob_next_tail.is_fp = 1'b0;
	     t_rob_next_tail.fetch_cycle = t_alloc_uop2.fetch_cycle;
	     t_rob_next_tail.alloc_cycle = r_cycle;
	     t_rob_next_tail.complete_cycle = 'd0;
`endif
	     //t_uop.has_delay_slot
	     t_rob_next_tail.in_delay_slot = t_uop.has_delay_slot;

	     if(t_uop2.dst_valid)
	       begin
		  t_rob_next_tail.valid_dst = 1'b1;
		  /* this is correct, we do not want the renamed version */
		  t_rob_next_tail.ldst = t_uop2.dst[4:0];
		  t_rob_next_tail.pdst = n_prf_entry2;
		  t_rob_next_tail.old_pdst = (t_uop.dst_valid && (t_uop.dst == t_uop2.dst)) ? t_rob_tail.pdst : r_alloc_rat[t_uop2.dst[4:0]];
	       end
	     else if(t_uop2.fp_dst_valid)
	       begin
		  t_rob_next_tail.valid_fp_dst = 1'b1;
		  t_rob_next_tail.ldst = t_uop2.dst[4:0];
		  t_rob_next_tail.pdst = n_fp_prf_entry2;
		  t_rob_next_tail.old_pdst = (t_uop.fp_dst_valid && (t_uop.dst == t_uop2.dst)) ? t_rob_tail.pdst :r_fp_alloc_rat[t_uop2.dst[4:0]];
	       end



	     if(t_fold_uop2)
	       begin
`ifdef ENABLE_CYCLE_ACCOUNTING
		  t_rob_next_tail.complete_cycle = r_cycle;
`endif		  
		  if(t_uop2.op == II)
		    begin
		       t_rob_next_tail.faulted = 1'b1;
		       t_rob_next_tail.is_ii = 1'b1;
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
	  end
	else
	  begin
	     if(t_alloc)
	       begin
		  r_rob_complete[r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= t_fold_uop;
	       end
	     if(t_alloc_two)
	       begin
		  r_rob_complete[r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]] <= t_fold_uop2;
	       end
	     if(t_complete_valid_1)
	       begin
		  //$display("rob entry %d marked complete by port 1", t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]);
		  r_rob_complete[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]] <= t_complete_bundle_1.complete;
	       end
	     if(t_complete_valid_2)
	       begin
		  //$display("rob entry %d marked complete by port 2", t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]);
                  r_rob_complete[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]] <= t_complete_bundle_2.complete;
	       end
	     if(core_mem_rsp_valid)
	       begin
		  //$display("rob entry %d marked complete by mem port", core_mem_rsp.rob_ptr);
		  r_rob_complete[core_mem_rsp.rob_ptr] <= 1'b1;
	       end
	  end
     end // always_ff@ (posedge clk)
   
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
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].target_pc <= t_complete_bundle_1.restart_pc;
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].is_ii <= t_complete_bundle_1.is_ii;
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_br <= t_complete_bundle_1.take_br;
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].data <= t_complete_bundle_1.data;
`ifdef ENABLE_CYCLE_ACCOUNTING
		  r_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].complete_cycle <= r_cycle;
`endif	    
	       end
	     if(t_complete_valid_2)
	       begin
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].faulted <= t_complete_bundle_2.faulted;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].target_pc <= t_complete_bundle_2.restart_pc;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].is_ii <= t_complete_bundle_2.is_ii;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_br <= t_complete_bundle_2.take_br;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].data <= t_complete_bundle_2.data;
`ifdef ENABLE_CYCLE_ACCOUNTING
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].complete_cycle <= r_cycle;
		  r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].is_fp <= 1'b1;
`endif	    	     
	       end
	     if(core_mem_rsp_valid)
	       begin
		  r_rob[core_mem_rsp.rob_ptr].data <= core_mem_rsp.data;
		  r_rob[core_mem_rsp.rob_ptr].faulted <= core_mem_rsp.faulted;
`ifdef ENABLE_CYCLE_ACCOUNTING
		  r_rob[core_mem_rsp.rob_ptr].complete_cycle <= r_cycle;
		  r_rob[core_mem_rsp.rob_ptr].is_mem <= core_mem_rsp.was_mem;
		  r_rob[core_mem_rsp.rob_ptr].missed_l1d <= core_mem_rsp.missed_l1d;
`endif	    	     	     
	       end
	  end
     end

   
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
	t_clr_mask = uq_wait|mq_wait|fq_wait;
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
		       //$display("cycle %d, 2 rob ptr %d complete\n", r_cycle, t_complete_bundle_2.rob_ptr);
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
	
	t_rob_empty = (r_rob_head_ptr == r_rob_tail_ptr);
	t_rob_next_empty = (r_rob_next_head_ptr == r_rob_tail_ptr);
	
	t_rob_full = (r_rob_head_ptr[`LG_ROB_ENTRIES-1:0] == r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]) && (r_rob_head_ptr != r_rob_tail_ptr);
	t_rob_next_full = (r_rob_head_ptr[`LG_ROB_ENTRIES-1:0] == r_rob_next_tail_ptr[`LG_ROB_ENTRIES-1:0]) && (r_rob_head_ptr != r_rob_next_tail_ptr);
     end // always_comb


   always_comb
     begin
	t_rob_head = r_rob[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]];
	t_rob_next_head = r_rob[r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0]];
	t_rob_head_complete = r_rob_complete[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]];
	t_rob_next_head_complete = r_rob_complete[r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0]];
     end // always_comb

   

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i < N_HILO_ENTRIES; i = i + 1)
	       begin
		  r_hilo_prf_free[i] <= (i==0) ? 1'b0 : 1'b1;
		  r_retire_hilo_prf_free[i] <= (i==0) ? 1'b0 : 1'b1;
	       end
	  end
	else
	  begin
	     r_hilo_prf_free <= t_rat_copy ? r_retire_hilo_prf_free : n_hilo_prf_free;
	     r_retire_hilo_prf_free <= n_retire_hilo_prf_free;
	  end
     end // always_ff@ (posedge clk)


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i < N_FCR_ENTRIES; i = i + 1)
	       begin
		  r_fcr_prf_free[i] <= (i==0) ? 1'b0 : 1'b1;
		  r_retire_fcr_prf_free[i] <= (i==0) ? 1'b0 : 1'b1;
	       end
	  end
	else
	  begin
	     r_fcr_prf_free <= t_rat_copy ? r_retire_fcr_prf_free : n_fcr_prf_free;
	     r_retire_fcr_prf_free <= n_retire_fcr_prf_free;
	  end
     end // always_ff@ (posedge clk)
   


   popcount #(`LG_PRF_ENTRIES) cnt_fpr (.in(r_fp_prf_free), 
					.out(t_fp_prf_free_cnt));
   

   find_first_set#(`LG_PRF_ENTRIES) ffs_fp(.in(r_fp_prf_free),
					   .y(t_fp_ffs));

   always_comb
     begin
	t_fp_prf_free = r_fp_prf_free;
	t_fp_prf_free[t_fp_ffs[`LG_PRF_ENTRIES-1:0]] = 1'b0;
     end

   
   find_first_set#(`LG_PRF_ENTRIES) ffs_fp2(.in(t_fp_prf_free),
					    .y(t_fp_ffs2));

   
   
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

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i < N_PRF_ENTRIES; i = i + 1)
	       begin
		  r_fp_prf_free[i] <= (i < 32) ? 1'b0 : 1'b1;
		  r_retire_fp_prf_free[i] <= (i < 32) ? 1'b0 : 1'b1;
	       end
	  end
	else
	  begin
	     r_fp_prf_free <= t_rat_copy ? r_retire_fp_prf_free : n_fp_prf_free;
	     r_retire_fp_prf_free <= n_retire_fp_prf_free;
	  end
     end // always_ff@ (posedge clk)


   
   find_first_set#(`LG_HILO_PRF_ENTRIES) ffs_hilo(.in(r_hilo_prf_free),
						 .y(t_hilo_ffs));

   always_comb
     begin
	n_hilo_prf_free = r_hilo_prf_free;
	n_hilo_prf_entry = t_hilo_ffs[`LG_HILO_PRF_ENTRIES-1:0];
	
	if(t_alloc & t_uop.hilo_dst_valid)
	  begin
	     n_hilo_prf_free[n_hilo_prf_entry] = 1'b0;
	  end
	if(t_free_hilo)
	  begin
	     n_hilo_prf_free[t_free_hilo_ptr] = 1'b1;
	  end
	
     end // always_comb

   find_first_set#(`LG_FCR_PRF_ENTRIES) ffs_fcr(.in(r_fcr_prf_free),
						 .y(t_fcr_ffs));

   always_comb
     begin
	n_fcr_prf_free = r_fcr_prf_free;
	n_fcr_prf_entry = t_fcr_ffs[`LG_FCR_PRF_ENTRIES-1:0];
	
	if(t_alloc & t_uop.fcr_dst_valid)
	  begin
	     n_fcr_prf_free[n_fcr_prf_entry] = 1'b0;
	  end
	if(t_free_fcr)
	  begin
	     n_fcr_prf_free[t_free_fcr_ptr] = 1'b1;
	  end
	
     end // always_comb


   // always_ff@(negedge clk)
   //   begin
   // 	if(t_alloc && t_uop.fp_dst_valid)
   // 	  begin
   // 	     $display("cycle %d : allocating n_fp_prf_entry = %d, %d, %d, %d, pc = %x", r_cycle, n_fp_prf_entry, 
   // 		      t_alloc_uop.srcA, t_alloc_uop.srcB, t_alloc_uop.srcC, t_uop.pc);	     
   // 	  end
   // 	if(t_alloc_two && t_uop2.fp_dst_valid)
   // 	  begin
   // 	     $display("cycle %d : allocating n_fp_prf_entry2 = %d, %d, %d, %d, pc = %x", r_cycle, n_fp_prf_entry2, 
   // 		      t_alloc_uop2.srcA, t_alloc_uop2.srcB, t_alloc_uop2.srcC, t_uop2.pc);
   // 	  end
   //   end
   
   always_comb
     begin
	n_fp_prf_free = r_fp_prf_free;
	n_fp_prf_entry = t_fp_ffs[`LG_PRF_ENTRIES-1:0];
	n_fp_prf_entry2 = t_fp_ffs2[`LG_PRF_ENTRIES-1:0];
	if(t_alloc && t_uop.fp_dst_valid)
	  begin
	     n_fp_prf_free[n_fp_prf_entry] = 1'b0;
	  end
	if(t_alloc_two && t_uop2.fp_dst_valid)
	  begin
	     n_fp_prf_free[n_fp_prf_entry2] = 1'b0;
	  end
	if(t_free_fp_reg)
	  begin
	     n_fp_prf_free[t_free_fp_reg_ptr] = 1'b1;
	  end
	if(t_free_fp_two_reg)
	  begin
	     n_fp_prf_free[t_free_fp_reg_two_ptr] = 1'b1;
	  end
     end   


   popcount #(`LG_PRF_ENTRIES) cnt_gpr (.in(r_prf_free), .out(t_prf_free_cnt));
   
   //always_ff@(negedge clk)
   //begin
   //$display("%d free gpr prf entries", t_prf_free_cnt);
   //end
   
   find_first_set#(`LG_PRF_ENTRIES) ffs_gpr(.in(r_prf_free),
					    .y(t_gpr_ffs));

   always_comb
     begin
	t_prf_free = r_prf_free;
	t_prf_free[t_gpr_ffs[`LG_PRF_ENTRIES-1:0]] = 1'b0;
     end
   
   find_first_set#(`LG_PRF_ENTRIES) ffs_gpr2(.in(t_prf_free),
					     .y(t_gpr_ffs2));

   
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

   
   decode_mips32 dec0 (.insn(insn.data), 
		      .pc(insn.pc), 
		      .insn_pred(insn.pred), 
		      .pht_idx(insn.pht_idx),
		      .insn_pred_target(insn.pred_target),
`ifdef ENABLE_CYCLE_ACCOUNTING
		      .fetch_cycle(insn.fetch_cycle),
`endif		      
		      .uop(t_dec_uop));

   decode_mips32 dec1 (.insn(insn_two.data), 
		      .pc(insn_two.pc), 
		      .insn_pred(insn_two.pred), 
		      .pht_idx(insn_two.pht_idx),
		      .insn_pred_target(insn_two.pred_target),
`ifdef ENABLE_CYCLE_ACCOUNTING
		      .fetch_cycle(insn_two.fetch_cycle),
`endif		      
		      .uop(t_dec_uop2));


   
   logic t_push_1, t_push_2;
   
   always_comb
     begin
	t_any_complete = t_complete_valid_1 | t_complete_valid_2 | core_mem_rsp_valid;
	t_push_1 = t_alloc && !t_fold_uop;
	t_push_2 = t_alloc_two && !t_fold_uop2;
     end


   
   exec e (
	   .clk(clk), 
	   .reset(reset),
	   .divide_ready(t_divide_ready),
`ifdef VERILATOR
	   .clear_cnt(r_clear_cnt),
`endif
	   .ds_done(r_ds_done),
	   .machine_clr(r_machine_clr),
	   .restart_complete(t_restart_complete),
	   .delayslot_rob_ptr(r_delayslot_rob_ptr),
	   .cpr0_status_reg(t_cpr0_status_reg),
	   .mq_wait(mq_wait),
	   .uq_wait(uq_wait),
	   .fq_wait(fq_wait),
	   .uq_empty(t_uq_empty),
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
	   .exception_wr_cpr0_val(t_exception_wr_cpr0_val),
	   .exception_wr_cpr0_ptr(t_exception_wr_cpr0_ptr),
	   .exception_wr_cpr0_data(t_exception_wr_cpr0_data[31:0]),
	   .mem_req(t_mem_req),
	   .mem_req_valid(t_mem_req_valid),
	   .mem_req_ack(core_mem_req_ack),
	   .mem_rsp_dst_ptr(core_mem_rsp.dst_ptr),
	   .mem_rsp_dst_valid(core_mem_rsp.dst_valid),
	   .mem_rsp_fp_dst_valid(core_mem_rsp.fp_dst_valid),
	   .mem_rsp_load_data(core_mem_rsp.data),
	   .mem_rsp_rob_ptr(core_mem_rsp.rob_ptr),
	   .monitor_rsp_data(r_monitor_rsp_data)
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
	r_dq <= n_dq;
     end

   always_ff@(negedge clk)
     begin
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
	n_dq = r_dq;
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
		  n_dq[r_dq_tail_ptr[`LG_DQ_ENTRIES-1:0]] = t_dec_uop;
		  n_dq_tail_ptr = r_dq_tail_ptr + 'd1;
		  n_dq_next_tail_ptr = r_dq_next_tail_ptr + 'd1;
		  n_dq_cnt = n_dq_cnt + 'd1;
	       end
	     else if(insn_valid && !t_dq_full && !t_dq_next_full && insn_valid_two)
	       begin
		  //push two instructions
		  n_dq[r_dq_tail_ptr[`LG_DQ_ENTRIES-1:0]] = t_dec_uop;
		  n_dq[r_dq_next_tail_ptr[`LG_DQ_ENTRIES-1:0]] = t_dec_uop2;
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
