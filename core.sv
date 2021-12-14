`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

module core(clk, 
	    reset,
	    head_of_rob_ptr,
	    resume,
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
	    
	    retire_valid,
	    retire_two_valid,
	    retire_delay_slot,
	    retire_pc,
	    retire_two_pc,
	    retired_call,
	    retired_ret,
	    monitor_req_reason,
	    monitor_req_valid,
	    monitor_rsp_valid,
	    monitor_rsp_data_valid,
	    monitor_rsp_data,
	    got_break,
	    got_syscall,
	    got_ud,
	    inflight);
   input logic clk;
   input logic reset;
   output logic [`LG_ROB_ENTRIES-1:0] head_of_rob_ptr;
   input logic resume;
   input logic [(`M_WIDTH-1):0] resume_pc;
   output logic 		ready_for_resume;
   output logic flush_req;
   output logic flush_cl_req;
   output logic [(`M_WIDTH-1):0] flush_cl_addr;
   input logic 	l1d_flush_complete;
   input logic 	l1i_flush_complete;
   
   input 	insn_fetch_t insn;
      
   input logic 			insn_valid;
   
   output logic 		insn_ack;
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
   output logic [(`M_WIDTH-1):0] 	  retire_reg_data;
   output logic 			  retire_reg_valid;
   output logic 			  retire_reg_fp_valid;

   output logic [4:0] 			  retire_reg_two_ptr;
   output logic [(`M_WIDTH-1):0] 	  retire_reg_two_data;
   output logic 			  retire_reg_two_valid;

   
   output logic 			  retire_valid;
   output logic 			  retire_two_valid;
   
   output logic 			  retire_delay_slot;
   
   output logic [(`M_WIDTH-1):0] 	  retire_pc;
   output logic [(`M_WIDTH-1):0] 	  retire_two_pc;
   output logic 			  retired_call;
   output logic 			  retired_ret;
   
   
   output logic [15:0] 			  monitor_req_reason;
   output logic 			  monitor_req_valid;
   input logic 				  monitor_rsp_valid;
   input logic 				  monitor_rsp_data_valid;
   input logic [(`M_WIDTH-1):0] 	  monitor_rsp_data;
   output logic 			  got_break;
   output logic 			  got_syscall;
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
   logic [`LG_DQ_ENTRIES:0] 		  r_dq_tail_ptr, n_dq_tail_ptr;
   logic 				  t_dq_empty, t_dq_full;
   logic 				  r_got_restart_ack, n_got_restart_ack;
   
   rob_entry_t r_rob[N_ROB_ENTRIES-1:0];
   rob_entry_t n_rob[N_ROB_ENTRIES-1:0];
   rob_entry_t t_rob_head;
   rob_entry_t t_rob_next_head;
   rob_entry_t t_rob_tail;

   logic [N_PRF_ENTRIES-1:0] n_prf_free, r_prf_free;   
   logic [N_PRF_ENTRIES-1:0] n_retire_prf_free, r_retire_prf_free;

   logic [N_PRF_ENTRIES-1:0] n_fp_prf_free, r_fp_prf_free;   
   logic [N_PRF_ENTRIES-1:0] n_retire_fp_prf_free, r_retire_fp_prf_free;
   
   logic [N_HILO_ENTRIES-1:0] n_hilo_prf_free,r_hilo_prf_free;
   logic [N_HILO_ENTRIES-1:0] n_retire_hilo_prf_free, r_retire_hilo_prf_free;
   logic [`LG_HILO_PRF_ENTRIES-1:0] n_hilo_prf_entry;
   logic [`LG_HILO_PRF_ENTRIES:0]   t_hilo_prf_idx;

   logic [N_FCR_ENTRIES-1:0] n_fcr_prf_free,r_fcr_prf_free;
   logic [N_FCR_ENTRIES-1:0] n_retire_fcr_prf_free, r_retire_fcr_prf_free;
   logic [`LG_FCR_PRF_ENTRIES-1:0] n_fcr_prf_entry;
   logic [`LG_FCR_PRF_ENTRIES:0]   t_fcr_prf_idx;
   
   logic [`LG_PRF_ENTRIES-1:0]  n_prf_entry;
   logic [`LG_PRF_ENTRIES-1:0] 	n_fp_prf_entry;

   logic [`LG_ROB_ENTRIES:0] r_rob_head_ptr, n_rob_head_ptr;
   logic [`LG_ROB_ENTRIES:0] r_rob_next_head_ptr, n_rob_next_head_ptr;
   
   logic [`LG_ROB_ENTRIES:0] r_rob_tail_ptr, n_rob_tail_ptr;
   

   logic [`LG_ROB_ENTRIES:0] r_inflight, n_inflight;
   logic [`LG_ROB_ENTRIES:0] r_insns_in_rob_cnt, n_insns_in_rob_cnt;
      
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

   
   logic 		     t_rob_empty, t_rob_full;
   logic 		     t_alloc, t_retire, t_retire_two, t_rat_copy, t_clr_rob;
   logic 		     t_fold_uop;
   
   logic 		     n_in_delay_slot, r_in_delay_slot;
   
   logic 		     t_clr_dq;
   logic 		     t_enough_iprfs, t_enough_hlprfs, t_enough_fprfs, 
			     t_enough_fcrprfs;
   
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
   logic 		     n_got_syscall, r_got_syscall;
   logic 		     n_got_ud, r_got_ud;

   logic 		     n_l1i_flush_complete, r_l1i_flush_complete;
   logic 		     n_l1d_flush_complete, r_l1d_flush_complete;
   
   logic 		     t_in_32fp_reg_mode;
   logic [(`M_WIDTH-1):0]    r_arch_a0;

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
   
   logic 			   t_free_hilo;
   logic [`LG_HILO_PRF_ENTRIES-1:0] t_free_hilo_ptr;
   
   logic 			    t_free_fcr;
   logic [`LG_FCR_PRF_ENTRIES-1:0]  t_free_fcr_ptr;
   

   logic [`LG_HILO_PRF_ENTRIES:0]   t_hilo_ffs;
   logic [`LG_FCR_PRF_ENTRIES:0]    t_fcr_ffs;
   logic [`LG_PRF_ENTRIES:0] 	    t_fp_ffs;
   logic [`LG_PRF_ENTRIES:0] 	    t_gpr_ffs;

   uop_t r_uq[N_UQ_ENTRIES];
   uop_t n_uq[N_UQ_ENTRIES];
   
   logic [`LG_UQ_ENTRIES:0]  r_uq_head_ptr, n_uq_head_ptr;
   logic [`LG_UQ_ENTRIES:0]  r_uq_tail_ptr, n_uq_tail_ptr;
   logic 		     t_uq_full, t_uq_empty;
   logic 		     t_uq_read;
   logic 		     n_ready_for_resume, r_ready_for_resume;
   
   logic 		     t_exception_wr_cpr0_val;
   logic [4:0] 		     t_exception_wr_cpr0_ptr;
   logic [`M_WIDTH-1:0]      t_exception_wr_cpr0_data;
   
   mem_req_t t_mem_req;
   logic 		     t_mem_req_valid;
   logic 		     t_monitor_req_valid;
   logic [(`M_WIDTH-1):0]    r_monitor_rsp_data, n_monitor_rsp_data;
   logic 		     r_monitor_rsp_data_valid, n_monitor_rsp_data_valid;

   logic 		     n_machine_clr, r_machine_clr;
   logic 		     n_flush_req, r_flush_req;
   logic 		     n_flush_cl_req, r_flush_cl_req;
   logic [(`M_WIDTH-1):0]    n_flush_cl_addr, r_flush_cl_addr;

   logic 		     t_can_retire_rob_head;
   
   logic [`LG_ROB_ENTRIES-1:0] n_delayslot_rob_ptr, r_delayslot_rob_ptr;
   typedef enum logic [4:0] {HALT = 'd0,
			     WAIT_FOR_DELAY_SLOT = 'd1,
			     DRAIN = 'd2, 
			     RAT = 'd3,
			     DELAY_SLOT = 'd4,
			     ALLOC_FOR_SERIALIZE = 'd5,
			     WAIT_FOR_SERIALIZE = 'd6,
			     MONITOR_FLUSH_CACHE = 'd7,
			     HANDLE_MONITOR = 'd8,
			     ALLOC_FOR_MONITOR = 'd9,
			     WAIT_FOR_MONITOR= 'd10,
			     ACTIVE = 'd11,
			     FLUSH_FOR_HALT = 'd12,
			     HALT_WAIT_FOR_RESTART = 'd13,
			     WAIT_FOR_SERIALIZE_AND_RESTART = 'd14,
			     WRITE_EPC = 'd15,
			     WRITE_CAUSE = 'd16,
			     WRITE_BADVADDR = 'd17
			     } state_t;
   
   state_t r_state, n_state;
   

   always_comb
     begin
	core_mem_req_valid = 1'b0;
	core_mem_req = t_mem_req;
	
	if(t_mem_req_valid)
	  begin
	     if(t_mem_req.is_store)
	       begin
		  /* head of memory queue */
		  if((r_state == DRAIN) && (
					    (t_mem_req.rob_ptr != r_rob_head_ptr[`LG_ROB_ENTRIES-1:0])
					    || r_has_nullifying_delay_slot
					    )
		     )
		    begin

		       core_mem_req.op = (t_mem_req.op == MEM_SC) ? MEM_DEAD_SC :  MEM_DEAD_ST;
		       core_mem_req_valid = 1'b1;
		    end
		  else
		    begin
		       core_mem_req_valid = t_mem_req.rob_ptr == r_rob_head_ptr[`LG_ROB_ENTRIES-1:0];
		    end
		  //if(t_mem_req.rob_ptr == r_rob_head_ptr[`LG_ROB_ENTRIES-1:0])
	       end
	     else
	       begin
		  core_mem_req_valid = 1'b1;
		  if((r_state == DRAIN) && ((t_mem_req.rob_ptr != r_rob_head_ptr[`LG_ROB_ENTRIES-1:0])
					    || r_has_nullifying_delay_slot
					    ) 
		     && !t_mem_req.is_fp)
		    begin
		       //if(t_mem_req.is_fp)
		       core_mem_req.op = MEM_DEAD_LD;
		    end
	       end
	  end
     end

   assign ready_for_resume = r_ready_for_resume;
   assign head_of_rob_ptr = r_rob_head_ptr[`LG_ROB_ENTRIES-1:0];
   assign flush_req = r_flush_req;
   assign flush_cl_req = r_flush_cl_req;
   assign flush_cl_addr = r_flush_cl_addr;
   

   assign monitor_req_reason = r_monitor_reason;
   assign monitor_req_valid = t_monitor_req_valid;
   
   assign got_break = r_got_break;
   assign got_syscall = r_got_syscall;
   assign got_ud = r_got_ud;
   assign inflight = r_insns_in_rob_cnt;
   
   uop_t t_uop, t_dec_uop, t_alloc_uop;
      
   assign insn_ack = !t_dq_full && insn_valid;
   assign restart_pc = r_restart_pc;
   assign restart_src_pc = r_restart_src_pc;
   assign restart_src_is_indirect = r_restart_src_is_indirect;

   
   assign restart_valid = r_restart_valid;

   
   assign branch_pc = r_branch_pc;
   assign branch_pc_valid = r_branch_valid;
   assign branch_fault = r_branch_fault;
   assign branch_pht_idx = r_branch_pht_idx;
   
   assign took_branch = r_took_branch;
   
   
   logic [31:0] r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : r_cycle + 'd1;

     end
   
   // logic [31:0] r_last_retire_cycle;
   // always_ff@(posedge clk)
   //   begin
   // 	r_last_retire_cycle <= reset ? 'd0  : (t_retire ? r_cycle : r_last_retire_cycle);
   //   end
   // logic [31:0] t_last_cycle;
   // always_ff@(negedge clk)
   //   begin
   // 	t_last_cycle = (r_cycle - r_last_retire_cycle);
   // 	if(t_last_cycle > 'd10000)
   // 	  begin
   // 	     $display("no retire in %d cycles! rob_head_ptr %d, pc at head %x, insns in rob %d", 
   // 		      (r_cycle - r_last_retire_cycle),
   // 		      r_rob_head_ptr,
   // 		      t_rob_head.pc,
   // 		      r_insns_in_rob_cnt
   // 		      );

   // 	  end
   // 	if(t_last_cycle > 'd15000)
   // 	  begin
   // 	     $stop();
   // 	  end
   //   end

   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_flush_req <= 1'b0;
	     r_flush_cl_req <= 1'b0;
	     r_flush_cl_addr <= 'd0;
	     r_inflight <= 'd0;
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
	     r_monitor_rsp_data_valid <= 1'b0;
	     r_got_break <= 1'b0;
	     r_got_syscall <= 1'b0;
	     r_got_ud <= 1'b0;
	     r_ready_for_resume <= 1'b0;
	     r_l1i_flush_complete <= 1'b0;
	     r_l1d_flush_complete <= 1'b0;
	  end
	else
	  begin
	     r_flush_req <= n_flush_req;
	     r_flush_cl_req <= n_flush_cl_req;
	     r_flush_cl_addr <= n_flush_cl_addr;
	     r_inflight <= n_inflight;
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
	     r_monitor_rsp_data_valid <= n_monitor_rsp_data_valid;
	     r_got_break <= n_got_break;
	     r_got_syscall <= n_got_syscall;
	     r_got_ud <= n_got_ud;
	     r_ready_for_resume <= n_ready_for_resume;
	     r_l1i_flush_complete <= n_l1i_flush_complete;
	     r_l1d_flush_complete <= n_l1d_flush_complete;	     
	  end
     end // always_ff@ (posedge clk)

//`define DEBUG
`ifdef DEBUG
   logic [4:0] total_inflight;   
   always_comb
     begin
	total_inflight = 'd0;
	for(logic [4:0] i = r_rob_head_ptr; i != (r_rob_tail_ptr); i=i+1)
	  begin
	     total_inflight = total_inflight + (r_rob[i[3:0]].complete ? 'd0 : 'd1);
	  end
     end

   always_ff@(negedge clk)
     begin
	if(r_inflight != total_inflight)
	  begin
	     $display("rob complete count %d, r_inflight = %d",
		      total_inflight, r_inflight);
	     $stop();
	  end
     end
`endif //  `ifdef DEBUG
   
   always_comb
     begin
	n_inflight = r_inflight;
	if(t_alloc && !t_fold_uop)
	  begin
	     n_inflight = n_inflight + 'd1;
	     //$display("incr inflight due to alloc");
	  end
	if(t_complete_valid_1)
	  begin
	     n_inflight = n_inflight - 'd1;
	     //$display("decr complete valid 1 for rob entry %d",
	     //t_complete_bundle_1.rob_ptr);
	  end
	if(t_complete_valid_2)
	  begin
	     n_inflight = n_inflight - 'd1;
	     //$display("decr complete valid 2 for rob entry %d", 
	     //t_complete_bundle_2.rob_ptr);
	  end
	if(core_mem_rsp_valid)
	  begin
	     n_inflight = n_inflight - 'd1;
	     //$display("decr mem rsp");
	  end
     end // always_comb
   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_state <= HALT;
	     r_machine_clr <= 1'b0;
	     r_delayslot_rob_ptr <= 'd0;
	     r_got_restart_ack <= 1'b0;
	     r_cause <= 5'd0;
	  end
	else
	  begin
	     r_state <= n_state;
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
	     r_arch_a0 <= t_rob_head.data;
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
   	     retire_valid <= t_retire;
	     retire_two_valid <= t_retire_two;
   	     retire_pc <= t_rob_head.pc;
	     retire_two_pc <= t_rob_next_head.pc;
	     retire_delay_slot <= t_rob_head.in_delay_slot && t_retire;
	     retired_ret <= t_rob_head.is_ret && t_retire;
	     retired_call <= t_rob_head.is_call && t_retire;
	     r_monitor_reason <= n_monitor_reason;
   	  end
     end

//`define DEBUG
//`define DUMP_ROB
`ifdef DUMP_ROB
   always_ff@(negedge clk)
     begin
	$display("cycle %d : state = %d, alu complete %b, mem complete %b,head_ptr %d, inflight %d, complete %b,  can_retire_rob_head %b, head pc %x, empty %b, full %b", 
		 r_cycle,
		 r_state,
		 t_complete_valid_1,
		 core_mem_rsp_valid,
		 r_rob_head_ptr,
		 r_inflight,
		 t_rob_head.complete && !t_rob_empty, 
		 t_can_retire_rob_head,
		 t_rob_head.pc,
		 t_rob_empty, 
		 t_rob_full);
	
	for(logic [4:0] i = r_rob_head_ptr; i != (r_rob_tail_ptr); i=i+1)
	  begin
	     $display("\trob entry %d, pc %x, complete %b",
		      i[3:0], r_rob[i[3:0]].pc, r_rob[i[3:0]].complete);
	  end
     end // always_ff@ (negedge clk)
`endif
   
   always_comb
     begin
	t_exception_wr_cpr0_val = 1'b0;
	t_exception_wr_cpr0_ptr = 5'd0;
	t_exception_wr_cpr0_data = 64'd0;
	n_cause = r_cause;
	
	n_machine_clr = r_machine_clr;
	n_delayslot_rob_ptr = r_delayslot_rob_ptr;
	t_alloc = 1'b0;
	n_in_delay_slot = r_in_delay_slot;
	t_retire = 1'b0;
	t_retire_two = 1'b0;
	t_rat_copy = 1'b0;
	t_clr_rob = 1'b0;
	t_clr_dq = 1'b0;
	n_state = r_state;
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
	n_monitor_rsp_data_valid = r_monitor_rsp_data_valid;
	t_enough_iprfs = !((t_uop.dst_valid) && (r_prf_free == 'd0));
	t_enough_hlprfs = !((t_uop.hilo_dst_valid) && (r_hilo_prf_free == 'd0));
	t_enough_fprfs = !((t_uop.fp_dst_valid) && (r_fp_prf_free == 'd0));
	t_enough_fcrprfs = !((t_uop.fcr_dst_valid) && (r_fcr_prf_free == 'd0));
	t_fold_uop = (t_uop.op == NOP || 
		      t_uop.op == J  ||
		      t_uop.op == II || 
		      t_uop.op == ERET);
	n_flush_req = 1'b0;
	n_flush_cl_req = 1'b0;
	n_flush_cl_addr = r_flush_cl_addr;
	n_got_break = r_got_break;
	n_got_syscall = r_got_syscall;
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
	
	if(t_rob_head.complete && !t_rob_empty)
	  begin
	     t_can_retire_rob_head = (t_rob_head.has_delay_slot || t_rob_head.has_nullifying_delay_slot) ? (r_insns_in_rob_cnt > 'd1) : 1'b1;
	  end

	//if(
	  //begin
	//$display("could retire two insns @ cycle %d!", r_cycle);
	  //end
	
	
	       
	unique case (r_state)
	  ACTIVE:
	    begin
	       if(t_can_retire_rob_head)
		 begin
		    if(t_rob_head.faulted)
		      begin
			 //$display("cycle %d : got fault for %x, t_insns_in_rob = %d, eret %b, break %d, ii %d, trap %d, tlb %b",
			 //r_cycle, t_rob_head.pc, r_insns_in_rob_cnt,
			 //t_rob_head.is_eret, t_rob_head.is_break,
			 //t_rob_head.is_ii,t_rob_head.take_trap,
			 //t_rob_head.exception_tlb_refill
			 //	  );
			 if(t_rob_head.is_eret)
			   begin
			      $stop();
			   end
			 else if(t_rob_head.is_break)
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
			 else if(t_rob_head.take_trap)
			   begin
			      n_flush_req = 1'b1;
			      n_cause = 5'd13;
			      n_state = WRITE_EPC;
			   end
			 else if(t_rob_head.exception_tlb_refill)
			   begin
			      n_flush_req = 1'b1;
			      n_cause = t_rob_head.is_store ? 5'd3 : 5'd2;
			      n_state = WRITE_BADVADDR;
			      $display("exception for tlb refill for pc %x, vaddr %x, is store %b", 
				       t_rob_head.pc, t_rob_head.data, t_rob_head.is_store);
			   end
			 else if(t_rob_head.exception_tlb_modified)
			   begin
			      $stop();
			   end
			 else if(t_rob_head.exception_tlb_invalid)
			   begin
			      $stop();
			   end
			 // else if(t_rob_head.is_syscall)
			 //   begin
			 //      n_got_syscall = 1'b1;
			 //      n_cause = 5'd8;
			 //      n_flush_req = 1'b1;
			 //      n_state = WRITE_EPC;
			 //   end
			 else
			   begin
			      n_state = DRAIN;
			   end // else: !if(t_rob_head.is_ii)
			 n_machine_clr = 1'b1;
			 n_delayslot_rob_ptr = r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0];
			 n_restart_pc = t_rob_head.target_pc;
			 n_restart_src_pc = t_rob_head.pc;
			 n_restart_src_is_indirect = t_rob_head.is_indirect && !t_rob_head.is_ret;
			 //if(n_restart_src_is_indirect)
			 //$display("mispredict for pc %x->%x", t_rob_head.pc, t_rob_head.target_pc);
			 
			 n_has_delay_slot = t_rob_head.has_delay_slot;
			 n_has_nullifying_delay_slot = t_rob_head.has_nullifying_delay_slot;
			 n_take_br = t_rob_head.take_br;
			 t_bump_rob_head = 1'b1;
		      end
		    else
		      begin
			 if(t_uop.serializing_op && !t_dq_empty)
			   begin
			      if(/*r_inflight*/t_rob_empty)
				begin
				   n_state = (t_uop.op == MONITOR || t_uop.op == SYSCALL) ? 
					     HANDLE_MONITOR : ALLOC_FOR_SERIALIZE;
				   n_monitor_reason = t_uop.imm;
				end
			   end
			 else
			   begin
			      t_alloc = !t_rob_full && !t_uq_full 
					&& t_enough_iprfs
					&& t_enough_hlprfs
					&& t_enough_fprfs
					&& t_enough_fcrprfs
					&& !t_dq_empty;
			   end
		      end // else: !if(t_rob_head.faulted)
		    t_retire = t_rob_head.complete;
`ifdef RETIRE_TWO
		    t_retire_two = ((r_insns_in_rob_cnt > 'd1) 
		    		    && !t_rob_head.faulted
		    		    && !t_rob_next_head.faulted 				    
		    		    && t_rob_head.complete
		    		    && t_rob_next_head.complete				    
				    && !t_rob_head.is_br
				    && !t_rob_next_head.is_ret
				    && !t_rob_next_head.is_call
				    && !t_rob_next_head.valid_fp_dst
				    && !t_rob_next_head.valid_fcr_dst
		    		    && !t_rob_next_head.valid_hilo_dst);
`endif
		 end // if (t_rob_head.complete)
	       else if(!t_dq_empty)
		 begin
		    if(t_uop.serializing_op)
		      begin
			 if(/*r_inflight*/t_rob_empty)
			   begin
			      if(t_uop.op == MONITOR)
				begin
				   n_monitor_reason = t_uop.imm;
				   case(t_uop.imm)
				     'd50: /* get cycle */
				       begin
					  n_state = HANDLE_MONITOR;
				       end
				     'd52: /* flush cacheline */
				       begin
					  n_state = MONITOR_FLUSH_CACHE;
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
			      else if(t_uop.op == SYSCALL)
				begin
				   n_flush_req = 1'b1;
				   n_state = MONITOR_FLUSH_CACHE;
				end
			      else
				begin
				   n_state =  ALLOC_FOR_SERIALIZE;
				end
			   end // if (...
		      end
		    else
		      begin
			 t_alloc = !t_rob_full && !t_uq_full 
				   && t_enough_iprfs
				   && t_enough_hlprfs
				   && t_enough_fprfs
				   && t_enough_fcrprfs
				   && !t_dq_empty;
		      end
		 end
	    end // case: ACTIVE
	  WAIT_FOR_DELAY_SLOT:
	    begin
	       //$display("waiting for delay slot..");
	       //if(t_rob_head.complete)
	       //begin
	       n_state = RAT;
	       n_restart_valid = 1'b1;
	       //end
	    end
	  DRAIN:	    
	    begin
	       if(r_inflight == 'd0)
		 begin
		    if(r_has_nullifying_delay_slot)
		      begin
			 if(r_take_br)
			   begin
			      /* we predicted not taken, was taken */
			      t_retire = 1'b1;
			      n_state = WAIT_FOR_DELAY_SLOT;
			      //$display("likely branch : predict NT, was T\n");
			   end
			 else 
			   begin
			      /* we predicted take, was not taken */
			      //$display("likely branch : predict T, was NT\n");
			      n_state = RAT;
			      n_restart_valid = 1'b1;
			   end
		      end
		    else if(r_has_delay_slot)
		      begin
			 t_retire = 1'b1;
			 n_state = WAIT_FOR_DELAY_SLOT;
		      end
		    else
		      begin
			 n_state = RAT;
			 n_restart_valid = 1'b1;
		      end
		 end
	    end // case: DRAIN
	  RAT:
	    begin
	       t_rat_copy = 1'b1;
	       t_clr_rob = 1'b1;
	       t_clr_dq = 1'b1;
	       if(n_got_restart_ack)
		 begin
		    //$display("RESTART PIPELINE AT %d", r_cycle);
		    n_state = ACTIVE;
		 end
	       n_machine_clr = 1'b0;
	    end
	  ALLOC_FOR_SERIALIZE:
	    begin
	       t_alloc = !t_rob_full && !t_uq_full 
			 && (r_prf_free != 'd0) 
			   && !t_dq_empty;			 
	       n_state = t_uop.must_restart ? WAIT_FOR_SERIALIZE_AND_RESTART :
			 WAIT_FOR_SERIALIZE;
	    end
	  WAIT_FOR_SERIALIZE:
	    begin
	       if(t_any_complete)
		 begin
		    n_state = ACTIVE;
		 end
	    end
	  WAIT_FOR_SERIALIZE_AND_RESTART:
	    begin
	       if(t_rob_head.complete)
		 begin
		    t_clr_dq = 1'b1;
		    n_restart_pc = t_rob_head.target_pc;
		    n_restart_src_pc = t_rob_head.pc;
		    n_restart_src_is_indirect = 1'b0;
		    n_restart_valid = 1'b1;
		    if(n_got_restart_ack)
		      begin
			 //$display("RESTART PIPELINE AT %d", r_cycle);
			 n_state = ACTIVE;
		      end
		 end
	    end
	  MONITOR_FLUSH_CACHE:
	    begin
`ifdef DEBUG
	       $display("r_cycle = %d, n_l1i_flush_complete = %b, n_l1d_flush_complete = %b", 
			r_cycle, n_l1i_flush_complete, n_l1d_flush_complete);
`endif
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
		    n_monitor_rsp_data_valid = monitor_rsp_data_valid;
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
	       if(t_rob_head.complete)
		 begin
		    t_clr_dq = 1'b1;
		    n_restart_pc = t_rob_head.target_pc;
		    n_restart_src_pc = t_rob_head.pc;
		    n_restart_src_is_indirect = 1'b0;
		    n_restart_valid = 1'b1;
		    if(n_got_restart_ack)
		      begin
			 //$display(">>>> monitor/syscall at %x complete at cycle %d, restart to %x",
			 //t_rob_head.pc, r_cycle, n_restart_pc);			 
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
		    n_got_syscall = 1'b0;
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
	       t_exception_wr_cpr0_data = t_rob_head.in_delay_slot ? (t_rob_head.pc - 'd4) : t_rob_head.pc;
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
	     n_in_delay_slot = t_uop.has_delay_slot;
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
	     for(integer i = 0; i < N_UQ_ENTRIES; i = i + 1)
	       begin
		  r_uq[i].srcA_valid <= 1'b0;
		  r_uq[i].srcB_valid <= 1'b0;
		  r_uq[i].srcC_valid <= 1'b0;
		  r_uq[i].dst_valid <= 1'b0;
	       end
	  end
	else
	  begin
	     r_uq <= n_uq;
	  end
     end
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_uq_head_ptr <= 'd0;
	     r_uq_tail_ptr <= 'd0;
	  end
	else
	  begin
	     r_uq_head_ptr <= n_uq_head_ptr;
	     r_uq_tail_ptr <= n_uq_tail_ptr;
	  end
     end // always_ff@ (posedge clk)


      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_rob_head_ptr <= 'd0;
	     r_rob_tail_ptr <= 'd0;
	     r_rob_next_head_ptr <= 'd1;
	     r_insns_in_rob_cnt <= 'd0;
	  end
	else
	  begin
	     r_rob_head_ptr <= n_rob_head_ptr;
	     r_rob_tail_ptr <= n_rob_tail_ptr;
	     r_rob_next_head_ptr <= n_rob_next_head_ptr;
	     r_insns_in_rob_cnt <= n_insns_in_rob_cnt;
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
   
   
   always_comb
     begin
	n_alloc_rat = r_alloc_rat;
	n_hilo_alloc_rat = r_hilo_alloc_rat;
	n_fp_alloc_rat = r_fp_alloc_rat;
	n_fcr_alloc_rat = r_fcr_alloc_rat;
	
	t_alloc_uop = t_uop;

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
	if(t_uop.srcC_valid || t_uop.fp_srcC_valid)
	  begin
	     t_alloc_uop.srcC = t_uop.fp_srcC_valid ?
				r_fp_alloc_rat[t_uop.srcC[4:0]] :
				r_alloc_rat[t_uop.srcC[4:0]];
	  end
	     
	if(t_uop.hilo_src_valid)
	  begin
	     t_alloc_uop.hilo_src = r_hilo_alloc_rat;
	  end
	else if(t_uop.fcr_src_valid)
	  begin
	     t_alloc_uop.hilo_src = r_fcr_alloc_rat;
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
     end // always_comb


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
	     n_branch_pc = t_retire_two ? t_rob_next_head.pc : t_rob_head.pc;
	     n_took_branch = t_retire_two ? t_rob_next_head.take_br : t_rob_head.take_br;
	     n_branch_valid = t_retire_two ? t_rob_next_head.is_br :  t_rob_head.is_br;
	     n_branch_fault = t_rob_head.faulted;
	     n_branch_pht_idx = t_retire_two ? t_rob_next_head.pht_idx : t_rob_head.pht_idx;
	  end // if (t_retire)
	
     end // always_comb
   
   
   always_comb
     begin
	n_rob = r_rob;
	t_rob_tail.complete  = 1'b0;
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
	t_rob_tail.take_trap = 1'b0;
	t_rob_tail.exception_tlb_refill = 1'b0;
	t_rob_tail.exception_tlb_modified = 1'b0;
	t_rob_tail.exception_tlb_invalid = 1'b0;
	t_rob_tail.is_store = 1'b0;
	t_rob_tail.is_call = 1'b0;
	t_rob_tail.is_ret = 1'b0;
	t_rob_tail.is_break = 1'b0;
	t_rob_tail.is_syscall = 1'b0;
	t_rob_tail.take_br = 1'b0;
	t_rob_tail.is_br = 1'b0;
	t_rob_tail.is_indirect = 1'b0;
	t_rob_tail.in_delay_slot = r_in_delay_slot;
	t_rob_tail.data = {`M_WIDTH{1'b0}};
	t_rob_tail.pht_idx = 'd0;
	
	if(t_alloc)
	  begin
	     t_rob_tail.pc = t_alloc_uop.pc;
	     t_rob_tail.is_br = t_alloc_uop.is_br;
	     t_rob_tail.has_delay_slot = t_alloc_uop.has_delay_slot;
	     t_rob_tail.has_nullifying_delay_slot = t_alloc_uop.has_nullifying_delay_slot;
	     t_rob_tail.pht_idx = t_alloc_uop.pht_idx;
	     t_rob_tail.is_store = t_alloc_uop.is_store;
	     
	     t_rob_tail.is_call = t_alloc_uop.op == JAL || t_alloc_uop.op == JALR || t_alloc_uop.op == BAL;
	     t_rob_tail.is_ret = (t_alloc_uop.op == JR) && (t_uop.srcA == 'd31);
	     t_rob_tail.is_syscall = (t_alloc_uop.op == SYSCALL);
	     t_rob_tail.is_break  = (t_alloc_uop.op == BREAK);
	     t_rob_tail.is_eret = 1'b0;
	     t_rob_tail.is_indirect = t_alloc_uop.op == JALR || t_alloc_uop.op == JR;
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
		  t_rob_tail.complete  = 1'b1;
		  if(t_uop.op == II)
		    begin
		       t_rob_tail.faulted = 1'b1;
		       t_rob_tail.is_ii = 1'b1;
		    end
		  else if(t_uop.op == ERET)
		    begin
		       t_rob_tail.faulted = 1'b1;
		       t_rob_tail.is_eret = 1'b1;
		    end
		  else if(t_uop.op == J)
		    begin
		       t_rob_tail.take_br = 1'b1;
		    end
	       end
	     
	     n_rob[r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]] = t_rob_tail;
	  end // if (t_alloc)
	if(t_complete_valid_1)
	  begin
	     n_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].complete = t_complete_bundle_1.complete;
	     n_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].faulted = t_complete_bundle_1.faulted;
	     n_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].target_pc = t_complete_bundle_1.restart_pc;
	     n_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].is_ii = t_complete_bundle_1.is_ii;
	     n_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_trap = t_complete_bundle_1.take_trap;	     
	     n_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_br = t_complete_bundle_1.take_br;
	     n_rob[t_complete_bundle_1.rob_ptr[`LG_ROB_ENTRIES-1:0]].data = t_complete_bundle_1.data;
	  end
	if(t_complete_valid_2)
	  begin
	     //if(r_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].complete)
	     //begin
	     //$display("ROB entry %d is already complete!", t_complete_bundle_2.rob_ptr);
	     //$stop();
	     //end
	     n_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].complete = t_complete_bundle_2.complete;
	     n_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].faulted = t_complete_bundle_2.faulted;
	     n_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].target_pc = t_complete_bundle_2.restart_pc;
	     n_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].is_ii = t_complete_bundle_2.is_ii;
	     n_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_br = t_complete_bundle_2.take_br;
	     n_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].take_trap = t_complete_bundle_2.take_trap;	     
	     n_rob[t_complete_bundle_2.rob_ptr[`LG_ROB_ENTRIES-1:0]].data = t_complete_bundle_2.data;
	  end
	if(core_mem_rsp_valid)
	  begin
	     n_rob[core_mem_rsp.rob_ptr].complete = 1'b1;
	     n_rob[core_mem_rsp.rob_ptr].data = core_mem_rsp.data[`M_WIDTH-1:0];
	     n_rob[core_mem_rsp.rob_ptr].faulted = core_mem_rsp.faulted;
	     n_rob[core_mem_rsp.rob_ptr].exception_tlb_refill = core_mem_rsp.exception_tlb_refill;
	     n_rob[core_mem_rsp.rob_ptr].exception_tlb_modified = core_mem_rsp.exception_tlb_modified;
	     n_rob[core_mem_rsp.rob_ptr].exception_tlb_invalid = core_mem_rsp.exception_tlb_invalid;
	     
	  end
     end // always_comb
   


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i < N_ROB_ENTRIES; i=i+1)
	       begin
		  r_rob[i].complete <= 1'b0;
		  r_rob[i].faulted <= 1'b0;
		  r_rob[i].valid_dst <= 1'b0;
		  r_rob[i].ldst <= 'd0;
		  r_rob[i].pdst <= 'd0;
		  r_rob[i].old_pdst <= 'd0;
	       end
	  end
	else
	  begin
	     if(t_clr_rob)
	       begin
		  for(integer i = 0; i < N_ROB_ENTRIES; i=i+1)
		    begin
		       r_rob[i].complete <= 1'b0;
		       r_rob[i].faulted <= 1'b0;
		    end
	       end
	     else
	       begin
		  r_rob <= n_rob;
	       end
	  end
     end


   //always_ff@(negedge clk)
   //begin
   //if((r_rob_next_head_ptr != t_next_rob_head_ptr))
   //begin
   //$display("reset = %d, r_rob_next_head_ptr = %d, t_next_rob_head_ptr = %d",
   //reset, r_rob_next_head_ptr, t_next_rob_head_ptr);
   //$stop();
   //end
   //end
    
   
   always_comb
     begin
	n_rob_head_ptr = r_rob_head_ptr;
	n_rob_tail_ptr = r_rob_tail_ptr;
	n_rob_next_head_ptr = r_rob_next_head_ptr;
	
	n_insns_in_rob_cnt = r_insns_in_rob_cnt;
	
	if(t_clr_rob)
	  begin
	     n_rob_head_ptr = 'd0;
	     n_rob_tail_ptr = 'd0;
	     n_rob_next_head_ptr = 'd1;
	     n_insns_in_rob_cnt = 'd0;
	  end
	else
	  begin
	     if(t_alloc)
	       begin
		  n_rob_tail_ptr = r_rob_tail_ptr + 'd1;
	       end
	     if(t_retire || t_bump_rob_head)
	       begin
		  n_rob_head_ptr = t_retire_two ? r_rob_head_ptr + 'd2 : 
				   r_rob_head_ptr + 'd1;
		  n_rob_next_head_ptr = t_retire_two ? r_rob_next_head_ptr + 'd2 : 
					r_rob_next_head_ptr + 'd1;
	       end
	     
	     if(t_retire_two)
	       begin
		  if(t_alloc)
		    begin
		       n_insns_in_rob_cnt = r_insns_in_rob_cnt - 'd1;		  
		    end
		  else
		    begin
		       n_insns_in_rob_cnt = r_insns_in_rob_cnt - 'd2;
		    end
	       end
	     else
	       begin
		  if(t_alloc && !(t_retire || t_bump_rob_head))
		    begin
		       n_insns_in_rob_cnt = r_insns_in_rob_cnt + 'd1;
		    end
		  else if(!t_alloc && (t_retire || t_bump_rob_head))
		    begin
		       n_insns_in_rob_cnt = r_insns_in_rob_cnt - 'd1;		  
		    end
	       end // else: !if(t_retire_two)
	  end
	t_rob_empty = (r_rob_head_ptr == r_rob_tail_ptr);
	t_rob_full = (r_rob_head_ptr[`LG_ROB_ENTRIES-1:0] == r_rob_tail_ptr[`LG_ROB_ENTRIES-1:0]) && (r_rob_head_ptr != r_rob_tail_ptr);
     end // always_comb


   always_comb
     begin
	t_rob_head = r_rob[r_rob_head_ptr[`LG_ROB_ENTRIES-1:0]];
	t_rob_next_head = r_rob[r_rob_next_head_ptr[`LG_ROB_ENTRIES-1:0]];
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
   

   find_first_set#(`LG_PRF_ENTRIES) ffs_fp(.in(r_fp_prf_free),
					   .y(t_fp_ffs));
   
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


   
   always_comb
     begin
	n_fp_prf_free = r_fp_prf_free;
	n_fp_prf_entry = t_fp_ffs[`LG_PRF_ENTRIES-1:0];
	if(t_alloc && t_uop.fp_dst_valid)
	  begin
	     n_fp_prf_free[n_fp_prf_entry] = 1'b0;
	  end
	if(t_free_fp_reg)
	  begin
	     n_fp_prf_free[t_free_fp_reg_ptr] = 1'b1;
	  end
     end   
   
   find_first_set#(`LG_PRF_ENTRIES) ffs_gpr(.in(r_prf_free),
					    .y(t_gpr_ffs));

   
   always_comb
     begin
	n_prf_free = r_prf_free;
	n_prf_entry = t_gpr_ffs[`LG_PRF_ENTRIES-1:0];
	
	if(t_alloc & t_uop.dst_valid)
	  begin
	     n_prf_free[n_prf_entry] = 1'b0;
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

   
   decode_mips32 dec (.in_64b_fpreg_mode(t_in_32fp_reg_mode),
		      .insn(insn.data), .pc(insn.pc), 
		      .insn_pred(insn.pred), .pht_idx(insn.pht_idx),
		      .insn_pred_target(insn.pred_target),
		      .uop(t_dec_uop));
   

   always_comb
     begin
	t_any_complete = t_complete_valid_1 | t_complete_valid_2 | core_mem_rsp_valid;
     end
	
   exec e (
	   .clk(clk), 
	   .reset(reset),
	   .machine_clr(r_machine_clr),
	   .delayslot_rob_ptr(r_delayslot_rob_ptr),
	   .in_32fp_reg_mode(t_in_32fp_reg_mode),
	   .uq_empty(t_uq_empty),
	   .uq_full(t_uq_full),	   
	   .uq_uop(t_alloc_uop),
	   .uq_push(t_alloc && !t_fold_uop),
	   .complete_bundle_1(t_complete_bundle_1),
	   .complete_valid_1(t_complete_valid_1),
	   .complete_bundle_2(t_complete_bundle_2),
	   .complete_valid_2(t_complete_valid_2),
	   .exception_wr_cpr0_val(t_exception_wr_cpr0_val),
	   .exception_wr_cpr0_ptr(t_exception_wr_cpr0_ptr),
	   .exception_wr_cpr0_data(t_exception_wr_cpr0_data),
	   .mem_req(t_mem_req),
	   .mem_req_valid(t_mem_req_valid),
	   .mem_req_ack(core_mem_req_ack),
	   .mem_rsp_dst_ptr(core_mem_rsp.dst_ptr),
	   .mem_rsp_dst_valid(core_mem_rsp.dst_valid),
	   .mem_rsp_fp_dst_valid(core_mem_rsp.fp_dst_valid),
	   .mem_rsp_load_data(core_mem_rsp.data),
	   .monitor_rsp_data(r_monitor_rsp_data),
	   .monitor_rsp_data_valid(r_monitor_rsp_data_valid)
	   );


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_dq_head_ptr <= 'd0;
	     r_dq_tail_ptr <= 'd0;
	  end
	else
	  begin
	     r_dq_head_ptr <= t_clr_rob ? 'd0 :n_dq_head_ptr;
	     r_dq_tail_ptr <= t_clr_rob ? 'd0 :n_dq_tail_ptr;
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	r_dq <= n_dq;
     end

   
   always_comb
     begin
	n_dq = r_dq;
	n_dq_tail_ptr = r_dq_tail_ptr;
	n_dq_head_ptr = r_dq_head_ptr;
	t_dq_empty = (r_dq_tail_ptr == r_dq_head_ptr);
	t_dq_full = (r_dq_tail_ptr[`LG_DQ_ENTRIES-1:0] == r_dq_head_ptr[`LG_DQ_ENTRIES-1:0]) && (r_dq_tail_ptr != r_dq_head_ptr);
	
	t_uop = r_dq[r_dq_head_ptr[`LG_DQ_ENTRIES-1:0]];
	if(t_clr_dq)
	  begin
	     n_dq_tail_ptr = 'd0;
	     n_dq_head_ptr = 'd0;
	  end
	else
	  begin
	     if(insn_valid && !t_dq_full)
	       begin
		  n_dq[r_dq_tail_ptr[`LG_DQ_ENTRIES-1:0]] = t_dec_uop;
		  n_dq_tail_ptr = r_dq_tail_ptr + 'd1;
	       end
	     if(t_alloc)
	       begin
		  n_dq_head_ptr = r_dq_head_ptr + 'd1;
	       end
	  end
     end // always_comb


   //always_ff@(negedge clk)
   //begin
   //if(t_alloc)
   //$display("allocating opcode %d for pc %x\n", t_uop.op, t_uop.pc);
   //end
   // always_ff@(negedge clk)
   //   begin
   // 	if(t_clr_dq)
   // 	  begin
   // 	     $display("clearing dq at cycle %d\n", r_cycle);
   // 	  end
   // 	if(t_dq_empty)
   // 	  begin
   // 	     $display("dq empty at cycle %d\n", r_cycle);
   // 	  end
   //   end // always_ff@ (negedge clk)
   
    // always_ff@(negedge clk)
    //   begin
    // 	 $display("n_dq_head_ptr = %d, n_dq_tail_ptr = %d, full = %b, empty = %b, t_alloc = %b, t_clr_dq = %b, alloc pc %x, cycle %d\n",
    // 		  n_dq_head_ptr, n_dq_tail_ptr, t_dq_full, t_dq_empty, t_alloc, t_clr_dq, t_uop.pc, r_cycle);
    //   end
   


   
   // initial
   //   begin
   // 	$dumpfile("mips32.vcd");
   // 	$dumpvars();
   //   end
   
endmodule
