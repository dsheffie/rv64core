`include "uop.vh"
`include "rob.vh"

`ifdef VERILATOR
import "DPI-C" function void csr_putchar(input byte x);
import "DPI-C" function longint csr_gettime();
import "DPI-C" function void term_sim();

import "DPI-C" function void report_exec(input int int_valid, 
					 input int int_blocked,
					 input int int2_valid, 
					 input int int2_blocked, 
					 input int mem_valid, 
					 input int mem_blocked,
					 input int fp_valid, 
					 input int fp_blocked,
					 input int iq_full,
					 input int mq_full,
					 input int fq_full,
					 input int blocked_by_store,
					 input int int_ready,
					 input int int_ready2
					 );
`endif

module exec(clk, 
	    reset,
	    putchar_fifo_out,
	    putchar_fifo_empty,
	    putchar_fifo_pop,
	    cause,
	    epc,
	    tval,
	    irq,
	    mie,
	    mip,
	    mideleg,
	    mstatus,
	    exc_pc,
	    update_csr_exc,
	    priv,
	    priv_update,
	    page_table_root,
	    paging_active,
	    clear_tlb,
	    mode64,
	    retire,
	    retire_two,
`ifdef VERILATOR
	    clear_cnt,
`endif
	    divide_ready,
	    ds_done,
	    mem_dq_clr,
	    restart_complete,
	    uq_wait,
	    mq_wait,
	    uq_full,
	    uq_next_full,
	    uq_uop,
	    uq_uop_two,
	    uq_push,
	    uq_push_two,
	    complete_bundle_1,
	    complete_valid_1,
	    complete_bundle_2,
	    complete_valid_2,
	    mem_req, 
	    mem_req_valid, 
	    mem_req_ack,
	    core_store_data_valid,
	    core_store_data,
	    core_store_data_ack,
	    //tell rob store data has been read
	    core_store_data_ptr,
	    core_store_data_ptr_valid,
	    mem_rsp_dst_ptr,
	    mem_rsp_dst_valid,
	    mem_rsp_load_data,
	    branch_valid,
	    branch_fault);
   input logic clk;
   input logic reset;
   output logic [7:0] putchar_fifo_out;
   output logic       putchar_fifo_empty;
   input logic 	      putchar_fifo_pop;
   
   output logic [1:0] priv;
   output logic       priv_update;
   input	      cause_t cause;
   input logic [63:0] epc;
   input logic [63:0] tval;
   input logic 	      irq;
   output logic [63:0] mip;
   output logic [63:0] mie;
   output logic [63:0] mideleg;
   output logic [63:0] mstatus;
   
   output logic [63:0] exc_pc;
   input logic	       update_csr_exc;
   output logic [63:0] page_table_root;
   
   output logic	       paging_active;
   
   output logic	      clear_tlb;
   input logic	      mode64;
   input logic	      retire;
   input logic	      retire_two;
   
`ifdef VERILATOR
   input logic [31:0] clear_cnt;
`endif
   output logic       divide_ready;
   input logic ds_done;
   input logic mem_dq_clr;
   input logic restart_complete;
   
   localparam N_ROB_ENTRIES = (1<<`LG_ROB_ENTRIES);   
   output logic [N_ROB_ENTRIES-1:0]  uq_wait;   
   output logic [N_ROB_ENTRIES-1:0]  mq_wait;
   
   output logic 			     uq_full;
   output logic 			     uq_next_full;
   
   input 				     uop_t uq_uop;
   input 				     uop_t uq_uop_two;
   
   input logic 				     uq_push;
   input logic 				     uq_push_two;
   
   output 	complete_t complete_bundle_1;
   output logic complete_valid_1;
   output 	complete_t complete_bundle_2;
   output logic complete_valid_2;   


   output 	mem_req_t mem_req;
   output 	logic mem_req_valid;
   input logic 	      mem_req_ack;

   output logic 	      core_store_data_valid;
   output 		      mem_data_t core_store_data;
   input logic 		      core_store_data_ack;
   
   output logic [`LG_ROB_ENTRIES-1:0] core_store_data_ptr;
   output logic 		      core_store_data_ptr_valid;
   
   
   input logic [`LG_PRF_ENTRIES-1:0] mem_rsp_dst_ptr;
   input logic 			     mem_rsp_dst_valid;
   input logic [`M_WIDTH-1:0] 	     mem_rsp_load_data;

   input logic 			     branch_valid;
   input logic 			     branch_fault;
   
   
   localparam N_INT_SCHED_ENTRIES = 1<<`LG_INT_SCHED_ENTRIES;
   
   localparam N_MQ_ENTRIES = (1<<`LG_MQ_ENTRIES);
   localparam N_INT_PRF_ENTRIES = (1<<`LG_PRF_ENTRIES);
   
   localparam N_UQ_ENTRIES = (1<<`LG_UQ_ENTRIES);
   localparam N_MEM_UQ_ENTRIES = (1<<`LG_MEM_UQ_ENTRIES);
   localparam N_MEM_DQ_ENTRIES = (1<<`LG_MEM_DQ_ENTRIES);
   
   logic [N_INT_PRF_ENTRIES-1:0]  r_prf_inflight;
   
   logic 			  t_wr_int_prf, t_wr_int_prf2;
   logic			  r_clear_tlb, t_clear_tlb;
   logic [1:0]			  r_priv, n_priv;
   assign priv = r_priv;
   assign clear_tlb = r_clear_tlb;
   
   logic 	t_take_br, t_take_br2;
   logic 	t_mispred_br, t_mispred_br2;
   logic 	t_alu_valid, t_alu_valid2;
   logic 	t_got_break;
   
   mem_req_t r_mem_q[N_MQ_ENTRIES-1:0] ;
   logic [`LG_MQ_ENTRIES:0] r_mq_head_ptr, n_mq_head_ptr;
   logic [`LG_MQ_ENTRIES:0] r_mq_tail_ptr, n_mq_tail_ptr;
   logic [`LG_MQ_ENTRIES:0] r_mq_next_tail_ptr, n_mq_next_tail_ptr;
   mem_req_t t_mem_tail, t_mem_head;
   logic 		    mem_q_full,mem_q_next_full, mem_q_empty;


   mem_data_t r_mdq[N_MQ_ENTRIES-1:0];
   mem_data_t t_mdq_tail, t_mdq_head;
   
   logic [`LG_MQ_ENTRIES:0] r_mdq_head_ptr, n_mdq_head_ptr;
   logic [`LG_MQ_ENTRIES:0] r_mdq_tail_ptr, n_mdq_tail_ptr;
   logic [`LG_MQ_ENTRIES:0] r_mdq_next_tail_ptr, n_mdq_next_tail_ptr;
   logic 		    mem_mdq_full,mem_mdq_next_full, mem_mdq_empty;
   
   wire [`LG_PRF_ENTRIES-1:0] w_mul_prf_ptr;
   logic [`LG_PRF_ENTRIES-1:0] r_mul_prf_ptr;
   logic 		       r_mul_complete;
   wire [`LG_PRF_ENTRIES-1:0] w_div_prf_ptr;
   logic [`LG_PRF_ENTRIES-1:0] r_div_prf_ptr;
   logic 		       r_div_complete;

   logic 	t_pop_uq,t_pop_mem_uq,t_pop_mem_dq,t_pop_uq2;
   
   logic 	r_mem_ready, r_dq_ready;
   logic	r_paging_active;
   
   localparam E_BITS = `M_WIDTH-16;
   localparam HI_EBITS = `M_WIDTH-32;
   
   logic [`M_WIDTH-1:0] t_result, t_result2;
   wire [`M_WIDTH-2:0] 	w_zf = 'd0;
      
   
   logic [`M_WIDTH-1:0] t_pc, t_pc_2;
   logic 	t_srcs_rdy;

   
   wire [`M_WIDTH-1:0] w_srcA, w_srcB;
   logic [`M_WIDTH-1:0] t_srcA_2, t_srcB_2;
   wire [`M_WIDTH-1:0] 	w_srcA_2, w_srcB_2;
   
   wire [`M_WIDTH-1:0] w_mem_srcA, w_mem_srcB;
   
   logic [`M_WIDTH-1:0] r_mem_result, r_int_result, r_int_result2;
   
   logic 	r_fwd_int_srcA, r_fwd_int_srcB, r_fwd_int2_srcA, r_fwd_int2_srcB;
   logic 	r_fwd_int_srcA2, r_fwd_int_srcB2, r_fwd_int2_srcA2, r_fwd_int2_srcB2;
   
   logic 	r_fwd_mem_srcA, r_fwd_mem_srcB, r_fwd_mem_srcA2, r_fwd_mem_srcB2;

   logic t_fwd_int_mem_srcA,t_fwd_int_mem_srcB,t_fwd_int2_mem_srcA, t_fwd_int2_mem_srcB, 
	 t_fwd_mem_mem_srcA,t_fwd_mem_mem_srcB;
   logic r_fwd_int_mem_srcA,r_fwd_int_mem_srcB,r_fwd_int2_mem_srcA,r_fwd_int2_mem_srcB,
	 r_fwd_mem_mem_srcA,r_fwd_mem_mem_srcB;
   
      
   logic [`M_WIDTH-1:0] t_srcA, t_srcB;
   logic [`M_WIDTH-1:0] t_mem_srcA, t_mem_srcB;
   
   
   
   logic 	t_has_cause;
   cause_t      t_cause;
   
   logic	t_wr_csr_en, t_rd_csr_en;
   logic [63:0] t_rd_csr, t_wr_csr;
   logic	t_wr_priv;
   logic [1:0] 	t_priv;
   

   logic [63:0]	r_stvec, r_sscratch;
   logic [63:0]	r_sepc, r_stval;
   logic [63:0]	r_satp, r_mstatus, r_mideleg, r_medeleg;
   logic [63:0]	r_mcounteren, r_mie, r_mscratch, r_mepc;
   logic [63:0]	r_mtvec, r_mtval, r_misa, r_mip,  r_scounteren;
   logic [63:0]	r_mcause, r_scause;
   logic [63:0]	r_pmpaddr0, r_pmpaddr1, r_pmpaddr2, r_pmpaddr3, r_pmpcfg0;
   
   assign mie = r_mie;
   assign mip = r_mip;
   assign mideleg = r_mideleg;
   assign mstatus = r_mstatus;
   
   logic 	t_signed_shift;
   logic 	t_left_shift;

   logic	t_zero_shift_upper;
   logic [5:0] 	t_shift_amt;
   wire [`M_WIDTH-1:0] w_shifter_out;

   logic	       t_start_mul,t_is_mulw,t_signed_mul;

   logic 	t_mul_complete;
   logic [`M_WIDTH-1:0] t_mul_result;
   
   logic [`LG_ROB_ENTRIES-1:0] t_rob_ptr_out;

   
   
   logic [`MAX_LAT:0] r_wb_bitvec, n_wb_bitvec;

   /* divider */
   logic 	t_div_ready, t_signed_div, t_is_rem, t_start_div32, t_start_div64;
   logic [`LG_ROB_ENTRIES-1:0] t_div_rob_ptr;
   logic [`M_WIDTH-1:0]        t_div_result;
   logic 			    t_div_complete;

   logic [N_ROB_ENTRIES-1:0] 	    r_uq_wait, r_mq_wait;
   /* non mem uop queue */
   uop_t r_uq[N_UQ_ENTRIES];
   uop_t uq, uq2, int_uop, int_uop2;
   logic 			    r_start_int2;
   logic 			    r_start_int;
   
   
   
   logic 			    t_uq_read, t_uq_empty, t_uq_full, t_uq_next_full, t_uq_next_empty;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_head_ptr, n_uq_head_ptr;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_tail_ptr, n_uq_tail_ptr;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_next_head_ptr, n_uq_next_head_ptr;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_next_tail_ptr, n_uq_next_tail_ptr;

   /* mem uop queue */
   uop_t r_mem_uq[N_MEM_UQ_ENTRIES];
   uop_t t_mem_uq, mem_uq;
   logic 	      t_mem_uq_read, t_mem_uq_empty, t_mem_uq_full,
		      t_mem_uq_next_full;
   
   logic [`LG_MEM_UQ_ENTRIES:0]  r_mem_uq_head_ptr, n_mem_uq_head_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0]  r_mem_uq_tail_ptr, n_mem_uq_tail_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0] r_mem_uq_next_head_ptr, n_mem_uq_next_head_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0] r_mem_uq_next_tail_ptr, n_mem_uq_next_tail_ptr;

   /* mem data queue */
   dq_t r_mem_dq[N_MEM_DQ_ENTRIES];
   dq_t t_dq0, t_dq1, t_mem_dq, mem_dq;
   mem_data_t t_core_store_data;
   
   logic 	      t_mem_dq_read, t_mem_dq_empty, t_mem_dq_full,
		      t_mem_dq_next_full;
   
   logic [`LG_MEM_DQ_ENTRIES:0]  r_mem_dq_head_ptr, n_mem_dq_head_ptr;
   logic [`LG_MEM_DQ_ENTRIES:0]  r_mem_dq_tail_ptr, n_mem_dq_tail_ptr;
   logic [`LG_MEM_DQ_ENTRIES:0] r_mem_dq_next_head_ptr, n_mem_dq_next_head_ptr;
   logic [`LG_MEM_DQ_ENTRIES:0] r_mem_dq_next_tail_ptr, n_mem_dq_next_tail_ptr;


   
   logic             t_push_two_mem,  t_push_two_int;
   logic             t_push_one_mem,  t_push_one_int;
   logic 	     t_push_two_dq, t_push_one_dq;
   
   logic 			t_flash_clear;
   always_comb
     begin
	t_flash_clear = ds_done;
     end

   always_comb
     begin
	uq_full = t_uq_full || t_mem_uq_full || t_mem_dq_full;
	uq_next_full = t_uq_next_full || t_mem_uq_next_full || t_mem_dq_next_full;
     end
   
   always_ff@(posedge clk)
     begin
	if(reset || t_flash_clear)
	  begin
	     r_uq_head_ptr <= 'd0;
	     r_uq_tail_ptr <= 'd0;
	     r_uq_next_head_ptr <= 'd1;
	     r_uq_next_tail_ptr <= 'd1;	     
	  end
	else
	  begin
	     r_uq_head_ptr <=  n_uq_head_ptr;
	     r_uq_tail_ptr <=  n_uq_tail_ptr;
	     r_uq_next_head_ptr <= n_uq_next_head_ptr;
	     r_uq_next_tail_ptr <= n_uq_next_tail_ptr;	     
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	if(reset  || t_flash_clear)
	  begin
	     r_mem_uq_head_ptr <= 'd0;
	     r_mem_uq_tail_ptr <= 'd0;
	     r_mem_uq_next_head_ptr <= 'd1;
	     r_mem_uq_next_tail_ptr <= 'd1;
	  end
	else
	  begin
	     r_mem_uq_head_ptr <= n_mem_uq_head_ptr;
	     r_mem_uq_tail_ptr <= n_mem_uq_tail_ptr;
	     r_mem_uq_next_head_ptr <= n_mem_uq_next_head_ptr;
	     r_mem_uq_next_tail_ptr <= n_mem_uq_next_tail_ptr;
	  end
     end // always_ff@ (posedge clk// )
   
   always_ff@(posedge clk)
     begin
	if(reset  || mem_dq_clr)
	  begin
	     r_mem_dq_head_ptr <= 'd0;
	     r_mem_dq_tail_ptr <= 'd0;
	     r_mem_dq_next_head_ptr <= 'd1;
	     r_mem_dq_next_tail_ptr <= 'd1;
	  end
	else
	  begin
	     r_mem_dq_head_ptr <= n_mem_dq_head_ptr;
	     r_mem_dq_tail_ptr <= n_mem_dq_tail_ptr;
	     r_mem_dq_next_head_ptr <= n_mem_dq_next_head_ptr;
	     r_mem_dq_next_tail_ptr <= n_mem_dq_next_tail_ptr;
	  end
     end // always_ff@ (posedge clk// )
   
   

   always_comb
     begin
	n_mem_uq_head_ptr = r_mem_uq_head_ptr;
	n_mem_uq_tail_ptr = r_mem_uq_tail_ptr;
	n_mem_uq_next_head_ptr = r_mem_uq_next_head_ptr;
	n_mem_uq_next_tail_ptr = r_mem_uq_next_tail_ptr;
	
	n_mem_dq_head_ptr = r_mem_dq_head_ptr;
	n_mem_dq_tail_ptr = r_mem_dq_tail_ptr;
	n_mem_dq_next_head_ptr = r_mem_dq_next_head_ptr;
	n_mem_dq_next_tail_ptr = r_mem_dq_next_tail_ptr;


	
	t_mem_uq_empty = (r_mem_uq_head_ptr == r_mem_uq_tail_ptr);
	t_mem_uq_full = (r_mem_uq_head_ptr != r_mem_uq_tail_ptr) && (r_mem_uq_head_ptr[`LG_MEM_UQ_ENTRIES-1:0] == r_mem_uq_tail_ptr[`LG_MEM_UQ_ENTRIES-1:0]);

	t_mem_uq_next_full = (r_mem_uq_head_ptr != r_mem_uq_next_tail_ptr) && 
			     (r_mem_uq_head_ptr[`LG_MEM_UQ_ENTRIES-1:0] == r_mem_uq_next_tail_ptr[`LG_MEM_UQ_ENTRIES-1:0]);

	t_mem_dq_empty = (r_mem_dq_head_ptr == r_mem_dq_tail_ptr);
	t_mem_dq_full = (r_mem_dq_head_ptr != r_mem_dq_tail_ptr) && (r_mem_dq_head_ptr[`LG_MEM_DQ_ENTRIES-1:0] == r_mem_dq_tail_ptr[`LG_MEM_DQ_ENTRIES-1:0]);

	t_mem_dq_next_full = (r_mem_dq_head_ptr != r_mem_dq_next_tail_ptr) && 
			     (r_mem_dq_head_ptr[`LG_MEM_DQ_ENTRIES-1:0] == r_mem_dq_next_tail_ptr[`LG_MEM_DQ_ENTRIES-1:0]);
		
	t_mem_uq = r_mem_uq[r_mem_uq_head_ptr[`LG_MEM_UQ_ENTRIES-1:0]];

	t_mem_dq = r_mem_dq[r_mem_dq_head_ptr[`LG_MEM_DQ_ENTRIES-1:0]];

	t_push_two_mem = uq_push && uq_push_two && uq_uop.is_mem && uq_uop_two.is_mem;
	t_push_one_mem = ((uq_push && uq_uop.is_mem) || (uq_push_two && uq_uop_two.is_mem)) && !t_push_two_mem;

	t_push_two_dq = uq_push && uq_push_two && 
			uq_uop.is_mem && uq_uop.srcB_valid && 
			uq_uop_two.is_mem && uq_uop_two.srcB_valid;
	
	t_push_one_dq = (uq_push_two && uq_uop_two.is_mem && uq_uop_two.srcB_valid) || 
			(uq_push && uq_uop.is_mem && uq_uop.srcB_valid);
	
	
	if(t_push_two_dq)
	  begin
	     n_mem_dq_tail_ptr = r_mem_dq_tail_ptr + 'd2;
	     n_mem_dq_next_tail_ptr = r_mem_dq_next_tail_ptr + 'd2;	     
	  end
	else if(t_push_one_dq)
	  begin
	     n_mem_dq_tail_ptr = r_mem_dq_tail_ptr + 'd1;
	     n_mem_dq_next_tail_ptr = r_mem_dq_next_tail_ptr + 'd1;
	  end
	
	/* these need work */
	if(t_push_two_mem)
	  begin
	     n_mem_uq_tail_ptr = r_mem_uq_tail_ptr + 'd2;
	     n_mem_uq_next_tail_ptr = r_mem_uq_next_tail_ptr + 'd2;

	  end
	else if(uq_push_two && uq_uop_two.is_mem || uq_push && uq_uop.is_mem)
	  begin
	     n_mem_uq_tail_ptr = r_mem_uq_tail_ptr + 'd1;
	     n_mem_uq_next_tail_ptr = r_mem_uq_next_tail_ptr + 'd1;
	  end
	
	if(t_pop_mem_uq)
	  begin
	     n_mem_uq_head_ptr = r_mem_uq_head_ptr + 'd1;
	  end
	if(t_pop_mem_dq)
	  begin
	     n_mem_dq_head_ptr = r_mem_dq_head_ptr + 'd1;
	  end
     end // always_comb

   always_ff@(posedge clk)
     begin
	mem_uq <= t_mem_uq;
	mem_dq <= t_mem_dq;
     end

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mq_wait <= 'd0;
	     r_uq_wait <= 'd0;
	  end
	else if(restart_complete)
	  begin
	     r_mq_wait <= 'd0;
	     r_uq_wait <= 'd0;
	  end
	else
	  begin
	     //mem port
	     if(t_push_two_mem)
	       begin
		  r_mq_wait[uq_uop_two.rob_ptr] <= 1'b1;
		  r_mq_wait[uq_uop.rob_ptr] <= 1'b1;
	       end
	     else if(t_push_one_mem)
	       begin
		  r_mq_wait[uq_uop.is_mem ? uq_uop.rob_ptr : uq_uop_two.rob_ptr] <= 1'b1; 
	       end
	     if(t_pop_mem_uq)
	       begin
		  r_mq_wait[t_mem_uq.rob_ptr] <= 1'b0;		  
	       end
	     
	     //int port
	     if(t_push_two_int)
	       begin
		  r_uq_wait[uq_uop.rob_ptr] <= 1'b1;
		  r_uq_wait[uq_uop_two.rob_ptr] <= 1'b1;
	       end
	     else if(t_push_one_int)
	       begin
		  r_uq_wait[uq_uop.is_int ? uq_uop.rob_ptr : uq_uop_two.rob_ptr] <= 1'b1; 
	       end
	     
	     if(r_start_int)
	       begin
		  r_uq_wait[int_uop.rob_ptr] <= 1'b0;
	       end

	  end // else: !if(reset)
     end // always_ff@ (posedge clk)

   
   always_ff@(posedge clk)
     begin
	if(t_push_two_mem)
	  begin
	     //$display("cycle %d : pushing mem ops for rob slots %d & %d", r_cycle, uq_uop_two.rob_ptr, uq_uop.rob_ptr);
	     r_mem_uq[r_mem_uq_next_tail_ptr[`LG_MEM_UQ_ENTRIES-1:0]] <= uq_uop_two;
	     r_mem_uq[r_mem_uq_tail_ptr[`LG_MEM_UQ_ENTRIES-1:0]] <= uq_uop;
	  end
	else if(t_push_one_mem)
	  begin
	     //$display("cycle %d : pushing mem ops for rob slots %d", r_cycle, uq_uop.rob_ptr);
	     r_mem_uq[r_mem_uq_tail_ptr[`LG_MEM_UQ_ENTRIES-1:0]] <= uq_uop.is_mem ? uq_uop : uq_uop_two;
	  end	
     end // always_ff@ (posedge clk)


   always_comb     
     begin
	t_dq0.rob_ptr = uq_uop.rob_ptr;	
	t_dq0.src_ptr = uq_uop.srcB;
	t_dq1.rob_ptr = uq_uop_two.rob_ptr;
	t_dq1.src_ptr = uq_uop_two.srcB;
`ifdef ENABLE_CYCLE_ACCOUNTING
	t_dq0.fetch_cycle = uq_uop.fetch_cycle;
	t_dq1.fetch_cycle = uq_uop_two.fetch_cycle;
`endif
     end

       

   
   always_ff@(posedge clk)
     begin
	if(t_push_two_dq)
	  begin
	     r_mem_dq[r_mem_dq_next_tail_ptr[`LG_MEM_DQ_ENTRIES-1:0]] <= t_dq1;
	     r_mem_dq[r_mem_dq_tail_ptr[`LG_MEM_DQ_ENTRIES-1:0]] <= t_dq0;
	  end
	else if(t_push_one_dq)
	  begin
	     r_mem_dq[r_mem_dq_tail_ptr[`LG_MEM_DQ_ENTRIES-1:0]] <= uq_uop.is_mem && uq_uop.srcB_valid ? t_dq0 : t_dq1;
	  end	
     end
   
   

   
   always_comb
     begin
	n_uq_head_ptr = r_uq_head_ptr;
	n_uq_tail_ptr = r_uq_tail_ptr;
	n_uq_next_head_ptr = r_uq_next_head_ptr;
	n_uq_next_tail_ptr = r_uq_next_tail_ptr;
	
	
	t_uq_empty = (r_uq_head_ptr == r_uq_tail_ptr);
	t_uq_next_empty = (r_uq_next_head_ptr == r_uq_tail_ptr);
	
	t_uq_full = (r_uq_head_ptr != r_uq_tail_ptr) && 
		    (r_uq_head_ptr[`LG_UQ_ENTRIES-1:0] == r_uq_tail_ptr[`LG_UQ_ENTRIES-1:0]);

	t_uq_next_full = (r_uq_head_ptr != r_uq_next_tail_ptr) && 
			 (r_uq_head_ptr[`LG_UQ_ENTRIES-1:0] == r_uq_next_tail_ptr[`LG_UQ_ENTRIES-1:0]);

	t_push_two_int = uq_push && uq_push_two && uq_uop.is_int && uq_uop_two.is_int;
	t_push_one_int = ((uq_push && uq_uop.is_int) || (uq_push_two && uq_uop_two.is_int)) && !t_push_two_int;
	
	uq = r_uq[r_uq_head_ptr[`LG_UQ_ENTRIES-1:0]];
	uq2 = r_uq[r_uq_next_head_ptr[`LG_UQ_ENTRIES-1:0]];

	
	if(t_push_two_int)
	  begin	     
	     n_uq_tail_ptr = r_uq_tail_ptr + 'd2;
	     n_uq_next_tail_ptr = r_uq_next_tail_ptr + 'd2;
	  end
	else if(uq_push_two && uq_uop_two.is_int || uq_push && uq_uop.is_int)
	  begin	     
	     n_uq_tail_ptr = r_uq_tail_ptr + 'd1;
	     n_uq_next_tail_ptr = r_uq_next_tail_ptr + 'd1;
	  end

	if(t_pop_uq2)
	  begin
	     n_uq_next_head_ptr = r_uq_next_head_ptr + 'd2;
	     n_uq_head_ptr = r_uq_head_ptr + 'd2;
	  end
	else if(t_pop_uq)
	  begin
	     n_uq_head_ptr = r_uq_head_ptr + 'd1;
	     n_uq_next_head_ptr = r_uq_next_head_ptr + 'd1;	     
	  end
     end // always_comb

   always_ff@(posedge clk)
     begin
	if(t_push_two_int)
	  begin
	     //if(uq_uop.op == RET) $display("pushing RET with srcA = %x", uq_uop.srcA);
	     //if(uq_uop_two.op == RET) $display("pushing RET with srcA = %x", uq_uop_two.srcA);
	     r_uq[r_uq_tail_ptr[`LG_UQ_ENTRIES-1:0]] <= uq_uop;
	     r_uq[r_uq_next_tail_ptr[`LG_UQ_ENTRIES-1:0]] <= uq_uop_two;	     
	  end
	else if(t_push_one_int)
	  begin
	     //if(uq_uop.op == RET) $display("pushing RET with srcA = %x, pc %x", 
	     //uq_uop.is_int ? uq_uop.srcA : uq_uop_two.srcA,
	     //uq_uop.is_int ? uq_uop.pc : uq_uop_two.pc,
	     //);
	     r_uq[r_uq_tail_ptr[`LG_UQ_ENTRIES-1:0]] <= uq_uop.is_int ? uq_uop : uq_uop_two;
	  end
	
     end // always_ff@ (posedge clk)
   
   logic [63:0]        r_cycle, r_retired_insns, r_branches, r_branch_faults;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : r_cycle + 'd1;
	r_branches <= reset ? 'd0 : branch_valid ? (r_branches + 'd1) : r_branches;
	r_branch_faults <= reset ? 'd0 : branch_fault ? (r_branch_faults + 'd1) : r_branch_faults;
     end

   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_retired_insns <= 'd0;
	  end
	else if(retire_two)
	  begin
	     r_retired_insns <= r_retired_insns + 'd2;
	  end
	else if(retire) 
	  begin
	     r_retired_insns <= r_retired_insns + 'd1;
	  end
     end // always_ff@ (posedge clk)


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_wb_bitvec <= 'd0;
	  end
	else
	  begin
	     r_wb_bitvec <= n_wb_bitvec;
	  end
     end // always_ff@ (posedge clk)


   always_comb
     begin
	for(integer i = (`MAX_LAT-1); i > -1; i = i-1)
	  begin
	     n_wb_bitvec[i] = r_wb_bitvec[i+1];	     
	  end
	
	n_wb_bitvec[`DIV64_LAT] = t_start_div64&r_start_int;
	
	if(t_start_mul&r_start_int)
	  begin
	     n_wb_bitvec[`MUL_LAT] = 1'b1;
	  end
     end // always_comb

   
   
   always_comb
     begin
	t_srcA = r_fwd_int_srcA ? r_int_result :
		 r_fwd_int2_srcA ? r_int_result2 :
		 r_fwd_mem_srcA ? r_mem_result :
		 w_srcA;
	
	t_srcB = r_fwd_int_srcB ? r_int_result :
		 r_fwd_int2_srcB ? r_int_result2 :		 
		 r_fwd_mem_srcB ? r_mem_result :
		 w_srcB;

	t_srcA_2 = r_fwd_int_srcA2 ? r_int_result :
		   r_fwd_int2_srcA2 ? r_int_result2 :
		   r_fwd_mem_srcA2 ? r_mem_result :
		   w_srcA_2;

	t_srcB_2 = r_fwd_int_srcB2 ? r_int_result :
		   r_fwd_int2_srcB2 ? r_int_result2 :
		   r_fwd_mem_srcB2 ? r_mem_result :
		   w_srcB_2;
	
	
	t_mem_srcA = r_fwd_int_mem_srcA ? r_int_result :
		     r_fwd_mem_mem_srcA ? r_mem_result :
		     r_fwd_int2_mem_srcA ? r_int_result2 :
		     w_mem_srcA;

	t_mem_srcB = r_fwd_int_mem_srcB ? r_int_result :
		     r_fwd_mem_mem_srcB ? r_mem_result :
		     r_fwd_int2_mem_srcB ? r_int_result2 :
		     w_mem_srcB;	
     end // always_comb




   //does this scheduler entry contain a valid uop?
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_sched_valid, r_alu_sched_valid2;
   logic [`LG_INT_SCHED_ENTRIES:0] t_alu_sched_alloc_ptr, t_alu_sched_alloc_ptr2;
   logic 			   t_alu_sched_full, t_alu_sched_full2;
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_alloc_entry, t_alu_select_entry, t_alu_alloc_entry2, t_alu_select_entry2;
   
   uop_t r_alu_sched_uops[N_INT_SCHED_ENTRIES-1:0], t_picked_uop;
   uop_t r_alu_sched_uops2[N_INT_SCHED_ENTRIES-1:0], t_picked_uop2;
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_entry_rdy, t_alu_entry_rdy2;
   logic [`LG_INT_SCHED_ENTRIES:0] t_alu_sched_select_ptr, t_alu_sched_select_ptr2;
   
	
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_srcA_rdy, r_alu_srcB_rdy, r_alu_srcA_rdy2, r_alu_srcB_rdy2;
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_srcA_match, t_alu_srcB_match, t_alu_srcA_match2, t_alu_srcB_match2;
   
   logic 			   t_alu_alloc_srcA_match, t_alu_alloc_srcB_match, t_alu_alloc_srcA_match2, t_alu_alloc_srcB_match2;
   
   
   wire [N_INT_SCHED_ENTRIES-1:0] w_alu_sched_oldest_ready, w_alu_sched_oldest_ready2;

   
   find_first_set#(`LG_INT_SCHED_ENTRIES) ffs_int_sched_alloc( .in(~r_alu_sched_valid),
							      .y(t_alu_sched_alloc_ptr));
   
   find_first_set#(`LG_INT_SCHED_ENTRIES) ffs_int_sched_select( .in(w_alu_sched_oldest_ready),
								.y(t_alu_sched_select_ptr));

   find_first_set#(`LG_INT_SCHED_ENTRIES) ffs_int_sched_alloc2( .in(~r_alu_sched_valid2),
								.y(t_alu_sched_alloc_ptr2));
   
   find_first_set#(`LG_INT_SCHED_ENTRIES) ffs_int_sched_select2( .in(w_alu_sched_oldest_ready2),
								 .y(t_alu_sched_select_ptr2));
     
   always_comb
     begin
	t_alu_alloc_entry = 'd0;
	t_alu_select_entry = 'd0;
	if(t_pop_uq)
	  begin
	     t_alu_alloc_entry[t_alu_sched_alloc_ptr[`LG_INT_SCHED_ENTRIES-1:0]] = 1'b1;
	  end
	if(t_alu_entry_rdy != 'd0)
	  begin
	     t_alu_select_entry[t_alu_sched_select_ptr[`LG_INT_SCHED_ENTRIES-1:0]] = 1'b1;
	  end
     end // always_comb

   always_comb
     begin
	t_alu_alloc_entry2 = 'd0;
	t_alu_select_entry2 = 'd0;
	if(t_pop_uq2)
	  begin
	     t_alu_alloc_entry2[t_alu_sched_alloc_ptr2[`LG_INT_SCHED_ENTRIES-1:0]] = 1'b1;
	  end
	if(t_alu_entry_rdy2 != 'd0)
	  begin
	     t_alu_select_entry2[t_alu_sched_select_ptr2[`LG_INT_SCHED_ENTRIES-1:0]] = 1'b1;
	  end
     end // always_comb


   always_comb
     begin
	t_picked_uop = r_alu_sched_uops[t_alu_sched_select_ptr[`LG_INT_SCHED_ENTRIES-1:0]];
     end

   always_comb
     begin
	t_picked_uop2 = r_alu_sched_uops2[t_alu_sched_select_ptr2[`LG_INT_SCHED_ENTRIES-1:0]];
     end
   
   
   always_ff@(posedge clk)
     begin
	int_uop <= t_picked_uop;
     end
   
   always_ff@(posedge clk)
     begin
	r_start_int <= reset ? 1'b0 : ((t_alu_entry_rdy != 'd0) & !ds_done);
     end // always_comb
   
   always_ff@(posedge clk)
     begin
	int_uop2 <= t_picked_uop2;
     end
   
   always_ff@(posedge clk)
     begin
	r_start_int2 <= reset ? 1'b0 : ((t_alu_entry_rdy2 != 'd0) & !ds_done);
     end
   
   always_comb
     begin
	//allocation forwarding
	t_alu_alloc_srcA_match = uq.srcA_valid && (
						   (mem_rsp_dst_valid & (mem_rsp_dst_ptr == uq.srcA)) ||
						   (r_start_int2 && t_wr_int_prf2 && (int_uop2.dst == uq.srcA)) ||
						   (r_start_int && t_wr_int_prf & (int_uop.dst == uq.srcA))
						   );
	t_alu_alloc_srcB_match = uq.srcB_valid && (
						   (mem_rsp_dst_valid & (mem_rsp_dst_ptr == uq.srcB)) ||
						   (r_start_int2 && t_wr_int_prf2 && (int_uop2.dst == uq.srcB)) ||						   
						   (r_start_int && t_wr_int_prf & (int_uop.dst == uq.srcB))
						   );
	
	t_alu_alloc_srcA_match2 = uq2.srcA_valid && (
						     (mem_rsp_dst_valid & (mem_rsp_dst_ptr == uq2.srcA)) ||
						     (r_start_int2 && t_wr_int_prf2 && (int_uop2.dst == uq2.srcA)) ||
						     (r_start_int && t_wr_int_prf & (int_uop.dst == uq2.srcA))
						     );
	t_alu_alloc_srcB_match2 = uq2.srcB_valid && (
						     (mem_rsp_dst_valid & (mem_rsp_dst_ptr == uq2.srcB)) ||
						     (r_start_int2 && t_wr_int_prf2 && (int_uop2.dst == uq2.srcB)) ||						   
						     (r_start_int && t_wr_int_prf & (int_uop.dst == uq2.srcB))
						     );		
     end // always_comb
  

   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_sched_mask_valid;
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_sched_matrix [N_INT_SCHED_ENTRIES-1:0];

   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_sched_mask_valid2;
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_sched_matrix2 [N_INT_SCHED_ENTRIES-1:0];
   
   
   always_comb
     begin
	t_alu_sched_mask_valid = r_alu_sched_valid & (~t_alu_select_entry);
	t_alu_sched_mask_valid2 = r_alu_sched_valid2 & (~t_alu_select_entry2);	
     end

   generate
      for(genvar i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
	begin
	   assign w_alu_sched_oldest_ready[i] = t_alu_entry_rdy[i] & (~(|(t_alu_entry_rdy & r_alu_sched_matrix[i])));
	   always_ff@(posedge clk)
	     begin
		if(reset || t_flash_clear)
		  begin
		     r_alu_sched_matrix[i] <= 'd0;
		  end
		else if(t_alu_alloc_entry[i])
		  begin
		     r_alu_sched_matrix[i] <= t_alu_sched_mask_valid;
		     
		  end
		else if(t_alu_entry_rdy != 'd0)
		  begin
		     r_alu_sched_matrix[i] <= r_alu_sched_matrix[i] & (~t_alu_select_entry);
		  end
	     end
	end // for (genvar i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
   endgenerate

   generate
      for(genvar i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
	begin
	   assign w_alu_sched_oldest_ready2[i] = t_alu_entry_rdy2[i] & (~(|(t_alu_entry_rdy2 & r_alu_sched_matrix2[i])));
	   always_ff@(posedge clk)
	     begin
		if(reset || t_flash_clear)
		  begin
		     r_alu_sched_matrix2[i] <= 'd0;
		  end
		else if(t_alu_alloc_entry2[i])
		  begin
		     r_alu_sched_matrix2[i] <= t_alu_sched_mask_valid2;
		     
		  end
		else if(t_alu_entry_rdy2 != 'd0)
		  begin
		     r_alu_sched_matrix2[i] <= r_alu_sched_matrix2[i] & (~t_alu_select_entry2);
		  end
	     end
	end // for (genvar i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
   endgenerate
   

   
   // always_ff@(negedge clk)
   //   begin
   // 	for(integer i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
   // 	  begin
   // 	     if(r_alu_sched_valid[i])
   // 	       begin
   // 		  $display("entry for pc %x is %b ready, src A %b, src B %b", 
   // 			   r_alu_sched_uops[i].pc, t_alu_entry_rdy[i],
   // 			   r_alu_srcA_rdy[i],
   // 			   r_alu_srcB_rdy[i]);
   // 	       end
   // 	  end
   //   end
   
   
   generate
      for(genvar i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
	begin
	   always_comb
	     begin
		t_alu_srcA_match[i] = r_alu_sched_uops[i].srcA_valid && (
									 (mem_rsp_dst_valid & (mem_rsp_dst_ptr == r_alu_sched_uops[i].srcA)) ||
									 (r_mul_complete && (r_mul_prf_ptr == r_alu_sched_uops[i].srcA)) ||
									 (r_div_complete && (r_div_prf_ptr == r_alu_sched_uops[i].srcA)) ||
									 (r_start_int2 && t_wr_int_prf2 & (int_uop2.dst == r_alu_sched_uops[i].srcA)) ||			 
									 (r_start_int && t_wr_int_prf & (int_uop.dst == r_alu_sched_uops[i].srcA))
									 );
		t_alu_srcB_match[i] = r_alu_sched_uops[i].srcB_valid && (
									 (mem_rsp_dst_valid & (mem_rsp_dst_ptr == r_alu_sched_uops[i].srcB)) ||
									 (r_mul_complete && (r_mul_prf_ptr == r_alu_sched_uops[i].srcB)) ||
									 (r_div_complete && (r_div_prf_ptr == r_alu_sched_uops[i].srcB)) ||
									 (r_start_int2 && t_wr_int_prf2 & (int_uop2.dst == r_alu_sched_uops[i].srcB)) ||
									 (r_start_int && t_wr_int_prf & (int_uop.dst == r_alu_sched_uops[i].srcB))
									 );
		
		t_alu_entry_rdy[i] = r_alu_sched_valid[i] &&
				     (uses_div(r_alu_sched_uops[i].op) ?  t_div_ready :  
				      (uses_mul(r_alu_sched_uops[i].op) ?  !r_wb_bitvec[`MUL_LAT+2] : !r_wb_bitvec[1]))
				     ? (
					(t_alu_srcA_match[i] |r_alu_srcA_rdy[i]) & 
					(t_alu_srcB_match[i] |r_alu_srcB_rdy[i]) 
					 ) : 1'b0;
	     end // always_comb
	   
	   always_ff@(posedge clk)
	     begin
		if(reset)
		  begin
		     r_alu_srcA_rdy[i] <= 1'b0;
		     r_alu_srcB_rdy[i] <= 1'b0;
		  end
		else
		  begin
		     if(t_alu_alloc_entry[i])
		       begin //allocating to this entry
			  r_alu_srcA_rdy[i] <= uq.srcA_valid ? (!r_prf_inflight[uq.srcA] | t_alu_alloc_srcA_match) : 1'b1;
			  r_alu_srcB_rdy[i] <= uq.srcB_valid ? (!r_prf_inflight[uq.srcB] | t_alu_alloc_srcB_match) : 1'b1;
		       end
		     else if(t_alu_select_entry[i])
		       begin
			  r_alu_srcA_rdy[i] <= 1'b0;
			  r_alu_srcB_rdy[i] <= 1'b0;
		       end
		     else if(r_alu_sched_valid[i])
		       begin
			  r_alu_srcA_rdy[i] <= r_alu_srcA_rdy[i] | t_alu_srcA_match[i];
			  r_alu_srcB_rdy[i] <= r_alu_srcB_rdy[i] | t_alu_srcB_match[i];
		       end // else: !if(t_pop_uq&&(t_alu_sched_alloc_ptr == i))
		     
		  end // else: !if(reset)
	     end // always_ff@ (posedge clk)
	end // for (genvar i = 0; i < LG_INT_SCHED_ENTRIES; i=i+1)
   endgenerate


   generate
      for(genvar i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
	begin
	   always_comb
	     begin
		t_alu_srcA_match2[i] = r_alu_sched_uops2[i].srcA_valid && (
									   (mem_rsp_dst_valid & (mem_rsp_dst_ptr == r_alu_sched_uops2[i].srcA)) ||
									   (r_mul_complete && (r_mul_prf_ptr == r_alu_sched_uops2[i].srcA)) ||
									   (r_div_complete && (r_div_prf_ptr == r_alu_sched_uops2[i].srcA)) ||
									   (r_start_int2 && t_wr_int_prf2 & (int_uop2.dst == r_alu_sched_uops2[i].srcA)) ||			 
									   (r_start_int && t_wr_int_prf & (int_uop.dst == r_alu_sched_uops2[i].srcA))
									   );
		
		t_alu_srcB_match2[i] = r_alu_sched_uops2[i].srcB_valid && (
									   (mem_rsp_dst_valid & (mem_rsp_dst_ptr == r_alu_sched_uops2[i].srcB)) ||
									   (r_mul_complete && (r_mul_prf_ptr == r_alu_sched_uops2[i].srcB)) ||
									   (r_div_complete && (r_div_prf_ptr == r_alu_sched_uops2[i].srcB)) ||
									   (r_start_int2 && t_wr_int_prf2 & (int_uop2.dst == r_alu_sched_uops2[i].srcB)) ||
									   (r_start_int && t_wr_int_prf & (int_uop.dst == r_alu_sched_uops2[i].srcB))
									   );
		
		t_alu_entry_rdy2[i] = r_alu_sched_valid2[i] 
				      ? (
					 (t_alu_srcA_match2[i] |r_alu_srcA_rdy2[i]) & 
					 (t_alu_srcB_match2[i] |r_alu_srcB_rdy2[i]) 
					 ) : 1'b0;
	     end // always_comb
	   
	   always_ff@(posedge clk)
	     begin
		if(reset)
		  begin
		     r_alu_srcA_rdy2[i] <= 1'b0;
		     r_alu_srcB_rdy2[i] <= 1'b0;
		  end
		else
		  begin
		     if(t_alu_alloc_entry2[i])
		       begin //allocating to this entry
			  r_alu_srcA_rdy2[i] <= uq2.srcA_valid ? (!r_prf_inflight[uq2.srcA] | t_alu_alloc_srcA_match2) : 1'b1;
			  r_alu_srcB_rdy2[i] <= uq2.srcB_valid ? (!r_prf_inflight[uq2.srcB] | t_alu_alloc_srcB_match2) : 1'b1;
		       end
		     else if(t_alu_select_entry2[i])
		       begin
			  r_alu_srcA_rdy2[i] <= 1'b0;
			  r_alu_srcB_rdy2[i] <= 1'b0;
		       end
		     else if(r_alu_sched_valid2[i])
		       begin
			  r_alu_srcA_rdy2[i] <= r_alu_srcA_rdy2[i] | t_alu_srcA_match2[i];
			  r_alu_srcB_rdy2[i] <= r_alu_srcB_rdy2[i] | t_alu_srcB_match2[i];
		       end // else: !if(t_pop_uq&&(t_alu_sched_alloc_ptr == i))
		     
		  end // else: !if(reset)
	     end // always_ff@ (posedge clk)
	end // for (genvar i = 0; i < LG_INT_SCHED_ENTRIES; i=i+1)
   endgenerate
   
   
   always_comb
     begin
	t_pop_uq = 1'b0;
	t_pop_uq2 = 1'b0;
	t_alu_sched_full = (&r_alu_sched_valid);
	t_alu_sched_full2 = (&r_alu_sched_valid2);
	
	t_pop_uq = !(t_flash_clear | t_uq_empty | t_alu_sched_full);
`ifdef SECOND_EXEC_PORT
	t_pop_uq2 = t_uq_next_empty ? 1'b0 : (t_pop_uq & uq2.is_cheap_int & (!t_alu_sched_full2));
`endif
     end // always_comb


   logic t_left_shift2, t_signed_shift2;
   wire [`M_WIDTH-1:0] w_shifter_out2;
   logic	       t_zero_shift_upper2;
   logic [5:0] t_shift_amt2;   
   
   shift_right #(.LG_W(`LG_M_WIDTH))
   s1(.is_left(t_left_shift2), 
      .is_signed(t_signed_shift2), 
      .data(t_zero_shift_upper2 ? {{32{(t_signed_shift2 ? t_srcA_2[31] : 1'b0)}}, t_srcA_2[31:0]} : t_srcA_2), 
      .distance(t_shift_amt2), 
      .y(w_shifter_out2));

   
   
  
   wire [`M_WIDTH-1:0] w_pc2_4;
   mwidth_add npc_2 (.A(int_uop2.pc), .B('d4), .Y(w_pc2_4));
 
   logic  t_sub2, t_addi_2;
   wire [31:0] w_s_sub32, w_c_sub32;
   wire [31:0] w_add32;

   logic       t_sub, t_addi;


   csa #(.N(32)) csa0 (.a(t_srcA[31:0]), 
		       .b( (t_addi ? int_uop.rvimm[31:0] : (t_sub ? ~t_srcB[31:0] : t_srcB[31:0]))), 
		       .cin(t_sub ? 32'd1 : 32'd0), 
		       .s(w_s_sub32), 
		       .cout(w_c_sub32) );
   
   wire [31:0] w_add32_srcA = {w_c_sub32[30:0], 1'b0};
   wire [31:0] w_add32_srcB = w_s_sub32;
   
   ppa32 add0 (.A(w_add32_srcA), .B(w_add32_srcB), .Y(w_add32));

   wire [63:0] w_as64_, w_as64_2_;
   addsub #(.W(64)) as0 
     (
      .A(t_srcA), 
      .B(t_addi ? int_uop.rvimm : t_srcB), 
      .is_sub(t_sub), 
      .Y(w_as64_)
      );
   
   addsub #(.W(64)) as1
     (
      .A(t_srcA_2), 
      .B(t_addi_2 ? int_uop2.rvimm : t_srcB_2), 
      .is_sub(t_sub2), 
      .Y(w_as64_2_)
      );

   wire [63:0] w_as64_sext = {{32{w_as64_[31]}}, w_as64_[31:0]};
   wire [63:0] w_as64 = mode64 ? w_as64_ : w_as64_sext;
   
   wire [63:0] w_as64_2_sext = {{32{w_as64_2_[31]}}, w_as64_2_[31:0]};
   wire [63:0] w_as64_2 = mode64 ? w_as64_2_ : w_as64_2_sext;
   
   
   wire [`M_WIDTH-1:0] w_indirect_target2;
   mwidth_add itgt (.A(t_srcA_2), .B(int_uop2.rvimm), .Y(w_indirect_target2));

   wire [63:0] w_fe_indirect_target2 = {int_uop2.jmp_imm,int_uop2.imm};
   
   wire	       w_mispredicted_indirect2 = w_indirect_target2 != w_fe_indirect_target2;
   
   
   always_comb
     begin
	t_sub2 = 1'b0;
	t_addi_2 = 1'b0;
	t_take_br2 = 1'b0;
	t_mispred_br2= 1'b0;
	t_pc_2 = int_uop2.pc;
	t_left_shift2 = 1'b0;
	t_signed_shift2 = 1'b0;
	t_shift_amt2 = 'd0;
	t_alu_valid2 = 1'b0;
	t_result2 = 'd0;
	t_wr_int_prf2 = 1'b0;
	t_zero_shift_upper2 = 1'b0;
`ifdef SECOND_EXEC_PORT	
	case(int_uop2.op)
	  BNE:
	    begin
	       t_take_br2 = t_srcA_2 != t_srcB_2;
	       t_mispred_br2 = int_uop2.br_pred != t_take_br2;
	       t_pc_2 = t_take_br2 ? int_uop2.rvimm : w_pc2_4;
	       t_alu_valid2 = 1'b1;	       
	    end
	  BEQ:
	    begin
	       t_take_br2 = t_srcA_2 == t_srcB_2;
	       t_mispred_br2 = int_uop2.br_pred != t_take_br2;
	       t_pc_2 = t_take_br2 ? int_uop2.rvimm : w_pc2_4;
	       t_alu_valid2 = 1'b1;	       
	    end
	  BLT:
	    begin
	       t_take_br2 = $signed(t_srcA_2) < $signed(t_srcB_2);
	       t_mispred_br2 = int_uop2.br_pred != t_take_br2;
	       t_pc_2 = t_take_br2 ? int_uop2.rvimm : w_pc2_4;
	       t_alu_valid2 = 1'b1;	       	       
	    end
	  BGE:
	    begin
	       t_take_br2 = $signed(t_srcA_2) >= $signed(t_srcB_2);
	       t_mispred_br2 = int_uop2.br_pred != t_take_br2;
	       t_pc_2 = t_take_br2 ? int_uop2.rvimm : w_pc2_4;
	       t_alu_valid2 = 1'b1;	       	       
	    end
	  BLTU:
	    begin
	       t_take_br2 = t_srcA_2 < t_srcB_2;
	       t_mispred_br2 = int_uop2.br_pred != t_take_br2;
	       t_pc_2 = t_take_br2 ? int_uop2.rvimm : w_pc2_4;
	       t_alu_valid2 = 1'b1;	       	       
	    end
	  BGEU:
	    begin
	       t_take_br2 = t_srcA_2 >= t_srcB_2;
	       t_mispred_br2 = int_uop2.br_pred != t_take_br2;
	       t_pc_2 = t_take_br2 ? int_uop2.rvimm : w_pc2_4;
	       t_alu_valid2 = 1'b1;	       	       
	    end
	  JAL:
	    begin
	       t_take_br2 = 1'b1;
	       t_mispred_br2 = int_uop2.br_pred != 1'b1;
	       t_pc_2 = int_uop2.rvimm;
	       t_result2 = w_pc2_4;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  JALR:
	    begin
	       t_take_br2 = 1'b1;
	       t_mispred_br2 = w_mispredicted_indirect2;	       
	       t_pc_2 = w_indirect_target2;
	       t_alu_valid2 = 1'b1;
	       t_result2 = w_pc2_4;
	       t_wr_int_prf2 = 1'b1;
	    end
	  JR:
	    begin
	       t_take_br2 = 1'b1;
	       t_mispred_br2 = w_mispredicted_indirect2;	       
	       t_pc_2 = w_indirect_target2;
	       t_alu_valid2 = 1'b1;
	    end
	  RET:
	    begin
	       t_take_br2 = 1'b1;
	       t_mispred_br2 = w_mispredicted_indirect2;	       
	       t_pc_2 = w_indirect_target2;
	       t_alu_valid2 = 1'b1;
	    end	  
	  ADDI:
	    begin
	       t_addi_2 = 1'b1;
	       t_result2 = w_as64_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  ADDIW:
	    begin
	       t_addi_2 = 1'b1;
	       t_result2 = w_as64_2_sext;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end	  
	  ADDU:
	    begin
	       t_result2 = w_as64_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  ADDW:
	    begin
	       t_result2 = w_as64_2_sext;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  SLT:
	    begin
	       t_result2 = {w_zf, $signed(t_srcA_2) < $signed(t_srcB_2)};
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  SLTU:
	    begin
	       t_result2 = {w_zf, t_srcA_2 < t_srcB_2};
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  SLTI:
	    begin
	       t_result2 = {w_zf, $signed(t_srcA_2) < $signed(int_uop2.rvimm)};
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  SLTIU:
	    begin
	       t_result2 = {w_zf, t_srcA_2 < int_uop2.rvimm};
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end	       
	  SUBU:
	    begin
	       t_sub2 = 1'b1;
	       t_result2 = w_as64_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  SUBW:
	    begin
	       t_sub2 = 1'b1;
	       t_result2 = w_as64_2_sext;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  ANDI:
	    begin
	       t_result2 = int_uop2.rvimm & t_srcA_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  ORI:
	    begin
	       t_result2 = int_uop2.rvimm | t_srcA_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  XORI:
	    begin
	       t_result2 = int_uop2.rvimm ^ t_srcA_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  AND:
	    begin
	       t_result2 = t_srcA_2 & t_srcB_2;	       
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  OR:
	    begin
	       t_result2 = t_srcA_2 | t_srcB_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  XOR:
	    begin
	       t_result2 = t_srcA_2 ^ t_srcB_2;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  SRL:
	    begin
	       t_shift_amt2 = {(mode64 ? t_srcB_2[5] : 1'b0), t_srcB_2[4:0]};	
	       t_result2 = w_shifter_out2;
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	    end
	  SRLI:
	    begin
	       t_shift_amt2 = {(mode64 ? int_uop2.rvimm[5] : 1'b0), int_uop2.rvimm[4:0]};
	       t_result2 = w_shifter_out2;
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;	       
	    end
	  SRA:
	    begin
	       t_signed_shift2 = 1'b1;
	       t_shift_amt2 = {(mode64 ? t_srcB_2[5] : 1'b0), t_srcB_2[4:0]};	       
	       t_result2 = w_shifter_out2;
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	    end
	  SRAW:
	    begin
	       t_signed_shift2 = 1'b1;
	       t_shift_amt2 = {1'b0,t_srcB_2[4:0]};	       
	       t_result2 = {{32{w_shifter_out2[31]}}, w_shifter_out2[31:0]};
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	    end
	  SRLW:
	    begin
	       t_zero_shift_upper2 = 1'b1;	       
	       t_shift_amt2 = {1'b0,t_srcB_2[4:0]};	       
	       t_result2 = {{32{w_shifter_out2[31]}}, w_shifter_out2[31:0]};
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	    end	  
	  SRAIW:
	    begin
	       t_signed_shift2 = 1'b1;
	       t_shift_amt2 = {1'b0, int_uop2.rvimm[4:0]};
	       t_result2 = {{32{w_shifter_out2[31]}}, w_shifter_out2[31:0]};
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;	
	       t_zero_shift_upper2 = 1'b1;	
	    end
	  SRLIW:
	    begin
	       t_shift_amt2 = {1'b0, int_uop2.rvimm[4:0]};
	       t_result2 = {{32{w_shifter_out2[31]}}, w_shifter_out2[31:0]};
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	       t_zero_shift_upper2 = 1'b1;
	    end
	  
	  SRAI:
	    begin
	       t_signed_shift2 = 1'b1;
	       t_shift_amt2 = {(mode64 ? int_uop2.rvimm[5] : 1'b0), int_uop2.rvimm[4:0]};
	       t_result2 = w_shifter_out2;
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;		
	    end	  
	  SLL:
	    begin
	       t_left_shift2 = 1'b1;
	       t_shift_amt2 = {(mode64 ? t_srcB_2[5] : 1'b0), t_srcB_2[4:0]};
	       t_result2 = w_shifter_out2;
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	    end
	  SLLW:
	    begin
	       t_left_shift2 = 1'b1;
	       t_shift_amt2 = {1'b0, t_srcB_2[4:0]};
	       t_result2 = {{32{w_shifter_out2[31]}}, w_shifter_out2[31:0]};
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	    end	  
	  SLLI:
	    begin
	       t_left_shift2 = 1'b1;	       
	       t_shift_amt2 = {(mode64 ? int_uop2.rvimm[5] : 1'b0), int_uop2.rvimm[4:0]};
	       t_result2 = w_shifter_out2;
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;		
	    end
	  SLLIW:
	    begin
	       t_left_shift2 = 1'b1;
	       t_shift_amt2 = int_uop2.rvimm[5:0];	       
	       t_result2 = {{32{w_shifter_out2[31]}}, w_shifter_out2[31:0]};
	       t_wr_int_prf2 = 1'b1;
	       t_alu_valid2 = 1'b1;
	    end	  
	  AUIPC:
	    begin
	       t_result2 = int_uop2.rvimm;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  LUI:
	    begin
	       t_result2 = int_uop2.rvimm;
	       t_alu_valid2 = 1'b1;
	       t_wr_int_prf2 = 1'b1;
	    end
	  default:
	    begin
	    end
	endcase // case (int_uop2.op)
`endif
     end

   
   
   always_ff@(posedge clk)
     begin
	if(reset || t_flash_clear)
	  begin
	     r_alu_sched_valid <= 'd0;
	  end
	else
	  begin
	     if(t_pop_uq)
	       begin
		  r_alu_sched_valid[t_alu_sched_alloc_ptr[`LG_INT_SCHED_ENTRIES-1:0]] <= 1'b1;
		  r_alu_sched_uops[t_alu_sched_alloc_ptr[`LG_INT_SCHED_ENTRIES-1:0]] <= uq;
	       end
	     if(t_alu_entry_rdy != 'd0)
	       begin
		  r_alu_sched_valid[t_alu_sched_select_ptr[`LG_INT_SCHED_ENTRIES-1:0]] <= 1'b0;
	       end
	  end // else: !if(reset)
     end // always_ff@ (posedge clk)


   always_ff@(posedge clk)
     begin
	if(reset || t_flash_clear)
	  begin
	     r_alu_sched_valid2 <= 'd0;
	  end
	else
	  begin
	     if(t_pop_uq2)
	       begin
		  r_alu_sched_valid2[t_alu_sched_alloc_ptr2[`LG_INT_SCHED_ENTRIES-1:0]] <= 1'b1;
		  r_alu_sched_uops2[t_alu_sched_alloc_ptr2[`LG_INT_SCHED_ENTRIES-1:0]] <= uq2;
	       end
	     if(t_alu_entry_rdy2 != 'd0)
	       begin
		  r_alu_sched_valid2[t_alu_sched_select_ptr2[`LG_INT_SCHED_ENTRIES-1:0]] <= 1'b0;
	       end
	  end // else: !if(reset)
     end // always_ff@ (posedge clk)
   
   
   
   
   shift_right #(.LG_W(`LG_M_WIDTH)) 
   s0(.is_left(t_left_shift), 
      .is_signed(t_signed_shift), 
      .data(t_zero_shift_upper ? {{32{(t_signed_shift ? t_srcA[31] : 1'b0)}}, t_srcA[31:0]} : t_srcA),
      .distance(t_shift_amt), 
      .y(w_shifter_out));

   

   
   always_ff@(posedge clk)
     begin
	r_mul_prf_ptr <= w_mul_prf_ptr;
	r_div_prf_ptr <= w_div_prf_ptr;
	r_mul_complete <= reset ? 1'b0 : t_mul_complete;
	r_div_complete <= reset ? 1'b0 : t_div_complete;
     end

   
   mul m
     (
      .clk(clk), 
      .reset(reset), 
      .is_signed(t_signed_mul),
      .is_high(int_uop.op == MULHU || int_uop.op == MULH),
      .go(t_start_mul&r_start_int),
      .is_mulw(t_is_mulw),
      .src_A(t_srcA),
      .src_B(t_srcB),
      .rob_ptr_in(int_uop.rob_ptr),
      .prf_ptr_in(int_uop.dst),
      .y(t_mul_result),
      .complete(t_mul_complete),
      .rob_ptr_out(t_rob_ptr_out),
      .prf_ptr_val_out(),
      .prf_ptr_out(w_mul_prf_ptr)
      );
   
   always_ff@(negedge clk)
     begin
	//if(t_wr_csr_en && int_uop.imm[5:0] == SSTATUS)
	//begin
	//$display(">> %x writes %x to sstatus old %x at cycle %d",
	//int_uop.pc, t_wr_csr, r_mstatus, r_cycle);
	//end
	//if(t_wr_csr_en && int_uop.imm[5:0] == MSTATUS)
	//begin
	// $display(">> %x writes %x to mstatus old %x at cycle %d",
	//int_uop.pc, t_wr_csr, r_mstatus, r_cycle);
	//end	
	//if((int_uop.op == SRLIW) & r_start_int)
	//begin
	//$display("portA pc %x src A = %x, imm = %x, result %x", 
	//int_uop.pc, t_srcA, t_shift_amt, t_result);
	// end

	//if((int_uop2.op == SRLIW) & r_start_int2)
	//begin
	//$display("portA pc %x src A = %x, imm = %x, result %x", 
	//int_uop2.pc, t_srcA_2, t_shift_amt2, t_result2);
	 // end


	// if(t_start_mul&r_start_int)
	//   begin
	//      $display("multiplier dest ptr entry %d", int_uop.dst);
	//   end
	// if(t_mul_complete)
	//   begin
	//      $display("r_start_int %b, t_wr_int_prf %b,ptr %d, result %x",
	// 	      r_start_int, t_wr_int_prf,w_mul_prf_ptr, t_mul_result);
	//      if(r_start_int & t_wr_int_prf)
	//        $stop();
	//   end

	// if(t_div_complete)
	//   begin
	//      $display("divider writes back to rob %d, prf %d, value %x at cycle %d",
	// 	      t_div_rob_ptr, w_div_prf_ptr, t_div_result[31:0], r_cycle);
	//   end
	
	
	if(t_mul_complete & t_div_complete)
	  $stop();
	
	if(t_mul_complete & r_start_int & t_wr_int_prf)
	  $stop();

	if(t_div_complete & r_start_int & t_wr_int_prf)
	  begin
	     $display("divide completes but pc %x started at cycle %d", 
		      int_uop.pc, r_cycle);
	     $stop();
	  end
     end
   
	       
   //t_zero_shift_upper
   wire [63:0] w_divA = t_zero_shift_upper ? 
	       {{32{(t_signed_div ? t_srcA[31] : 1'b0)}}, t_srcA[31:0]} : 
	       t_srcA;

   wire [63:0] w_divB = t_zero_shift_upper ? 
	       {{32{(t_signed_div ? t_srcB[31] : 1'b0)}}, t_srcB[31:0]} : 
	       t_srcB;
	       
   divider #(.LG_W(6))
   d64 (
	.clk(clk), 
	.reset(reset),
	.inA(w_divA),
	.inB(w_divB),
	.rob_ptr_in(int_uop.rob_ptr),
	.prf_ptr_in(int_uop.dst),
	.is_signed_div(t_signed_div),
	.is_w(t_zero_shift_upper),
        .is_rem(t_is_rem),
	.start_div(t_start_div64),
	.y(t_div_result),
	.rob_ptr_out(t_div_rob_ptr),
	.prf_ptr_out(w_div_prf_ptr),
	.complete(t_div_complete),
	.ready(t_div_ready)
	);

   assign divide_ready = t_div_ready;




   
   always_comb
     begin
	n_mq_head_ptr = r_mq_head_ptr;
	n_mq_tail_ptr = r_mq_tail_ptr;
	n_mq_next_tail_ptr = r_mq_next_tail_ptr;
	
	if(r_mem_ready)
	  begin
	     n_mq_tail_ptr = r_mq_tail_ptr + 'd1;
	     n_mq_next_tail_ptr = r_mq_next_tail_ptr + 'd1;
	  end
	if(mem_req_ack)
	  begin
	     n_mq_head_ptr = r_mq_head_ptr + 'd1;
	  end
	
	t_mem_head = r_mem_q[r_mq_head_ptr[`LG_MQ_ENTRIES-1:0]];
	
	mem_q_empty = (r_mq_head_ptr == r_mq_tail_ptr);
	
	mem_q_full = (r_mq_head_ptr != r_mq_tail_ptr) &&
		     (r_mq_head_ptr[`LG_MQ_ENTRIES-1:0] == r_mq_tail_ptr[`LG_MQ_ENTRIES-1:0]);

	mem_q_next_full = (r_mq_head_ptr != r_mq_next_tail_ptr) &&
			  (r_mq_head_ptr[`LG_MQ_ENTRIES-1:0] == r_mq_next_tail_ptr[`LG_MQ_ENTRIES-1:0]);
	
     end // always_comb
   
   always_ff@(posedge clk)
     begin
	if(r_mem_ready)
	  begin
	     r_mem_q[r_mq_tail_ptr[`LG_MQ_ENTRIES-1:0]] <= t_mem_tail;
	  end
     end

   
   always_comb
     begin
	n_mdq_head_ptr = r_mdq_head_ptr;
	n_mdq_tail_ptr = r_mdq_tail_ptr;
	n_mdq_next_tail_ptr = r_mdq_next_tail_ptr;
	
	if(r_dq_ready)
	  begin
	     n_mdq_tail_ptr = r_mdq_tail_ptr + 'd1;
	     n_mdq_next_tail_ptr = r_mdq_next_tail_ptr + 'd1;
	  end
	
	if(core_store_data_ack)
	  begin
	     n_mdq_head_ptr = r_mdq_head_ptr + 'd1;
	  end

	core_store_data = r_mdq[r_mdq_head_ptr[`LG_MQ_ENTRIES-1:0]];
			       
	mem_mdq_empty = (r_mdq_head_ptr == r_mdq_tail_ptr);
	
	mem_mdq_full = (r_mdq_head_ptr != r_mdq_tail_ptr) &&
		     (r_mdq_head_ptr[`LG_MQ_ENTRIES-1:0] == r_mdq_tail_ptr[`LG_MQ_ENTRIES-1:0]);

	mem_mdq_next_full = (r_mdq_head_ptr != r_mdq_next_tail_ptr) &&
			    (r_mdq_head_ptr[`LG_MQ_ENTRIES-1:0] == r_mdq_next_tail_ptr[`LG_MQ_ENTRIES-1:0]);
     end // always_comb
   

   
   assign mem_req = t_mem_head;
   assign mem_req_valid = !mem_q_empty;
   assign uq_wait = r_uq_wait;
   assign mq_wait = r_mq_wait;
   assign core_store_data_valid = !mem_mdq_empty;
   assign paging_active = r_paging_active;
   
   
   always_ff@(posedge clk)
     begin
	r_mq_head_ptr <= reset ? 'd0 : n_mq_head_ptr;
	r_mq_tail_ptr <= reset ? 'd0 : n_mq_tail_ptr;
	r_mq_next_tail_ptr <= reset ? 'd1 : n_mq_next_tail_ptr;

	r_mdq_head_ptr <= (reset || mem_dq_clr) ? 'd0 : n_mdq_head_ptr;
	r_mdq_tail_ptr <= (reset || mem_dq_clr) ? 'd0 : n_mdq_tail_ptr;
	r_mdq_next_tail_ptr <= (reset || mem_dq_clr) ? 'd1 : n_mdq_next_tail_ptr;	
     end

   
   always_ff@(posedge clk)
     begin
	if(reset || ds_done)
	  begin
	     r_prf_inflight <= 'd0;
	  end
	else
	  begin
	     if(uq_push && uq_uop.dst_valid)
	       begin
		  r_prf_inflight[uq_uop.dst] <= 1'b1;
	       end
	     if(uq_push_two && uq_uop_two.dst_valid)
	       begin
		  r_prf_inflight[uq_uop_two.dst] <= 1'b1;
	       end
	     if(mem_rsp_dst_valid)
	       begin
		  r_prf_inflight[mem_rsp_dst_ptr] <= 1'b0;
	       end
	     
	     if(r_start_int && t_wr_int_prf)
	       begin
		  r_prf_inflight[int_uop.dst] <= 1'b0;
	       end
	     else if(t_mul_complete)
	       begin
		  r_prf_inflight[w_mul_prf_ptr] <= 1'b0;
	       end
	     else if(t_div_complete)
	       begin
		  r_prf_inflight[w_div_prf_ptr] <= 1'b0;
	       end
	     if(r_start_int2 && t_wr_int_prf2)
	       begin
		  r_prf_inflight[int_uop2.dst] <= 1'b0;
	       end	     
	  end
     end // always_ff@ (posedge clk)
   
   

   
`ifdef VERILATOR
   always_ff@(negedge clk)
     begin
	report_exec(t_uq_empty ? 32'd0 : 32'd1,
		    t_pop_uq ? 32'd1 : 32'd0,
		    !t_uq_next_empty & t_pop_uq & uq2.is_cheap_int ? 32'd1 : 32'd0,
		    t_pop_uq2 ? 32'd1 : 32'd0,
		    t_mem_uq_empty ? 32'd0 : 32'd1,
		    t_pop_mem_uq ? 32'd1 : 32'd0,
		    32'd1,
		    32'd0,
		    t_uq_full ? 32'd1 : 32'd0,
		    t_mem_uq_full ? 32'd1 : 32'd0,
		    32'd0,
		    32'd0,
		    {{(32-N_INT_SCHED_ENTRIES){1'b0}}, t_alu_entry_rdy},
		    {{(32-N_INT_SCHED_ENTRIES){1'b0}}, t_alu_entry_rdy2}
		    );
     end
`endif //  `ifdef VERILATOR

   
   wire [`M_WIDTH-1:0] w_pc4;
   wire [`M_WIDTH-1:0] w_indirect_target;
   mwidth_add add2 (.A(t_srcA), .B(int_uop.rvimm), .Y(w_indirect_target));

   wire [63:0]	       w_fe_indirect_target = {int_uop.jmp_imm,int_uop.imm};
   wire		       w_mispredicted_indirect = w_indirect_target != w_fe_indirect_target;
      
   mwidth_add add3 (.A(int_uop.pc), .B('d4), .Y(w_pc4));

   always_comb
     begin
	t_sub = 1'b0;
	t_addi = 1'b0;
	t_pc = int_uop.pc;
	t_result ='d0;
	t_has_cause = 1'b0;
	t_cause = MISALIGNED_FETCH;
	t_clear_tlb = 1'b0;
	t_wr_csr_en = 1'b0;
	t_rd_csr_en = 1'b0;
	t_wr_priv = 1'b0;
	t_priv = 2'd0;
	t_wr_csr = 64'd0;
	t_wr_int_prf = 1'b0;
	t_take_br = 1'b0;
	t_mispred_br = 1'b0;
	t_alu_valid = 1'b0;
	t_got_break = 1'b0;
	t_signed_shift = 1'b0;
	t_left_shift = 1'b0;
	t_shift_amt = 'd0;
	t_start_mul = 1'b0;
	t_signed_mul = 1'b0;
	t_is_mulw = 1'b0;
	
	t_signed_div = 1'b0;
	t_is_rem = 1'b0;
	t_start_div32 = 1'b0;
	t_start_div64 = 1'b0;	
	t_zero_shift_upper = 1'b0;
	
	case(int_uop.op)
	  //riscv
	  DIV:
	    begin
	       t_signed_div = 1'b1;
	       t_start_div64 = r_start_int&!ds_done;	       
	    end
	  DIVW:
	    begin
	       t_signed_div = 1'b1;
	       t_zero_shift_upper = 1'b1;	       
	       t_start_div64 = r_start_int&!ds_done;	       
	    end
	  DIVU:
	    begin
	       t_start_div64 = r_start_int&!ds_done;
	    end
	  DIVUW:
	    begin
	       t_zero_shift_upper = 1'b1;
	       t_start_div64 = r_start_int&!ds_done;
	    end
	  REM:
	    begin
	       t_signed_div = 1'b1;
	       t_is_rem = 1'b1;
	       t_start_div64 = r_start_int&!ds_done;	       
	    end
	  REMU:
	    begin
	       t_is_rem = 1'b1;
	       t_start_div64 = r_start_int&!ds_done;
	    end
	  REMW:
	    begin
	       t_zero_shift_upper = 1'b1;	       
	       t_signed_div = 1'b1;
	       t_is_rem = 1'b1;
	       t_start_div64 = r_start_int&!ds_done;	       
	    end
	  REMUW:
	    begin
	       t_zero_shift_upper = 1'b1;
	       t_is_rem = 1'b1;
	       t_start_div64 = r_start_int&!ds_done;
	    end
	  MUL:
	    begin
	       t_signed_mul = 1'b1;	       	       
	       t_start_mul = r_start_int&!ds_done;
	    end
	  MULW:
	    begin
	       t_is_mulw = 1'b1;	       
	       t_signed_mul = 1'b1;	       
	       t_start_mul = r_start_int&!ds_done;	       
	    end
	  MULH:
	    begin
	       t_signed_mul = 1'b1;
	       t_start_mul = r_start_int&!ds_done;
	    end
	  MULHU:
	    begin
	       t_start_mul = r_start_int&!ds_done;
	    end
	  ADDI:
	    begin
	       t_addi = 1'b1;
	       t_result = w_as64;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  ADDIW:
	    begin
	       t_addi = 1'b1;
	       t_result = w_as64_sext;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end	  
	  ADDU:
	    begin
	       t_result = w_as64;	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  ADDW:
	    begin
	       t_result = w_as64_sext;	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SUBW:
	    begin
	       t_sub = 1'b1;
	       t_result = w_as64_sext;	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end	  	  
	  RDCYCLE:
	    begin
	       t_result = r_cycle[`M_WIDTH-1:0];
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	       t_pc = w_pc4;	       
	    end
	  RDINSTRET:
	    begin
	       t_result = r_retired_insns[`M_WIDTH-1:0];
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	       t_pc = w_pc4;
	    end
	  RDBRANCH:
	    begin
	       t_result = r_branches[`M_WIDTH-1:0];      
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	       t_pc = w_pc4;	       
	    end
	  RDFAULTEDBRANCH:
	    begin
	       t_result = r_branch_faults[`M_WIDTH-1:0];
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	       t_pc = w_pc4;
	    end
	  AND:
	    begin
	       t_result = t_srcA & t_srcB;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;	       
	    end
	  SUBU:
	    begin
	       t_sub = 1'b1;
	       t_result = w_as64;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  ANDI:
	    begin
	       t_result = t_srcA & int_uop.rvimm;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  AUIPC:
	    begin
	       t_result = int_uop.rvimm;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  BEQ:
	    begin
	       t_take_br = (t_srcA == t_srcB);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? int_uop.rvimm : w_pc4;	       
	       t_alu_valid = 1'b1;
	    end
	  BGE:
	    begin
	       t_take_br = $signed(t_srcA) >= $signed(t_srcB);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? int_uop.rvimm : w_pc4;	       
	       t_alu_valid = 1'b1;
	    end
	  BGEU:
	    begin
	       t_take_br = t_srcA >= t_srcB;
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? int_uop.rvimm : w_pc4;
	       t_alu_valid = 1'b1;
	    end
	  BLT:
	    begin
	       t_take_br = $signed(t_srcA) < $signed(t_srcB);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? int_uop.rvimm : w_pc4;
	       t_alu_valid = 1'b1;
	    end
	  BLTU:
	    begin
	       t_take_br = t_srcA < t_srcB;
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? int_uop.rvimm : w_pc4;
	       t_alu_valid = 1'b1;
	    end
	  
	  BNE:
	    begin
	       t_take_br = (t_srcA != t_srcB);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? int_uop.rvimm : w_pc4;
	       t_alu_valid = 1'b1;
	    end
	  JAL:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = int_uop.rvimm;
	       t_result = w_pc4;
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	    end
	  JALR:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = w_mispredicted_indirect;	       
	       t_pc = w_indirect_target;
	       t_alu_valid = 1'b1;
	       t_result = w_pc4;
	       t_wr_int_prf = 1'b1;
	    end
	  JR:
	     begin
		t_take_br = 1'b1;
		t_mispred_br = w_mispredicted_indirect;
		t_pc = w_indirect_target;
		t_alu_valid = 1'b1;
	     end
	  RET:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = w_mispredicted_indirect;
	       t_pc = w_indirect_target;
	       t_alu_valid = 1'b1;
	    end
	  LUI:
	    begin
	       t_result = int_uop.rvimm;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;	       
	    end
	   OR:
	     begin
		t_result = t_srcA | t_srcB;	       
		t_wr_int_prf = 1'b1;
		t_alu_valid = 1'b1;
	     end
	   ORI:
	     begin
		t_result = t_srcA | int_uop.rvimm;	       
		t_wr_int_prf = 1'b1;
		t_alu_valid = 1'b1;
	     end
	  SLL:
	    begin
	       t_left_shift = 1'b1;
	       t_shift_amt = {(mode64 ? t_srcB[5] : 1'b0), t_srcB[4:0]};
	       t_result = w_shifter_out;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLLW:
	    begin
	       t_left_shift = 1'b1;
	       t_shift_amt = {1'b0, t_srcB[4:0]};
	       t_result = {{32{w_shifter_out[31]}}, w_shifter_out[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end	  
	  SLLI:
	    begin
	       t_left_shift = 1'b1;
	       t_shift_amt = {(mode64 ? int_uop.rvimm[5] : 1'b0), int_uop.rvimm[4:0]};	       
	       t_result = w_shifter_out;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLLIW:
	    begin
	       t_left_shift = 1'b1;
	       t_shift_amt = {1'b0, int_uop.rvimm[4:0]};	       
	       t_result = {{32{w_shifter_out[31]}}, w_shifter_out[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLT:
	    begin
	       t_result = {w_zf, $signed(t_srcA) < $signed(t_srcB)};	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLTU:
	    begin
	       t_result = {w_zf, t_srcA < t_srcB};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLTI:
	    begin
	       t_result = {w_zf, $signed(t_srcA) < $signed(int_uop.rvimm)};	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLTIU:
	    begin
	       t_result = {w_zf, t_srcA < int_uop.rvimm};	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRAIW:
	    begin
	       t_zero_shift_upper = 1'b1;	       
	       t_signed_shift = 1'b1;
	       t_shift_amt = {1'b0, int_uop.rvimm[4:0]};	       
	       t_result = {{32{w_shifter_out[31]}}, w_shifter_out[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end	  	  
	  SRLIW:
	    begin
	       t_zero_shift_upper = 1'b1;
	       t_shift_amt = {1'b0, int_uop.rvimm[4:0]};	       
	       t_result = {{32{w_shifter_out[31]}}, w_shifter_out[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end	  	  
	  SRAI:
	    begin
	       t_signed_shift = 1'b1;
	       t_shift_amt = {(mode64 ? int_uop.rvimm[5] : 1'b0), int_uop.rvimm[4:0]};	       	       
	       t_result = w_shifter_out;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRAW:
	    begin
	       t_signed_shift = 1'b1;
	       t_shift_amt = {1'b0,t_srcB[4:0]};
	       t_result = {{32{w_shifter_out[31]}}, w_shifter_out[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRLW:
	    begin
	       t_zero_shift_upper = 1'b1;	       
	       t_shift_amt = {1'b0,t_srcB[4:0]};
	       t_result = {{32{w_shifter_out[31]}}, w_shifter_out[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end	  	  
	  SRA:
	    begin
	       t_signed_shift = 1'b1;
	       t_shift_amt = {(mode64 ? t_srcB[5] : 1'b0), t_srcB[4:0]};
	       t_result = w_shifter_out;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRL:
	    begin
	       t_shift_amt = {(mode64 ? t_srcB[5] : 1'b0), t_srcB[4:0]};	       
	       t_result = w_shifter_out;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRLI:
	     begin
		t_shift_amt = {(mode64 ? int_uop.rvimm[5] : 1'b0), int_uop.rvimm[4:0]};		
		t_result = w_shifter_out;
		t_wr_int_prf = 1'b1;
		t_alu_valid = 1'b1;
	     end
	  XORI:
	    begin
	       t_result = t_srcA ^ int_uop.rvimm;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  XOR:
	    begin
	       t_result = t_srcA ^ t_srcB;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRET:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_wr_csr_en = r_start_int;
	       t_wr_csr = t_rd_csr;
	       t_wr_priv = r_start_int;
	       t_priv = {1'b0, r_mstatus[8]};	       
	       t_pc = r_sepc;
	       t_alu_valid = 1'b1;
	    end
	  MRET:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_wr_csr_en = r_start_int;
	       t_wr_csr = t_rd_csr;
	       t_wr_priv = r_start_int;
	       t_priv = w_mpp;
	       t_pc = r_mepc;
	       t_alu_valid = 1'b1;
	    end
	  ECALL:
	    begin
	       t_has_cause = 1'b1;
	       t_cause = r_priv=='d0 ? USER_ECALL :
			 r_priv=='d1 ? SUPERVISOR_ECALL :
			 r_priv=='d2 ? HYPERVISOR_ECALL :
			 MACHINE_ECALL;
	       t_alu_valid = 1'b1;
	    end
	  EBREAK:
	    begin
	       t_has_cause = 1'b1;
	       t_cause = BREAKPOINT;
	       t_alu_valid = 1'b1;
	    end
	  SFENCEVMA:
	    begin
	       t_pc = w_pc4;
	       t_alu_valid = 1'b1;
	       t_clear_tlb = r_start_int;
	    end
	  CSRRW:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_result = t_rd_csr;
	       t_wr_csr_en = r_start_int;
	       t_wr_csr = t_srcA;
	       t_wr_int_prf = int_uop.dst_valid;
	       t_alu_valid = 1'b1;
	       t_pc = w_pc4;	       
	    end
	  CSRRS:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_result = t_rd_csr;
	       t_wr_csr_en = r_start_int & (int_uop.imm[10:6] != 'd0);
	       t_wr_csr = t_rd_csr | t_srcA;
	       t_wr_int_prf = int_uop.dst_valid;
	       t_alu_valid = 1'b1;
	       t_pc = w_pc4;	       
	    end
	  CSRRC:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_result = t_rd_csr;
	       t_wr_csr_en = r_start_int & (int_uop.imm[10:6] != 'd0);
	       t_wr_csr = t_rd_csr & (~t_srcA);
	       t_wr_int_prf = int_uop.dst_valid;
	       t_alu_valid = 1'b1;
	       t_pc = w_pc4;	       
	    end
	  CSRRWI:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_wr_csr_en = r_start_int;
	       t_wr_csr = {59'd0, int_uop.imm[10:6]};
	       t_result = t_rd_csr;
	       t_wr_int_prf = int_uop.dst_valid;
	       t_alu_valid = 1'b1;
	       t_pc = w_pc4;
	    end
	  CSRRSI:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_wr_csr_en = (int_uop.imm[10:6] != 'd0)&r_start_int;
	       t_wr_csr = t_rd_csr | {59'd0, int_uop.imm[10:6]};
	       t_result = t_rd_csr;
	       t_wr_int_prf = int_uop.dst_valid;
	       t_alu_valid = 1'b1;
	       t_pc = w_pc4;
	    end	  
	  CSRRCI:
	    begin
	       t_rd_csr_en = 1'b1;
	       t_wr_csr_en = (int_uop.imm[10:6] != 'd0)&r_start_int;
	       t_wr_csr = t_rd_csr & (~{59'd0, int_uop.imm[10:6]});
	       t_result = t_rd_csr;
	       t_wr_int_prf = int_uop.dst_valid;
	       t_alu_valid = 1'b1;
	       t_pc = w_pc4;
	    end
	  BREAK:
	    begin
	       t_cause = BREAKPOINT;
	    end
	  II:
	    begin
	       t_has_cause = 1'b1;
	       t_cause = ILLEGAL_INSTRUCTION;
	       t_alu_valid = 1'b1;
	    end
	  FETCH_PF:
	    begin
	       t_has_cause = 1'b1;
	       t_cause = FETCH_PAGE_FAULT;
	       t_alu_valid = 1'b1;
	    end
	  default:
	    begin
	       t_has_cause = 1'b1;
	       t_cause = ILLEGAL_INSTRUCTION;
	       t_alu_valid = 1'b1;
	    end
	endcase // case (int_uop.op)
     end // always_comb



   logic	       t_delegate;

   assign page_table_root = {8'd0, r_satp[43:0], 12'd0};
   
   
   always_comb
     begin
	t_delegate = 1'b0;
	if(r_priv[1]  ==  1'b0)
	  begin
	     t_delegate = irq ? r_mideleg[31:0][cause] : r_medeleg[15:0][cause[3:0]];
	  end
	exc_pc = t_delegate ? r_stvec : r_mtvec;
     end
   
   always_ff@(posedge clk)
     begin
	r_clear_tlb <= reset ? 1'b0 : t_clear_tlb;
     end

   logic r_priv_update0, r_priv_update;
   assign priv_update = r_priv_update;
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_priv_update0 <= 1'b0;
	     r_priv_update <= 1'b0;
	  end
	else
	  begin
	     r_priv_update0 <= update_csr_exc|t_wr_priv;
	     r_priv_update <= r_priv_update0;
	  end
     end
   
   always_comb
     begin
	n_priv = r_priv;
	if(update_csr_exc)
	  begin
	     n_priv = t_delegate ? 'd1 : 'd3;
	  end
	else if(t_wr_priv)
	  begin
	     n_priv = t_priv;
	  end	
     end // always_comb

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin /* begin in machine priv mode */
	     r_priv <= 2'd3;
	  end
	else
	  begin
	     r_priv <= n_priv;	    
	  end
     end // always_ff@ (posedge clk)

   always_ff@(negedge clk)
     begin
	if(r_priv != n_priv)
	  begin
	     $display("cycle %d, priv change %d -> %d, update_csr_exc %b, wr_priv %b\n", 
		      r_cycle, r_priv, n_priv, update_csr_exc, t_wr_priv);
	  end
     end
   

   logic [3:0] r_rd_pc_idx, n_rd_pc_idx;
   logic [3:0] r_wr_pc_idx, n_wr_pc_idx;
   logic [7:0] r_pc_buf [7:0];
   
   
   always_comb
     begin
	n_wr_pc_idx = r_wr_pc_idx;
	n_rd_pc_idx = r_rd_pc_idx;
	t_push_putchar = t_wr_csr_en & (RDBRANCH_CSR == int_uop.imm[5:0]);
	if(t_push_putchar)
	  begin
	     n_wr_pc_idx = r_wr_pc_idx + 'd1;
	  end
	if(putchar_fifo_pop)
	  begin
	     n_rd_pc_idx = r_rd_pc_idx + 'd1;
	  end
     end // always_comb

   always_ff@(posedge clk)
     begin
	r_wr_pc_idx <= reset ? 'd0 : n_wr_pc_idx;
	r_rd_pc_idx <= reset ? 'd0 : n_rd_pc_idx;
     end

   always_ff@(posedge clk)
     begin
	if(t_push_putchar)
	  begin
	     r_pc_buf[r_wr_pc_idx[2:0]] <= t_wr_csr[7:0];
	  end
     end
   assign putchar_fifo_out = r_pc_buf[r_rd_pc_idx[2:0]];
   assign putchar_fifo_empty = r_wr_pc_idx == r_rd_pc_idx;
   wire w_putchar_fifo_full = (r_wr_pc_idx[2:0] == r_rd_pc_idx[2:0]) & (r_wr_pc_idx[3] != r_rd_pc_idx[3]);

   always_comb
     begin
	t_rd_csr = 'd0;
	case(int_uop.imm[5:0])
	  SSTATUS:
	    t_rd_csr = r_mstatus & 64'h3000de133;
	  SIE:
	    t_rd_csr = r_mie & r_mideleg;
	  STVEC:
	    t_rd_csr = r_stvec;
	  SSCRATCH:
	    t_rd_csr = r_sscratch;
	  SEPC:
	    t_rd_csr = r_sepc;
	  SCAUSE:
	    t_rd_csr = r_scause;
	  SCOUNTEREN:
	    t_rd_csr = r_scounteren;
	  STVAL:
	    t_rd_csr = r_stval;
	  SIP:
	    t_rd_csr = r_mip & r_mideleg;
	  SATP:
	    t_rd_csr = r_satp;
	  MSTATUS:
	    t_rd_csr = r_mstatus;
	  MCAUSE:
	    t_rd_csr = r_mcause;
	  MCOUNTEREN:
	    t_rd_csr = r_mcounteren;
	  MISA:
	    t_rd_csr = r_misa;
	  MTVEC:
	    t_rd_csr = r_mtvec;
	  MIE:
	    t_rd_csr = r_mie;
	  MIDELEG:
	    t_rd_csr = r_mideleg;
	  MEDELEG:
	    t_rd_csr = r_medeleg;
	  MEPC:
	    t_rd_csr = r_mepc;
	  MSCRATCH:
	    t_rd_csr = r_mscratch;
	  MIP:
	    t_rd_csr = r_mip;
	  PMPADDR0:
	    t_rd_csr = r_pmpaddr0;
	  PMPADDR1:
	    t_rd_csr = r_pmpaddr1;
	  PMPADDR2:
	    t_rd_csr = r_pmpaddr2;
	  PMPADDR3:
	    t_rd_csr = r_pmpaddr3;
	  PMPCFG0:
	    t_rd_csr = r_pmpcfg0;
	  RDBRANCH_CSR:
	    begin
	       t_rd_csr = {63'd0, w_putchar_fifo_full};
	       //$display("w_putchar_fifo_full = %b, putchar_fifo_empty = %b, t_rd_csr = %x, int_uop.op = %d", 
	       //w_putchar_fifo_full, putchar_fifo_empty, t_rd_csr,int_uop.op);
	    end
	  RDFAULTEDBRANCH_CSR:
	    t_rd_csr = 'd0;
	  RDTIME_CSR:
`ifdef VERILATOR
	    t_rd_csr = csr_gettime();
`else
	  t_rd_csr = {20'd0, r_cycle[63:20]};
`endif
	  default:
	    begin
	       if(t_rd_csr_en)
		 begin
		    $display("read csr %d unimplemented for pc %x", int_uop.imm[5:0], int_uop.pc);
		    $stop();
		 end
	    end
	endcase
     end // always_comb

   logic r_satp_armed;
   logic n_paging_active;
   always_comb
     begin
	n_paging_active = r_satp_armed & (!r_priv[1]);
     end
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_paging_active <= 1'b0;
	  end
	else
	  begin
	     r_paging_active <= n_paging_active;
	  end
     end // always_ff@ (posedge clk)

   always_ff@(negedge clk)
     begin
	if(r_paging_active != n_paging_active)
	  $display("paging switch to %b from %b at %d", n_paging_active, r_paging_active, r_cycle);
     end


   logic t_push_putchar;

   wire [1:0] w_mpp = r_mstatus[12:11];
   wire	      w_spp = r_mstatus[8];
   wire	      w_mpie = r_mstatus[7];
   wire	      w_spie = r_mstatus[5];
   
   
   wire [3:0] w_mret_mstatus_b30 = 
	      w_mpp == 2'd0 ? {r_mstatus[3:1], w_mpie} :
	      w_mpp == 2'd1 ? {r_mstatus[3:2], w_mpie, r_mstatus[0]} :
	      w_mpp == 2'd2 ? {r_mstatus[3], w_mpie, r_mstatus[1:0]} :
	      {w_mpie, r_mstatus[2:0]};

   wire [1:0] w_sret_mstatus_b10 = 
	      w_spp ? {w_spie, r_mstatus[0]} : {r_mstatus[1], w_spie};

	      
   wire [63:0] w_mret_mstatus = {r_mstatus[63:13],
				 2'd0,
				 r_mstatus[10:8],
				 1'b1, /*mpie*/
				 r_mstatus[6:4],
				 w_mret_mstatus_b30
				 };

   wire [63:0] w_sret_mstatus = {r_mstatus[63:9],
				 1'd0, /* spp */
				 r_mstatus[7:6],
				 1'b1, /*spie*/
				 r_mstatus[4:2],
				 w_sret_mstatus_b10
				 };

   wire w_ie = 
	r_priv == 2'd0 ? r_mstatus[0] :
	r_priv == 2'd1 ? r_mstatus[1] :
	r_priv == 2'd2 ? r_mstatus[2] :
	r_mstatus[3];
   
   wire [63:0] w_exc_del_mstatus = {r_mstatus[63:9],
				    /* spp */ r_priv[0],
				    r_mstatus[7:6],
				    /* spie */ w_ie,
				    r_mstatus[4:2],
				    /* sie */ 1'b0,
				    r_mstatus[0]
				    };

   wire [63:0] w_exc_mstatus = {r_mstatus[63:13],
				/* mpp */ r_priv,
				r_mstatus[10:8],
				/* mpie */ w_ie, //bit 7
				r_mstatus[6:4],
				1'b0, /* mie */
				r_mstatus[2:0]
				};

   
   logic [63:0]	r_foo;
   always_ff@(posedge clk)
     begin
	r_foo <= reset ? 'd0 : r_mstatus;
     end
    always_ff@(negedge clk)
      begin
	if(r_start_int && int_uop.op == MRET)
	  begin
	     $display("MRET to %x from %x at cycle %d, mstatus %x old %x",
		      r_mepc, int_uop.pc, r_cycle, w_mret_mstatus, r_mstatus);
	  end
	//if(r_start_int && int_uop.op == SRET)
	//begin
	//   $display("SRET to %x from %x at cycle %d, mstatus %x",
	//	      r_mepc, int_uop.pc, r_cycle, w_mret_mstatus);
	//end	
     end
  
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_scounteren <= 'd0;
	     r_satp <= 'd0;
	     r_stval <= 'd0;
	     r_scause <= 'd0;
	     r_sepc <= 'd0;
	     
	     r_mcause <= 'd0;
	     
	     r_mie <= 'd0;
	     r_mip <= 'd0;
	     r_mstatus <= 64'ha00000000;
	     r_mtvec <= 'd0;
	     r_mcounteren <= 'd0;
	     r_mideleg <= 'd0;
	     r_medeleg <= 'd0;

	     r_pmpaddr0 <= 'd0;
	     r_pmpaddr1 <= 'd0;
	     r_pmpaddr2 <= 'd0;
	     r_pmpaddr3 <= 'd0;
	     r_pmpcfg0 <= 'd0;
	     r_mscratch <= 'd0;
	     r_mepc <= 'd0;
	     r_mtval <= 'd0;
	     r_satp_armed <= 1'b0;
	     r_misa <= 64'h8000000000141101;
	  end // if (reset)
	else if(update_csr_exc)
	  begin
	     $display("trapping, delegate = %b, epc = %x", t_delegate, epc);
	     if(t_delegate)
	       begin
		  $display("delegate cause %x, tval %x, epc %x, mstatus %x",
			   cause, tval, epc, w_exc_del_mstatus);
		  r_scause <= {irq, 58'd0, cause};
		  r_stval <= tval;
		  r_sepc <= epc;
		  r_mstatus <= w_exc_del_mstatus;
	       end
	     else
	       begin
		  r_mcause <= {irq, 58'd0, cause};
		  r_mtval <= tval;
		  r_mepc <= epc;
		  r_mstatus <= w_exc_mstatus;
	       end
	  end
	else if(t_wr_csr_en)
	  begin
	     case(int_uop.imm[5:0])
	       SSTATUS:
		 begin
		    r_mstatus <= (t_wr_csr & 64'hde133) | (r_mstatus & (~(64'hde133)));
		 end
	       SIE:
		 r_mie <= (r_mie & ~(r_mideleg)) | (t_wr_csr & r_mideleg);	       
	       STVEC:
		 r_stvec <= t_wr_csr;
	       SSCRATCH:
		 begin
		    r_sscratch <= t_wr_csr;
		 end
	       SEPC:
		 r_sepc <= t_wr_csr;
	       SCOUNTEREN:
		 r_scounteren <= t_wr_csr;
	       SCAUSE:
		 r_scause <= t_wr_csr;
	       STVAL:
		 r_stval <= t_wr_csr;
	       SIP:
		 r_mip <= (r_mip & ~(r_mideleg)) | (t_wr_csr & r_mideleg);
	       SATP:
		 begin
		    if((t_wr_csr[63:60] == 4'h8) && (t_wr_csr[59:44] == 'd0))
		      begin
			 r_satp_armed <= 1'b1;
			 r_satp <= t_wr_csr;
		      end
		    else if(t_wr_csr[63:60] == 4'h0)
		      begin
			 r_satp_armed <= 1'b0;			 
			 r_satp <= t_wr_csr;
		      end
		 end
	       MSTATUS:
		 begin
		    if(int_uop.op == MRET)
		      begin
			 //mpp : bits 12:11
			 //mpie : bit 7
			 r_mstatus <= w_mret_mstatus;
		      end
		    else if(int_uop.op == SRET)
		      begin
			 r_mstatus <= w_sret_mstatus;
		      end
		    else
		      begin
			 r_mstatus <= (t_wr_csr & 64'he79bb) | (r_mstatus & 64'hfffffffffff18644);
		      end
		 end
	       MCOUNTEREN:
		 r_mcounteren <= t_wr_csr;
	       MISA:
		 r_misa <= t_wr_csr;
	       MTVEC:
		 r_mtvec <= t_wr_csr;
	       MIE:
		 r_mie <= t_wr_csr;
	       MIP:
		 r_mip <= t_wr_csr;
	       MCAUSE:
		 r_mcause <= t_wr_csr;
	       MEDELEG:
		 r_medeleg <= t_wr_csr;
	       MIDELEG:
		 r_mideleg <= t_wr_csr;
	       MSCRATCH:
		 r_mscratch <= t_wr_csr;
	       MEPC:
		 r_mepc <= t_wr_csr;
	       PMPADDR0:
		 r_pmpaddr0 <= t_wr_csr;
	       PMPADDR1:
		 r_pmpaddr1 <= t_wr_csr;
	       PMPADDR2:
		 r_pmpaddr2 <= t_wr_csr;
	       PMPADDR3:
		 r_pmpaddr3 <= t_wr_csr;
	       PMPCFG0:
		 r_pmpcfg0 <= t_wr_csr;
	       RDBRANCH_CSR:
		 begin
		    //csr_putchar(t_wr_csr[7:0]);
		 end
`ifdef VERILATOR
	       RDFAULTEDBRANCH_CSR:
		 begin
		    term_sim();
		 end
`endif	       	       
	       default:
		 begin
		    $display("write csr implement %d for pc %x opcode %d", 
			     int_uop.imm[5:0], int_uop.pc, int_uop.op);
		    $stop();
		 end

	     endcase // case (int_uop.imm[4:0])
	  end // if (t_wr_csr_en)
	 // else if(1'b1)
	 // begin
	 //    r_mip <= 64'd128;
	 // end
     end // always_ff@ (posedge clk)



   
   wire [`M_WIDTH-1:0] w_agu_addr;
   mwidth_add agu (.A(t_mem_srcA), .B(mem_uq.rvimm), .Y(w_agu_addr));

   wire w_mem_srcA_ready = t_mem_uq.srcA_valid ? (!r_prf_inflight[t_mem_uq.srcA] | t_fwd_int_mem_srcA | t_fwd_int2_mem_srcA | t_fwd_mem_mem_srcA) : 1'b1;


   wire w_dq_ready = !r_prf_inflight[t_mem_dq.src_ptr] | t_fwd_int_mem_srcB | t_fwd_mem_mem_srcB | t_fwd_int2_mem_srcB;
   	
   always_comb
     begin
	t_pop_mem_uq = (!t_mem_uq_empty) && (!(mem_q_next_full||mem_q_full)) && w_mem_srcA_ready && !t_flash_clear;

	t_pop_mem_dq = (!t_mem_dq_empty) && !mem_dq_clr && w_dq_ready
		       && (!(mem_mdq_next_full||mem_mdq_full)) ;
     end


   //need another queue to hold store data
   
   always_comb
     begin
	t_core_store_data.rob_ptr = mem_dq.rob_ptr;
	t_core_store_data.data = t_mem_srcB;
`ifdef ENABLE_CYCLE_ACCOUNTING
	t_core_store_data.fetch_cycle = mem_dq.fetch_cycle;
`endif
	core_store_data_ptr = mem_dq.rob_ptr;
	core_store_data_ptr_valid = r_dq_ready;
     end

   always_ff@(posedge clk)
     begin
	if(r_dq_ready)
	  begin
	     r_mdq[r_mdq_tail_ptr[`LG_MQ_ENTRIES-1:0]] <= t_core_store_data;
	  end
     end

   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mem_ready <= 1'b0;
	     r_dq_ready <= 1'b0;
	  end
	else
	  begin
	     r_mem_ready <= t_pop_mem_uq;
	     r_dq_ready <= t_pop_mem_dq;
	  end
     end // always_ff@ (posedge clk)

   
   //cases were address wraps the cacheline
   
   wire w_bad_16b_addr = &w_agu_addr[3:0];
   wire w_bad_32b_addr = (&w_agu_addr[3:2]) & (|w_agu_addr[1:0]);
   wire	w_bad_64b_addr = w_agu_addr[3] & (|w_agu_addr[2:0]);

   logic [3:0] r_restart_counter;
   always_ff@(posedge clk)
     begin
	r_restart_counter <= reset ? 'd0 : 
			     (restart_complete ? r_restart_counter + 'd1 : r_restart_counter);
     end
   
   always_comb
     begin
	t_mem_tail.op = MEM_LW;
	t_mem_tail.addr = w_agu_addr;
	t_mem_tail.rob_ptr = mem_uq.rob_ptr;
	t_mem_tail.dst_valid = 1'b0;
	t_mem_tail.dst_ptr = mem_uq.dst;
	t_mem_tail.is_load = 1'b0;
	t_mem_tail.is_store = 1'b0;
	t_mem_tail.is_atomic = 1'b0;
	t_mem_tail.amo_op = mem_uq.jmp_imm[4:0];
	t_mem_tail.data = 'd0;
	t_mem_tail.spans_cacheline = 1'b0;
	t_mem_tail.unaligned = 1'b0;
	t_mem_tail.pc = mem_uq.pc;
	t_mem_tail.has_cause = 1'b0;
	t_mem_tail.cause = MISALIGNED_FETCH;
`ifdef ENABLE_CYCLE_ACCOUNTING
	t_mem_tail.fetch_cycle = mem_uq.fetch_cycle;
	t_mem_tail.restart_id = r_restart_counter;
`endif
	case(mem_uq.op)
	  SB:
	    begin
	       t_mem_tail.op = MEM_SB;
	       t_mem_tail.is_store = 1'b1;
	       t_mem_tail.dst_valid = 1'b0;
	    end // case: SB
	  SH:
	    begin
	       t_mem_tail.op = w_bad_16b_addr ? MEM_NOP : MEM_SH;
	       t_mem_tail.is_store = ~w_bad_16b_addr;
	       t_mem_tail.dst_valid = 1'b0;
	       t_mem_tail.spans_cacheline = w_bad_16b_addr;
	       t_mem_tail.unaligned = w_agu_addr[0];
	    end // case: SW
	  SW:
	    begin
	       t_mem_tail.op = w_bad_32b_addr ? MEM_NOP : MEM_SW;
	       t_mem_tail.is_store = ~w_bad_32b_addr;
	       t_mem_tail.dst_valid = 1'b0;
	       t_mem_tail.spans_cacheline = w_bad_32b_addr;
	       t_mem_tail.unaligned = |w_agu_addr[1:0];
	    end // case: SW
	  SD:
	    begin
	       t_mem_tail.op = w_bad_64b_addr ? MEM_NOP : MEM_SD;
	       t_mem_tail.is_store = ~w_bad_64b_addr;	       
	       t_mem_tail.dst_valid = 1'b0;
	       t_mem_tail.spans_cacheline = w_bad_64b_addr;
	       t_mem_tail.unaligned = |w_agu_addr[2:0];
	    end // case: SW
	  SCW:
	    begin
	       t_mem_tail.op = MEM_SCW;
	       t_mem_tail.is_atomic = 1'b1;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.dst_ptr = mem_uq.dst;
	       t_mem_tail.spans_cacheline = (w_agu_addr[1:0] != 2'd0);
	       t_mem_tail.unaligned = |w_agu_addr[1:0];
	    end // case: SW
	  SCD:
	    begin
	       t_mem_tail.op = MEM_SCD;
	       t_mem_tail.is_atomic = 1'b1;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.dst_ptr = mem_uq.dst;
	       t_mem_tail.spans_cacheline = (w_agu_addr[2:0] != 3'd0);
	       t_mem_tail.unaligned = |w_agu_addr[2:0];
	    end // case: SW
	  AMOW:
	    begin
	       t_mem_tail.op = MEM_AMOW;
	       t_mem_tail.is_atomic = 1'b1;	       
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.dst_ptr = mem_uq.dst;
	       t_mem_tail.spans_cacheline = (w_agu_addr[1:0] != 2'd0);
	       t_mem_tail.unaligned = |w_agu_addr[1:0];
	    end // case: SW
	 AMOD:
	    begin
	       t_mem_tail.op = MEM_AMOD;
	       t_mem_tail.is_atomic = 1'b1;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.dst_ptr = mem_uq.dst;
	       t_mem_tail.spans_cacheline = (w_agu_addr[2:0] != 3'd0);
	       t_mem_tail.unaligned = |w_agu_addr[2:0];
	    end // case: SW	  
	  
	  LW:
	    begin
	       t_mem_tail.is_load = 1'b1;
	       t_mem_tail.op = w_bad_32b_addr ? MEM_NOP : MEM_LW;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.spans_cacheline = w_bad_32b_addr;
	       t_mem_tail.unaligned = |w_agu_addr[1:0];
	    end // case: LW
	  LWU:
	    begin
	       t_mem_tail.is_load = 1'b1;
	       t_mem_tail.op = w_bad_32b_addr ? MEM_NOP : MEM_LWU;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.spans_cacheline = w_bad_32b_addr;
	       t_mem_tail.unaligned = |w_agu_addr[1:0];
	    end // case: LW	  
	  LD:
	    begin
	       t_mem_tail.is_load = 1'b1;
	       t_mem_tail.op = w_bad_64b_addr ? MEM_NOP : MEM_LD;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.spans_cacheline = w_bad_64b_addr;
	       t_mem_tail.unaligned = |w_agu_addr[2:0];
	    end // case: LW
	  LB:
	    begin
	       t_mem_tail.is_load = 1'b1;	       
	       t_mem_tail.op = MEM_LB;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	    end
	  LBU:
	    begin
	       t_mem_tail.is_load = 1'b1;	       
	       t_mem_tail.op = MEM_LBU;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	    end // case: LBU
	  LHU:
	    begin
	       t_mem_tail.is_load = 1'b1;
	       t_mem_tail.op = MEM_LHU;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.spans_cacheline = w_agu_addr[0];
	       t_mem_tail.unaligned = w_agu_addr[0];
	    end // case: LBU
	  LH:
	    begin
	       t_mem_tail.is_load = 1'b1;
	       t_mem_tail.op = w_bad_16b_addr ? MEM_NOP : MEM_LH;
	       t_mem_tail.dst_valid = mem_uq.dst_valid;
	       t_mem_tail.spans_cacheline = w_bad_16b_addr;
	       t_mem_tail.unaligned = w_agu_addr[0];
	    end // case: LH
	  
	  default:
	    begin
	       
	    end
	endcase // case (mem_uq.op)

     end // always_comb

   

   always_ff@(posedge clk)
     begin
	r_int_result <= t_result;
	r_int_result2 <= t_result2;
	r_mem_result <= mem_rsp_load_data;
     end

   always_comb
     begin
	t_fwd_int_mem_srcA = r_start_int && t_wr_int_prf &&(t_mem_uq.srcA == int_uop.dst);
	t_fwd_int_mem_srcB = r_start_int && t_wr_int_prf &&(t_mem_dq.src_ptr == int_uop.dst);
	t_fwd_int2_mem_srcA = r_start_int2 && t_wr_int_prf2 &&(t_mem_uq.srcA == int_uop2.dst);
	t_fwd_int2_mem_srcB = r_start_int2 && t_wr_int_prf2 &&(t_mem_dq.src_ptr == int_uop2.dst);
	
	t_fwd_mem_mem_srcA = mem_rsp_dst_valid && (t_mem_uq.srcA == mem_rsp_dst_ptr);
	t_fwd_mem_mem_srcB = mem_rsp_dst_valid && (t_mem_dq.src_ptr == mem_rsp_dst_ptr);
     end
   
   always_ff@(posedge clk)
     begin
	r_fwd_int_mem_srcA <= t_fwd_int_mem_srcA;
	r_fwd_int_mem_srcB <= t_fwd_int_mem_srcB;
	r_fwd_int2_mem_srcA <= t_fwd_int2_mem_srcA;
	r_fwd_int2_mem_srcB <= t_fwd_int2_mem_srcB;
	
	r_fwd_mem_mem_srcA <= t_fwd_mem_mem_srcA;
	r_fwd_mem_mem_srcB <= t_fwd_mem_mem_srcB;
	
	r_fwd_int_srcA <= r_start_int && t_wr_int_prf && (t_picked_uop.srcA == int_uop.dst);
	r_fwd_int_srcB <= r_start_int && t_wr_int_prf && (t_picked_uop.srcB == int_uop.dst);
	r_fwd_int2_srcA <= r_start_int2 && t_wr_int_prf2 && (t_picked_uop.srcA == int_uop2.dst);
	r_fwd_int2_srcB <= r_start_int2 && t_wr_int_prf2 && (t_picked_uop.srcB == int_uop2.dst);

	r_fwd_int_srcA2 <= r_start_int && t_wr_int_prf && (t_picked_uop2.srcA == int_uop.dst);
	r_fwd_int_srcB2 <= r_start_int && t_wr_int_prf && (t_picked_uop2.srcB == int_uop.dst);
	r_fwd_int2_srcA2 <= r_start_int2 && t_wr_int_prf2 && (t_picked_uop2.srcA == int_uop2.dst);
	r_fwd_int2_srcB2 <= r_start_int2 && t_wr_int_prf2 && (t_picked_uop2.srcB == int_uop2.dst);
	
	
	r_fwd_mem_srcA <= mem_rsp_dst_valid && (t_picked_uop.srcA == mem_rsp_dst_ptr);
	r_fwd_mem_srcB <= mem_rsp_dst_valid && (t_picked_uop.srcB == mem_rsp_dst_ptr);
	r_fwd_mem_srcA2 <= mem_rsp_dst_valid && (t_picked_uop2.srcA == mem_rsp_dst_ptr);
	r_fwd_mem_srcB2 <= mem_rsp_dst_valid && (t_picked_uop2.srcB == mem_rsp_dst_ptr);	
     end

   // always@(negedge clk)
   //   begin
   // 	if(r_fwd_int2_srcA | r_fwd_int2_srcB)
   // 	  begin
   // 	     $display("need forward (1->2) %b, %b, int_uop.pc %x",
   // 		      r_fwd_int2_srcA, r_fwd_int2_srcB, int_uop.pc);
   // 	     //$stop();
   // 	  end
	
   // 	if(r_fwd_int_srcA2 | r_fwd_int_srcB2)
   // 	  begin
   // 	     $display("need forward (1->2) %b, %b, %x, %x, %x, %x, int_uop2.pc %x",
   // 		      r_fwd_int_srcA2, r_fwd_int_srcB2,
   // 		      t_srcA_2, t_srcB_2,
   // 		      w_srcA_2, w_srcB_2,
   // 		      int_uop2.pc);
   // 	     //$stop();
   // 	  end
	
   // 	if(r_fwd_int2_srcA2 | r_fwd_int2_srcB2)
   // 	  begin
   // 	     $display("need forward (2->2) %b, %b, int_uop2.pc %x",
   // 		      r_fwd_int2_srcA2, r_fwd_int2_srcB2, int_uop2.pc);
	     
   // 	      //   $stop();
   // 	   end	
   //   end
   
   
   // always_ff@(negedge clk)
   //   begin
	
   // 	$display("at cycle %d, mul ptr = %d %b, div ptr = %d %b, int_uop.dst %d", 
   // 		 r_cycle, 
   // 		 w_mul_prf_ptr, t_mul_complete,
   // 		 w_div_prf_ptr, t_div_complete,
   // 		 int_uop.dst);
   //   end
   

   rf6r3w #(.WIDTH(`M_WIDTH), .LG_DEPTH(`LG_PRF_ENTRIES)) 
   intprf (.clk(clk),
	   .rdptr0(t_picked_uop.srcA),
	   .rdptr1(t_picked_uop.srcB),
	   .rdptr2(t_mem_uq.srcA),
	   .rdptr3(t_mem_dq.src_ptr),
	   .rdptr4(t_picked_uop2.srcA),
	   .rdptr5(t_picked_uop2.srcB),
	   .wrptr0(t_mul_complete ? w_mul_prf_ptr :
		   t_div_complete ? w_div_prf_ptr :
		   int_uop.dst),
	   .wrptr1(mem_rsp_dst_ptr),
	   .wrptr2(int_uop2.dst),
	   .wen0(t_mul_complete | t_div_complete | (r_start_int & t_wr_int_prf)),
	   .wen1(mem_rsp_dst_valid),
	   .wen2(r_start_int2 & t_wr_int_prf2),
	   .wr0(t_mul_complete ? t_mul_result :
		t_div_complete ? t_div_result :
		t_result),
	   .wr1(mem_rsp_load_data),
	   .wr2(t_result2),
	   .rd0(w_srcA),
	   .rd1(w_srcB),
	   .rd2(w_mem_srcA),
	   .rd3(w_mem_srcB),
	   .rd4(w_srcA_2),
	   .rd5(w_srcB_2)
	   
	   );
   
   // always_ff@(negedge clk)
   //   begin
   // 	if(mem_rsp_dst_valid)
   // 	  begin
   // 	     $display("mem writeback value %x", mem_rsp_load_data);
   // 	  end
   //   end

   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     complete_valid_1 <= 1'b0;
	     complete_valid_2 <= 1'b0;
	  end
	else
	  begin
	     complete_valid_1 <= r_start_int && t_alu_valid || t_mul_complete || t_div_complete;
	     complete_valid_2 <= r_start_int2;
	  end
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	complete_bundle_2.rob_ptr <= int_uop2.rob_ptr;
	complete_bundle_2.complete <= t_alu_valid2;
	complete_bundle_2.faulted <= t_mispred_br2;
	complete_bundle_2.restart_pc <= t_pc_2;
	complete_bundle_2.cause <= MISALIGNED_FETCH;
	complete_bundle_2.has_cause <= 1'b0;
	complete_bundle_2.take_br <= t_take_br2;
	complete_bundle_2.data <= t_result2;
     end
     

   
   always_ff@(posedge clk)
     begin
	if(t_mul_complete || t_div_complete)
	  begin
	     complete_bundle_1.rob_ptr <= t_mul_complete ? 
					  t_rob_ptr_out : 
					  t_div_rob_ptr;
	     complete_bundle_1.complete <= 1'b1;
	     complete_bundle_1.faulted <= 1'b0;
	     complete_bundle_1.restart_pc <= 'd0;
	     complete_bundle_1.cause <= MISALIGNED_FETCH;
	     complete_bundle_1.has_cause <= 1'b0;	     
	     complete_bundle_1.take_br <= 1'b0;
	     complete_bundle_1.data <= t_mul_complete ? t_mul_result : t_div_result;
	  end
	else
	  begin
	     complete_bundle_1.rob_ptr <= int_uop.rob_ptr;
	     complete_bundle_1.complete <= t_alu_valid;
	     complete_bundle_1.faulted <= t_mispred_br || t_has_cause;
	     complete_bundle_1.restart_pc <= t_pc;
	     complete_bundle_1.cause <= t_cause;
	     complete_bundle_1.has_cause <= t_has_cause;
	     complete_bundle_1.take_br <= t_take_br;
	     complete_bundle_1.data <= t_result;
	  end
	//(uq.rob_ptr == 'd5) ? 1'b1 : 1'b0;
     end


endmodule
