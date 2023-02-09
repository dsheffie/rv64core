`include "uop.vh"
`include "rob.vh"

`ifdef DEBUG_FPU
import "DPI-C" function int fp64_to_fp32(input longint a);
import "DPI-C" function longint fp32_to_fp64(input int a);
`endif

`ifdef VERILATOR
import "DPI-C" function void report_exec(input int int_valid, 
					 input int int_blocked,
					 input int mem_valid, 
					 input int mem_blocked,
					 input int fp_valid, 
					 input int fp_blocked,
					 input int iq_full,
					 input int mq_full,
					 input int fq_full,
					 input int blocked_by_store,
					 input int int_ready
					 );
`endif

module exec(clk, 
	    reset,
`ifdef VERILATOR
	    clear_cnt,
`endif
	    divide_ready,
	    ds_done,
	    machine_clr,
	    restart_complete,
	    delayslot_rob_ptr,
	    cpr0_status_reg,
	    uq_wait,
	    mq_wait,
	    fq_wait,
	    uq_empty,
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
	    exception_wr_cpr0_val,
	    exception_wr_cpr0_ptr,
	    exception_wr_cpr0_data,
	    mem_req, 
	    mem_req_valid, 
	    mem_req_ack,
	    mem_rsp_dst_ptr,
	    mem_rsp_dst_valid,
	    mem_rsp_rob_ptr,
	    mem_rsp_fp_dst_valid,
	    mem_rsp_load_data,
	    monitor_rsp_data);
   input logic clk;
   input logic reset;
`ifdef VERILATOR
   input logic [31:0] clear_cnt;
`endif
   output logic       divide_ready;
   input logic ds_done;
   input logic machine_clr;
   input logic restart_complete;
   input logic [`LG_ROB_ENTRIES-1:0] delayslot_rob_ptr;
   output logic [(`M_WIDTH-1):0]     cpr0_status_reg;
   
   localparam N_ROB_ENTRIES = (1<<`LG_ROB_ENTRIES);   
   output logic [N_ROB_ENTRIES-1:0]  uq_wait;   
   output logic [N_ROB_ENTRIES-1:0]  mq_wait;
   output logic [N_ROB_ENTRIES-1:0]  fq_wait;
   
   output logic 		     uq_empty;
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

   input logic 	exception_wr_cpr0_val;
   input logic [4:0] exception_wr_cpr0_ptr;
   input logic [`M_WIDTH-1:0] exception_wr_cpr0_data;
   
   output 	mem_req_t mem_req;
   output 	logic mem_req_valid;
   input logic 	      mem_req_ack;
   
   input logic [`LG_PRF_ENTRIES-1:0] mem_rsp_dst_ptr;
   input logic 			     mem_rsp_dst_valid;
   input logic 			     mem_rsp_fp_dst_valid;
   input logic [63:0] 		     mem_rsp_load_data;
   input logic [`LG_ROB_ENTRIES-1:0] mem_rsp_rob_ptr;
     
   input logic [`M_WIDTH-1:0] monitor_rsp_data;
   
   
   localparam N_MQ_ENTRIES = (1<<`LG_MQ_ENTRIES);
   localparam N_INT_PRF_ENTRIES = (1<<`LG_PRF_ENTRIES);
   localparam N_HILO_PRF_ENTRIES = (1<<`LG_HILO_PRF_ENTRIES);
   localparam N_FCR_PRF_ENTRIES = (1<<`LG_FCR_PRF_ENTRIES);
   
   localparam N_FP_PRF_ENTRIES = (1<<`LG_PRF_ENTRIES);
   localparam N_UQ_ENTRIES = (1<<`LG_UQ_ENTRIES);
   localparam N_MEM_UQ_ENTRIES = (1<<`LG_MEM_UQ_ENTRIES);
   localparam N_FP_UQ_ENTRIES = (1<<`LG_FP_UQ_ENTRIES);
      
   logic [(`M_WIDTH-1):0] r_int_prf [N_INT_PRF_ENTRIES-1:0];
   logic [63:0] r_fp_prf [N_FP_PRF_ENTRIES-1:0];
   logic [63:0] r_hilo_prf[N_HILO_PRF_ENTRIES-1:0];
   logic [7:0] 	r_fcr_prf[N_FCR_PRF_ENTRIES-1:0];
   
   localparam FP_ZP = (`LG_PRF_ENTRIES-5);
   localparam Z_BITS = 64-`M_WIDTH;      
   
   logic [N_INT_PRF_ENTRIES-1:0]  r_prf_inflight, n_prf_inflight;
   logic [N_FP_PRF_ENTRIES-1:0]   r_fp_prf_inflight, n_fp_prf_inflight;
   logic [N_FCR_PRF_ENTRIES-1:0]  r_fcr_prf_inflight, n_fcr_prf_inflight;
   logic [N_HILO_PRF_ENTRIES-1:0] r_hilo_inflight, n_hilo_inflight;


   logic [63:0] 		  t_fpu_result;
   logic 			  t_fpu_result_valid;
   logic 			  t_fpu_fcr_valid;
   logic [`LG_FCR_PRF_ENTRIES-1:0] t_fpu_fcr_ptr;
   

   logic [`LG_PRF_ENTRIES-1:0] t_fpu_dst_ptr;
   logic [`LG_ROB_ENTRIES-1:0] t_fpu_rob_ptr;

   logic 		       t_sp_div_valid, t_dp_div_valid;
   logic [63:0] 	       t_dp_div_result;
   logic [31:0] 	       t_sp_div_result;
   logic [`LG_PRF_ENTRIES-1:0] t_sp_div_dst_ptr;
   logic [`LG_ROB_ENTRIES-1:0] t_sp_div_rob_ptr;
   logic [`LG_PRF_ENTRIES-1:0] t_dp_div_dst_ptr;
   logic [`LG_ROB_ENTRIES-1:0] t_dp_div_rob_ptr;

   
   
   logic 			  t_wr_int_prf, t_wr_cpr0;
   
   logic 	t_wr_hilo;
   logic 	t_take_br;
   logic 	t_mispred_br;
   logic 	t_alu_valid;
   logic 	t_got_break;

      
   mem_req_t r_mem_q[N_MQ_ENTRIES-1:0];
   logic [`LG_MQ_ENTRIES:0] r_mq_head_ptr, n_mq_head_ptr;
   logic [`LG_MQ_ENTRIES:0] r_mq_tail_ptr, n_mq_tail_ptr;
   mem_req_t t_mem_tail, t_mem_head;
   logic 		    mem_q_full, mem_q_empty;
   

   logic 	t_pop_uq,t_pop_mem_uq,t_pop_fp_uq;
   logic 	t_push_mq;

   logic 	t_start_fpu;
   
   
   localparam E_BITS = `M_WIDTH-16;
   localparam HI_EBITS = `M_WIDTH-32;
   
   logic [`M_WIDTH-1:0] t_simm, t_mem_simm;
   logic [`M_WIDTH-1:0] t_result;
   logic [`M_WIDTH-1:0] t_cpr0_result;

   
   logic [63:0] t_hilo_result;
   
   logic [`M_WIDTH-1:0] t_pc, t_pc4, t_pc8;
   logic [27:0] t_jaddr;
   logic 	t_srcs_rdy, t_fp_srcs_rdy;
   logic 	t_fp_wr_prf;
   logic [63:0] t_fp_result;
   

   logic [31:0] r_srcA, r_srcB;
   
   logic [31:0] r_mem_result, r_int_result;
   logic 	r_fwd_int_srcA, r_fwd_int_srcB;
   logic 	r_fwd_mem_srcA, r_fwd_mem_srcB;

   logic [63:0] r_int_hilo, r_mul_hilo, r_div_hilo;
   logic [63:0] r_src_hilo;
   logic 	r_fwd_hilo_int, r_fwd_hilo_mul, r_fwd_hilo_div;
      
   logic [31:0] t_srcA, t_srcB;
   logic [31:0] t_mem_srcA, t_mem_srcB;
   
   logic [63:0] 	t_fp_srcA, t_fp_srcB, t_fp_srcC;
   logic [63:0] 	t_mem_fp_srcB, t_mem_fp_srcC;
   
   
   logic [63:0] t_src_hilo;
   logic [`M_WIDTH-1:0] t_cpr0_srcA;
   
   
   logic 	t_unimp_op;
   logic 	t_fault;
   
   logic 	t_signed_shift;
   logic [4:0] 	t_shift_amt;
   
   logic [31:0] t_shift_right;

   logic 	t_start_mul;
   logic 	t_mul_complete;
   logic [63:0] t_mul_result;
   
   logic 	t_hilo_prf_ptr_val_out;
   logic [`LG_ROB_ENTRIES-1:0] t_rob_ptr_out;

   
   logic [`LG_HILO_PRF_ENTRIES-1:0] t_hilo_prf_ptr_out;
   
   logic [`MAX_LAT:0] r_wb_bitvec, n_wb_bitvec;
   logic [`FP_MAX_LAT:0] r_fp_wb_bitvec, n_fp_wb_bitvec;

   /* divider */
   logic 	t_div_ready, t_signed_div, t_start_div32;
   logic [`LG_ROB_ENTRIES-1:0] t_div_rob_ptr_out;
   logic [63:0] 	       t_div_result;
   logic [`LG_HILO_PRF_ENTRIES-1:0] t_div_hilo_prf_ptr_out;
   logic 			    t_div_complete;

   logic [N_ROB_ENTRIES-1:0] 	    r_uq_wait, r_mq_wait, r_fq_wait;
   /* non mem uop queue */
   uop_t r_uq[N_UQ_ENTRIES];
   uop_t uq, int_uop;
   logic 			    r_start_int;
   
   
   
   logic 			    t_uq_read, t_uq_empty, t_uq_full, t_uq_next_full;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_head_ptr, n_uq_head_ptr;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_tail_ptr, n_uq_tail_ptr;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_next_head_ptr, n_uq_next_head_ptr;
   logic [`LG_UQ_ENTRIES:0] 	    r_uq_next_tail_ptr, n_uq_next_tail_ptr;

   /* mem uop queue */
   uop_t r_mem_uq[N_MEM_UQ_ENTRIES];
   uop_t mem_uq;
   logic 	      t_mem_uq_read, t_mem_uq_empty, t_mem_uq_full,
		      t_mem_uq_next_full;
   logic [`LG_MEM_UQ_ENTRIES:0]  r_mem_uq_head_ptr, n_mem_uq_head_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0]  r_mem_uq_tail_ptr, n_mem_uq_tail_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0] r_mem_uq_next_head_ptr, n_mem_uq_next_head_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0] r_mem_uq_next_tail_ptr, n_mem_uq_next_tail_ptr;

   /* fp uop queue */
   uop_t r_fp_uq[N_FP_UQ_ENTRIES];
   uop_t fp_uq;
   logic 	      t_fp_uq_read, t_fp_uq_empty, t_fp_uq_full, 
		      t_fp_uq_next_full;
   logic 	      t_push_two_mem, t_push_two_fp, t_push_two_int;
   logic 	      t_push_one_mem, t_push_one_fp, t_push_one_int;
   
   logic [`LG_FP_UQ_ENTRIES:0]  r_fp_uq_head_ptr, n_fp_uq_head_ptr;
   logic [`LG_FP_UQ_ENTRIES:0]  r_fp_uq_tail_ptr, n_fp_uq_tail_ptr;
   logic [`LG_FP_UQ_ENTRIES:0]  r_fp_uq_next_head_ptr, n_fp_uq_next_head_ptr;
   logic [`LG_FP_UQ_ENTRIES:0]  r_fp_uq_next_tail_ptr, n_fp_uq_next_tail_ptr;

   logic 			t_flash_clear;
   always_comb
     begin
	t_flash_clear = ds_done;
	
     end

   always_comb
     begin
	uq_full = t_uq_full || t_mem_uq_full || t_fp_uq_full;
	uq_next_full = t_uq_next_full || t_mem_uq_next_full || t_fp_uq_next_full;
	uq_empty = t_uq_empty;
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


   always_comb
     begin
	n_mem_uq_head_ptr = r_mem_uq_head_ptr;
	n_mem_uq_tail_ptr = r_mem_uq_tail_ptr;
	n_mem_uq_next_head_ptr = r_mem_uq_next_head_ptr;
	n_mem_uq_next_tail_ptr = r_mem_uq_next_tail_ptr;
	
	t_mem_uq_empty = (r_mem_uq_head_ptr == r_mem_uq_tail_ptr);
	t_mem_uq_full = (r_mem_uq_head_ptr != r_mem_uq_tail_ptr) && (r_mem_uq_head_ptr[`LG_MEM_UQ_ENTRIES-1:0] == r_mem_uq_tail_ptr[`LG_MEM_UQ_ENTRIES-1:0]);

	t_mem_uq_next_full = (r_mem_uq_head_ptr != r_mem_uq_next_tail_ptr) && 
			     (r_mem_uq_head_ptr[`LG_MEM_UQ_ENTRIES-1:0] == r_mem_uq_next_tail_ptr[`LG_MEM_UQ_ENTRIES-1:0]);

	
	mem_uq = r_mem_uq[r_mem_uq_head_ptr[`LG_MEM_UQ_ENTRIES-1:0]];

	t_push_two_mem = uq_push && uq_push_two && uq_uop.is_mem && uq_uop_two.is_mem;
	t_push_one_mem = ((uq_push && uq_uop.is_mem) || (uq_push_two && uq_uop_two.is_mem)) && !t_push_two_mem;
	
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
     end // always_comb


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_mq_wait <= 'd0;
	     r_uq_wait <= 'd0;
	     r_fq_wait <= 'd0;
	  end
	else if(restart_complete)
	  begin
	     r_mq_wait <= 'd0;
	     r_uq_wait <= 'd0;
	     r_fq_wait <= 'd0;
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
		  r_mq_wait[mem_uq.rob_ptr] <= 1'b0;		  
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
	     //fp port
	     if(t_push_two_fp)
	       begin
		  r_fq_wait[uq_uop.rob_ptr] <= 1'b1;
		  r_fq_wait[uq_uop_two.rob_ptr] <= 1'b1;		  
	       end
	     else if(t_push_one_fp)
	       begin
		  r_fq_wait[uq_uop.is_fp ? uq_uop.rob_ptr : uq_uop_two.rob_ptr] <= 1'b1;
	       end
	     if(t_pop_fp_uq)
	       begin
		  r_fq_wait[fp_uq.rob_ptr] <= 1'b0;
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

   
   always_ff@(posedge clk)
     begin
	if(reset || t_flash_clear)
	  begin
	     r_fp_uq_head_ptr <= 'd0;
	     r_fp_uq_tail_ptr <= 'd0;
	     r_fp_uq_next_head_ptr <= 'd1;
	     r_fp_uq_next_tail_ptr <= 'd1;
	     
	  end
	else
	  begin
	     r_fp_uq_head_ptr <= n_fp_uq_head_ptr;
	     r_fp_uq_tail_ptr <= n_fp_uq_tail_ptr;
	     r_fp_uq_next_head_ptr <= n_fp_uq_next_head_ptr;
	     r_fp_uq_next_tail_ptr <= n_fp_uq_next_tail_ptr;
	  end
     end
   
   always_comb
     begin
	n_fp_uq_head_ptr = r_fp_uq_head_ptr;
	n_fp_uq_tail_ptr = r_fp_uq_tail_ptr;
	n_fp_uq_next_head_ptr = r_fp_uq_next_head_ptr;
	n_fp_uq_next_tail_ptr = r_fp_uq_next_tail_ptr;
	
	t_fp_uq_empty = (r_fp_uq_head_ptr == r_fp_uq_tail_ptr);
	t_fp_uq_full = (r_fp_uq_head_ptr != r_fp_uq_tail_ptr) && (r_fp_uq_head_ptr[`LG_FP_UQ_ENTRIES-1:0] == r_fp_uq_tail_ptr[`LG_FP_UQ_ENTRIES-1:0]);
	t_fp_uq_next_full = (r_fp_uq_head_ptr != r_fp_uq_next_tail_ptr) && 
			    (r_fp_uq_head_ptr[`LG_FP_UQ_ENTRIES-1:0] == r_fp_uq_next_tail_ptr[`LG_FP_UQ_ENTRIES-1:0]);
	
	fp_uq = r_fp_uq[r_fp_uq_head_ptr[`LG_FP_UQ_ENTRIES-1:0]];

	t_push_two_fp = uq_push && uq_push_two && uq_uop.is_fp && uq_uop_two.is_fp;
	t_push_one_fp = ((uq_push && uq_uop.is_fp) || (uq_push_two && uq_uop_two.is_fp)) && !t_push_two_fp;
	
	if(t_push_two_fp)
	  begin
	     n_fp_uq_tail_ptr = r_fp_uq_tail_ptr + 'd2;
	     n_fp_uq_next_tail_ptr = r_fp_uq_next_tail_ptr + 'd2;
	  end
	else if(uq_push_two && uq_uop_two.is_fp || uq_push && uq_uop.is_fp)
	  begin
	     n_fp_uq_tail_ptr = r_fp_uq_tail_ptr + 'd1;
	     n_fp_uq_next_tail_ptr = r_fp_uq_next_tail_ptr + 'd1;
	  end
	
	if(t_pop_fp_uq)
	  begin
	     n_fp_uq_head_ptr = r_fp_uq_head_ptr + 'd1;
	  end
     end // always_comb


   always_ff@(posedge clk)
     begin
	if(t_push_two_fp)
	  begin
	     r_fp_uq[r_fp_uq_tail_ptr[`LG_FP_UQ_ENTRIES-1:0]] <= uq_uop;
	     r_fp_uq[r_fp_uq_next_tail_ptr[`LG_FP_UQ_ENTRIES-1:0]] <= uq_uop_two;
	  end
	else if(t_push_one_fp)
	  begin
	     r_fp_uq[r_fp_uq_tail_ptr[`LG_FP_UQ_ENTRIES-1:0]] <= uq_uop.is_fp ? uq_uop : uq_uop_two;	     
	  end
	
     end

   
   always_comb
     begin
	n_uq_head_ptr = r_uq_head_ptr;
	n_uq_tail_ptr = r_uq_tail_ptr;
	n_uq_next_head_ptr = r_uq_next_head_ptr;
	n_uq_next_tail_ptr = r_uq_next_tail_ptr;
	
	
	t_uq_empty = (r_uq_head_ptr == r_uq_tail_ptr);
	t_uq_full = (r_uq_head_ptr != r_uq_tail_ptr) && 
		    (r_uq_head_ptr[`LG_UQ_ENTRIES-1:0] == r_uq_tail_ptr[`LG_UQ_ENTRIES-1:0]);
	
	t_uq_next_full = (r_uq_head_ptr != r_uq_next_tail_ptr) && 
			 (r_uq_head_ptr[`LG_UQ_ENTRIES-1:0] == r_uq_next_tail_ptr[`LG_UQ_ENTRIES-1:0]);

	t_push_two_int = uq_push && uq_push_two && uq_uop.is_int && uq_uop_two.is_int;
	t_push_one_int = ((uq_push && uq_uop.is_int) || (uq_push_two && uq_uop_two.is_int)) && !t_push_two_int;
	
	uq = r_uq[r_uq_head_ptr[`LG_UQ_ENTRIES-1:0]];
	
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

	
	if(t_pop_uq)
	  begin
	     n_uq_head_ptr = r_uq_head_ptr + 'd1;
	  end
     end // always_comb

   always_ff@(posedge clk)
     begin
	if(t_push_two_int)
	  begin
	     r_uq[r_uq_tail_ptr[`LG_UQ_ENTRIES-1:0]] <= uq_uop;
	     r_uq[r_uq_next_tail_ptr[`LG_UQ_ENTRIES-1:0]] <= uq_uop_two;	     
	  end
	else if(t_push_one_int)
	  begin
	     r_uq[r_uq_tail_ptr[`LG_UQ_ENTRIES-1:0]] <= uq_uop.is_int ? uq_uop : uq_uop_two;
	  end
	
     end // always_ff@ (posedge clk)
   
   logic [31:0]        r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : r_cycle + 'd1;
     end
   


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
	n_wb_bitvec[`DIV32_LAT] = t_start_div32&r_start_int;
	
	if(t_start_mul&r_start_int)
	  begin
	     n_wb_bitvec[`MUL_LAT] = 1'b1;
	  end
     end // always_comb

   always_ff@(posedge clk)
     begin
	r_fp_wb_bitvec <= reset ? 'd0 : n_fp_wb_bitvec;
     end
   
   always_comb
     begin
	for(integer i = (`FP_MAX_LAT-1); i > -1; i = i-1)
	  begin
	     n_fp_wb_bitvec[i] = r_fp_wb_bitvec[i+1];
	  end
	n_fp_wb_bitvec[`FPU_LAT-1] = t_start_fpu;
     end
   
   always_comb
     begin
	t_srcA = r_fwd_int_srcA ? r_int_result :
		 r_fwd_mem_srcA ? r_mem_result :
		 r_srcA;
	
	t_srcB = r_fwd_int_srcB ? r_int_result :
		 r_fwd_mem_srcB ? r_mem_result :
		 r_srcB;

	t_src_hilo = r_fwd_hilo_int ? r_int_hilo :
		     r_fwd_hilo_mul ? r_mul_hilo :
		     r_fwd_hilo_div ? r_div_hilo :
		     r_src_hilo;

	
	t_mem_srcA = r_int_prf[mem_uq.srcA];
	t_mem_srcB = r_int_prf[mem_uq.srcB];
	
	t_mem_fp_srcB = r_fp_prf[mem_uq.srcB];
`ifdef ENABLE_FPU
	t_fp_srcA = r_fp_prf[fp_uq.srcA];
	t_fp_srcB = r_fp_prf[fp_uq.srcB];
	t_fp_srcC = r_fp_prf[fp_uq.srcC];
`endif
     end // always_comb



   localparam LG_INT_SCHED_ENTRIES = 2;
   localparam N_INT_SCHED_ENTRIES = 1<<LG_INT_SCHED_ENTRIES;

   //does this scheduler entry contain a valid uop?
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_sched_valid;
   logic [LG_INT_SCHED_ENTRIES:0] t_alu_sched_alloc_ptr;
   logic 			  t_alu_sched_full;
   
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_alloc_entry, t_alu_select_entry;

   uop_t r_alu_sched_uops[N_INT_SCHED_ENTRIES-1:0];
   uop_t t_picked_uop;

   
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_entry_rdy;
   logic [LG_INT_SCHED_ENTRIES:0]  t_alu_sched_select_ptr;
   
	
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_srcA_rdy, 
				   r_alu_srcB_rdy, 
				   r_alu_hilo_rdy,
				   r_alu_fcr_rdy;

   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_srcA_match, 
				   t_alu_srcB_match, 
				   t_alu_hilo_match,
				   t_alu_fcr_match;

   logic t_alu_alloc_srcA_match, 
	 t_alu_alloc_srcB_match, 
	 t_alu_alloc_hilo_match,
	 t_alu_alloc_fcr_match;

   
   find_first_set#(LG_INT_SCHED_ENTRIES) ffs_int_sched_alloc( .in(~r_alu_sched_valid),
							      .y(t_alu_sched_alloc_ptr));

   //find_first_set#(LG_INT_SCHED_ENTRIES) ffs_int_sched_select( .in(t_alu_entry_rdy),
//							       .y(t_alu_sched_select_ptr//));

   fair_sched#(LG_INT_SCHED_ENTRIES) ffs_int_sched_select( .clk(clk),
							   .rst(reset),
							   .in(t_alu_entry_rdy),
							   .y(t_alu_sched_select_ptr));

   
   always_comb
     begin
	t_alu_alloc_entry = 'd0;
	t_alu_select_entry = 'd0;
	if(t_pop_uq)
	  begin
	     t_alu_alloc_entry[t_alu_sched_alloc_ptr[LG_INT_SCHED_ENTRIES-1:0]] = 1'b1;
	  end
	if(t_alu_entry_rdy != 'd0)
	  begin
	     t_alu_select_entry[t_alu_sched_select_ptr[LG_INT_SCHED_ENTRIES-1:0]] = 1'b1;
	  end
     end // always_comb




   always_comb
     begin
	t_picked_uop = r_alu_sched_uops[t_alu_sched_select_ptr[LG_INT_SCHED_ENTRIES-1:0]];
     end
   
   always_ff@(posedge clk)
     begin
	int_uop <= t_picked_uop;
     end

   always_ff@(posedge clk)
     begin
	r_start_int <= reset ? 1'b0 : ((t_alu_entry_rdy != 'd0) & !ds_done);
     end // always_comb

   
   
   always_comb
     begin
	//allocation forwarding
	t_alu_alloc_srcA_match = uq.srcA_valid && (
						   (mem_rsp_dst_valid & (mem_rsp_dst_ptr == uq.srcA)) ||
						   (r_start_int && t_wr_int_prf & (int_uop.dst == uq.srcA))
						   );
	t_alu_alloc_srcB_match = uq.srcB_valid && (
						   (mem_rsp_dst_valid & (mem_rsp_dst_ptr == uq.srcB)) ||
						   (r_start_int && t_wr_int_prf & (int_uop.dst == uq.srcB))
						   );

	t_alu_alloc_hilo_match = uq.hilo_src_valid && (
						       (t_hilo_prf_ptr_val_out & (t_hilo_prf_ptr_out == uq.hilo_src)) ||
						       (t_div_complete && (t_div_hilo_prf_ptr_out == uq.hilo_src)) ||
						       (r_start_int && t_wr_hilo && (int_uop.hilo_dst == uq.hilo_src))
						       );
	
	t_alu_alloc_fcr_match = uq.fcr_src_valid && (
						     (t_fpu_fcr_valid & (t_fpu_fcr_ptr == uq.hilo_src))
						     );		
     end // always_comb
  
   
   generate
      for(genvar i = 0; i < N_INT_SCHED_ENTRIES; i=i+1)
	begin
	   always_comb
	     begin
		t_alu_srcA_match[i] = r_alu_sched_uops[i].srcA_valid && (
									 (mem_rsp_dst_valid & (mem_rsp_dst_ptr == r_alu_sched_uops[i].srcA)) ||
									 (r_start_int && t_wr_int_prf & (int_uop.dst == r_alu_sched_uops[i].srcA))
									 );
		t_alu_srcB_match[i] = r_alu_sched_uops[i].srcB_valid && (
									 (mem_rsp_dst_valid & (mem_rsp_dst_ptr == r_alu_sched_uops[i].srcB)) ||
									 (r_start_int && t_wr_int_prf & (int_uop.dst == r_alu_sched_uops[i].srcB))
									 );
		
		t_alu_hilo_match[i] = r_alu_sched_uops[i].hilo_src_valid && (
									     (t_hilo_prf_ptr_val_out & (t_hilo_prf_ptr_out == r_alu_sched_uops[i].hilo_src)) ||
									     (t_div_complete && (t_div_hilo_prf_ptr_out == r_alu_sched_uops[i].hilo_src)) ||
									     (r_start_int && t_wr_hilo && (int_uop.hilo_dst == r_alu_sched_uops[i].hilo_src))
									     );
		
		t_alu_fcr_match[i] = r_alu_sched_uops[i].fcr_src_valid && (
									   (t_fpu_fcr_valid & (t_fpu_fcr_ptr == r_alu_sched_uops[i].hilo_src))
									   );

		//is_mult(r_alu_sched_uops[i].op);
		
		t_alu_entry_rdy[i] = r_alu_sched_valid[i] &&
`ifdef SINGLE_CYCLE_INT_DIVIDE
				     ( (is_mult(r_alu_sched_uops[i].op) ?  !r_wb_bitvec[`MUL_LAT+1] : !r_wb_bitvec[1]) )
`else
				     (is_div(r_alu_sched_uops[i].op) ?  t_div_ready :  (is_mult(r_alu_sched_uops[i].op) ?  !r_wb_bitvec[`MUL_LAT+1] : !r_wb_bitvec[1]))
`endif
				     ? (
					(t_alu_srcA_match[i] |r_alu_srcA_rdy[i]) & 
					(t_alu_srcB_match[i] |r_alu_srcB_rdy[i]) &
					(t_alu_hilo_match[i] |r_alu_hilo_rdy[i]) & 
					(t_alu_fcr_match[i] | r_alu_fcr_rdy[i])) : 1'b0;
	     end // always_comb
	   
	   always_ff@(posedge clk)
	     begin
		if(reset)
		  begin
		     r_alu_srcA_rdy[i] <= 1'b0;
		     r_alu_srcB_rdy[i] <= 1'b0;
		     r_alu_hilo_rdy[i] <= 1'b0;
		     r_alu_fcr_rdy[i] <= 1'b0;
		  end
		else
		  begin
		     if(t_alu_alloc_entry[i])
		       begin //allocating to this entry
			  r_alu_srcA_rdy[i] <= uq.srcA_valid ? (!r_prf_inflight[uq.srcA] | t_alu_alloc_srcA_match) : 1'b1;
			  r_alu_srcB_rdy[i] <= uq.srcB_valid ? (!r_prf_inflight[uq.srcB] | t_alu_alloc_srcB_match) : 1'b1;
			  r_alu_hilo_rdy[i] <= uq.hilo_src_valid ? (!r_hilo_inflight[uq.hilo_src] | t_alu_alloc_hilo_match) : 1'b1;
			  r_alu_fcr_rdy[i] <= uq.fcr_src_valid ? (!r_fcr_prf_inflight[uq.hilo_src] | t_alu_alloc_fcr_match): 1'b1;
		       end
		     else if(t_alu_select_entry[i])
		       begin
			  r_alu_srcA_rdy[i] <= 1'b0;
			  r_alu_srcB_rdy[i] <= 1'b0;
			  r_alu_hilo_rdy[i] <= 1'b0;
			  r_alu_fcr_rdy[i] <= 1'b0;
		       end
		     else if(r_alu_sched_valid[i])
		       begin
			  r_alu_srcA_rdy[i] <= r_alu_srcA_rdy[i] | t_alu_srcA_match[i];
			  r_alu_srcB_rdy[i] <= r_alu_srcB_rdy[i] | t_alu_srcB_match[i];
			  r_alu_hilo_rdy[i] <= r_alu_hilo_rdy[i] | t_alu_hilo_match[i];
			  r_alu_fcr_rdy[i] <= r_alu_fcr_rdy[i] | t_alu_fcr_match[i];
		       end // else: !if(t_pop_uq&&(t_alu_sched_alloc_ptr == i))
		     
		  end // else: !if(reset)
	     end // always_ff@ (posedge clk)
	end // for (genvar i = 0; i < LG_INT_SCHED_ENTRIES; i=i+1)
   endgenerate
   
   
   
   always_comb
     begin
	t_pop_uq = 1'b0;
	t_alu_sched_full = (&r_alu_sched_valid);
	
	//t_pop_uq = t_flash_clear ? 1'b0 :
	//t_uq_empty ? 1'b0 : 
	//!t_srcs_rdy ? 1'b0 : 
	//(r_wb_bitvec[0]) ? 1'b0 :
	//	   t_start_mul & r_wb_bitvec[`MUL_LAT] ? 1'b0 : 
	//(t_start_div32 & (!t_div_ready || r_wb_bitvec[`DIV32_LAT])) ? 1'b0 :
	//1'b1;

	t_pop_uq = !(t_flash_clear || t_uq_empty ||t_alu_sched_full);
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
		  r_alu_sched_valid[t_alu_sched_alloc_ptr[LG_INT_SCHED_ENTRIES-1:0]] <= 1'b1;
		  r_alu_sched_uops[t_alu_sched_alloc_ptr[LG_INT_SCHED_ENTRIES-1:0]] <= uq;
	       end
	     if(t_alu_entry_rdy != 'd0)
	       begin
		  r_alu_sched_valid[t_alu_sched_select_ptr[LG_INT_SCHED_ENTRIES-1:0]] <= 1'b0;
	       end
	  end // else: !if(reset)
     end
   
   
   shift_right #(.LG_W(5)) s0(.is_signed(t_signed_shift), .data(t_srcA[31:0]), 
			      .distance(t_shift_amt), .y(t_shift_right));
   
   mul m(.clk(clk), 
	 .reset(reset), 
	 .opcode(int_uop.op), 
	 .go(t_start_mul&r_start_int),
	 .src_A(t_srcA[31:0]),
	 .src_B(t_srcB[31:0]),
	 .rob_ptr_in(int_uop.rob_ptr),
	 .hilo_prf_ptr_in(int_uop.hilo_dst),
	 .y(t_mul_result),
	 .complete(t_mul_complete),
	 .rob_ptr_out(t_rob_ptr_out),
	 .hilo_prf_ptr_val_out(t_hilo_prf_ptr_val_out),
	 .hilo_prf_ptr_out(t_hilo_prf_ptr_out)
	 );

   divider #(.LG_W(5)) d32 (
	   .clk(clk), 
	   .reset(reset),
	   .srcA(t_srcA[31:0]),
	   .srcB(t_srcB[31:0]),
	   .rob_ptr_in(int_uop.rob_ptr),
	   .hilo_prf_ptr_in(int_uop.hilo_dst),
	   .is_signed_div(t_signed_div),
	   .start_div(t_start_div32),
	   .y(t_div_result),
	   .rob_ptr_out(t_div_rob_ptr_out),
	   .hilo_prf_ptr_out(t_div_hilo_prf_ptr_out),
	   .complete(t_div_complete),
	   .ready(t_div_ready)
	   );

   assign divide_ready = t_div_ready;

   
   always_comb
     begin
	n_mq_head_ptr = r_mq_head_ptr;
	n_mq_tail_ptr = r_mq_tail_ptr;
	if(t_push_mq)
	  begin
	     n_mq_tail_ptr = r_mq_tail_ptr + 'd1;
	  end
	if(mem_req_ack)
	  begin
	     n_mq_head_ptr = r_mq_head_ptr + 'd1;
	  end
	
	t_mem_head = r_mem_q[r_mq_head_ptr[`LG_MQ_ENTRIES-1:0]];
	
	mem_q_empty = (r_mq_head_ptr == r_mq_tail_ptr);
	
	mem_q_full = (r_mq_head_ptr != r_mq_tail_ptr) &&
		     (r_mq_head_ptr[`LG_MQ_ENTRIES-1:0] == r_mq_tail_ptr[`LG_MQ_ENTRIES-1:0]);
	
     end // always_comb

   always_ff@(posedge clk)
     begin
	if(t_push_mq)
	  begin
	     r_mem_q[r_mq_tail_ptr[`LG_MQ_ENTRIES-1:0]] = t_mem_tail;
	  end
     end


   assign mem_req = t_mem_head;
   assign mem_req_valid = !mem_q_empty;
   assign uq_wait = r_uq_wait;
   assign mq_wait = r_mq_wait;
   assign fq_wait = r_fq_wait;   
   
   
   always_ff@(posedge clk)
     begin
				       r_mq_head_ptr <= reset ? 'd0 : n_mq_head_ptr;
				       r_mq_tail_ptr <= reset ? 'd0 : n_mq_tail_ptr;
     end

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_prf_inflight <= 'd0;
	     r_fp_prf_inflight <= 'd0;
	     r_hilo_inflight <= 'd0;
	     r_fcr_prf_inflight <= 'd0;
	  end
	else
	  begin
	     r_prf_inflight <= ds_done ? 'd0 : n_prf_inflight;
	     r_fp_prf_inflight <= ds_done ? 'd0 : n_fp_prf_inflight;
	     r_hilo_inflight <= ds_done ? 'd0 : n_hilo_inflight;
	     r_fcr_prf_inflight <= ds_done ? 'd0 : n_fcr_prf_inflight;
	  end
     end // always_ff@ (posedge clk)

   
   always_comb
     begin
	n_prf_inflight = r_prf_inflight;

	
	if(uq_push && uq_uop.dst_valid)
	  begin
	     n_prf_inflight[uq_uop.dst] = 1'b1;
	  end
	if(uq_push_two && uq_uop_two.dst_valid)
	  begin
	     n_prf_inflight[uq_uop_two.dst] = 1'b1;
	  end
	
	
	if(mem_rsp_dst_valid)
	  begin
	     n_prf_inflight[mem_rsp_dst_ptr] = 1'b0;
	  end
	if(r_start_int && t_wr_int_prf)
	  begin
	     n_prf_inflight[int_uop.dst] = 1'b0;
	  end
     end // always_comb
   
   always_comb
     begin
	n_hilo_inflight = r_hilo_inflight;
	if(uq_push && uq_uop.hilo_dst_valid)
	  begin
	     n_hilo_inflight[uq_uop.hilo_dst] = 1'b1;
	  end
	
	if(uq_push_two && uq_uop_two.hilo_dst_valid)
	  begin
	     n_hilo_inflight[uq_uop_two.hilo_dst] = 1'b1;
	  end
	
	if(t_hilo_prf_ptr_val_out)
	  begin
	     n_hilo_inflight[t_hilo_prf_ptr_out] = 1'b0;
	  end
	if(t_div_complete)
	  begin
	     n_hilo_inflight[t_div_hilo_prf_ptr_out] = 1'b0;
	  end
	if(r_start_int && t_wr_hilo)
	  begin
	     n_hilo_inflight[int_uop.hilo_dst] = 1'b0;
	  end

     end // always_comb

   
   always_comb
     begin

	n_fcr_prf_inflight = r_fcr_prf_inflight;

	if(uq_push && uq_uop.fcr_dst_valid)
	  begin
	     //$display("at cycle %d marking fcr %d inflight for rob ptr %d", 
	     //      r_cycle, uq_uop.hilo_dst, uq_uop.rob_ptr);
	     n_fcr_prf_inflight[uq_uop.hilo_dst] = 1'b1;
	  end
	if(uq_push_two && uq_uop_two.fcr_dst_valid)
	  begin
	     //$display("at cycle %d marking fcr %d inflight for rob ptr %d", 
	     //      r_cycle, uq_uop.hilo_dst, uq_uop.rob_ptr);
	     n_fcr_prf_inflight[uq_uop_two.hilo_dst] = 1'b1;
	  end
	if(t_fpu_fcr_valid)
	  begin
	     n_fcr_prf_inflight[t_fpu_fcr_ptr] = 1'b0;
	  end
     end

   always_comb
     begin
	n_fp_prf_inflight = r_fp_prf_inflight;
	
	if(uq_push && uq_uop.fp_dst_valid)
	  begin
	     n_fp_prf_inflight[uq_uop.dst] = 1'b1;
	  end

	if(uq_push_two && uq_uop_two.fp_dst_valid)
	  begin
	     n_fp_prf_inflight[uq_uop_two.dst] = 1'b1;
	  end
	
	if(mem_rsp_fp_dst_valid)
	  begin
	     n_fp_prf_inflight[mem_rsp_dst_ptr] = 1'b0;
	  end

	if(t_fpu_result_valid)
	  begin
	     n_fp_prf_inflight[t_fpu_dst_ptr] = 1'b0;
	  end
	else if(t_sp_div_valid)
	  begin
	     n_fp_prf_inflight[t_sp_div_dst_ptr] = 1'b0;
	  end
	else if(t_dp_div_valid)
	  begin
	     n_fp_prf_inflight[t_dp_div_dst_ptr] = 1'b0;
	  end	
	if(t_fp_wr_prf)
	  begin
	     n_fp_prf_inflight[fp_uq.dst] = 1'b0;
	  end
     end // always_comb
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i < N_FCR_PRF_ENTRIES; i=i+1)
	       begin
		  r_fcr_prf[i] <= 'd0;
	       end
	  end
	else if(t_fpu_fcr_valid)
	  begin
	     r_fcr_prf[t_fpu_fcr_ptr] <= t_fpu_result[7:0];
	  end
     end

   
`ifdef VERILATOR
   logic t_blocked_by_store;
   always_comb
     begin
	t_blocked_by_store = t_mem_uq_empty ? 1'b0 : !t_pop_mem_uq  & is_store(mem_uq.op) & 
			     !r_prf_inflight[mem_uq.srcA] & r_prf_inflight[mem_uq.srcB] &
			     !mem_q_full;
     end
   always_ff@(negedge clk)
     begin
	report_exec(uq_empty ? 32'd0 : 32'd1,
		    t_pop_uq ? 32'd1 : 32'd0,
		    t_mem_uq_empty ? 32'd0 : 32'd1,
		    t_pop_mem_uq ? 32'd1 : 32'd0,
		    t_fp_uq_empty ? 32'd0 : 32'd1,
		    t_pop_fp_uq ? 32'd1 : 32'd0,
		    t_uq_full ? 32'd1 : 32'd0,
		    t_mem_uq_full ? 32'd1 : 32'd0,
		    t_fp_uq_full ? 32'd1 : 32'd0,
		    t_blocked_by_store ? 32'd1 : 32'd0,
		    {28'd0, t_alu_entry_rdy}
		    );
     end
`endif //  `ifdef VERILATOR

   wire [31:0] w_add32;
   wire [31:0] w_s_sub32, w_c_sub32;
   
   csa #(.N(32)) csa0 (.a(t_srcA), 
		       .b(int_uop.op == SUBU ? ~t_srcB : ((int_uop.op == ADDIU ? {{E_BITS{int_uop.imm[15]}},int_uop.imm} : t_srcB))), 
		       .cin(int_uop.op == SUBU ? 32'd1 : 32'd0), .s(w_s_sub32), .cout(w_c_sub32) );

   wire [31:0] w_add_srcA = {w_c_sub32[30:0], 1'b0};
   wire [31:0] w_add_srcB = w_s_sub32;
      
   ppa32 add0 (.A(w_add_srcA), .B(w_add_srcB), .Y(w_add32));
   

   always_comb
     begin
	t_pc = int_uop.pc;
	t_pc4 = int_uop.pc + {{HI_EBITS{1'b0}}, 32'd4};
	t_pc8 = int_uop.pc + {{HI_EBITS{1'b0}}, 32'd8};
	t_result = {`M_WIDTH{1'b0}};
	t_cpr0_result = {`M_WIDTH{1'b0}};
	t_unimp_op = 1'b0;
	t_fault = 1'b0;
	t_simm = {{E_BITS{int_uop.imm[15]}},int_uop.imm};
	t_wr_int_prf = 1'b0;
	t_wr_cpr0 = 1'b0;
	t_take_br = 1'b0;
	t_mispred_br = 1'b0;
	t_jaddr = {int_uop.jmp_imm[9:0],int_uop.imm,2'd0};
	t_alu_valid = 1'b0;
	t_hilo_result = 'd0;
	t_wr_hilo = 1'b0;
	t_got_break = 1'b0;
	t_signed_shift = 1'b0;
	t_shift_amt = 5'd0;
	t_start_mul = 1'b0;
	t_signed_div = 1'b0;
	t_start_div32 = 1'b0;	

	
	case(int_uop.op)
	  BREAK:
	    begin
	       t_alu_valid = 1'b1;
	       t_got_break = 1'b1;
	       t_fault = 1'b1;
	       //t_unimp_op = 1'b1;
	    end
	  SLL:
	    begin
	       t_result = t_srcA << int_uop.srcB;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;

	    end
	  MOVT:
	    begin
	       if(r_fcr_prf[int_uop.hilo_src][int_uop.srcC[2:0]]==1'b1)
		 begin
		    t_result = t_srcA;
		 end
	       else
		 begin
		    t_result = t_srcB;
		 end
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  MOVF:
	    begin
	       if(r_fcr_prf[int_uop.hilo_src][int_uop.srcC[2:0]]==1'b1)
		 begin
		    t_result = t_srcB;
		 end
	       else
		 begin
		    t_result = t_srcA;
		 end
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRA:
	    begin
	       t_signed_shift = 1'b1;
	       t_shift_amt = int_uop.srcB[4:0];
	       //t_result = $signed(t_srcA) >> $signed(int_uop.srcB[4:0]);
	       //$display("t_result = %b, t_shift_right = %b", t_result, t_shift_right);
	       
	       t_result = {{HI_EBITS{t_shift_right[31]}}, t_shift_right};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end // case: SRA
	  SRAV:
	    begin
	       t_signed_shift = 1'b1;
	       t_shift_amt = t_srcB[4:0];
	       t_result = {{HI_EBITS{t_shift_right[31]}}, t_shift_right};	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRL:
	    begin
	       t_result = t_srcA >> int_uop.srcB;	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLLV:
	    begin
	       t_result = t_srcA << (t_srcB[4:0]);
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SRLV:
	    begin
	       t_result = t_srcA >> (t_srcB[4:0]);
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  MTLO:
	    begin
	       t_hilo_result = {t_src_hilo[63:32], t_srcA[31:0]};
	       t_wr_hilo = 1'b1;
	       t_alu_valid = 1'b1;	       
	    end
	  MTHI:
	    begin
	       t_hilo_result = {t_srcA[31:0], t_src_hilo[31:0] };
	       t_wr_hilo = 1'b1;
	       t_alu_valid = 1'b1;	       
	    end
	  MFLO:
	    begin
	       t_result = {{HI_EBITS{1'b0}}, t_src_hilo[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  MFHI:
	    begin
	       t_result = {{HI_EBITS{1'b0}},t_src_hilo[63:32]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  ADDU:
	    begin
	       t_result = w_add32;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  MULT:
	    begin
	       t_start_mul = r_start_int&!ds_done;
	    end
	  MULTU:
	    begin
	       t_start_mul = r_start_int&!ds_done;
	    end
`ifdef SINGLE_CYCLE_INT_DIVIDE
	  DIV:
	    begin
	       t_alu_valid = 1'b1;	       
	       t_hilo_result[31:0] = $signed(t_srcA[31:0]) / $signed(t_srcB[31:0]);
	       t_hilo_result[63:32] = $signed(t_srcA[31:0]) % $signed(t_srcB[31:0]);
	       t_wr_hilo = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  DIVU:
	    begin
	       t_alu_valid = 1'b1;	       
	       t_hilo_result[31:0] = t_srcA[31:0] / t_srcB[31:0];
	       t_hilo_result[63:32] = t_srcA[31:0] % t_srcB[31:0];
	       t_wr_hilo = 1'b1;
	       t_alu_valid = 1'b1;
	    end
`else // !`ifdef VERILATOR
	  DIV:
	    begin
	       t_signed_div = 1'b1;
	       t_start_div32 = r_start_int&!ds_done;	       
	    end
	  DIVU:
	    begin
	       t_start_div32 = r_start_int&!ds_done;
	    end
`endif
	  SUBU:
	    begin
	       t_result = w_add32;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  AND:
	    begin
	       t_result = t_srcA & t_srcB;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  MOV:
	    begin
	       t_result = t_srcA;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  OR:
	    begin
	       t_result = t_srcA | t_srcB;
	       t_wr_int_prf = 1'b1;//int_uop.dst_valid;
	       t_alu_valid = 1'b1;
	    end
	  XOR:
	    begin
	       t_result = t_srcA ^ t_srcB;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  NOR:
	    begin
	       t_result = ~(t_srcA | t_srcB);
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLT:
	    begin
	       t_result = (($signed(t_srcB) <  $signed(t_srcA)) ? 'd1 : 'd0);
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end // case: SLT
	  SLTU:
	    begin
	       t_result = (t_srcB <  t_srcA) ? 'd1 : 'd0;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end // case: SLTU
	  BEQ:
	    begin
	       t_take_br = (t_srcA  == t_srcB);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BEQL:
	    begin
	       t_take_br = (t_srcA  == t_srcB);
	       t_mispred_br = int_uop.br_pred != t_take_br || !t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BNE:
	    begin
	       t_take_br = (t_srcA  != t_srcB);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end // case: BNE
	  BGEZ:
	    begin
	       t_take_br = (t_srcA[31] == 1'b0);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;	       
	       t_alu_valid = 1'b1;
	    end
	  BGEZAL:
	    begin
	       t_take_br = (t_srcA[31] == 1'b0);
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;	  
     	       t_result = t_take_br ?  int_uop.pc + {{HI_EBITS{1'b0}}, 32'd8} : t_srcB;
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	    end // case: BGEZAL
	  BAL:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_result = int_uop.pc + {{HI_EBITS{1'b0}}, 32'd8};
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	    end
	  BLTZ:
	    begin
	       t_take_br = ($signed(t_srcA) < $signed({`M_WIDTH{1'b0}}));
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BLEZ:
	    begin
	       t_take_br = ($signed(t_srcA) <= $signed({`M_WIDTH{1'b0}}));
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BLEZL:
	    begin
	       t_take_br = ($signed(t_srcA) < $signed({`M_WIDTH{1'b0}})) || (t_srcA == {`M_WIDTH{1'b0}});
	       t_mispred_br = int_uop.br_pred != t_take_br || !t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BGTZ:
	    begin
	       t_take_br = ($signed(t_srcA) > $signed({`M_WIDTH{1'b0}}));
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;	       
	       t_alu_valid = 1'b1;
	    end
	  BNEL:
	    begin
	       t_take_br = (t_srcA  != t_srcB);
	       t_mispred_br = (int_uop.br_pred != t_take_br) /* || !t_take_br */;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BLTZL:
	    begin
	       t_take_br = $signed(t_srcA) < $signed({`M_WIDTH{1'b0}});
	       t_mispred_br = (int_uop.br_pred != t_take_br) || !t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BGTZL:
	    begin
	       t_take_br = ($signed(t_srcA) > $signed({`M_WIDTH{1'b0}}));
	       t_mispred_br = (int_uop.br_pred != t_take_br) || !t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BGEZL:
	    begin
	       t_take_br = ($signed(t_srcA) >= $signed({`M_WIDTH{1'b0}}));
	       t_mispred_br = (int_uop.br_pred != t_take_br) || !t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  // J:
	  //   begin
	  //      t_take_br = 1'b1;
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = {t_pc4[`M_WIDTH-1:28],t_jaddr};
	  //      t_alu_valid = 1'b1;
	  //      t_srcs_rdy = 1'b1;	       
	  //   end
	  JAL:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = {t_pc4[`M_WIDTH-1:28],t_jaddr};
	       t_result = int_uop.pc + {{HI_EBITS{1'b0}}, 32'd8};
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	    end
	  JR:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = (t_srcA != {int_uop.jmp_imm,int_uop.imm});
	       t_pc = t_srcA;
	       t_alu_valid = 1'b1;
	    end
	  JALR:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = (t_srcA != {int_uop.jmp_imm,int_uop.imm});
	       t_pc = t_srcA;
	       t_alu_valid = 1'b1;
	       t_result = int_uop.pc + {{HI_EBITS{1'b0}},32'd8};
	       t_wr_int_prf = 1'b1;
	    end
	  MONITOR:
	    begin
	       t_take_br = 1'b1;
	       t_mispred_br = 1'b1;
	       t_pc = t_srcA;
	       t_alu_valid = 1'b1;
	       t_result = monitor_rsp_data;
	       t_wr_int_prf = 1'b1;
	    end
	  ANDI:
	    begin
	       t_result = t_srcA & {{E_BITS{1'b0}},int_uop.imm};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  ORI:
	    begin
	       t_result = t_srcA | {{E_BITS{1'b0}},int_uop.imm};	       
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  XORI:
	    begin
	       t_result = t_srcA ^ {{E_BITS{1'b0}},int_uop.imm};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  LUI:
	    begin
	       t_result = {{HI_EBITS{int_uop.imm[15]}},int_uop.imm, 16'd0};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  ADDIU:
	    begin
	       t_result = w_add32;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  MOVI:
	    begin
	       t_result = {{HI_EBITS{t_simm[31]}}, t_simm[31:0]};
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLTI:
	    begin
	       t_result = (($signed(t_srcA) < $signed(t_simm)) ? 'd1 : 'd0);
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  SLTIU:
	    begin
	       t_result = (t_srcA < t_simm ? 'd1 : 'd0);
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  MFC0:
	    begin	       
	       t_result = int_uop.srcA[4:0] == 'd12 ? cpr0_status_reg : 'd0;
	       t_alu_valid = 1'b1;
	       t_wr_int_prf = 1'b1;
	       t_pc = t_pc4;	       
	    end
	  MTC0:
	    begin
	       t_wr_cpr0 = 1'b1;	       
	       t_alu_valid = 1'b1;
	       t_pc = t_pc4;
	    end // case: MTC0
	  II:
	    begin
	       t_unimp_op = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	   BC1TL:
	     begin
		t_take_br = r_fcr_prf[int_uop.hilo_src][int_uop.srcC[2:0]];
		t_mispred_br = int_uop.br_pred != t_take_br;
		t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
		t_alu_valid = 1'b1;
	     end
	  BC1F:
	    begin
	       t_take_br = r_fcr_prf[int_uop.hilo_src][int_uop.srcC[2:0]]==1'b0;
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;	       
	    end
	  BC1FL:
	    begin
	       t_take_br = r_fcr_prf[int_uop.hilo_src][int_uop.srcC[2:0]]==1'b0;
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  BC1T:
	    begin
	       t_take_br = r_fcr_prf[int_uop.hilo_src][int_uop.srcC[2:0]];
	       t_mispred_br = int_uop.br_pred != t_take_br;
	       t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	       t_alu_valid = 1'b1;
	    end
	  default:
	    begin
	       t_unimp_op = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	endcase // case (int_uop.op)

	
     end // always_comb


`ifdef ENABLE_FPU
   logic [31:0] t_sp_trunc, t_w_sp_cvt;
   logic [63:0] t_dp_trunc, t_w_dp_cvt;

   logic 	t_fp_div_sp_active, t_fp_div_dp_active,t_fp_div_active;

   logic 	t_start_dp_div, t_start_sp_div, t_is_fp_sqrt;
   
   fp_trunc_to_int32 #(.W(32)) sp_trunc (.clk(clk),
				.in(t_fp_srcA[31:0]),
				.en((fp_uq.op == TRUNC_SP_W) && t_fp_srcs_rdy),
				.out(t_sp_trunc));
   
   fp_trunc_to_int32 #(.W(64)) dp_trunc (.clk(clk),
				.in(t_fp_srcA),
				.en((fp_uq.op == TRUNC_DP_W) && t_fp_srcs_rdy),
				.out(t_dp_trunc));
   
   fp_convert #(.W(32)) int_sp_convert (.clk(clk),
					.in(t_fp_srcA[31:0]),
					.en((fp_uq.op == CVT_W_SP) && t_fp_srcs_rdy),					.out(t_w_sp_cvt));
   
   fp_convert #(.W(64)) int_dp_convert (.clk(clk),
					.in({{32{t_fp_srcA[31]}},t_fp_srcA[31:0]}),
					.en((fp_uq.op == CVT_W_DP) && t_fp_srcs_rdy),					.out(t_w_dp_cvt));

   always_comb
     begin
	t_fp_div_active = t_fp_div_sp_active|t_fp_div_dp_active|t_sp_div_valid|t_dp_div_valid;
     end

   
   fp_div #(.LG_PRF_WIDTH(`LG_PRF_ENTRIES), 
	    .LG_ROB_WIDTH(`LG_ROB_ENTRIES),
	    .W(32)) sp_fpu_div(
			       .y(t_sp_div_result),
			       .valid(t_sp_div_valid),
			       .dst_ptr_out(t_sp_div_dst_ptr),
			       .rob_ptr_out(t_sp_div_rob_ptr),
			       .active(t_fp_div_sp_active),
			       .is_sqrt(t_is_fp_sqrt),
			       .clk(clk), 
			       .reset(reset), 
			       .a(t_fp_srcA[31:0]), 
			       .b(t_fp_srcB[31:0]),
			       .start(t_start_sp_div),
			       .rob_ptr_in(fp_uq.rob_ptr),
			       .dst_ptr_in(fp_uq.dst)
			       );
   

   fp_div #(.LG_PRF_WIDTH(`LG_PRF_ENTRIES), 
	    .LG_ROB_WIDTH(`LG_ROB_ENTRIES),
	    .W(64)) dp_fpu_div(.y(t_dp_div_result),
			       .valid(t_dp_div_valid),
			       .dst_ptr_out(t_dp_div_dst_ptr),
			       .rob_ptr_out(t_dp_div_rob_ptr),
			       .active(t_fp_div_dp_active),
			       .is_sqrt(t_is_fp_sqrt),
			       .clk(clk), 
			       .reset(reset), 
			       .a(t_fp_srcA), 
			       .b(t_fp_srcB),
			       .start(t_start_dp_div),
			       .rob_ptr_in(fp_uq.rob_ptr),
			       .dst_ptr_in(fp_uq.dst)
			       );


   logic [31:0] t_cvt_dp_sp;
   logic [10:0] t_cvt_dp_sp_exp;
   always_comb
     begin
 `ifdef DEBUG_FPU
	t_cvt_dp_sp = fp64_to_fp32(t_fp_srcA);
 `else
	t_cvt_dp_sp_exp = t_fp_srcA[62:52] - 11'd896;
	t_cvt_dp_sp = {t_fp_srcA[63], t_cvt_dp_sp_exp[7:0], t_fp_srcA[51:29]};
 `endif
     end
   
   fpu #(.LG_PRF_WIDTH(`LG_PRF_ENTRIES), 
	 .LG_ROB_WIDTH(`LG_ROB_ENTRIES),
	 .LG_FCR_WIDTH(`LG_FCR_PRF_ENTRIES),
	 .FPU_LAT(`FPU_LAT)) 
   fpu0(
	.y(t_fpu_result),
	.val(t_fpu_result_valid),
	.cmp_val(t_fpu_fcr_valid),
	.dst_ptr_out(t_fpu_dst_ptr),
	.rob_ptr_out(t_fpu_rob_ptr),
	.fcr_ptr_out(t_fpu_fcr_ptr),
	.clk(clk),
	.reset(reset),
	.pc(fp_uq.pc),
	.opcode(fp_uq.op),
	.src_a(t_fp_srcA),
	.src_b(t_fp_srcB),
	.src_c(t_fp_srcC),
	.src_fcr(r_fcr_prf[fp_uq.hilo_src]),
	.start(t_start_fpu && t_pop_fp_uq),
	.rob_ptr_in(fp_uq.rob_ptr),
	.dst_ptr_in(fp_uq.dst),
	.fcr_ptr_in(fp_uq.hilo_dst),
	.fcr_sel(fp_uq.imm[2:0])
	);


   logic t_fpu_srcA_rdy;
   logic t_fpu_srcB_rdy;
   logic t_fpu_srcC_rdy;
   logic t_fpu_fcr_rdy;
   logic t_fpu_srcs_rdy;
   
   always_comb
     begin
	t_fpu_srcA_rdy = fp_uq.fp_srcA_valid ? !r_fp_prf_inflight[fp_uq.srcA] : 1'b1;
	t_fpu_srcB_rdy = fp_uq.fp_srcB_valid ? !r_fp_prf_inflight[fp_uq.srcB] : 1'b1;
	t_fpu_srcC_rdy = fp_uq.fp_srcC_valid ? !r_fp_prf_inflight[fp_uq.srcC] : 1'b1;
	t_fpu_fcr_rdy = fp_uq.fcr_src_valid ? !r_fcr_prf_inflight[fp_uq.hilo_src] : 1'b1;
	t_fpu_srcs_rdy = t_fpu_srcA_rdy && t_fpu_srcB_rdy && t_fpu_srcC_rdy && t_fpu_fcr_rdy;
     end // always_comb

   
   always_comb
     begin
	t_start_fpu = 1'b0;
	t_start_dp_div = 1'b0;
	t_start_sp_div = 1'b0;
	t_is_fp_sqrt = 1'b0;
	t_fp_wr_prf = 1'b0;
	t_pop_fp_uq = 1'b0;
	t_fp_srcs_rdy = 1'b0;
	t_fp_result = 64'd0;
	case(fp_uq.op)
	  SP_MOV:
	    begin
	       t_fp_result = {32'd0, t_fp_srcA[31:0]};
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       //$display("cycle %d, fp move sources ready %b, wb bitvec %b", 
	       //r_cycle, t_fp_srcs_rdy, r_fp_wb_bitvec);
	       t_fp_wr_prf = t_fp_srcs_rdy; 
	    end // case: FP_MOV	    
	  DP_MOV:
	    begin
	       t_fp_result = t_fp_srcA;
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       //$display("cycle %d, fp move sources ready %b, wb bitvec %b", 
	       //r_cycle, t_fp_srcs_rdy, r_fp_wb_bitvec);
	       t_fp_wr_prf = t_fp_srcs_rdy; 
	    end // case: FP_MOV
	  FP_MOVT:
	    begin
	       if(r_fcr_prf[fp_uq.hilo_src][fp_uq.srcC[2:0]]==1'b1)
		 begin
		    t_fp_result = t_fp_srcA;
		 end
	       else
		 begin
		    t_fp_result = t_fp_srcB;
		 end
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy; 
	    end // case: FP_MOVZ
	  FP_MOVF:
	    begin
	       if(r_fcr_prf[fp_uq.hilo_src][fp_uq.srcC[2:0]]==1'b0)
		 begin
		    t_fp_result = t_fp_srcA;
		 end
	       else
		 begin
		    t_fp_result = t_fp_srcB;
		 end
	       
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy; 
	    end
	  
	  TRUNC_DP_W:
	    begin
	       t_fp_result = t_dp_trunc;
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy; 	       
	    end

	  TRUNC_SP_W:
	    begin
	       t_fp_result = {32'd0, t_sp_trunc};
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy; 
	    end
	  
	  CVT_SP_DP:
	    begin
 `ifdef DEBUG_FPU
	       t_fp_result = fp32_to_fp64(t_fp_srcA[31:0]);
 `else
	       t_fp_result = {t_fp_srcA[31], (11'd896 + {3'd0, t_fp_srcA[30:23]}),
			      t_fp_srcA[22:0], 29'd0};
 `endif
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy;
	    end
	  CVT_DP_SP:
	    begin
	       t_fp_result = {32'd0, t_cvt_dp_sp};
	       t_fp_srcs_rdy = t_fpu_srcs_rdy 
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy;
	    end
	  CVT_W_DP:
	    begin
	       t_fp_result = t_w_dp_cvt;
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy; 	       
	    end
	  CVT_W_SP:
	    begin
	       t_fp_result = {32'd0, t_w_sp_cvt};
	       t_fp_srcs_rdy = t_fpu_srcs_rdy
			       && !t_fp_uq_empty
			       && !t_fp_div_active
			       && !r_fp_wb_bitvec[0];
	       t_fp_wr_prf = t_fp_srcs_rdy; 	       
	    end
	  SP_ADD:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy
			     && !t_fp_div_active
			     && !t_fp_uq_empty;	       
	    end
	  SP_SUB:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy
			     && !t_fp_div_active
			     && !t_fp_uq_empty;	       	       
	    end
	  SP_MUL:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy
			     && !t_fp_div_active
			     && !t_fp_uq_empty;	       
	    end
	  DP_ADD:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy
			     && !t_fp_div_active
			     && !t_fp_uq_empty;	       
	    end
	  DP_SUB:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy
			     && !t_fp_div_active
			     && !t_fp_uq_empty;	       
	    end
	  
	  DP_MUL:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy
			     && !t_fp_div_active
			     && !t_fp_uq_empty;	       
	    end

	  DP_DIV:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy && (r_fp_wb_bitvec == 'd0);
	       t_start_dp_div = t_fp_srcs_rdy
				&& !t_fp_div_active
				&& !t_fp_uq_empty;
	       
	    end
	  SP_DIV:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy && (r_fp_wb_bitvec == 'd0);;
	       t_start_sp_div = t_fp_srcs_rdy
				&& !t_fp_div_active
				&& !t_fp_uq_empty;
	    end

	  DP_SQRT:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy && (r_fp_wb_bitvec == 'd0);
	       t_start_dp_div = t_fp_srcs_rdy
				&& !t_fp_div_active
				&& !t_fp_uq_empty;
	       t_is_fp_sqrt = 1'b1;
	    end
	  SP_SQRT:
	    begin
	       t_fp_srcs_rdy = t_fpu_srcs_rdy && (r_fp_wb_bitvec == 'd0);
	       t_start_sp_div = t_fp_srcs_rdy
				&& !t_fp_div_active
				&& !t_fp_uq_empty;
	       t_is_fp_sqrt = 1'b1;				   
	    end
	  
	  DP_CMP_LT:
	    begin
	       t_fp_srcs_rdy =!t_fp_uq_empty && t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy && !t_fp_div_active;
	    end
	  
	  SP_CMP_LT:
	    begin
	       t_fp_srcs_rdy =!t_fp_uq_empty && t_fpu_srcs_rdy;	       
	       t_start_fpu = t_fp_srcs_rdy && !t_fp_div_active;
	    end

	  DP_CMP_LE:
	    begin
	       t_fp_srcs_rdy =!t_fp_uq_empty && t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy && !t_fp_div_active;
	    end
	  
	  SP_CMP_LE:
	    begin
	       t_fp_srcs_rdy =!t_fp_uq_empty && t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy && !t_fp_div_active;
	    end

	  DP_CMP_EQ:
	    begin
	       t_fp_srcs_rdy =!t_fp_uq_empty && t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy && !t_fp_div_active;
	    end
	  
	  SP_CMP_EQ:
	    begin
	       t_fp_srcs_rdy =!t_fp_uq_empty && t_fpu_srcs_rdy;
	       t_start_fpu = t_fp_srcs_rdy && !t_fp_div_active;
	    end

	  
	  default:
	    begin
	    end
	endcase // case (fp_uq.op)
	t_pop_fp_uq = t_fp_uq_empty || t_flash_clear ? 1'b0 : t_fp_srcs_rdy && (!t_fp_div_active);
     end
`else
   always_comb
     begin
	t_start_fpu = 1'b0;
	t_fp_wr_prf = 1'b0;
	t_pop_fp_uq = 1'b0;
	t_fp_srcs_rdy = 1'b0;
	t_fp_result = 64'd0;
     end
`endif // !`ifdef ENABLE_FPU


   wire [31:0] w_agu32;
   ppa32 agu (.A(t_mem_srcA), .B({{E_BITS{mem_uq.imm[15]}},mem_uq.imm}), .Y(w_agu32));

   logic       t_mem_ready, t_mem_srcA_ready, t_mem_srcB_ready;
   
   always_comb
     begin
	t_mem_srcA_ready = mem_uq.srcA_valid ? !r_prf_inflight[mem_uq.srcA] : 1'b1;
	t_mem_srcB_ready = mem_uq.srcB_valid ? !r_prf_inflight[mem_uq.srcB] : 
			   mem_uq.fp_srcB_valid ? !r_fp_prf_inflight[mem_uq.srcB] : 
			   1'b1;
	t_mem_ready = (!t_mem_uq_empty) && (!mem_q_full) && t_mem_srcA_ready && t_mem_srcB_ready;
     end
   
   always_comb
     begin
	t_pop_mem_uq = 1'b0;
	t_mem_simm = {{E_BITS{mem_uq.imm[15]}},mem_uq.imm};
	t_push_mq = t_mem_ready;
	t_mem_tail.op = MEM_LW;
	t_mem_tail.addr = w_agu32;
	t_mem_tail.data = 'd0;
	t_mem_tail.rob_ptr = mem_uq.rob_ptr;
	t_mem_tail.dst_valid = 1'b0;
	t_mem_tail.fp_dst_valid = 1'b0;
	t_mem_tail.dst_ptr = mem_uq.dst;
	t_mem_tail.is_store = 1'b0;
	t_mem_tail.lwc1_lo = 1'b0;
	t_mem_tail.in_storebuf = 1'b0;
	t_mem_tail.is_fp = 1'b0;
	t_mem_tail.pc = mem_uq.pc;
	t_mem_tail.uuid = r_cycle;
	
	case(mem_uq.op)
	  SB:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SB;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.data = {{Z_BITS{1'b0}}, t_mem_srcB}; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b0;
		 end
	    end // case: SB
	  SH:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SH;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.data = {{Z_BITS{1'b0}},t_mem_srcB}; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b0;
		 end
	    end // case: SW
	  SW:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SW;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.data = {{Z_BITS{1'b0}},t_mem_srcB}; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b0;
		 end
	    end // case: SW
	  SDC1:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SDC1;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.data = t_mem_fp_srcB; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b0;
		    t_mem_tail.is_fp = 1'b1;
		 end
	    end // case: SDC1
	  SWC1_MERGE:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SWC1_MERGE;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.lwc1_lo = mem_uq.jmp_imm[0];		    
		    t_mem_tail.data = t_mem_fp_srcB; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b0;
		    t_mem_tail.is_fp = 1'b1;		    
		 end
	    end
	  SC:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SC;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.data = {{Z_BITS{1'b0}},t_mem_srcB}; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;		    
		 end
	    end // case: SW
	  SWR:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SWR;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.data = {{Z_BITS{1'b0}},t_mem_srcB}; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b0;
		 end
	    end // case: SW
	  SWL:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_SWL;
		    t_mem_tail.is_store = 1'b1;
		    t_mem_tail.data = {{Z_BITS{1'b0}}, t_mem_srcB}; /* needs byte swap */
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b0;
		 end
	    end // case: SW	  
	  LW:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LW;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		 end
	    end // case: LW
	  LDC1:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LDC1;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.fp_dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		    t_mem_tail.is_fp = 1'b1;		    
		 end
	    end // case: LDC1
	  LWC1_MERGE:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LWC1_MERGE;
		    t_mem_tail.lwc1_lo = mem_uq.jmp_imm[0];
		    t_mem_tail.data = t_mem_fp_srcB;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.fp_dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		    t_mem_tail.is_fp = 1'b1;
		 end
	    end
	  LWL:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LWL;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		    t_mem_tail.data = {{Z_BITS{1'b0}}, t_mem_srcB};
		 end
	    end // case: LWL
	  LWR:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LWR;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		    t_mem_tail.data =  {{Z_BITS{1'b0}}, t_mem_srcB};
		 end
	    end // case: LWR
	  LB:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LB;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		 end
	    end
	  LBU:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LBU;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		 end
	    end // case: LBU
	  LHU:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LHU;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		 end
	    end // case: LBU
	  LH:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_LH;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		 end
	    end // case: LH
	  MFC1_MERGE:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_MFC1_MERGE;
		    t_mem_tail.lwc1_lo = mem_uq.jmp_imm[0];		    
		    t_mem_tail.data = t_mem_fp_srcB;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		    t_mem_tail.is_fp = 1'b1;
		 end
	    end
	  MTC1_MERGE:
	    begin
	       if(t_mem_ready)
		 begin
		    t_mem_tail.op = MEM_MTC1_MERGE;
		    t_mem_tail.lwc1_lo = mem_uq.jmp_imm[0];
		    t_mem_tail.data = t_mem_fp_srcB;
		    t_mem_tail.rob_ptr = mem_uq.rob_ptr;
		    t_mem_tail.fp_dst_valid = 1'b1;
		    t_mem_tail.dst_ptr = mem_uq.dst;
		    t_mem_tail.is_fp = 1'b1;
		 end
	    end
	  default:
	    begin
	    end
	endcase // case (mem_uq.op)
	t_pop_mem_uq = (t_mem_uq_empty || t_flash_clear)  ? 1'b0 : t_mem_ready;
     end // always_comb

   
   initial
     begin
	r_int_prf[0] = 'd0;
     end

   always_ff@(posedge clk)
     begin
	r_int_result <= t_result;
	r_mem_result <= mem_rsp_load_data[31:0];
	r_int_hilo <= t_hilo_result;
	r_mul_hilo <= t_mul_result;
	r_div_hilo <= t_div_result;
     end

   always_ff@(posedge clk)
     begin
	r_fwd_int_srcA <= r_start_int && t_wr_int_prf && (t_picked_uop.srcA == int_uop.dst);
	r_fwd_int_srcB <= r_start_int && t_wr_int_prf && (t_picked_uop.srcB == int_uop.dst);	
	r_fwd_mem_srcA <= mem_rsp_dst_valid && (t_picked_uop.srcA == mem_rsp_dst_ptr);
	r_fwd_mem_srcB <= mem_rsp_dst_valid && (t_picked_uop.srcB == mem_rsp_dst_ptr);

	r_fwd_hilo_int <= r_start_int && t_wr_hilo && (t_picked_uop.hilo_src == int_uop.hilo_dst);
	r_fwd_hilo_mul <= t_hilo_prf_ptr_val_out && (t_picked_uop.hilo_src == t_hilo_prf_ptr_out);
	r_fwd_hilo_div <= t_div_complete && (t_picked_uop.hilo_src == t_div_hilo_prf_ptr_out);
     end
   
   always_ff@(posedge clk)
     begin
	r_srcA <= r_int_prf[t_picked_uop.srcA];
	r_srcB <= r_int_prf[t_picked_uop.srcB];
	
	if(r_start_int && t_wr_int_prf)
	  begin
	     r_int_prf[int_uop.dst] <=  t_result;
	  end
	//2nd write port
	if(mem_rsp_dst_valid)
	  begin
	     r_int_prf[mem_rsp_dst_ptr] <= mem_rsp_load_data[31:0];
	  end
     end // always_ff@ (posedge clk)

  
   always_ff@(posedge clk)
     begin
	r_src_hilo <= r_hilo_prf[t_picked_uop.hilo_src];
	
	if(r_start_int && t_wr_hilo)
	  begin
	     r_hilo_prf[int_uop.hilo_dst] <= t_hilo_result;
	  end
	else if(t_hilo_prf_ptr_val_out)
	  begin
	     r_hilo_prf[t_hilo_prf_ptr_out] <= t_mul_result;
	  end
	else if(t_div_complete)
	  begin
	     r_hilo_prf[t_div_hilo_prf_ptr_out] <= t_div_result;
	  end	     
     end // always_ff@ (posedge clk)


   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     cpr0_status_reg <= 'd4194308; 
	  end
	else
	  begin
	     if(r_start_int && t_wr_cpr0)
	       begin
		  cpr0_status_reg <= t_srcA;
	       end
	  end
     end
   


   always_ff@(posedge clk)
     begin
	if(t_fp_wr_prf)
	  begin
	     r_fp_prf[fp_uq.dst] <= t_fp_result;
	  end
	else if(t_fpu_result_valid)
	  begin
	     r_fp_prf[t_fpu_dst_ptr] <= t_fpu_result;
	  end
	else if(t_sp_div_valid)
	  begin
	     r_fp_prf[t_sp_div_dst_ptr] <= {32'd0, t_sp_div_result};
	  end
	else if(t_dp_div_valid)
	  begin
	     r_fp_prf[t_dp_div_dst_ptr] <= t_dp_div_result;
	  end
	if(mem_rsp_fp_dst_valid)
	  begin
	     r_fp_prf[mem_rsp_dst_ptr] <= mem_rsp_load_data;
	  end
     end
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     complete_valid_1 <= 1'b0;
	  end
	else
	  begin
	     complete_valid_1 <= r_start_int && t_alu_valid || t_mul_complete || t_div_complete;
	  end
     end // always_ff@ (posedge clk)

   
   always_ff@(posedge clk)
     begin
	if(t_mul_complete || t_div_complete)
	  begin
	     complete_bundle_1.rob_ptr <= t_mul_complete ? t_rob_ptr_out : t_div_rob_ptr_out;
	     complete_bundle_1.complete <= 1'b1;
	     complete_bundle_1.faulted <= 1'b0;
	     complete_bundle_1.restart_pc <= 'd0;
	     complete_bundle_1.is_ii <= 1'b0;
	     complete_bundle_1.take_br <= 1'b0;
	     complete_bundle_1.data <= {32'd0, t_mul_result[`M_WIDTH-1:0]};
	  end
	else
	  begin
	     complete_bundle_1.rob_ptr <= int_uop.rob_ptr;
	     complete_bundle_1.complete <= t_alu_valid;
	     complete_bundle_1.faulted <= t_mispred_br || t_unimp_op || t_fault;
	     complete_bundle_1.restart_pc <= t_pc;
	     complete_bundle_1.is_ii <= t_unimp_op;
	     complete_bundle_1.take_br <= t_take_br;
	     complete_bundle_1.data <=  {32'd0, t_result};
	  end
	//(uq.rob_ptr == 'd5) ? 1'b1 : 1'b0;
     end


  always_ff@(posedge clk)
    begin
       if(reset)
	 begin
	    complete_valid_2 <= 1'b0;
	 end
       else
	 begin
	    complete_valid_2 <= t_fp_wr_prf || t_fpu_result_valid || t_fpu_fcr_valid || t_sp_div_valid || t_dp_div_valid;
	 end
    end // always_ff@ (posedge clk)


   //t_fpu_fcr_valid
   always_ff@(posedge clk)
     begin
	if(t_fpu_result_valid || t_fpu_fcr_valid)
	  begin
	     complete_bundle_2.rob_ptr <= t_fpu_rob_ptr;
	     complete_bundle_2.complete <= 1'b1;
	     complete_bundle_2.faulted <= 1'b0;	
	     complete_bundle_2.restart_pc <= 'd0;
	     complete_bundle_2.is_ii <= 1'b0;
	     complete_bundle_2.take_br <= 1'b0;
	     complete_bundle_2.data <= t_fpu_result;
	  end // if (t_fpu_result_valid )
	else if(t_sp_div_valid)
	  begin
	     complete_bundle_2.rob_ptr <= t_sp_div_rob_ptr;
	     complete_bundle_2.complete <= 1'b1;
	     complete_bundle_2.faulted <= 1'b0;	
	     complete_bundle_2.restart_pc <= 'd0;
	     complete_bundle_2.is_ii <= 1'b0;
	     complete_bundle_2.take_br <= 1'b0;
	     complete_bundle_2.data <= {32'd0, t_sp_div_result};	     
	  end
	else if(t_dp_div_valid)
	  begin
	     complete_bundle_2.rob_ptr <= t_dp_div_rob_ptr;
	     complete_bundle_2.complete <= 1'b1;
	     complete_bundle_2.faulted <= 1'b0;	
	     complete_bundle_2.restart_pc <= 'd0;
	     complete_bundle_2.is_ii <= 1'b0;
	     complete_bundle_2.take_br <= 1'b0;
	     complete_bundle_2.data <= t_dp_div_result;
	  end
	else
	  begin
	     complete_bundle_2.rob_ptr <= fp_uq.rob_ptr;
	     complete_bundle_2.complete <= 1'b1;
	     complete_bundle_2.faulted <= 1'b0;	
	     complete_bundle_2.restart_pc <= 'd0;
	     complete_bundle_2.is_ii <= 1'b0;
	     complete_bundle_2.take_br <= 1'b0;
	     complete_bundle_2.data <= t_fp_result;
	  end
     end
endmodule
