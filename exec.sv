`include "uop.vh"
`include "rob.vh"

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
	    mem_dq_clr,
	    restart_complete,
	    cpr0_status_reg,
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
	    exception_wr_cpr0_val,
	    exception_wr_cpr0_ptr,
	    exception_wr_cpr0_data,
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
	    mem_rsp_rob_ptr,
	    mem_rsp_load_data,
	    monitor_rsp_data);
   input logic clk;
   input logic reset;
`ifdef VERILATOR
   input logic [31:0] clear_cnt;
`endif
   output logic       divide_ready;
   input logic ds_done;
   input logic mem_dq_clr;
   input logic restart_complete;
   output logic [(`M_WIDTH-1):0]     cpr0_status_reg;
   
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


   input logic 	exception_wr_cpr0_val;
   input logic [4:0] exception_wr_cpr0_ptr;
   input logic [`M_WIDTH-1:0] exception_wr_cpr0_data;
   
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
   input logic [31:0] 		     mem_rsp_load_data;
   input logic [`LG_ROB_ENTRIES-1:0] mem_rsp_rob_ptr;
   
   input logic [`M_WIDTH-1:0] monitor_rsp_data;
   
   localparam N_INT_SCHED_ENTRIES = 1<<`LG_INT_SCHED_ENTRIES;
   
   localparam N_MQ_ENTRIES = (1<<`LG_MQ_ENTRIES);
   localparam N_INT_PRF_ENTRIES = (1<<`LG_PRF_ENTRIES);
   localparam N_HILO_PRF_ENTRIES = (1<<`LG_HILO_PRF_ENTRIES);
   
   localparam N_UQ_ENTRIES = (1<<`LG_UQ_ENTRIES);
   localparam N_MEM_UQ_ENTRIES = (1<<`LG_MEM_UQ_ENTRIES);
   localparam N_MEM_DQ_ENTRIES = (1<<`LG_MEM_DQ_ENTRIES);
   
   logic [63:0] r_hilo_prf[N_HILO_PRF_ENTRIES-1:0];
      
   logic [N_INT_PRF_ENTRIES-1:0]  r_prf_inflight, n_prf_inflight;
   logic [N_HILO_PRF_ENTRIES-1:0] r_hilo_inflight, n_hilo_inflight;
   
   logic 			  t_wr_int_prf, t_wr_cpr0;
   
   logic 	t_wr_hilo;
   logic 	t_take_br;
   logic 	t_mispred_br;
   logic 	t_alu_valid;
   logic 	t_got_break;

      
   
   mem_req_t r_mem_q[N_MQ_ENTRIES-1:0];
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
   
   
   

   logic 	t_pop_uq,t_pop_mem_uq,t_pop_mem_dq;
   logic 	r_mem_ready, r_dq_ready;
   
   
   localparam E_BITS = `M_WIDTH-16;
   localparam HI_EBITS = `M_WIDTH-32;
   
   logic [`M_WIDTH-1:0] t_simm, t_mem_simm;
   logic [`M_WIDTH-1:0] t_result;
   logic [`M_WIDTH-1:0] t_cpr0_result;

   
   logic [63:0] t_hilo_result;
   
   logic [`M_WIDTH-1:0] t_pc, t_pc4, t_pc8;
   logic [27:0] t_jaddr;
   logic 	t_srcs_rdy;

   
   wire [31:0] w_srcA, w_srcB;
   wire [31:0] w_mem_srcA, w_mem_srcB;
   
   logic [31:0] r_mem_result, r_int_result;
   logic 	r_fwd_int_srcA, r_fwd_int_srcB;
   logic 	r_fwd_mem_srcA, r_fwd_mem_srcB;

   logic t_fwd_int_mem_srcA,t_fwd_int_mem_srcB,t_fwd_mem_mem_srcA,t_fwd_mem_mem_srcB;
   logic r_fwd_int_mem_srcA,r_fwd_int_mem_srcB,r_fwd_mem_mem_srcA,r_fwd_mem_mem_srcB;
   
   logic [63:0] r_int_hilo, r_mul_hilo, r_div_hilo;
   logic [63:0] r_src_hilo;
   logic 	r_fwd_hilo_int, r_fwd_hilo_mul, r_fwd_hilo_div;
      
   logic [31:0] t_srcA, t_srcB;
   logic [31:0] t_mem_srcA, t_mem_srcB;
   
   
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

   /* divider */
   logic 	t_div_ready, t_signed_div, t_start_div32;
   logic [`LG_ROB_ENTRIES-1:0] t_div_rob_ptr_out;
   logic [63:0] 	       t_div_result;
   logic [`LG_HILO_PRF_ENTRIES-1:0] t_div_hilo_prf_ptr_out;
   logic 			    t_div_complete;

   logic [N_ROB_ENTRIES-1:0] 	    r_uq_wait, r_mq_wait;
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
   uop_t t_mem_uq, mem_uq;
   logic 	      t_mem_uq_read, t_mem_uq_empty, t_mem_uq_full,
		      t_mem_uq_next_full;
   
   logic [`LG_MEM_UQ_ENTRIES:0]  r_mem_uq_head_ptr, n_mem_uq_head_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0]  r_mem_uq_tail_ptr, n_mem_uq_tail_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0] r_mem_uq_next_head_ptr, n_mem_uq_next_head_ptr;
   logic [`LG_MEM_UQ_ENTRIES:0] r_mem_uq_next_tail_ptr, n_mem_uq_next_tail_ptr;

   /* mem data queue */
   //uop_t r_mem_uq[N_MEM_UQ_ENTRIES];
  // uop_t t_mem_uq, mem_uq;
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

   
   always_comb
     begin
	t_srcA = r_fwd_int_srcA ? r_int_result :
		 r_fwd_mem_srcA ? r_mem_result :
		 w_srcA;
	
	t_srcB = r_fwd_int_srcB ? r_int_result :
		 r_fwd_mem_srcB ? r_mem_result :
		 w_srcB;

	t_mem_srcA = r_fwd_int_mem_srcA ? r_int_result :
		     r_fwd_mem_mem_srcA ? r_mem_result :
		     w_mem_srcA;

	t_mem_srcB = r_fwd_int_mem_srcB ? r_int_result :
		     r_fwd_mem_mem_srcB ? r_mem_result :
		     w_mem_srcB;
	
	t_src_hilo = r_fwd_hilo_int ? r_int_hilo :
		     r_fwd_hilo_mul ? r_mul_hilo :
		     r_fwd_hilo_div ? r_div_hilo :
		     r_src_hilo;
     end // always_comb




   //does this scheduler entry contain a valid uop?
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_sched_valid;
   logic [`LG_INT_SCHED_ENTRIES:0] t_alu_sched_alloc_ptr;
   logic 			  t_alu_sched_full;
   
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_alloc_entry, t_alu_select_entry;

   uop_t r_alu_sched_uops[N_INT_SCHED_ENTRIES-1:0];
   uop_t t_picked_uop;

   
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_entry_rdy;
   logic [`LG_INT_SCHED_ENTRIES:0]  t_alu_sched_select_ptr;
   
	
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_srcA_rdy, 
				   r_alu_srcB_rdy, 
				   r_alu_hilo_rdy;
   
   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_srcA_match, 
				   t_alu_srcB_match, 
				   t_alu_hilo_match;
   


   logic t_alu_alloc_srcA_match, 
	 t_alu_alloc_srcB_match, 
	 t_alu_alloc_hilo_match;
   
   wire [N_INT_SCHED_ENTRIES-1:0] w_alu_sched_oldest_ready;
   
   find_first_set#(`LG_INT_SCHED_ENTRIES) ffs_int_sched_alloc( .in(~r_alu_sched_valid),
							      .y(t_alu_sched_alloc_ptr));

   find_first_set#(`LG_INT_SCHED_ENTRIES) ffs_int_sched_select( .in(w_alu_sched_oldest_ready),
								.y(t_alu_sched_select_ptr));

   
   
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
	t_picked_uop = r_alu_sched_uops[t_alu_sched_select_ptr[`LG_INT_SCHED_ENTRIES-1:0]];
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
	
     end // always_comb
  

   logic [N_INT_SCHED_ENTRIES-1:0] t_alu_sched_mask_valid;
   logic [N_INT_SCHED_ENTRIES-1:0] r_alu_sched_matrix [N_INT_SCHED_ENTRIES-1:0];

   
   always_comb
     begin
	t_alu_sched_mask_valid = r_alu_sched_valid & (~t_alu_select_entry);
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

   //always_ff@(negedge clk)
   //begin
   //if(t_alu_entry_rdy != 'd0)
   //	  $display("w_alu_sched_oldest = %b, w_alu_sched_oldest_ready = %b, t_alu_entry_rdy = %b", 
   //w_alu_sched_oldest, w_alu_sched_oldest_ready, t_alu_entry_rdy);
   //end
   
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
		

		//is_mult(r_alu_sched_uops[i].op);
		
		t_alu_entry_rdy[i] = r_alu_sched_valid[i] &&
				     (is_div(r_alu_sched_uops[i].op) ?  t_div_ready :  (is_mult(r_alu_sched_uops[i].op) ?  !r_wb_bitvec[`MUL_LAT+2] : !r_wb_bitvec[1]))
				     ? (
					(t_alu_srcA_match[i] |r_alu_srcA_rdy[i]) & 
					(t_alu_srcB_match[i] |r_alu_srcB_rdy[i]) &
					(t_alu_hilo_match[i] |r_alu_hilo_rdy[i]) ) : 1'b0;
	     end // always_comb
	   
	   always_ff@(posedge clk)
	     begin
		if(reset)
		  begin
		     r_alu_srcA_rdy[i] <= 1'b0;
		     r_alu_srcB_rdy[i] <= 1'b0;
		     r_alu_hilo_rdy[i] <= 1'b0;
		  end
		else
		  begin
		     if(t_alu_alloc_entry[i])
		       begin //allocating to this entry
			  r_alu_srcA_rdy[i] <= uq.srcA_valid ? (!r_prf_inflight[uq.srcA] | t_alu_alloc_srcA_match) : 1'b1;
			  r_alu_srcB_rdy[i] <= uq.srcB_valid ? (!r_prf_inflight[uq.srcB] | t_alu_alloc_srcB_match) : 1'b1;
			  r_alu_hilo_rdy[i] <= uq.hilo_src_valid ? (!r_hilo_inflight[uq.hilo_src] | t_alu_alloc_hilo_match) : 1'b1;
		       end
		     else if(t_alu_select_entry[i])
		       begin
			  r_alu_srcA_rdy[i] <= 1'b0;
			  r_alu_srcB_rdy[i] <= 1'b0;
			  r_alu_hilo_rdy[i] <= 1'b0;
		       end
		     else if(r_alu_sched_valid[i])
		       begin
			  r_alu_srcA_rdy[i] <= r_alu_srcA_rdy[i] | t_alu_srcA_match[i];
			  r_alu_srcB_rdy[i] <= r_alu_srcB_rdy[i] | t_alu_srcB_match[i];
			  r_alu_hilo_rdy[i] <= r_alu_hilo_rdy[i] | t_alu_hilo_match[i];
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
		  r_alu_sched_valid[t_alu_sched_alloc_ptr[`LG_INT_SCHED_ENTRIES-1:0]] <= 1'b1;
		  r_alu_sched_uops[t_alu_sched_alloc_ptr[`LG_INT_SCHED_ENTRIES-1:0]] <= uq;
	       end
	     if(t_alu_entry_rdy != 'd0)
	       begin
		  r_alu_sched_valid[t_alu_sched_select_ptr[`LG_INT_SCHED_ENTRIES-1:0]] <= 1'b0;
	       end
	  end // else: !if(reset)
     end
   
   
   shift_right #(.LG_W(5)) s0(.is_signed(t_signed_shift), .data(t_srcA[31:0]), 
			      .distance(t_shift_amt), .y(t_shift_right));
   
   mul m(.clk(clk), 
	 .reset(reset), 
	 .is_signed(int_uop.op != MULTU), 
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
	if(reset)
	  begin
	     r_prf_inflight <= 'd0;
	     r_hilo_inflight <= 'd0;
	  end
	else
	  begin
	     r_prf_inflight <= ds_done ? 'd0 : n_prf_inflight;
	     r_hilo_inflight <= ds_done ? 'd0 : n_hilo_inflight;
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

   

   
`ifdef VERILATOR
   logic t_blocked_by_store;
   always_comb
     begin
	t_blocked_by_store = t_mem_uq_empty ? 1'b0 : !t_pop_mem_uq  & is_store(mem_uq.op) & 
			     !r_prf_inflight[mem_uq.srcA] &
			     !mem_q_full;
     end
   always_ff@(negedge clk)
     begin
	report_exec(t_uq_empty ? 32'd0 : 32'd1,
		    t_pop_uq ? 32'd1 : 32'd0,
		    t_mem_uq_empty ? 32'd0 : 32'd1,
		    t_pop_mem_uq ? 32'd1 : 32'd0,
		    32'd1,
		    32'd0,
		    t_uq_full ? 32'd1 : 32'd0,
		    t_mem_uq_full ? 32'd1 : 32'd0,
		    32'd0,
		    t_blocked_by_store ? 32'd1 : 32'd0,
		    {{(32-N_INT_SCHED_ENTRIES){1'b0}}, t_alu_entry_rdy}
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

   always_ff@(negedge clk)
     begin
	if(r_start_int)
	  begin
	     $display("start int fired for opcode %d, pc %x", int_uop.op, int_uop.pc);
	  end
     end
   

   always_comb
     begin
	t_pc = int_uop.pc;
	t_pc4 = int_uop.pc + 32'd4;
	t_pc8 = int_uop.pc + 32'd8;
	t_result = 32'd0;
	t_cpr0_result = 32'd0;
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
	  //riscv
	  AUIPC:
	    begin
	       t_result = t_pc + int_uop.rvimm;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  ADDI:
	    begin
	       t_result = t_srcA + int_uop.rvimm;
	       t_wr_int_prf = 1'b1;
	       t_alu_valid = 1'b1;
	    end

	  // //old mips instructions
	  // BREAK:
	  //   begin
	  //      t_alu_valid = 1'b1;
	  //      t_got_break = 1'b1;
	  //      t_fault = 1'b1;
	  //      //t_unimp_op = 1'b1;
	  //   end
	  // SLL:
	  //   begin
	  //      t_result = t_srcA << int_uop.srcB;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;

	  //   end
	  // SRA:
	  //   begin
	  //      t_signed_shift = 1'b1;
	  //      t_shift_amt = int_uop.srcB[4:0];
	  //      //t_result = $signed(t_srcA) >> $signed(int_uop.srcB[4:0]);
	  //      //$display("t_result = %b, t_shift_right = %b", t_result, t_shift_right);
	       
	  //      t_result = {{HI_EBITS{t_shift_right[31]}}, t_shift_right};
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end // case: SRA
	  // SRAV:
	  //   begin
	  //      t_signed_shift = 1'b1;
	  //      t_shift_amt = t_srcB[4:0];
	  //      t_result = {{HI_EBITS{t_shift_right[31]}}, t_shift_right};	       
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // SRL:
	  //   begin
	  //      t_result = t_srcA >> int_uop.srcB;	       
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // SLLV:
	  //   begin
	  //      t_result = t_srcA << (t_srcB[4:0]);
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // SRLV:
	  //   begin
	  //      t_result = t_srcA >> (t_srcB[4:0]);
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // MTLO:
	  //   begin
	  //      t_hilo_result = {t_src_hilo[63:32], t_srcA[31:0]};
	  //      t_wr_hilo = 1'b1;
	  //      t_alu_valid = 1'b1;	       
	  //   end
	  // MTHI:
	  //   begin
	  //      t_hilo_result = {t_srcA[31:0], t_src_hilo[31:0] };
	  //      t_wr_hilo = 1'b1;
	  //      t_alu_valid = 1'b1;	       
	  //   end
	  // MFLO:
	  //   begin
	  //      t_result = {{HI_EBITS{1'b0}}, t_src_hilo[31:0]};
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // MFHI:
	  //   begin
	  //      t_result = {{HI_EBITS{1'b0}},t_src_hilo[63:32]};
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // ADDU:
	  //   begin
	  //      t_result = w_add32;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // MULT:
	  //   begin
	  //      t_start_mul = r_start_int&!ds_done;
	  //   end
	  // MULTU:
	  //   begin
	  //      t_start_mul = r_start_int&!ds_done;
	  //   end
	  // DIV:
	  //   begin
	  //      t_signed_div = 1'b1;
	  //      t_start_div32 = r_start_int&!ds_done;	       
	  //   end
	  // DIVU:
	  //   begin
	  //      t_start_div32 = r_start_int&!ds_done;
	  //   end
	  // SUBU:
	  //   begin
	  //      t_result = w_add32;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // AND:
	  //   begin
	  //      t_result = t_srcA & t_srcB;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // MOV:
	  //   begin
	  //      t_result = t_srcA;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // OR:
	  //   begin
	  //      t_result = t_srcA | t_srcB;
	  //      t_wr_int_prf = 1'b1;//int_uop.dst_valid;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // XOR:
	  //   begin
	  //      t_result = t_srcA ^ t_srcB;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // NOR:
	  //   begin
	  //      t_result = ~(t_srcA | t_srcB);
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // SLT:
	  //   begin
	  //      t_result = (($signed(t_srcB) <  $signed(t_srcA)) ? 'd1 : 'd0);
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end // case: SLT
	  // SLTU:
	  //   begin
	  //      t_result = (t_srcB <  t_srcA) ? 'd1 : 'd0;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end // case: SLTU
	  // BEQ:
	  //   begin
	  //      t_take_br = (t_srcA  == t_srcB);
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BEQL:
	  //   begin
	  //      t_take_br = (t_srcA  == t_srcB);
	  //      t_mispred_br = int_uop.br_pred != t_take_br || !t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BNE:
	  //   begin
	  //      t_take_br = (t_srcA  != t_srcB);
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end // case: BNE
	  // BGEZ:
	  //   begin
	  //      t_take_br = (t_srcA[31] == 1'b0);
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;	       
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BGEZAL:
	  //   begin
	  //      t_take_br = (t_srcA[31] == 1'b0);
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;	  
     	  //      t_result = t_take_br ?  int_uop.pc + {{HI_EBITS{1'b0}}, 32'd8} : t_srcB;
	  //      t_alu_valid = 1'b1;
	  //      t_wr_int_prf = 1'b1;
	  //   end // case: BGEZAL
	  // BAL:
	  //   begin
	  //      t_take_br = 1'b1;
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_result = int_uop.pc + {{HI_EBITS{1'b0}}, 32'd8};
	  //      t_alu_valid = 1'b1;
	  //      t_wr_int_prf = 1'b1;
	  //   end
	  // BLTZ:
	  //   begin
	  //      t_take_br = ($signed(t_srcA) < $signed({`M_WIDTH{1'b0}}));
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BLEZ:
	  //   begin
	  //      t_take_br = ($signed(t_srcA) <= $signed({`M_WIDTH{1'b0}}));
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BLEZL:
	  //   begin
	  //      t_take_br = ($signed(t_srcA) < $signed({`M_WIDTH{1'b0}})) || (t_srcA == {`M_WIDTH{1'b0}});
	  //      t_mispred_br = int_uop.br_pred != t_take_br || !t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BGTZ:
	  //   begin
	  //      t_take_br = ($signed(t_srcA) > $signed({`M_WIDTH{1'b0}}));
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;	       
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BNEL:
	  //   begin
	  //      t_take_br = (t_srcA  != t_srcB);
	  //      t_mispred_br = (int_uop.br_pred != t_take_br) /* || !t_take_br */;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BLTZL:
	  //   begin
	  //      t_take_br = $signed(t_srcA) < $signed({`M_WIDTH{1'b0}});
	  //      t_mispred_br = (int_uop.br_pred != t_take_br) || !t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BGTZL:
	  //   begin
	  //      t_take_br = ($signed(t_srcA) > $signed({`M_WIDTH{1'b0}}));
	  //      t_mispred_br = (int_uop.br_pred != t_take_br) || !t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // BGEZL:
	  //   begin
	  //      t_take_br = ($signed(t_srcA) >= $signed({`M_WIDTH{1'b0}}));
	  //      t_mispred_br = (int_uop.br_pred != t_take_br) || !t_take_br;
	  //      t_pc = t_take_br ? (t_pc4 + {t_simm[`M_WIDTH-3:0], 2'd0}) : t_pc8;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // // J:
	  // //   begin
	  // //      t_take_br = 1'b1;
	  // //      t_mispred_br = int_uop.br_pred != t_take_br;
	  // //      t_pc = {t_pc4[`M_WIDTH-1:28],t_jaddr};
	  // //      t_alu_valid = 1'b1;
	  // //      t_srcs_rdy = 1'b1;	       
	  // //   end
	  // JAL:
	  //   begin
	  //      t_take_br = 1'b1;
	  //      t_mispred_br = int_uop.br_pred != t_take_br;
	  //      t_pc = {t_pc4[`M_WIDTH-1:28],t_jaddr};
	  //      t_result = int_uop.pc + {{HI_EBITS{1'b0}}, 32'd8};
	  //      t_alu_valid = 1'b1;
	  //      t_wr_int_prf = 1'b1;
	  //   end
	  // JR:
	  //   begin
	  //      t_take_br = 1'b1;
	  //      t_mispred_br = (t_srcA != {int_uop.jmp_imm,int_uop.imm});
	  //      t_pc = t_srcA;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // JALR:
	  //   begin
	  //      t_take_br = 1'b1;
	  //      t_mispred_br = (t_srcA != {int_uop.jmp_imm,int_uop.imm});
	  //      t_pc = t_srcA;
	  //      t_alu_valid = 1'b1;
	  //      t_result = int_uop.pc + {{HI_EBITS{1'b0}},32'd8};
	  //      t_wr_int_prf = 1'b1;
	  //   end
	  // MONITOR:
	  //   begin
	  //      t_take_br = 1'b1;
	  //      t_mispred_br = 1'b1;
	  //      t_pc = t_srcA;
	  //      t_alu_valid = 1'b1;
	  //      t_result = monitor_rsp_data;
	  //      t_wr_int_prf = 1'b1;
	  //   end
	  // ANDI:
	  //   begin
	  //      t_result = t_srcA & {{E_BITS{1'b0}},int_uop.imm};
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // ORI:
	  //   begin
	  //      t_result = t_srcA | {{E_BITS{1'b0}},int_uop.imm};	       
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // XORI:
	  //   begin
	  //      t_result = t_srcA ^ {{E_BITS{1'b0}},int_uop.imm};
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // LUI:
	  //   begin
	  //      t_result = {{HI_EBITS{int_uop.imm[15]}},int_uop.imm, 16'd0};
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // ADDIU:
	  //   begin
	  //      t_result = w_add32;
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // MOVI:
	  //   begin
	  //      t_result = {{HI_EBITS{t_simm[31]}}, t_simm[31:0]};
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // SLTI:
	  //   begin
	  //      t_result = (($signed(t_srcA) < $signed(t_simm)) ? 'd1 : 'd0);
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // SLTIU:
	  //   begin
	  //      t_result = (t_srcA < t_simm ? 'd1 : 'd0);
	  //      t_wr_int_prf = 1'b1;
	  //      t_alu_valid = 1'b1;
	  //   end
	  // MFC0:
	  //   begin	       
	  //      t_result = int_uop.srcA[4:0] == 'd12 ? cpr0_status_reg : 'd0;
	  //      t_alu_valid = 1'b1;
	  //      t_wr_int_prf = 1'b1;
	  //      t_pc = t_pc4;	       
	  //   end
	  // MTC0:
	  //   begin
	  //      t_wr_cpr0 = 1'b1;	       
	  //      t_alu_valid = 1'b1;
	  //      t_pc = t_pc4;
	  //   end // case: MTC0
	  II:
	    begin
	       t_unimp_op = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	  default:
	    begin
	       t_unimp_op = 1'b1;
	       t_alu_valid = 1'b1;
	    end
	endcase // case (int_uop.op)

	
     end // always_comb




   wire [31:0] w_agu32;
   ppa32 agu (.A(t_mem_srcA), .B({{E_BITS{mem_uq.imm[15]}},mem_uq.imm}), .Y(w_agu32));

   wire w_mem_srcA_ready = t_mem_uq.srcA_valid ? (!r_prf_inflight[t_mem_uq.srcA] | t_fwd_int_mem_srcA | t_fwd_mem_mem_srcA) : 1'b1;


   wire w_dq_ready = !r_prf_inflight[t_mem_dq.src_ptr] | t_fwd_int_mem_srcB | t_fwd_mem_mem_srcB;
   	
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

   
   
   
   //always_ff@(negedge clk)
     //begin
   //if(r_dq_ready)
   //begin
   //$display("cycle %d : popping dq, rob ptr %d, src ptr %d, pc %x, tag %d", r_cycle, mem_dq.rob_ptr, mem_dq.src_ptr, mem_dq.pc, mem_dq.tag);
   //	  end
	//if(r_mem_ready)
	  //begin
	    // $display("cycle %d, popping aq , rob ptr %d, srcb ptr %d, srcB val %b", r_cycle, mem_uq.rob_ptr, mem_uq.srcB, mem_uq.srcB_valid);
	  //end
   //end

   //always_ff@(posedge clk)
   //begin
   //core_store_data <= t_core_store_data;
   //end
   
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

   
   //$stop();
   //end
   
   always_comb
     begin
	t_mem_simm = {{E_BITS{mem_uq.imm[15]}},mem_uq.imm};
	t_mem_tail.op = MEM_LW;
	t_mem_tail.addr = w_agu32;
	t_mem_tail.rob_ptr = mem_uq.rob_ptr;
	t_mem_tail.dst_valid = 1'b0;
	t_mem_tail.dst_ptr = mem_uq.dst;
	t_mem_tail.is_store = 1'b0;
	t_mem_tail.data = 32'd0;
	t_mem_tail.bad_addr = 1'b0;
	case(mem_uq.op)
	  SB:
	    begin
	       t_mem_tail.op = MEM_SB;
	       t_mem_tail.is_store = 1'b1;
	       t_mem_tail.dst_valid = 1'b0;
	    end // case: SB
	  SH:
	    begin
	       t_mem_tail.op = MEM_SH;
	       t_mem_tail.is_store = 1'b1;
	       t_mem_tail.dst_valid = 1'b0;
	       t_mem_tail.bad_addr = w_agu32[0];
	    end // case: SW
	  SW:
	    begin
	       t_mem_tail.op = MEM_SW;
	       t_mem_tail.is_store = 1'b1;
	       t_mem_tail.dst_valid = 1'b0;
	       t_mem_tail.bad_addr = (w_agu32[1:0] != 2'd0);
	    end // case: SW
	  SC:
	    begin
	       t_mem_tail.op = MEM_SC;
	       t_mem_tail.is_store = 1'b1;
	       t_mem_tail.dst_valid = 1'b1;
	       t_mem_tail.dst_ptr = mem_uq.dst;
	       t_mem_tail.bad_addr = (w_agu32[1:0] != 2'd0);		    
	    end // case: SW
	  SWR:
	    begin
	       t_mem_tail.op = MEM_SWR;
	       t_mem_tail.is_store = 1'b1;
	       t_mem_tail.dst_valid = 1'b0;
	    end // case: SW
	  SWL:
	    begin
	       t_mem_tail.op = MEM_SWL;
	       t_mem_tail.is_store = 1'b1;
	       t_mem_tail.dst_valid = 1'b0;
	    end // case: SW	  
	  LW:
	    begin
	       t_mem_tail.op = MEM_LW;
	       t_mem_tail.dst_valid = 1'b1;
	       t_mem_tail.bad_addr = (w_agu32[1:0] != 2'd0);
	    end // case: LW
	  LWL:
	    begin
	       t_mem_tail.op = MEM_LWL;
	       t_mem_tail.dst_valid = 1'b1;
	       t_mem_tail.dst_ptr = mem_uq.dst;
	    end // case: LWL
	  LWR:
	    begin
	       t_mem_tail.op = MEM_LWR;
	       t_mem_tail.rob_ptr = mem_uq.rob_ptr;
	       t_mem_tail.dst_valid = 1'b1;
	    end // case: LWR
	  LB:
	    begin
	       t_mem_tail.op = MEM_LB;
	       t_mem_tail.dst_valid = 1'b1;
	    end
	  LBU:
	    begin
	       t_mem_tail.op = MEM_LBU;
	       t_mem_tail.dst_valid = 1'b1;
	    end // case: LBU
	  LHU:
	    begin
	       t_mem_tail.op = MEM_LHU;
	       t_mem_tail.dst_valid = 1'b1;
	       t_mem_tail.bad_addr = w_agu32[0];
	    end // case: LBU
	  LH:
	    begin
	       t_mem_tail.op = MEM_LH;
	       t_mem_tail.dst_valid = 1'b1;
	       t_mem_tail.bad_addr = w_agu32[0];
	    end // case: LH
	  default:
	    begin
	    end
	endcase // case (mem_uq.op)

     end // always_comb

   

   always_ff@(posedge clk)
     begin
	r_int_result <= t_result;
	r_mem_result <= mem_rsp_load_data[31:0];
	r_int_hilo <= t_hilo_result;
	r_mul_hilo <= t_mul_result;
	r_div_hilo <= t_div_result;
     end

   always_comb
     begin
	t_fwd_int_mem_srcA = r_start_int && t_wr_int_prf &&(t_mem_uq.srcA == int_uop.dst);
	t_fwd_int_mem_srcB = r_start_int && t_wr_int_prf &&(t_mem_dq.src_ptr == int_uop.dst);
	t_fwd_mem_mem_srcA = mem_rsp_dst_valid && (t_mem_uq.srcA == mem_rsp_dst_ptr);
	t_fwd_mem_mem_srcB = mem_rsp_dst_valid && (t_mem_dq.src_ptr == mem_rsp_dst_ptr);
     end
   
   always_ff@(posedge clk)
     begin
	r_fwd_int_mem_srcA <= t_fwd_int_mem_srcA;
	r_fwd_int_mem_srcB <= t_fwd_int_mem_srcB;
	r_fwd_mem_mem_srcA <= t_fwd_mem_mem_srcA;
	r_fwd_mem_mem_srcB <= t_fwd_mem_mem_srcB;
	
	r_fwd_int_srcA <= r_start_int && t_wr_int_prf && (t_picked_uop.srcA == int_uop.dst);
	r_fwd_int_srcB <= r_start_int && t_wr_int_prf && (t_picked_uop.srcB == int_uop.dst);
	
	r_fwd_mem_srcA <= mem_rsp_dst_valid && (t_picked_uop.srcA == mem_rsp_dst_ptr);
	r_fwd_mem_srcB <= mem_rsp_dst_valid && (t_picked_uop.srcB == mem_rsp_dst_ptr);

	r_fwd_hilo_int <= r_start_int && t_wr_hilo && (t_picked_uop.hilo_src == int_uop.hilo_dst);
	r_fwd_hilo_mul <= t_hilo_prf_ptr_val_out && (t_picked_uop.hilo_src == t_hilo_prf_ptr_out);
	r_fwd_hilo_div <= t_div_complete && (t_picked_uop.hilo_src == t_div_hilo_prf_ptr_out);
     end


   rf4r2w #(.WIDTH(32), .LG_DEPTH(`LG_PRF_ENTRIES)) 
   intprf (.clk(clk),
	   .rdptr0(t_picked_uop.srcA),
	   .rdptr1(t_picked_uop.srcB),
	   .rdptr2(t_mem_uq.srcA),
	   .rdptr3(t_mem_dq.src_ptr),
	   .wrptr0(int_uop.dst),
	   .wrptr1(mem_rsp_dst_ptr),
	   .wen0(r_start_int && t_wr_int_prf),
	   .wen1(mem_rsp_dst_valid),
	   .wr0(t_result),
	   .wr1(mem_rsp_load_data[31:0]),
	   .rd0(w_srcA),
	   .rd1(w_srcB),
	   .rd2(w_mem_srcA),
	   .rd3(w_mem_srcB)
	   );
   
   

  
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
	     complete_bundle_1.data <= t_mul_result[`M_WIDTH-1:0];
	  end
	else
	  begin
	     complete_bundle_1.rob_ptr <= int_uop.rob_ptr;
	     complete_bundle_1.complete <= t_alu_valid;
	     complete_bundle_1.faulted <= t_mispred_br || t_unimp_op || t_fault;
	     complete_bundle_1.restart_pc <= t_pc;
	     complete_bundle_1.is_ii <= t_unimp_op;
	     complete_bundle_1.take_br <= t_take_br;
	     complete_bundle_1.data <= t_result;
	  end
	//(uq.rob_ptr == 'd5) ? 1'b1 : 1'b0;
     end


endmodule
