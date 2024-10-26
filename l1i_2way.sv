`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

`ifdef VERILATOR
import "DPI-C" function void record_fetch(int push1, int push2, int push3, int push4,
					  longint pc0, longint pc1, longint pc2, longint pc3,
					  int bubble, int fq_full);
`endif


module l1i_2way(clk,
	   reset,
	   l1i_state,
	   priv,
	   page_table_root,	   	   
	   paging_active,
	   clear_tlb,
	   mode64,
	   page_walk_req_va,
	   page_walk_req_valid,
	   page_walk_rsp_valid,
	   page_walk_rsp,
	   flush_req,
	   flush_complete,
	   restart_pc,
	   restart_src_pc,
	   restart_src_is_indirect,
	   restart_valid,
	   restart_ack,
	   retire_valid,
	   retired_call,
	   retired_ret,
	   retire_reg_ptr,
	   retire_reg_data,
	   retire_reg_valid,
	   branch_pc_valid,
	   branch_pc_is_indirect,
	   branch_pc,
	   target_pc,
	   took_branch,
	   branch_fault,
	   branch_pht_idx,
	   
	   insn,
	   insn_valid,
	   insn_ack,
	   
	   insn_two,
	   insn_valid_two,
	   insn_ack_two,
	   
	   //output to the memory system
	   mem_req_valid, 
	   mem_req_addr, 
	   mem_req_opcode,
	   //reply from memory system
	   mem_rsp_valid,
	   mem_rsp_load_data,
	   cache_accesses,
	   cache_hits,
	   tlb_accesses,
	   tlb_hits
	   );

   input logic clk;
   input logic reset;
   output logic [3:0] l1i_state;
   input logic paging_active;
   input logic	       clear_tlb;
   input logic [1:0]   priv;
   input logic [63:0]  page_table_root;      
   input logic mode64;
   
   output      logic [63:0] page_walk_req_va;
   output logic		    page_walk_req_valid;
   
   input logic		    page_walk_rsp_valid;
   input 		    page_walk_rsp_t 	    page_walk_rsp;

   
   input logic 	      flush_req;
   output logic       flush_complete;
   //restart signals
   input logic [`M_WIDTH-1:0] restart_pc;
   input logic [`M_WIDTH-1:0] restart_src_pc;
   input logic 	      restart_src_is_indirect;
   input logic 	      restart_valid;
   output logic       restart_ack;
   //return stack signals
   input logic 			retire_valid;
   input logic 			retired_call;
   input logic 			retired_ret;

   input logic [4:0] 		retire_reg_ptr;
   input logic [`M_WIDTH-1:0] 	retire_reg_data;
   input logic 			retire_reg_valid;

   input logic 			branch_pc_valid;
   input logic			branch_pc_is_indirect;
   input logic [`M_WIDTH-1:0] 	branch_pc;
   input logic [`M_WIDTH-1:0]	target_pc;   
   
   input logic 			took_branch;
   input logic 			branch_fault;
   
   input logic [`LG_PHT_SZ-1:0] branch_pht_idx;

   
   output insn_fetch_t insn;
   output logic insn_valid;
   input logic 	insn_ack;
   
   output 	insn_fetch_t insn_two;
   output logic insn_valid_two;
   input logic 	insn_ack_two;

   output logic mem_req_valid;
   
   localparam L1I_NUM_SETS = 1 << `LG_L1I_NUM_SETS;
   localparam L1I_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1I_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);
   localparam LG_WORDS_PER_CL = `LG_L1D_CL_LEN - 2;
   localparam WORDS_PER_CL = 1<<LG_WORDS_PER_CL;
   localparam N_TAG_BITS = `PA_WIDTH - `LG_L1I_NUM_SETS - `LG_L1D_CL_LEN;
   localparam IDX_START = `LG_L1D_CL_LEN;
   localparam IDX_STOP  = `LG_L1D_CL_LEN + `LG_L1I_NUM_SETS;
   localparam WORD_START = 2;
   localparam WORD_STOP = WORD_START+LG_WORDS_PER_CL;
   localparam N_FQ_ENTRIES = 1 << `LG_FQ_ENTRIES;
   localparam RETURN_STACK_ENTRIES = 1 << `LG_RET_STACK_ENTRIES;
   localparam PHT_ENTRIES = 1 << `LG_PHT_SZ;
   localparam BTB_ENTRIES = 1 << `LG_BTB_SZ;

   output logic [(`PA_WIDTH-1):0] mem_req_addr;
   output logic [3:0] 			  mem_req_opcode;
   input logic 				  mem_rsp_valid;
   input logic [L1I_CL_LEN_BITS-1:0] 	  mem_rsp_load_data;
   output logic [63:0] 			  cache_accesses;
   output logic [63:0] 			  cache_hits;
   output logic [63:0] 			  tlb_accesses;
   output logic [63:0] 			  tlb_hits;   

   logic [N_TAG_BITS-1:0]		  t_cache_tag, r_cache_tag;

   wire					  w_last_out;
   
   wire [N_TAG_BITS-1:0]		  w_tag_out0, w_tag_out1;

   logic 				  r_pht_update;
   logic [1:0] 				  r_pht_out;
   logic [1:0]				  t_pht_out;
   logic [1:0] 				  t_pht_val;
   logic [7:0]				  r_pht_out_vec, r_pht_update_out;
   logic [7:0]				  t_pht_val_vec;
   
   logic 				  t_do_pht_wr;
   
   logic [`LG_PHT_SZ-1:0] 		  n_pht_idx,r_pht_idx;
   logic [`LG_PHT_SZ-1:0] 		  r_pht_update_idx;
   
   logic 				  r_take_br;
   
   logic [(`M_WIDTH-1):0] 		  r_btb[BTB_ENTRIES-1:0];
   
   
   logic [(3*WORDS_PER_CL)-1:0] 	  w_jump_out0, w_jump_out1;
   
   logic [`LG_L1I_NUM_SETS-1:0] 	    t_cache_idx, r_cache_idx;   
   logic 				    r_mem_req_valid, n_mem_req_valid;
   logic [(`M_WIDTH-1):0] 		    r_mem_req_addr, n_mem_req_addr;

   wire [127:0]				    w_array_out0, w_array_out1;   
   

   insn_fetch_t r_fq[N_FQ_ENTRIES-1:0];
   
   logic [`LG_FQ_ENTRIES:0] r_fq_head_ptr, n_fq_head_ptr;
   logic [`LG_FQ_ENTRIES:0] r_fq_next_head_ptr, n_fq_next_head_ptr;
   logic [`LG_FQ_ENTRIES:0] r_fq_next_tail_ptr, n_fq_next_tail_ptr;
   logic [`LG_FQ_ENTRIES:0] r_fq_next3_tail_ptr, n_fq_next3_tail_ptr;
   logic [`LG_FQ_ENTRIES:0] r_fq_next4_tail_ptr, n_fq_next4_tail_ptr;
   
   logic [`LG_FQ_ENTRIES:0] r_fq_tail_ptr, n_fq_tail_ptr;
   logic 		    r_resteer_bubble, n_resteer_bubble;
   
   
   logic 		    fq_full, fq_next_empty, fq_empty;
   logic 		    fq_full2, fq_full3, fq_full4;
   
   
   logic [(`M_WIDTH-1):0]   r_spec_return_stack [RETURN_STACK_ENTRIES-1:0];
   logic [(`M_WIDTH-1):0]   r_arch_return_stack [RETURN_STACK_ENTRIES-1:0];
   logic [`LG_RET_STACK_ENTRIES-1:0] n_arch_rs_tos, r_arch_rs_tos;
   logic [`LG_RET_STACK_ENTRIES-1:0] n_spec_rs_tos, r_spec_rs_tos, t_next_spec_rs_tos;   
   
   logic [`GBL_HIST_LEN-1:0] 	     n_arch_gbl_hist, r_arch_gbl_hist;
   logic [`GBL_HIST_LEN-1:0] 	     n_spec_gbl_hist, r_spec_gbl_hist;

   logic [`GBL_HIST_LEN-1:0] 	     r_last_spec_gbl_hist;
   

   logic [LG_WORDS_PER_CL-1:0]	     t_insn_idx,t_branch_idx;
   
   logic [63:0] 			 n_cache_accesses, r_cache_accesses;
   logic [63:0] 			 n_cache_hits, r_cache_hits;
   
function logic [31:0] select_cl32(logic [L1I_CL_LEN_BITS-1:0] cl, logic[LG_WORDS_PER_CL-1:0] pos);
   logic [31:0] 			 w32;
   case(pos)
     2'd0:
       w32 = cl[31:0];
     2'd1:
       w32 = cl[63:32];
     2'd2:
       w32 = cl[95:64];
     2'd3:
       w32 = cl[127:96];
   endcase // case (pos)
   return w32;
endfunction // select_cl32

function logic [63:0] select_jal_simm(logic [L1I_CL_LEN_BITS-1:0] cl, logic[LG_WORDS_PER_CL-1:0] pos);
   localparam				 PP = (`M_WIDTH-32);   
   logic [31:0] 			 w32;
   case(pos)
     2'd0:
       w32 = cl[31:0];
     2'd1:
       w32 = cl[63:32];
     2'd2:
       w32 = cl[95:64];
     2'd3:
       w32 = cl[127:96];
   endcase // case (pos)
   return {{(11+PP){w32[31]}}, w32[31], w32[19:12], w32[20], w32[30:21], 1'b0};
endfunction   

function logic [63:0] select_br_simm(logic [L1I_CL_LEN_BITS-1:0] cl, logic[LG_WORDS_PER_CL-1:0] pos);
   localparam				 PP = (`M_WIDTH-32);   
   logic [31:0] 			 w32;
   case(pos)
     2'd0:
       w32 = cl[31:0];
     2'd1:
       w32 = cl[63:32];
     2'd2:
       w32 = cl[95:64];
     2'd3:
       w32 = cl[127:96];
   endcase // case (pos)
   return {{(19+PP){w32[31]}}, w32[31], w32[7], w32[30:25], w32[11:8], 1'b0};
endfunction   

   
function logic [2:0] select_pd(logic [11:0] cl, logic[LG_WORDS_PER_CL-1:0] pos);
   logic [2:0] j;
   case(pos)
     2'd0:
       j = cl[2:0];
     2'd1:
       j = cl[5:3];
     2'd2:
       j = cl[8:6];
     2'd3:
       j = cl[11:9];
   endcase // case (pos)
   return j;
endfunction

   
   typedef enum logic [3:0] {INITIALIZE = 'd0,
			     IDLE = 'd1,
                             ACTIVE = 'd2,
                             INJECT_RELOAD = 'd3,
			     RELOAD_TURNAROUND = 'd4,
                             FLUSH_CACHE = 'd5,
			     WAIT_FOR_NOT_FULL = 'd6,
			     INIT_PHT = 'd7,
			     TLB_MISS = 'd8,
			     TLB_MISS_TURNAROUND = 'd9
			    } state_t;
   
   logic [(`M_WIDTH-1):0] r_pc, n_pc, r_miss_pc, n_miss_pc;
   logic [(`M_WIDTH-1):0] r_cache_pc, n_cache_pc;
   logic [(`M_WIDTH-1):0] r_btb_pc;


   wire [(`M_WIDTH-1):0]  w_cache_pc4 = r_cache_pc + 'd4;
   wire [(`M_WIDTH-1):0]  w_cache_pc8 = r_cache_pc + 'd8;
   wire [(`M_WIDTH-1):0]  w_cache_pc12 = r_cache_pc + 'd12;
   wire [(`M_WIDTH-1):0]  w_cache_pc16 = r_cache_pc + 'd16;
   wire [(`M_WIDTH-1):0]  w_cache_pc20 = r_cache_pc + 'd20;         
   
   
   state_t n_state, r_state;
   assign l1i_state = r_state;
   logic 		  r_restart_req, n_restart_req;
   logic 		  r_restart_ack, n_restart_ack;
   logic 		  r_req, n_req;
   logic		  w_valid_out0, w_valid_out1;
   
   logic 		  t_miss, t_hit;
   
   logic 		  t_push_insn, t_push_insn2,
			  t_push_insn3, t_push_insn4;
   
   logic		  t_unaligned_fetch;
   logic		  n_page_fault, r_page_fault;
   logic		  n_tlb_miss, r_tlb_miss;
   
   wire [`PA_WIDTH-1:0]	  w_tlb_pc;
   wire			  w_tlb_hit;  
   logic		  t_reload_tlb;
   
   logic 		  t_clear_fq;
   logic 		  r_flush_req, n_flush_req;
   logic 		  r_flush_complete, n_flush_complete;
   logic 		  t_take_br, t_is_cflow;
   logic		  t_take_br0, t_take_br1, t_take_br2, t_take_br3;
   
   logic 		  t_update_spec_hist;
   
   logic [31:0] 	  t_insn_data, t_insn_data2, t_insn_data3, t_insn_data4;
   logic [`M_WIDTH-1:0]   t_jal_simm, t_br_simm;
   logic 		  t_is_call, t_is_ret;
   logic [`M_WIDTH-1:0]	  t_ret_pc;
   
   logic [4:0]		  t_spec_branch_marker;
   logic [3:0]		  t_branch_marker, t_any_branch;
   logic [2:0] 		  t_first_branch;
   logic [2:0]		  t_taken_branch_idx;

   logic [63:0]		  r_branch_pc;   
   logic 		  t_init_pht, t_init_rsb;
   logic [`LG_PHT_SZ-1:0] r_init_pht_idx, n_init_pht_idx;


   
   localparam PP = (`M_WIDTH-32);
   localparam SEXT = `M_WIDTH-16;
   insn_fetch_t t_insn, t_insn2, t_insn3, t_insn4;
   logic [2:0] t_pd, t_first_pd;
   logic [2:0] t_pd0, t_pd1, t_pd2, t_pd3;
   logic       t_tcb0, t_tcb1, t_tcb2, t_tcb3;
   logic       t_br0,t_br1,t_br2,t_br3;
   
   
   logic [63:0] 	  r_cycle;
   always_ff@(posedge clk)
     begin
	r_cycle <= reset ? 'd0 : r_cycle + 'd1;
     end

   assign flush_complete = r_flush_complete;
   
   assign insn_valid = !fq_empty;
   assign insn_valid_two = !(fq_next_empty || fq_empty);

   
   assign restart_ack = r_restart_ack;

   assign mem_req_valid = r_mem_req_valid;
   assign mem_req_addr = r_mem_req_addr[`PA_WIDTH-1:0];
   assign mem_req_opcode = MEM_LW;
   assign cache_hits = r_cache_hits;
   assign cache_accesses = r_cache_accesses;

   assign page_walk_req_valid = r_tlb_miss;
   assign page_walk_req_va = r_miss_pc;
   
   
   wire [`M_WIDTH-1:0] w_restart_pc = restart_pc;
   
   always_comb
     begin
	n_fq_tail_ptr = r_fq_tail_ptr;
	n_fq_head_ptr = r_fq_head_ptr;
	n_fq_next_head_ptr = r_fq_next_head_ptr;
	n_fq_next_tail_ptr = r_fq_next_tail_ptr;
	n_fq_next3_tail_ptr = r_fq_next3_tail_ptr;
	n_fq_next4_tail_ptr = r_fq_next4_tail_ptr;
	
	fq_empty = (r_fq_head_ptr == r_fq_tail_ptr);
	fq_next_empty = (r_fq_next_head_ptr == r_fq_tail_ptr);
	
	fq_full = (r_fq_head_ptr != r_fq_tail_ptr) &&
		  (r_fq_head_ptr[`LG_FQ_ENTRIES-1:0] == r_fq_tail_ptr[`LG_FQ_ENTRIES-1:0]);
	
	fq_full2 = (r_fq_head_ptr != r_fq_next_tail_ptr) &&
		   (r_fq_head_ptr[`LG_FQ_ENTRIES-1:0] == r_fq_next_tail_ptr[`LG_FQ_ENTRIES-1:0]) || fq_full;
	
	fq_full3 = (r_fq_head_ptr != r_fq_next3_tail_ptr) &&
		   (r_fq_head_ptr[`LG_FQ_ENTRIES-1:0] == r_fq_next3_tail_ptr[`LG_FQ_ENTRIES-1:0]) || fq_full2;
	
	fq_full4 = (r_fq_head_ptr != r_fq_next4_tail_ptr) &&
		   (r_fq_head_ptr[`LG_FQ_ENTRIES-1:0] == r_fq_next4_tail_ptr[`LG_FQ_ENTRIES-1:0]) || fq_full3;
	
	insn = r_fq[r_fq_head_ptr[`LG_FQ_ENTRIES-1:0]];
	insn_two = r_fq[r_fq_next_head_ptr[`LG_FQ_ENTRIES-1:0]];

	if(t_push_insn4)
	  begin
	     n_fq_tail_ptr = r_fq_tail_ptr + 'd4;
	     n_fq_next_tail_ptr = r_fq_next_tail_ptr + 'd4;
	     n_fq_next3_tail_ptr = r_fq_next3_tail_ptr + 'd4;
	     n_fq_next4_tail_ptr = r_fq_next4_tail_ptr + 'd4;
	  end
	else if(t_push_insn3)
	  begin
	     n_fq_tail_ptr = r_fq_tail_ptr + 'd3;
	     n_fq_next_tail_ptr = r_fq_next_tail_ptr + 'd3;
	     n_fq_next3_tail_ptr = r_fq_next3_tail_ptr + 'd3;
	     n_fq_next4_tail_ptr = r_fq_next4_tail_ptr + 'd3;
	  end
	else if(t_push_insn2)
	  begin
	     n_fq_tail_ptr = r_fq_tail_ptr + 'd2;
	     n_fq_next_tail_ptr = r_fq_next_tail_ptr + 'd2;
	     n_fq_next3_tail_ptr = r_fq_next3_tail_ptr + 'd2;
	     n_fq_next4_tail_ptr = r_fq_next4_tail_ptr + 'd2;
	  end
	else if(t_push_insn)
	  begin
	     n_fq_tail_ptr = r_fq_tail_ptr + 'd1;
	     n_fq_next_tail_ptr = r_fq_next_tail_ptr + 'd1;
	     n_fq_next3_tail_ptr = r_fq_next3_tail_ptr + 'd1;
	     n_fq_next4_tail_ptr = r_fq_next4_tail_ptr + 'd1;
	  end
	
	if(insn_ack && !insn_ack_two)
	  begin
	     n_fq_head_ptr = r_fq_head_ptr + 'd1;
	     n_fq_next_head_ptr = r_fq_next_head_ptr + 'd1;
	  end
	else if(insn_ack && insn_ack_two)
	  begin
	     n_fq_head_ptr = r_fq_head_ptr + 'd2;
	     n_fq_next_head_ptr = r_fq_next_head_ptr + 'd2;
	  end
     end // always_comb

   always_ff@(posedge clk)
     begin
	if(t_push_insn)
	  begin
	     //$display("t_insn.pc = %x, t_clear_fq=%b", t_insn.pc,t_clear_fq);
	     r_fq[r_fq_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn;
	  end
	else if(t_push_insn2)
	  begin
	     //$display("t_insn.pc = %x, t_clear_fq=%b", t_insn.pc,t_clear_fq);
	     //$display("t_insn2.pc = %x", t_insn2.pc);	     
	     r_fq[r_fq_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn;
	     r_fq[r_fq_next_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn2;
	  end
	else if(t_push_insn3)
	  begin
	     //$display("t_insn.pc = %x, t_clear_fq=%b", t_insn.pc,t_clear_fq);	     	     
	     //$display("t_insn2.pc = %x", t_insn2.pc);
	     //$display("t_insn3.pc = %x", t_insn3.pc);	     	     	     
	     r_fq[r_fq_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn;
	     r_fq[r_fq_next_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn2;
	     r_fq[r_fq_next3_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn3;	  	     
	  end
	else if(t_push_insn4)
	  begin
	     //$display("cycle %d t_insn4.pc = %x, idx = %x, t_take_br = %b", r_cycle, t_insn4.pc, r_pht_idx, t_take_br);	     	     	     
	     r_fq[r_fq_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn;
	     r_fq[r_fq_next_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn2;
	     r_fq[r_fq_next3_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn3;
	     r_fq[r_fq_next4_tail_ptr[`LG_FQ_ENTRIES-1:0]] <= t_insn4;	  	     	     
	  end
     end // always_ff@ (posedge clk)


   //always_ff@(negedge clk)
   //begin
   //if(t_update_spec_hist)
   //	  begin
   //$display("cycle %d cache pc %x : spec hist update to %b", 
   //r_cycle, r_cache_pc, n_spec_gbl_hist);
   //end
   //if(restart_valid)
   ///begin
   //$display("mispredict cycle %d branch at %x, target %x, pht %x", r_cycle, branch_pc, target_pc, branch_pht_idx[(`LG_BTB_SZ-1):0]);
   //end
   //end
   
   always_ff@(posedge clk)
     begin
	if(r_state == INIT_PHT)
	  begin
	     r_btb[r_init_pht_idx[`LG_BTB_SZ-1:0]] <= 64'd0;
	  end
	else if(branch_pc_is_indirect & branch_pc_valid)
	  begin
	     r_btb[branch_pht_idx[(`LG_BTB_SZ-1):0]] <= target_pc;
	  end	
     end // always_ff@ (posedge clk)

   always_ff@(posedge clk)
     begin
	r_btb_pc <= reset ? 'd0 : r_btb[n_pht_idx[(`LG_BTB_SZ-1):0]];
     end


   wire w_hit0 = w_valid_out0 ? w_tag_out0 == w_tlb_pc[`PA_WIDTH-1:IDX_STOP] : 1'b0;
   wire	w_hit1 = w_valid_out1 ? w_tag_out1 == w_tlb_pc[`PA_WIDTH-1:IDX_STOP] : 1'b0;   
   wire	w_hit = w_hit0 | w_hit1;
   wire [127:0]	w_array = w_hit0 ? w_array_out0 : w_array_out1;
   wire [(3*WORDS_PER_CL)-1:0] w_jump = w_hit0 ? w_jump_out0 : w_jump_out1;

   // always_ff@(negedge clk)
   //   begin
   // 	if(r_req)
   // 	  begin
   // 	     $display("cycle %d, hit 0 %b, hit 1 %b", r_cycle, w_hit0, w_hit1);
   // 	  end
   //   end
   logic			r_reload, n_reload;
   logic [63:0]			t_br_disp, t_j_disp;
   logic [15:0]			n_wait_cycles,r_wait_cycles;
   
   always_ff@(posedge clk)
     begin
	r_wait_cycles <= reset ? 'd0 : n_wait_cycles;
     end
   always_comb
     begin
	n_wait_cycles = r_wait_cycles;
	n_page_fault = r_page_fault;
	n_pc = r_pc;
	n_miss_pc = r_miss_pc;
	n_cache_pc = 'd0;
	n_state = r_state;
	n_reload = r_reload;
	
	n_restart_ack = 1'b0;
	n_flush_req = r_flush_req | flush_req;
	n_flush_complete = 1'b0;
	t_cache_idx = 'd0;
	t_cache_tag = 'd0;
	n_req = 1'b0;
	n_mem_req_valid = 1'b0;
	n_mem_req_addr = r_mem_req_addr;
	n_resteer_bubble = 1'b0;
	t_next_spec_rs_tos = r_spec_rs_tos+'d1;
	n_restart_req = restart_valid | r_restart_req;
		
	t_miss = r_req && !w_hit;
	t_hit = r_req && w_hit;

	t_insn_idx = r_cache_pc[WORD_STOP-1:WORD_START];
	
	t_pd = select_pd(w_jump, t_insn_idx);
	
	
	t_pd0 = select_pd(w_jump, 'd0);
	t_pd1 = select_pd(w_jump, 'd1);
	t_pd2 = select_pd(w_jump, 'd2);
	t_pd3 = select_pd(w_jump, 'd3);	
	
	t_br0 = (t_pd0 != 3'd0);
	t_br1 = (t_pd1 != 3'd0);
	t_br2 = (t_pd2 != 3'd0);
	t_br3 = (t_pd3 != 3'd0);	
	
	
	t_insn_data  = select_cl32(w_array, t_insn_idx);
	t_insn_data2 = select_cl32(w_array, t_insn_idx + 2'd1);
	t_insn_data3 = select_cl32(w_array, t_insn_idx + 2'd2);
	t_insn_data4 = select_cl32(w_array, t_insn_idx + 2'd3);

	r_pht_out = t_insn_idx == 2'd0 ? r_pht_out_vec[1:0] :
		    t_insn_idx == 2'd1 ? r_pht_out_vec[3:2] :
		    t_insn_idx == 2'd2 ? r_pht_out_vec[5:4] :
		    r_pht_out_vec[7:6];
	
	t_tcb0 = (((t_pd0 == 3'd1) & (r_pht_out_vec[1]==1'b0)) | (t_pd0 == 3'd0))==1'b0;
	t_tcb1 = (((t_pd1 == 3'd1) & (r_pht_out_vec[3]==1'b0)) | (t_pd1 == 3'd0))==1'b0;
	t_tcb2 = (((t_pd2 == 3'd1) & (r_pht_out_vec[5]==1'b0)) | (t_pd2 == 3'd0))==1'b0;
	t_tcb3 = (((t_pd3 == 3'd1) & (r_pht_out_vec[7]==1'b0)) | (t_pd3 == 3'd0))==1'b0;	
	
	t_spec_branch_marker = ({1'b1,
				 t_tcb3,
				 t_tcb2,
				 t_tcb1,
				 t_tcb0
				 } >> t_insn_idx);
	

	t_branch_marker = ({t_tcb3,
			    t_tcb2,
			    t_tcb1,
			    t_tcb0
			    } >> t_insn_idx);

	t_any_branch = ({t_br3,
                         t_br2,
                         t_br1,
                         t_br0
                         } >> t_insn_idx);
	
	t_taken_branch_idx = 'd7;
	casez(t_branch_marker)
	  4'b???1:
	    t_taken_branch_idx = 'd0;
	  4'b??10:
	    t_taken_branch_idx = 'd1;
	  4'b?100:
	    t_taken_branch_idx = 'd2;
	  4'b1000:
	    t_taken_branch_idx = 'd3;
	  default:
	    t_taken_branch_idx = 'd7;
	endcase	
	
	t_first_branch = 'd0;
	casez(t_spec_branch_marker)
	  5'b????1:
	    t_first_branch = 'd0;
	  5'b???10:
	    t_first_branch = 'd1;
	  5'b??100:
	    t_first_branch = 'd2;
	  5'b?1000:
	    t_first_branch = 'd3;
	  5'b10000:
	    t_first_branch = 'd4;
	  default:
	    t_first_branch = 'd7;
	endcase

	t_branch_idx = t_taken_branch_idx[1:0] + t_insn_idx;
	t_pht_out = t_branch_idx=='d0 ? r_pht_out_vec[1:0] :
		    t_branch_idx=='d1 ? r_pht_out_vec[3:2] :
		    t_branch_idx=='d2 ? r_pht_out_vec[5:4] :
		    r_pht_out_vec[7:6];
	
	t_first_pd = select_pd(w_jump, t_branch_idx);
	
		
	t_jal_simm = {{(11+PP){t_insn_data[31]}}, t_insn_data[31], t_insn_data[19:12], t_insn_data[20], t_insn_data[30:21], 1'b0};
	
	t_br_simm = {{(19+PP){t_insn_data[31]}}, t_insn_data[31], t_insn_data[7], t_insn_data[30:25], t_insn_data[11:8], 1'b0};

	t_br_disp = select_br_simm(w_array, t_branch_idx);
	t_j_disp = select_jal_simm(w_array, t_branch_idx);
	
	
	t_clear_fq = 1'b0;
	t_push_insn = 1'b0;
	t_push_insn2 = 1'b0;
	t_push_insn3 = 1'b0;
	t_push_insn4 = 1'b0;
	t_unaligned_fetch = 1'b0;
	
	t_take_br = 1'b0;
	t_take_br0 = 1'b0;
	t_take_br1 = 1'b0;
	t_take_br2 = 1'b0;
	t_take_br3 = 1'b0;	
	t_is_cflow = 1'b0;
	t_update_spec_hist = 1'b0;
	t_is_call = 1'b0;
	t_ret_pc = w_cache_pc4;
	t_is_ret = 1'b0;
	t_init_pht = 1'b0;
	t_init_rsb = 1'b0;
	n_init_pht_idx = r_init_pht_idx;
	t_reload_tlb = 1'b0;
	n_tlb_miss = 1'b0;
	
	case(r_state)
	  INITIALIZE:
	    begin
	       n_state = INIT_PHT;
	    end
	  INIT_PHT:
	    begin
	       t_init_pht = 1'b1;
	       t_init_rsb = 1'b1;	       
	       n_init_pht_idx = r_init_pht_idx + 'd1;
	       if(r_init_pht_idx == (PHT_ENTRIES-1))
		 begin
		    n_state = FLUSH_CACHE;	       		    
		    t_cache_idx = 0;
		 end
	    end
	  IDLE:
	    begin
	       if(n_restart_req)
		 begin
		    n_restart_ack = 1'b1;
		    n_restart_req = 1'b0;
		    n_pc = w_restart_pc;
		    //$display("restart ack, pc %x at cycle %d", n_pc, r_cycle);
		    n_state = ACTIVE;
		    t_clear_fq = 1'b1;
		 end
	    end	  
	  ACTIVE:
	    begin
	       t_cache_idx = r_pc[IDX_STOP-1:IDX_START];
	       t_cache_tag = r_pc[(`PA_WIDTH-1):IDX_STOP];
	       /* accessed with this address */
	       n_cache_pc = r_pc;
	       n_req = 1'b1;
	       n_pc = r_pc + 'd4;
	       if(r_resteer_bubble)
		 begin
		    //do nothing?
		 end
	       else if(n_flush_req)
		 begin
		    n_flush_req = 1'b0;
		    t_clear_fq = 1'b1;
		    n_state = FLUSH_CACHE;
		    t_cache_idx = 0;
		 end
	       else if(n_restart_req)
		 begin
		    n_restart_ack = 1'b1;
		    n_restart_req = 1'b0;
		    n_pc = w_restart_pc;
		    //$display("restart ack, pc %x at cycle %d", n_pc, r_cycle);		    
		    n_req = 1'b0;
		    n_state = ACTIVE;
		    t_clear_fq = 1'b1;
		    n_page_fault = 1'b0;
		 end // if (n_restart_req)
	       else if(r_page_fault)
		 begin
		    if(!fq_full)
		      begin
			 //$display("taking page fault for pc %x at cycle %d, paging_active %b", 
			 //r_cache_pc, r_cycle, paging_active);
			 n_page_fault = 1'b0;
			 t_push_insn = 1'b1;
		      end
		 end
	       else if(!w_tlb_hit & r_req && paging_active)
		 begin
		    //$display("iside TLB MISS for r_cache_pc %x, r_cycle %d", r_cache_pc, r_cycle);
		    n_state = TLB_MISS;
		    n_pc = r_pc;
		    n_miss_pc = r_cache_pc;
		    n_tlb_miss = 1'b1;
		 end
	       else if(t_miss)
		 begin
		    n_state = INJECT_RELOAD;
		    n_mem_req_addr = paging_active ? {32'd0, w_tlb_pc[`PA_WIDTH-1:`LG_L1D_CL_LEN], {`LG_L1D_CL_LEN{1'b0}}} : 
				     {r_cache_pc[`M_WIDTH-1:`LG_L1D_CL_LEN], {`LG_L1D_CL_LEN{1'b0}}};
		    n_mem_req_valid = 1'b1;
		    n_miss_pc = r_cache_pc;
		    n_pc = r_pc;
		    n_reload = r_cycle[0];//!w_last_out;
		    //$display("cycle %d :missed for pc %x, set idx %d, replace way %d, w_tag_out0 = %x w_tag_out1 = %x",
		    //r_cycle,
		    //n_mem_req_addr,
		    //r_cache_pc[11:4],
		    //n_reload,
		    //	     {w_tag_out0, r_cache_pc[11:4], 4'd0},
		    //{w_tag_out1, r_cache_pc[11:4], 4'd0}
		    //);
		 end
	       else if(t_hit && !fq_full)
		 begin
		    if((t_taken_branch_idx == 'd3) & !fq_full4 & (t_first_pd == 3'd1 | t_first_pd == 3'd3))
		      begin
			 t_update_spec_hist = 1'b1;
			 t_push_insn4 = 1'b1;
			 t_is_cflow = 1'b1;
			 t_take_br = 1;
			 n_pc = w_cache_pc12 + ((t_first_pd==3'd3)? t_j_disp :	t_br_disp);
			 //$display("branch = %x, n_pc = %x", w_cache_pc12, n_pc);
		      end // if ((t_taken_branch_idx == 'd3) & !fq_full4)
		    else if((t_taken_branch_idx == 'd2) & !fq_full3 & (t_first_pd == 3'd1 | t_first_pd == 3'd3))
		      begin
			 t_update_spec_hist = 1'b1;
			 t_push_insn3 = 1'b1;
			 t_is_cflow = 1'b1;
			 t_take_br = 1;
			 n_pc = w_cache_pc8 + ((t_first_pd=='d3) ? t_j_disp :t_br_disp);
			 //$display("branch = %x, n_pc = %x", w_cache_pc8, n_pc);			 
		      end
		    else if((t_taken_branch_idx == 'd1) & !fq_full2 & (t_first_pd == 3'd1 | t_first_pd == 3'd3))
		      begin
			 t_update_spec_hist = 1'b1;
			 t_push_insn2 = 1'b1;
			 t_is_cflow = 1'b1;
			 t_take_br = 1;
			 n_pc = w_cache_pc4 + ((t_first_pd=='d3) ? t_j_disp : t_br_disp);
			 //$display("branch = %x, n_pc = %x", w_cache_pc4, n_pc);			 
		      end // if ((t_taken_branch_idx == 'd1) & !fq_full2)
		    else
		      begin
			 t_update_spec_hist = (t_pd != 3'd0);
			 
			 if(t_pd == 3'd5 || t_pd == 3'd3) /* jal and j */
			   begin
			      t_is_cflow = 1'b1;
			      t_take_br = 1;
			      t_is_call = (t_pd == 3'd5);
			      n_pc = r_cache_pc + t_jal_simm;
			      t_push_insn = 1'b1;
			   end
			 else if(t_pd == 3'd1 && r_pht_out[1])
			   begin
			      t_is_cflow = 1'b1;
			      t_take_br = 1;			 
			      n_pc = (r_cache_pc + t_br_simm);
			      t_push_insn = 1'b1;
			   end
			 else if(t_pd == 3'd2) /* return */
			   begin
			      t_is_cflow = 1'b1;
			      t_is_ret = 1'b1;
			      t_take_br = 1'b1;
			      n_pc = r_spec_return_stack[t_next_spec_rs_tos];
			      t_push_insn = 1'b1;
			   end // if (t_pd == 4'd7)
			 else if(t_pd == 3'd4 || t_pd == 3'd6)
			   begin
			      t_is_cflow = 1'b1;			 
			      t_take_br = 1'b1;
			      t_is_call = (t_pd == 3'd6);
			      n_pc = r_btb_pc;
			      t_push_insn = 1'b1;			      
			   end
			 //$display("branch = %x, n_pc = %x", r_cache_pc, n_pc);			 
		      end // else: !if((t_taken_branch_idx == 'd1) & !fq_full2)

		    n_resteer_bubble = t_is_cflow;
		    
		    //initial push multiple logic
		    if(t_is_cflow==1'b0)
		      begin
			 if(t_first_branch == 'd4 && !fq_full4)
			   begin
			      t_push_insn4 = 1'b1;
			      t_cache_idx = r_cache_idx + 'd1;
			      n_cache_pc = w_cache_pc16;
			      t_cache_tag = n_cache_pc[(`PA_WIDTH-1):IDX_STOP];
			      n_pc = w_cache_pc20;
			      t_update_spec_hist = |t_any_branch;
			   end
			 else if(t_first_branch == 'd3 && !fq_full3)
			   begin
			      t_push_insn3 = 1'b1;
			      n_cache_pc = w_cache_pc12;
			      n_pc = w_cache_pc16;
			      t_cache_tag = n_cache_pc[(`PA_WIDTH-1):IDX_STOP];
			      t_update_spec_hist = |t_any_branch;			      
			      if(t_insn_idx != 0)
				begin
				   t_cache_idx = r_cache_idx + 'd1;
				end
			      //
			   end
			 else if(t_first_branch == 'd2 && !fq_full2)
			   begin
			      t_push_insn2 = 1'b1;
			      n_cache_pc = w_cache_pc8;
			      t_cache_tag = n_cache_pc[(`PA_WIDTH-1):IDX_STOP];
			      n_pc = w_cache_pc12;
			      t_update_spec_hist = |t_any_branch;
			      if(t_insn_idx == 2)
				begin
				   t_cache_idx = r_cache_idx + 'd1;
				end
			   end
			 else
			   begin
			      t_push_insn = 1'b1;
			   end // else: !if(t_first_branch == 'd2 && !fq_full2)
		      end
		 end // if (t_hit && !fq_full)
	       else if(t_hit && fq_full)
		 begin
		    //$display("full insn queue at cycle %d", r_cycle);
		    n_pc = r_pc;
		    n_miss_pc = r_cache_pc;
		    n_state = WAIT_FOR_NOT_FULL;
		 end
	    end
	  INJECT_RELOAD:
	    begin
	       n_wait_cycles = r_wait_cycles + 'd1;
	       if(&r_wait_cycles)
		 begin
		    $display("icache fetch request for %x timed out at cycle %d, r_miss_pc %x, phys addr %x, cycles %d", 
			     r_pc, r_cycle, r_miss_pc, r_mem_req_addr, r_wait_cycles);
		    $stop();
		 end
	       if(mem_rsp_valid)
		 begin
		    //$display("icache fetch request for %x returns with data %x at cycle %d, r_miss_pc %x", r_pc, mem_rsp_load_data, r_cycle, r_miss_pc);
		    n_state = RELOAD_TURNAROUND;
		    n_wait_cycles = 'd0;
		 end
	    end
	  RELOAD_TURNAROUND:
	    begin
	       t_cache_idx = r_miss_pc[IDX_STOP-1:IDX_START];
	       t_cache_tag = r_miss_pc[(`PA_WIDTH-1):IDX_STOP];
	       if(n_flush_req)
		 begin
		    n_flush_req = 1'b0;
		    t_clear_fq = 1'b1;
		    n_state = FLUSH_CACHE;
		    t_cache_idx = 0;
		 end
	       else if(n_restart_req)
		 begin
		    n_restart_ack = 1'b1;
		    n_restart_req = 1'b0;
		    n_pc = w_restart_pc;
		    //$display("restart ack, pc %x at cycle %d", n_pc, r_cycle);		    
		    n_req = 1'b0;
		    n_state = ACTIVE;
		    t_clear_fq = 1'b1;
		    n_page_fault = 1'b0;
		 end // if (n_restart_req)
	       else if(!fq_full)
		 begin
		    /* accessed with this address */
		    n_cache_pc = r_miss_pc;
		    n_req = 1'b1;
		    n_state = ACTIVE;
		 end
	    end
	  FLUSH_CACHE:
	    begin
	       if(r_cache_idx == (L1I_NUM_SETS-1))
		 begin
		    //$display("REQ FLUSHING COMPLETE at %d", r_cycle);
		    n_flush_complete = 1'b1;
		    n_state = IDLE;
		 end
	       t_cache_idx = r_cache_idx + 'd1;
	    end
	  WAIT_FOR_NOT_FULL:
	    begin
	       t_cache_idx = r_miss_pc[IDX_STOP-1:IDX_START];
	       t_cache_tag = r_miss_pc[(`PA_WIDTH-1):IDX_STOP];
	       n_cache_pc = r_miss_pc;
	       if(n_flush_req)
		 begin
		    n_flush_req = 1'b0;
		    //n_flush_complete = 1'b1;
		    t_clear_fq = 1'b1;
		    n_state = FLUSH_CACHE;
		    t_cache_idx = 0;		    
		 end	       
	       else if(!fq_full)
		 begin
		    n_req = 1'b1;
		    n_state = ACTIVE;
		 end
	       else if(n_restart_req)
		 begin
		    n_restart_ack = 1'b1;
		    n_restart_req = 1'b0;
		    n_pc = w_restart_pc;
		    //$display("restart ack, pc %x at cycle %d", n_pc, r_cycle);
		    n_req = 1'b0;
		    n_state = ACTIVE;
		    t_clear_fq = 1'b1;
		    n_page_fault = 1'b0;
		 end // if (n_restart_req)
	    end // case: WAIT_FOR_NOT_FULL
	  TLB_MISS:
	    begin
	       if(page_walk_rsp_valid)
	          begin
	             n_page_fault = page_walk_rsp.fault;
	             t_reload_tlb = page_walk_rsp.fault==1'b0;
		     n_state = TLB_MISS_TURNAROUND;
		     //$display("mmu returns for %x, page fault %b at cycle %d", 
		     //r_miss_pc, n_page_fault, r_cycle);
		     //if(t_page_walk_pa != page_walk_rsp_pa)
		     //begin
		     //$display("va %x : local %x vs mmu %x", r_miss_pc, t_page_walk_pa, page_walk_rsp_pa);
		     //$stop();
		     //end
	          end
	       
	       //n_state = ACTIVE;
	       //n_req = 1'b1;
	       //n_cache_pc = r_miss_pc;
	       //t_cache_idx = r_miss_pc[IDX_STOP-1:IDX_START];
	       //t_cache_tag = r_miss_pc[(`M_WIDTH-1):IDX_STOP];	       
	       //n_page_fault = &t_page_walk_pa;
	       //t_reload_tlb = (&t_page_walk_pa)==1'b0;
	    end // case: TLB_MISS
	  TLB_MISS_TURNAROUND:
	    begin
	       n_cache_pc = r_miss_pc;
	       t_cache_idx = r_miss_pc[IDX_STOP-1:IDX_START];
	       t_cache_tag = r_miss_pc[(`PA_WIDTH-1):IDX_STOP];
	       n_state = ACTIVE;
	       n_req = 1'b1;
	    end
	  default:
	    begin
	    end
	endcase // case (r_state)
     end // always_comb

   // always_ff@(negedge clk)
   //   begin
   // 	if(t_page_fault)
   // 	  begin
   // 	     $display("took instruction page fault for va %x, got pa %x at cycle %d, priv %d",
   // 		      r_cache_pc,
   // 		      r_cache_pc_pa, 
   // 		      r_cycle, 
   // 		      priv);
   // 	  end
   //   end

   
   always_comb
     begin
	n_cache_accesses = r_cache_accesses;
	n_cache_hits = r_cache_hits;	
	if(t_hit)
	  begin
	     n_cache_hits = r_cache_hits + 'd1;
	  end
	if(r_req)
	  begin
	     n_cache_accesses = r_cache_accesses + 'd1;
	  end
     end
   
   always_comb
     begin
	t_insn.insn_bytes = t_insn_data;
	t_insn.page_fault = r_page_fault;
	t_insn.pc = r_cache_pc;
	t_insn.pred_target = n_pc;
	t_insn.pred = t_taken_branch_idx=='d0;
	t_insn.pht_idx = r_pht_idx;
`ifdef	ENABLE_CYCLE_ACCOUNTING
	t_insn.fetch_cycle = r_cycle;
`endif
	t_insn2.insn_bytes = t_insn_data2;
	t_insn2.page_fault = 1'b0;
	t_insn2.pc = w_cache_pc4;
	t_insn2.pred_target = n_pc;
	t_insn2.pred = t_taken_branch_idx=='d1;
	t_insn2.pht_idx = r_pht_idx;
`ifdef	ENABLE_CYCLE_ACCOUNTING
	t_insn2.fetch_cycle = r_cycle;
`endif
	t_insn3.insn_bytes = t_insn_data3;
	t_insn3.page_fault = 1'b0;	
	t_insn3.pc = w_cache_pc8;
	t_insn3.pred_target = n_pc;
	t_insn3.pred = t_taken_branch_idx=='d2;
	t_insn3.pht_idx = r_pht_idx;
`ifdef	ENABLE_CYCLE_ACCOUNTING
	t_insn3.fetch_cycle = r_cycle;
`endif
	t_insn4.insn_bytes = t_insn_data4;
	t_insn4.page_fault = 1'b0;	
	t_insn4.pc = w_cache_pc12;
	t_insn4.pred_target = n_pc;
	t_insn4.pred = t_taken_branch_idx=='d3;
	t_insn4.pht_idx = r_pht_idx;
`ifdef	ENABLE_CYCLE_ACCOUNTING
	t_insn4.fetch_cycle = r_cycle;
`endif
     end // always_comb
   
   logic t_wr_valid_ram_en0, t_wr_valid_ram_en1;
   logic t_valid_ram_value0, t_valid_ram_value1;

   logic t_last_ram_value, t_last_ram_en;
   
   logic [`LG_L1I_NUM_SETS-1:0] t_valid_ram_idx;


   

   always_comb
     begin
	t_last_ram_en = r_req || (r_state == FLUSH_CACHE);
	t_last_ram_value = w_hit1;
     end
   
   //compute_pht_idx cpi0 (.pc(n_cache_pc), .hist(r_spec_gbl_hist), .idx(n_pht_idx));
   compute_pht_idx cpi0 (.pc({n_cache_pc[63:4], 4'd0}), .hist(r_spec_gbl_hist), .idx(n_pht_idx));

   logic r_print_pht;
   always_ff@(posedge clk)
     begin
	r_print_pht <= reset ? 1'b0 : (n_cache_pc[63:4] == 60'h1099);
     end
   
   // always_ff@(negedge clk)
   //   begin
   // 	if(n_cache_pc[63:4] == 60'h1099)
   // 	  begin
   // 	     $display("cycle %d, n_cache_pc = %x, n_pht_idx = %x, hist = %b, br idx %d, take %b", 
   // 		      r_cycle, 
   // 		      n_cache_pc, 
   // 		      n_pht_idx, 
   // 		      r_spec_gbl_hist,
   // 		      t_branch_idx,
   // 		      t_take_br);
   // 	  end
   // 	if(r_print_pht)
   // 	  begin
   // 	     $display("cycle %d, pht read %b", r_cycle, r_pht_out_vec);
   // 	  end
	
   // 	if(r_pht_update && r_branch_pc[63:4] == 60'h1099)
   // 	  begin
   // 	     $display("cycle %d, t_pht_val = %b, r_take_br = %b, %b", 
   // 		      r_cycle, t_pht_val, r_take_br,
   // 		      r_arch_gbl_hist);
   // 	  end
	
   // 	if(branch_pc[63:4] == 60'h1099 && branch_pc_valid)
   // 	  begin
   // 	     $display("cycle %d,branch pht idx= %d", r_cycle, branch_pht_idx);
   // 	  end//branch_pht_idx
	
   // 	if(branch_pc_valid & branch_pht_idx == 'd48539)
   // 	  begin
   // 	     $display("cycle %d, branch at pc %x maps to pht entry",
   // 		      r_cycle, branch_pc);
   // 	  end
	
   //   end   

   
   always_comb
     begin
	t_wr_valid_ram_en0 = (mem_rsp_valid && (r_reload == 1'b0)) || (r_state == FLUSH_CACHE);
	t_wr_valid_ram_en1 = (mem_rsp_valid && (r_reload == 1'b1)) || (r_state == FLUSH_CACHE);
	
	t_valid_ram_value0 = (r_state != FLUSH_CACHE);
	t_valid_ram_value1 = (r_state != FLUSH_CACHE);
	
	t_valid_ram_idx = mem_rsp_valid ? r_mem_req_addr[IDX_STOP-1:IDX_START] : r_cache_idx;
     end

   always_comb
     begin
	t_pht_val = 'd0;
	case(r_branch_pc[3:2])
	  2'd0:
	    t_pht_val = r_pht_update_out[1:0];
	  2'd1:
	    t_pht_val = r_pht_update_out[3:2];
	  2'd2:
	    t_pht_val = r_pht_update_out[5:4];
	  2'd3:
	    t_pht_val = r_pht_update_out[7:6];
	endcase // case (r_branch_pc[4:3])
	
	t_do_pht_wr = r_pht_update;

	case(t_pht_val)
	  2'd0:
	    begin
	       if(r_take_br)
		 begin
		    t_pht_val =  2'd1;
		 end
	       else
		 begin
		    t_do_pht_wr = 1'b0;
		 end
	    end
	  2'd1:
	    begin
	       t_pht_val = r_take_br ? 2'd2 : 2'd0;
	    end
	  2'd2:
	    begin
	       t_pht_val = r_take_br ? 2'd3 : 2'd1;
	    end
	  2'd3:
	    begin
	       if(!r_take_br)
		 begin
		    t_pht_val = 2'd2;
		 end
	       else
		 begin
		    t_do_pht_wr = 1'b0;
		 end
	    end
	endcase // case (r_pht_update_out)
	t_pht_val_vec = 8'd0;
	case(r_branch_pc[3:2])
	  2'd0:
	    t_pht_val_vec = {r_pht_update_out[7:2], t_pht_val};
	  2'd1:
	    t_pht_val_vec = {r_pht_update_out[7:4], t_pht_val, r_pht_update_out[1:0]};
	  2'd2:
	    t_pht_val_vec = {r_pht_update_out[7:6], t_pht_val, r_pht_update_out[3:0]};
	  2'd3:
	    t_pht_val_vec = {t_pht_val, r_pht_update_out[5:0]};
	endcase // case (r_branch_pc[4:3])
     end // always_comb

   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_pht_idx <= 'd0;
	     r_last_spec_gbl_hist <= 'd0;
	     r_pht_update <= 1'b0;
	     r_pht_update_idx <= 'd0;
	     r_take_br <= 1'b0;
	     r_branch_pc <= 'd0;
	  end
	else
	  begin
	     r_pht_idx <= n_pht_idx;
	     r_last_spec_gbl_hist <= r_spec_gbl_hist;
	     r_pht_update <= branch_pc_valid;
	     r_pht_update_idx <= branch_pht_idx;
	     r_take_br <= took_branch;
	     r_branch_pc <= branch_pc;
	  end
     end // always_ff@


   
   tlb #(.LG_N(4), .ISIDE(1)) 
   itlb(
	.clk(clk), 
	.reset(reset),
	.priv(priv),
	.clear(clear_tlb),
	.active(paging_active),
	.req(n_req),
	.va(n_cache_pc),
	.pa(w_tlb_pc),
	.hit(w_tlb_hit),
	.dirty(),
	.readable(),
	.writable(),
	.user(),
	.zero_page(),
	.tlb_hits(tlb_hits),
	.tlb_accesses(tlb_accesses),	
	.replace_va(r_miss_pc),			
	.replace(t_reload_tlb),			
	.page_walk_rsp(page_walk_rsp)
	);
   
   
`ifdef VERILATOR
   always_ff@(negedge clk)
     begin
	//$display("fe in state %d at cycle %d", r_state, r_cycle);
	//$display("%b %b %b %b", t_push_insn, t_push_insn2, t_push_insn3, t_push_insn4);
	record_fetch(t_push_insn ? 32'd1 : 32'd0,
		     t_push_insn2 ? 32'd1 : 32'd0,
		     t_push_insn3 ? 32'd1 : 32'd0,
		     t_push_insn4 ? 32'd1 : 32'd0,
		     { {(64-`M_WIDTH){1'b0}},t_insn.pc},
		     { {(64-`M_WIDTH){1'b0}},t_insn2.pc},
		     { {(64-`M_WIDTH){1'b0}},t_insn3.pc},
		     { {(64-`M_WIDTH){1'b0}},t_insn4.pc},		     
		     r_resteer_bubble ? 32'd1 : 32'd0,
		     fq_full ? 32'd1 : 32'd0);
	
	
     end
`endif


   ram1r1w #(.WIDTH(1), .LG_DEPTH(`LG_L1I_NUM_SETS))
   last_array (
	       .clk(clk),
	       .rd_addr(t_cache_idx),
	       .wr_addr(t_cache_idx),
	       .wr_data(t_last_ram_value),
	       .wr_en(t_last_ram_en),
	       .rd_data(w_last_out)
	       );

   	  
   ram2r1w #(.WIDTH(8), .LG_DEPTH(`LG_PHT_SZ) ) pht0
     (
      .clk(clk),
      .rd_addr0(n_pht_idx),
      .rd_addr1(branch_pht_idx),
      .wr_addr(t_init_pht ? r_init_pht_idx : r_pht_update_idx),
      .wr_data(t_init_pht ? 8'b01010101 : t_pht_val_vec),
      .wr_en(t_init_pht || t_do_pht_wr),
      .rd_data0(r_pht_out_vec),
      .rd_data1(r_pht_update_out)
      );

   
         
   ram1r1w #(.WIDTH(1), .LG_DEPTH(`LG_L1I_NUM_SETS))
   valid_array0 (
	   .clk(clk),
	   .rd_addr(t_cache_idx),
	   .wr_addr(t_valid_ram_idx),
	   .wr_data(t_valid_ram_value0),
	   .wr_en(t_wr_valid_ram_en0),
	   .rd_data(w_valid_out0)
	   );

   
   ram1r1w #(.WIDTH(N_TAG_BITS), .LG_DEPTH(`LG_L1I_NUM_SETS))
   tag_array0 (
	   .clk(clk),
	   .rd_addr(t_cache_idx),
	   .wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
	   .wr_data(r_mem_req_addr[`PA_WIDTH-1:IDX_STOP]),
	   .wr_en(mem_rsp_valid & (r_reload == 1'b0)),
	   .rd_data(w_tag_out0)
	   );
   
   ram1r1w #(.WIDTH(L1I_CL_LEN_BITS), .LG_DEPTH(`LG_L1I_NUM_SETS)) 
   insn_array0 (
	   .clk(clk),
	   .rd_addr(t_cache_idx),
	   .wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
	   .wr_data({mem_rsp_load_data[127:96],
		     mem_rsp_load_data[95:64],
		     mem_rsp_load_data[63:32], 
		     mem_rsp_load_data[31:0]}),
	   .wr_en(mem_rsp_valid & (r_reload == 1'b0)),
	   .rd_data(w_array_out0)
	   );


   
   ram1r1w #(.WIDTH(1), .LG_DEPTH(`LG_L1I_NUM_SETS))
   valid_array1 (
	   .clk(clk),
	   .rd_addr(t_cache_idx),
	   .wr_addr(t_valid_ram_idx),
	   .wr_data(t_valid_ram_value1),
	   .wr_en(t_wr_valid_ram_en1),
	   .rd_data(w_valid_out1)
	   );

   
   ram1r1w #(.WIDTH(N_TAG_BITS), .LG_DEPTH(`LG_L1I_NUM_SETS))
   tag_array1 (
    	       .clk(clk),
	       .rd_addr(t_cache_idx),
	       .wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
	       .wr_data(r_mem_req_addr[`PA_WIDTH-1:IDX_STOP]),
	       .wr_en(mem_rsp_valid & (r_reload == 1'b1)),
	       .rd_data(w_tag_out1)
	       );
   
   ram1r1w #(.WIDTH(L1I_CL_LEN_BITS), .LG_DEPTH(`LG_L1I_NUM_SETS)) 
   insn_array1 (
	   .clk(clk),
	   .rd_addr(t_cache_idx),
		.wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
		.wr_data({mem_rsp_load_data[127:96],
			  mem_rsp_load_data[95:64],
			  mem_rsp_load_data[63:32], 
			  mem_rsp_load_data[31:0]}),
		.wr_en(mem_rsp_valid & (r_reload == 1'b1)),
		.rd_data(w_array_out1)
		);

   
   wire [2:0] w_pd0, w_pd1, w_pd2, w_pd3;
   predecode pd0 (.insn(mem_rsp_load_data[31:0]),   .pd(w_pd0));
   predecode pd1 (.insn(mem_rsp_load_data[63:32]),  .pd(w_pd1));
   predecode pd2 (.insn(mem_rsp_load_data[95:64]),  .pd(w_pd2));
   predecode pd3 (.insn(mem_rsp_load_data[127:96]), .pd(w_pd3));   
   
   
   ram1r1w #(.WIDTH(3*WORDS_PER_CL), .LG_DEPTH(`LG_L1I_NUM_SETS))
   pd_data0 (
	    .clk(clk),
	    .rd_addr(t_cache_idx),
	    .wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
	    .wr_data({w_pd3,w_pd2,w_pd1,w_pd0}),
	    .wr_en(mem_rsp_valid & (r_reload == 1'b0)),
	    .rd_data(w_jump_out0)
	    );

   ram1r1w #(.WIDTH(3*WORDS_PER_CL), .LG_DEPTH(`LG_L1I_NUM_SETS))
   pd_data1 (
	    .clk(clk),
	    .rd_addr(t_cache_idx),
	    .wr_addr(r_mem_req_addr[IDX_STOP-1:IDX_START]),
	    .wr_data({w_pd3,w_pd2,w_pd1,w_pd0}),
	    .wr_en(mem_rsp_valid & (r_reload == 1'b1)),
	    .rd_data(w_jump_out1)
	    );
	    
	     
   always_comb
     begin
	n_spec_rs_tos = r_spec_rs_tos;
	if(t_init_rsb)
	  begin
	     n_spec_rs_tos = r_spec_rs_tos + 'd1;
	  end
	else if(n_restart_ack)
	  begin
	     n_spec_rs_tos = r_arch_rs_tos;
	  end
	else if(t_is_call)
	  begin
	     n_spec_rs_tos = r_spec_rs_tos - 'd1;
	  end
	else if(t_is_ret)
	  begin
	     n_spec_rs_tos = r_spec_rs_tos + 'd1;
	  end
     end

   always_ff@(posedge clk)
     begin
	if(t_init_rsb)
	  begin
	     r_spec_return_stack[r_spec_rs_tos] <= 64'd0;
	  end
	else if(t_is_call)
	  begin
	     r_spec_return_stack[r_spec_rs_tos] <= t_ret_pc;
	  end
	else if(n_restart_ack)
	  begin
	     r_spec_return_stack <= r_arch_return_stack;
	  end
     end // always_ff@ (posedge clk)
   
   always_ff@(posedge clk)
     begin
	if(t_init_rsb)
	  begin
	     r_arch_return_stack[r_arch_rs_tos] <= 64'd0;
	  end
	else if(retire_reg_valid && retire_valid && retired_call)
	  begin
	     r_arch_return_stack[r_arch_rs_tos] <= retire_reg_data;
	  end
     end
   always_comb
     begin
	n_arch_rs_tos = r_arch_rs_tos;
	if(t_init_rsb)
	  begin
	     n_arch_rs_tos = r_arch_rs_tos + 'd1;
	  end
	else if(retire_valid && retired_call)
	  begin
	     n_arch_rs_tos = r_arch_rs_tos - 'd1;
	  end
	else if(retire_valid && retired_ret)
	  begin
	     n_arch_rs_tos = r_arch_rs_tos + 'd1;
	  end
     end

   always_comb
     begin
	n_spec_gbl_hist = r_spec_gbl_hist;
	if(n_restart_ack)
	  begin
	     n_spec_gbl_hist = n_arch_gbl_hist;
	  end
	else if(t_update_spec_hist)
	  begin
	     n_spec_gbl_hist = {r_spec_gbl_hist[`GBL_HIST_LEN-2:0], t_take_br};
	  end
     end // always_comb


   always_comb
     begin
	n_arch_gbl_hist = r_arch_gbl_hist;
	if(branch_pc_valid)
	  begin
	     n_arch_gbl_hist = {r_arch_gbl_hist[`GBL_HIST_LEN-2:0], took_branch};
	  end
     end

    //   begin
    // 	if(n_restart_ack)
    // 	  begin
    // 	     $display("restart pc %x, paging enabled %b, r_state = %d, n_state = %d", 
    // 		      n_pc, paging_active, r_state, n_state);
    // 	  end
    //   end
   
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin	  
	     r_tlb_miss <= 1'b0;
	     r_state <= INITIALIZE;
	     r_reload <= 1'b0;
	     r_page_fault <= 1'b0;
	     r_init_pht_idx <= 'd0;
	     r_pc <= 'd0;
	     r_miss_pc <= 'd0;
	     r_cache_pc <= 'd0;
	     r_restart_ack <= 1'b0;
	     r_cache_idx <= 'd0;
	     r_cache_tag <= 'd0;
	     r_req <= 1'b0;
	     r_mem_req_valid <= 1'b0;
	     r_mem_req_addr <= 'd0;
	     r_fq_head_ptr <= 'd0;
	     r_fq_next_head_ptr <= 'd1;
	     r_fq_next_tail_ptr <= 'd1;
	     r_fq_next3_tail_ptr <= 'd1;
	     r_fq_next4_tail_ptr <= 'd1;
	     r_fq_tail_ptr <= 'd0;
	     r_restart_req <= 1'b0;
	     r_flush_req <= 1'b0;
	     r_flush_complete <= 1'b0;
	     r_spec_rs_tos <= RETURN_STACK_ENTRIES-1;
	     r_arch_rs_tos <= RETURN_STACK_ENTRIES-1;
	     r_arch_gbl_hist <= 'd0;
	     r_spec_gbl_hist <= 'd0;
	     r_cache_hits <= 'd0;
	     r_cache_accesses <= 'd0;
	     r_resteer_bubble <= 1'b0;
	  end
	else
	  begin
	     r_tlb_miss <= n_tlb_miss;
	     r_state <= n_state;
	     r_reload <= n_reload;
	     r_page_fault <= n_page_fault;
	     r_init_pht_idx <= n_init_pht_idx;
	     r_pc <= n_pc;
	     r_miss_pc <= n_miss_pc;
	     r_cache_pc <= n_cache_pc;
	     r_restart_ack <= n_restart_ack;
	     r_cache_idx <= t_cache_idx;
	     r_cache_tag <= t_cache_tag;	     
	     r_req <= n_req;
	     r_mem_req_valid <= n_mem_req_valid;
	     r_mem_req_addr <= n_mem_req_addr;
	     r_fq_head_ptr <= t_clear_fq ? 'd0 : n_fq_head_ptr;
	     r_fq_next_head_ptr <= t_clear_fq ? 'd1 : n_fq_next_head_ptr;
	     r_fq_next_tail_ptr <= t_clear_fq ? 'd1 : n_fq_next_tail_ptr;
	     r_fq_next3_tail_ptr <= t_clear_fq ? 'd2 : n_fq_next3_tail_ptr;
	     r_fq_next4_tail_ptr <= t_clear_fq ? 'd3 : n_fq_next4_tail_ptr;
	     r_fq_tail_ptr <= t_clear_fq ? 'd0 : n_fq_tail_ptr;
	     r_restart_req <= n_restart_req;
	     r_flush_req <= n_flush_req;
	     r_flush_complete <= n_flush_complete;
	     r_spec_rs_tos <= n_spec_rs_tos;
	     r_arch_rs_tos <= n_arch_rs_tos;
	     r_arch_gbl_hist <= n_arch_gbl_hist;
	     r_spec_gbl_hist <= n_spec_gbl_hist;
	     r_cache_hits <= n_cache_hits;
	     r_cache_accesses <= n_cache_accesses;
	     r_resteer_bubble <= n_resteer_bubble;	     
	  end
     end
   
endmodule
