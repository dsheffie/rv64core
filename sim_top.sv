
import "DPI-C" pure function longint read_mem64(longint addr);
import "DPI-C" function void write_mem64(longint addr, longint data);
import "DPI-C" function void load_mem();


module sim_top();
   reg clk;
   reg reset;
   reg r_resume, n_resume;
   wire w_ready_for_resume, w_mem_req_valid;
   wire [3:0] w_core_state, w_l1i_state, w_l1d_state;
   wire [63:0] w_mem_req_addr;
   wire [3:0] w_mem_req_opcode;
   wire [127:0] w_mem_req_store_data;
   wire w_retire_valid, w_retire_two_valid;
   wire [63:0] w_retire_pc, w_retire_two_pc;
   logic [127:0] n_data, r_data;
   logic 	n_ack, r_ack;
   logic [31:0] n_last_cnt, r_last_cnt;
   
  //reg [63:0] r_mem [0:(1<<29)-1];
    
   initial
  begin
     load_mem();
     //$readmemh("/home/dsheffie/linux.mem", r_mem);

    clk = 1'b0;
    reset = 1'b1;
    #1000
    reset = 1'b0;
  end
  
  always
    clk = #5 !clk;
  
  reg [63:0] r_cycles;
   
  
  
  always_ff@(posedge clk)
  begin
     r_resume <= reset ? 1'b0 : n_resume;
     r_cycles <= reset ? 64'd0 : (r_cycles + 64'd1);
     r_ack <= reset ? 1'b0 : n_ack;
     r_data <= reset ? 64'd0 : n_data;
     r_last_cnt <= reset ? 'd0 : n_last_cnt;
  end

   always_comb
     begin
	if(w_retire_valid || w_retire_two_valid)
	  begin
	     n_last_cnt = 'd0;
	  end
	else
	  begin
	     n_last_cnt = r_last_cnt + 'd1;
	  end
     end // always_comb
  
   always_ff@(negedge clk)
     begin
	if(w_retire_valid) $display("retire port a %x at %d", w_retire_pc, r_cycles);
	if(w_retire_two_valid) $display("retire port b %x at %d", w_retire_two_pc, r_cycles);
	$display("core state %d, l1i state %d, l2d state %d", 
		 w_core_state, w_l1i_state, w_l1d_state);
     end
	
  
  
  always_comb
    begin
       n_data = r_data;
       n_ack = 1'b0;
       n_resume = w_ready_for_resume;
       if(w_mem_req_valid)
	 begin
	    $display("memory request for addr %x", w_mem_req_addr);
	    if(w_mem_req_opcode == 'd4)
	      begin
		 n_data = {read_mem64(w_mem_req_addr + 'd8), read_mem64(w_mem_req_addr)};
		 n_ack = 1'b1;
	      end
	    else if(w_mem_req_opcode == 'd7)
	      begin
		 $stop();
	      end
	 end
    end // always_comb
   
  
  
  
 core_l1d_l1i 
   uut(
       .clk(clk), 
       .reset(reset),
       .syscall_emu(1'b0),
       .core_state(w_core_state),
       .l1i_state(w_l1i_state),
       .l1d_state(w_l1d_state),
       .n_inflight(),
       .memq_empty(),
       .took_exc(),
       .paging_active(),
       .page_table_root(),
       .extern_irq(1'b0),
       .in_flush_mode(),
       .resume(r_resume),
       .resume_pc(64'h1000),
       .ready_for_resume(w_ready_for_resume),
       .mem_req_valid(w_mem_req_valid), 
       .mem_req_addr(w_mem_req_addr), 
       .mem_req_store_data(w_mem_req_store_data),
       .mem_req_opcode(w_mem_req_opcode),
       .mem_rsp_valid(r_ack),
       .mem_rsp_load_data(r_data),
       .alloc_valid(),
       .alloc_two_valid(),
       .iq_one_valid(),
       .iq_none_valid(),
       .in_branch_recovery(),
       .retire_reg_ptr(),
       .retire_reg_data(),
       .retire_reg_valid(),
       .retire_reg_two_ptr(),
       .retire_reg_two_data(),
       .retire_reg_two_valid(),
       .retire_valid(w_retire_valid),
       .retire_two_valid(w_retire_two_valid),
       .rob_empty(),
       .retire_pc(w_retire_pc),
       .retire_two_pc(w_retire_two_pc),
       .branch_pc(),
       .branch_pc_valid(),		    
       .branch_fault(),
       .l1i_cache_accesses(),
       .l1i_cache_hits(),
       .l1d_cache_accesses(),
       .l1d_cache_hits(),
       .l2_cache_accesses(),
       .l2_cache_hits(),
       .monitor_ack(1'b0),
       .got_break(),
       .got_ud(),
       .got_bad_addr(),
       .got_monitor(),
       .inflight(),
       .epc()
       );
   
   
   
 
 
endmodule
