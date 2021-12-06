`include "machine.vh"

module wrap(clk, 
	    reset,
	    resume,
	    resume_pc,
	    ready_for_resume,
	    is_write,
	    addr,
	    d_out,
	    d_out_valid,
	    d_in,
	    d_in_valid,
	    //retire_reg_ptr,
	    //retire_reg_data,
	    //retire_reg_valid,
	    //retire_valid,
	    monitor_req_reason,
	    monitor_req_valid,
	    monitor_rsp_valid,
	    monitor_rsp_data_valid,
	    monitor_rsp_data,
	    got_break,
	    got_syscall,
	    got_ud);

   input logic clk;
   input logic reset;
   input logic resume;
   input logic [(`M_WIDTH-1):0] resume_pc;
   output logic 		ready_for_resume;

   localparam LG_D_WIDTH = 4;
   
   localparam D_WIDTH = 1 << LG_D_WIDTH;
   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);
   localparam LG_N_WORDS = (`LG_L1D_CL_LEN + 3) - LG_D_WIDTH;
   localparam N_WORDS = 1 << LG_N_WORDS;
   
   output logic 		is_write;
   output logic [(`M_WIDTH-1):0] addr;
   output logic [D_WIDTH-1:0] 	 d_out;
   output logic 		 d_out_valid;
   
   input logic [D_WIDTH-1:0] 	 d_in;
   input logic 			 d_in_valid;
   
  
   

   //output logic [4:0] 			  retire_reg_ptr;
   //output logic [(`M_WIDTH-1):0] 	  retire_reg_data;
   //output logic 			  retire_reg_valid;
   //output logic 			  retire_valid;

   output logic [15:0] 			  monitor_req_reason;
   output logic 			  monitor_req_valid;
   input logic 				  monitor_rsp_valid;
   input logic 				  monitor_rsp_data_valid;
   input logic [(`M_WIDTH-1):0] 	  monitor_rsp_data;
   
   output logic 				  got_break;
   output logic 				  got_syscall;
   output logic 				  got_ud;


    /* these signals get folded */
   logic 			t_mem_req_ack;
   
   logic 			mem_req_valid;
   logic [`M_WIDTH-1:0] 	mem_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] 	mem_req_store_data;
   logic [4:0] 			   mem_req_opcode;
   
   logic 			   mem_rsp_valid;
   logic [L1D_CL_LEN_BITS-1:0] 	   mem_rsp_load_data;
   logic [4:0] 			   mem_rsp_opcode;

   typedef enum logic [1:0] {
			     IDLE = 'd0,
			     WRITE = 'd1,
			     READ = 'd2		    
   } state_t;

   state_t n_state, r_state;
   logic [L1D_CL_LEN_BITS-1:0] r_buf, n_buf, t_buf;
   logic 			   r_is_write, n_is_write;
   logic [ (`M_WIDTH-1):0] 	   r_addr, n_addr;
   logic [4:0] 			   r_mem_req_opcode, n_mem_req_opcode;
   logic [LG_N_WORDS:0] 	   r_cnt, n_cnt;
   logic 			   n_valid, r_valid;
   logic 			   r_mem_rsp_valid,n_mem_rsp_valid;
   
   logic [D_WIDTH-1:0] 		   t_buf_rd[N_WORDS-1:0];

   
  always_comb
	begin
	   t_buf = r_buf;
	   if(d_in_valid)
	     begin
		if(r_cnt == 'd0)
		  begin
		     t_buf[15:0] = d_in;
		  end
		else if(r_cnt == 'd1)
		  begin
		     t_buf[31:16] = d_in;		     
		  end
		else if(r_cnt == 'd2)
		  begin
		     t_buf[47:32] = d_in;		     
		  end
		else if(r_cnt == 'd3)
		  begin
		     t_buf[63:48] = d_in;		     
		  end
		else if(r_cnt == 'd4)
		  begin
		     t_buf[79:64] = d_in;		     
		  end
		else if(r_cnt == 'd5)
		  begin
		     t_buf[95:80] = d_in;
		  end
		else if(r_cnt == 'd6)
		  begin
		     t_buf[111:96] = d_in;		     
		  end
		else
		  begin
		     t_buf[127:112] = d_in;		     
		  end						 
	     end
	end // always_comb
   
   
   generate
      genvar 			   i;
      for(i = 0; i < N_WORDS; i=i+1)
	begin
	   assign t_buf_rd[i] = r_buf[((i+1)*D_WIDTH)-1:i*D_WIDTH];
	end
   endgenerate


   
   assign is_write = r_is_write;
   assign addr = r_addr;
   assign d_out = t_buf_rd[r_cnt];
   assign d_out_valid = r_valid;
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_buf <= 'd0;
	     r_state <= IDLE;
	     r_is_write <= 1'b0;
	     r_addr <= 'd0;
	     r_mem_req_opcode <= 'd0;
	     r_cnt <= 'd0;
	     r_valid <= 1'b0;
	     r_mem_rsp_valid <= 1'b0;	     
	  end
	else
	  begin
	     r_buf <= n_buf;
	     r_state <= n_state;
	     r_is_write <= n_is_write;
	     r_addr <= n_addr;
	     r_mem_req_opcode <= n_mem_req_opcode;
	     r_cnt <= n_cnt;
	     r_valid <= n_valid;
	     r_mem_rsp_valid <= n_mem_rsp_valid;	     
	  end
     end // always_ff@ (posedge clk)

   always_comb
     begin
	n_state = r_state;
	n_buf = r_buf;
	n_is_write = r_is_write;
	n_addr = r_addr;
	n_mem_req_opcode = r_mem_req_opcode;
	t_mem_req_ack = 1'b1;
	n_cnt = r_cnt;	
	n_valid = 1'b0;
	n_mem_rsp_valid = 1'b0;
	
	case(r_state)
	  IDLE:
	    begin
	       if(mem_req_valid)
		 begin
		    n_buf = mem_req_store_data;
		    n_mem_req_opcode = mem_req_opcode;
		    t_mem_req_ack = 1'b1;
		    n_cnt = 'd0;		    
		    if(mem_req_opcode == MEM_LW)
		      begin
			 n_is_write = 1'b0;
			 n_state = READ;
		      end
		    else
		      begin
			 n_is_write = 1'b1;
			 n_state = WRITE;
		      end
		 end
	    end // case: IDLE
	  READ:
	    begin
	       n_buf = t_buf;
	       if(d_in_valid)
		 begin
		    n_cnt = r_cnt + 'd1;
		    n_addr = r_addr + (1<<(LG_D_WIDTH-3));
		    if(r_cnt == (N_WORDS-1))
		      begin
			 n_state = IDLE;
			 n_mem_rsp_valid = 1'b1;
		      end
		 end
	    end
	  WRITE:
	    begin
	       n_cnt = r_cnt + 'd1;
	       n_addr = r_addr + (1<<(LG_D_WIDTH-3));
	       n_valid = 1'b1;
	       if(r_cnt == (N_WORDS-1))
		 begin
		    n_state = IDLE;
		    n_mem_rsp_valid = 1'b1;
		 end
	    end
	  default:
	    begin
	    end
	endcase // case (r_state)
	
     end
   

   core_l1d_l1i mips(
		     .clk(clk),
		     .reset(reset),
		     .resume(resume),
		     .resume_pc(resume_pc),
		     .ready_for_resume(ready_for_resume),

		     .mem_req_ack(t_mem_req_ack),
		     
		     .mem_req_valid(mem_req_valid), 
		     .mem_req_addr(mem_req_addr), 
		     .mem_req_store_data(mem_req_store_data), 
		     .mem_req_opcode(mem_req_opcode),
		     
		     .mem_rsp_valid(r_mem_rsp_valid),
		     .mem_rsp_load_data(r_buf),
		     .mem_rsp_tag('d0),
		     .mem_rsp_opcode(r_mem_req_opcode),
		     
		     .retire_reg_ptr(retire_reg_ptr),
		     .retire_reg_data(retire_reg_data),
		     .retire_reg_valid(retire_reg_valid),
		     .retire_valid(retire_valid),
		     .monitor_req_reason(monitor_req_reason),
		     .monitor_req_valid(monitor_req_valid),
		     .monitor_rsp_valid(monitor_rsp_valid),
		     .monitor_rsp_data_valid(monitor_rsp_data_valid),
		     .monitor_rsp_data(monitor_rsp_data),
		     .got_break(got_break),
		     .got_syscall(got_syscall),
		     .got_ud(got_ud)
		     );
   		     
endmodule	    
