module fair_sched#(parameter LG_N = 2)(clk, rst, in, y);
   localparam N = 1<<LG_N;
   input logic clk;
   input logic rst;
   input logic [N-1:0] in;
   output logic [LG_N:0] y;

   logic 		 any_valid = |in;
   
   
   logic [LG_N-1:0] 	 r_cnt, n_cnt;

   logic [(2*N)-1:0] 	 t_in2 = {in,in};
   logic [(2*N)-1:0] 	 t_in_shift = t_in2 << r_cnt;
   logic [N-1:0] 	 t_in =  t_in_shift[(2*N)-1:N];
   logic [LG_N:0] 	 t_y;
   
   always_ff@(posedge clk)
     begin
	if(rst)
	  begin
	     r_cnt <= 'd0;
	  end
	else
	  begin
	     r_cnt <= any_valid ? r_cnt + 'd1 : r_cnt;
	  end
     end // always_ff@ (posedge clk)

   find_first_set#(LG_N) f (.in(t_in), .y(t_y));
   logic [LG_N-1:0] t_yy = t_y[LG_N-1:0] - r_cnt;
   
   always_comb
     begin
	y = {(LG_N+1){1'b1}};
	if(any_valid)
	  begin
	     y = {1'b0, t_yy};
	  end
     end

   always_ff@(negedge clk)
     begin
	if(any_valid)
	  begin
	     if(in[y[LG_N-1:0]] == 1'b0)
	       begin
		  $display("input %b, r_cnt %d, t_in %b, t_y = %d, y = %d", in, r_cnt, t_in, t_y, y);
		  $stop();
	       end
	  end
     end
   
endmodule
