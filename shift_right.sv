
/* note - this is a funnel shifter */

`include "machine.vh"

module shift_right#(parameter LG_W=5)(y, is_left, is_signed, data, distance);
   localparam W = 1<<LG_W;
   input logic is_left;
   input logic is_signed;
   input logic [W-1:0] data;
   input logic [LG_W-1:0] distance;
   output [W-1:0] y;
     
   wire w_sb = is_signed ? data[W-1] : 1'b0;
   wire [(2*W)-1:0] w_data = is_left ? {data, {W{1'b0}}} : {{W{w_sb}}, data};

   wire [LG_W:0]    w_inv_dist = (W - {1'b0,distance});
   wire [LG_W:0]    w_distance = is_left ? w_inv_dist[LG_W:0] : {1'b0, distance};

`ifdef FPGA
   wire [(2*W)-1:0] w_shift = w_data >> w_distance;
   assign y = w_shift[W-1:0];
`else   
   /* verilator lint_off UNOPTFLAT */
   wire [(2*W)-1:0] w_shift [LG_W:0];
   assign w_shift[0] = w_distance[0] ? (w_data >> 1) : w_data;
   generate
      for(genvar i = 1; i < (LG_W+1); i = i + 1)
	begin
	   assign w_shift[i] = w_distance[i] ? (w_shift[i-1] >> (1<<i)) : w_shift[i-1];
	end
   endgenerate
   assign y = w_shift[LG_W][W-1:0];
`endif

   // always_comb
   //   begin
   // 	if(is_left)
   // 	  begin
   // 	     $display("y = %x, distance = %d, w_distance = %d, in = %b, w_shift = %b", 
   // 		      y, distance, w_distance, data, w_shift);
   // 	  end
   //   end
   
   // logic 	    sb;
   // logic [31:0]    t_y;
   // always_comb
   //   begin
   // 	t_y = 'd0;
   // 	sb = is_signed ? data[W-1] : 1'b0;
   // 	case(distance)
   // 	  'd0:
   // 	    t_y = data;
   // 	  'd1:
   // 	    t_y = {sb, data[31:1]};
   // 	  'd2:
   // 	    t_y = {{2{sb}}, data[31:2]};
   // 	  'd3:
   // 	    t_y = {{3{sb}}, data[31:3]};
   // 	  'd4:
   // 	    t_y = {{4{sb}}, data[31:4]};
   // 	  'd5:
   // 	    t_y = {{5{sb}}, data[31:5]};
   // 	  'd6:
   // 	    t_y = {{6{sb}}, data[31:6]};
   // 	  'd7:
   // 	    t_y = {{7{sb}}, data[31:7]};
   // 	  'd8:
   // 	    t_y = {{8{sb}}, data[31:8]};
   // 	  'd9:
   // 	    t_y = {{9{sb}}, data[31:9]};
   // 	  'd10:
   // 	    t_y = {{10{sb}}, data[31:10]};
   // 	  'd11:
   // 	    t_y = {{11{sb}}, data[31:11]};
   // 	  'd12:
   // 	    t_y = {{12{sb}}, data[31:12]};
   // 	  'd13:
   // 	    t_y = {{13{sb}}, data[31:13]};
   // 	  'd14:
   // 	    t_y = {{14{sb}}, data[31:14]};
   // 	  'd15:
   // 	    t_y = {{15{sb}}, data[31:15]};
   // 	  'd16:
   // 	    t_y = {{16{sb}}, data[31:16]};
   // 	  'd17:
   // 	    t_y = {{17{sb}}, data[31:17]};
   // 	  'd18:
   // 	    t_y = {{18{sb}}, data[31:18]};
   // 	  'd19:
   // 	    t_y = {{19{sb}}, data[31:19]};
   // 	  'd20:
   // 	    t_y = {{20{sb}}, data[31:20]};
   // 	  'd21:
   // 	    t_y = {{21{sb}}, data[31:21]};
   // 	  'd22:
   // 	    t_y = {{22{sb}}, data[31:22]};
   // 	  'd23:
   // 	    t_y = {{23{sb}}, data[31:23]};
   // 	  'd24:
   // 	    t_y = {{24{sb}}, data[31:24]};
   // 	  'd25:
   // 	    t_y = {{25{sb}}, data[31:25]};
   // 	  'd26:
   // 	    t_y = {{26{sb}}, data[31:26]};
   // 	  'd27:
   // 	    t_y = {{27{sb}}, data[31:27]};
   // 	  'd28:
   // 	    t_y = {{28{sb}}, data[31:28]};
   // 	  'd29:
   // 	    t_y = {{29{sb}}, data[31:29]};
   // 	  'd30:
   // 	    t_y = {{30{sb}}, data[31:30]};
   // 	  'd31:
   // 	    t_y = {{31{sb}}, data[31]};
   // 	endcase // case (distance)
   //   end // always@ (*)

      
   // always_ff@(negedge clk)
   //   begin
   // 	//$display("is_signed %b  y = %x, data>>distance = %x", is_signed, y, t_y);
   // 	if(y != t_y)
   // 	  begin
   // 	     $stop();
   // 	  end
   //   end // always_ff@ (negedge clk)
   
   
endmodule // shftRight
