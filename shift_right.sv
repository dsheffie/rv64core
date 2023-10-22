
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
   
   
endmodule // shftRight
