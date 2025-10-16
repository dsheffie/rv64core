`include "machine.vh"

`ifdef VERILATOR
import "DPI-C" function void check_fp32_mul(input int a, input int b, input int y);
import "DPI-C" function void check_fp32_add(input int a, input int b, input int y);
`endif


module mul(clk,
	   reset,
	   is_signed,
	   is_high,
	   go,
	   is_mulw,
	   is_fp_add,
	   is_fp_sub,
	   is_fp_mul,
	   src_A,
	   src_B,
	   rob_ptr_in,
	   prf_ptr_in,
	   y,
	   complete,
	   rob_ptr_out,
	   prf_ptr_val_out,
	   prf_ptr_out);
   
   input logic clk;
   input logic reset;
   input logic is_signed;
   input logic is_high;
   input logic go;
   input logic is_mulw;
   input logic is_fp_add;
   input logic is_fp_sub;
   input logic is_fp_mul;   
   
   input logic [`M_WIDTH-1:0] src_A;
   input logic [`M_WIDTH-1:0] src_B;
   
   input logic [`LG_ROB_ENTRIES-1:0] rob_ptr_in;
   input logic [`LG_PRF_ENTRIES-1:0] prf_ptr_in;
   
   
   output logic [`M_WIDTH-1:0] 	     y;
   output logic 			  complete;
   output logic [`LG_ROB_ENTRIES-1:0] 	  rob_ptr_out;
   output logic 			  prf_ptr_val_out;
   output logic [`LG_PRF_ENTRIES-1:0] prf_ptr_out;

   logic [`MUL_LAT:0] 		      r_is_high;
   logic [`MUL_LAT:0]		      r_is_mulw;
   logic [`MUL_LAT:0] 		      r_complete;
   logic [`MUL_LAT:0]		      r_is_fp_add;
   logic [`MUL_LAT:0]		      r_is_fp_sub;
   logic [`MUL_LAT:0]		      r_is_fp_mul;   
   
   logic [`MUL_LAT:0] 			   r_gpr_val;
   logic [`LG_PRF_ENTRIES-1:0] 		   r_gpr_ptr[`MUL_LAT:0];
   logic [`LG_ROB_ENTRIES-1:0] 		   r_rob_ptr[`MUL_LAT:0];
  

   assign complete = r_complete[`MUL_LAT];
   assign rob_ptr_out = r_rob_ptr[`MUL_LAT];
   
   assign prf_ptr_val_out = r_gpr_val[`MUL_LAT];
   assign prf_ptr_out = r_gpr_ptr[`MUL_LAT];

   logic [(`M_WIDTH*2)-1:0] 			   t_mul;
   logic [(`M_WIDTH*2)-1:0] 			   r_mul[`MUL_LAT:0];


   wire [(`M_WIDTH*2)-1:0]			   w_sext_A = {{`M_WIDTH{src_A[`M_WIDTH-1]}}, src_A};
   wire [(`M_WIDTH*2)-1:0] 			   w_sext_B = {{`M_WIDTH{src_B[`M_WIDTH-1]}}, src_B};

   wire [63:0]					   w_mulw = {{32{r_mul[`MUL_LAT][31]}}, r_mul[`MUL_LAT][31:0]};
	
   wire [31:0]					   w_fp_mul, w_fp_add;
   
   fp_mul #(.W(32), .MUL_LAT(`MUL_LAT+1)) sp_mul0
     (
      .y(w_fp_mul),
      .clk(clk),
      .a(src_A[31:0]),
      .b(src_B[31:0]),
      .en(1'b1)
      );

   fp_add #(.W(32), .ADD_LAT(`MUL_LAT+1)) sp_add0
     (
      .y(w_fp_add),
      .clk(clk),
      .a(src_A[31:0]),
      .b(src_B[31:0]),
      .sub(is_fp_sub),
      .en(1'b1)
      );


`ifdef VERILATOR
   logic [31:0]			      r_srcA[`MUL_LAT:0];
   logic [31:0]			      r_srcB[`MUL_LAT:0];      
   always_ff@(posedge clk)
     begin
	for(integer i = 0; i <= `MUL_LAT; i=i+1)
	  begin
	     if(i == 0)
	       begin
		  r_srcA[0] <= src_A[31:0];
		  r_srcB[0] <= src_B[31:0];
	       end
	     else
	       begin
		  r_srcA[i] <= r_srcA[i-1];
		  r_srcB[i] <= r_srcB[i-1];
	       end
	  end
     end // always_ff@ (posedge clk)
		
   always_ff@(negedge clk)
     begin
	if(complete & r_is_fp_mul[`MUL_LAT])
	  begin
	     //$display("src_A = %b", r_srcA[`MUL_LAT]);
	     //$display("src_B = %b", r_srcB[`MUL_LAT]);	     
             //$display("w_mul = %x", y[31:0]);
	     check_fp32_mul(r_srcA[`MUL_LAT], r_srcB[`MUL_LAT], y[31:0]);
	  end
	if(complete & r_is_fp_add[`MUL_LAT])
	  begin
	     check_fp32_add(r_srcA[`MUL_LAT], r_srcB[`MUL_LAT], y[31:0]);
	  end
     end // always_ff@ (negedge clk)

`endif
   
			   
   always_comb
     begin
	t_mul = is_signed ? ($signed(w_sext_A) * $signed(w_sext_B)) 
	  : src_A * src_B;
	
	if(r_is_high[`MUL_LAT]			   )
	  begin
	     y = r_mul[`MUL_LAT][(`M_WIDTH*2)-1:`M_WIDTH];
	  end
	else if(r_is_fp_add[`MUL_LAT])
	  begin
	     y = {32'd0, w_fp_add};
	  end
	else if(r_is_fp_sub[`MUL_LAT])
	  begin
	     y = {32'd0, w_fp_add};
	  end
	else if(r_is_fp_mul[`MUL_LAT])
	  begin
	     y = {32'd0, w_fp_mul};
	  end	
	else
	  begin
	     y = r_is_mulw[`MUL_LAT] ? w_mulw : r_mul[`MUL_LAT][`M_WIDTH-1:0];	     
	  end
     end // always_comb
   
   always_ff@(posedge clk)
     begin
	r_mul[0] <= t_mul;
	for(integer i = 1; i <= `MUL_LAT; i=i+1)
	  begin
	     r_mul[i] <= r_mul[i-1];
	  end
     end
      
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i <= `MUL_LAT; i=i+1)
	       begin
		  r_rob_ptr[i] <= 'd0;
		  r_gpr_ptr[i] <= 'd0;
	       end
	     r_complete <= 'd0;
	     r_gpr_val <= 'd0;
	     r_is_high <= 'd0;
	     r_is_mulw <= 'd0;
	     r_is_fp_add <= 'd0;
	     r_is_fp_sub <= 'd0;
	     r_is_fp_mul <= 'd0;   	     
	  end
	else
	  begin
	     for(integer i = 0; i <= `MUL_LAT; i=i+1)
	       begin
		  if(i == 0)
		    begin
		       r_complete[0] <= go;
		       r_is_high[0] <= is_high;
		       r_rob_ptr[0] <= rob_ptr_in;
		       r_gpr_val[0] <= go;
		       r_gpr_ptr[0] <= prf_ptr_in;
		       r_is_mulw[0] <= is_mulw;
		       r_is_fp_add[0] <= is_fp_add;
		       r_is_fp_sub[0] <= is_fp_sub;
		       r_is_fp_mul[0] <= is_fp_mul;
		    end
		  else
		    begin
		       r_complete[i] <= r_complete[i-1];
		       r_is_high[i] <= r_is_high[i-1];
		       r_rob_ptr[i] <= r_rob_ptr[i-1];
		       r_gpr_val[i] <= r_gpr_val[i-1];
		       r_gpr_ptr[i] <= r_gpr_ptr[i-1];
		       r_is_mulw[i] <= r_is_mulw[i-1];
		       r_is_fp_add[i] <= r_is_fp_add[i-1];
		       r_is_fp_sub[i] <= r_is_fp_sub[i-1];
		       r_is_fp_mul[i] <= r_is_fp_mul[i-1];		       
		    end
	       end
	  end
     end // always_ff@ (posedge clk)

   
endmodule
