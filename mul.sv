`include "machine.vh"

module ff(q,d,clk);
   parameter N = 1;
   input logic [N-1:0] d;
   input logic 	       clk;
   output logic [N-1:0] q;
   always_ff@(posedge clk)
     begin
	q <= d;
     end // always_ff@ (posedge clk)
endmodule // dff

module mul(clk,
	   reset,
	   is_signed,
	   is_high,
	   go,
	   is_64b_mul,
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
   input logic is_64b_mul;
   
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
   logic [`MUL_LAT:0]		      r_is_64b;
   logic [`MUL_LAT:0] 		      r_complete;
   logic [`MUL_LAT:0] 			   r_gpr_val;
   logic [`LG_PRF_ENTRIES-1:0] 		   r_gpr_ptr[`MUL_LAT:0];
   logic [`LG_ROB_ENTRIES-1:0] 		   r_rob_ptr[`MUL_LAT:0];
  

   assign complete = r_complete[`MUL_LAT];
   assign rob_ptr_out = r_rob_ptr[`MUL_LAT];
   
   assign prf_ptr_val_out = r_gpr_val[`MUL_LAT];
   assign prf_ptr_out = r_gpr_ptr[`MUL_LAT];

   logic [(`M_WIDTH*2)-1:0] 			   t_mul;
   logic [(`M_WIDTH*2)-1:0] 			   r_mul[`MUL_LAT:0];

   
   wire [(`M_WIDTH*2)-1:0] 			   w_sext32_A = {{(`M_WIDTH+32){src_A[31]}}, src_A[31:0]};
   wire [(`M_WIDTH*2)-1:0] 			   w_sext32_B = {{(`M_WIDTH+32){src_B[31]}}, src_B[31:0]};

    wire [(`M_WIDTH*2)-1:0]			   w_sext64_A = {{`M_WIDTH{src_A[`M_WIDTH-1]}}, src_A};
   wire [(`M_WIDTH*2)-1:0] 			   w_sext64_B = {{`M_WIDTH{src_B[`M_WIDTH-1]}}, src_B};

   wire [(`M_WIDTH*2)-1:0]			   w_sext_A = is_64b_mul ? w_sext64_A : w_sext32_A;
   wire [(`M_WIDTH*2)-1:0]			   w_sext_B = is_64b_mul ? w_sext64_B : w_sext32_B;

   wire [`M_WIDTH-1:0]				   w_zext_A = is_64b_mul ? src_A : {32'd0, src_A[31:0]};
   wire [`M_WIDTH-1:0]				   w_zext_B = is_64b_mul ? src_B : {32'd0, src_B[31:0]};

   wire [63:0]					   w_mul = src_A[31:0] * src_B[31:0];
   
   always_ff@(negedge clk)
     begin
	if(go)
	  begin
	     $display("srcA = %x, srcB = %x, t_mul = %x, w_mul = %x", src_A, src_B, t_mul, w_mul);
	  end
     end
				   
   always_comb
     begin
	t_mul = is_signed ? ($signed(w_sext_A) * $signed(w_sext_B)) 
	  : w_zext_A * w_zext_B;
	if(r_is_high[`MUL_LAT])
	  begin
	     y = r_is_64b[`MUL_LAT] ? r_mul[`MUL_LAT][((2*`M_WIDTH)-1):`M_WIDTH] : {{32{r_mul[`MUL_LAT][63]}}, r_mul[`MUL_LAT][63:32]};
	  end
	else
	  begin
	     y = r_mul[`MUL_LAT][`M_WIDTH-1:0];	     
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
	     r_is_64b <= 'd0;
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
		       r_is_64b[0] <= is_64b_mul;
		    end
		  else
		    begin
		       r_complete[i] <= r_complete[i-1];
		       r_is_high[i] <= r_is_high[i-1];
		       r_rob_ptr[i] <= r_rob_ptr[i-1];
		       r_gpr_val[i] <= r_gpr_val[i-1];
		       r_gpr_ptr[i] <= r_gpr_ptr[i-1];
		       r_is_64b[i] <= r_is_64b[i-1];
		    end
	       end
	  end
     end // always_ff@ (posedge clk)

   
endmodule
