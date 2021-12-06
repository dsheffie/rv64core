`include "uop.vh"

module mul(clk,reset,opcode,go,
	   src_A,
	   src_B,
	   src_hilo,
	   rob_ptr_in,
	   gpr_prf_ptr_in,
	   hilo_prf_ptr_in,
	   y,complete,
	   rob_ptr_out,
	   gpr_prf_ptr_val_out,
	   gpr_prf_ptr_out,
	   hilo_prf_ptr_val_out,
	   hilo_prf_ptr_out);
   
   input logic clk;
   input logic reset;
   input opcode_t opcode;   
   input logic go;
   
   input logic [31:0] src_A;
   input logic [31:0] src_B;
   input logic [63:0] src_hilo;
   
   input logic [`LG_ROB_ENTRIES-1:0] rob_ptr_in;
   input logic [`LG_PRF_ENTRIES-1:0] gpr_prf_ptr_in;
   input logic [`LG_HILO_PRF_ENTRIES-1:0] hilo_prf_ptr_in;
   
   
   output logic [63:0] 			  y;
   output logic 			  complete;
   output logic [`LG_ROB_ENTRIES-1:0] rob_ptr_out;
   output logic 		      gpr_prf_ptr_val_out;
   output logic [`LG_PRF_ENTRIES-1:0] gpr_prf_ptr_out;
   output logic 			   hilo_prf_ptr_val_out;
   output logic [`LG_HILO_PRF_ENTRIES-1:0] hilo_prf_ptr_out;
   
   logic [63:0] 			   r_mul[`MUL_LAT:0];
   logic [`MUL_LAT:0] 			   r_complete;
   logic [`MUL_LAT:0] 			   r_do_madd;
   logic [`MUL_LAT:0] 			   r_do_msub;
   logic [`MUL_LAT:0] 			   r_hilo_val;
   logic [`LG_HILO_PRF_ENTRIES-1:0] 	   r_hilo_ptr[`MUL_LAT:0];
   logic [`MUL_LAT:0] 			   r_gpr_val;
   logic [`LG_PRF_ENTRIES-1:0] 	   r_gpr_ptr[`MUL_LAT:0];
   logic [63:0] 		   r_madd[`MUL_LAT:0];
   logic [`LG_ROB_ENTRIES-1:0] 	   r_rob_ptr[`MUL_LAT:0];
  
   
   logic [63:0] 			   t_mul;

   assign complete = r_complete[`MUL_LAT];
   assign rob_ptr_out = r_rob_ptr[`MUL_LAT];
   assign gpr_prf_ptr_val_out = r_gpr_val[`MUL_LAT];
   assign gpr_prf_ptr_out = r_gpr_ptr[`MUL_LAT];
   
   assign hilo_prf_ptr_val_out = r_hilo_val[`MUL_LAT];
   assign hilo_prf_ptr_out = r_hilo_ptr[`MUL_LAT];

   always_comb
     begin
	y = r_mul[`MUL_LAT];
	if(r_do_madd[`MUL_LAT])
	  begin
	     y = r_mul[`MUL_LAT] + r_madd[`MUL_LAT];
	  end
	else if(r_do_msub[`MUL_LAT])
	  begin
	     y = r_mul[`MUL_LAT] - r_madd[`MUL_LAT];
	  end
     end
   // always_ff@(negedge clk)
   //   begin
   // 	if(go)
   // 	  $display("exec MUL = %b, MULT = %b, MULTU = %b, MADD = %b", 
   // 		   opcode==MUL, opcode==MULT, opcode==MULTU, opcode==MADD);
   //   end
   
   always_comb
     begin
	if(opcode == MULTU)
	  begin
	     t_mul = src_A * src_B;
	  end
	else
	  begin
	     t_mul = $signed(src_A) * $signed(src_B);
	  end
     end

   //always_comb
   //begin
   //if(go) $display("starting multiplier\n");
   //end
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i <= `MUL_LAT; i=i+1)
	       begin
		  r_mul[i] <= 'd0;
		  r_rob_ptr[i] <= 'd0;
		  r_gpr_ptr[i] <= 'd0;
		  r_hilo_ptr[i] <= 'd0;
		  r_madd[i] <= 'd0;
	       end
	     r_complete <= 'd0;
	     r_do_madd <= 'd0;
	     r_do_msub <= 'd0;
	     r_gpr_val <= 'd0;
	     r_hilo_val <= 'd0;
	  end
	else
	  begin
	     for(integer i = 0; i <= `MUL_LAT; i=i+1)
	       begin
		  if(i == 0)
		    begin
		       r_mul[0] <= t_mul;
		       r_do_madd[0] <= go & (opcode == MADD);
		       r_do_msub[0] <= go & (opcode == MSUB);
		       r_complete[0] <= go;
		       r_rob_ptr[0] <= rob_ptr_in;
		       r_gpr_val[0] <= go && (opcode == MUL);
		       r_hilo_val[0] <= go && (opcode != MUL);
		       r_gpr_ptr[0] <= gpr_prf_ptr_in;
		       r_hilo_ptr[0] <= hilo_prf_ptr_in;
		       r_madd[0] <= src_hilo;
		    end
		  else
		    begin
		       r_mul[i] <= r_mul[i-1];
		       r_do_madd[i] <= r_do_madd[i-1];
		       r_do_msub[i] <= r_do_msub[i-1];
		       r_complete[i] <= r_complete[i-1];
		       r_rob_ptr[i] <= r_rob_ptr[i-1];
		       r_gpr_val[i] <= r_gpr_val[i-1];
		       r_hilo_val[i] <= r_hilo_val[i-1];
		       r_gpr_ptr[i]  <= r_gpr_ptr[i-1];
		       r_hilo_ptr[i] <= r_hilo_ptr[i-1];
		       r_madd[i] <= r_madd[i-1];
		    end
	       end
	  end
     end // always_ff@ (posedge clk)

   
endmodule
