`include "fp_compare.vh"

module fpu(clk,
	   reset,
	   pc,
	   opcode,
	   start,
	   src_a,
	   src_b,
	   src_c,
	   src_fcr,
	   rob_ptr_in,
	   dst_ptr_in,
	   fcr_ptr_in,
	   fcr_sel,
	   val,
	   cmp_val,
	   y,
	   rob_ptr_out,
	   dst_ptr_out,
	   fcr_ptr_out
	   );

   parameter LG_PRF_WIDTH = 4;
   parameter LG_ROB_WIDTH = 4;
   parameter LG_FCR_WIDTH = 4;
   parameter FPU_LAT = 2;
   
   input logic clk;
   input logic reset;
   input logic [63:0] pc;
   input opcode_t opcode;
   input logic start;
   
   input logic [63:0] src_a;
   input logic [63:0] src_b;
   input logic [63:0] src_c;
   input logic [7:0]  src_fcr;
   
   input logic [LG_ROB_WIDTH-1:0] rob_ptr_in;
   input logic [LG_PRF_WIDTH-1:0] dst_ptr_in;
   input logic [LG_FCR_WIDTH-1:0] fcr_ptr_in;
   input logic [2:0] 		  fcr_sel;
   
   output logic 		  val;
   output logic 		  cmp_val;
   
   output logic [63:0] 		  y;
   output logic [LG_ROB_WIDTH-1:0] rob_ptr_out;
   output logic [LG_PRF_WIDTH-1:0] dst_ptr_out;
   output logic [LG_FCR_WIDTH-1:0] fcr_ptr_out;
   
   logic [31:0] 		   t_sp_adder_result;
   logic [63:0] 		   t_dp_adder_result;
   logic [31:0] 		   t_sp_mult_result;
   logic [63:0] 		   t_dp_mult_result;
      
   logic [FPU_LAT-1:0] 		   r_val;
   logic [LG_PRF_WIDTH-1:0] 	   r_ptr [FPU_LAT-1:0];
   logic [LG_ROB_WIDTH-1:0] 	   r_rob [FPU_LAT-1:0];
   logic [LG_FCR_WIDTH-1:0] 	   r_fcr[FPU_LAT-1:0];
   logic [2:0] 			   r_fcr_sel[FPU_LAT-1:0];
   logic [7:0] 			   r_fcr_reg[FPU_LAT-1:0];
   logic [7:0] 			   fcr_reg;
   opcode_t r_opcode[FPU_LAT-1:0];

   
   assign dst_ptr_out = r_ptr[0];
   assign rob_ptr_out = r_rob[0];
   assign fcr_ptr_out = r_fcr[0];
   assign fcr_reg = r_fcr_reg[0];

   wire 			   w_sp_cmp, w_dp_cmp;
   fp_cmp_t t_sp_cmp_type, t_dp_cmp_type;

   always_comb
     begin
	t_sp_cmp_type = CMP_NONE;
	case(opcode)
	  SP_CMP_LT:
	    begin
	       t_sp_cmp_type = CMP_LT;
	    end
	  SP_CMP_EQ:
	    begin
	       t_sp_cmp_type = CMP_EQ;
	    end
	  SP_CMP_LE:	    
	    begin
	       t_sp_cmp_type = CMP_LE;	       
	    end
	  default:
	    begin
	    end
	endcase
     end // always_comb

   always_comb
     begin
	t_dp_cmp_type = CMP_NONE;
	case(opcode)
	  DP_CMP_LT:
	    begin
	       t_dp_cmp_type = CMP_LT;
	    end
	  DP_CMP_EQ:
	    begin
	       t_dp_cmp_type = CMP_EQ;
	    end
	  DP_CMP_LE:	    
	    begin
	       t_dp_cmp_type = CMP_LE;	       
	    end
	  default:
	    begin
	    end
	endcase
     end // always_comb
   
   
   fp_compare #(.W(32), .D(FPU_LAT)) 
   sp_cmp(.clk(clk),
	  .pc(pc),
	  .a(src_a[31:0]),
	  .b(src_b[31:0]),
	  .start(start && (t_sp_cmp_type != CMP_NONE)),
	  .cmp_type(t_sp_cmp_type), 
	  .y(w_sp_cmp));

   fp_compare #(.W(64), .D(FPU_LAT)) 
   dp_cmp(.clk(clk),
	  .pc(pc),
	  .a(src_a), 
	  .b(src_b),
	  .start(start && (t_dp_cmp_type != CMP_NONE)),
	  .cmp_type(t_dp_cmp_type), 
	  .y(w_dp_cmp));

   
   function logic [63:0] handle_fcr(logic b, logic [2:0] sel, logic [7:0] fcr_reg);
      logic [63:0] 		   y;
      case(sel)
	3'd0:
	  begin
	     y = {56'd0, fcr_reg[7:1], b};
	  end
	3'd1:
	  begin
	     y = {56'd0, fcr_reg[7:2], b, fcr_reg[0]};
	  end
	3'd2:
	  begin
	     y = {56'd0, fcr_reg[7:3], b, fcr_reg[1:0]};
	  end
	3'd3:
	  begin
	     y = {56'd0, fcr_reg[7:4], b, fcr_reg[2:0]};
	  end
	3'd4:
	  begin
	     y = {56'd0, fcr_reg[7:5], b, fcr_reg[3:0]};		      
	  end
	3'd5:
	  begin
	     y = {56'd0, fcr_reg[7:6], b, fcr_reg[4:0]};		     
	  end
	3'd6:
	  begin
	     y = {56'd0, fcr_reg[7], b, fcr_reg[5:0]};		      
	  end
	3'd7:
	  begin
	     y = {56'd0, b, fcr_reg[6:0]};		      
	  end
      endcase // case (sel)
      return y;
   endfunction // handle_fcr
   
   always_comb
     begin
	y = 'd0;
	val = 1'b0;
	cmp_val = 1'b0;
	case(r_opcode[0])
	  SP_ADD:
	    begin
	       y = {32'd0, t_sp_adder_result};
	       val = r_val[0];
	    end
	  SP_SUB:
	    begin
	       y = {32'd0, t_sp_adder_result};
	       val = r_val[0];
	    end
	  DP_ADD:
	    begin
	       y = t_dp_adder_result;
	       val = r_val[0];
	    end
	  DP_SUB:
	    begin
	       y = t_dp_adder_result;
	       val = r_val[0];
	    end
	  SP_MUL:
	    begin
	       y = {32'd0, t_sp_mult_result};
	       val = r_val[0];
	    end
	  DP_MUL:
	    begin
	       y = t_dp_mult_result;
	       val = r_val[0];	       
	    end
	  SP_CMP_LT:
	    begin
	       cmp_val = r_val[0];
	       y = handle_fcr(w_sp_cmp, r_fcr_sel[0], fcr_reg);
	    end
	  SP_CMP_LE:
	    begin
	       cmp_val = r_val[0];
	       y = handle_fcr(w_sp_cmp, r_fcr_sel[0], fcr_reg);
	    end
	  SP_CMP_EQ:
	    begin
	       cmp_val = r_val[0];
	       y = handle_fcr(w_sp_cmp, r_fcr_sel[0], fcr_reg);
	    end
	  DP_CMP_LT:
	    begin
	       cmp_val = r_val[0];
	       y = handle_fcr(w_dp_cmp, r_fcr_sel[0], fcr_reg);
	    end
	  DP_CMP_LE:
	    begin
	       cmp_val = r_val[0];
	       y = handle_fcr(w_dp_cmp, r_fcr_sel[0], fcr_reg);
	    end
	  DP_CMP_EQ:
	    begin
	       cmp_val = r_val[0];
	       y = handle_fcr(w_dp_cmp, r_fcr_sel[0], fcr_reg);
	    end
	  
	  default:
	    begin
	    end
	endcase // case (r_opcode[0])
     end // always_comb

   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_val <= 'd0;	     
	  end
	else
	  begin
	     r_val[FPU_LAT-1] <= start;
	     for(integer i = (FPU_LAT-1); i > 0; i=i-1)
	       begin
		  r_val[i-1] <= r_val[i];
	       end
	  end
     end
   
   always_ff@(posedge clk)
     begin
	r_opcode[FPU_LAT-1] <= opcode;
	r_ptr[FPU_LAT-1] <= dst_ptr_in;
	r_fcr[FPU_LAT-1] <= fcr_ptr_in;
	r_rob[FPU_LAT-1] <= rob_ptr_in;
	r_fcr_sel[FPU_LAT-1] <= fcr_sel;
	r_fcr_reg[FPU_LAT-1] <= src_fcr;
	for(integer i = (FPU_LAT-1); i > 0; i=i-1)
	  begin
	     r_opcode[i-1] <= r_opcode[i];
	     r_ptr[i-1] <= r_ptr[i];
	     r_fcr[i-1] <= r_fcr[i];
	     r_rob[i-1] <= r_rob[i];
	     r_fcr_sel[i-1] <= r_fcr_sel[i];
	     r_fcr_reg[i-1] <= r_fcr_reg[i];
	  end
     end // always_ff@ (posedge clk)

   fp_add #(.W(32), .ADD_LAT(FPU_LAT)) 
   sa (.clk(clk),
       .sub(opcode == SP_SUB),
       .a(src_a[31:0]),
       .b(src_b[31:0]),
       .en(opcode == SP_ADD || opcode == SP_SUB),
       .y(t_sp_adder_result)
       );
   
   fp_add #(.W(64), .ADD_LAT(FPU_LAT)) 
   sd (.clk(clk),
       .sub(opcode == DP_SUB),
       .a(src_a),
       .b(src_b),
       .en(opcode == DP_ADD || opcode == DP_SUB),
       .y(t_dp_adder_result)
       );
   
   fp_mul #(.W(32), .MUL_LAT(FPU_LAT)) 
   sm (.clk(clk),
       .a(src_a[31:0]),
       .b(src_b[31:0]),
       .en(opcode == SP_MUL),
       .y(t_sp_mult_result)
       );
   
   fp_mul #(.W(64), .MUL_LAT(FPU_LAT)) 
   dm (.clk(clk),
       .a(src_a[63:0]),
       .b(src_b[63:0]),
       .en(opcode == DP_MUL),
       .y(t_dp_mult_result)
       );

   
endmodule // fpu
