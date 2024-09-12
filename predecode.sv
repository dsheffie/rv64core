

/*
typedef enum logic [3:0] {
			  NOT_CFLOW = 'd0,
			  IS_COND_BR = 'd1,
 			  IS_RET = 'd2,
			  IS_J = 'd3,
			  IS_JR = 'd4,
			  IS_JAL = 'd5,
			  IS_JALR = 'd6
			  } jump_t;
*/




module predecode(insn, pd);
   input logic [31:0] insn;
   output logic [2:0] pd;
   logic [6:0] 	      opcode;
   
   logic [4:0] 	      rd, rs1;
   logic 	      rd_is_link, rs1_is_link;
   
   always_comb
     begin
	pd = 'd0;
	opcode = insn[6:0];
	rd = insn[11:7];
	rs1 = insn[19:15];
	rd_is_link = (rd == 'd1) || (rd == 'd5);
	rs1_is_link = (rs1 == 'd1) || (rs1 == 'd5);
	
	case(opcode)
	  7'h63: /* cond branches */
	    begin
	       pd = 'd1;
	    end
	  7'h67: /* jalr and jr */
	    begin
	       //$display("rd = %d, rs1 = %d, rd link %b, rs1 link %b", rd, rs1, rd_is_link, rs1_is_link);
	       if(rd == 'd0)
		 begin
		    pd = rs1_is_link ? 'd2 /* return */: 'd4; /*jr */
		 end
	       else
		 begin
		    /* jalr */
		    pd = 'd6;
		 end

	    end
	  7'h6f:
	    begin
	       //$display("rd = %d, rs1 = %d", rd, rs1);
	       if(rd_is_link)
		 begin
		    pd = 'd5 /*jal*/;
		 end
	       else		 
		 begin
		    pd = 'd3; /* j */
		 end
	    end
	  default:
	    begin
	    end
	endcase // case (opcode)
     end // always_comb
endmodule // predecode
