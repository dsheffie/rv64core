

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




module predecode(pc, insn, pd);
   input logic [63:0] pc;
   input logic [31:0] insn;
   output logic [3:0] pd;
   logic [6:0] 	      opcode;
   
   logic [4:0] 	      rd, rs1;
   logic 	      rd_is_link, rs1_is_link;
   logic	      rd_eq_rs1;
   
   
   
   always_comb
     begin
	pd = 'd0;
	opcode = insn[6:0];
	rd = insn[11:7];
	rs1 = insn[19:15];
	rd_is_link = (rd == 'd1) || (rd == 'd5);
	rs1_is_link = (rs1 == 'd1) || (rs1 == 'd5);
	rd_eq_rs1 = (rd == rs1);
	
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
		    //00000000004a106c : rs1_is_link = 0, rd_is_link = 1
		    if(rs1_is_link & rd_is_link)
		      begin
			 pd = /*rd_eq_rs1 ? 'd5 :*/ 'd7;
		      end
		    else if(rs1_is_link & (rd_is_link == 1'b0))
		      begin
			 pd = 'd4;
		      end
		    else if((rs1_is_link == 1'b0) & (rd_is_link))
		      begin
			 /* indirect function call */
			 pd = 'd6;
		      end
		    else
		      begin
			 pd = 'd4;
		      end

		    //pd = rs1_is_link ? 'd6 /* jalr */ : 'd4 /* jr */;
		    /* jalr */
		    //
		    //$display("%x : rs1_is_link = %b, rd_is_link = %b\n", pc, rs1_is_link, rd_is_link);
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
