`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

module decode_riscv(insn, 
		    pc, 
		    insn_pred, 
		    pht_idx, 
		    insn_pred_target,
`ifdef ENABLE_CYCLE_ACCOUNTING   		     
		    fetch_cycle,
`endif
		    uop);
   
   input logic [31:0] insn;
   input logic [`M_WIDTH-1:0] pc;
   input logic 	      insn_pred;
   input logic [`LG_PHT_SZ-1:0] pht_idx;
   input logic [`M_WIDTH-1:0] 	insn_pred_target;
`ifdef ENABLE_CYCLE_ACCOUNTING   
   input logic [63:0] 		fetch_cycle;
`endif
   output 	uop_t uop;

   wire [6:0] 	opcode = insn[6:0];
   localparam ZP = (`LG_PRF_ENTRIES-5);
   wire [`LG_PRF_ENTRIES-1:0] rd = {{ZP{1'b0}},insn[11:7]};
   wire [`LG_PRF_ENTRIES-1:0] rs1 = {{ZP{1'b0}},insn[19:15]};
   wire [`LG_PRF_ENTRIES-1:0] rs2 = {{ZP{1'b0}},insn[24:20]};
   
   wire 	is_nop = (insn == 32'd0);
   logic 	rd_is_link, rs1_is_link;   
   

   wire [`LG_PRF_ENTRIES-1:0] 	rt = {{ZP{1'b0}},insn[20:16]};
   wire [`LG_PRF_ENTRIES-1:0] 	shamt = {{ZP{1'b0}},insn[10:6]};

   logic [31:0] 		t_imm;
   wire [31:0] 			w_pc_imm;
   always_comb
     begin
	t_imm = 'd0;
	case(opcode)
	  7'h17: /* auipc */
	    begin
	       t_imm = {insn[31:12], 12'd0};
	    end
	  7'h63: /* branches */
	    begin
	       t_imm = {{19{insn[31]}}, insn[31], insn[7], insn[30:25], insn[11:8], 1'b0};
	    end
	  7'h6f: /* jal and j */
	    begin
	       t_imm = {{11{insn[31]}}, insn[31], insn[19:12], insn[20], insn[30:21], 1'b0}; 
	    end
	  default:
	    begin
	    end
	  endcase
     end
   
   ppa32 imm_add (.A(pc), .B(t_imm), .Y(w_pc_imm));
   
   always_comb
     begin
	rd_is_link = (rd == 'd1) || (rd == 'd5);
	rs1_is_link = (rs1 == 'd1) || (rs1 == 'd5);
	uop.op = II;
	uop.srcA = 'd0;
	uop.srcB = 'd0;
	uop.dst = 'd0;
	uop.srcA_valid = 1'b0;
	uop.srcB_valid = 1'b0;
	uop.fp_srcA_valid = 1'b0;
	uop.fp_srcB_valid = 1'b0;
	
	uop.dst_valid = 1'b0;
	uop.fp_dst_valid = 1'b0;
	
	uop.imm = 16'd0;
	uop.jmp_imm = {(`M_WIDTH-16){1'b0}};
	uop.rvimm = 32'd0;
	
	uop.pc = pc;
	uop.serializing_op = 1'b0;
	uop.must_restart = 1'b0;
	uop.rob_ptr = 'd0;
	uop.br_pred = 1'b0;
	uop.is_br = 1'b0;
	uop.pht_idx = pht_idx;
	uop.is_mem = 1'b0;
	uop.is_int = 1'b0;
	uop.is_cheap_int = 1'b0;
	uop.is_store = 1'b0;
`ifdef ENABLE_CYCLE_ACCOUNTING
	uop.fetch_cycle = fetch_cycle;
`endif

	case(opcode)
	  7'h3:
	    begin
	       uop.dst = rd;
	       uop.srcA = rs1;
	       uop.dst_valid = (rd != 'd0);
	       uop.srcA_valid = 1'b1;
	       uop.is_mem = 1'b1;
	       uop.rvimm = {{20{insn[31]}}, insn[31:20]};
	       case(insn[14:12])
		 3'd0:
		   begin
		      uop.op = LB;
		   end
		 3'd1:
		   begin
		      uop.op = LH;
		   end
		 3'd2:
		   begin
		      uop.op = LW;
		   end
		 3'd4:
		   begin
		      uop.op = LBU;
		   end
		 3'd5:
		   begin
		      uop.op = LHU;
		   end		 
		 default:
		   begin
		   end
	       endcase
	    end // case: 7'h3
	  7'hf:
	    begin
	       uop.op = NOP;
	    end
	  7'h13:
	    begin
	       uop.dst = rd;
	       uop.srcA = rs1;
	       uop.dst_valid = (rd != 'd0);
	       uop.srcA_valid = (rd != 'd0);	       	       
	       uop.is_int = 1'b1;
	       uop.rvimm = {{20{insn[31]}}, insn[31:20]};
	       case(insn[14:12])
		 3'd0: /* addi */
		   begin
		      uop.op = (rd == 'd0) ? NOP : ADDI;
		      uop.is_cheap_int = 1'b1;
		   end
		 3'd1:
		   begin
		      uop.op = (rd == 'd0) ? NOP : SLLI;
		      uop.is_cheap_int = 1'b1;
		   end
		 3'd2:
		   begin
		      uop.op = (rd == 'd0) ? NOP : SLTI;
		      uop.is_cheap_int = 1'b1;
		   end
		 3'd3:
		   begin
		      uop.op = (rd == 'd0) ? NOP : SLTIU;
		      uop.is_cheap_int = 1'b1;
		   end
		 3'd4:
		   begin
		      uop.op = (rd == 'd0) ? NOP : XORI;
		      uop.is_cheap_int = 1'b1;		      
		   end
		 3'd5:
		   begin
		      case(insn[31:25])
			7'h0:
			  begin
			     uop.op = (rd == 'd0) ? NOP : SRLI;
			     uop.is_cheap_int = 1'b1;
			  end
			7'h20:
			  begin
			     uop.op = (rd == 'd0) ? NOP : SRAI;
			     uop.is_cheap_int = 1'b1;
			  end
			default:
			  begin
			  end
		      endcase
		   end
		 3'd6:
		   begin
		      uop.op = (rd == 'd0) ? NOP : ORI;	
		      uop.is_cheap_int = 1'b1;		      	      		      
		   end
		 3'd7:
		   begin
		      uop.op = (rd == 'd0) ? NOP : ANDI;
		      uop.is_cheap_int = 1'b1;		      		      
		   end
	       endcase // case (inst[14:12])
	    end // case: 7'h13
	  7'h17: /* auipc */
	    begin
	       uop.op = (rd == 'd0) ? NOP : AUIPC;
	       uop.dst = rd;
	       uop.dst_valid = (rd != 'd0);
	       uop.is_int = 1'b1;
	       uop.is_cheap_int = 1'b1;
	       uop.rvimm = w_pc_imm;
	    end
	  7'h23:
	    begin
	       uop.srcA = rs1;
	       uop.srcB = rs2;
	       uop.srcA_valid = 1'b1;
	       uop.srcB_valid = 1'b1;
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	       uop.rvimm = {{20{insn[31]}}, insn[31:25], insn[11:7]};
	       case(insn[14:12])
		 3'd0:
		   begin
		      uop.op = SB;
		   end
		 3'd1:
		   begin
		      uop.op = SH;
		   end
		 3'd2:
		   begin
		      uop.op = SW;
		   end
		 default:
		   begin
		   end
	       endcase
	    end
	  7'h33:
	    begin
	       uop.dst = rd;
	       uop.dst_valid = (rd != 'd0);
	       uop.srcA_valid = 1'b1;
	       uop.srcA = rs1;
	       uop.srcB_valid = 1'b1;
	       uop.srcB = rs2;
	       uop.is_int = 1'b1;
	       case(insn[14:12])
		 3'd0:
		   begin
		      case(insn[31:25])
			7'h0:
			  begin
			     uop.op = (rd != 'd0) ? ADDU : NOP;
			     uop.is_cheap_int = 1'b1;
			  end
			7'h1:
			  begin
			     uop.op = (rd != 'd0) ? MUL : NOP;
			  end
			7'h20:
			  begin
			     uop.op = (rd != 'd0) ? SUBU : NOP;
			     uop.is_cheap_int = 1'b1;
			  end
			default:
			  begin
			  end
		      endcase // case (insn[31:25])
		   end // case: 3'd0
		 3'd1:
		   begin
		      case(insn[31:25])
			7'd0:
			  begin
			     uop.op = (rd != 'd0) ? SLL : NOP;
			     uop.is_cheap_int = 1'b1;			     
			  end
			7'h1:
			  begin
			     uop.op = (rd != 'd0) ? MULH : NOP;
			  end
			default:
			  begin
			  end
		      endcase
		   end // case: 3'd1
		 3'd2:
		   begin
		      case(insn[31:25])
			7'd0:
			  begin
			     uop.op = (rd != 'd0) ? SLT : NOP;
			     uop.is_cheap_int = 1'b1;
			  end
			default:
			  begin
			  end
		      endcase
		   end
		 3'd3:
		   begin
		      case(insn[31:25])
			7'h0:
			  begin
			     uop.op = (rd != 'd0) ? SLTU : NOP;
			     uop.is_cheap_int = 1'b1;
			  end
			7'h1:
			  begin
			     uop.op = (rd != 'd0) ? MULHU : NOP;
			  end
			default:
			  begin
			  end
		      endcase
		   end
		 3'd4:
		   begin
		      case(insn[31:25])
			7'h0:
			  begin
			     uop.op = (rd != 'd0) ? XOR : NOP;
			     uop.is_cheap_int = 1'b1;			     
			  end
			7'h1:
			  begin
			     uop.op = (rd != 'd0) ? DIV : NOP;
			  end
			default:
			  begin
			  end
		      endcase // case (insn[31:25])
		   end // case: 3'd4
		 3'd5:
		   begin
		      case(insn[31:25])
			7'h0:
			  begin
			     uop.op = (rd != 'd0) ? SRL : NOP;
			     uop.is_cheap_int = 1'b1;			     
			  end
			7'h1:
			  begin
			     uop.op = (rd != 'd0) ? DIVU : NOP;
			  end
			7'h20:
			  begin
			     uop.op = (rd != 'd0) ? SRA : NOP;
			     uop.is_cheap_int = 1'b1;
			  end
			default:
			  begin
			  end
		      endcase // case (insn[31:25])
		   end // case: 3'd5
		 3'd6:
		   begin
		      case(insn[31:25])
			7'h0:
			  begin
			     uop.op = (rd != 'd0) ? OR : NOP;
			     uop.is_cheap_int = 1'b1;
			  end
			7'h1:
			  begin
			     uop.op = (rd != 'd0) ? REM : NOP;
			  end
			default:
			  begin
			  end
		      endcase // case (insn[31:25])
		   end
		 3'd7:
		   begin
		      case(insn[31:25])
			7'h0:
			  begin
			     uop.op = (rd != 'd0) ? AND : NOP;
			     uop.is_cheap_int = 1'b1;
			  end
			7'h1:
			  begin
			     uop.op = (rd != 'd0) ? REMU : NOP;
			  end
			default:
			  begin
			  end
		      endcase // case (insn[31:25])
		      
		   end
		 default:
		   begin
		   end
	       endcase
	    end
	  7'h37: /* lui */
	    begin
	       uop.op = (rd == 'd0) ? NOP : LUI;
	       uop.dst = rd;
	       uop.dst_valid = (rd != 'd0);
	       uop.is_int = 1'b1;
	       uop.is_cheap_int = 1'b1;
	       uop.rvimm = {insn[31:12], 12'd0};
	    end
	  7'h63: /* branches */
	    begin
	       uop.srcA = rs1;
	       uop.srcB = rs2;
	       uop.srcA_valid = 1'b1;
	       uop.srcB_valid = 1'b1;
	       uop.is_int = 1'b1;
	       uop.rvimm = w_pc_imm;
	       uop.br_pred = insn_pred;
	       uop.is_br = 1'b1;
	       uop.is_cheap_int = 1'b1;
	       case(insn[14:12])
		 3'd0:
		   begin
		      uop.op = BEQ;
		   end
		 3'd1:
		   begin
		      uop.op = BNE;
		   end
		 3'd4:
		   begin
		      uop.op = BLT;
		   end
		 3'd5:
		   begin
		      uop.op = BGE;
		   end
		 3'd6:
		   begin
		      uop.op = BLTU;
		   end
		 3'd7:
		   begin
		      uop.op = BGEU;
		   end
		 default:
		   begin
		   end
	       endcase
	    end
	    
	  7'h67: /* jalr and jr*/
	    begin
	       uop.srcA_valid = 1'b1;
	       uop.srcA = rs1;
	       uop.is_int = 1'b1;
	       uop.is_br = 1'b1;
	       uop.imm = insn_pred_target[15:0];
	       uop.jmp_imm = insn_pred_target[`M_WIDTH-1:16];
	       uop.rvimm = {{20{insn[31]}}, insn[31:20]};
	       uop.br_pred = 1'b1;
	       uop.is_br = 1'b1;
	       uop.is_cheap_int = 1'b1;	       
	       if(rd == 'd0)
		 begin
		    uop.op = rs1_is_link ? RET : JR;
		 end
	       else
		 begin
		    uop.op = JALR;
		    uop.dst_valid = 1'b1;
		    uop.dst = rd;
		 end
	    end // case: 7'h67
	  7'h6f: /* jal and j */
	    begin
	       uop.rvimm = w_pc_imm;
	       uop.is_br = 1'b1;	       
	       if(rd == 'd0)
		 begin
		    uop.op = J;
		    uop.br_pred = 1'b1;
		    uop.is_br = 1'b1;
		    uop.is_int = 1'b1;		    
		 end
	       else
		 begin
		    uop.op = JAL;		    
		    uop.dst_valid = 1'b1;
		    uop.dst = rd;	  
		    uop.br_pred = 1'b1;
		    uop.is_br = 1'b1;
		    uop.is_int = 1'b1;
		    uop.is_cheap_int = 1'b1;
		 end
	    end
	  7'h73: /* this is a bunch of system stuff I dont care about currently */
	    begin
	       uop.is_int = 1'b1;
	       uop.op = NOP;	       	       
	       if(insn[31:7] == 'd0) /* treat as brk */
		 begin
		    uop.op = BREAK;
		    uop.serializing_op = 1'b1;
		 end
	       else if(insn[31:20] == 'd1 && insn[19:7] == 'd0)
		 begin
		    uop.op = MONITOR;
		    uop.serializing_op = 1'b1;
		    uop.must_restart = 1'b1;		    
		 end
	       else
		 begin
		    case(insn[14:12])
		      3'd2: /*CSRRS */
			begin
			   if(insn[31:20] == 12'hc00)
			     begin
				uop.op = (rd == 'd0) ? NOP : RDCYCLE;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.is_cheap_int = 1'b1;
			     end
			   else if(insn[31:20] == 12'hc80)
			     begin
				uop.op = (rd == 'd0) ? NOP : RDCYCLEH;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.is_cheap_int = 1'b1;	
			     end
			   else if(insn[31:20] == 12'hc02)
			     begin
				uop.op = (rd == 'd0) ? NOP : RDINSTRET;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
			     end
			   else if(insn[31:20] == 12'hc82)
			     begin
				uop.op = (rd == 'd0) ? NOP : RDINSTRETH;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
			     end
			   else if(insn[31:20] == 12'hc03)
			     begin
				uop.op = (rd == 'd0) ? NOP : RDBRANCH;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
			     end
			   else if(insn[31:20] == 12'hc04)
			     begin
				uop.op = (rd == 'd0) ? NOP : RDFAULTEDBRANCH;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
			     end
			   // else
			   //   begin
			   // 	$display("wtf is at pc %x, sel %x", 
			   // 		 pc, insn[31:20]);
			   //   end
			end
		      default:
			begin
			end
		    endcase
		 end
	    end
	  default:
	    begin
	    end
	endcase // case (opcode)
     end // always_comb
endmodule
   
