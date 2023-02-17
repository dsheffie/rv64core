`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

module decode_mips32(insn, 
		     pc, insn_pred, pht_idx, insn_pred_target,
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

   wire [5:0] 	opcode = insn[31:26];
   wire 	is_nop = (insn == 32'd0);
   wire 	is_ehb = (insn == 32'd192);
   
   
   /* how many zero pad bits for reg specifiers */
   localparam ZP = (`LG_PRF_ENTRIES-5);
   
   wire [`LG_PRF_ENTRIES-1:0]	rs = {{ZP{1'b0}},insn[25:21]};
   wire [`LG_PRF_ENTRIES-1:0] 	rt = {{ZP{1'b0}},insn[20:16]};
   wire [`LG_PRF_ENTRIES-1:0] 	rd = {{ZP{1'b0}},insn[15:11]};

   wire [`LG_PRF_ENTRIES-1:0] 	fs = {{ZP{1'b0}},insn[15:11]};
   wire [`LG_PRF_ENTRIES-1:0] 	ft = {{ZP{1'b0}},insn[20:16]};
   wire [`LG_PRF_ENTRIES-1:0] 	fd = {{ZP{1'b0}},insn[10:6]};

   
   wire [`LG_PRF_ENTRIES-1:0] shamt = {{ZP{1'b0}},insn[10:6]};
   
   always_comb
     begin
	uop.op = II;
	uop.srcA = 'd0;
	uop.srcB = 'd0;
	uop.dst = 'd0;
	uop.srcA_valid = 1'b0;
	uop.srcB_valid = 1'b0;
	uop.fp_srcA_valid = 1'b0;
	uop.fp_srcB_valid = 1'b0;
	uop.hilo_dst_valid = 1'b0;
	uop.hilo_src_valid = 1'b0;
	uop.hilo_dst = 'd0;
	uop.hilo_src = 'd0;
	
	uop.dst_valid = 1'b0;
	uop.fp_dst_valid = 1'b0;
	
	uop.has_delay_slot = 1'b0;
	uop.has_nullifying_delay_slot = 1'b0;
	uop.imm = 16'd0;
	uop.jmp_imm = {(`M_WIDTH-16){1'b0}};
	uop.pc = pc;
	uop.serializing_op = 1'b0;
	uop.must_restart = 1'b0;
	uop.rob_ptr = 'd0;
	uop.br_pred = 1'b0;
	uop.is_br = 1'b0;
	uop.pht_idx = pht_idx;
	uop.is_mem = 1'b0;
	uop.is_int = 1'b0;
	uop.is_store = 1'b0;
`ifdef ENABLE_CYCLE_ACCOUNTING
	uop.fetch_cycle = fetch_cycle;
`endif
	case(opcode)
	  6'd0: /* rtype */
	    begin
	       case(insn[5:0])
		 6'd0: /* sll */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = shamt;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = is_nop||is_ehb ? NOP :SLL;
		      uop.is_int = 1'b1;		      
		   end
		 6'd2: /* srl */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = shamt;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SRL;
		      uop.is_int = 1'b1;
		   end
		 6'd3: /* sra */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = shamt;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SRA;
		      uop.is_int = 1'b1;
		   end
		 6'd4: /* sllv */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SLLV;
		      uop.is_int = 1'b1;
		   end
		 6'd5:
		   begin
		      uop.op = MONITOR;
		      uop.serializing_op = 1'b1;
		      uop.has_delay_slot = 1'b0;
		      uop.must_restart = 1'b1;
		      uop.dst = 'd2;
		      uop.dst_valid = 1'b1;
		      uop.srcA = 'd31;
		      uop.srcA_valid = 1'b1;
		      uop.imm = {1'b0, insn[21:7]};
		      uop.is_int = 1'b1;
		   end // case: 6'd5
		 6'd6: /* srlv */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SRLV;
		      uop.is_int = 1'b1;
		   end
		 6'd7: /* srav */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SRAV;
		      uop.is_int = 1'b1;
		   end		 
		 6'd8: /* jr */
		   begin
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.has_delay_slot = 1'b1;
		      uop.op = JR;
		      uop.imm = insn_pred_target[15:0];
		      uop.jmp_imm = insn_pred_target[`M_WIDTH-1:16];
		      uop.is_br = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd9: /* jalr */
		   begin
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.has_delay_slot = 1'b1;
		      uop.op = JALR;
		      uop.dst_valid = rd != 'd0;
		      uop.dst = rd;
		      uop.imm = insn_pred_target[15:0];
		      uop.jmp_imm = insn_pred_target[`M_WIDTH-1:16];
		      uop.is_br = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd13:
		   begin
		      uop.op = BREAK;
		      uop.serializing_op = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd15: /* sync - treat as nop */
		   begin
		      uop.op = NOP;
		      uop.is_int = 1'b1;
		   end
		 6'd16:
		   begin
		      uop.op = (rd == 'd0) ? NOP : MFHI;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.hilo_src_valid = 1'b1;
		      uop.is_int = 1'b1;		      
		   end
		 6'd17:
		   begin
		      uop.op = MTHI;
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.hilo_src_valid = 1'b1;		      
		      uop.hilo_dst_valid = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd18:
		   begin
		      uop.op = (rd == 'd0) ? NOP : MFLO;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.hilo_src_valid = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd19:
		   begin
		      uop.op = MTLO;
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.hilo_src_valid = 1'b1;		      
		      uop.hilo_dst_valid = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd24: /* mult */
		   begin
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rt;
		      uop.srcB_valid = 1'b1;
		      uop.hilo_dst_valid = 1'b1;
		      uop.op = MULT;
		      uop.is_int = 1'b1;
		   end
		 6'd25: /* multu */
		   begin
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rt;
		      uop.srcB_valid = 1'b1;
		      uop.hilo_dst_valid = 1'b1;
		      uop.op = MULTU;
		      uop.is_int = 1'b1;
		   end
		 6'd26: /* div */
		   begin
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rt;
		      uop.srcB_valid = 1'b1;
		      uop.hilo_dst_valid = 1'b1;
		      uop.op = DIV;
		      uop.is_int = 1'b1;
		   end
		 6'd27: /* divu */
		   begin
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rt;
		      uop.srcB_valid = 1'b1;
		      uop.hilo_dst_valid = 1'b1;
		      uop.op = DIVU;
		      uop.is_int = 1'b1;
		   end		 
		 6'd33: /* addu */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : ADDU;
		      uop.is_int = 1'b1;
		   end
		 6'd35: /* subu */
		   begin
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rt;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SUBU;
		      uop.is_int = 1'b1;
		   end
		 6'd36: /* and */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : AND;
		      uop.is_int = 1'b1;
		   end
		 6'd37: /* or */
		   begin
		      if(rs == 'd0)
			begin
			   uop.srcA = rt;
			   uop.srcA_valid = 1'b1;
			   uop.dst = rd;
			   uop.dst_valid = (rd != 'd0);
			   uop.op = (rd == 'd0) ? NOP : MOV;
			   uop.is_int = 1'b1;
			end
		      else
			begin
			   uop.srcA = rt;
			   uop.srcA_valid = 1'b1;
			   uop.srcB = rs;
			   uop.srcB_valid = 1'b1;
			   uop.dst = rd;
			   uop.dst_valid = (rd != 'd0);
			   uop.op = (rd == 'd0) ? NOP : OR;
			   uop.is_int = 1'b1;
			end
		   end
		 6'd38: /* xor */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : XOR;
		      uop.is_int = 1'b1;
		   end	
		 6'd39: /* nor */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : NOR;
		      uop.is_int = 1'b1;
		   end
		 6'd42: /* slt */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SLT;
		      uop.is_int = 1'b1;
		   end
		 6'd43: /* sltu */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : SLTU;
		      uop.is_int = 1'b1;
		   end
		 6'd52: /* teq */
		   begin
		      uop.op = NOP;
		      uop.is_int = 1'b1;
		   end
		 default:
		   begin
		   end
	       endcase // case (insn[5:0])
	    end // case: 6'd0
	  /* end-rtype */
	  6'd1: /* BGEZ through BLTZ */
	    begin
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_br = 1'b1;
	       uop.is_int = 1'b1;
	       uop.br_pred = insn_pred;

	       
	       case(rt[4:0])
		 'd0: /* BLTZ */
		   begin
		      uop.op = BLTZ;
		   end
		 'd1: /* BGEZ */
		   begin
		      uop.op = BGEZ;
		   end
		 'd2: /* BLTZL */
		   begin
		      uop.op = BLTZL;
		      uop.has_nullifying_delay_slot = 1'b1;
		   end
		 'd3:
		   begin /* BGEZL */
		      uop.op = BGEZL;
		      uop.has_nullifying_delay_slot = 1'b1;
		   end
		 'd17:
		   begin /* BGEZAL */
		      uop.op = (rs == 'd0) ? BAL : BGEZAL;
		      uop.dst_valid = 1'b1;
		      uop.dst = 'd31;
		      uop.srcB = 'd31;
		      uop.srcB_valid = (rs == 'd0) ? 1'b0 : 1'b1;
		   end
		 default:
		   begin
		      uop.op = II;
		   end
	       endcase // case (rt[1:0])
	    end // case: 6'd1
	  6'd2: /* J - just fold */
	    begin
	       uop.op = J;
	       uop.is_br = 1'b1;
	       uop.is_int = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       //uop.imm = insn[15:0];
	       //uop.jmp_imm = insn[25:16];
	       //uop.br_pred = 1'b1;
	    end
	  6'd3:
	    begin
	       uop.op = JAL;
	       uop.has_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.jmp_imm = {{(`M_WIDTH-26){1'b0}}, insn[25:16]};
	       uop.dst_valid = 1'b1;
	       uop.dst = 'd31;	  
	       uop.br_pred = 1'b1;
	       uop.is_br = 1'b1;
	       uop.is_int = 1'b1;	       
	    end
	  6'd4: /* BEQ */
	    begin
	       uop.op = BEQ;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.br_pred = insn_pred;
	       uop.is_br = 1'b1;
	       uop.is_int = 1'b1;
	    end // case: 6'd4
	  6'd5: /* BNE */
	    begin
	       //$display("decoded bne, rs = %d, rs = %d", rs, rt);
	       uop.op = BNE;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.br_pred = insn_pred;
	       uop.is_br = 1'b1;
	       uop.is_int = 1'b1;
	    end // case: 6'd5
	  6'd6: /* BLEZ */
	    begin	    
	       uop.op = BLEZ;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.br_pred = insn_pred;
	       uop.is_br = 1'b1;
	       uop.is_int = 1'b1;	       
	    end
	  6'd7: /* BGTZ */
	    begin
	       uop.op = BGTZ;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.br_pred = insn_pred;
	       uop.is_br = 1'b1;
	       uop.is_int = 1'b1;
	    end
	  6'd9: /* ADDIU */
	    begin
	       if(rs == 'd0)
		 begin
		    uop.op = (rt == 'd0) ? NOP : MOVI;
		    uop.dst_valid = (rt != 'd0);
		    uop.is_int = 1'b1;
		    uop.dst = rt;
		    uop.imm = insn[15:0];		    
		 end
	       else
		 begin
		    uop.op = (rt == 'd0) ? NOP : ADDIU;
		    uop.srcA_valid = 1'b1;
		    uop.srcA = rs;
		    uop.dst_valid = (rt != 'd0);
		    uop.is_int = 1'b1;
		    uop.dst = rt;
		    uop.imm = insn[15:0];
		 end
	    end
	  6'd10: /* SLTI */
	    begin
	       uop.op = (rt == 'd0) ? NOP : SLTI;
	       uop.srcA_valid = 1'b1;
	       uop.srcA = rs;
	       uop.dst_valid = (rt != 'd0);
	       uop.dst = rt;
	       uop.is_int = 1'b1;	       
	       uop.imm = insn[15:0];
	    end
	  6'd11: /* SLTIU */
	    begin
	       uop.op = (rt == 'd0) ? NOP : SLTIU;
	       uop.srcA_valid = 1'b1;
	       uop.srcA = rs;
	       uop.dst_valid = (rt != 'd0);
	       uop.dst = rt;
	       uop.is_int = 1'b1;	       
	       uop.imm = insn[15:0];
	    end
	  6'd12: /* ANDI */
	    begin
	       uop.op = (rt == 'd0) ? NOP : ANDI;
	       uop.srcA_valid = 1'b1;
	       uop.srcA = rs;
	       uop.dst_valid = (rt != 'd0);
	       uop.dst = rt;
	       uop.is_int = 1'b1;
	       uop.imm = insn[15:0];	       
	    end
	  6'd13: /* ORI */
	    begin
	       uop.op = (rt == 'd0) ? NOP : ORI;
	       uop.srcA_valid = 1'b1;
	       uop.srcA = rs;	       
	       uop.dst_valid = (rt != 'd0);
	       uop.dst = rt;
	       uop.is_int = 1'b1;
	       uop.imm = insn[15:0];
	       //$display("ORI : dest %d, src %d, imm = %d", uop.dst, uop.srcA, uop.imm);
	       
	    end
	  6'd14: /* XORI */
	    begin
	       uop.op = (rt == 'd0) ? NOP : XORI;
	       uop.srcA_valid = 1'b1;
	       uop.srcA = rs;	       
	       uop.dst_valid = (rt != 'd0);
	       uop.dst = rt;
	       uop.is_int = 1'b1;
	       uop.imm = insn[15:0];	       
	    end
	  6'd15: /* LUI*/
	    begin
	       uop.op = (rt == 'd0) ? NOP : LUI;
	       uop.dst_valid = (rt != 'd0);
	       uop.dst = rt;
	       uop.is_int = 1'b1;
	       uop.imm = insn[15:0];
	    end
	  6'd16: /* coproc0 */
	    begin
	       case(insn[25:21]) /* switch on RS */
		 5'd0: /* mfc0 */
		   begin
		      if(rd == 'd12)
			begin
			   uop.op = MFC0;
			   uop.dst = rt;
			   uop.dst_valid = 1'b1;
			   uop.srcA = rd;
			   uop.is_int = 1'b1;
			   uop.serializing_op = 1'b1;
			   uop.must_restart = 1'b1;
			end
		   end
		 5'd4: /* mtc0 */
		   begin
		      if(rd == 'd12)
			begin
			   uop.op = MTC0;
			   uop.dst = rd;
			   uop.srcA = rt;
			   uop.srcA_valid = 1'b1;
			   uop.serializing_op = 1'b1;
			   uop.has_delay_slot = 1'b0;
			   uop.is_int = 1'b1;
			   uop.must_restart = 1'b1;
			end // if (rd == 'd12)
		      else
			begin
			   uop.op = NOP;
			end
		   end // case: 5'd4
		 default:
		   begin
		   end
	       endcase // case (insn[25:21])
	    end // case: 6'd16
	  6'd17: /* coproc1 */
	    begin
	       if((insn[25:21]==5'd0) && (insn[10:0] == 11'd0))
		 begin /* mfc1 */
		    uop.dst = rt;
		    uop.dst_valid = 1'b1;
		    uop.op = MFC1_MERGE;
		    uop.srcB = {{ZP{1'b0}}, rd[4:1], 1'b0};
		    uop.jmp_imm = { {(`M_WIDTH-17){1'b0}}, rd[0]};
		    uop.fp_srcB_valid = 1'b1;
		    uop.is_mem = 1'b1;
		 end
	       else if((insn[25:21]==5'd4) && (insn[10:0] == 11'd0))
		 begin /* mtc1 */
		    uop.srcA = rt;
		    uop.srcA_valid = 1'b1;
		    uop.op = MTC1_MERGE;
		    uop.dst = {{ZP{1'b0}}, rd[4:1], 1'b0};
		    uop.srcB = {{ZP{1'b0}}, rd[4:1], 1'b0};
		    uop.jmp_imm = { {(`M_WIDTH-17){1'b0}}, rd[0]};
		    uop.fp_srcB_valid = 1;			 
		    uop.fp_dst_valid = 1'b1;
		    uop.is_mem = 1'b1;
		 end // if ((insn[25:21]==5'd4) && (insn[10:0] == 11'd0))
	    end // case: 6'd17
	  6'd20: /* BEQL */
	    begin
	       uop.op = BEQL;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.has_nullifying_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_br = 1'b1;
	       uop.br_pred = insn_pred;
	       uop.is_int = 1'b1;
	    end // case: 6'd20
	  6'd21: /* BNEL */
	    begin
	       uop.op = BNEL;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.has_nullifying_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_br = 1'b1;
	       uop.br_pred = insn_pred;
	       uop.is_int = 1'b1;
	    end // case: 6'd21
	  6'd22: /* BLEZL */
	    begin	    
	       uop.op = BLEZL;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.has_nullifying_delay_slot = 1'b1;	       
	       uop.imm = insn[15:0];
	       uop.is_br = 1'b1;
	       uop.br_pred = insn_pred;
	       uop.is_int = 1'b1;
	    end // case: 6'd22
	  6'd23: /* BGTZL */
	    begin
	       uop.op = BGTZL;
	       uop.dst_valid = 1'b0;
	       uop.dst = 'd0;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.has_delay_slot = 1'b1;
	       uop.has_nullifying_delay_slot = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_br = 1'b1;
	       uop.br_pred = insn_pred;
	       uop.is_int = 1'b1;
	    end
	  6'd32: /* LB */
	    begin
	       uop.op = LB;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd33: /* LH */
	    begin
	       uop.op = LH;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd34: /* LWL */
	    begin
	       uop.op = LWL;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd35: /* LW */
	    begin
	       uop.op = LW;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd36: /* LBU */
	    begin
	       uop.op = LBU;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd37: /* LHU */
	    begin
	       uop.op = LHU;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd38: /* LWR */
	    begin
	       uop.op = LWR;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd40: /* SB */
	    begin
	       uop.op = SB;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	    end	    
	  6'd41: /* SH */
	    begin
	       uop.op = SH;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;	       
	    end
	  6'd42:
	    begin
	       uop.op = SWL;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	    end
	  6'd43: /* SW */
	    begin
	       uop.op = SW;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	    end
	  6'd46: /* SWR */
	    begin
	       uop.op = SWR;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.srcB_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	    end
	  6'd48: /* LL - hack treat load-linked as normal load*/
	    begin
	       uop.op = LW;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.dst = rt;
	       uop.dst_valid = (rt != 'd0);
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd51: /* PREF */
	    begin
	       uop.op = NOP;
	       uop.is_int = 1'b1;
	    end
	  
	  default:
	    begin
	    end
	endcase // case (insn[5:0])
     end // always_comb
   

endmodule
   
