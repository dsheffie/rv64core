`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

module decode_mips32(in_64b_fpreg_mode, insn, 
		     pc, insn_pred, pht_idx, insn_pred_target,
`ifdef ENABLE_CYCLE_ACCOUNTING   		     
		     fetch_cycle,
`endif
		     uop);
   input logic in_64b_fpreg_mode;
   
   input logic [31:0] insn;
   input logic [`M_WIDTH-1:0] pc;
   input logic 	      insn_pred;
   input logic [`LG_PHT_SZ-1:0] pht_idx;
   input logic [`M_WIDTH-1:0] 	insn_pred_target;
`ifdef ENABLE_CYCLE_ACCOUNTING   
   input logic [63:0] 		fetch_cycle;
`endif
   output 	uop_t uop;

   logic [5:0] 	opcode = insn[31:26];
   logic 	is_nop = (insn == 32'd0);
   logic 	is_ehb = (insn == 32'd192);
   
   
   /* how many zero pad bits for reg specifiers */
   localparam ZP = (`LG_PRF_ENTRIES-5);
   localparam FCR_ZP = (`LG_PRF_ENTRIES-3);
   
   logic [`LG_PRF_ENTRIES-1:0]	rs = {{ZP{1'b0}},insn[25:21]};
   logic [`LG_PRF_ENTRIES-1:0] 	rt = {{ZP{1'b0}},insn[20:16]};
   logic [`LG_PRF_ENTRIES-1:0] 	rd = {{ZP{1'b0}},insn[15:11]};

   logic [`LG_PRF_ENTRIES-1:0] 	fs = {{ZP{1'b0}},insn[15:11]};
   logic [`LG_PRF_ENTRIES-1:0] 	ft = {{ZP{1'b0}},insn[20:16]};
   logic [`LG_PRF_ENTRIES-1:0] 	fd = {{ZP{1'b0}},insn[10:6]};

   
   logic [`LG_PRF_ENTRIES-1:0] shamt = {{ZP{1'b0}},insn[10:6]};
   
   always_comb
     begin
	uop.op = II;
	uop.srcA = 'd0;
	uop.srcB = 'd0;
	uop.srcC = 'd0;
	uop.dst = 'd0;
	uop.srcA_valid = 1'b0;
	uop.srcB_valid = 1'b0;
	uop.srcC_valid = 1'b0;
	uop.fp_srcA_valid = 1'b0;
	uop.fp_srcB_valid = 1'b0;
	uop.fp_srcC_valid = 1'b0;
	uop.fcr_dst_valid = 1'b0;
	uop.fcr_src_valid = 1'b0;
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
	uop.is_fp = 1'b0;
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
		 6'd1: /* movf and movt */
		   begin
		      if(rd == 'd0)
			begin
			   uop.op = NOP;
			   uop.dst_valid = 1'b0;
			end
		      else
			begin
			   uop.dst = rd;
			   uop.srcA = rs;
			   uop.srcA_valid = 1'b1;
			   uop.srcB = rd;
			   uop.srcB_valid = 1'b1;
			   uop.srcC = {{FCR_ZP{1'b0}}, insn[20:18]};
			   uop.fcr_src_valid = 1'b1;			   
			   uop.op = insn[16] ? MOVT : MOVF;
			   uop.dst_valid = 1'b1;
			end
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
		 6'd10: /* MOVZ */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.srcC = rd;
		      uop.srcC_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : MOVZ;
		      uop.is_int = 1'b1;
		   end
		 6'd11: /* MOVN */
		   begin
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.srcC = rd;
		      uop.srcC_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.op = (rd == 'd0) ? NOP : MOVN;
		      uop.is_int = 1'b1;
		   end // case: 6'd11
		 6'd12:
		   begin
		      uop.op = SYSCALL;
		      /* hacks for user-level emulation */
		      uop.srcA = 'd7;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = 'd2;
		      uop.srcB_valid = 1'b1;
		      uop.dst = 'd2;
		      uop.dst_valid = 1'b1;
		      /* serializing semantics */
		      uop.must_restart = 1'b1;
		      uop.serializing_op = 1'b1;
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
		      uop.op = SYNC;
		      uop.is_mem = 1'b1;
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
		      uop.op = MFC0;
		      uop.dst = rt;
		      uop.dst_valid = 1'b1;
		      uop.srcA = rd;
		      uop.is_int = 1'b1;
		      uop.serializing_op = 1'b1;
		      uop.must_restart = 1'b1;
		   end
		 5'd4: /* mtc0 */
		   begin
		      uop.op = MTC0;
		      uop.dst = rd;
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.serializing_op = 1'b1;
		      uop.has_delay_slot = 1'b0;
		      uop.is_int = 1'b1;
		      uop.must_restart = 1'b1;
		   end // case: 5'd4
		 default:
		   begin
		   end
	       endcase // case (insn[25:21])
	    end // case: 6'd16
	  6'd17: /* coproc1 */
	    begin
	       if((insn[25:21]==5'd8))
		 begin
		    //uop.srcA = {{`LG_PRF_ENTRIES-3}
		    //$display("decoding floating point branch instruction for %x", 
		    //uop.pc);
		    uop.fcr_src_valid = 1'b1;
		    uop.has_delay_slot = 1'b1;
		    uop.imm = insn[15:0];
		    uop.br_pred = insn_pred;
		    uop.is_br = 1'b1;
		    uop.is_int = 1'b1;
		    uop.srcC = {{FCR_ZP{1'b0}}, insn[20:18]};
		    //insn[17]; //likely
		    //insn[16]; //true or false
		    case(insn[17:16])
		      2'b00: //bc1f
			begin
			   uop.op = BC1F;
			end
		      2'b01: //bc1t
			begin
			   uop.op = BC1T;
			end
		      2'b10: //bc1fl;
			begin
			   uop.op = BC1FL;
			   uop.has_nullifying_delay_slot = 1'b1;
			end
		      2'b11: //bc1tl
			begin
			   uop.op = BC1TL;
			   uop.has_nullifying_delay_slot = 1'b1;
			end
		    endcase // case (insn[17:16])
		 end // if ((insn[25:21]==5'd8))
	       else if((insn[25:21]==5'd0) && (insn[10:0] == 11'd0))
		 begin /* mfc1 */
		    uop.dst = rt;
		    uop.dst_valid = 1'b1;
		    if(in_64b_fpreg_mode)		      
		      begin
			 uop.op = MFC1;			 
			 uop.srcB = rd;			 
		      end		    
		    else
		      begin
			 uop.op = MFC1_MERGE;
			 uop.srcB = {{ZP{1'b0}}, rd[4:1], 1'b0};
			 uop.jmp_imm = { {(`M_WIDTH-17){1'b0}}, rd[0]};
		      end
		    uop.fp_srcB_valid = 1'b1;
		    uop.is_mem = 1'b1;
		 end
	       else if((insn[25:21]==5'd4) && (insn[10:0] == 11'd0))
		 begin /* mtc1 */
		    uop.srcA = rt;
		    uop.srcA_valid = 1'b1;
		    if(in_64b_fpreg_mode)
		      begin
			 uop.op = MTC1;
			 uop.dst = rd;
		      end
		    else
		      begin
			 uop.op = MTC1_MERGE;
			 uop.dst = {{ZP{1'b0}}, rd[4:1], 1'b0};
			 uop.srcB = {{ZP{1'b0}}, rd[4:1], 1'b0};
			 uop.jmp_imm = { {(`M_WIDTH-17){1'b0}}, rd[0]};
			 uop.fp_srcB_valid = 1;			 
		      end
		    uop.fp_dst_valid = 1'b1;
		    uop.imm = insn[15:0];
		    uop.is_mem = 1'b1;
		 end // if ((insn[25:21]==5'd4) && (insn[10:0] == 11'd0))
	       else
		 begin
`ifdef ENABLE_FPU
		    case(insn[5:0])
		      6'd0:
			begin /* ADD.fmt */
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = ft;
			   uop.fp_srcB_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16: /* flt */
			       begin
				  uop.op = SP_ADD;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = !in_64b_fpreg_mode & ft[0];
				  uop.jmp_imm[3] = !in_64b_fpreg_mode & fd[0];
			       end
			     5'd17: /* dbl */
			       uop.op = DP_ADD;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			end // case: 6'd0
		      6'd1:
			begin /* SUB.fmt */
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = ft;
			   uop.fp_srcB_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16: /* flt */
			       begin
				  uop.op = SP_SUB;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = !in_64b_fpreg_mode & ft[0];
				  uop.jmp_imm[3] = !in_64b_fpreg_mode & fd[0];
			       end
			     5'd17: /* dbl */
			       uop.op = DP_SUB;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			end // case: 6'd0
		      6'd2: /* MUL.fmt */
			begin
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = ft;
			   uop.fp_srcB_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16: /* flt */
			       begin
				  uop.op = SP_MUL;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = !in_64b_fpreg_mode & ft[0];
				  uop.jmp_imm[3] = !in_64b_fpreg_mode & fd[0];
			       end
			     5'd17: /* dbl */
			       uop.op = DP_MUL;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			end // case: 6'd2
		      6'd3: /* div.fmt */
			begin
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = ft;
			   uop.fp_srcB_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16: /* flt */
			       begin
				  uop.op = SP_DIV;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = !in_64b_fpreg_mode & ft[0];
				  uop.jmp_imm[3] = !in_64b_fpreg_mode & fd[0];
			       end
			     5'd17: /* dbl */
			       uop.op = DP_DIV;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			end // case: 6'd3
		      6'd4: /* sqrt.fmt */
			begin
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16: /* flt */
			       begin
				  uop.op = SP_SQRT;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = 1'b0;
				  uop.jmp_imm[3] = !in_64b_fpreg_mode & fd[0];
			       end
			     5'd17: /* dbl */
			       uop.op = DP_SQRT;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			end
		      6'd6: /* MOV.fmt */
			begin
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16:
			       uop.op = SP_MOV;
			     5'd17:
			       uop.op = DP_MOV;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			end
		      6'd13: /* trunc.w */
			begin
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16: /* sp */
			       uop.op =  TRUNC_SP_W;
			     5'd17: /* dbl */
			       uop.op =  TRUNC_DP_W;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			   if(!in_64b_fpreg_mode)
			     begin
				uop.dst = {{ZP{1'b0}}, fd[4:1], 1'b0};
				uop.srcB = {{ZP{1'b0}}, fd[4:1], ~fd[0]};
				uop.srcC = {{ZP{1'b0}}, 4'd0, fd[0]};
				uop.fp_srcB_valid = 1;				
			     end
			   else
			     begin
				uop.dst = fd;				
			     end
			end // case: 6'd13
		      6'd17: /* fmovc */
			begin
			   uop.dst = fd;
			   if(fd[0])
			     begin
				uop.op = II;
			     end
			   else if(insn[16])
			     begin
				uop.op = FP_MOVT;
			     end
			   else
			     begin
				uop.op = FP_MOVF;
			     end
			   uop.fp_dst_valid = 1'b1;
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = fd;
			   uop.fp_srcB_valid = 1'b1;
			   uop.srcC = {{FCR_ZP{1'b0}}, insn[20:18]};
			   uop.fcr_src_valid = 1'b1;
			   uop.is_fp = 1'b1;
			end // case: 6'd17
 `ifdef FMOVZ_FMOVN
		      6'd18: /* fmovz */
			begin
			   uop.dst = fd;
			   uop.op = fd[0] ? II:  FP_MOVZ;
			   uop.fp_dst_valid = 1'b1;
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = fd;
			   uop.fp_srcB_valid = 1'b1;
			   uop.srcC = rt;
			   uop.srcC_valid = 1'b1;			   
			   uop.is_fp = 1'b1;
			end
		      6'd19: /* fmovn */
			begin
			   uop.dst = fd;
			   uop.op = fd[0] ? II : FP_MOVN;
			   uop.fp_dst_valid = 1'b1;
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = fd;
			   uop.fp_srcB_valid = 1'b1;
			   uop.srcC = rt;
			   uop.srcC_valid = 1'b1;			   
			   uop.is_fp = 1'b1;
			end // case: 6'd19
 `endif
		      6'd32: /* cvt.s */
			begin
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd17: /* dbl */
			       uop.op = CVT_DP_SP;
			     5'd20: /* int32 */
			       uop.op = CVT_W_SP;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			   if(!in_64b_fpreg_mode)
			     begin
				uop.dst = {{ZP{1'b0}}, fd[4:1], 1'b0};
				uop.srcA = {{ZP{1'b0}}, fs[4:1], 1'b0};
				uop.srcB = {{ZP{1'b0}}, fd[4:1], ~fd[0]};
				uop.srcC = {{ZP{1'b0}}, 3'd0, fs[0], fd[0]};
				uop.fp_srcB_valid = 1;				
			     end
			end // case: 6'd32
		      6'd33: /* cvt.d */
			begin
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.dst = fd;
			   uop.fp_dst_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   case(insn[25:21])
			     5'd16: /* flt */
			       uop.op = CVT_SP_DP;
			     5'd20: /* int32 */
			       uop.op = CVT_W_DP;
			     default:
			       uop.op = II;
			   endcase // case (insn[25:21])
			end
		      /* FP comparison begins here */
		      6'd48: /* c.fmt.f */
			begin
			end
		      6'd49: /* c.fmt.un */
			begin
			end
		      6'd50: /* c.fmt.eq */
			begin
			   case(insn[25:21])
			     5'd16:
			       begin
				  uop.op = SP_CMP_EQ;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = !in_64b_fpreg_mode & ft[0];
			       end
			     5'd17:
			       uop.op = DP_CMP_EQ;
			     default:
			       uop.op = II;
			   endcase
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = ft;
			   uop.fp_srcB_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   uop.fcr_dst_valid = 1'b1;
			   uop.fcr_src_valid = 1'b1;
			   //save cc
			   uop.imm = {13'd0, insn[10:8]};
			end
		      6'd51: /* c.fmt.ueq */
			begin
			end
		      6'd52: /* c.fmt.olt */
			begin
			end
		      6'd53: /* c.fmt.ult */
			begin
			end
		      6'd54: /* c.fmt.ole */
			begin
			end
		      6'd55: /* c.fmt.ule */
			begin
			end
		      6'd56: /* c.fmt.sf */
			begin
			end
		      6'd57: /* c.fmt.ngle */
			begin
			end
		      6'd58: /* c.fmt.seq */
			begin
			end
		      6'd59: /* c.fmt.ngl */
			begin
			end
		      6'd60: /* c.fmt.lt */
			begin
			   case(insn[25:21])
			     5'd16:
			       begin
				  uop.op = SP_CMP_LT;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = !in_64b_fpreg_mode & ft[0];
			       end
			     5'd17:
			       uop.op = DP_CMP_LT;
			     default:
			       uop.op = II;
			   endcase
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = ft;
			   uop.fp_srcB_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   uop.fcr_dst_valid = 1'b1;
			   uop.fcr_src_valid = 1'b1;
			   //save cc
			   uop.imm = {13'd0, insn[10:8]};
			end
		      6'd61: /* c.fmt.nge */
			begin
			end
		      6'd62: /* c.fmt.le */
			begin
			   case(insn[25:21])
			     5'd16:
			       begin
				  uop.op = SP_CMP_LE;
				  uop.jmp_imm[0] = 1'b1;
				  uop.jmp_imm[1] = !in_64b_fpreg_mode & fs[0];
				  uop.jmp_imm[2] = !in_64b_fpreg_mode & ft[0];
			       end
			     5'd17:
			       uop.op = DP_CMP_LE;
			     default:
			       uop.op = II;
			   endcase
			   uop.srcA = fs;
			   uop.fp_srcA_valid = 1'b1;
			   uop.srcB = ft;
			   uop.fp_srcB_valid = 1'b1;
			   uop.is_fp = 1'b1;
			   uop.fcr_dst_valid = 1'b1;
			   uop.fcr_src_valid = 1'b1;
			   //save cc
			   uop.imm = {13'd0, insn[10:8]};
			end
		      6'd63: /*c.fmt.ngt */
			begin
			end  
		      default:
			begin
			end
		    endcase // case (insn[5:0])
`else
`endif		    
		 end // else: !if((insn[25:21]==5'd4) && (insn[10:0] == 11'd0))
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
	  6'd28: /* SPECIAL2 */
	    begin
	       case(insn[5:0])
		 6'd0:
		   begin
		      uop.op = MADD;
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.hilo_dst_valid = 1'b1;
		      uop.hilo_src_valid = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd2:
		   begin
		      uop.op = (rd == 'd0) ? NOP : MUL;
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.is_int = 1'b1;
		   end // case: 6'd2
		 6'd4:
		   begin
		      uop.op = MSUB;
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;
		      uop.hilo_dst_valid = 1'b1;
		      uop.hilo_src_valid = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 6'd32:
		   begin
		      uop.op = (rd == 'd0) ? NOP : CLZ;
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.dst = rd;
		      uop.dst_valid = (rd != 'd0);
		      uop.is_int = 1'b1;
		   end
		 default:
		   begin
		   end
	       endcase
	    end // case: 6'd28
	  6'd31: /* special3 */
	    begin
	       case(insn[5:0])
		 6'd0: /* ext */
		   begin
		      uop.op = EXT;
		      uop.srcA = rs;
		      uop.srcA_valid = 1'b1;
		      uop.dst = rt;
		      uop.dst_valid = 1'b1;
		      uop.imm = insn[15:0];
		      uop.is_int = 1'b1;
		   end
		 6'd4:
		   begin /* ins */
		      uop.op = INS;
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.srcB = rs;
		      uop.srcB_valid = 1'b1;		      
		      uop.dst = rt;
		      uop.dst_valid = 1'b1;
		      uop.imm = insn[15:0];
		      uop.is_int = 1'b1;
		   end
		 6'd32:
		   begin
		      uop.op = insn[10:6] == 5'd16 ? SEB : SEH;
		      uop.dst = rd;
		      uop.dst_valid = 1'b1;
		      uop.srcA = rt;
		      uop.srcA_valid = 1'b1;
		      uop.is_int = 1'b1;
		   end
		 default:
		   begin
		   end
	       endcase // case (insn[5:0])
	    end // case: 6'd31
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
	  6'd49:
	    begin
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       //$display("lwc1 entering machine for pc %x", pc);
	       if(in_64b_fpreg_mode)
		 begin
		    uop.op = LWC1;		    
		    uop.dst = rt;
		 end
	       else
		 begin
		    uop.op = LWC1_MERGE;
		    uop.dst = {{ZP{1'b0}}, rt[4:1], 1'b0};
		    uop.srcB = {{ZP{1'b0}}, rt[4:1], 1'b0};
		    uop.fp_srcB_valid = 1;
		    uop.jmp_imm = { {(`M_WIDTH-17){1'b0}}, rt[0]};
		 end
	       uop.fp_dst_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  6'd51: /* PREF */
	    begin
	       uop.op = NOP;
	       uop.is_int = 1'b1;
	    end
	  6'd53: /* LDC1 */
	    begin
	       uop.op = LDC1;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.dst = rt;
	       uop.fp_dst_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	    end
	  // 6'd56: /* SC */
	  //   begin
	  //      uop.op = SC;
	  //      uop.dst = rt;
	  //      uop.dst_valid = 1'b1;
	  //      uop.srcA = rs;
	  //      uop.srcA_valid = 1'b1;
	  //      uop.srcB = rt;
	  //      uop.srcB_valid = 1'b1;
	  //      uop.imm = insn[15:0];
	  //      uop.is_mem = 1'b1;
	  //      uop.is_store = 1'b1;
	  //   end // case: 6'd56
	  
	  6'd57: /* SWC1 */
	    begin
	       if(in_64b_fpreg_mode)
		 begin
		    uop.op = SWC1;
		    uop.srcB = rt;		    
		 end
	       else
		 begin
		    uop.op = SWC1_MERGE;
		    uop.srcB = {{ZP{1'b0}}, rt[4:1], 1'b0};
		    uop.jmp_imm = { {(`M_WIDTH-17){1'b0}}, rt[0]};		    
		 end
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.fp_srcB_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	    end
	  6'd61: /* SDC1 */
	    begin
	       uop.op = SDC1;
	       uop.srcA = rs;
	       uop.srcA_valid = 1'b1;
	       uop.srcB = rt;
	       uop.fp_srcB_valid = 1'b1;
	       uop.imm = insn[15:0];
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	    end
	  default:
	    begin
	    end
	endcase // case (insn[5:0])
     end // always_comb
   

endmodule
   
