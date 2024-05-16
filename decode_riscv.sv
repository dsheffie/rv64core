`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

function csr_t decode_csr(logic [11:0] csr);
   csr_t   x;
   case(csr)
     12'h100:
       x= SSTATUS;
     12'h104:
       x = SIE;
     12'h105:
       x = STVEC;
     12'h106:
       x = SCOUNTEREN;
     12'h140:
       x = SSCRATCH;
     12'h141:
       x = SEPC;
     12'h142:
       x = SCAUSE;
     12'h143:
       x = STVAL;
     12'h144:
       x = SIP;
     12'h180:
       x = SATP;
     12'h300:
       x = MSTATUS;
     12'h301: /* misa */
       x = MISA;
     12'h302:
       x = MEDELEG;
     12'h303:
       x = MIDELEG;
     12'h304:
       x = MIE;
     12'h305:
       x = MTVEC;
     12'h306:
       x = MCOUNTEREN;
     12'h340:
       x = MSCRATCH;
     12'h341:
       x = MEPC;
     12'h342:
       x = MCAUSE;
     12'h343:
       x = MTVEC;
     12'h344:
       x = MIP;
     12'h3a0:
       x = PMPCFG0;
     12'h3b0:
       x = PMPADDR0;
     12'h3b1:
       x = PMPADDR1;
     12'h3b2:
       x = PMPADDR2;      
     12'h3b3:
       x = PMPADDR3;      
     12'hc00:
       x = RDCYCLE_CSR;
     12'hc01:
       x = RDTIME_CSR;
     12'hc02:
       x = RDINSTRET_CSR;
     12'hc03:
       x = RDBRANCH_CSR;
     12'hc04:
       x = RDFAULTEDBRANCH_CSR;
     12'hf14:
       x = MHARTID;
     default:
       x = BADCSR;
   endcase // case (op)
   return x;
endfunction // is_store



module decode_riscv(
		    mode64,
		    insn,
		    page_fault,
		    irq,
		    pc, 
		    insn_pred, 
		    pht_idx, 
		    insn_pred_target,
`ifdef ENABLE_CYCLE_ACCOUNTING   		     
		    fetch_cycle,
`endif
		    syscall_emu,
		    uop);

   input logic mode64;
   input logic [31:0] insn;
   input logic	      page_fault;
   input logic 	      irq;
   input logic [`M_WIDTH-1:0] pc;
   input logic 	      insn_pred;
   input logic [`LG_PHT_SZ-1:0] pht_idx;
   input logic [`M_WIDTH-1:0] 	insn_pred_target;
`ifdef ENABLE_CYCLE_ACCOUNTING   
   input logic [63:0] 		fetch_cycle;
`endif
   input logic			syscall_emu;
   output 	uop_t uop;

   wire [6:0] 	opcode = page_fault|irq ? 'd0 : insn[6:0];
   
   localparam ZP = (`LG_PRF_ENTRIES-5);
   wire [`LG_PRF_ENTRIES-1:0] rd = {{ZP{1'b0}},insn[11:7]};
   wire [`LG_PRF_ENTRIES-1:0] rs1 = {{ZP{1'b0}},insn[19:15]};
   wire [`LG_PRF_ENTRIES-1:0] rs2 = {{ZP{1'b0}},insn[24:20]};
   
   logic 	rd_is_link, rs1_is_link;   
   

   wire [`LG_PRF_ENTRIES-1:0] 	rt = {{ZP{1'b0}},insn[20:16]};
   wire [`LG_PRF_ENTRIES-1:0] 	shamt = {{ZP{1'b0}},insn[10:6]};

   logic [`M_WIDTH-1:0] 		t_imm;
   localparam PP = (`M_WIDTH-32);
   csr_t csr_id;   
   wire [`M_WIDTH-1:0] 		w_pc_imm, w_pc_imm_;
   always_comb
     begin

	t_imm = 'd0;
	case(opcode)
	  7'h17: /* auipc */
	    begin
	       t_imm = {{PP{insn[31]}}, insn[31:12], 12'd0};
	    end
	  7'h63: /* branches */
	    begin
	       t_imm = {{(19+PP){insn[31]}}, insn[31], insn[7], insn[30:25], insn[11:8], 1'b0};
	    end
	  7'h6f: /* jal and j */
	    begin
	       t_imm = {{(11+PP){insn[31]}}, insn[31], insn[19:12], insn[20], insn[30:21], 1'b0}; 	    end
	  default:
	    begin
	    end
	  endcase
     end


   mwidth_add imm_add (.A(pc), .B(t_imm), .Y(w_pc_imm_));
   assign w_pc_imm = mode64 ? w_pc_imm_ : {{32{w_pc_imm_[31]}}, w_pc_imm_[31:0]};
		    
   always_comb
     begin
	csr_id = decode_csr(insn[31:20]);
	rd_is_link = (rd == 'd1) || (rd == 'd5);
	rs1_is_link = (rs1 == 'd1) || (rs1 == 'd5);
	uop.op = page_fault ? FETCH_PF : (irq ? IRQ : II);
	uop.srcA = 'd0;
	uop.srcB = 'd0;
	uop.dst = 'd0;
	uop.srcA_valid = 1'b0;
	uop.srcB_valid = 1'b0;
	
	uop.dst_valid = 1'b0;
	
	uop.imm = 16'd0;
	uop.jmp_imm = {(`M_WIDTH-16){1'b0}};
	uop.rvimm = 'd0;
	
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
	uop.raw_insn = insn;
`endif
	case(opcode)
	  7'h3:
	    begin
	       uop.dst = rd;
	       uop.srcA = rs1;
	       uop.dst_valid = (rd != 'd0);
	       uop.srcA_valid = 1'b1;
	       uop.is_mem = 1'b1;
	       uop.rvimm = {{(20+PP){insn[31]}}, insn[31:20]};
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
		 3'd3:
		   begin
		      uop.op = LD;
		   end
		 3'd4:
		   begin
		      uop.op = LBU;
		   end
		 3'd5:
		   begin
		      uop.op = LHU;
		   end
		 3'd6:
		   begin
		      uop.op = LWU;
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
	       uop.rvimm = {{(20+PP){insn[31]}}, insn[31:20]};
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
		      case(insn[31:26])
			6'h0:
			  begin
			     uop.op = (rd == 'd0) ? NOP : SRLI;
			     uop.is_cheap_int = 1'b1;
			  end
			6'h10:
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
	  7'h1b:
	    begin
	       if(mode64)
		 begin
		    uop.dst = rd;
		    uop.srcA = rs1;
		    uop.dst_valid = (rd != 'd0);
		    uop.srcA_valid = (rd != 'd0);	       	       
		    uop.is_int = 1'b1;
		    uop.rvimm = {{(20+PP){insn[31]}}, insn[31:20]};
		    case(insn[14:12])
		      3'd0: /* addiw */
			begin
			   uop.op = (rd == 'd0) ? NOP : ADDIW;
			   uop.is_cheap_int = 1'b1;
			end
		      3'd1: /* slliw */
			begin
			   uop.op = (rd == 'd0) ? NOP : SLLIW;
			   uop.is_cheap_int = 1'b1;
			end
		      3'd5: /* sraiw */
			begin
			   uop.op = (rd == 'd0) ? NOP : 
				    (insn[31:26] == 'd0) ? SRLIW : 
				    SRAIW;
			   uop.is_cheap_int = 1'b1;		      
			end
		      default:
			begin
			end
		    endcase
		 end // if (mode64)
	    end // case: 7'h1b
	  7'h23:
	    begin
	       uop.srcA = rs1;
	       uop.srcB = rs2;
	       uop.srcA_valid = 1'b1;
	       uop.srcB_valid = 1'b1;
	       uop.is_mem = 1'b1;
	       uop.is_store = 1'b1;
	       uop.rvimm = {{(20+PP){insn[31]}}, insn[31:25], insn[11:7]};
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
		 3'd3:
		   begin
		      uop.op = SD;
		   end
		 default:
		   begin
		   end
	       endcase
	    end // case: 7'h23
	  7'h2f:
	    begin
	       if(insn[14:12] == 3'd2 || insn[14:12] == 3'd3)
		 begin
		    uop.srcA = rs1;
		    uop.srcB = rs2;		    
		    uop.dst = rd;		    
		    uop.dst_valid = (rd != 'd0);
		    uop.srcA_valid = 1'b1;
		    uop.is_mem = 1'b1;
		    uop.jmp_imm = {43'd0, insn[31:27]};
		    uop.serializing_op = 1'b1;
		    case(insn[31:27])
		      5'd0:
			begin /* amoadd */
			   uop.op = insn[14:12]==3'd2 ? AMOW : AMOD;
			   uop.srcB_valid = 1'b1;
			end
		      5'd1:
			begin /* amoswap */
			   uop.op = insn[14:12]==3'd2 ? AMOW : AMOD;
			   uop.srcB_valid = 1'b1;			   
			end		      
		      5'd2: /* load linked - punt */
			begin
			   uop.op = insn[14:12]==3'd2 ? LW : LD;
			end
		      5'd3: /* store conditional */
			begin
			   uop.op = insn[14:12]==3'd2 ? SCW : SCD;
			   uop.srcB_valid = 1'b1;			   
			end
		      5'd8:
			begin /* amoor */
			   uop.op = insn[14:12]==3'd2 ? AMOW : AMOD;
			   uop.srcB_valid = 1'b1;			   
			end		      		      
		      5'd12:
			begin /* amoand */
			   uop.op = insn[14:12]==3'd2 ? AMOW : AMOD;
			   uop.srcB_valid = 1'b1;			   
			end
		      5'd28:
			begin /* amomaxu */
			   uop.op = insn[14:12]==3'd2 ? AMOW : AMOD;
			   uop.srcB_valid = 1'b1;			   
			end
		      default:
			begin
			end
		    endcase // case (insn[31:27])
		 end
	    end // case: 7'h2f
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
	       uop.rvimm = { {PP{insn[31]}}, insn[31:12], 12'd0};
	    end
	  7'h3b:
	    begin
	       if(mode64)
		 begin
		    uop.dst = rd;
		    uop.dst_valid = (rd != 'd0);
		    uop.srcA_valid = 1'b1;
		    uop.srcA = rs1;
		    uop.srcB_valid = 1'b1;
		    uop.srcB = rs2;
		    uop.is_int = 1'b1;	       
		    if(insn[14:12] == 'd0 && insn[31:25] == 'd0)
		      begin
			 uop.op = (rd != 'd0) ? ADDW : NOP;
			 uop.is_cheap_int = 1'b1;		    
		      end
		    else if(insn[14:12] == 'd0 && insn[31:25] == 'd32)
		      begin
			 uop.op = (rd != 'd0) ? SUBW : NOP;
			 uop.is_cheap_int = 1'b1;		    
		      end
		    else if(insn[14:12] == 'd0 && insn[31:25] == 'd1)
		      begin
			 uop.op = (rd != 'd0) ? MULW : NOP;
		      end
		    else if(insn[14:12] == 'd1 && insn[31:25] == 'd0)
		      begin
			 uop.op = (rd != 'd0) ? SLLW : NOP;
			 uop.is_cheap_int = 1'b1;
		      end
		    else if(insn[14:12] == 'd4 && insn[31:25] == 'd1)
		      begin
			 uop.op = (rd != 'd0) ? DIVW : NOP;
		      end
		    else if(insn[14:12] == 'd5 && insn[31:25] == 'd0)
		      begin
			 uop.op = (rd != 'd0) ? SRLW : NOP;
			 uop.is_cheap_int = 1'b1;			 
		      end		    
		    else if(insn[14:12] == 'd5 && insn[31:25] == 'd1)
		      begin
			 uop.op = (rd != 'd0) ? DIVUW : NOP;
		      end
		    else if(insn[14:12] == 'd5 && insn[31:25] == 'd32)
		      begin
			 uop.op = (rd != 'd0) ? SRAW : NOP;
			 uop.is_cheap_int = 1'b1;			 
		      end		    
		    else if(insn[14:12] == 'd6 && insn[31:25] == 'd1)
		      begin
			 uop.op = (rd != 'd0) ? REMW : NOP;
		      end
		    else if(insn[14:12] == 'd7 && insn[31:25] == 'd1)
		      begin
			 uop.op = (rd != 'd0) ? REMUW : NOP;
		      end
		 end // if (mode64)
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
	       uop.rvimm = {{(20+PP){insn[31]}}, insn[31:20]};
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
	       if(insn[31:7] == 'd0) /* treat as brk */
		 begin /* ecall */
		    uop.op = syscall_emu ? BREAK : ECALL;
		    uop.serializing_op = 1'b1;
		    uop.must_restart = (syscall_emu == 1'b0);
		 end
	       else if(insn[31:20] == 'd1 && insn[19:7] == 'd0)
		 begin /* ebreak */
		    uop.op = syscall_emu ? MONITOR : EBREAK;
		    uop.serializing_op = 1'b1;
		    uop.must_restart = 1'b1;		    
		 end
	       else if ((insn[31:25] == 'd9) && (rd == 'd0))
		 begin
		    uop.op = SFENCEVMA;
		    uop.serializing_op = 1'b1;
		    uop.must_restart = 1'b1;
		 end
	       else if((insn[31:20] == 12'h002) && insn[19:7] == 'd0)
		 begin
		    uop.op = II; //URET
		 end
	       else if((insn[31:20] == 12'h102) && insn[19:7] == 'd0)
		 begin
		    uop.op = SRET;
		    uop.serializing_op = 1'b1;
		    uop.must_restart = 1'b1;
		    uop.imm = {10'd0, MSTATUS};			     
		 end
	       else if((insn[31:20] == 12'h105) && insn[19:7] == 'd0)
		 begin /* WFI - treat as nop */
		    uop.op = NOP;
		 end	       
	       else if((insn[31:20] == 12'h202) && insn[19:7] == 'd0)
		 begin
		    uop.op = II; //HRET
		 end	       
	       else if((insn[31:20] == 12'h302) && insn[19:7] == 'd0)
		 begin
		    uop.op = MRET;
		    uop.serializing_op = 1'b1;
		    uop.must_restart = 1'b1;
		    uop.imm = {10'd0, MSTATUS};
		 end
	       else		 
		 begin
		    uop.imm = {5'd0, rs1[4:0], csr_id};

		    if(csr_id != BADCSR)
		      begin
			 case(insn[14:12])
			   3'd1: /* CRRRW */
			     begin
				uop.op = CSRRW;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
				uop.must_restart = 1'b1;
				uop.srcA = rs1;
				uop.srcA_valid = 1'b1;
			     end
			   3'd2: /*CSRRS */
			     begin
				if(rs1 == 'd0) /* fastpath */
				  begin
				     if(csr_id == MHARTID)
				       begin /* mhartid - always 0 */
					  uop.op = (rd == 'd0) ? NOP : ADDU;
					  uop.dst = rd;
					  uop.dst_valid = (rd != 'd0);
					  uop.srcA = 'd0;
					  uop.srcA_valid = 1'b1;
					  uop.srcB = 'd0;
					  uop.srcB_valid = 1'b1;				     
					  uop.is_cheap_int = 1'b1;
				       end
				     else if(csr_id == RDCYCLE_CSR)
				       begin
					  uop.op = (rd == 'd0) ? NOP : RDCYCLE;
					  uop.dst = rd;
					  uop.dst_valid = (rd != 'd0);
					  uop.serializing_op = 1'b1;				
				       end
				     else if(csr_id == RDINSTRET_CSR)
				       begin
					  uop.op = (rd == 'd0) ? NOP : RDINSTRET;
					  uop.dst = rd;
					  uop.dst_valid = (rd != 'd0);
					  uop.serializing_op = 1'b1;
				       end
				     // else if(csr_id == RDBRANCH_CSR)
				     //   begin
				     // 	  uop.op = (rd == 'd0) ? NOP : RDBRANCH;
				     // 	  uop.dst = rd;
				     // 	  uop.dst_valid = (rd != 'd0);
				     // 	  uop.serializing_op = 1'b1;
				     //   end
				     else if(csr_id == RDFAULTEDBRANCH_CSR)
				       begin
					  uop.op = (rd == 'd0) ? NOP : RDFAULTEDBRANCH;
					  uop.dst = rd;
					  uop.dst_valid = (rd != 'd0);
					  uop.serializing_op = 1'b1;
				       end
				     else
				       begin
					  uop.op = CSRRS;
					  uop.dst = rd;
					  uop.dst_valid = (rd != 'd0);
					  uop.serializing_op = 1'b1;
					  uop.must_restart = 1'b1;
				       end
				  end // if (insn[19:15] == 5'd0)
				else
				  begin
				     uop.op = CSRRS;
				     uop.dst = rd;
				     uop.dst_valid = (rd != 'd0);
				     uop.serializing_op = 1'b1;
				     uop.must_restart = 1'b1;
				     uop.srcA = rs1;
				     uop.srcA_valid = 1'b1;
				  end			   
			     end // case: 3'd2
			   3'd3: /* CRRRC */
			     begin
				uop.op = CSRRC;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
				uop.must_restart = 1'b1;
				uop.srcA = rs1;
				uop.srcA_valid = 1'b1;
			     end
			   3'd5: /* CRRRWI */
			     begin
				uop.op = CSRRWI;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
				uop.must_restart = 1'b1;
			     end
			   3'd6: /* CRRRSI */
			     begin
				uop.op = CSRRSI;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
				uop.must_restart = 1'b1;
			     end		      
			   3'd7: /* CRRRSI */
			     begin
				uop.op = CSRRCI;
				uop.dst = rd;
				uop.dst_valid = (rd != 'd0);
				uop.serializing_op = 1'b1;
				uop.must_restart = 1'b1;
			     end		      
			   default:
			     begin
			     end
			 endcase // case (insn[14:12])
		      end // if (csr_id != BADCSR)
		    //$display("pc %x, sel %x, rs1 %x, csr_id %x, generated op %d, nop %b", pc, insn[14:12], rs1, csr_id, uop.op, uop.op==NOP);		    
		 end // else: !if((insn[31:20] == 12'h302) && insn[19:7] == 'd0)
	    end
	  default:
	    begin
	    end
	endcase // case (opcode)
     end // always_comb
endmodule
   
