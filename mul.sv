`include "uop.vh"

module csa(a,b,cin,s,cout);
   parameter N = 64;
   input [N-1:0] a;
   input [N-1:0] b;
   input [N-1:0] cin;
   output [N-1:0] s;
   output [N-1:0] cout;

   wire [N-1:0]   w_xor_ab = a^b;
   assign s = w_xor_ab ^ cin;
   assign cout = a&b | (cin & w_xor_ab);
endmodule // csa

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
   
   logic [`MUL_LAT:0] 			   r_complete;
   logic [`MUL_LAT:0] 			   r_do_madd;
   logic [`MUL_LAT:0] 			   r_do_msub;
   logic [`MUL_LAT:0] 			   r_hilo_val;
   logic [`LG_HILO_PRF_ENTRIES-1:0] 	   r_hilo_ptr[`MUL_LAT:0];
   logic [`MUL_LAT:0] 			   r_gpr_val;
   logic [`LG_PRF_ENTRIES-1:0] 	   r_gpr_ptr[`MUL_LAT:0];
   logic [63:0] 		   r_madd[`MUL_LAT:0];
   logic [`LG_ROB_ENTRIES-1:0] 	   r_rob_ptr[`MUL_LAT:0];
  
   wire [63:0] 			   r_mul31;   

   assign complete = r_complete[`MUL_LAT];
   assign rob_ptr_out = r_rob_ptr[`MUL_LAT];
   assign gpr_prf_ptr_val_out = r_gpr_val[`MUL_LAT];
   assign gpr_prf_ptr_out = r_gpr_ptr[`MUL_LAT];
   
   assign hilo_prf_ptr_val_out = r_hilo_val[`MUL_LAT];
   assign hilo_prf_ptr_out = r_hilo_ptr[`MUL_LAT];

   logic 			   t_signed;
   
   always_comb
     begin
	t_signed = opcode != MULTU;
	y = r_mul31;
     end // always_comb

   wire [63:0] w_pp_us[31:0];
   wire [63:0] w_pp[31:0];
   logic [63:0] r_pp[`MUL_LAT:0][31:0];
   
   //compute unsigned partial products
   generate
      for(genvar i = 0; i < 32; i=i+1)
	begin
	   assign w_pp_us[i] = { {(32-i){1'b0}}, (src_A & {32{src_B[i]}} ), {i{1'b0}} };
	end
      
      for(genvar i = 1; i < 32; i=i+1)
	begin
	   assign w_pp[i] = {w_pp_us[i][63:(32+i)], (t_signed ? ~w_pp_us[i][31+i] : w_pp_us[i][31+i]), w_pp_us[i][(30+i):0]};
	end
   endgenerate
   assign w_pp[0] = {w_pp_us[0][63:33], (t_signed ? {1'b1, ~w_pp_us[0][31]} : {1'b0, w_pp_us[0][31]} ), w_pp_us[0][30:0]};
   assign w_pp[31] = t_signed ? {1'b1, w_pp_us[31][62], ~w_pp_us[31][61:31], 31'd0} : w_pp_us[31];

   always_ff@(posedge clk)
     begin
	r_pp[0] <= w_pp;
	for(integer i = 1; i < `MUL_LAT; i=i+1)
	  begin
	     r_pp[i] <= r_pp[i-1];
	  end
     end

   
   wire [63:0] w_c0, w_s0;
   csa #(.N(64)) c0 (.a(w_pp[0]), .b(w_pp[1]), .cin(w_pp[2]), .s(w_s0), .cout(w_c0));
   wire [63:0] w_c1, w_s1;
   csa #(.N(64)) c1 (.a(w_s0), .b({w_c0[62:0], 1'b0}), .cin(w_pp[3]), .s(w_s1), .cout(w_c1));
   wire [63:0] w_c2, w_s2;
   csa #(.N(64)) c2 (.a(w_s1), .b({w_c1[62:0], 1'b0}), .cin(w_pp[4]), .s(w_s2), .cout(w_c2));
   wire [63:0] w_c3, w_s3;
   csa #(.N(64)) c3 (.a(w_s2), .b({w_c2[62:0], 1'b0}), .cin(w_pp[5]), .s(w_s3), .cout(w_c3));
   wire [63:0] w_c4, w_s4;
   csa #(.N(64)) c4 (.a(w_s3), .b({w_c3[62:0], 1'b0}), .cin(w_pp[6]), .s(w_s4), .cout(w_c4));
   wire [63:0] w_c5, w_s5;
   csa #(.N(64)) c5 (.a(w_s4), .b({w_c4[62:0], 1'b0}), .cin(w_pp[7]), .s(w_s5), .cout(w_c5));
   wire [63:0] w_c6, w_s6;
   csa #(.N(64)) c6 (.a(w_s5), .b({w_c5[62:0], 1'b0}), .cin(w_pp[8]), .s(w_s6), .cout(w_c6));
   wire [63:0] w_c7, w_s7;
   csa #(.N(64)) c7 (.a(w_s6), .b({w_c6[62:0], 1'b0}), .cin(w_pp[9]), .s(w_s7), .cout(w_c7));
   wire [63:0] w_c8, w_s8;
   csa #(.N(64)) c8 (.a(w_s7), .b({w_c7[62:0], 1'b0}), .cin(w_pp[10]), .s(w_s8), .cout(w_c8));
   wire [63:0] w_c9, w_s9;
   csa #(.N(64)) c9 (.a(w_s8), .b({w_c8[62:0], 1'b0}), .cin(w_pp[11]), .s(w_s9), .cout(w_c9));
   wire [63:0] w_c10, w_s10;
   csa #(.N(64)) c10 (.a(w_s9), .b({w_c9[62:0], 1'b0}), .cin(w_pp[12]), .s(w_s10), .cout(w_c10));
   wire [63:0] w_c11, w_s11;
   csa #(.N(64)) c11 (.a(w_s10), .b({w_c10[62:0], 1'b0}), .cin(w_pp[13]), .s(w_s11), .cout(w_c11));

   wire [63:0] r_s11, r_c11;
   ff #(.N(64)) ff_s0 (.clk(clk), .d(w_s11), .q(r_s11));
   ff #(.N(64)) ff_c0 (.clk(clk), .d(w_c11), .q(r_c11));
      
   wire [63:0] w_c12, w_s12;
   csa #(.N(64)) c12 (.a(r_s11), .b({r_c11[62:0], 1'b0}), .cin(r_pp[0][14]), .s(w_s12), .cout(w_c12));
   wire [63:0] w_c13, w_s13;
   csa #(.N(64)) c13 (.a(w_s12), .b({w_c12[62:0], 1'b0}), .cin(r_pp[0][15]), .s(w_s13), .cout(w_c13));
   wire [63:0] w_c14, w_s14;
   csa #(.N(64)) c14 (.a(w_s13), .b({w_c13[62:0], 1'b0}), .cin(r_pp[0][16]), .s(w_s14), .cout(w_c14));
   wire [63:0] w_c15, w_s15;
   csa #(.N(64)) c15 (.a(w_s14), .b({w_c14[62:0], 1'b0}), .cin(r_pp[0][17]), .s(w_s15), .cout(w_c15));
   wire [63:0] w_c16, w_s16;
   csa #(.N(64)) c16 (.a(w_s15), .b({w_c15[62:0], 1'b0}), .cin(r_pp[0][18]), .s(w_s16), .cout(w_c16));
   wire [63:0] w_c17, w_s17;
   csa #(.N(64)) c17 (.a(w_s16), .b({w_c16[62:0], 1'b0}), .cin(r_pp[0][19]), .s(w_s17), .cout(w_c17));
   wire [63:0] w_c18, w_s18;
   csa #(.N(64)) c18 (.a(w_s17), .b({w_c17[62:0], 1'b0}), .cin(r_pp[0][20]), .s(w_s18), .cout(w_c18));
   wire [63:0] w_c19, w_s19;
   csa #(.N(64)) c19 (.a(w_s18), .b({w_c18[62:0], 1'b0}), .cin(r_pp[0][21]), .s(w_s19), .cout(w_c19));
   wire [63:0] w_c20, w_s20;
   csa #(.N(64)) c20 (.a(w_s19), .b({w_c19[62:0], 1'b0}), .cin(r_pp[0][22]), .s(w_s20), .cout(w_c20));
   wire [63:0] w_c21, w_s21;
   csa #(.N(64)) c21 (.a(w_s20), .b({w_c20[62:0], 1'b0}), .cin(r_pp[0][23]), .s(w_s21), .cout(w_c21));
   wire [63:0] w_c22, w_s22;
   csa #(.N(64)) c22 (.a(w_s21), .b({w_c21[62:0], 1'b0}), .cin(r_pp[0][24]), .s(w_s22), .cout(w_c22));

   wire [63:0] r_s22, r_c22;
   ff #(.N(64)) ff_s1 (.clk(clk), .d(w_s22), .q(r_s22));
   ff #(.N(64)) ff_c1 (.clk(clk), .d(w_c22), .q(r_c22));
   
   wire [63:0] w_c23, w_s23;
   csa #(.N(64)) c23 (.a(r_s22), .b({r_c22[62:0], 1'b0}), .cin(r_pp[1][25]), .s(w_s23), .cout(w_c23));
   wire [63:0] w_c24, w_s24;
   csa #(.N(64)) c24 (.a(w_s23), .b({w_c23[62:0], 1'b0}), .cin(r_pp[1][26]), .s(w_s24), .cout(w_c24));
   wire [63:0] w_c25, w_s25;
   csa #(.N(64)) c25 (.a(w_s24), .b({w_c24[62:0], 1'b0}), .cin(r_pp[1][27]), .s(w_s25), .cout(w_c25));
   wire [63:0] w_c26, w_s26;
   csa #(.N(64)) c26 (.a(w_s25), .b({w_c25[62:0], 1'b0}), .cin(r_pp[1][28]), .s(w_s26), .cout(w_c26));
   wire [63:0] w_c27, w_s27;
   csa #(.N(64)) c27 (.a(w_s26), .b({w_c26[62:0], 1'b0}), .cin(r_pp[1][29]), .s(w_s27), .cout(w_c27));
   wire [63:0] w_c28, w_s28;
   csa #(.N(64)) c28 (.a(w_s27), .b({w_c27[62:0], 1'b0}), .cin(r_pp[1][30]), .s(w_s28), .cout(w_c28));
   wire [63:0] w_c29, w_s29;
   csa #(.N(64)) c29 (.a(w_s28), .b({w_c28[62:0], 1'b0}), .cin(r_pp[1][31]), .s(w_s29), .cout(w_c29));


   wire [63:0] w_mul31 = w_s29 + {w_c29[62:0], 1'b0};


   ff #(.N(64)) s0 (.clk(clk), .d(w_mul31), .q(r_mul31));
   
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     for(integer i = 0; i <= `MUL_LAT; i=i+1)
	       begin
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
