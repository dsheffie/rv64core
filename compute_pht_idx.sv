`include "machine.vh"

module compute_pht_idx(pc, hist, idx);
   input logic [`M_WIDTH-1:0] pc;
   input logic [`GBL_HIST_LEN-1:0] hist;
   output logic [`LG_PHT_SZ-1:0]   idx;

   assign idx = hist ^ pc[17:2] /*^ pc[33:18]) ^ pc[49:34]*/;
   
endmodule
