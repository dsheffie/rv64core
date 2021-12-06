   
module shift_right#(parameter LG_W=5)(y,is_signed, data, distance);
   localparam W = 1<<LG_W;
   input logic is_signed;
   input logic [W-1:0] data;
   input logic [LG_W-1:0] distance;
   output logic [W-1:0] y;
     
   logic 	       sb;
         
   always_comb
     begin
	y = 'd0;
	sb = is_signed ? data[W-1] : 1'b0;
	case(distance)
	  'd0:
	    y = data;
	  'd1:
	    y = {sb, data[31:1]};
	  'd2:
	    y = {{2{sb}}, data[31:2]};
	  'd3:
	    y = {{3{sb}}, data[31:3]};
	  'd4:
	    y = {{4{sb}}, data[31:4]};
	  'd5:
	    y = {{5{sb}}, data[31:5]};
	  'd6:
	    y = {{6{sb}}, data[31:6]};
	  'd7:
	    y = {{7{sb}}, data[31:7]};
	  'd8:
	    y = {{8{sb}}, data[31:8]};
	  'd9:
	    y = {{9{sb}}, data[31:9]};
	  'd10:
	    y = {{10{sb}}, data[31:10]};
	  'd11:
	    y = {{11{sb}}, data[31:11]};
	  'd12:
	    y = {{12{sb}}, data[31:12]};
	  'd13:
	    y = {{13{sb}}, data[31:13]};
	  'd14:
	    y = {{14{sb}}, data[31:14]};
	  'd15:
	    y = {{15{sb}}, data[31:15]};
	  'd16:
	    y = {{16{sb}}, data[31:16]};
	  'd17:
	    y = {{17{sb}}, data[31:17]};
	  'd18:
	    y = {{18{sb}}, data[31:18]};
	  'd19:
	    y = {{19{sb}}, data[31:19]};
	  'd20:
	    y = {{20{sb}}, data[31:20]};
	  'd21:
	    y = {{21{sb}}, data[31:21]};
	  'd22:
	    y = {{22{sb}}, data[31:22]};
	  'd23:
	    y = {{23{sb}}, data[31:23]};
	  'd24:
	    y = {{24{sb}}, data[31:24]};
	  'd25:
	    y = {{25{sb}}, data[31:25]};
	  'd26:
	    y = {{26{sb}}, data[31:26]};
	  'd27:
	    y = {{27{sb}}, data[31:27]};
	  'd28:
	    y = {{28{sb}}, data[31:28]};
	  'd29:
	    y = {{29{sb}}, data[31:29]};
	  'd30:
	    y = {{30{sb}}, data[31:30]};
	  'd31:
	    y = {{31{sb}}, data[31]};
	endcase // case (distance)
     end // always@ (*)
endmodule // shftRight
