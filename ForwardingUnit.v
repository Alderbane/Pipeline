
module ForwardingUnit
(
	input WB_WB,
	input WB_MEM,
	input [4:0] RS_EX,
	input [4:0] RT_EX,
	input [4:0] RTorRD_MEM,
	input [4:0] RTorRD_WB,
	output [1:0] A,
	output [1:0] B

);

assign A = 	(WB_MEM == 1'b1 &&  RTorRD_MEM != 0 && RS_EX == RTorRD_MEM)?										2'b10:
				(WB_WB == 1'b1 && RTorRD_WB != 0 && RTorRD_MEM != RS_EX && RS_EX == RTorRD_WB)?			2'b11:
																																														2'b00;

assign B = 	(WB_MEM == 1'b1 && RTorRD_MEM != 0 && RT_EX == RTorRD_MEM)?	    								2'b10:
				(RT_EX == RTorRD_WB && WB_WB == 1'b1 && RTorRD_MEM != RT_EX && RTorRD_WB != 0)?			2'b11:
																																														2'b00;

endmodule
//muxregfile//
