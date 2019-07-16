
module ForwardingUnit
(
	input [4:0] WB_WB,
	input [4:0] WB_MEM,
	input [4:0] RS_EX,
	input [4:0] RT_EX,
	input [4:0] RTorRD_MEM,
	input [4:0] RTorRD_WB,
	output [1:0] A,
	output [1:0] B
	
);

assign A = 	(RS_EX == RTorRD_MEM && WB_MEM == 1'b1)?		2'b10:
				(RS_EX == RTorRD_WB && WB_WB == 1'b1)?			2'b11:
																			2'b00;

assign B = 	(RT_EX == RTorRD_MEM && WB_MEM == 1'b1)?		2'b10:
				(RT_EX == RTorRD_WB && WB_WB == 1'b1)?			2'b11:
																			2'b00;

endmodule
//muxregfile//



