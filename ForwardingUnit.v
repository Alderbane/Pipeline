
module ForwardingUnit
#(
	parameter N=32
)
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

assign A = (1)? 0:0;


endmodule
//muxregfile//



