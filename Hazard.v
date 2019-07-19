/******************************************************************
* Description
*		This is the HazardUnit:
* Version:
*	1.0
* Author:
*Luis David Gallegos Godoy
* email:
*	is709571@iteso.mx
* Date:
*	17/07/2019
******************************************************************/
module Hazard
(
input [9:0] Instruction_ID,
input [4:0] RT_EX,
input MemRead_EX,
input RegWrite_wire_EX,
input RegWrite_wire_MEM,
input [4:0] WriteRegister_wire,
input [4:0] WriteRegister_wire_MEM,
output DWrite,
output PCWrite,
output Bubble

);
wire [4:0] RS;
wire [4:0] RT;
assign RS = Instruction_ID[9:5];
assign RT = Instruction_ID[4:0];

//(((RS || RT) == WriteRegister_wire) && RegWrite_wire == 1'b1) || (((RS || RT) == WriteRegister_wire_MEM) && RegWrite_wire_MEM == 1'b1)

assign DWrite = (MemRead_EX == 1'b1 && (RT_EX == RT || RT_EX == RS))? 1'b0: 1'b1;

assign PCWrite = DWrite;

assign Bubble = !PCWrite;


endmodule
