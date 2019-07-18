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
output DWrite,
output PCWrite,
output Bubble

);


assign DWrite = (MemRead_EX == 1'b1 && (RT_EX == Instruction_ID[4:0] || RT_EX == Instruction_ID[9:5]))? 1'b0: 1'b1;
assign PCWrite = (MemRead_EX == 1'b1 && (RT_EX == Instruction_ID[4:0] || RT_EX == Instruction_ID[9:5]))? 1'b0: 1'b1;
assign Bubble = (MemRead_EX == 1'b1 && (RT_EX == Instruction_ID[4:0] || RT_EX == Instruction_ID[9:5]))? 1'b1: 1'b0;


endmodule
