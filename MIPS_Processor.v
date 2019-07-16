/**********************
* Description
*	This is the top-level of a MIPS processor
* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer organization class at ITESO.
**********************/


module MIPS_Processor
#(
	parameter MEMORY_DEPTH = 64
)

(
	// Inputs
	input clk,
	input reset,
	input [7:0] PortIn,
	// Output
	output [31:0] ALUResultOut,
	output [31:0] PortOut
);
//**********************/
//**********************/
assign  PortOut = 0;

//**********************/
//**********************/
// Data types to connect modules
wire ALUSrc_wire;
wire BranchNE_wire;
wire BranchEQ_wire;
wire Jump_wire;
wire JR_wire;
wire MemRead_wire;
wire MemWrite_wire;
wire MemtoReg_wire;
wire NotZeroANDBrachNE;
wire ORForBranch;
wire RegDst_wire;
wire RegWrite_wire;
wire RegWriteORJAL_wire;
wire ZeroANDBrachEQ;
wire Zero_wire;
wire ALUSrc_wire_EX;
wire RegWrite_wire_EX;
wire Jump_wire_EX;
wire MemRead_wire_EX;
wire MemtoReg_wire_EX;
wire MemWrite_wire_EX;
wire RegDst_wire_EX;
wire BranchNE_wire_EX;
wire BranchEQ_wire_EX;

wire [3:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [3:0] ALUOp_wire_EX;
wire [4:0] WriteRegister_wire;
wire [4:0] RAorWriteReg_wire;
wire [4:0] RT_I_wire_EX;
wire [4:0] RD_R_wire_EX;
wire [4:0] shamt_EX;


wire [31:0] JumpAddr_EX;
wire [31:0] PC_4_wire_EX;
wire [31:0] ReadData1_wire_EX;
wire [31:0] ReadData2_wire_EX;
wire [31:0] InmmediateExtend_wire_EX;
wire [31:0] ALUResult_wire;
wire [31:0] BranchToPC_wire;
wire [31:0] BranchAddrSh2_wire;	//Branch address shifted 2 bits
wire [31:0] BranchOrPC4_wire;
wire [31:0] Instruction_wire;
wire [31:0] Instruction_wire_ID;
wire [31:0] InmmediateExtend_wire;
wire [31:0] JumpOrPC4OrBranch_wire;
wire [31:0] JumpAddrSh2_wire; //Jump address shifted 2 bits
wire [31:0] JAL_Address_or_ALU_Result_wire;
wire [31:0] JumpAddr;
wire [31:0] JOrPC4OrBranchOrJR_wire;
wire [31:0] LinkOrWord_wire;
wire [31:0] MUX_PC_wire;
wire [31:0] MemOut_wire;
wire [31:0] MemOrAlu_wire;
wire [31:0] MemoryAddressx4_wire;
wire [31:0] MemoryAddress_wire;
wire [31:0] PC_wire;
wire [31:0] PC_4_wire;
wire [31:0] PC_4_wire_ID;
wire [31:0] ReadData1_wire;
wire [31:0] ReadData2_wire;
wire [31:0] ReadData2OrInmmediate_wire;

wire [173:0] MEM_wire;
wire [31:0] ReadData1_wire_MEM;
wire JR_wire_MEM;
wire [31:0] JumpAddr_MEM;
wire BranchNE_wire_MEM;
wire BranchEQ_wire_MEM;
wire RegWrite_wire_MEM;
wire Jump_wire_MEM;
wire MemRead_wire_MEM;
wire MemtoReg_wire_MEM;
wire MemWrite_wire_MEM;
wire [31:0] BranchToPC_wire_MEM;
wire Zero_wire_MEM;
wire [31:0] ALUResult_wire_MEM;
wire [31:0] ReadData2_wire_MEM;
wire [4:0] WriteRegister_wire_MEM;

wire [103:0] WB_wire;
wire MemtoReg_wire_WB;
wire [31:0] JOrPC4OrBranchOrJR_wire_WB;
wire RegWrite_wire_WB;
wire Jump_wire_WB;
wire [31:0] MemOut_wire_WB;
wire [31:0] ALUResult_wire_WB;
wire [4:0] WriteRegister_wire_WB;


wire [63:0] ID_wire;
wire [187:0] EX_wire;

integer ALUStatus;


//**********************/
//**********************/

ANDGate
Gate_BranchEQANDZero
(
	.A(BranchEQ_wire_MEM),
	.B(Zero_wire_MEM), //bit menos significativo del opcode porque J 000010 y JAL 000011
	.C(ZeroANDBrachEQ)
);

//**********************/

ANDGate
Gate_BranchNEANDZero
(
	.A(BranchNE_wire_MEM),
	.B(!Zero_wire_MEM), //Si zero es diferente de 1,, significa que es diferente
	.C(NotZeroANDBrachNE)
);

//**********************/

ORGate
Gate_BeqOrBNE
(
	.A(NotZeroANDBrachNE),
	.B(ZeroANDBrachEQ),
	.C(ORForBranch)
);

//**********************/
Control
ControlUnit
(
	.OP(Instruction_wire_ID[31:26]),
	.ALUFunction(Instruction_wire_ID[5:0]),
	.RegDst(RegDst_wire),
	.BranchNE(BranchNE_wire),
	.BranchEQ(BranchEQ_wire),
	.ALUOp(ALUOp_wire),
	.ALUSrc(ALUSrc_wire),
	.RegWrite(RegWrite_wire),
	.Jump(Jump_wire),
	.MemRead(MemRead_wire),
	.MemtoReg(MemtoReg_wire),
	.MemWrite(MemWrite_wire),
	.JR(JR)
	);

 PC_Register
#(
	.N(32)
)

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o
ProgramCounter
(
	.clk(clk),
	.reset(reset),
	.NewPC(JOrPC4OrBranchOrJR_wire),
	.PCValue(PC_wire)
);


ProgramMemory
#(
	.MEMORY_DEPTH(MEMORY_DEPTH)
)
ROMProgramMemory
(
	.Address(PC_wire),
	.Instruction(Instruction_wire)
);

Adder32bits
PC_Puls_4
(
	.Data0(PC_wire),
	.Data1(4),

	.Result(PC_4_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ShiftLeft2
JumpShifter
(
	.DataInput({6'b0,Instruction_wire_ID[25:0]}),
   .DataOutput(JumpAddrSh2_wire)
);

Adder32bits
JumpAddr_4
(
	.Data0(32'hFFC00000), //complemento a 2 de 00400000 para
	.Data1({PC_4_wire_ID[31:28], JumpAddrSh2_wire[27:0]}),

	.Result(JumpAddr)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ShiftLeft2
BranchShifter
(
	 .DataInput(InmmediateExtend_wire_EX),
   .DataOutput(BranchAddrSh2_wire)
);

Adder32bits
BranchAddr_4
(
	.Data0(PC_4_wire_EX),
	.Data1(BranchAddrSh2_wire),

	.Result(BranchToPC_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForBranch
(
	.Selector(ORForBranch),
	.MUX_Data0(PC_4_wire),
	.MUX_Data1(BranchToPC_wire_MEM),

	.MUX_Output(BranchOrPC4_wire)

);


Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJump
(
	.Selector(Jump_wire_MEM),
	.MUX_Data0(BranchOrPC4_wire),
	.MUX_Data1(JumpAddr_MEM),

	.MUX_Output(JumpOrPC4OrBranch_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJumpRegister
(
	.Selector(JR_wire_MEM),
	.MUX_Data0(JumpOrPC4OrBranch_wire),
	.MUX_Data1(ReadData1_wire_MEM),

	.MUX_Output(JOrPC4OrBranchOrJR_wire)

);

//**********************/
//**********************/
//**********************/
//**********************/
//**********************/
//||||||||IF:IntructionFetch|||||||||//




//||||||||ID:RegisterFile|||||||||//

RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(RegWrite_wire_WB),
	.WriteRegister(RAorWriteReg_wire),
	.ReadRegister1(Instruction_wire_ID[25:21]),
	.ReadRegister2(Instruction_wire_ID[20:16]),
	.WriteData(JAL_Address_or_ALU_Result_wire),
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)

);

SignExtend
SignExtendForConstants
(
	.DataInput(Instruction_wire_ID[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);



//||||||||EX:Execute|||||||||//

ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_wire_EX),
	.ALUFunction(InmmediateExtend_wire_EX[5:0]),
	.ALUOperation(ALUOperation_wire)

);


ALU
Arithmetic_Logic_Unit
(
	.ALUOperation(ALUOperation_wire),
	.A(ReadData1_wire_EX),
	.B(ReadData2OrInmmediate_wire),
	.shamt(shamt_EX),
	.Zero(Zero_wire),
	.ALUResult(ALUResult_wire)
);

assign ALUResultOut = ALUResult_wire;

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(ALUSrc_wire_EX),
	.MUX_Data0(ReadData2_wire_EX),
	.MUX_Data1(InmmediateExtend_wire_EX),

	.MUX_Output(ReadData2OrInmmediate_wire)

);

Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(RegDst_wire_EX),
	.MUX_Data0(RT_I_wire_EX),
	.MUX_Data1(RD_R_wire_EX),

	.MUX_Output(WriteRegister_wire)

);


//||||||||MEM:Memory|||||||||//

Adder32bits
SubstractToMemoryAddress
(
	.Data0(ALUResult_wire_MEM),
	.Data1(32'hEFFF0000),

	.Result(MemoryAddressx4_wire)
);

assign MemoryAddress_wire = {1'b0, 1'b0, MemoryAddressx4_wire[31:2]};


DataMemory
#(
	.DATA_WIDTH(32),
	.MEMORY_DEPTH(1024)
)
Memory
(
	.WriteData(ReadData2_wire_MEM),
	.Address(MemoryAddress_wire),
	.MemWrite(MemWrite_wire_MEM),
	.MemRead(MemRead_wire_MEM),
	.clk(clk),
	.ReadData(MemOut_wire)
);


//||||||||WB:WriteBack|||||||||//

Multiplexer2to1
#(
	.NBits(32)
)
MUX_MemtoReg
(
	.Selector(MemtoReg_wire_WB),
	.MUX_Data0(ALUResult_wire_WB),
	.MUX_Data1(MemOut_wire_WB),

	.MUX_Output(MemOrAlu_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_JAL_address_Or_ALU_Result
(
	.Selector(Jump_wire_WB),
	.MUX_Data0(MemOrAlu_wire),
	.MUX_Data1(JOrPC4OrBranchOrJR_wire_WB),

	.MUX_Output(JAL_Address_or_ALU_Result_wire)

);

Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForJalorRorIType
(
	.Selector(Jump_wire_WB),
	.MUX_Data0(WriteRegister_wire_WB),
	.MUX_Data1(5'b11111),

	.MUX_Output(RAorWriteReg_wire)

);


//||||||||PIPES|||||||||//

Pipe
#(
.N(64)
)
IF_ID_Pipe
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({PC_4_wire, Instruction_wire}),
.DataOutput(ID_wire)
);

assign PC_4_wire_ID = ID_wire [63:32];
assign Instruction_wire_ID = ID_wire [31:0];

Pipe
#(
.N(189)
)
ID_EX_Pipe
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({JR_wire, JumpAddr, PC_4_wire_ID, RegDst_wire, BranchNE_wire, BranchEQ_wire, ALUOp_wire, ALUSrc_wire, RegWrite_wire, Jump_wire, MemRead_wire, MemtoReg_wire, MemWrite_wire, ReadData1_wire, ReadData2_wire, InmmediateExtend_wire, Instruction_wire_ID[20:16], Instruction_wire_ID[15:11], Instruction_wire_ID[10:6]}),
.DataOutput(EX_wire)
);

assign shamt_EX = EX_wire[4:0];
assign RD_R_wire_EX = EX_wire[9:5];
assign RT_I_wire_EX = EX_wire[14:10];
assign InmmediateExtend_wire_EX = EX_wire[46:15];
assign ReadData2_wire_EX = EX_wire[78:47];
assign ReadData1_wire_EX = EX_wire[110:79];
assign MemWrite_wire_EX = EX_wire[111];
assign MemtoReg_wire_EX = EX_wire[112];
assign MemRead_wire_EX = EX_wire[113];
assign Jump_wire_EX = EX_wire[114];
assign RegWrite_wire_EX = EX_wire[115];
assign ALUSrc_wire_EX = EX_wire[116];
assign ALUOp_wire_EX = EX_wire[120:117];
assign BranchEQ_wire_EX = EX_wire[121];
assign BranchNE_wire_EX = EX_wire[122];
assign RegDst_wire_EX = EX_wire[123];
assign PC_4_wire_EX = EX_wire[155:124];
assign JumpAddr_EX = EX_wire[187:156];
assign JR_wire_EX = EX_wire[188];

Pipe
#(
.N(174)
)
EX_MEM_Pipe
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({ReadData1_wire_EX, JR_wire_EX, JumpAddr_EX, BranchNE_wire_EX, BranchEQ_wire_EX, RegWrite_wire_EX, Jump_wire_EX, MemRead_wire_EX, MemtoReg_wire_EX, MemWrite_wire_EX, BranchToPC_wire, Zero_wire, ALUResult_wire, ReadData2_wire_EX, WriteRegister_wire}),
.DataOutput(MEM_wire)
);


assign ReadData1_wire_MEM = MEM_wire[173:142];
assign JR_wire_MEM = MEM_wire[141];
assign JumpAddr_MEM = MEM_wire[140:109];
assign BranchNE_wire_MEM = MEM_wire[108];
assign BranchEQ_wire_MEM = MEM_wire[107];
assign RegWrite_wire_MEM = MEM_wire[106];
assign Jump_wire_MEM = MEM_wire[105];
assign MemRead_wire_MEM = MEM_wire[104];
assign MemtoReg_wire_MEM = MEM_wire[103];
assign MemWrite_wire_MEM = MEM_wire[102];
assign BranchToPC_wire_MEM = MEM_wire[101:70];
assign Zero_wire_MEM = MEM_wire[69];
assign ALUResult_wire_MEM = MEM_wire[68:37];
assign ReadData2_wire_MEM = MEM_wire[36:5];
assign WriteRegister_wire_MEM = MEM_wire[4:0];

Pipe
#(
.N(104)
)
MEM_WB_Pipe
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({MemtoReg_wire_MEM, JOrPC4OrBranchOrJR_wire, RegWrite_wire_MEM, Jump_wire_MEM, MemOut_wire, ALUResult_wire_MEM, WriteRegister_wire_MEM}),
.DataOutput(WB_wire)
);

assign MemtoReg_wire_WB = WB_wire[103];
assign JOrPC4OrBranchOrJR_wire_WB = WB_wire[102:71];
assign RegWrite_wire_WB = WB_wire[70];
assign Jump_wire_WB = WB_wire[69];
assign MemOut_wire_WB = WB_wire[68:37];
assign ALUResult_wire_WB = WB_wire[36:5];
assign WriteRegister_wire_WB = WB_wire[4:0];

endmodule
