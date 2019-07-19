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
	parameter MEMORY_DEPTH = 128
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
wire Bubble_wire;
wire Jump_wire;
wire JR_wire;
wire MemRead_wire;
wire MemWrite_wire;
wire MemtoReg_wire;
wire NotZeroANDBrachNE;
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
wire PCEnable_Wire;
wire IFFlush_wire;
wire IFFlushOrBranch_wire;
wire DEnable_Wire;
wire RegWrite_wire_MEM;
wire Jump_wire_MEM;
wire MemRead_wire_MEM;
wire MemtoReg_wire_MEM;
wire MemWrite_wire_MEM;
wire MemtoReg_wire_WB;
wire RegWrite_wire_WB;
wire Jump_wire_WB;
wire BubbleOrEQ_wire;

wire [3:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [3:0] ALUOp_wire_EX;
wire [4:0] WriteRegister_wire;
wire [4:0] RAorWriteReg_wire;
wire [4:0] RT_wire_EX;
wire [4:0] RD_wire_EX;
wire [4:0] RS_wire_EX;
wire [4:0] shamt_EX;
wire [4:0] WriteRegister_wire_MEM;
wire [4:0] WriteRegister_wire_WB;

wire [10:0] Control_wire;
wire [10:0] Control_wire_ID;

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
wire [31:0] JumpAddr_wire;
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
wire [31:0] MUXFwdA1to2_wire;
wire [31:0] MUXFwdR1_wire;
wire [31:0] MUXFwdB1to2_wire;
wire [31:0] MUXFwdR2_wire;
wire [31:0] ReadData1_wire_MEM;
wire [31:0] BranchToPC_wire_MEM;
wire [31:0] ALUResult_wire_MEM;
wire [31:0] ReadData2_wire_MEM;
wire [31:0] MemOut_wire_WB;
wire [31:0] ALUResult_wire_WB;
wire [31:0] PC_4_wire_MEM;
wire [31:0] PC_4_wire_WB;

wire [63:0] ID_wire;
wire [135:0] WB_wire;
wire [204:0] MEM_wire;
wire [193:0] EX_wire;

integer ALUStatus;

//|||||||||||||||||||||||||||||||||||||||||||||||\\
//||||||||||||||CompuertasLogicas|||||||||||||||//
//|||||||||||||||||||||||||||||||||||||||||||||||\\

ORGate
Gate_BranchORIFFlush
(
	.A(IFFlush_wire),
	.B(Zero_wire),
	.C(IFFlushOrBranch_wire)
);

ORGate
OrBranchEq
(
	.A(Bubble_wire),
	.B(Zero_wire),
	.C(BubbleOrEQ_wire)
);

//|||||||||||||||||||||||||||||||||||||||||||||||\\
//||||||||||||||IF:IntructionFetch|||||||||||||||//
//|||||||||||||||||||||||||||||||||||||||||||||||\\

PC_Register
#(
 .N(32)
)
ProgramCounter
(
	.clk(clk),
	.reset(reset),
	.NewPC(JOrPC4OrBranchOrJR_wire),
	.enable(PCEnable_Wire),
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

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForBranch
(
	.Selector(Zero_wire),
	.MUX_Data0(PC_4_wire),
	.MUX_Data1(BranchToPC_wire),

	.MUX_Output(BranchOrPC4_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJump
(
	.Selector(Jump_wire),
	.MUX_Data0(BranchOrPC4_wire),
	.MUX_Data1(JumpAddr_wire),

	.MUX_Output(JumpOrPC4OrBranch_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJumpRegister
(
	.Selector(JR_wire),
	.MUX_Data0(JumpOrPC4OrBranch_wire),
	.MUX_Data1(ReadData1_wire),

	.MUX_Output(JOrPC4OrBranchOrJR_wire)

);


//||||||||||||||||||||||||||||||||||||||||||||\\
//||||||||||||||ID:RegisterFile|||||||||||||||//
//||||||||||||||||||||||||||||||||||||||||||||\\

Hazard
HazardUnit
(
	.Instruction_ID(Instruction_wire_ID[25:16]),
	.RT_EX(RT_wire_EX),
	.MemRead_EX(MemRead_wire_EX),
	.DWrite(DEnable_Wire),
	.PCWrite(PCEnable_Wire),
	.Bubble(Bubble_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

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
	.JR(JR_wire),
	.IFFlush(IFFlush_wire)
	);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

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

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Equals
Data1EqualsData2
(
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire),
	.BranchEQ(BranchEQ_wire),
	.BranchNE(BranchNE_wire),
	.ORForBranch(Zero_wire)

);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

SignExtend
SignExtendForConstants
(
	.DataInput(Instruction_wire_ID[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ShiftLeft2
BranchShifter
(
	 .DataInput(InmmediateExtend_wire),
   .DataOutput(BranchAddrSh2_wire)
);
//-o-o-o-o-o-o-o
Adder32bits
BranchAddr_4
(
	.Data0(PC_4_wire_ID),
	.Data1(BranchAddrSh2_wire),

	.Result(BranchToPC_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ShiftLeft2
JumpShifter
(
	.DataInput({6'b0,Instruction_wire_ID[25:0]}),
   .DataOutput(JumpAddrSh2_wire)
);
//-o-o-o-o-o-o-o
Adder32bits
JumpAddr_4
(
	.Data0(32'hFFC00000), //complemento a 2 de 00400000 para
	.Data1({PC_4_wire_ID[31:28], JumpAddrSh2_wire[27:0]}),

	.Result(JumpAddr_wire)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

assign Control_wire = {RegDst_wire, ALUOp_wire, ALUSrc_wire, RegWrite_wire, Jump_wire, MemRead_wire, MemtoReg_wire, MemWrite_wire};

Multiplexer2to1
#(
.NBits(11)
)
MUXControlHDU
(
.Selector(BubbleOrEQ_wire),  //HDU Selector
.MUX_Data0(Control_wire),
.MUX_Data1(11'b0),
.MUX_Output(Control_wire_ID)
);

//|||||||||||||||||||||||||||||||||||||||\\
//||||||||||||||EX:Execute|||||||||||||||//
//|||||||||||||||||||||||||||||||||||||||\\

ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_wire_EX),
	.ALUFunction(InmmediateExtend_wire_EX[5:0]),
	.ALUOperation(ALUOperation_wire)

);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ALU
Arithmetic_Logic_Unit
(
	.ALUOperation(ALUOperation_wire),
	.A(MUXFwdR1_wire),
	.B(ReadData2OrInmmediate_wire),
	.shamt(shamt_EX),
	.ALUResult(ALUResult_wire)
);

assign ALUResultOut = ALUResult_wire;

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

wire [1:0] A;
wire [1:0] B;

ForwardingUnit
FwdUnit
(
	.WB_WB(RegWrite_wire_WB),
	.WB_MEM(RegWrite_wire_MEM),
	.RS_EX(RS_wire_EX),
	.RT_EX(RT_wire_EX),
	.RTorRD_MEM(WriteRegister_wire_MEM),
	.RTorRD_WB(RAorWriteReg_wire),
	.A(A),
	.B(B)

);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(ALUSrc_wire_EX),
	.MUX_Data0(MUXFwdR2_wire),
	.MUX_Data1(InmmediateExtend_wire_EX),

	.MUX_Output(ReadData2OrInmmediate_wire)

);

//-o-o-o-o-o-o-o

Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(RegDst_wire_EX),
	.MUX_Data0(RT_wire_EX),
	.MUX_Data1(RD_wire_EX),

	.MUX_Output(WriteRegister_wire)

);

//-o-o-o-o-o-o-o

Multiplexer2to1
#(
.NBits(32)
)
MuxFwdUnitA1
(
.Selector(A[0]),
.MUX_Data0(ALUResult_wire_MEM),
.MUX_Data1(JAL_Address_or_ALU_Result_wire),
.MUX_Output(MUXFwdA1to2_wire)
);

//-o-o-o-o-o-o-o

Multiplexer2to1
#(
.NBits(32)
)
MuxFwdUnitA2
(
.Selector(A[1]),
.MUX_Data0(ReadData1_wire_EX),
.MUX_Data1(MUXFwdA1to2_wire),
.MUX_Output(MUXFwdR1_wire) //usar este para sustituir
);

//-o-o-o-o-o-o-o

Multiplexer2to1
#(
.NBits(32)
)
MuxFwdUnitB1
(
.Selector(B[0]),
.MUX_Data0(ALUResult_wire_MEM),
.MUX_Data1(JAL_Address_or_ALU_Result_wire),
.MUX_Output(MUXFwdB1to2_wire)
);

//-o-o-o-o-o-o-o

Multiplexer2to1
#(
.NBits(32)
)
MuxFwdUnitB2
(
.Selector(B[1]),
.MUX_Data0(ReadData2_wire_EX),
.MUX_Data1(MUXFwdB1to2_wire),
.MUX_Output(MUXFwdR2_wire)
);


//|||||||||||||||||||||||||||||||||||||||\\
//||||||||||||||MEM:Memory|||||||||||||||//
//|||||||||||||||||||||||||||||||||||||||\\

Adder32bits
SubstractToMemoryAddress
(
	.Data0(ALUResult_wire_MEM),
	.Data1(32'hEFFF0000),

	.Result(MemoryAddressx4_wire)
);

assign MemoryAddress_wire = {1'b0, 1'b0, MemoryAddressx4_wire[31:2]};

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

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


//|||||||||||||||||||||||||||||||||||||||||\\
//||||||||||||||WB:WriteBack|||||||||||||||//
//|||||||||||||||||||||||||||||||||||||||||\\

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
	.NBits(5)
)
MUX_ForJalorRorIType
(
	.Selector(Jump_wire_WB),
	.MUX_Data0(WriteRegister_wire_WB),
	.MUX_Data1(5'b11111),

	.MUX_Output(RAorWriteReg_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_JAL_address_Or_ALU_Result
(
	.Selector(Jump_wire_WB),
	.MUX_Data0(MemOrAlu_wire),
	.MUX_Data1(PC_4_wire_WB),

	.MUX_Output(JAL_Address_or_ALU_Result_wire)

);


//||||||||||||||||||||||\\
//||||||||PIPES|||||||||//
//||||||||||||||||||||||\\

Pipe
#(
.N(64)
)
IF_ID_Pipe
(
.clk(clk),
.reset(reset),
.enable(DEnable_Wire),
.Flush(IFFlushOrBranch_wire),
.DataInput({PC_4_wire, Instruction_wire}),
.DataOutput(ID_wire)

);

assign PC_4_wire_ID = ID_wire [63:32];
assign Instruction_wire_ID = ID_wire [31:0];

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Pipe
#(
.N(160)
)
ID_EX_Pipe
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({Jump_wire, Instruction_wire_ID[25:21], PC_4_wire_ID, Control_wire_ID, ReadData1_wire, ReadData2_wire, InmmediateExtend_wire, Instruction_wire_ID[20:16], Instruction_wire_ID[15:11], Instruction_wire_ID[10:6]}),
.DataOutput(EX_wire),
.Flush(1'b0)
);

assign shamt_EX = EX_wire[4:0];
assign RD_wire_EX = EX_wire[9:5];
assign RT_wire_EX = EX_wire[14:10];
assign InmmediateExtend_wire_EX = EX_wire[46:15];
assign ReadData2_wire_EX = EX_wire[78:47];
assign ReadData1_wire_EX = EX_wire[110:79];
assign MemWrite_wire_EX = EX_wire[111];
assign MemtoReg_wire_EX = EX_wire[112];
assign MemRead_wire_EX = EX_wire[113];
assign Jump_wire_EX = EX_wire[159];
assign RegWrite_wire_EX = EX_wire[115];
assign ALUSrc_wire_EX = EX_wire[116];
assign ALUOp_wire_EX = EX_wire[120:117];
assign RegDst_wire_EX = EX_wire[121];
assign PC_4_wire_EX = EX_wire[153:122];
assign RS_wire_EX = EX_wire[158:154];

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Pipe
#(
.N(170)
)
EX_MEM_Pipe
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({PC_4_wire_EX,ReadData1_wire_EX, RegWrite_wire_EX, Jump_wire_EX, MemRead_wire_EX, MemtoReg_wire_EX, MemWrite_wire_EX, BranchToPC_wire, ALUResult_wire, ReadData2_wire_EX, WriteRegister_wire}),
.DataOutput(MEM_wire),
.Flush(1'b0)
);

assign WriteRegister_wire_MEM = MEM_wire[4:0];
assign ReadData2_wire_MEM = MEM_wire[36:5];
assign ALUResult_wire_MEM = MEM_wire[68:37];
assign BranchToPC_wire_MEM = MEM_wire[100:69];
assign MemWrite_wire_MEM = MEM_wire[101];
assign MemtoReg_wire_MEM = MEM_wire[102];
assign MemRead_wire_MEM = MEM_wire[103];
assign Jump_wire_MEM = MEM_wire[104];
assign RegWrite_wire_MEM = MEM_wire[105];
assign ReadData1_wire_MEM = MEM_wire[137:106];
assign PC_4_wire_MEM = MEM_wire[169:138];

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

Pipe
#(
.N(104)
)
MEM_WB_Pipe
(
.clk(clk),
.reset(reset),
.enable(1'b1),
.DataInput({PC_4_wire_MEM ,MemtoReg_wire_MEM, RegWrite_wire_MEM, Jump_wire_MEM, MemOut_wire, ALUResult_wire_MEM, WriteRegister_wire_MEM}),
.DataOutput(WB_wire),
.Flush(1'b0)
);

assign WriteRegister_wire_WB = WB_wire[4:0];
assign ALUResult_wire_WB = WB_wire[36:5];
assign MemOut_wire_WB = WB_wire[68:37];
assign Jump_wire_WB = WB_wire[69];
assign RegWrite_wire_WB = WB_wire[70];
assign MemtoReg_wire_WB = WB_wire[71];
assign PC_4_wire_WB = WB_wire[103:72];

endmodule
