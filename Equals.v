/******************************************************************
* Description
*		This is the Equals:
* Version:
*	1.0
* Author:
*Luis David Gallegos Godoy
* email:
*	is709571@iteso.mx
* Date:
*	17/07/2019
******************************************************************/
module Equals
(
	input [31:0] ReadData1,
	input [31:0] ReadData2,
  input BranchEQ,
  input BranchNE,
	output ORForBranch
);

wire Zero;
wire ZeroANDBrachEQ;
wire NotZeroANDBrachNE;

assign Zero = (ReadData1 == ReadData2) ? 1'b1 : 1'b0;

ANDGate
Gate_BranchEQANDZero
(
	.A(BranchEQ),
	.B(Zero), //bit menos significativo del opcode porque J 000010 y JAL 000011
	.C(ZeroANDBrachEQ)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ANDGate
Gate_BranchNEANDZero
(
	.A(BranchNE),
	.B(!Zero), //Si zero es diferente de 1, significa que es diferente
	.C(NotZeroANDBrachNE)
);

//-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

ORGate
Gate_BeqOrBNE
(
	.A(NotZeroANDBrachNE),
	.B(ZeroANDBrachEQ),
	.C(ORForBranch)
);



endmodule
