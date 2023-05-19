`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
	// TODO: Please add your logic design here
	`define ALUOP_AND  3'b000
	`define ALUOP_OR   3'b001
	`define ALUOP_XOR  3'b100
	`define ALUOP_NOR  3'b101 
	`define ALUOP_ADD  3'b010
	`define ALUOP_SUB  3'b110
	`define ALUOP_SLT  3'b111
	`define ALUOP_SLTU 3'b011	// use `define to delete "magic numbers" 
	
	wire op_and  = ALUop == `ALUOP_AND;
	wire op_or   = ALUop == `ALUOP_OR;
	wire op_xor  = ALUop == `ALUOP_XOR;
	wire op_nor  = ALUop == `ALUOP_NOR;
	wire op_add  = ALUop == `ALUOP_ADD;
	wire op_sub  = ALUop == `ALUOP_SUB;
	wire op_slt  = ALUop == `ALUOP_SLT;
	wire op_sltu = ALUop == `ALUOP_SLTU;		// decode the alu_op signals
	
	wire [`DATA_WIDTH - 1:0] and_res = A & B;		// calculate the result of and 
	wire [`DATA_WIDTH - 1:0] or_res  = A | B;		// calculate the result of or
	wire [`DATA_WIDTH - 1:0] xor_res = A ^ B;		// calculate the result of xor
	wire [`DATA_WIDTH - 1:0] nor_res = ~(A | B);		// calculate the result of nor
	wire [`DATA_WIDTH - 1:0] B_2 = (op_sub || op_slt || op_sltu)? ~B : B;
	wire cin = (op_sub || op_slt || op_sltu)? 1: 0;
	wire [`DATA_WIDTH - 1:0] out;
	wire cout;
	assign {cout, out} = A + B_2 + cin;			
	assign CarryOut = (op_sub || op_slt || op_sltu)? !cout: cout; // calculate the CarryOut flag
	wire [`DATA_WIDTH - 1:0] add_res = out;			// calculate the result of add 
	wire [`DATA_WIDTH - 1:0] sub_res = out;			// calculate the result of sub 
	wire [`DATA_WIDTH - 1:0] slt_res = out [`DATA_WIDTH - 1] ^ Overflow;		// calculate the result of slt
	wire [`DATA_WIDTH - 1:0] sltu_res = CarryOut;		// calculate the result of sltu  

	assign Result = {`DATA_WIDTH{op_and}} & and_res |		// choose the final Result
			{`DATA_WIDTH{op_or }} & or_res  |
			{`DATA_WIDTH{op_xor}} & xor_res |
			{`DATA_WIDTH{op_nor}} & nor_res |   
			{`DATA_WIDTH{op_add}} & add_res |
			{`DATA_WIDTH{op_sub}} & sub_res |
			{`DATA_WIDTH{op_slt}} & slt_res |
			{`DATA_WIDTH{op_sltu}}& sltu_res;

	wire CarryIn_31 = A [`DATA_WIDTH - 1] ^ B_2 [`DATA_WIDTH - 1] ^ out [`DATA_WIDTH - 1];		
	//out = a ^ b ^ cin, so cin = a ^ b ^ out 
	assign Overflow = CarryIn_31 ^ cout;		// calculate the Overflow flag
	assign Zero = Result == 32'b0;			// calculate the Zero flag
	
endmodule
