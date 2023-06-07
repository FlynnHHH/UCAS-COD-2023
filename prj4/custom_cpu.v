`timescale 10ns / 1ns

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output reg [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
  wire [69:0] inst_retire;

// TODO: Please add your custom CPU code here
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;
	assign inst_retire = {RF_wen, RF_waddr, RF_wdata, PC};
	
	`define ALUOP_AND  3'b000
	`define ALUOP_OR   3'b001
	`define ALUOP_XOR  3'b100
	`define ALUOP_NOR  3'b101 
	`define ALUOP_ADD  3'b010
	`define ALUOP_SUB  3'b110
	`define ALUOP_SLT  3'b111
	`define ALUOP_SLTU 3'b011 // define ALU operation code	
	// instruction type
	`define BType  4'b0000
	`define IType  4'b0001
	`define IAlu   4'b0010
	`define ILoad  4'b0011
	`define IShift 4'b0100
	`define JType  4'b0101
	`define RAlu   4'b0110
	`define RShift 4'b0111
	`define SType  4'b1000
	`define UType  4'b1001
	// define instruction type code

	wire [6:0] opcode = valid_Instruction[6:0];
	wire [4:0] rs1 = valid_Instruction[19:15];
	wire [4:0] rs2 = valid_Instruction[24:20];
	wire [4:0] rd = valid_Instruction[11:7];	
	wire [6:0] funct7 = valid_Instruction[31:25];
	wire [2:0] funct3 = valid_Instruction[14:12];
	wire [4:0] shamt = valid_Instruction[24:20];
	wire [11:0] I_imm = valid_Instruction[31:20];
	wire [11:0] S_imm = {valid_Instruction[31:25], valid_Instruction[11:7]};
	wire [12:0] B_imm = {valid_Instruction[31], valid_Instruction[7], valid_Instruction[30:25], valid_Instruction[11:8], 1'b0};
	wire [31:0] U_imm = {valid_Instruction[31:12], 12'b0};
	wire [31:0] J_imm = {{12{valid_Instruction[31]}}, valid_Instruction[19:12], valid_Instruction[20], valid_Instruction[30:21], 1'b0};
	// define instruction field
	wire [3:0] type = {4{opcode == 7'b1100011}} & `BType|
			{4{opcode == 7'b1100111}} & `IType |	// jalr
			{4{opcode == 7'b0010011 && {funct3[1:0]} != 2'b01}} & `IAlu |
			{4{opcode == 7'b0000011}} & `ILoad |
			{4{opcode == 7'b0010011 && {funct3[1:0]} == 2'b01}} & `IShift |
			{4{opcode == 7'b1101111}} & `JType |	// jal
			{4{opcode == 7'b0110011 && {funct3[1:0]} != 2'b01}} & `RAlu |
			{4{opcode == 7'b0110011 && {funct3[1:0]} == 2'b01}} & `RShift |
			{4{opcode == 7'b0100011}} & `SType |
			{4{{opcode[6],opcode[4:0]} == 6'b010111}} & `UType;	// lui, auipc
	// decode the opcode
	//PC
	wire [31:0] PC_next = PC + 4;
	reg [31:0] PC_pre;
	always @(posedge clk) begin
		if (rst) PC <= 32'b0;
		else if(current_state == EX)
			PC <= (type == `IType || type == `JType)? jumpaddr: (isbranch? PC_branch: PC_next);
		else if(valid_Instruction == 32'b0 && current_state == ID) 
			PC <= PC_next;
		else 
			PC <= PC;
	end 
	always @(posedge clk ) begin
		if (current_state == IF) PC_pre <= PC;
		else PC_pre <= PC_pre;
	end
	
    wire [4:0] RF_raddr1 = rs1;
    wire [4:0] RF_raddr2 = rs2;
    wire [31:0] RF_rdata1;
    wire [31:0] RF_rdata2;
	// ALU control	
	wire [31:0] I_extend = {{20{I_imm[11]}}, I_imm};
	wire [31:0] S_extend = {{20{S_imm[11]}}, S_imm};
	wire [31:0] alunum1 = RF_rdata1;
	wire [31:0] alunum2 = {{32{type == `IAlu || type == `ILoad || type == `IType}} & I_extend} |
				{{32{type == `SType}} & S_extend} |
				{{32{type == `RAlu || type == `BType}} & RF_rdata2};
	wire [2:0] ALU_control = {{3{type == `RAlu && funct3 == 3'b000}} & {funct7[5], 2'b10}} | // add & sub
				{{3{type == `IAlu && funct3 == 3'b000}} & `ALUOP_ADD} | // addi
				{{3{type == `RAlu && funct3[2:1] == 2'b11 }} & ~funct3} | // and & or
				{{3{type == `RAlu && funct3 == 3'b100}} & funct3} | // xor
				{{3{type == `IAlu && funct3[2:1] == 2'b11 }} & ~funct3} | // andi & ori
				{{3{type == `IAlu && funct3 == 3'b100}} & funct3} | // xori
				{{3{type == `RAlu && funct3[2:1] == 2'b01}} & {~funct3[0], 2'b11}} | // slt & sltu
				{{3{type == `IAlu && funct3[2:1] == 2'b01}} & {~funct3[0], 2'b11}} | // slti & sltiu
				{{3{type == `BType && funct3[2:1] == 2'b00}} & `ALUOP_SUB} | // beq & bne
				{{3{type == `BType && funct3[2:1] == 2'b10}} & `ALUOP_SLT} | // blt & bge
				{{3{type == `BType && funct3[2:1] == 2'b11}} & `ALUOP_SLTU} | // bltu & bgeu
				{{3{type == `ILoad || type == `SType || type == `IType}} & `ALUOP_ADD}; // lw & sw & jalr
	wire Zero;
	wire [31:0] ALU_result;
	alu alu_module(
		.A(alunum1),
		.B(alunum2),
		.ALUop(ALU_control),
		.Result(ALU_result),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero)
	);

	// shift control
	`define SHIFTOP_SLL  2'b00
	`define SHIFTOP_SRL  2'b10
	`define SHIFTOP_SRA  2'b11 // define SHIFT operation code
	
	wire [4:0] shiftbit = {{5{type == `IShift}} & shamt} |
						{{5{type == `RShift}} & RF_rdata2[4:0]}; 
	wire [1:0] SHIFT_control = {funct3[2],funct7[5]};
	wire [31:0] SHIFT_result;
	shifter shifter_module(RF_rdata1, shiftbit, SHIFT_control, SHIFT_result); // shifter module
	
	// reg_file control
	assign RF_wen = current_state == WB;
	assign RF_waddr = rd;
	assign RF_wdata = (type == `RAlu || type == `IAlu)? ALU_result:
			(type == `RShift || type == `IShift)? SHIFT_result:
			(type == `UType && opcode[5])? U_imm:
			(type == `UType && ~opcode[5])? PC_auipc:
			(type == `JType || type == `IType)? PC_pre + 4:
			(type == `ILoad)? load_result:32'b0; // write data
	reg_file reg_file_module(
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(RF_raddr1),
		.raddr2(RF_raddr2),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2)
		); 

	// jump control
	wire [31:0] PC_jump;
	alu alu_jump(
		.A(J_imm),
		.B(PC_pre),
		.ALUop(`ALUOP_ADD),
		.Result(PC_jump),
		.Overflow(),
		.CarryOut(),
		.Zero()
	);
	wire [31:0] jumpaddr = (type == `IType)? {ALU_result[31:1],1'b0}: PC_jump;

	// branch control
	wire [31:0] PC_branch;
	wire isbranch = (type == `BType && ((funct3 == 3'b000 || funct3 == 3'b101 || funct3 == 3'b111) && Zero || (funct3 == 3'b001 || funct3 == 3'b100 || funct3 == 3'b110) && ~Zero));
	wire [31:0] branch_offset = {{19{B_imm[12]}}, B_imm};
	alu alu_branch(
		.A(branch_offset),
		.B(PC_pre),
		.ALUop(`ALUOP_ADD),
		.Result(PC_branch),
		.Overflow(),
		.CarryOut(),
		.Zero()
	);
	// auipc
	wire [31:0] PC_auipc;
	alu alu_auipc(
		.A(U_imm),
		.B(PC_pre),
		.ALUop(`ALUOP_ADD),
		.Result(PC_auipc),
		.Overflow(),
		.CarryOut(),
		.Zero()
	);
	// load control
	wire [1:0] n = ALU_result[1:0];

	wire [31:0] lb_result = (n[1] & n[0])? {{24{valid_Read_data[31]}},valid_Read_data[31:24]}:
				((n[1] & ~n[0])? {{24{valid_Read_data[23]}},valid_Read_data[23:16]}:
				((~n[1] & n[0])? {{24{valid_Read_data[15]}},valid_Read_data[15:8]}:{{24{valid_Read_data[7]}},valid_Read_data[7:0]}));
	wire [31:0] lbu_result = {{24{1'b0}},lb_result[7:0]};

	wire [31:0] lh_result = (~n[1])? {{16{valid_Read_data[15]}},valid_Read_data[15:0]}:{{16{valid_Read_data[31]}},valid_Read_data[31:16]}; 
	wire [31:0] lhu_result = {{16{1'b0}},lh_result[15:0]};

	wire [31:0] lw_result = valid_Read_data[31:0];

	
	wire [31:0] load_result = (funct3 == 3'b000)? lb_result:
				(funct3 == 3'b001)? lh_result:
				(funct3 == 3'b010)? lw_result:
				(funct3 == 3'b100)? lbu_result: lhu_result;

	//store control
	assign Address = {ALU_result[31:2],2'b00};//aligned address

	wire [3:0] sb_strb = (n[1] & n[0])? 4'b1000:
					 ((n[1] & ~n[0])? 4'b0100:
					 ((~n[1] & n[0])? 4'b0010: 4'b0001));
	wire [3:0] sh_strb = (n[1])? 4'b1100: 4'b0011;
	wire [3:0] sw_strb = 4'b1111;

	assign Write_strb = (funct3 == 3'b000)? sb_strb:
			(funct3 == 3'b001)? sh_strb: sw_strb;

	wire [31:0] sb_data = (n[1] & n[0])? {RF_rdata2[7:0],{24{1'b0}}}:
				((n[1] & ~n[0])? {{8{1'b0}},RF_rdata2[7:0],{16{1'b0}}}:
				((~n[1] & n[0])? {{16{1'b0}},RF_rdata2[7:0],{8{1'b0}}}: {{24{1'b0}},RF_rdata2[7:0]}));

	wire [31:0] sh_data = (n[1])? {RF_rdata2[15:0],{16{1'b0}}}: {{16{1'b0}},RF_rdata2[15:0]};

	wire [31:0] sw_data = RF_rdata2[31:0];

	assign Write_data = (funct3 == 3'b000)? sb_data:
			(funct3 == 3'b001)? sh_data: sw_data;

	// 3-part FSM
	// decode the state of the FSM using one-hot encoding
	parameter INIT = 9'b000000001,
	          IF   = 9'b000000010,
	          IW   = 9'b000000100,
	          ID   = 9'b000001000,
	          EX   = 9'b000010000,
	          LD   = 9'b000100000,
	          ST   = 9'b001000000,
	          RDW  = 9'b010000000,
	          WB   = 9'b100000000;	
	reg [8:0] current_state, next_state;
	reg [31:0] valid_Instruction, valid_Read_data;
	// part 1
	always @(posedge clk ) begin
		if(rst) current_state <= INIT;
		else current_state <= next_state;
	end
	// part 2
	always @(*) begin
		case (current_state)
			INIT: next_state = IF;
			IF: if(Inst_Req_Ready) next_state = IW; else next_state = IF;
			IW: if(Inst_Valid) next_state = ID; else next_state = IW;
			ID: if(valid_Instruction[31:0] == 32'b0) next_state = IF; else next_state = EX;
			EX: if(type == `ILoad) next_state = LD;
				else if(type == `SType) next_state = ST;
				else if(type == `BType) next_state = IF;
				else next_state = WB;
			LD: if(Mem_Req_Ready) next_state = RDW; else next_state = LD;
			RDW: if(Read_data_Valid) next_state = WB; else next_state = RDW;
			WB: next_state = IF;
			ST: if(Mem_Req_Ready) next_state = IF; else next_state = ST;
			default: next_state = INIT;
		endcase
	end
	// part 3
	assign Inst_Req_Valid = current_state == IF;
	assign Inst_Ready      = (current_state == IW || current_state == INIT);
	assign Read_data_Ready = (current_state == RDW || current_state == INIT);
	assign MemRead         = current_state == LD;
	assign MemWrite        = current_state == ST;
	always @(posedge clk) begin
		valid_Instruction <= (Inst_Ready && Inst_Valid)? Instruction: valid_Instruction;
	end
	always @(posedge clk) begin
		valid_Read_data <= (Read_data_Ready && Read_data_Valid)? Read_data: valid_Read_data;
	end
	
	// performance counter
	// The number of cycle
	reg [31:0] cycle_cnt;
	always @(posedge clk) begin
		if(rst) cycle_cnt <= 32'd0;
		else cycle_cnt <= cycle_cnt + 32'd1;
	end
	assign cpu_perf_cnt_0 = cycle_cnt;
endmodule
