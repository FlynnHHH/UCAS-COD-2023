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
	
	`define SPECIAL 6'b000000
	`define ALUOP_AND  3'b000
	`define ALUOP_OR   3'b001
	`define ALUOP_XOR  3'b100
	`define ALUOP_NOR  3'b101 
	`define ALUOP_ADD  3'b010
	`define ALUOP_SUB  3'b110
	`define ALUOP_SLT  3'b111
	`define ALUOP_SLTU 3'b011 // define ALU operation code	
	// instruction type
	`define RAlu 4'b0000
	`define RShift 4'b0001
	`define RJump 4'b0010
	`define RMove 4'b0011
	`define REGIMM 4'b0100
	`define JType 4'b0101
	`define IBranch 4'b0110
	`define IAlu 4'b0111
	`define lui 4'b1000
	`define ILoad 4'b1001
	`define IStore 4'b1010 // define instruction type code

	wire [5:0] opcode = valid_Instruction[31:26];
	wire [4:0] rs = valid_Instruction[25:21];
	wire [4:0] base = valid_Instruction[25:21];
	wire [4:0] rt = valid_Instruction[20:16];
	wire [4:0] rd = valid_Instruction[15:11];	
	wire [4:0] REG = valid_Instruction[20:16];	
	wire [4:0] shamt = valid_Instruction[10:6];
	wire [5:0] func = valid_Instruction[5:0];
	wire [15:0] imm = valid_Instruction[15:0];
	wire [25:0] Instr_index = valid_Instruction[25:0]; // define some wire variations to decode the instruction
	wire [3:0] type = {4{opcode == `SPECIAL && func[5] == 1'b1}} & `RAlu |
					{4{opcode == `SPECIAL && func[5:3] == 3'b000}} & `RShift |
					{4{opcode == `SPECIAL && {func[5:3], func[1]} == 4'b0010}} & `RJump |
					{4{opcode == `SPECIAL && {func[5:3], func[1]} == 4'b0011}} & `RMove |
					{4{opcode == 6'b000001}} & `REGIMM |
					{4{opcode[5:1] == 5'b00001}} & `JType |
					{4{opcode[5:2] == 4'b0001}} & `IBranch |
					{4{opcode[5:3] == 3'b001 && opcode != 6'b001111}} & `IAlu |
					{4{opcode == 6'b001111}} & `lui |
					{4{opcode[5:3] == 3'b100}} & `ILoad |
					{4{opcode[5:3] == 3'b101}} & `IStore; // decode the opcode
	//PC
	wire [31:0] PC_next = PC + 4;
	wire [31:0] branch_PC;
	reg [31:0] PC_pre;
	always @(posedge clk) begin
		if (rst) PC <= 32'b0;
		else if(current_state == EX)
			PC <= (type == `RJump || type == `JType) ? jumpaddr:((branch & isbranch)?branch_PC:PC_next);
		else if(valid_Instruction == 32'b0 && current_state == ID) 
			PC <= PC_next;
		else 
			PC <= PC;
	end 
	always @(posedge clk ) begin
		if (current_state == IF) PC_pre <= PC;
		else PC_pre <= PC_pre;
	end
	alu alu_branch(
		.A(offset_two_extend),
		.B(PC_next),
		.ALUop(`ALUOP_ADD),
		.Result(branch_PC),
		.Overflow(),
		.CarryOut(),
		.Zero()
	);
    	wire [4:0] RF_raddr1;
    	wire [4:0] RF_raddr2;
    	wire [31:0] RF_rdata1;
    	wire [31:0] RF_rdata2;
	// ALU control	
	wire [31:0] alunum1 = (type == `RMove)? RF_rdata2: RF_rdata1;
	wire [31:0] alunum2 = {{32{ALUSrc}} & extend} |
				{{32{type == `RAlu || type == `IBranch}} & RF_rdata2} |
				{{32{type == `RMove || type == `REGIMM}} & 32'b0} ;
	wire [2:0] ALU_control = {{3{type == `RAlu && func[3:2] == 2'b00}} & {func[1], 2'b10}} | // addu & subu
							{{3{type == `IAlu && opcode[2:1] == 2'b00}} & {opcode[1], 2'b10}} | // addiu
							{{3{type == `RAlu && func[3:2] == 2'b01}} & {func[1], 1'b0, func[0]}} | // and & or & xor & nor
							{{3{type == `IAlu && opcode[2] == 1'b1}} & {opcode[1], 1'b0, opcode[0]}} | // andi & ori & xori & nori
							{{3{type == `RAlu && func[3:2] == 2'b10}} & {~func[0], 2'b11}} | // slt & sltu
							{{3{type == `IAlu && opcode[2:1] == 2'b01}} & {~opcode[0], 2'b11}} | // slti & sltiu
							{{3{type == `RMove}} & `ALUOP_SUB} | // movz & movn
							{{3{type == `REGIMM}} & `ALUOP_SLT} | //bltz & bgez
							{{3{type == `IBranch && opcode[1] == 1'b0}} & `ALUOP_SUB} | // beq & bne
							{{3{type == `IBranch && opcode[1] == 1'b1}} & `ALUOP_SLT} | // blez & bgtz
							{{3{type == `ILoad || type == `IStore}} & `ALUOP_ADD}; // lw & sw
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
	); // ALU module
	// shift control
	`define SHIFTOP_SLL  2'b00
	`define SHIFTOP_SRL  2'b10
	`define SHIFTOP_SRA  2'b11 // define SHIFT operation code
	
	wire [4:0] shiftbit = {{5{type == `RShift && func[5:2] == 4'b0000 }} & shamt} |
						{{5{type == `RShift && func[5:2] == 4'b0001}} & RF_rdata1[4:0]}; 
	wire [1:0] SHIFT_control = func[1:0];
	wire [31:0] SHIFT_result;
	shifter shifter_module(RF_rdata2, shiftbit, SHIFT_control, SHIFT_result); // shifter module
	
	// main control unit
	wire RegDst = (type == `IAlu || type == `ILoad || type == `lui)? 1'b0:1'b1;
	wire branch = (type == `IBranch || type == `REGIMM)? 1'b1:1'b0;
	wire ALUSrc = (type == `IAlu || type == `ILoad || type == `IStore)? 1'b1:1'b0;
	assign RF_wen = (type ==`IStore || (type ==`JType && opcode[0] == 0) || type == `IBranch || type == `REGIMM || type == `RJump && func[0] == 0 || type == `RMove && ~func[0] ^ Zero || current_state != WB)? 1'b0:1'b1;
	assign RF_waddr = (opcode == 6'b000011)? 31:((RegDst)? rd:rt); // jal writes r31
	// assign RF_wdata = {{32{type == `RAlu || type == `IAlu}} & ALU_result} | 
	// 		{{32{type == `RShift}} & SHIFT_result} | 
	// 		{{32{type == `RJump && func[0] == 1 || type == `JType && opcode[0] == 1}} & (PC[31:0] + 8) } | // jal & jalr
	// 		{{32{type == `RMove && func[0] ^ Zero}} & RF_rdata1} | 
	// 		{{32{type == `lui}} & lui_extend}; // write data to register file
	
	assign RF_wdata = (type == `RAlu || type == `IAlu)? ALU_result:
					  ((type == `RShift)? SHIFT_result:
					  ((opcode == 6'b000011 || (type == `RJump && func == 6'b001001))? PC_pre + 8:
					  ((type == `ILoad)? load_result:
					  ((type == `RMove)? RF_rdata1:lui_extend))));

	assign RF_raddr1 = rs;
	assign RF_raddr2 = (type == `REGIMM)? 5'b0:rt;
	reg_file reg_file_module(
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(RF_raddr1),
		.raddr2(RF_raddr2),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2)
		); // register file module 

	// sign extend
	wire [31:0] lui_extend = {{16{type == `lui}} & imm, 16'b0};
	wire [31:0] sign_extend = {{16{imm[15]}}, imm};
	wire [31:0] zero_extend = {16'b0, imm};
	wire [31:0] offset_two_extend = {{14{imm[15]}}, imm, 2'b0};
	wire [31:0] instr_two_extend = {PC_next[31:28], Instr_index[25:0], 2'b0};
	wire [31:0] extend = (type == `IAlu && opcode[2] == 1 || type == `ILoad && opcode[5:1] == 5'b10010)? zero_extend : sign_extend;

	// jump control
	wire [31:0] jumpaddr = {{32{type == `RJump}} & RF_rdata1} |
						{{32{type == `JType}} & instr_two_extend};

	// branch control
	wire isbranch = ((type == `REGIMM && REG[0] ^ ~Zero ) || 
					(type == `IBranch && opcode[1] == 0 && opcode[0] ^ Zero) ||
					(type == `IBranch && opcode[1:0] == 2'b10 && (~Zero || RF_rdata1 == 32'b0)) ||
					(type == `IBranch && opcode[1:0] == 2'b11 && Zero && RF_rdata1 != 32'b0) )? 1'b1:1'b0;
	
	// load control
	wire [1:0] n = ALU_result[1:0];

	wire [31:0] lb_result = (n[1] & n[0])? {{24{valid_Read_data[31]}},valid_Read_data[31:24]}:
					   ((n[1] & ~n[0])? {{24{valid_Read_data[23]}},valid_Read_data[23:16]}:
					   ((~n[1] & n[0])? {{24{valid_Read_data[15]}},valid_Read_data[15:8]}:{{24{valid_Read_data[7]}},valid_Read_data[7:0]}));
	wire [31:0] lbu_result = {{24{1'b0}},lb_result[7:0]};

	wire [31:0] lh_result = (~n[1])? {{16{valid_Read_data[15]}},valid_Read_data[15:0]}:{{16{valid_Read_data[31]}},valid_Read_data[31:16]}; 
	wire [31:0] lhu_result = {{16{1'b0}},lh_result[15:0]};

	wire [31:0] lw_result = valid_Read_data[31:0];
	wire [31:0] lwl_result = (n[1] & n[0])? valid_Read_data[31:0]: 
						((n[1] & ~n[0])? {valid_Read_data[23:0],RF_rdata2[7:0]}:
						((~n[1] & n[0])? {valid_Read_data[15:0],RF_rdata2[15:0]}:{valid_Read_data[7:0],RF_rdata2[23:0]}));

	wire [31:0] lwr_result = (~n[1] & ~n[0])? valid_Read_data[31:0]:
						((~n[1] & n[0])? {RF_rdata2[31:24],valid_Read_data[31:8]}:
						((n[1] & ~n[0])? {RF_rdata2[31:16],valid_Read_data[31:16]}:{RF_rdata2[31:8],valid_Read_data[31:24]}));

	
	wire [31:0] load_result = (opcode == 6'b100000)? lb_result:
						 ((opcode == 6'b100001)? lh_result:
						 ((opcode == 6'b100011)? lw_result:
						 ((opcode == 6'b100100)? lbu_result:
						 ((opcode == 6'b100101)? lhu_result:
						 ((opcode == 6'b100010)? lwl_result: lwr_result)))));

	//store control

	assign Address = {ALU_result[31:2],2'b00};//aligned address

	wire [3:0] sb_strb = (n[1] & n[0])? 4'b1000:
					 ((n[1] & ~n[0])? 4'b0100:
					 ((~n[1] & n[0])? 4'b0010: 4'b0001));
	wire [3:0] sh_strb = (n[1])? 4'b1100: 4'b0011;
	wire [3:0] sw_strb = 4'b1111;
	
	wire [3:0] swl_strb = {ALU_result[1] & ALU_result[0], ALU_result[1], ALU_result[1] | ALU_result[0], 1'b1};
	wire [3:0] swr_strb = {1'b1, ~(ALU_result[1] & ALU_result[0]), ~ALU_result[1], ~ALU_result[1] & ~ALU_result[0]};

	assign Write_strb = (opcode == 6'b101000)? sb_strb:
						((opcode == 6'b101001)? sh_strb:
						((opcode == 6'b101011)? sw_strb:
						((opcode == 6'b101010)? swl_strb:
						((opcode == 6'b101110)? swr_strb:swr_strb))));

	wire [31:0] sb_data = (n[1] & n[0])? {RF_rdata2[7:0],{24{1'b0}}}:
					 ((n[1] & ~n[0])? {{8{1'b0}},RF_rdata2[7:0],{16{1'b0}}}:
					 ((~n[1] & n[0])? {{16{1'b0}},RF_rdata2[7:0],{8{1'b0}}}: {{24{1'b0}},RF_rdata2[7:0]}));

	wire [31:0] sh_data = (n[1])? {RF_rdata2[15:0],{16{1'b0}}}: {{16{1'b0}},RF_rdata2[15:0]};

	wire [31:0] sw_data = RF_rdata2[31:0];

	wire [31:0] swl_data = (swl_strb == 4'b0001)? {{24{1'b0}},RF_rdata2[31:24]}:
					  ((swl_strb == 4'b0011)? {{16{1'b0}},RF_rdata2[31:16]}:
					  ((swl_strb == 4'b0111)? {{8{1'b0}},RF_rdata2[31:8]}:RF_rdata2[31:0]));

	wire [31:0] swr_data = (swr_strb == 4'b1111)? RF_rdata2[31:0]:
					  ((swr_strb == 4'b1110)? {RF_rdata2[23:0],{8{1'b0}}}:
					  ((swr_strb == 4'b1100)? {RF_rdata2[15:0],{16{1'b0}}}:{RF_rdata2[7:0],{24{1'b0}}}));

	assign Write_data = (opcode == 6'b101000)? sb_data:
						((opcode == 6'b101001)? sh_data:
						((opcode == 6'b101011)? sw_data:
						((opcode == 6'b101010)? swl_data:
						((opcode == 6'b101110)? swr_data: swr_data))));

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
				else if(type == `IStore) next_state = ST;
				else if (type == `REGIMM || type == `IBranch || opcode[5:0] == 6'b000010) 
					next_state = IF;
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
