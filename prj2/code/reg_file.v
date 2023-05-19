`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

	reg [`DATA_WIDTH - 1:0] rf [`DATA_WIDTH - 1:0];   //declaration
	
	always @(posedge clk) begin                        // write using sequential logic
		if (wen && waddr) rf[waddr] <= wdata;
	end
	
	assign rdata1 = (raddr1 == 5'b0)? 32'b0: rf[raddr1];                       //read using combinational logic
	assign rdata2 = (raddr2 == 5'b0)? 32'b0: rf[raddr2];
	
endmodule
