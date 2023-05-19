`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module shifter (
	input  [`DATA_WIDTH - 1:0] A,
	input  [              4:0] B,
	input  [              1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);

wire left = Shiftop == 2'b00;
wire cal_right = Shiftop == 2'b11;
wire log_right = Shiftop == 2'b10;

assign Result = {32{left}} & (A << B) |
	      {32{cal_right}} & (({32{A[31]}} << (6'd32 - {1'b0, B})) | (A >> B)) |
	      {32{log_right}} & (A >> B);

endmodule
