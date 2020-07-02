//=====================================
//Author:		Chen Yun-Ru (May)
//Filename:		Adder.v
//Description:	Add Operation
//Version:		0.1
//=====================================
`include "def.v"

module Adder(
	Data_in1,
	Data_in2,
	Data_in3,
	Psum,
	Bias,
	Mode,
	Result
);

	input signed[`INTERNAL_BITS-1:0] Data_in1,Data_in2,Data_in3;
	input signed[`INTERNAL_BITS-1:0] Psum;
	input signed[`DATA_BITS-1:0] Bias;
	input signed[1:0] Mode;
	output reg signed[`INTERNAL_BITS-1:0] Result;

//complete your design here
always@(*) begin
case(Mode)
2'd0:Result=Data_in1+Data_in2+Data_in3;
2'd1:Result=Data_in1+Data_in2+Data_in3+Psum;
2'd2:Result=Data_in1+Data_in2+Data_in3+Psum+Bias;
default:Result=32'd0;
endcase
end
	


endmodule
