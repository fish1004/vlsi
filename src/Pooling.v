//=====================================
//Author:		Chen Yun-Ru (May)
//Filename:		Pooling.v
//Description:	Max Pooling Operation
//Version:		0.1
//=====================================
`include "def.v"

module Pooling(
	clk,
	rst,
	en,
	Data_in,
	Data_out
);

	input clk;
	input rst;
	input en;
	input signed [`INTERNAL_BITS-1:0] Data_in;
	output signed [`INTERNAL_BITS-1:0] Data_out;
        reg  signed [`INTERNAL_BITS-1:0] temp;
	
assign Data_out=temp;	
always@(posedge clk or posedge rst)begin
  if(rst)begin
  temp<=32'd0;
  end
  else if(en==1'd0)begin
  temp<=32'd0;
  end
  else if(en==1'd1 && (Data_in>=temp))
  temp<=Data_in;
  else 
  temp<=temp;
end



endmodule
