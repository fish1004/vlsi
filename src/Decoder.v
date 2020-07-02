//=====================================
//Author:		Chen Yun-Ru (May)
//Filename:		Decoder.v
//Description:	Index Decode
//Version:		0.1
//=====================================
`include "def.v"

module Decoder(
	clk,
	rst,
	en,
	Data_in,
	Index
);

	input clk;
	input rst;
	input en;
	input signed[`INTERNAL_BITS-1:0] Data_in;
	output [`INTERNAL_BITS-1:0] Index;
        reg signed[`INTERNAL_BITS-1:0] counter0;
        reg signed[`INTERNAL_BITS-1:0] temp;
        reg signed[`INTERNAL_BITS-1:0] temp0;  //for compare

//complete your design here
assign Index=temp;
always @(posedge clk or posedge rst)begin
  if(rst)begin
    temp<=32'd0; 
    counter0<=32'd0;
    temp0<=32'd0;    
  end
  else if(counter0==32'd0 &&en==1'd1&&(Data_in>temp0))begin
    temp<=32'd0;
    temp0<=Data_in;
    counter0<=counter0+3'd1;
  end  
  else if(en==1'd1&&(Data_in>temp0))begin
    counter0<=counter0+32'd1;
    temp<=counter0;
    temp0<=Data_in;
  end
  else if(en==1'd1)begin
    temp<=temp;
    counter0<=counter0+3'd1;
  end
  else begin
    temp<=32'd0;
    counter0<=32'd0;
    temp0<=32'd0;
  end
end
endmodule
