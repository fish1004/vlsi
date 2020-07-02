//=====================================
//Author:		Chen Yun-Ru (May)
//Filename:		PE.v
//Description:	MAC Operation
//Version:		0.1
//=====================================
`include "def.v"

module PE( 
	clk,
	rst,
	IF_w,
	W_w,
	IF_in,
	W_in,
	Result
);

	input clk;
	input rst;
	input IF_w,W_w;
	input [`DATA_BITS-1:0] IF_in,W_in; 
	output [`INTERNAL_BITS-1:0] Result;
	
    reg signed [`DATA_BITS-1:0] weight [2:0];
    reg signed [`DATA_BITS-1:0] feature [2:0];

//complete your design here
assign Result = weight[0]*feature[0]+weight[1]*feature[1]+weight[2]*feature[2];

	always @(posedge clk or posedge rst)begin
	  if(rst)begin
	    weight[0] <= 16'b0;
	    weight[1] <= 16'b0;
	    weight[2] <= 16'b0;
	    feature[0] <= 16'b0;
	    feature[1] <= 16'b0;
	    feature[2] <= 16'b0;
	  end
	  else if(W_w && IF_w)begin
	    weight[0] <= W_in;
	    weight[1] <= weight[0];
	    weight[2] <= weight[1];	
	    feature[0] <= IF_in;
	    feature[1] <= feature[0];
	    feature[2] <= feature[1];
	  end
	  else if(W_w)begin
	    weight[0] <= W_in;
	    weight[1] <= weight[0];
	    weight[2] <= weight[1];
	  end
	  else if(IF_w)begin
	    feature[0] <= IF_in;
	    feature[1] <= feature[0];
	    feature[2] <= feature[1];
	  end
	end


	
	
endmodule
