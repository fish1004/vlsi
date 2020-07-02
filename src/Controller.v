//=====================================
//Author:		Chen Yun-Ru (May)
//Filename:		Controller.v
//Description:	Controller
//Version:		0.1
//=====================================
`include "../include/def.v"

module Controller(
	clk,
	rst,
	START,
	DONE,
	//ROM
	ROM_IM_CS,ROM_W_CS,ROM_B_CS,
	ROM_IM_OE,ROM_W_OE,ROM_B_OE,
	ROM_IM_A,ROM_W_A,ROM_B_A,
	//SRAM
	SRAM_CENA,SRAM_CENB,
	SRAM_WENB,
	SRAM_AA,SRAM_AB,
	//PE
	PE1_IF_w,PE2_IF_w,PE3_IF_w,
	PE1_W_w,PE2_W_w,PE3_W_w,
	//Pooling
	Pool_en,
	//Decoder
	Decode_en,
	//Adder
	Adder_mode,
	//MUX
	MUX1_sel,MUX2_sel,MUX3_sel
);
  parameter INIT    = 5'b00000,      //0
            READ_W  = 5'b00001,     //1
            WRITE   = 5'b00010,        //2
            READ_C  = 5'b00011,//3
            READ_9  = 5'b00100,//4
            Pool0    =5'b00101,//5
            Pool0_w  =5'b00110,//6
            DONE0    =5'b00111,//7
            INIT2    =5'b01000,//8
            READ_W2  = 5'b01001,//9
            WRITE2   = 5'b01010,//10
            READ_C2  = 5'b01011,//11
            READ_92  = 5'b01100,//12
            Pool3    = 5'b01101,//13
            Pool3_w  = 5'b01110,//14
            Full     = 5'b01111,//15
            Full_w   = 5'b10000,//16
            Full1    = 5'b10001,//17
            Full1_w  = 5'b10010,//18
            Decode   = 5'b10011;//19
	input clk;
	input rst;
	input START;
	output reg DONE;
	output reg ROM_IM_CS,ROM_W_CS,ROM_B_CS;
	output reg ROM_IM_OE,ROM_W_OE,ROM_B_OE;
	output reg [`ROM_IM_ADDR_BITS-1:0] ROM_IM_A;
	output reg [`ROM_W_ADDR_BITS-1:0] ROM_W_A;
	output reg [`ROM_B_ADDR_BITS-1:0] ROM_B_A;
	output reg SRAM_CENA,SRAM_CENB;
	output reg SRAM_WENB;
	output reg [`SRAM_ADDR_BITS-1:0] SRAM_AA,SRAM_AB;                       //
	output reg PE1_IF_w,PE2_IF_w,PE3_IF_w;       
	output reg PE1_W_w,PE2_W_w,PE3_W_w;
	output reg Pool_en; 
	output reg Decode_en;
	output reg [1:0] Adder_mode;
	output reg [1:0] MUX2_sel;
	output reg MUX1_sel,MUX3_sel; 
        reg    [3:0] counter;                 //for PE convolution
        reg    [`ROM_IM_ADDR_BITS-1:0] column0 ;   // for 28bits layer0
        reg    [`ROM_IM_ADDR_BITS-1:0] row0    ;   // for 28bits layer0  
        reg   init; //for row0
        reg   [`ROM_W_ADDR_BITS-1:0] rom_w;// for weight
        reg   [`ROM_B_ADDR_BITS-1:0] rom_b;// for bias
        reg   [4:0]  state,n_state;     // for state
        reg   [12:0]  pool_ram,pool_row,pool_column,pool_row_f;//for pool0
        reg   [12:0]  conv2_AA,conv2_temp,conv2_temp0,conv2_temp1; //for conv2   temp0 for AA
        reg   [2:0]   conv2_init;
        reg   [12:0] full_count,full_f,full_AB;  // for fully connective  f=feature w=weight 
        reg   [`ROM_W_ADDR_BITS-1:0] full_w;
        reg   [12:0]  decode_column;

always@(posedge clk or posedge rst)begin  //change state
if(rst)
	state<=INIT;
else
	state<=n_state;
end
always@(*)begin             //state change for PE       conv0
	case(state)
	INIT: n_state=READ_W;
	READ_W: n_state=(counter==4'd10)? WRITE:READ_W;                       
	WRITE:  n_state=(column0==20'd29&&row0!=20'd27)? READ_9 : (row0!=20'd28 && column0!=20'd29)? READ_C : (row0==20'd27&&rom_w==17'd45)? Pool0:INIT;
	READ_9: n_state=(counter==4'd10)? WRITE :READ_9;      
	READ_C: n_state=(counter==4'd4)? WRITE : READ_C;
        Pool0:  n_state=(pool_ram==13'd5880)? INIT2:(counter==4'd6)? Pool0_w:Pool0;            
        Pool0_w: n_state= Pool0;
	INIT2: n_state=READ_W2;
	READ_W2: n_state=(counter==4'd11)? WRITE2:READ_W2;                       
	WRITE2:  n_state=(column0==20'd13&&row0!=20'd11)? READ_92 : (row0!=20'd12 && column0!=20'd13)? READ_C2 : (row0==20'd11&&rom_w==17'd855)?Pool3:INIT2;              
	READ_92: n_state=(counter==4'd11)? WRITE2 :READ_92;      
	READ_C2: n_state=(counter==4'd5)? WRITE2 : READ_C2;
        Pool3:  n_state=(pool_ram==13'd5244)? Full:(counter==4'd6)? Pool3_w:Pool3;            
        Pool3_w: n_state= Pool3;
        Full:    n_state=(full_w==17'd98064)? Full1:(counter==4'd11)? Full_w :Full;
        Full_w:  n_state=Full;
        Full1:   n_state=(full_w==17'd99864)? Decode:(counter==4'd11)? Full1_w :Full1;
        Full1_w: n_state=Full1;
	DONE0 : n_state= DONE0;
        Decode: n_state=(counter==4'd12)? DONE0:Decode;
	default: n_state=INIT;
	endcase
end

always@(posedge clk or posedge rst)begin  //for conv0
    if(rst)begin
      counter <=4'd0;
      column0<=10'd0;
      row0<=10'd0;
      rom_w<=17'd0;
      rom_b<=8'd0;
      SRAM_AB<=13'd0;
      pool_ram<=13'd0;
      init<=1'd1;
      conv2_init<=3'd0;
      full_w<=17'd0;
      Adder_mode<=2'd0;
      MUX2_sel<=2'd0;     
    end
    else if(state==INIT)begin
      if(init==1'd0)begin
        rom_w<=rom_w+17'd9;
        rom_b<=rom_b+8'd1;
        row0<=10'd0;
        Adder_mode<=2'd2;
        MUX3_sel<=1'd1;
        MUX2_sel<=2'd0;
      end
      else begin
        rom_w<=17'd0;
        rom_b<=8'd0;
        row0<=10'd0;
      end
    end 
    else if(state==READ_W)begin
      counter <=counter + 4'd1;               //counter for PE
      column0<=10'd2;
      init<=1'd1;
      ROM_B_A<=rom_b;
     end
    else if(state==WRITE)begin
      if(column0==10'd29 )begin
        row0 <= row0+10'd1;
        counter <= 4'd0;
        column0<=column0+10'd1;
        SRAM_AB<=13'd1+SRAM_AB;
      end
      else if(column0==10'd2 && init==1'd1) begin
        row0 <= 10'd0;
        counter <= 4'd0;
        column0<=column0+10'd1;
        SRAM_AB<=13'd1+SRAM_AB;
      end
      else begin
        counter <= 4'd0;
        column0<=column0+10'd1;
        SRAM_AB<=13'd1+SRAM_AB;
      end
    end
    else if(state==READ_C)begin
      counter <= counter+4'd1;
    end
    else if(state==READ_9)begin
      counter <= counter+4'd1;
      column0 <= 10'd2;
      init<=1'd0;
    end
    else if(state==Pool0)begin             //for pool
      if(init==1'd0)begin
      counter <= 4'd0;
      init<=1'd1;
      pool_ram<=13'd4704;
      pool_column<=13'd0;
      pool_row<=13'd0;
      MUX2_sel<=2'd2;
      end
      else begin
        counter <= counter+4'd1;
      end
    end
    else if(state==Pool0_w)begin
      if(pool_column==13'd26 && pool_row_f==13'd26)begin
        pool_row<=pool_row+13'd28;
        pool_column<=13'd0;
        pool_row_f<=13'd0;
      end  
      else if(pool_column==13'd26)begin
        pool_row<=pool_row+13'd2;
        pool_ram<=pool_ram+13'd1;         //for AB
        pool_row_f<=pool_row_f+13'd2;
        pool_column<=13'd0;                   //for AA  pool1
        counter<=4'd0;
      end
      else begin
      pool_row<=pool_row;
      pool_ram<=pool_ram+13'd1;                   
      pool_column<=pool_column+13'd2;
      pool_row_f<=pool_row_f;                   
      counter<=4'd0;
      end
    end   
    else if(state==INIT2 )begin
      if(init==1'd1 &&rom_b==8'd5)begin
        init<=1'd0;
        rom_w<=rom_w+17'd9;
        rom_b<=rom_b+8'd1;
        row0<=10'd0;
        SRAM_AB<=13'd0;
        conv2_AA<=13'd4704;
        conv2_init<=3'd0;
        MUX3_sel<=1'd1;
        conv2_temp<=13'd0;
        conv2_temp1<=13'd0;
        conv2_temp0<=13'd0;
        MUX2_sel<=2'd1;
        Adder_mode<=2'd1;
      end
      else if(init==1'd1 || conv2_init==3'd4)begin
        init<=1'd0;
        rom_w<=rom_w+17'd9;
        conv2_AA<=conv2_AA+13'd196;
        conv2_init<=conv2_init+3'd1;
        MUX3_sel<=1'd0;
        SRAM_AB<=conv2_temp;
        conv2_temp1<=conv2_temp0;
        MUX2_sel<=2'd0;
        Adder_mode<=2'd2;
        row0<=10'd0;
      end      
      else if(init==1'd1 || conv2_init==3'd5)begin
        init<=1'd0;
        rom_w<=rom_w+17'd9;
        rom_b<=rom_b+8'd1;
        conv2_init<=3'd0;
        MUX3_sel<=1'd1;
        conv2_AA<=13'd4704;
        conv2_temp<=conv2_temp+13'd144;
        conv2_temp0<=conv2_temp0+13'd144;
        MUX2_sel<=2'd1;
        Adder_mode<=2'd1;
        row0<=10'd0;
      end
      else begin
        rom_w<=rom_w+17'd9;
        row0<=10'd0;
        conv2_AA<=conv2_AA+13'd196;
        conv2_init<=conv2_init+3'd1;
        MUX3_sel<=1'd0;
        SRAM_AB<=conv2_temp;
        conv2_temp1<=conv2_temp0;
        MUX2_sel<=2'd1;  
        Adder_mode<=2'd1;
      end 
    end
    else if(state==READ_W2)begin
      counter <=counter + 4'd1;               
      column0<=10'd2;       
      init<=1'd1;
      ROM_B_A<=rom_b;
    end
    else if(state==WRITE2)begin
      if(column0==10'd13 )begin
        row0 <= row0+10'd1;
        counter <= 4'd0;
        column0<=column0+10'd1;
        SRAM_AB<=13'd1+SRAM_AB;
        conv2_temp1<=conv2_temp1+13'd1;
      end
      else if(column0==10'd2 && init==1'd1) begin
        row0 <= 10'd0;
        counter <= 4'd0;
        column0<=column0+10'd1;
        SRAM_AB<=13'd1+SRAM_AB;
        conv2_temp1<=conv2_temp1+13'd1;
      end
      else begin
        counter <= 4'd0;
        column0<=column0+10'd1;
        SRAM_AB<=13'd1+SRAM_AB;
        conv2_temp1<=conv2_temp1+13'd1;
      end
    end
    else if(state==READ_C2)begin
      counter <= counter+4'd1;
    end
    else if(state==READ_92)begin
      counter <= counter+4'd1;
      column0 <= 10'd2;
      init<=1'd0;
    end
    else if(state==Pool3)begin             //for pool
      if(init==1'd0)begin
      counter <= 4'd0;
      init<=1'd1;
      pool_ram<=13'd4704;
      pool_column<=13'd0;
      pool_row<=13'd0;
      pool_row_f<=13'd0;
      MUX2_sel<=2'd2;
      end
      else begin
        counter <= counter+4'd1;
      end
    end
    else if(state==Pool3_w)begin
       if(pool_column==13'd10)begin
        pool_row<=pool_row+13'd2;
        pool_ram<=pool_ram+13'd1;         //for AB
        pool_row_f<=pool_row_f+13'd2;
        pool_column<=13'd0;                   //for AA  pool1
        counter<=4'd0;
      end
      else begin
        pool_row<=pool_row;
        pool_ram<=pool_ram+13'd1;                   
        pool_column<=pool_column+13'd2;
        pool_row_f<=pool_row_f;                   
        counter<=4'd0;
      end
    end
    else if(state==Full)begin
      if(init==1'd1)begin
        full_f<=13'd4704;
        full_w<=17'd864;
        counter<=4'd0;
        init<=1'd0;
        full_count<=13'd0;
        rom_b<=rom_b+8'd1;
        full_AB<=13'd0;
        MUX3_sel<=1'd1;
        MUX2_sel<=2'd1;
        Adder_mode<=2'd1;      
      end
      else if(full_count==13'd59)begin
        counter<=counter+4'd1;
        ROM_B_A<=rom_b;
        MUX2_sel<=2'd0;
        Adder_mode<=2'd2;   
      end
      else begin
        counter<=counter+4'd1;
        ROM_B_A<=rom_b;
        MUX2_sel<=2'd1;
        Adder_mode<=2'd1;   
      end
    end              
    else if(state==Full_w)begin
      if(full_count==13'd59)begin
        full_f<=13'd4704;
        full_w<=full_w+17'd9;
        counter<=4'd0;
        full_count<=13'd0;
        rom_b<=rom_b+8'd1;
        full_AB<=full_AB+13'd1;
        MUX3_sel<=1'd1;
      end
      else begin
        full_f<=full_f+13'd9;
        full_w<=full_w+17'd9;
        counter<=4'd0;
        full_count<=full_count+13'd1;
        rom_b<=rom_b;
        MUX3_sel<=1'd0;
      end 
    end
    else if(state==Full1)begin
      if(init==1'd0)begin
        full_f<=13'd0;
        full_w<=17'd98064;
        counter<=4'd0;
        init<=1'd1;
        full_count<=13'd0;
        rom_b<=rom_b;
        full_AB<=13'd4704;
        MUX3_sel<=1'd1;
        Adder_mode<=2'd1;
        MUX2_sel<=2'd1;       
      end
      else if(full_count==13'd19)begin
        counter<=counter+4'd1;
        ROM_B_A<=rom_b;
        Adder_mode<=2'd2;  
      end
      else begin
        counter<=counter+4'd1;
        ROM_B_A<=rom_b;
        Adder_mode<=2'd1;  
      end
    end              
    else if(state==Full1_w)begin
      if(full_count==13'd19)begin
        full_f<=13'd0;
        full_w<=full_w+17'd9;
        counter<=4'd0;
        full_count<=13'd0;
        rom_b<=rom_b+8'd1;
        full_AB<=full_AB+13'd1;
        MUX3_sel<=1'd1;
      end
      else begin
        full_f<=full_f+13'd9;
        full_w<=full_w+17'd9;
        counter<=4'd0;
        full_count<=full_count+13'd1;
        rom_b<=rom_b;
        MUX3_sel<=1'd0;
      end 
    end
    else if(state==Decode)begin
      if(init==1'd1)begin
        counter<=4'd0;
        init<=1'd0;
        MUX2_sel<=2'd3;
      end
      else  begin
        counter<=counter+4'd1;
      end
    end
end    

always@(*)begin                 //for conv0's each state
case(state)
INIT:begin                          //INIT
  ROM_IM_A=10'd0;
  PE1_IF_w=1'd0;
  PE2_IF_w=1'd0;
  PE3_IF_w=1'd0;
  ROM_W_A= 17'd0;
  PE1_W_w=1'd0;
  PE2_W_w=1'd0;
  PE3_W_w=1'd0;
  Pool_en=1'd0;
  Decode_en=1'd0;
end
READ_W :begin  
  if(counter==4'd0)begin   //read_W
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;   
  end        
  else if(counter==4'd1)begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd2)begin
    ROM_IM_A=10'd1;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd1;
    PE1_W_w=1'd1;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end	
  else if(counter==4'd3)begin
    ROM_IM_A=10'd2;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd2;
    PE1_W_w=1'd1;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd4)begin
    ROM_IM_A=10'd30;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd3;
    PE1_W_w=1'd1;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd5)begin
    ROM_IM_A=10'd31;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd4;
    PE1_W_w=1'd0;
    PE2_W_w=1'd1;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd6)begin
    ROM_IM_A=10'd32;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd5;
    PE1_W_w=1'd0;
    PE2_W_w=1'd1;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd7)begin
    ROM_IM_A=10'd60;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd6;
    PE1_W_w=1'd0;
    PE2_W_w=1'd1;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd8)begin
    ROM_IM_A=10'd61;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= rom_w+17'd7;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd1;
  end		
  else if(counter==4'd9)begin
    ROM_IM_A=10'd62;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= rom_w+17'd8;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd1;
  end	
  else if(counter==4'd10)begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd1;
  end
end
WRITE:begin                       //Write
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
end
READ_C:begin                  //read_C
  if(counter==4'd0)begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd1)begin
    ROM_IM_A=((row0<<5)-(row0<<1))+column0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd2)begin
    ROM_IM_A=((row0+10'd1)<<5)-((row0+10'd1)<<1)+column0;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd3)begin
    ROM_IM_A=((row0+10'd2)<<5)-((row0+10'd2)<<1)+column0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd4)begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end

end
READ_9:begin                             //read_9
  if(counter==4'd0)begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd1)begin
    ROM_IM_A=(row0<<5)-(row0<<1);
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd2)begin
    ROM_IM_A=10'd1+(row0<<5)-(row0<<1);
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd3)begin
    ROM_IM_A=10'd2+(row0<<5)-(row0<<1);
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd4)begin
    ROM_IM_A=((row0+10'd1)<<5)-((row0+10'd1)<<1);
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd5)begin
    ROM_IM_A=10'd1+((row0+10'd1)<<5)-((row0+10'd1)<<1);
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd6)begin
    ROM_IM_A=10'd2+((row0+10'd1)<<5)-((row0+10'd1)<<1);
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd7)begin
    ROM_IM_A=((row0+10'd2)<<5)-((row0+10'd2)<<1);
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd8)begin
    ROM_IM_A=10'd1+((row0+10'd2)<<5)-((row0+10'd2)<<1);
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd9)begin
    ROM_IM_A=10'd2+((row0+10'd2)<<5)-((row0+10'd2)<<1);
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd10)begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
end
Pool0:begin
  if(counter==4'd0)begin
    SRAM_AB=13'd0;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
  end
  else if(counter==4'd1)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+(pool_row<<5)-(pool_row<<2);
    Pool_en=1'd0;
  end
  else if(counter==4'd2)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+13'd1+(pool_row<<5)-(pool_row<<2);
    Pool_en=1'd1;
  end
  else if(counter==4'd3)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+((pool_row+13'd1)<<5)-((pool_row+13'd1)<<2);
    Pool_en=1'd1;
  end
  else if(counter==4'd4)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+((pool_row+13'd1)<<5)-((pool_row+13'd1)<<2)+13'd1;
    Pool_en=1'd1;
  end
  else if(counter==4'd5)begin
    SRAM_AB=pool_ram;
    SRAM_AA=13'd0;
    Pool_en=1'd1;
  end
  else if(counter==4'd6)begin
    SRAM_AB=pool_ram;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
  end
  else begin
    SRAM_AB=13'd0;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
  end

end
Pool0_w:begin
    SRAM_AB=13'd0;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
end
INIT2:begin                          //INIT2
  SRAM_AA=13'd0;
  PE1_IF_w=1'd0;
  PE2_IF_w=1'd0;
  PE3_IF_w=1'd0;
  ROM_W_A= 17'd0;
  PE1_W_w=1'd0;
  PE2_W_w=1'd0;
  PE3_W_w=1'd0;
end
READ_W2 :begin  
  if(counter==4'd0)begin   //read_W2
    SRAM_AA=13'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;   
  end        
  else if(counter==4'd1)begin
    SRAM_AA=conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd2)begin
    SRAM_AA=conv2_AA+13'd1;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd1;
    PE1_W_w=1'd1;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end	
  else if(counter==4'd3)begin
    SRAM_AA=conv2_AA+13'd2;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd2;
    PE1_W_w=1'd1;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd4)begin
    SRAM_AA=conv2_AA+13'd14;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd3;
    PE1_W_w=1'd1;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd5)begin
    SRAM_AA=conv2_AA+13'd15;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd4;
    PE1_W_w=1'd0;
    PE2_W_w=1'd1;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd6)begin
    SRAM_AA=conv2_AA+13'd16;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= rom_w+17'd5;
    PE1_W_w=1'd0;
    PE2_W_w=1'd1;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd7)begin
    SRAM_AA=conv2_AA+13'd28;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    SRAM_AA=conv2_AA+13'd28;
    ROM_W_A= rom_w+17'd6;
    PE1_W_w=1'd0;
    PE2_W_w=1'd1;
    PE3_W_w=1'd0;
  end		
  else if(counter==4'd8)begin
    SRAM_AA=conv2_AA+13'd29;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= rom_w+17'd7;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd1;
  end		
  else if(counter==4'd9)begin
    SRAM_AA=conv2_AA+13'd30;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= rom_w+17'd8;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd1;
  end	
  else if(counter==4'd10)begin
    SRAM_AA=conv2_temp1;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd1;
  end
  else if(counter==4'd11)begin
    SRAM_AA=conv2_temp1;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
end
WRITE2:begin                       //Write
    SRAM_AA=13'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
end
READ_C2:begin                  //read_C
  if(counter==4'd0)begin
    SRAM_AA=13'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd1)begin
    SRAM_AA=((row0<<4)-(row0<<1))+column0+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd2)begin
    SRAM_AA=((row0+13'd1)<<4)-((row0+13'd1)<<1)+column0+conv2_AA;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd3)begin
    SRAM_AA=((row0+10'd2)<<4)-((row0+10'd2)<<1)+column0+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd4)begin
    SRAM_AA=conv2_temp1;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd5)begin
    SRAM_AA=conv2_temp1;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
end
READ_92:begin                             //read_9
  if(counter==4'd0)begin
    SRAM_AA=13'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd1)begin
    SRAM_AA=(row0<<4)-(row0<<1)+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd2)begin
    SRAM_AA=13'd1+(row0<<4)-(row0<<1)+conv2_AA;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd3)begin
    SRAM_AA=13'd2+(row0<<4)-(row0<<1)+conv2_AA;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd4)begin
    SRAM_AA=((row0+13'd1)<<4)-((row0+13'd1)<<1)+conv2_AA;
    PE1_IF_w=1'd1;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd5)begin
    SRAM_AA=13'd1+((row0+13'd1)<<4)-((row0+13'd1)<<1)+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd6)begin
    SRAM_AA=13'd2+((row0+13'd1)<<4)-((row0+13'd1)<<1)+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd7)begin
    SRAM_AA=((row0+13'd2)<<4)-((row0+13'd2)<<1)+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd1;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd8)begin
    SRAM_AA=13'd1+((row0+10'd2)<<4)-((row0+13'd2)<<1)+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd9)begin
    SRAM_AA=13'd2+((row0+13'd2)<<4)-((row0+13'd2)<<1)+conv2_AA;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd10)begin
    SRAM_AA=conv2_temp1;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd1;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
  else if(counter==4'd11)begin
    SRAM_AA=conv2_temp1;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
  end
end
Pool3:begin
  if(counter==4'd0)begin
    SRAM_AB=13'd0;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
  end
  else if(counter==4'd1)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+(pool_row<<4)-(pool_row<<2);
    Pool_en=1'd0;
  end
  else if(counter==4'd2)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+13'd1+(pool_row<<4)-(pool_row<<2);
    Pool_en=1'd1;
  end
  else if(counter==4'd3)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+((pool_row+13'd1)<<4)-((pool_row+13'd1)<<2);
    Pool_en=1'd1;
  end
  else if(counter==4'd4)begin
    SRAM_AB=pool_ram;
    SRAM_AA=pool_column+((pool_row+13'd1)<<4)-((pool_row+13'd1)<<2)+13'd1;
    Pool_en=1'd1;
  end
  else if(counter==4'd5)begin
    SRAM_AB=pool_ram;
    SRAM_AA=13'd0;
    Pool_en=1'd1;
  end
  else if(counter==4'd6)begin
    SRAM_AB=pool_ram;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
  end
  else begin
    SRAM_AB=13'd0;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
  end

end
Pool3_w:begin
    SRAM_AB=13'd0;
    SRAM_AA=13'd0;
    Pool_en=1'd0;
end
Full:begin
       if(counter==4'd0)begin
         SRAM_AB=13'd0;
         SRAM_AA=13'd0;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
         
       end
       else if(counter==4'd1)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end
       else if(counter==4'd2)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+17'd1;
         PE1_IF_w=1'd1;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd1;
         PE1_W_w=1'd1;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd3)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+17'd2;
         PE1_IF_w=1'd1;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd2;
         PE1_W_w=1'd1;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd4)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+17'd3;
         PE1_IF_w=1'd1;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd3;
         PE1_W_w=1'd1;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd5)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+17'd4;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd1;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd4;
         PE1_W_w=1'd0;
         PE2_W_w=1'd1;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd6)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd5;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd1;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd5;
         PE1_W_w=1'd0;
         PE2_W_w=1'd1;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd7)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd6;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd1;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd6;
         PE1_W_w=1'd0;
         PE2_W_w=1'd1;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd8)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd7;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd1;
         ROM_W_A= full_w+17'd7;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd1;
       end   
       else if(counter==4'd9)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd8;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd1;
         ROM_W_A= full_w+17'd8;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd1;
       end   
       else if(counter==4'd10)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_AB;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd1;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd1;
       end    
       else if(counter==4'd11)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_AB;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end      

     end
Full_w:begin
         SRAM_AB=full_AB;
         SRAM_AA=13'd0;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;           
       end
Full1:begin
       if(counter==4'd0)begin
         SRAM_AB=13'd0;
         SRAM_AA=13'd0;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end
       else if(counter==4'd1)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end
       else if(counter==4'd2)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd1;
         PE1_IF_w=1'd1;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd1;
         PE1_W_w=1'd1;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd3)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd2;
         PE1_IF_w=1'd1;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd2;
         PE1_W_w=1'd1;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd4)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd3;
         PE1_IF_w=1'd1;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd3;
         PE1_W_w=1'd1;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd5)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+17'd4;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd1;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd4;
         PE1_W_w=1'd0;
         PE2_W_w=1'd1;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd6)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd5;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd1;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd5;
         PE1_W_w=1'd0;
         PE2_W_w=1'd1;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd7)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd6;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd1;
         PE3_IF_w=1'd0;
         ROM_W_A= full_w+17'd6;
         PE1_W_w=1'd0;
         PE2_W_w=1'd1;
         PE3_W_w=1'd0;
       end    
       else if(counter==4'd8)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd7;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd1;
         ROM_W_A= full_w+17'd7;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd1;
       end   
       else if(counter==4'd9)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_f+13'd8;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd1;
         ROM_W_A= full_w+17'd8;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd1;
       end   
       else if(counter==4'd10)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_AB;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd1;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd1;
       end    
       else if(counter==4'd11)begin
         SRAM_AB=full_AB;
         SRAM_AA=full_AB;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;
       end      

     end
Full1_w:begin
         SRAM_AB=full_AB;
         SRAM_AA=13'd0;
         PE1_IF_w=1'd0;
         PE2_IF_w=1'd0;
         PE3_IF_w=1'd0;
         ROM_W_A= 17'd0;
         PE1_W_w=1'd0;
         PE2_W_w=1'd0;
         PE3_W_w=1'd0;           
       end
Decode:begin
         if(counter==4'd0)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd0;
           Decode_en=1'd0;
         end
         else if(counter==4'd1)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4704;
           Decode_en=1'd0;
         end
         else if(counter==4'd2)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4705;
           Decode_en=1'd1;
         end
         else if(counter==4'd3)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4706;
           Decode_en=1'd1;
         end
         else if(counter==4'd4)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4707;
           Decode_en=1'd1;
         end
         else if(counter==4'd5)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4708;
           Decode_en=1'd1;
         end
         else if(counter==4'd6)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4709;
           Decode_en=1'd1;
         end
         else if(counter==4'd7)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4710;
           Decode_en=1'd1;
         end
         else if(counter==4'd8)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4711;
           Decode_en=1'd1;
         end
         else if(counter==4'd9)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4712;
           Decode_en=1'd1;
         end
         else if(counter==4'd10)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd4713;
           Decode_en=1'd1;
         end
         else if(counter==4'd11)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd0;
           Decode_en=1'd1;
         end
         else if(counter==4'd12)begin
           SRAM_AB=13'd0;
           SRAM_AA=13'd0;
           Decode_en=1'd0;
         end
           

       end
DONE0: begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
end
default:begin
    ROM_IM_A=10'd0;
    PE1_IF_w=1'd0;
    PE2_IF_w=1'd0;
    PE3_IF_w=1'd0;
    ROM_W_A= 17'd0;
    PE1_W_w=1'd0;
    PE2_W_w=1'd0;
    PE3_W_w=1'd0;
end
endcase
end

always@(*)begin  //for conv0 signal
case(state)
INIT:begin
  DONE=1'd0;
  ROM_IM_CS=1'd0;
  ROM_W_CS=1'd0;
  ROM_B_CS=1'd0;
  ROM_IM_OE=1'd0;
  ROM_W_OE=1'd0;
  ROM_B_OE=1'd0;
  SRAM_CENA=1'd0;
  SRAM_CENB=1'd0;
  SRAM_WENB=1'd0;
  MUX1_sel=1'd0;
end
READ_W:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
WRITE:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
READ_C:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
READ_9:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
Pool0:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;

end
Pool0_w:begin
  DONE=1'd0;
  ROM_IM_CS=1'd0;
  ROM_W_CS=1'd0;
  ROM_B_CS=1'd0;
  ROM_IM_OE=1'd0;
  ROM_W_OE=1'd0;
  ROM_B_OE=1'd0;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
INIT2:begin
  DONE=1'd0;
  ROM_IM_CS=1'd0;
  ROM_W_CS=1'd0;
  ROM_B_CS=1'd0;
  ROM_IM_OE=1'd0;
  ROM_W_OE=1'd0;
  ROM_B_OE=1'd0;
  SRAM_CENA=1'd0;
  SRAM_CENB=1'd0;
  SRAM_WENB=1'd0;
  MUX1_sel=1'd0;
end
READ_W2:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd0;
  MUX1_sel=1'd1;
end
WRITE2:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd1;
end
READ_C2:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd0;
  MUX1_sel=1'd1;
end
READ_92:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd0;
  MUX1_sel=1'd1;
end
Pool3:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;

end
Pool3_w:begin
  DONE=1'd0;
  ROM_IM_CS=1'd0;
  ROM_W_CS=1'd0;
  ROM_B_CS=1'd0;
  ROM_IM_OE=1'd0;
  ROM_W_OE=1'd0;
  ROM_B_OE=1'd0;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
Full:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd0;
  MUX1_sel=1'd1;
end
Full_w:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd1;
end
Full1:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd0;
  MUX1_sel=1'd1;
end
Full1_w:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd1;
end
Decode:begin
  DONE=1'd0;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd1;
end
DONE0:begin
  DONE=1'd1;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
default:begin
  DONE=1'd1;
  ROM_IM_CS=1'd1;
  ROM_W_CS=1'd1;
  ROM_B_CS=1'd1;
  ROM_IM_OE=1'd1;
  ROM_W_OE=1'd1;
  ROM_B_OE=1'd1;
  SRAM_CENA=1'd1;
  SRAM_CENB=1'd1;
  SRAM_WENB=1'd1;
  MUX1_sel=1'd0;
end
endcase
end                      


	



			



	
endmodule
