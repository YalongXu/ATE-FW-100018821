`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Jabil, Inc.
// Engineer: Philip Schmidt
// 
// Create Date:    10:20:43 02/19/2009 
// Design Name:    100018821 Test Firmware
// Module Name:    LED_Control 
// Project Name: 	 100018821-TFW
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`define CNTRL_ADD          3'h0    /* Control Register (R/W) 0 */
`define CLKDIV_ADD         3'h1    /* Aquasition clock divier Register (R/W) 10 */
`define DRIVE_ADD          3'h2    /* Count Register (R/W) 20 */
`define STATE_ADD				3'h3    /* Scratch Pad Register 30 */
`define MNL_ADD				3'h4	  /* Manually set the LED states */

`define LED_COUNT				8'd15

module LED_Control(
    output [31:0] OPB_DO,
    input [31:0] OPB_DI,
    input [2:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,				//32MHz
    input OPB_RST,
    input SYSCLK,
	 
    output HEARTBEAT_LED,				//0
    output CAN_RX,						//1
    output CAN_TX,						//2
    output P_SP_CAN_RX,					//3
    output P_SP_CAN_TX,					//4
    output STAT_LED1,					//5
    output STAT_LED2,					//6
    output STAT_LED3,					//7
    output SPARE_LED1,					//8
    output SPARE_LED2,					//9
    output SPARE_LED3,					//10
	 output SPARE_LED4,					//11
    output SPARE_LED5,					//12
    output SPARE_LED6,					//13
    output SRVC_MODE_LED,				//14
	 output TEN_HZ
    );
	 
     wire TenHz;
	 reg [31:0] clk_div = 32'd2000000;		// 40MHz/(2M * 2) = 10Hz
	//reg [31:0] clk_div = 32'd10;		// 40MHz/(10 * 2) = 1MHz
	 reg [3:0] control = 4'h0;
	 reg [14:0] sparedrive = 15'h3ff;
	 reg [2:0] statdrive = 3'b111;
	 reg [7:0] state = 8'h0;
	 reg [14:0] manual = 15'h2AA;
	 
	 assign HEARTBEAT_LED		= TenHz;	//= ~drive[0];
	 
	 assign STAT_LED3  		= ~statdrive[0]; 
	 assign STAT_LED2  		= ~statdrive[1]; 
	 assign STAT_LED1  		= ~statdrive[2]; 
	 
	 assign SPARE_LED6 	 	= sparedrive[0]; 
	 assign SPARE_LED5		= sparedrive[1]; 
	 assign SPARE_LED4		= sparedrive[2]; 
	 assign SPARE_LED3 		= sparedrive[3]; 
	 assign SPARE_LED2 		= sparedrive[4]; 
	 assign SPARE_LED1 		= sparedrive[5]; 
	 
	 assign CAN_RX 			= TenHz;	//~drive[10]; 
	 assign CAN_TX 			= ~TenHz;	//~drive[11]; 
	 assign P_SP_CAN_RX 	= TenHz;	//~drive[12]; 
	 assign P_SP_CAN_TX 	= ~TenHz;	//~drive[13]; 
	 
     assign SRVC_MODE_LED   = TenHz;	//= ~drive[14];


	 
	CLOCK_DIV  #(
		.DIV_WIDTH(32)
	) Clk10Hz(
		.CLK_DIV(clk_div),
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(TenHz)
	); 
	
	assign TEN_HZ = TenHz;
	
	always@(negedge TenHz ) begin //or posedge OPB_RST
		//if(OPB_RST) begin
		//	sparedrive  <= 15'h3ff;
        //    statdrive   <= 3'b111; 
		//	state       <= 8'h0;
		//end  
		if(control == 4'h0) begin
			if(state < 6) begin
				if(state == 8'h0) begin
					statdrive <= 3'h1;
                    sparedrive <= 6'h1;
				end
				else begin
					statdrive <= statdrive << 1;
                    sparedrive <= sparedrive << 1;
				end
			end
			else if(state < 12)begin
				if(state == 6) begin
					statdrive <= 3'h4;
					sparedrive <= 6'h20;
				end
				else begin
					statdrive <= statdrive >> 1;
                    sparedrive <= sparedrive >> 1;
				end
			end
		end
		else if(control == 4'h1) begin
			statdrive <= manual;
			sparedrive <= manual;
		end
		else if(control == 4'h2) begin
			statdrive <= 15'h5555;
            sparedrive <= 15'h5555;
		end
		if(state < 11)
			state <= state + 1;
		else
			state <= 0;
	end	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CNTRL_ADD)) ? {28'b0 , control} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLKDIV_ADD)) ? clk_div : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `DRIVE_ADD)) ? {23'b0, sparedrive, statdrive} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_ADD)) ? {24'b0 , state} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `MNL_ADD)) ? {17'h0 , manual} : 32'bz;
			
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			control <= 4'b0;
            manual  <= 15'h2AA;
            clk_div <= 32'd2000000;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`CNTRL_ADD: control <= OPB_DI[3:0];
				`CLKDIV_ADD: clk_div <= OPB_DI;
				`MNL_ADD: manual <= OPB_DI[14:0];
			endcase
		end	
	end
	
endmodule
