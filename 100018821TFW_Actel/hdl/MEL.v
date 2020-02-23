`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:53:30 09/17/2008 
// Design Name: 
// Module Name:    MEL 
// Project Name: 
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
`define MEL_CTRL_STATE_HOME 			3'h0
`define MEL_CTRL_STATE_MEL_OPEN		3'h1
`define MEL_CTRL_STATE_ACK				3'h2
`define MEL_CTRL_STATE_NOACK			3'h3
`define MEL_CTRL_STATE_ENABLE_OFF	3'h4

`define MEL_STATE_ADDR		5'h0
`define CNTR_RESET_ADDR		5'h1
`define ERROR_RESET_ADDR	5'h2
`define CNTR_DATA_ADDR		5'h3
`define MEL_ACK_EN_ADDR		5'h4
`define ACK_TIME_SET_ADDR	5'h5
`define FPGA2CPLD_ADDR		5'h6
`define STATE_LOG_ADDR		5'h7
`define STATE_LOG_SIZE		5'ha
`define FLOAT_CTRL_ADDR		5'h11
`define SERIAL_D_ADDR		5'h12


module MEL(
    input MEL_INT,
    input MEL_ENABLE,
    input MEL_ERROR,
    output reg MEL_ACK,
    output reg MEL_ERROR_RESET,
    input [3:1] MEL_XTRA,
	 inout [9:0] FPGA2CPLD,
	 
	 output reg FPGA2CPLD_CLK,
	 
    output [31:0] OPB_DO,
    input [15:0] OPB_DI,
    input [4:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,				//32MHz
    input OPB_RST,
    input SYSCLK					//40MHz

    );

	reg mel_ack_en = 1'b1;
	reg [6:0] state_log [9:0];		//{state, enable, int, error, }

	reg [15:0] ack_timer = 16'h0;
	reg [15:0] ack_time_set = 16'h5daa;
	reg cntr_en = 1'b0;
	reg cntr_rst = 1'b0;
	reg error_rst = 1'b0;
	reg log_clear = 1'b0;
	reg [15:0] serialInBuf = 16'h0;
	reg [15:0] serialIn = 16'h0;
	
	
	wire [31:0] cntr_data;
//	wire serialSync = MEL_XTRA[1];
//	wire serialD = MEL_XTRA[2];

	reg [15:0] clk_div = 16'd2;
	CLOCK_DIV pci_clk_div(
		.CLK_DIV(16'd2),//clk_div
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(FPGA2CPLD[0])
	);

	
	always@(negedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    serialInBuf <= 16'h0;
	  else
  		  serialInBuf <= {MEL_XTRA[2], serialInBuf[15:1]};
	end
	
	always@(posedge MEL_XTRA[1] or posedge OPB_RST) begin
	 if(OPB_RST)
	   	serialIn <= 16'h0;
	 else
			serialIn <= serialInBuf;
	end
		
	// MEL_STATE LOG
	always@(negedge SYSCLK) begin
		if(!log_clear) begin
			state_log[9] <= 7'h7f;
			state_log[8] <= 7'h7f;
			state_log[7] <= 7'h7f;
			state_log[6] <= 7'h7f;
			state_log[5] <= 7'h7f;
			state_log[4] <= 7'h7f;
			state_log[3] <= 7'h7f;
			state_log[2] <= 7'h7f;
			state_log[1] <= 7'h7f;
			state_log[0] <= 7'h7f;
			log_clear <= 1'b1;
		end
		else if((state_log[0] != { MEL_ENABLE, MEL_INT, MEL_ERROR, 1'b0, serialIn[6:4]})) begin
			state_log[9] <= state_log[8];
			state_log[8] <= state_log[7];
			state_log[7] <= state_log[6];
			state_log[6] <= state_log[5];
			state_log[5] <= state_log[4];
			state_log[4] <= state_log[3];
			state_log[3] <= state_log[2];
			state_log[2] <= state_log[1];
			state_log[1] <= state_log[0];
			state_log[0] <= { MEL_ENABLE, MEL_INT, MEL_ERROR, 1'b0, serialIn[6:4]};
		end
	end
	
	// MEL_ACK control
	always@(negedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)begin
	    MEL_ACK    <= 1'b0;
	    ack_timer  <= 16'h0;
	  end  
		else if(serialIn[6:4] == `MEL_CTRL_STATE_MEL_OPEN) begin
				if(mel_ack_en && (ack_timer > ack_time_set)) begin
					MEL_ACK <= 1'b1;
				end
				else begin
					ack_timer <= ack_timer + 1;
				end
		end
		else begin
			if(MEL_ACK)	MEL_ACK <= 1'b0;
			ack_timer <= 16'h0;
		end
	end
	
	//MEL_ERROR_RESET control
	always@(negedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    MEL_ERROR_RESET <= 1'b0;
	   else
		  MEL_ERROR_RESET <= error_rst;
	end
	
	// positive edge of MEL_INT to negative edge of MEL_ENABLE
	always@(negedge SYSCLK or posedge OPB_RST) begin
	    if(OPB_RST) 	        
            cntr_en <= 1'b0;
		else if(MEL_INT)
            cntr_en <= 1'b1;
		else if(!MEL_ENABLE)
            cntr_en <= 1'b0;
	end
	
	
	// Time out counter
	TIMER_COUNTER #(
		.DATA_WIDTH(32)				// 16 bit samples
	) counter(
		.ENABLE(cntr_en),
		.CLOCK(SYSCLK),
		.RESET(cntr_rst),
		.DATA(cntr_data)
   );
	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SERIAL_D_ADDR)) ? {15'b0 , MEL_XTRA[3], serialIn} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `MEL_STATE_ADDR)) ? {25'b0, MEL_ENABLE, MEL_INT, MEL_ERROR, 1'b0, serialIn[6:4]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CNTR_DATA_ADDR)) ? cntr_data : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ACK_TIME_SET_ADDR)) ? {16'b0 , ack_time_set} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `FPGA2CPLD_ADDR)) ? {27'b0 ,FPGA2CPLD_CLK, FPGA2CPLD} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR)) ? {26'b0 , state_log[0]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 1)) ? {25'b0 , state_log[1]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 2)) ? {25'b0 , state_log[2]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 3)) ? {25'b0 , state_log[3]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 4)) ? {25'b0 , state_log[4]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 5)) ? {25'b0 , state_log[5]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 6)) ? {25'b0 , state_log[6]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 7)) ? {25'b0 , state_log[7]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 8)) ? {25'b0 , state_log[8]} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_LOG_ADDR + 9)) ? {25'b0 , state_log[9]} : 32'bz; 
	 
			
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
		  ack_time_set  <= 16'h5daa;
		  FPGA2CPLD_CLK <= 1'b0;
		  mel_ack_en    <= 1'b1;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`CNTR_RESET_ADDR: cntr_rst <= OPB_DI[0];
				`ERROR_RESET_ADDR: error_rst <= OPB_DI[0];
				`MEL_ACK_EN_ADDR: mel_ack_en <= OPB_DI[0];
				`ACK_TIME_SET_ADDR: ack_time_set <= OPB_DI[15:0];
				`FLOAT_CTRL_ADDR: FPGA2CPLD_CLK <= OPB_DI[0];
			endcase
		end
	end
				
	

endmodule
