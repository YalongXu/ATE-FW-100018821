`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Jabil Inc.
// Engineer: Philip Schmidt
// 
// Create Date:    12:38:37 03/16/2009 
// Design Name: 	
// Module Name:    DAC_AD7849BR 
// Project Name: 
// Target Devices:  
// Tool versions: 10.1.02
// Description: Interface for Analog Devices AD7849B digital to analog converter
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`define TRIG_ADDR				3'h0
`define DONE_ADDR				3'h1
`define DATA_ADDR				3'h2
`define TX_CLK_ADDR			3'h3
`define BUFF_ADDR				3'h4
`define BIT_ADDR				3'h5
`define RST_CLR_ADDR			3'h6
`define AUTOLOAD_ADDR		3'h7

module DAC_AD7849BR(
    output [31:0] OPB_DO,
    input [15:0] OPB_DI,
    input [2:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,
    input OPB_RST,
    input SYSCLK,
	 
    output LDACb,
    output SCLK,
    output SDIN,
    output reg RSTb = 1'b0,
    output reg SYNC = 1'b1,
    output reg CLRb = 1'b1
    );
	 
	 reg trig = 1'b0;
	 reg done = 1'b0;
	 reg [15:0] tx_clk_div = 16'd20;
	 reg [15:0] data = 16'h0;
	 reg [15:0] buffer = 16'h0;
	 reg clk_en = 1'b0;
	 reg [4:0] bit_count = 5'h0;
	 reg AutoLoad = 1'b0;
	 reg load = 1'b1;
	 
	 wire tx_clk;
	 
	 assign SCLK = tx_clk & clk_en;
	 assign SDIN = (clk_en) ? buffer[15] : 1'bz;
	 assign LDACb = load & ~AutoLoad;
	 
	 CLOCK_DIV tx_clk_mod(
		.CLK_DIV(tx_clk_div),
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(tx_clk)
	 );
	
	 always@(posedge tx_clk or posedge OPB_RST) begin
	   if(OPB_RST)begin
	     bit_count <= 1'b0;
	     buffer    <= 16'b0;
	     clk_en    <= 1'b0;
	     SYNC      <= 1'b1;
	     load      <= 1'b1;
	     done      <= 1'b0;  
	   end
	   else if(trig && !done) begin
			if(bit_count < 1) begin
				buffer <= data;
				bit_count <= bit_count + 1;
				clk_en <= 1'b1;
				SYNC <= 1'b0;
				load <= 1'b1;
			end
			else if(bit_count < 16) begin
				buffer <= {buffer[14:0],buffer[15]};
				bit_count <= bit_count + 1;
			end
			else if(bit_count < 17) begin
				bit_count <= bit_count + 1;
				SYNC <= 1'b1;
			end
			else if(bit_count < 18) begin
				bit_count <= bit_count + 1;
				clk_en <= 1'b0;
			end
			else if(bit_count < 20) begin
				bit_count <= bit_count + 1;
			end
			else if(bit_count < 21) begin
				bit_count <= bit_count + 1;
				load <= 1'b0;
			end
			else if(bit_count < 22) begin
				bit_count <= bit_count + 1;
				load <= 1'b1;
			end
			else begin
				done <= 1'b1;
				clk_en <= 1'b0;
			end
		end
		else if(!trig) begin
			done <= 1'b0;
			bit_count <= 4'h0;
			SYNC <= 1'b1;
		end
	end
	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TRIG_ADDR)) 		? {31'b0 , trig} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `DONE_ADDR)) 		? {31'b0 , done} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `DATA_ADDR)) 		? {24'b0 , data} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TX_CLK_ADDR)) ? {16'b0, tx_clk_div} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `BUFF_ADDR)) 		? {21'b0, buffer} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `BIT_ADDR))	? {28'b0, bit_count} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RST_CLR_ADDR)) 	? {30'b0 , RSTb, CLRb} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `AUTOLOAD_ADDR)) 	? {31'b0 , AutoLoad} : 32'bz;
/* Read Access */	
	
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			trig <= 1'b0;
			data <= 8'h0;
			tx_clk_div <= 16'h2;
			RSTb <= 1'b0;
			CLRb <= 1'b1;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`TRIG_ADDR: trig <= OPB_DI[0];
				`DATA_ADDR: data <= OPB_DI[15:0];
				`TX_CLK_ADDR: tx_clk_div <= OPB_DI[15:0];
//				`TX_CLK_DIV_ADDR: tx_clk_div <= OPB_DI[15:0];
				`RST_CLR_ADDR: begin
					CLRb <= OPB_DI[0];
					RSTb <= OPB_DI[1];
				end
				`AUTOLOAD_ADDR: AutoLoad <= OPB_DI[0];
			endcase
		end	
	end

endmodule
