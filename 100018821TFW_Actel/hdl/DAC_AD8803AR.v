`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    00:01:53 09/16/2008 
// Design Name: 
// Module Name:    DAC_AD8803AR 
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
`define TRIG_ADDR				3'h0
`define DONE_ADDR				3'h1
`define DATA_ADDR				3'h2
`define ADDRESS_ADDR			3'h3
`define TX_CLK_DIV_ADDR		3'h4
`define BUFFER_ADDR			3'h5
`define BIT_COUNT_ADDR		3'h6
	
module DAC_AD8803AR(
    output ILIM_DAC_CLK,
    output ILIM_DAC_SDI,
    output reg ILIM_DAC_CS = 1'b1,
	 
    output [31:0] OPB_DO,
    input [15:0] OPB_DI,
    input [2:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,				//32MHz
    input OPB_RST,
    input SYSCLK					//40MHz

    );
	 
	 reg trig = 1'b0;
	 reg done = 1'b0;
	 reg [7:0] data = 8'h0;
	 reg [2:0] address = 3'h0;
	 reg [15:0] tx_clk_div = 16'h2;
	 reg [10:0] buffer = 11'h0;
	 reg clk_en = 1'b0;
	 reg [3:0] bit_count = 4'h0; 
	 
	 wire tx_clk;
	 
	 assign ILIM_DAC_CLK = ~(tx_clk & clk_en);
	 assign ILIM_DAC_SDI = (clk_en) ? buffer[10] : 1'bz;
	 
	 CLOCK_DIV tx_clk_mod(
		.CLK_DIV(tx_clk_div),
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(tx_clk)
	 );
	
	 always@(posedge tx_clk or posedge OPB_RST) begin
	   if(OPB_RST) begin
	     bit_count   <= 4'd0;
       buffer      <= 11'd0;
       clk_en      <= 1'b0; 
       ILIM_DAC_CS <= 1'b1; 
	   end
    	else if(trig && !done) begin
			if(bit_count < 1) begin
				buffer <= {address , data};
				bit_count <= bit_count + 1;
				clk_en <= 1'b1;
				ILIM_DAC_CS <=1'b0;
			end
			else if(bit_count < 11) begin
				buffer <= {buffer[9:0],buffer[10]};
				bit_count <= bit_count + 1;
			end
			else if(bit_count < 12) begin
				bit_count <= bit_count + 1;
				ILIM_DAC_CS <= 1'b1;
			end
			else begin
				done <= 1'b1;
				clk_en <= 1'b0;
			end
		end
		else if(!trig) begin
			done <= 1'b0;
			bit_count <= 4'h0;
			ILIM_DAC_CS <= 1'b1;
		end
	end
	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TRIG_ADDR)) ? {31'b0 , trig} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `DONE_ADDR)) ? {31'b0 , done} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `DATA_ADDR)) ? {24'b0 , data} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ADDRESS_ADDR)) ? {29'b0, address} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TX_CLK_DIV_ADDR)) ? {16'b0, tx_clk_div} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `BUFFER_ADDR)) ? {21'b0, buffer} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `BIT_COUNT_ADDR)) ? {28'b0, bit_count} : 32'bz;
/* Read Access */	
	
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			trig <= 1'b0;
			data <= 8'h0;
			address <= 3'h0;
			tx_clk_div <= 16'h2;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`TRIG_ADDR: trig <= OPB_DI[0];
				`DATA_ADDR: data <= OPB_DI[7:0];
				`ADDRESS_ADDR: address <= OPB_DI[2:0];
				`TX_CLK_DIV_ADDR: tx_clk_div <= OPB_DI[15:0];
			endcase
		end	
	end
	

endmodule
