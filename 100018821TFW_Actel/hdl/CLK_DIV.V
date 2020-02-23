`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    11:43:08 07/17/2008 
// Design Name: 
// Module Name:    CLK_DIV 
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
module CLOCK_DIV(
	CLK_DIV,
	CLK_IN,
	RST,
	CLK_OUT
);
	/** Parameters **/
	parameter DIV_WIDTH = 16;

	input [(DIV_WIDTH-1):0] CLK_DIV;		// # of clk_in cycles to half clk_out cycle
	input CLK_IN;
    input RST;
	output reg CLK_OUT;
	
	reg [(DIV_WIDTH-1):0] cntr;
	
	always@(posedge CLK_IN or posedge RST) begin
	  if(RST) begin
	    cntr     <= 16'd1;
	    CLK_OUT  <= 1'b0;
	  end
	    
		else if(cntr >= CLK_DIV) begin
			cntr <= 1;
			CLK_OUT <= ~CLK_OUT;
		end
		else
			cntr <= cntr + 1;
	end
	
endmodule