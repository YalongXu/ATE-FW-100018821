`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:47:48 08/26/2008 
// Design Name: 
// Module Name:    Timer_Counter 
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
module TIMER_COUNTER(
    ENABLE,
    CLOCK,
    RESET,
	 DATA
    );

	/**************************************************************************/
	/** Parameters **/
	parameter DATA_WIDTH = 16;

	/**************************************************************************/
	 input ENABLE;
	 input CLOCK;
	 input RESET;
   output [DATA_WIDTH-1:0] DATA;
	
	reg [DATA_WIDTH-1:0] count = 0;
	 
	assign DATA = count;
	
	always@(posedge CLOCK or posedge RESET) begin
		if(RESET) begin
			count <= 0;
		end
			
		else if(ENABLE) begin
			count <= count + 1;
		end
	end

endmodule
