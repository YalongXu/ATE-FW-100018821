////////////////////////////////////////////////////////////////////////////////
// Company: 		Jabil Circuit, Inc.
// Engineer:		Wade Jameson
//
// Create Date:    03:43:17 03/30/06
// Module Name:    DP_RAM_2R_1W
//
// Description:
//		Dual port RAM module.
//
// Revision:
// 1.00 - 3/30/2006		Wade Jameson    - Initial Release
// 
// 0.01 - 03/30/2006	Wade Jameson    - File Created
// 
////////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps

module DP_RAM_2R_1W(
	PA_DI,
	PA_DO,
	PA_ADDR,
	PA_WE,
	PA_CLK,
	PB_DO,
	PB_ADDR,
	PB_CLK
);

	/**************************************************************************/
	/** Parameters **/
	parameter DATA_WIDTH = 8;
	parameter ADDR_WIDTH = 8;

	/**************************************************************************/
	/** Pin Definitions **/
	input [DATA_WIDTH-1:0] PA_DI;
	output [DATA_WIDTH-1:0] PA_DO;
	input [ADDR_WIDTH-1:0] PA_ADDR;
	input PA_WE;
	input PA_CLK;
	output [DATA_WIDTH-1:0] PB_DO;
	input [ADDR_WIDTH-1:0] PB_ADDR;
	input PB_CLK;
	
	/**************************************************************************/
	/** RAM Block **/
	reg [DATA_WIDTH-1:0] ram[(2**ADDR_WIDTH)-1:0];

        reg [ADDR_WIDTH-1:0] PA_RADDR;
        reg [ADDR_WIDTH-1:0] PB_RADDR;

	always@(posedge PA_CLK) begin
		if(PA_WE)
			ram[PA_ADDR] <= PA_DI;
	        PA_RADDR <= PA_ADDR;
	end

        assign PA_DO = ram[PA_RADDR];

	always@(posedge PB_CLK) begin
	        PB_RADDR <= PB_ADDR;
	end

        assign PB_DO = ram[PB_RADDR];

	/**************************************************************************/
	/** Initialization - for simulation only **/
//	initial begin
//		PA_DO = 0;
//		PB_DO = 0;
//	end

endmodule
