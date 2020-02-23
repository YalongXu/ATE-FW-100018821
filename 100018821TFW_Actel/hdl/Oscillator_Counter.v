`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:29:52 08/21/2008 
// Design Name: 
// Module Name:    Oscillator_Counter 
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

`define CNTRLR          2'h0    /* Control Register (R/W) 0 */
`define CLKDIVR         2'h1    /* Aquasition clock divier Register (R/W) 10 */
`define COUNTR          2'h2    /* Count Register (R/W) 20 */
`define SPR					2'h3    /* Scratch Pad Register 30 */

module OSCILLATOR_COUNTER(
    output [31:0] OPB_DO,
    input [31:0] OPB_DI,
    input [1:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,				//32MHz
    input OPB_RST,
    input SYSCLK					//40MHz
); 
	
	//Module						
	wire cntr_en;							// counter_en
	wire divided_pci_clk;				// OPB_CLK divided by clk_div
	wire [15:0] count_data;					// counter register
	wire [2:0] control;					// control register (control[0] = Start Measurement, control[1] = Reset Counter, control[2] = Measurement Inprogress)
	reg resetb = 1'b0;
	reg startb = 1'b0;
	reg [15:0] clk_div = 16'd1000;
	reg cntr_trig = 1'b0;
	reg [31:0] sp = 32'h1f2e3d4c;
	
	assign control[0] = startb;
	assign control[1] = resetb;
	assign control[2] = (startb || cntr_trig);
	assign cntr_en = (cntr_trig && divided_pci_clk && ~resetb);

	CLOCK_DIV pci_clk_div(
		.CLK_DIV(clk_div),
		.CLK_IN(OPB_CLK),
		.RST(OPB_RST),
		.CLK_OUT(divided_pci_clk)
	);
	
	TIMER_COUNTER counter(
		.ENABLE(cntr_en),
		.CLOCK(SYSCLK),
		.RESET(resetb),
		.DATA(count_data)
    );
		
	always@(negedge divided_pci_clk or posedge OPB_RST) begin
	  if(OPB_RST)
	    cntr_trig <= 1'b0;
		else if(control[0])
			cntr_trig <= 1'b1;
		else 
			cntr_trig <= 1'b0;
	end
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CNTRLR)) ? {29'b0 , control} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLKDIVR)) ? {16'b0 , clk_div} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `COUNTR)) ? {16'b0 , count_data} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SPR)) ? sp : 32'bz;
			
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			startb <= 1'b0;
			resetb <= 1'b0;
            clk_div <= 16'd1000;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`CNTRLR: begin
					startb <= OPB_DI[0];
					resetb <= OPB_DI[1];
				end
				`CLKDIVR: clk_div <= OPB_DI[15:0];
				`SPR: sp <= OPB_DI;
			endcase
		end
		else if(startb && cntr_trig) begin
			startb <= 1'b0;
		end
		else if(resetb && ~cntr_trig) begin
			resetb <= 1'b0;
			startb <= 1'b0;
		end	
	end
				
endmodule