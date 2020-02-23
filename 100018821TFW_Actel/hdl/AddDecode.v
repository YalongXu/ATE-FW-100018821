`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    11:41:10 01/27/2009 
// Design Name: 
// Module Name:    8821_AddDecode 
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
`define SP1_ADDR			32'h000000		/* Scratch Pad 1 address */

`define OSC_ADDR		32'h000010		/* COUNTER address */
`define OSC_SIZE		32'h000010		/* COUNTER size */

`define LED_ADDR			32'h000020
`define LED_SIZE			32'h000014

`define DIO_ADDR			32'h000040		/* Digital Inputs Address */
`define DIO_SIZE			32'h000038		/* Digital Outputs Address */

`define LDAC_ADDR			32'h000080
`define LDAC_SIZE			32'h000020

`define RS485_ADDR		32'h000100		/* RS485 test module address */
`define RS485_SIZE		32'h0000c8		/* RS485 test module size */

`define BRG1_ADDR			32'h000200		/* BRG1 module address */
`define BRG2_ADDR			32'h000240		/* BRG2 module address */
`define COIL1_ADDR		32'h000280		/* BRK1 module address */
`define COIL2_ADDR		32'h0002c0		/* BRK2 test module address */
`define BRG_SIZE			32'h000024		/* BRG module size */

`define ILIM_DAC_ADDR	32'h000300		/* ILIM_DAC module address */
`define ILIM_DAC_SIZE	32'h00001c		/* ILIM_DAC module size */

`define AD1_ADDR			32'h004000		/* Analog to Digital converter 1 */
`define AD2_ADDR			32'h008000		/* Analog to Digital converter 2 */
`define AD_SIZE			32'h002018		/* Analog to Digital converter size */

`define CAN_ADDR			32'h400000		/* CAN Controller address */
`define CAN_SIZE			32'h006010		/* CAN Controller size */	

`define MEL_ADDR			32'h00b000		/* MEL module address */
`define MEL_SIZE			32'h00004C		/* MEL module size */

module AddDecode(
    input [31:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
	 
    output SP1_RE,
    output SP1_WE,
    output OSC_RE,
    output OSC_WE,
    output LED_RE,
    output LED_WE, 	
	 output DIO_RE,
	 output DIO_WE,
    output AD1_RE,
    output AD1_WE,
    output AD2_RE,
    output AD2_WE,
	 output LDAC_RE,
	 output LDAC_WE,
	 output CAN_RE,
	 output CAN_WE,
	 output RS485_RE,
	 output RS485_WE,
	 output BRG1_RE,
	 output BRG1_WE,
	 output BRG2_RE,
	 output BRG2_WE,
	 output COIL1_RE,
	 output COIL1_WE,
	 output COIL2_RE,
	 output COIL2_WE,
	 output ILIM_DAC_RE,
	 output ILIM_DAC_WE,
	 output MEL_RE,
	 output MEL_WE
    );
	 
	 /*Scratch Pads */
	 assign SP1_RE = (OPB_RE && (OPB_ADDR == `SP1_ADDR));
	 assign SP1_WE = (OPB_WE && (OPB_ADDR == `SP1_ADDR));
	 
	 assign OSC_WE = OPB_WE & (OPB_ADDR >= `OSC_ADDR) & (OPB_ADDR < (`OSC_ADDR + `OSC_SIZE));
	 assign OSC_RE = OPB_RE & (OPB_ADDR >= `OSC_ADDR) & (OPB_ADDR < (`OSC_ADDR + `OSC_SIZE));
		
	 assign LED_WE = OPB_WE & (OPB_ADDR >= `LED_ADDR) & (OPB_ADDR < (`LED_ADDR + `LED_SIZE));
	 assign LED_RE = OPB_RE & (OPB_ADDR >= `LED_ADDR) & (OPB_ADDR < (`LED_ADDR + `LED_SIZE));
		
	 assign DIO_WE = OPB_WE & (OPB_ADDR >= `DIO_ADDR) & (OPB_ADDR < (`DIO_ADDR + `DIO_SIZE));
	 assign DIO_RE = OPB_RE & (OPB_ADDR >= `DIO_ADDR) & (OPB_ADDR < (`DIO_ADDR + `DIO_SIZE));
	 
	 assign AD1_WE = OPB_WE  & (OPB_ADDR >= `AD1_ADDR) & (OPB_ADDR < (`AD1_ADDR + `AD_SIZE));
	 assign AD1_RE = OPB_RE & (OPB_ADDR >= `AD1_ADDR) & (OPB_ADDR < (`AD1_ADDR + `AD_SIZE));	
	 
	 assign AD2_WE = OPB_WE  & (OPB_ADDR >= `AD2_ADDR) & (OPB_ADDR < (`AD2_ADDR + `AD_SIZE));
	 assign AD2_RE = OPB_RE & (OPB_ADDR >= `AD2_ADDR) & (OPB_ADDR < (`AD2_ADDR + `AD_SIZE));
		
	 assign LDAC_WE = OPB_WE & (OPB_ADDR >= `LDAC_ADDR) & (OPB_ADDR < (`LDAC_ADDR + `LDAC_SIZE));
	 assign LDAC_RE = OPB_RE & (OPB_ADDR >= `LDAC_ADDR) & (OPB_ADDR < (`LDAC_ADDR + `LDAC_SIZE));
	
	 assign CAN_WE = OPB_WE & (OPB_ADDR >= `CAN_ADDR) & (OPB_ADDR < (`CAN_ADDR + `CAN_SIZE));
	 assign CAN_RE = OPB_RE & (OPB_ADDR >= `CAN_ADDR) & (OPB_ADDR < (`CAN_ADDR + `CAN_SIZE));
		
	 assign RS485_WE = OPB_WE & (OPB_ADDR >= `RS485_ADDR) & (OPB_ADDR < (`RS485_ADDR + `RS485_SIZE));
	 assign RS485_RE = OPB_RE & (OPB_ADDR >= `RS485_ADDR) & (OPB_ADDR < (`RS485_ADDR + `RS485_SIZE));
		
	 assign BRG1_WE = OPB_WE & (OPB_ADDR >= `BRG1_ADDR) & (OPB_ADDR < (`BRG1_ADDR + `BRG_SIZE));
	 assign BRG1_RE = OPB_RE & (OPB_ADDR >= `BRG1_ADDR) & (OPB_ADDR < (`BRG1_ADDR + `BRG_SIZE));
		
	 assign BRG2_WE = OPB_WE & (OPB_ADDR >= `BRG2_ADDR) & (OPB_ADDR < (`BRG2_ADDR + `BRG_SIZE));
	 assign BRG2_RE = OPB_RE & (OPB_ADDR >= `BRG2_ADDR) & (OPB_ADDR < (`BRG2_ADDR + `BRG_SIZE));
		
	 assign COIL1_WE = OPB_WE & (OPB_ADDR >= `COIL1_ADDR) & (OPB_ADDR < (`COIL1_ADDR + `BRG_SIZE));
	 assign COIL1_RE = OPB_RE & (OPB_ADDR >= `COIL1_ADDR) & (OPB_ADDR < (`COIL1_ADDR + `BRG_SIZE));
		
	 assign COIL2_WE = OPB_WE & (OPB_ADDR >= `COIL2_ADDR) & (OPB_ADDR < (`COIL2_ADDR + `BRG_SIZE));
	 assign COIL2_RE = OPB_RE & (OPB_ADDR >= `COIL2_ADDR) & (OPB_ADDR < (`COIL2_ADDR + `BRG_SIZE));
		
	 assign ILIM_DAC_WE = OPB_WE & (OPB_ADDR >= `ILIM_DAC_ADDR) & (OPB_ADDR < (`ILIM_DAC_ADDR + `ILIM_DAC_SIZE));
	 assign ILIM_DAC_RE = OPB_RE & (OPB_ADDR >= `ILIM_DAC_ADDR) & (OPB_ADDR < (`ILIM_DAC_ADDR + `ILIM_DAC_SIZE));
		
	 assign MEL_WE = OPB_WE & (OPB_ADDR >= `MEL_ADDR) & (OPB_ADDR < (`MEL_ADDR + `MEL_SIZE));
	 assign MEL_RE = OPB_RE & (OPB_ADDR >= `MEL_ADDR) & (OPB_ADDR < (`MEL_ADDR + `MEL_SIZE));
endmodule
