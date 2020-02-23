//////////////////////////////////////////////////////////////////////////////////
// Company:         Jabil
// Engineer:        Wade Jameson
// 
// Create Date:     08:11:26 08/25/2008
// Module Name:     PCI_EMU_TARGET
// Target Devices:  XC4VLX15-10FF668, XC3S1000-FG456, XC2S300E-PQ208
// Tool versions:   Xilinx ISE Webpack 10.1
//
// Description:
//      PCI bus target emulator.
//
// Dependencies:
//      (1) BUFG
//
// Revision History:
// V0.30    08/28/2008  W. Jameson      - Architecture Updates
//      * Changed the OPB_DI from an input port to an output port and changed 
//          OPB_DO from na output port to an input port to match the OPB 
//          architecture.
//
// V0.20    08/26/2008  W. Jameson      - Module Simulation Updates: With Host
//      * Changed the internal module register access to be synchronous to the 
//          PCI_CLK2 input rather than the OPB_CLK output - 9.6ns skew through
//          the BUFG (PCI_CLK2 -> OPB_CLK), which caused read and write errors.
//
// V0.10    08/25/2008  W. Jameson      - Initial Design
//
// V0.01    08/25/2008  W. Jameson      - File Created
//
// Additional Comments:
//      - All unused signals for emulation are configurable as GPIO, and the
//          unused clock inputs are configured as inputs.
//      - PCI_CLK2 is used as the clock input for the communications protocol.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

/* Commands Definitions */
`define TGT_MOD_R           4'h1            /* Target Module Read */
`define TGT_MOD_W           4'h2            /* Target Module Write */
`define TGT_DEV_R           4'h4            /* Target Device Read */
`define TGT_DEV_W           4'h8            /* Target Device Write */

/* Memory Map Definitions */
`define GPDDR               32'h00000000    /* GPIO Data Direction Register */
`define GPDR                32'h00000004    /* GPIO Data Register */
`define CKISR               32'h00000008    /* Clock Input Signal Status Register */
`define SPDR                32'h0000000c    /* Scratch Pad Data Register */

(* opt_mode="speed", optimize_primitives="yes" *)
module PCI_EMU_TARGET(
    /* PCI Bus Communications Signals */
    inout [31:0] PCI_AD,
    input PCI_CLK2,
    input [3:0] PCI_CBE,
    input PCI_FRAME,
    input PCI_DEVSEL,
    input PCI_RST,

    /* PCI GPI Signals */
    input PCI_CLK3,
    input PCI_CLK4,

    /* PCI GPIO Signals */
    inout PCI_TRDY,
    inout PCI_IRDY,
    inout PCI_PAR,
    inout PCI_STOP,
    inout PCI_GNT0,
    inout PCI_REQ0,
    inout PCI_GPERR,
    inout PCI_SERR,
    inout PCI_INTB,

    /* OPB Interface */
    input [31:0] OPB_DO,
    output [31:0] OPB_DI,
    output reg [31:0] OPB_ADDR = 32'b0,
    output OPB_RE,
    output OPB_WE,
    output OPB_CLK,
    output OPB_RST
);

/* PCI Command Register */
    reg [3:0] pci_cmd = 4'hf;

/* Access Control */
    wire mod_re = (pci_cmd == `TGT_MOD_R) & ~PCI_FRAME & ~PCI_DEVSEL & PCI_RST;
    wire mod_we = (pci_cmd == `TGT_MOD_W) & ~PCI_FRAME & ~PCI_DEVSEL & PCI_RST;

    assign OPB_RE = (pci_cmd == `TGT_DEV_R) & ~PCI_FRAME & ~PCI_DEVSEL & PCI_RST;
    assign OPB_WE = (pci_cmd == `TGT_DEV_W) & ~PCI_FRAME & ~PCI_DEVSEL & PCI_RST;

/* Clock Buffer */
 //   BUFG pci_clk_buf(.I(PCI_CLK2), .O(OPB_CLK));
	assign OPB_CLK = PCI_CLK2;

/* Reset Control */
    assign OPB_RST = ~PCI_RST;

/* Scratch Pad Register */
    reg [31:0] sp_dr = 32'b0;

/* GPIO Registers */
    reg [8:0] gpio_ddr = 9'b0;
    reg [8:0] gpio_dr = 9'b0;
    wire [8:0] gpio_rb = {
        PCI_INTB,
        PCI_SERR,
        PCI_GPERR,
        PCI_REQ0,
        PCI_GNT0,
        PCI_STOP,
        PCI_PAR,
        PCI_IRDY,
        PCI_TRDY
    };

    assign PCI_TRDY = (gpio_ddr[0]) ? gpio_dr[0] : 1'bz;
    assign PCI_IRDY = (gpio_ddr[1]) ? gpio_dr[1] : 1'bz;
    assign PCI_PAR = (gpio_ddr[2]) ? gpio_dr[2] : 1'bz;
    assign PCI_STOP = (gpio_ddr[3]) ? gpio_dr[3] : 1'bz;
    assign PCI_GNT0 = (gpio_ddr[4]) ? gpio_dr[4] : 1'bz;
    assign PCI_REQ0 = (gpio_ddr[5]) ? gpio_dr[5] : 1'bz;
    assign PCI_GPERR = (gpio_ddr[6]) ? gpio_dr[6] : 1'bz;
    assign PCI_SERR = (gpio_ddr[7]) ? gpio_dr[7] : 1'bz;
    assign PCI_INTB = (gpio_ddr[8]) ? gpio_dr[8] : 1'bz;

    wire [1:0] pci_ckgpi_rb = {PCI_CLK4, PCI_CLK3};

/* Address and Command Latch */
    reg addr_cmd_le = 1'b0;

    always@(negedge PCI_CLK2 or posedge OPB_RST) begin
        if(OPB_RST) begin
            addr_cmd_le <= 1'b0;
            pci_cmd <= 4'hf;
            OPB_ADDR <= 32'b0;
        end
        else if(!PCI_FRAME) begin
            (* full_case, parallel_case *)
            case(addr_cmd_le)
                1'b0: begin
                    OPB_ADDR <= PCI_AD;
                    pci_cmd <= PCI_CBE;
                    addr_cmd_le <= 1'b1;
                end
                1'b1: begin
                    addr_cmd_le <= 1'b1;
                end
            endcase
        end
        else begin
            addr_cmd_le <= 1'b0;
        end
    end

/* Read/Write Access */
    assign OPB_DI = PCI_AD;
    assign PCI_AD = (OPB_RE) ? OPB_DO : 32'bz;

    assign PCI_AD = (mod_re && (OPB_ADDR == `GPDDR)) ? {23'b0, gpio_ddr} : 32'bz;
    assign PCI_AD = (mod_re && (OPB_ADDR == `GPDR)) ? {23'b0, gpio_rb} : 32'bz;
    assign PCI_AD = (mod_re && (OPB_ADDR == `CKISR)) ? {30'b0, pci_ckgpi_rb} : 32'bz;
    assign PCI_AD = (mod_re && (OPB_ADDR == `SPDR)) ? sp_dr : 32'bz;

    always@(negedge PCI_CLK2 or posedge OPB_RST) begin
        if(OPB_RST) begin
            gpio_ddr <= 9'b0;
            gpio_dr <= 9'b0;
            sp_dr <= 32'b0;
        end
        else if(mod_we) begin
            (* full_case, parallel_case *)
            case(OPB_ADDR)
                `GPDDR: gpio_ddr <= PCI_AD[8:0];
                `GPDR: gpio_dr <= PCI_AD[8:0];
                `SPDR: sp_dr <= PCI_AD;
            endcase
        end
    end

endmodule


