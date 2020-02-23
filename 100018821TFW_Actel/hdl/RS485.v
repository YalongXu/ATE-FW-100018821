`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:35:07 09/11/2008 
// Design Name: 
// Module Name:    RS485 
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
`define RS_CNTRL_ADDR			6'h0		// control register address
`define D_CLK_ADDR				6'h1		// Data clock divider address
`define TST_PAT					6'h2

`define SYNC_DATA					6'h6
`define BMPLS_DATA				6'ha
`define TST_SPI_MISO_DATA		6'he
`define SP485_RX1_DATA			6'h12
`define SP485_RX2_DATA			6'h16
`define PLIFT_FAULT1_DATA		6'h1a
`define PLIFT_FAULT2_DATA		6'h1e
`define ENDAT_RX1_DATA			6'h22
`define ENDAT_RX2_DATA			6'h26
`define ENDAT_RX3_DATA			6'h2a
`define ENDAT_RX4_DATA			6'h2e
//`define CLK_SELECT				6'h33

module RS485(
    output [31:0] OPB_DO,
    input [31:0] OPB_DI,
    input [5:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,				//32MHz
    input OPB_RST,
    input SYSCLK,					//40MHz

	 input SYNC,
	 input BMPLS,
	 input TST_SPI_MISO,
	 input SP485_RX1,
	 input SP485_RX2,
	 input PLIFT_FAULT1,
	 input PLIFT_FAULT2,
	 input [4:1] ENDAT_RX,
	 
	 output TST_SPI_CLK,
	 output TST_SPI_MOSI,
	 output TST_SPI_CS,
	 output Sp485_TX1,
	 output Sp485_TX2,
	 output [4:1] ENDAT_CLK,
	 output [4:1] ENDAT_TX,
	 output PLIFT_PHASEA,
	 output PLIFT_PHASEB,
	 output PLIFT_PHASEC,
	 output PLIFT_ISAMPLE,
	 output PLIFT_ENABLE
);

	reg control = 1'b0;										// set high to trigger transfer
	reg done = 1'b0;
	reg [15:0] clk_div = 16'd20;  						// sysclk / 2*clk_div = data rate
	reg [7:0] bit_count = 8'h0;							
	reg [127:0] test_pattern 	= 128'h00010204081020408055AA3366CCEEFF;		// pattern to transfer
	reg [127:0] test_pattern_buf 	= 128'h00010204081020408055AA3366CCEEFF;				// output buffer

	reg [127:0] sync_d 				= 128'h0;
	reg [127:0] bmpls_d 				= 128'h0;
	reg [127:0] tst_spi_miso_d 	= 128'h0;
	reg [127:0] sp485_rx1_d 		= 128'h0;
	reg [127:0] sp485_rx2_d 		= 128'h0;
	reg [127:0] plift_fault1_d 	= 128'h0;
	reg [127:0] plift_fault2_d 	= 128'h0;
	
	reg [127:0] endat_rx1_d 	= 128'h0;
	reg [127:0] endat_rx2_d 	= 128'h0;
	reg [127:0] endat_rx3_d 	= 128'h0;
	reg [127:0] endat_rx4_d 	= 128'h0;
	
	reg clock_en = 1'b0;
	reg freerun = 1'b0;

	wire data_clk;
	
	assign TST_SPI_CLK = (clock_en & data_clk);
	assign TST_SPI_MOSI = test_pattern_buf[119];
	assign TST_SPI_CS = test_pattern_buf[111];
	assign Sp485_TX1 = test_pattern_buf[103];
	assign Sp485_TX2 = test_pattern_buf[95];
	assign PLIFT_PHASEA = test_pattern_buf[87];
	assign PLIFT_PHASEB = test_pattern_buf[79];
	assign PLIFT_PHASEC = test_pattern_buf[71];
	assign PLIFT_ISAMPLE = test_pattern_buf[63];
	assign PLIFT_ENABLE = test_pattern_buf[55];
	
	assign ENDAT_TX[1] = test_pattern_buf[47];
	assign ENDAT_TX[2] = test_pattern_buf[39];
	assign ENDAT_TX[3] = test_pattern_buf[31];
	assign ENDAT_TX[4] = test_pattern_buf[23];
	
	assign ENDAT_CLK[1] = (clock_en & data_clk);
	assign ENDAT_CLK[2] = test_pattern_buf[7];
	assign ENDAT_CLK[3] = test_pattern_buf[127];
	assign ENDAT_CLK[4] = test_pattern_buf[119];
	
	
	CLOCK_DIV pci_clk_div(
		.CLK_DIV(clk_div),
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(data_clk)
	);
	
	always@(posedge data_clk or posedge OPB_RST) begin
	    if(OPB_RST)begin
	        bit_count           <= 8'b0;
    	    sync_d 				<= 128'h0;
    	    bmpls_d 			<= 128'h0;
    	    tst_spi_miso_d 	    <= 128'h0;
    	    sp485_rx1_d 		<= 128'h0;
    	    sp485_rx2_d 		<= 128'h0;
    	    plift_fault1_d 	    <= 128'h0;
    	    plift_fault2_d 	    <= 128'h0;
    	    endat_rx1_d 	    <= 128'h0;
    	    endat_rx2_d 	    <= 128'h0;
       	    endat_rx3_d 	    <= 128'h0;
    	    endat_rx4_d 	    <= 128'h0;
	    end
		else if(clock_en) begin
				bit_count <= bit_count + 1;
				sync_d <= {sync_d[126:0], SYNC};
				bmpls_d <= {bmpls_d[126:0], BMPLS};				
				tst_spi_miso_d <= {tst_spi_miso_d[126:0], TST_SPI_MISO};
				sp485_rx1_d <= {sp485_rx1_d[126:0], SP485_RX1};
				sp485_rx2_d <= {sp485_rx2_d[126:0], SP485_RX2};
				plift_fault1_d <= {plift_fault1_d[126:0], PLIFT_FAULT1};
				plift_fault2_d <= {plift_fault2_d[126:0], PLIFT_FAULT2};
				endat_rx1_d <= {endat_rx1_d[126:0], ENDAT_RX[1]};
				endat_rx2_d <= {endat_rx2_d[126:0], ENDAT_RX[2]};
				endat_rx3_d <= {endat_rx3_d[126:0], ENDAT_RX[3]};
				endat_rx4_d <= {endat_rx4_d[126:0], ENDAT_RX[4]};				
		end
		else begin
			bit_count <= 8'h0;
		end
	end
	
	always@(negedge data_clk) begin
		if(!control) begin
			done <= 1'b0;
			test_pattern_buf <= test_pattern;
			clock_en <= 1'b0;
		end
		else if(bit_count > 0) begin
			test_pattern_buf <= {test_pattern_buf[126:0],test_pattern_buf[127]};
			if((bit_count > 127) & ~freerun) begin
				done <= 1'b1;
				clock_en <= 1'b0;
			end
		end
		else if(!done) begin
			clock_en <= 1'b1;
		end
	end

/* Read Access */

    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RS_CNTRL_ADDR)) 	? {29'h0,freerun, done, control} : 32'bz;

    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TST_PAT)) 			? test_pattern[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`TST_PAT + 1))) 	? test_pattern[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`TST_PAT + 2))) 	? test_pattern[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`TST_PAT + 3))) 	? test_pattern[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SYNC_DATA)) 			? sync_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SYNC_DATA + 1))) 	? sync_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SYNC_DATA + 2))) 	? sync_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SYNC_DATA + 3))) 	? sync_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `BMPLS_DATA)) 			? bmpls_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`BMPLS_DATA + 1))) 	? bmpls_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`BMPLS_DATA + 2))) 	? bmpls_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`BMPLS_DATA + 3))) 	? bmpls_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TST_SPI_MISO_DATA)) 			? tst_spi_miso_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`TST_SPI_MISO_DATA + 1))) 	? tst_spi_miso_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`TST_SPI_MISO_DATA + 2))) 	? tst_spi_miso_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`TST_SPI_MISO_DATA + 3))) 	? tst_spi_miso_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SP485_RX1_DATA)) 			? sp485_rx1_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SP485_RX1_DATA + 1))) 	? sp485_rx1_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SP485_RX1_DATA + 2))) 	? sp485_rx1_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SP485_RX1_DATA + 3))) 	? sp485_rx1_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SP485_RX2_DATA)) 			? sp485_rx2_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SP485_RX2_DATA + 1))) 	? sp485_rx2_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SP485_RX2_DATA + 2))) 	? sp485_rx2_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`SP485_RX2_DATA + 3))) 	? sp485_rx2_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `PLIFT_FAULT1_DATA)) 			? plift_fault1_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`PLIFT_FAULT1_DATA + 1))) 	? plift_fault1_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`PLIFT_FAULT1_DATA + 2))) 	? plift_fault1_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`PLIFT_FAULT1_DATA + 3))) 	? plift_fault1_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `PLIFT_FAULT2_DATA)) 			? plift_fault2_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`PLIFT_FAULT2_DATA + 1))) 	? plift_fault2_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`PLIFT_FAULT2_DATA + 2))) 	? plift_fault2_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`PLIFT_FAULT2_DATA + 3))) 	? plift_fault2_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENDAT_RX1_DATA)) 			? endat_rx1_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX1_DATA + 1))) 	? endat_rx1_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX1_DATA + 2))) 	? endat_rx1_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX1_DATA + 3))) 	? endat_rx1_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENDAT_RX2_DATA)) 			? endat_rx2_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX2_DATA + 1))) 	? endat_rx2_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX2_DATA + 2))) 	? endat_rx2_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX2_DATA + 3))) 	? endat_rx2_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENDAT_RX3_DATA)) 			? endat_rx3_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX3_DATA + 1))) 	? endat_rx3_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX3_DATA + 2))) 	? endat_rx3_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX3_DATA + 3))) 	? endat_rx3_d[127:96] : 32'bz;
	 
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENDAT_RX4_DATA)) 			? endat_rx4_d[31:0] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX4_DATA + 1))) 	? endat_rx4_d[63:32] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX4_DATA + 2))) 	? endat_rx4_d[95:64] : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == (`ENDAT_RX4_DATA + 3))) 	? endat_rx4_d[127:96] : 32'bz;
			
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			control <= 1'b0;
			clk_div <= 16'd20;
			freerun <= 1'b0;  
			test_pattern <= 40'ha987654321;
		//	clock_en <= 1'b0;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`RS_CNTRL_ADDR:begin
					control <= OPB_DI[0];
					freerun <= OPB_DI[2];
				end
				`D_CLK_ADDR: clk_div <= OPB_DI[15:0];
				`TST_PAT: test_pattern[31:0] <= OPB_DI;
				`TST_PAT+1: test_pattern[63:32] <= OPB_DI;
				`TST_PAT+2: test_pattern[95:64] <= OPB_DI;
				`TST_PAT+3: test_pattern[127:96] <= OPB_DI;
				//`CLK_SELECT: clock_select <= OPB_DI[0];
			endcase
		end	
	end

endmodule
