`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
// 
// Create Date:    14:48:15 06/10/2008 
// Design Name: 
// Module Name:    5101090_ADC 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description:
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: test
//
// Revision 2.01 - Bug fix
// Additional Comments: 
// Bug Description: 
// ADC convert result is NOT correct or -1; test would probably pass after ambient temperature goes up
// Resolution:
// 1. to fix one potential hardware bug: BUSY probably would NOT go from 0 to 1 after AD_CNVST is set as '0' from '1',
//    then shall try it several times (max repeat_times = 6).
//
//////////////////////////////////////////////////////////////////////////////////

/***** Memory Map *****/

`define D_RAM_ADDR					 12'h000		/* Start Address of RAM */
`define D_RAM_SIZE					 12'h800		/* size of RAM */
`define CNTRL_ADDR                12'h800    /* Control Register (R/W) */
`define CLK_DIV_AQ_ADDR           12'h801    /* Aquasition clock divier Register (R/W) */
`define CLK_DIV_SD_ADDR			    12'h802		/* Serial Data Clock divier Register (R/W) */
`define D_LENGTH_ADDR		       12'h803		/* Address of data length register */
`define STATUS_ADDR				    12'h804		/* Address of Status Register */
`define STATE_ADDR				    12'h805		/* Address of State Register */

`define S_START					5'h0
`define S_CNVST					5'h1
`define S_BUSY						5'h2	
`define S_READ0					5'h3	
`define S_READ1					5'h4	
`define S_READ2					5'h5	
`define S_READ3					5'h6	
`define S_READ4					5'h7	
`define S_READ5					5'h8	
`define S_READ6					5'h9	
`define S_READ7					5'ha	
`define S_READ8					5'hb	
`define S_READ9					5'hc	
`define S_READ10					5'hd	
`define S_READ11					5'he	
`define S_READ12					5'hf	
`define S_READ13					5'h10	
`define S_READ14					5'h11	
`define S_READ15					5'h12	
`define S_READ16					5'h13	
`define S_READ17					5'h14	
`define S_READ18					5'h15
`define S_WAIT						5'h16	
`define S_END						5'h17

module ADC_AD7663AS(
	output [31:0] OPB_DO,
	input [15:0] OPB_DI,
	input [11:0] OPB_ADDR,
	input OPB_RE,
	input OPB_WE,
	input OPB_CLK,				   //32MHz
	input OPB_RST,
	input SYSCLK,					//40MHz
	
	output reg AD_CNVST,
	output AD_SCLK,
	input AD_SDOUT,
	input AD_BUSY

);
	 
	 //Registers
	 reg [1:0] control = 2'h0;				// control[0]: start sample, control[1]: reset module, 
	 reg [3:0] status = 4'b00;				// status[0]: busy, status[1]: done, status[2]: aq timeout, status[3]: busy timout
	 reg [11:0] data_length = 12'h5;		// number of samples to collect
	 reg [15:0] clk_div_aq = 16'd200;	// aquasition clock defualt 40E6/(2*200) = 100KHz
	 reg [15:0] clk_div_sd = 16'd2;		// serial data clock defualt 40E6/(2*2)  = 10MHz
	 reg [11:0] ram_wr_pt = 12'h000;		// Ram Write Pointer
	 reg ram_wr_clk = 1'b0;					// ram write clock
	 reg [4:0] state = 5'h0;				// state of state machine
	 reg [15:0] sdout_buf = 16'h0;		// buffer for the AD_SDOUT signal
	 reg [11:0] data_count = 12'h0;		// count samples as they are saved to ram
	 reg [1:0] clk_aq_low = 2'h0;			// used to look for clk_aq edges
	 reg [1:0] clk_aq_high = 2'h0;		// used to look for clk_aq edges
	 reg ad_sclk_en = 1'b0;					// enable the sclk
	 reg [6:0] time_out_count = 7'h0;	// time out counter
     reg [4:0]repeat_times;
    
	 //signals
	 wire clk_aq;
	 wire clk_sd;	
	 wire [15:0] ram_opb_do;
	 
	 wire ram_re = OPB_RE &
							(OPB_ADDR >= `D_RAM_ADDR) & 
							(OPB_ADDR < (`D_RAM_ADDR + `D_RAM_SIZE));
							
	assign AD_SCLK = (clk_sd && ad_sclk_en);
//	assign CLK_SD_OUT = clk_sd;
//	assign CLK_AQ_OUT = clk_aq;
	
	CLOCK_DIV aqclkdiv(
		.CLK_DIV(clk_div_aq),
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(clk_aq)
	);
	
	CLOCK_DIV sclk(
		.CLK_DIV(clk_div_sd),
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(clk_sd)
	);
	
	
	
	DP_RAM_2R_1W #(
		.ADDR_WIDTH(12),				// 1K samples of data
		.DATA_WIDTH(16)				// 16 bit samples
	) data_in_ram(					   //data read from ADC
		.PA_DI(sdout_buf),			// data in buffer
		.PA_DO(),
		.PA_ADDR(ram_wr_pt),		   // pointer to next memory location
		.PA_WE(1'b1),					// ram is always write enabled 
		.PA_CLK(ram_wr_clk),		   // state machine controls clock signal
		.PB_DO(ram_opb_do),
		.PB_ADDR(OPB_ADDR[11:0]),
		.PB_CLK(OPB_CLK)
	);
	
		
	always@(negedge clk_sd or posedge OPB_RST) begin
	    if(OPB_RST)begin
	        AD_CNVST        <= 1'b1;
	        status          <= 4'b0;
            time_out_count  <= 7'b0;
            sdout_buf       <= 16'b0;
            ram_wr_pt       <= 11'b0;
		    ram_wr_clk      <= 1'b0;
            data_count      <= 12'b0;
            clk_aq_low      <= 2'h0;
            clk_aq_high     <= 2'h0;
            ad_sclk_en      <= 1'b0;
            state           <= `S_START;
            repeat_times    <=  5'd0;
	    end
	      
	    else if(control[1]) begin				//reset registers
			status <= 4'h0;
			ram_wr_pt <= 12'h000;
			ram_wr_clk <= 1'b0;
			state <= `S_START;
			sdout_buf <= 16'h0;
			data_count <= 12'h0;
			clk_aq_low <= 2'h0;
			clk_aq_high <= 2'h0;
            repeat_times    <=  5'd0;
		end
		else if(control[0] || status[0]) begin
			case(state)
				`S_START: begin
                    AD_CNVST        <= 1'b1;
                    time_out_count  <= 0;
					if((clk_aq_high == 2'h3) && (clk_aq_low == 2'h3))begin	// start action on rising edge of clk_aq
						state <= `S_CNVST;
						status[0] <= 1'b1;												// status busy
						status[1] <= 1'b0;												// status not done						
					end
					else if((clk_aq == 1'b0) && (clk_aq_low != 2'h3)) begin
						clk_aq_low <= clk_aq_low + 2'h1;
					end
					else if((clk_aq == 1'b1) && (clk_aq_low == 2'h3)) begin
						clk_aq_high <= clk_aq_high + 2'h1;
					end
				end
				`S_CNVST: begin
					AD_CNVST <= 1'b0;													   // set conversion start
					clk_aq_low <= 2'h0;													// clear aq_clk edge detection
					clk_aq_high <= 2'h0;
					if(AD_BUSY) begin														// wait for AD_BUSY to go high
						time_out_count <= 7'h0;						
						state <= `S_BUSY;
					end
					else if(time_out_count >= 7'h10) begin
						state <= `S_START;
						repeat_times <= repeat_times + 5'd1;					
					end
					//else if(time_out_count >= 7'h64) begin
                    else if(repeat_times >= 6) begin
						status[3] <= 1'b1;
						AD_CNVST <= 1'b1;													// clear conversion start
						state <= `S_END;
					end
					else begin
						time_out_count <= time_out_count +1;
					end
				end
				`S_BUSY: begin
					if(!AD_BUSY) begin
						state <= `S_READ0;										// wait for AD_BUSY to go low
						AD_CNVST <= 1'b1;
					end
					else if(time_out_count >= 7'h1a) begin
						status[2] <= 1'b1;
						state <= `S_END;
						AD_CNVST <= 1'b1;
					end
					else begin
						time_out_count <= time_out_count +1;
					end				
				end
				`S_READ0: begin 
					ad_sclk_en <= 1'b1;
					state <= state + 1;
				end
				`S_READ1: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//0
					state <= state + 1;
				end
				`S_READ2: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//1
					state <= state + 1;
				end
				`S_READ3: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//2
					state <= state + 1;
				end
				`S_READ4: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//3
					state <= state + 1;
				end
				`S_READ5: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//4
					state <= state + 1;
				end
				`S_READ6: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//5
					state <= state + 1;
				end
				`S_READ7: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//6
					state <= state + 1;
				end
				`S_READ8: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//7
					state <= state + 1;
				end
				`S_READ9: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//8
					state <= state + 1;
				end
				`S_READ10: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//9
					state <= state + 1;
				end
				`S_READ11: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//10
					state <= state + 1;
				end
				`S_READ12: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//11
					state <= state + 1;
				end
				`S_READ13: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//12
					state <= state + 1;
				end
				`S_READ14: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//13
					state <= state + 1;
				end
				`S_READ15: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//14
					state <= state + 1;
				end
				`S_READ16: begin 
					sdout_buf <= {sdout_buf[14:0],AD_SDOUT};//15
					ad_sclk_en <= 1'b0;
					state <= state + 1;
				end
				`S_READ17: begin
					data_count <= data_count + 1;
					ram_wr_clk <= 1'b1;
					state <= state + 1;
				end
				`S_READ18: begin
					ram_wr_clk <= 1'b0;
					ram_wr_pt <= ram_wr_pt + 1;
					if(data_count >= data_length)
						state <= `S_END;
					else
						state <= `S_WAIT;
				end
				`S_WAIT: begin																// wait for next rising edge of clk_aq
					if((clk_aq_high == 2'h3) && (clk_aq_low == 2'h3))begin	// start action on rising edge of clk_aq
						state <= `S_CNVST;
					end
					else if((clk_aq == 1'b0) && (clk_aq_low != 2'h3)) begin
						clk_aq_low <= clk_aq_low + 2'h1;
					end
					else if((clk_aq == 1'b1) && (clk_aq_low == 2'h3)) begin
						clk_aq_high <= clk_aq_high + 2'h1;
					end
				end
				`S_END: begin
					if(status[0]) begin													//reset state machine
						status[0] <= 1'b0;
						status[1] <= 1'b1;
						ram_wr_clk <= 1'b0;
						ram_wr_pt <= 12'h000;
						data_count <= 12'h0;
                        repeat_times <=  5'd0;						
					end					
				end
			endcase
		end
		else if((control == 2'h0) && (state != `S_START)) begin
			state <= `S_START;
		end
	end

///////////// OPB
/* Read Access */
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CNTRL_ADDR)) ? {30'b0 , control} : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_DIV_AQ_ADDR)) ?  {16'b0 , clk_div_aq} : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_DIV_SD_ADDR)) ?  {16'b0 , clk_div_sd} : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `D_LENGTH_ADDR)) ? {20'b0 , data_length} : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATE_ADDR)) ? {27'b0 , state} : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `STATUS_ADDR)) ? {30'b0 , status} : 32'bz;
	
	assign OPB_DO = (ram_re) ? {16'b0, ram_opb_do} : 32'bz;

/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
		  control         <= 2'b0;
          clk_div_aq      <= 16'd200;
          clk_div_sd      <= 16'd2;
          data_length     <= 12'h5;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`CNTRL_ADDR: control <= OPB_DI[1:0];
				`CLK_DIV_AQ_ADDR: clk_div_aq <= OPB_DI[15:0];
				`CLK_DIV_SD_ADDR: clk_div_sd <= OPB_DI[15:0];
				`D_LENGTH_ADDR: data_length <= OPB_DI[11:0];
			endcase
		end
		else if(state == `S_END) begin
			control <= 2'h0;
		end	
	end
					
endmodule
