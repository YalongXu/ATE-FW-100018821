`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:13:40 09/15/2008 
// Design Name: 
// Module Name:    BRIDGE_CONT 
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

`define PWM_F_CLK_DIV_ADDR		4'h1 //clock divider is # of sysclk cycles in a 1/3 pulse cycle
`define SAMP_TIME_SET_ADDR		4'h2
`define SAMP_TRIG_ADDR			4'h3
`define FAULT_ADDR				4'h4
`define ENABLE_ADDR				4'h5

`define POL_ADDR					4'h6
`define CYCLE_ADDR				4'h7
`define OVERI_ADDR				4'h8


module BRIDGE_CONT(
    output [31:0] OPB_DO,
    input [31:0] OPB_DI,
    input [3:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,					//32MHz
    input OPB_RST,
    input SYSCLK,					//40MHz
	 
	 output [2:0] PD_PWM,
	 output CUR_SAMP,
	 output PD_ENABLE,
	 input OVER_CURR,
	 input FAULTb  
);

	reg [15:0] clk_divider = 16'ha6a;//(16'd2666)	// frequency control	# of sysclk cycles in a 1/3 pulse cycle
	reg [15:0] cntr_freq = 0;				// frequency counter
	reg StrobeA = 1'b0;							// Leg A strobe
	reg StrobeB = 1'b0;							// Leg B strobe
	reg StrobeC = 1'b0;							// Leg C strobe

	reg [31:0] CycleCount = 32'h4b0;//(32'd1200)		// duty cycle control
	reg [31:0] cntr_dutyA = 32'h0;		// duty cycle counter A
	reg [31:0] cntr_dutyB = 32'h0;		// duty cycle counter B
	reg [31:0] cntr_dutyC = 32'h0;		// duty cycle counter C

	reg polarity = 1'b1;//					// polarity control
	reg PulseA = 1'b0;							
	reg PulseB = 1'b0;
	reg PulseC = 1'b0;		
	
	reg [31:0] sample_time_set = 32'h4b0;//32'd1200
//	reg [31:0] sample_time_count = 17'h0;
	reg sample_trig = 1'b1;
//	reg StrobeSample = 1'b0;//
	reg PulseSample = 1'b0;
	
	reg over_curr_buf = 1'b0;
	reg faultB_buf = 1'b0;//
	reg pdenable = 1'b0;//
		
	wire [15:0] full_clk_div = (clk_divider+clk_divider+clk_divider);
	wire [15:0] two_third_div = (clk_divider+clk_divider);
	
	assign PD_ENABLE = pdenable;
	assign CUR_SAMP = PulseSample;
	
	assign PD_PWM[0] = (polarity) ? PulseA : !PulseA;
	assign PD_PWM[1] = (polarity) ? PulseB : !PulseB;
	assign PD_PWM[2] = (polarity) ? PulseC : !PulseC;

///////////////////////////////////////////////////////////////////////////
// Frequency Strobes 	
	always@(negedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    cntr_freq <= 0;
		else if(cntr_freq >= full_clk_div) begin
			cntr_freq <= 0;
		end
		else begin
			cntr_freq <= cntr_freq + 1;			
		end
	end
	
	always@(posedge SYSCLK) begin
		case(cntr_freq)
			full_clk_div: StrobeA <= 1'b1;
			two_third_div: StrobeB <= 1'b1;
			clk_divider: StrobeC <= 1'b1;
			default: begin
				StrobeA <= 1'b0;
				StrobeB <= 1'b0;
				StrobeC <= 1'b0;
			end
		endcase
	end
/////////////////////////////////////////////////////////////////////////
// Pulse control
	always@(negedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    cntr_dutyA <= 32'h0;
		else if(cntr_dutyA < CycleCount)
				cntr_dutyA <= cntr_dutyA +1;
		else if(StrobeA) begin 
				cntr_dutyA <= 32'h0;
		end
	end
	always@(negedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    cntr_dutyB <= 32'h0;
		else if(cntr_dutyB < CycleCount)
				cntr_dutyB <= cntr_dutyB +1;
		else if(StrobeB) begin 
				cntr_dutyB <= 32'h0;
		end
	end	
	always@(negedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    cntr_dutyC <= 32'h0;
		else if(cntr_dutyC < CycleCount)
			cntr_dutyC <= cntr_dutyC +1;
		else if(StrobeC) begin 
				cntr_dutyC <= 32'h0;
		end
	end		
	
	
	always@(posedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST) begin
	    PulseA <= 1'b0;
			PulseSample <= 1'b0;
	    end
	    
		else if(cntr_dutyA >= CycleCount)begin
			PulseA <= 1'b0;
			PulseSample <= 1'b0;
		end
		else begin
			PulseA <= 1'b1;
			if(sample_trig)
				PulseSample <= 1'b1;
		end
	end
	always@(posedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    PulseB <= 1'b0;
		else if(cntr_dutyB >= CycleCount)
			PulseB <= 1'b0;
		else
			PulseB <= 1'b1;
	end
	always@(posedge SYSCLK or posedge OPB_RST) begin
	  if(OPB_RST)
	    PulseC <= 1'b0;
		else if(cntr_dutyC >= CycleCount)
			PulseC <= 1'b0;
		else
			PulseC <= 1'b1;
	end
//////////////////////////////////////////////////////////////////////////////	
//Sample Control
/*	always@(posedge SYSCLK)begin
		if(PulseSample) begin
			if(sample_time_count > sample_time_set) begin
				PulseSample <= 1'b0;
			end
			else begin
				sample_time_count <= sample_time_count + 1;
			end
		end
		else if(StrobeSample) begin
			PulseSample <= 1'b1;
		end
		else begin
			sample_time_count <= 17'h0;
		end
	end */
////////////////////////////////////////////////////////////////////////////////
//Fault Monitoring 32'h4b0;
	reg [15:0] over_i_set = 16'h258;
	reg over_i_rst = 1'b1;
	reg fault_reset = 1'b0;
	
	wire [15:0] over_i_count;
	
	TIMER_COUNTER #(
		.DATA_WIDTH(16)
	) over_i_counter(
		.ENABLE(1'b1),				// always enabled
		.CLOCK(SYSCLK),
		.RESET(over_i_rst),
		.DATA(over_i_count)
   );
		
	always@(posedge SYSCLK) begin
		if(fault_reset) begin
			over_curr_buf <= 1'b0;
			over_i_rst <= 1'b1;
			faultB_buf <= 1'b1;
		end
		else if(OVER_CURR) begin
			over_i_rst <= 1'b0;
			if(over_i_count > over_i_set)
				over_curr_buf <= 1'b1;
		end 
		else begin
			over_i_rst <= 1'b1;
		end
		if(!FAULTb)
			faultB_buf <= 1'b0;
	end
//////////////////////////////////////////////////////////////////////////////	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `PWM_F_CLK_DIV_ADDR)) ? {16'b0 , clk_divider} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SAMP_TIME_SET_ADDR)) ? {16'b0 , sample_time_set} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SAMP_TRIG_ADDR)) ? {31'b0 , sample_trig} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `FAULT_ADDR)) ? {30'b0 , over_curr_buf, faultB_buf} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENABLE_ADDR)) ? {31'b0 , pdenable} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `POL_ADDR)) ? {31'b0 , polarity} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CYCLE_ADDR)) ? CycleCount : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `OVERI_ADDR)) ? {16'b0 , over_i_set} : 32'bz;

/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			clk_divider <= 16'ha6a;	
			sample_trig <= 1'b0;
			sample_time_set <= 32'h4b0;
			pdenable <= 1'b0;
			polarity <= 1'b1;
			CycleCount <= 32'h4b0;
			over_i_set <= 16'h258;
			fault_reset <= 1'b0;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`PWM_F_CLK_DIV_ADDR: clk_divider <= OPB_DI[15:0];
				`SAMP_TIME_SET_ADDR: sample_time_set <= OPB_DI;
				`SAMP_TRIG_ADDR: sample_trig <= OPB_DI[0];
				`ENABLE_ADDR: pdenable <= OPB_DI[0];
				`POL_ADDR: polarity <= OPB_DI[0];
				`CYCLE_ADDR: CycleCount <= OPB_DI;
				`OVERI_ADDR: over_i_set <= OPB_DI[15:0];
				`FAULT_ADDR: fault_reset <= OPB_DI[0];
			endcase
		end
	end
endmodule
