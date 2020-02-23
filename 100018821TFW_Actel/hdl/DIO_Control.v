`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:40:19 03/16/2009 
// Design Name: 
// Module Name:    DIO_Control 
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
`define AD_ADDR			4'h0
`define PWR_CNT_ADDR		4'h1
`define PWR_STS_ADDR		4'h2
`define ENLP_CNT_ADDR	4'h3
`define ENLP_STS_ADDR	4'h4
`define LP_MON_ADDR		4'h5	
`define TXEN_ADDR			4'h6
`define LIFT_OUT_ADDR	4'h7
`define LIFT_IN_ADDR		4'h8
`define POS_IN_ADDR		4'h9
`define USER_IN_ADDR		4'hA
`define RES_EN_ADDR		4'hB
`define RES_POL_ADDR		4'hC
`define RES_CLK_ADDR		4'hD

module DIO_Control(
	 //OPB Interface
    output [31:0] OPB_DO,
    input [15:0] OPB_DI,
    input [3:0] OPB_ADDR,
    input OPB_RE,
    input OPB_WE,
    input OPB_CLK,
    input OPB_RST,
	 input SYSCLK,
	 
	 //ADC address lines
    output reg [5:0] AD_SEL = 6'h0,
    
	 //Power Supplies
	 output FPGA_VSNSR_ONb, 	//pwr_cnt
    output PWR_ENAB1,			//pwr_cnt
    output PWR_ENAB2,			//pwr_cnt		 
    input LIFT_PWR_ON,				//pwr_sts
    input P24V_PGOOD,					//pwr_sts
    input EM_OVERRIDE_ST,			//pwr_sts		
    input P28V_PGOOD1b,				//pwr_sts
    input P28V_PGOOD2b,				//pwr_sts
    
	 //Loop Monitors
	 output PWRENLP_CNTL,
    output BMENLP_CNTL,
    output MTNENLP_CNTL,
    input USR_MTNENLP_CNTL,
    output SPENLP_CNTL,
    output KVBMENLP_CNTL,
    input PWRENLP_STATE,
    input BMENLP_STATE,
    input MTNENLP_STATE,
    input USR_MTNENLP_STATE,
    input SPENLP_STATE,
    input KVBMENLP_STATE,
    output [1:0] LP_MON_SEL,
    output [2:0] LP_MON_A,
	 output EnLpCnt3,
    
	 //RS485 Direction control
	 output SP485_TXEN1,
    output SP485_TXEN2,
    output [4:1] ENDAT_TX_ENAB,
    
	 //Resolver Excitation
	 output [8:1] RES_PWM,
    
	 //Position Switches
	 output SENSOR_PWR_ENb,			//pwr_cnt
    input SENSOR_PWR_OVLb,
    input [6:1] P_SWb,
    input LOC_SNS_FPGA,
    
	 //User Interface
	 output USR_IF_PWR_EN,			//pwr_cnt
    input PEND_24V_OVLb_FPGA,
    input SP_24V_OVLb_FPGA,
    input L_PEND_MOT_ENABb,
    input R_PEND_MOT_ENABb,
    input L_SP_MOT_ENABb,
    input R_SP_MOT_ENABb,
    input SRVC_PEND_INST,
    input SRVC_PEND_MOT_ENABb,
    
	 //Lift Driver
	 output [4:1] SLIFT_GPO,
    input [4:1] SLIFT_GPIb,
    input SLIFT_PWR_OVLb,
    output SLIFT_PWR_RSTb,
	 output FPGA_SLIFT_MOT_EN_CTRL,
	 
	 //Power Driver
	 output FPGA_DRV_ENAB,			//pwr_cnt
	 input [2:1] PD_UVFLT			//pwr_sts	 
    );
	 
	 reg [5:0] pwr_cnt = 6'h3; 
	 reg [5:0] enlp_cnt = 6'h0;
	 reg [4:0] lp_mon_cnt = 5'h0;
	 reg [5:0] txen = 6'h0;
	 reg [5:0] lift_out = 6'h0;
	 reg [7:0] res_out_en = 8'h0;
	 reg [7:0] res_out_pol = 8'h55;	 
	 reg [15:0] res_clk_div = 16'h7d0;//16'h7d0=16'd2000
	 
	 wire [6:0] pwr_sts; 
	 wire [6:0] enlp_sts;
	 wire [4:0] lift_in;
	 wire [7:0] pos_in;
	 wire [7:0] user_in;
	 wire res_clk;
	 	 
	 assign FPGA_VSNSR_ONb = pwr_cnt[0];
	 assign SENSOR_PWR_ENb = pwr_cnt[1];
	 assign USR_IF_PWR_EN = pwr_cnt[2];
	 assign FPGA_DRV_ENAB = pwr_cnt[3];
	 assign PWR_ENAB1 = pwr_cnt[4];
	 assign PWR_ENAB2 = pwr_cnt[5];
	 
	 assign pwr_sts[0] = LIFT_PWR_ON;
	 assign pwr_sts[1] = P24V_PGOOD;
	 assign pwr_sts[2] = EM_OVERRIDE_ST;
	 assign pwr_sts[3] = P28V_PGOOD1b;
	 assign pwr_sts[4] = P28V_PGOOD2b;
	 assign pwr_sts[6:5] = PD_UVFLT;
	 
	 assign PWRENLP_CNTL = enlp_cnt[0];
	 assign BMENLP_CNTL = enlp_cnt[1];
	 assign MTNENLP_CNTL = enlp_cnt[2];
	 //assign USR_MTNENLP_CNTL = enlp_cnt[3];
	 assign EnLpCnt3 = enlp_cnt[3];
	 assign SPENLP_CNTL = enlp_cnt[4];
	 assign KVBMENLP_CNTL = enlp_cnt[5];
	 
	 assign enlp_sts[0] = PWRENLP_STATE;
	 assign enlp_sts[1] = BMENLP_STATE;
	 assign enlp_sts[2] = MTNENLP_STATE;
	 assign enlp_sts[3] = USR_MTNENLP_STATE;
	 assign enlp_sts[4] = SPENLP_STATE;
	 assign enlp_sts[5] = KVBMENLP_STATE;
	 assign enlp_sts[6] = USR_MTNENLP_CNTL;
	 
	 assign LP_MON_SEL = lp_mon_cnt[4:3];
	 assign LP_MON_A = lp_mon_cnt[2:0]; 
	 
	 assign ENDAT_TX_ENAB = txen[3:0];
	 assign SP485_TXEN1 = txen[4];	 
	 assign SP485_TXEN2 = txen[5];
	 
	 assign SLIFT_GPO = lift_out[3:0];
	 assign SLIFT_PWR_RSTb = lift_out[4];
	 assign FPGA_SLIFT_MOT_EN_CTRL = lift_out[5];
	 
	 assign lift_in[3:0] = SLIFT_GPIb;
	 assign lift_in[4] = SLIFT_PWR_OVLb;
	 
	 assign pos_in[5:0] = P_SWb;
	 assign pos_in[6] = SENSOR_PWR_OVLb;
	 assign pos_in[7] = LOC_SNS_FPGA;
	 
	 assign user_in[0] = R_SP_MOT_ENABb;
	 assign user_in[1] = L_SP_MOT_ENABb;
	 assign user_in[2] = R_PEND_MOT_ENABb;
	 assign user_in[3] = L_PEND_MOT_ENABb;
	 assign user_in[4] = SRVC_PEND_MOT_ENABb;
	 assign user_in[5] = SRVC_PEND_INST;
	 assign user_in[6] = SP_24V_OVLb_FPGA;
	 assign user_in[7] = PEND_24V_OVLb_FPGA;
	 
	 assign RES_PWM[1] = res_out_en[0] & (res_out_pol[0] ^ res_clk);
	 assign RES_PWM[2] = res_out_en[1] & (res_out_pol[1] ^ res_clk);
	 assign RES_PWM[3] = res_out_en[2] & (res_out_pol[2] ^ res_clk);
	 assign RES_PWM[4] = res_out_en[3] & (res_out_pol[3] ^ res_clk);
	 assign RES_PWM[5] = res_out_en[4] & (res_out_pol[4] ^ res_clk);
	 assign RES_PWM[6] = res_out_en[5] & (res_out_pol[5] ^ res_clk);
	 assign RES_PWM[7] = res_out_en[6] & (res_out_pol[6] ^ res_clk);
	 assign RES_PWM[8] = res_out_en[7] & (res_out_pol[7] ^ res_clk);
	 
	 CLOCK_DIV resolver_clk(
		.CLK_DIV(res_clk_div),
		.CLK_IN(SYSCLK),
		.RST(OPB_RST),
		.CLK_OUT(res_clk)
	);
	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `AD_ADDR))   	 ? {26'b0 , AD_SEL} 		 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `PWR_CNT_ADDR))  ? {26'b0 , pwr_cnt} 	 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `PWR_STS_ADDR))  ? {25'b0 , pwr_sts} 	 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENLP_CNT_ADDR)) ? {26'b0 , enlp_cnt} 	 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENLP_STS_ADDR)) ? {25'b0 , enlp_sts} 	 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `LP_MON_ADDR))   ? {27'b0 , lp_mon_cnt}  : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TXEN_ADDR))     ? {26'b0 , txen} 		 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `LIFT_OUT_ADDR)) ? {26'b0 , lift_out} 	 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `LIFT_IN_ADDR))  ? {27'b0 , lift_in} 	 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `POS_IN_ADDR))   ? {24'b0 , pos_in} 		 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `USER_IN_ADDR))  ? {24'b0 , user_in} 	 : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RES_EN_ADDR))   ? {24'b0 , res_out_en}  : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RES_POL_ADDR))  ? {24'b0 , res_out_pol} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RES_CLK_ADDR))  ? {16'b0 , res_clk_div} : 32'bz;

			
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			pwr_cnt <= 6'h3;
			enlp_cnt <= 6'h0;
			lp_mon_cnt <= 5'h0;
			txen <= 6'h0;
			lift_out <= 6'h0;
			res_out_en <= 8'h0;
			res_out_pol <= 8'h55;
			res_clk_div <= 16'h7d0;//16'h7d0 = 16'd2000
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`AD_ADDR: AD_SEL <= OPB_DI[5:0];
				`PWR_CNT_ADDR: pwr_cnt <= OPB_DI[5:0];
				`ENLP_CNT_ADDR: enlp_cnt <= OPB_DI[5:0];
				`LP_MON_ADDR: lp_mon_cnt <= OPB_DI[4:0];
				`TXEN_ADDR: txen <= OPB_DI[5:0];
				`LIFT_OUT_ADDR: lift_out <= OPB_DI[5:0];
				`RES_EN_ADDR: res_out_en <= OPB_DI[7:0];
				`RES_POL_ADDR: res_out_pol <= OPB_DI[7:0];
				`RES_CLK_ADDR:  res_clk_div <= OPB_DI[15:0];
			endcase
		end	
	end


endmodule
