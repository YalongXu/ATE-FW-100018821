`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Jabil	
// Engineer: Philip Schmidt
// 
// Create Date:   11:20:49 01/27/2009 
// Design Name: 	100018821 Test Firmware (TFW)
// Module Name:	top_100018821TFW 
// Project Name:	100018821-TFW 
// Target Devices: XC3S1000-FG456
// Tool versions: Xilinx Webpack 10.1
// Description: 
//						This is the top level design file of the couch controller test
//						firmware.
// Dependencies: 
//						ADC_AD7663AS.V			*Interface with the two ADCs
//						AddDecode.V				*Control the module read/write enables
//						CLK_DIV.V				*A simple clock divider
//						DAC_AD8803AR.V    	*Interface with the current limit DAC
//						dual_port_ram.V		*Simple Dual Port Ram
//						Oscillator_Counter.V	*Module to measre the frequency of SYSCLK
//						pci_emu_target.V		*pci bus target
//						Timer_Counter.v
//						
//
// Revision 0.02 - Full test Version 
// Additional Comments: First revision of complete firmware.
//
// Revision 0.01 - File Created
// Additional Comments: Fist version, limited capabilites for use with temporary
//								Test.
//////////////////////////////////////////////////////////////////////////////////
module top_100018821TFW(
    /* PCI Bus Communications Signals */
	 inout [31:0] PCI_AD,
    input PCI_CLK2,
    input [3:0] PCI_CBE,
    input PCI_FRAME,
    input PCI_DEVSEL,
    input PCI_RST,
    
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
	 
	 /* SYSCLK */
    input SYSCLK,
	 		
	 /* Digital Outputs */	 
	 output [5:0] AD_SEL,
	 
	 /* ADC 1 */
	 output AD1_CNVSTb,
	 output AD1_SCLK,
	 input AD_SDOUT1,
	 input AD_BUSY1,
	 
	 /* ADC 1 */
	 output AD2_CNVSTb,
	 output AD2_SCLK,
	 input AD_SDOUT2,
	 input AD_BUSY2,

	 /* LEDs */
    output HEARTBEAT_LED,
    output CAN_RX,
    output CAN_TX,
    output P_SP_CAN_RX,
    output P_SP_CAN_TX,
    output STAT_LED1,
    output STAT_LED2,
    output STAT_LED3,
    output SPARE_LED1,
    output SPARE_LED2,
    output SPARE_LED3,
	 output SPARE_LED4,
    output SPARE_LED5,
    output SPARE_LED6,
    output SRVC_MODE_LED,
	 
	 //Power Supply
    output FPGA_VSNSR_ONb,
    output PWR_ENAB1,
    output PWR_ENAB2,
	 input LIFT_PWR_ON,
	 input P24V_PGOOD,
	 input EM_OVERRIDE_ST,
	 input P28V_PGOOD1b,
	 input P28V_PGOOD2b,

	 //Network Interface: Enable Loops
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

	 //Network Interface: RS485
	 output Sp485_TX1,
	 output Sp485_TX2,
	 output Sp485_TXEN1,
	 output Sp485_TXEN2,
	 input Sp485_RX1,
	 input Sp485_RX2,
	 input SYNC,
	 input BMPLS,

	 //Network Interface: CAN
	 inout [7:0] CAN_D,
	 output CAN_CCLK_FPGA,
	 output CAN_ALE,
	 output CAN_RDb,
	 output CAN_WRb,
	 output CAN_RSTb,
	 output CAN_CS1b,
	 input CAN_INT1b,
	 output CAN_BUF_DIR,
	 output CAN_CS2b,
	 input CAN_INT2b,

	 //Postiion Sensor: Resolvers
	 output [8:1] RES_PWM,

	 //Position Sensor: ENDAT
	 output [4:1] ENDAT_TX_ENAB,
	 output [4:1] ENDAT_CLK,
	 output [4:1] ENDAT_TX,
	 input [4:1] ENDAT_RX,

	 //Position Sensor: Supplies
    output SENSOR_PWR_ENb,
    input SENSOR_PWR_OVLb,

	 //Position Sensor: Switches
    input [6:1] P_SWb,
    input LOC_SNS_FPGA,

	 //User IF
    output USR_IF_PWR_EN,
    input PEND_24V_OVLb_FPGA,
    input SP_24V_OVLb_FPGA,
    input L_PEND_MOT_ENABb_FPGA,
    input R_PEND_MOT_ENABb_FPGA,
    input L_SP_MOT_ENABb_FPGA,
    input R_SP_MOT_ENABb_FPGA,
    input SRVC_PEND_INST_FPGA,
    input SRVC_PEND_MOT_ENABb_FPGA,

	 //External Lift Driver: Smart
    output [4:1] SLIFT_GPO,
    output SLIFT_DAC_LDACb,
    output SLIFT_DAC_SCLK,
    output SLIFT_DAC_SDIN,
    output SLIFT_DAC_RSTINb,
    output SLIFT_DAC_SYNC,
    output SLIFT_DAC_CLRb,
    input [4:1] SLIFT_GPIb,
	 input SLIFT_PWR_OVLb,
	 output SLIFT_PWR_RSTb,
	 output FPGA_SLIFT_MOT_EN_CTRL,

	 //External Lift Driver: Power
	output PLIFT_PHASEA,
	output PLIFT_PHASEB,
	output PLIFT_PHASEC,	
	input PLIFT_FAULT1,
	input PLIFT_FAULT2,
	output PLIFT_ISAMPLE,
	output PLIFT_ENABLE,

	 //Test IF
	output TST_SPI_CLK,
	output TST_SPI_MOSI,
	output TST_SPI_CSb,
	input TST_SPI_MISO,
	
	 //POWER DRIVER (PD_)
	 output [3:1] PD_PWM1,
	 output [3:1] PD_PWM2,
	 output [2:1] PD_PWM3,
	 output [2:1] PD_PWM4,
	 output [4:1] PD_ENABLE,
	 output FPGA_DRV_ENAB,
	 output [2:1] PD_CUR_SL,
	 input [4:1] PD_FAULTb,
	 input [2:1] PD_UVFLT,
	 input [4:1] OVER_CURR,
	 
	 //Current Limit DAC (ILIM_)
	 output ILIM_DAC_CS,
	 output ILIM_DAC_SDI,
	 output ILIM_DAC_CLK,

	 //CPLD
	 inout [9:0] FPGA2CPLD,
	 output FPGA2CPLD_CLK,
	 input MEL_INT,
	 input MEL_ENABLE,
	 input MEL_ERROR,
	 output MEL_ACK,
	 output MEL_ERROR_RESET,
	 input [3:1] MEL_XTRA,

	 //Misc
	 output FPGA_DONE,
	 input RESET_IN,

	 //Test Points
	 output TP55,
	 //output TP56,
	 output TP23,
	 output TP24,
	 output TP66,
	 output TP67,
	 output TP68,
	 output TP69,
	 output TP57,
	 output TP59,
	 output TP60,
	 output TP61,
	 output TP62,
	 output TP63,
	 output TP64,
	 output TP65,
	 output TP25,

     //DEBUG

     output TP2
    );
	 
/***************************************/
// Declarations and Assignments 		   //	 
/***************************************/

	 /* OPB Interface */
	 wire [31:0] opb_di;
	 wire [31:0] opb_do;
	 wire [31:0] opb_addr;
	 wire opb_re;
	 wire opb_we;
	 wire opb_clk;
	 wire opb_rst;

	 /* Read and Write Enables */
	 wire sp_re;
	 wire sp_we;
	 wire osc_re;
	 wire osc_we;
	 wire LedCont_re;
	 wire LedCont_we;
	 wire dio_re;
	 wire dio_we;
	 wire ad1_re;
	 wire ad1_we;
	 wire ad2_re;
	 wire ad2_we;
	 wire ldac_re;
	 wire ldac_we;
	 wire can_re;
	 wire can_we;
	 wire rs485_re;
	 wire rs485_we;	 
	 wire brg1_re;
	 wire brg1_we;
	 wire brg2_re;
	 wire brg2_we;
	 wire coil1_re;
	 wire coil1_we;
	 wire coil2_re;
	 wire coil2_we;
	 wire ilim_dac_re;
	 wire ilim_dac_we;
	 wire mel_re;
	 wire mel_we;	 


     //DEBUG

     assign TP2 = SRVC_PEND_MOT_ENABb_FPGA;

     //DEBUG END
	 
	 /* Scratch Pad */
	 reg [31:0] dev_sp = 32'hf5f5f5f5;			//scratch pad register1
	 	 
	 /* Test Points */
	// assign TP55 = dev_sp[0]; //coil1
	 //assign TP56 = SYSCLK;//dev_sp[1];
	 assign TP23 = ENDAT_CLK[1];//dev_sp[2];
	 assign TP24 = MEL_XTRA[2];//dev_sp[3];
	// assign TP66 = dev_sp[4];
	// assign TP67 = dev_sp[5];
	 //assign TP68 = dev_sp[6];
	 assign TP69 = dev_sp[7];
	 assign TP57 = dev_sp[8];
	 //assign TP59 = dev_sp[9];
	 assign TP60 = dev_sp[10];
	 assign TP61 = dev_sp[11];
	 assign TP62 = dev_sp[12];
	 assign TP63 = dev_sp[13];
	 assign TP64 = dev_sp[14];
	 assign TP65 = dev_sp[15];
	 assign TP25 = dev_sp[16];

   assign FPGA_DONE = ~RESET_IN;

   
	 
/***************************************/
// Modules									   //	 
/***************************************/
	 PCI_EMU_TARGET pci_target(
			/* PCI Bus Communications Signals */
			.PCI_AD(PCI_AD),
			.PCI_CLK2(PCI_CLK2),
			.PCI_CBE(PCI_CBE),
			.PCI_FRAME(PCI_FRAME),
			.PCI_DEVSEL(PCI_DEVSEL),
			.PCI_RST(PCI_RST),

			/* PCI GPI Signals */
			.PCI_CLK3(1'b0),
			.PCI_CLK4(1'b0),

			/* PCI GPIO Signals */
			.PCI_TRDY(PCI_TRDY),
			.PCI_IRDY(PCI_IRDY),
			.PCI_PAR(PCI_PAR),
			.PCI_STOP(PCI_STOP),
			.PCI_GNT0(PCI_GNT0),
			.PCI_REQ0(PCI_REQ0),
			.PCI_GPERR(PCI_GPERR),
			.PCI_SERR(PCI_SERR),
			.PCI_INTB(PCI_INTB),

			/* OPB Interface */
			.OPB_DI(opb_di),
			.OPB_DO(opb_do),
			.OPB_ADDR(opb_addr),
			.OPB_RE(opb_re),
			.OPB_WE(opb_we),
			.OPB_CLK(opb_clk),
			.OPB_RST(opb_rst)
	);
	
	AddDecode add_dec(
			.OPB_ADDR(opb_addr),
			.OPB_RE(opb_re),
			.OPB_WE(opb_we),
			
			.SP1_RE(sp_re),
			.SP1_WE(sp_we),
			.OSC_RE(osc_re),
			.OSC_WE(osc_we),
			.LED_RE(LedCont_re),
			.LED_WE(LedCont_we),
			.DIO_RE(dio_re),
			.DIO_WE(dio_we),
			.AD1_RE(ad1_re),
			.AD1_WE(ad1_we),
			.AD2_RE(ad2_re),
			.AD2_WE(ad2_we),
			.LDAC_RE(ldac_re),
			.LDAC_WE(ldac_we),
			.CAN_RE(can_re),
			.CAN_WE(can_we),
			.RS485_RE(rs485_re),
			.RS485_WE(rs485_we),
			.BRG1_RE(brg1_re),
			.BRG1_WE(brg1_we),
			.BRG2_RE(brg2_re),
			.BRG2_WE(brg2_we),
			.COIL1_RE(coil1_re),
			.COIL1_WE(coil1_we),
			.COIL2_RE(coil2_re),
			.COIL2_WE(coil2_we),
			.ILIM_DAC_RE(ilim_dac_re),
			.ILIM_DAC_WE(ilim_dac_we),
			.MEL_RE(mel_re),
			.MEL_WE(mel_we)
   );
		 
	OSCILLATOR_COUNTER osc_count(
		 .OPB_DO(opb_do),
		 .OPB_DI(opb_di),
		 .OPB_ADDR(opb_addr[3:2]),
		 .OPB_RE(osc_re),
		 .OPB_WE(osc_we),
		 .OPB_CLK(opb_clk),
		 .OPB_RST(opb_rst),
		 .SYSCLK(SYSCLK)
	);
	
	LED_Control LedCont(
			.OPB_DO(opb_do),
			.OPB_DI(opb_di),
			.OPB_ADDR(opb_addr[4:2]),
			.OPB_RE(LedCont_re),
			.OPB_WE(LedCont_we),
			.OPB_CLK(opb_clk),
			.OPB_RST(opb_rst),
			.SYSCLK(SYSCLK),
	 
			.HEARTBEAT_LED(HEARTBEAT_LED),	
			.CAN_RX(CAN_RX),	
			.CAN_TX(CAN_TX),	
			.P_SP_CAN_RX(P_SP_CAN_RX),	
			.P_SP_CAN_TX(P_SP_CAN_TX),	
			.STAT_LED1(STAT_LED1),	
			.STAT_LED2(STAT_LED2),	
			.STAT_LED3(STAT_LED3),	
			.SPARE_LED1(SPARE_LED1),	
			.SPARE_LED2(SPARE_LED2),	
			.SPARE_LED3(SPARE_LED3),	
			.SPARE_LED4(SPARE_LED4),	
			.SPARE_LED5(SPARE_LED5),	
			.SPARE_LED6(SPARE_LED6),	
			.SRVC_MODE_LED(SRVC_MODE_LED),
			.TEN_HZ(TP59)
    );	 
	
	DIO_Control DioCont(
		.OPB_DO(opb_do),
		.OPB_DI(opb_di),
		.OPB_ADDR(opb_addr[5:2]),
		.OPB_RE(dio_re),
		.OPB_WE(dio_we),
		.OPB_CLK(opb_clk),
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK),
		
		//Power Supplies
		.AD_SEL(AD_SEL),
		.FPGA_VSNSR_ONb(FPGA_VSNSR_ONb),
		.PWR_ENAB1(PWR_ENAB1),
		.PWR_ENAB2(PWR_ENAB2),
		.LIFT_PWR_ON(LIFT_PWR_ON),
		.P24V_PGOOD(P24V_PGOOD),
		.EM_OVERRIDE_ST(EM_OVERRIDE_ST),
		.P28V_PGOOD1b(P28V_PGOOD1b),
		.P28V_PGOOD2b(P28V_PGOOD2b),
		
		//Network If
		.PWRENLP_CNTL(PWRENLP_CNTL),
		.BMENLP_CNTL(BMENLP_CNTL),
		.MTNENLP_CNTL(MTNENLP_CNTL),
		.USR_MTNENLP_CNTL(USR_MTNENLP_CNTL),
		.SPENLP_CNTL(SPENLP_CNTL),
		.KVBMENLP_CNTL(KVBMENLP_CNTL),
		.PWRENLP_STATE(PWRENLP_STATE),
		.BMENLP_STATE(BMENLP_STATE),
		.MTNENLP_STATE(MTNENLP_STATE),
		.USR_MTNENLP_STATE(USR_MTNENLP_STATE),
		.SPENLP_STATE(SPENLP_STATE),
		.KVBMENLP_STATE(KVBMENLP_STATE),
		.LP_MON_SEL(LP_MON_SEL),
		.LP_MON_A(LP_MON_A),
		.SP485_TXEN1(Sp485_TXEN1),
		.SP485_TXEN2(Sp485_TXEN2),
		.ENDAT_TX_ENAB(ENDAT_TX_ENAB),
		
		//Resolver Excitation
		.RES_PWM(RES_PWM),
		
		//Position Sensor: Supplies
		.SENSOR_PWR_ENb(SENSOR_PWR_ENb),
		.SENSOR_PWR_OVLb(SENSOR_PWR_OVLb),
		
		//Position Sensor: Switches
		.P_SWb(P_SWb),
		.LOC_SNS_FPGA(LOC_SNS_FPGA),
		
		//User IF
		.USR_IF_PWR_EN(USR_IF_PWR_EN),
		.PEND_24V_OVLb_FPGA(PEND_24V_OVLb_FPGA),
		.SP_24V_OVLb_FPGA(SP_24V_OVLb_FPGA),
		.L_PEND_MOT_ENABb(L_PEND_MOT_ENABb_FPGA),
		.R_PEND_MOT_ENABb(R_PEND_MOT_ENABb_FPGA),
		.L_SP_MOT_ENABb(L_SP_MOT_ENABb_FPGA),
		.R_SP_MOT_ENABb(R_SP_MOT_ENABb_FPGA),
		.SRVC_PEND_INST(SRVC_PEND_INST_FPGA),
		.SRVC_PEND_MOT_ENABb(SRVC_PEND_MOT_ENABb_FPGA),
		
		.SLIFT_GPO(SLIFT_GPO),
		.SLIFT_GPIb(SLIFT_GPIb),
		.SLIFT_PWR_OVLb(SLIFT_PWR_OVLb),
		.SLIFT_PWR_RSTb(SLIFT_PWR_RSTb),
		.FPGA_SLIFT_MOT_EN_CTRL(FPGA_SLIFT_MOT_EN_CTRL),
		.FPGA_DRV_ENAB(FPGA_DRV_ENAB),
		.PD_UVFLT(PD_UVFLT)
	);
	 
	ADC_AD7663AS ad1_mod(
			.OPB_DO(opb_do),
			.OPB_DI(opb_di[15:0]),
			.OPB_ADDR(opb_addr[13:2]),
			.OPB_RE(ad1_re),
			.OPB_WE(ad1_we),
			.OPB_CLK(opb_clk),
			.OPB_RST(opb_rst),
			.SYSCLK(SYSCLK),

			.AD_CNVST(AD1_CNVSTb),
			.AD_SCLK(AD1_SCLK),
			.AD_SDOUT(AD_SDOUT1),
			.AD_BUSY(AD_BUSY1)
	);
	
	ADC_AD7663AS ad2_mod(
			.OPB_DO(opb_do),
			.OPB_DI(opb_di[15:0]),
			.OPB_ADDR(opb_addr[13:2]),
			.OPB_RE(ad2_re),
			.OPB_WE(ad2_we),
			.OPB_CLK(opb_clk),
			.OPB_RST(opb_rst),
			.SYSCLK(SYSCLK),

			.AD_CNVST(AD2_CNVSTb),
			.AD_SCLK(AD2_SCLK),
			.AD_SDOUT(AD_SDOUT2),
			.AD_BUSY(AD_BUSY2)
	);
	
	DAC_AD7849BR LiftDac(
		.OPB_DO(opb_do),
		.OPB_DI(opb_di),
		.OPB_ADDR(opb_addr[4:2]),
		.OPB_RE(ldac_re),
		.OPB_WE(ldac_we),
		.OPB_CLK(opb_clk),
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK),
		
		.LDACb(SLIFT_DAC_LDACb),
		.SCLK(SLIFT_DAC_SCLK),
		.SDIN(SLIFT_DAC_SDIN),
		.RSTb(SLIFT_DAC_RSTINb),
		.SYNC(SLIFT_DAC_SYNC),
		.CLRb(SLIFT_DAC_CLRb)
	);
	
	CAN_SJA1000 can_control(
		 .OPB_DO(opb_do),
		 .OPB_DI(opb_di[15:0]),
		 .OPB_ADDR(opb_addr[17:2]),
		 .OPB_RE(can_re),
		 .OPB_WE(can_we),
		 .OPB_CLK(opb_clk),
		 .OPB_RST(opb_rst),
		 
		 .SYSCLK(SYSCLK),
		 
		 .CAN_CCLK(CAN_CCLK_FPGA),			
		 .CAN_RST(CAN_RSTb),
		 .CAN_INT1(CAN_INT1b),
		 .CAN_INT2(CAN_INT2b),
		 .CAN_AD(CAN_D),
		 .CAN_ALE(CAN_ALE),
		 .CAN_RD(CAN_RDb),
		 .CAN_WR(CAN_WRb),
		 .CAN_CS1(CAN_CS1b),
		 .CAN_CS2(CAN_CS2b),
		 .CAN_BUF_DIR1(CAN_BUF_DIR)
    );
		
	RS485 rs485_mod(
		.OPB_DO(opb_do),
		.OPB_DI(opb_di),
		.OPB_ADDR(opb_addr[7:2]),
		.OPB_RE(rs485_re),
		.OPB_WE(rs485_we),
		.OPB_CLK(opb_clk),
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK),

		.SYNC(SYNC),
		.BMPLS(BMPLS),
		.TST_SPI_MISO(TST_SPI_MISO),
		.SP485_RX1(Sp485_RX1),
		.SP485_RX2(Sp485_RX2),
		.ENDAT_RX(ENDAT_RX),
		.PLIFT_FAULT1(PLIFT_FAULT1),
		.PLIFT_FAULT2(PLIFT_FAULT2),	 

		.TST_SPI_CLK(TST_SPI_CLK),
		.TST_SPI_MOSI(TST_SPI_MOSI),
		.TST_SPI_CS(TST_SPI_CSb),
		.Sp485_TX1(Sp485_TX1),
		.Sp485_TX2(Sp485_TX2),
		.ENDAT_CLK(ENDAT_CLK),
		.ENDAT_TX(ENDAT_TX),
		.PLIFT_PHASEA(PLIFT_PHASEA),
		.PLIFT_PHASEB(PLIFT_PHASEB),
		.PLIFT_PHASEC(PLIFT_PHASEC),
		.PLIFT_ISAMPLE(PLIFT_ISAMPLE),
		.PLIFT_ENABLE(PLIFT_ENABLE)		
	);
	
	BRIDGE_CONT brg1(
		.OPB_DO(opb_do),
 		.OPB_DI(opb_di),
		.OPB_ADDR(opb_addr[5:2]),
		.OPB_RE(brg1_re),
		.OPB_WE(brg1_we),
		.OPB_CLK(opb_clk),				//32MHz
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK),					//80MHz
	 
		.PD_PWM(PD_PWM1),
		.CUR_SAMP(PD_CUR_SL[1]),
		.PD_ENABLE(PD_ENABLE[1]),
		.OVER_CURR(OVER_CURR[1]),
		.FAULTb(PD_FAULTb[1])  
	);
	
	BRIDGE_CONT brg2(
		.OPB_DO(opb_do),
 		.OPB_DI(opb_di),
		.OPB_ADDR(opb_addr[5:2]),
		.OPB_RE(brg2_re),
		.OPB_WE(brg2_we),
		.OPB_CLK(opb_clk),				//32MHz
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK),					//80MHz
	 
		.PD_PWM(PD_PWM2),
		.CUR_SAMP(PD_CUR_SL[2]),
		.PD_ENABLE(PD_ENABLE[2]),
		.OVER_CURR(OVER_CURR[2]),
		.FAULTb(PD_FAULTb[2])  
	);
	
	BRIDGE_CONT coil1(
		.OPB_DO(opb_do),
 		.OPB_DI(opb_di),
		.OPB_ADDR(opb_addr[5:2]),
		.OPB_RE(coil1_re),
		.OPB_WE(coil1_we),
		.OPB_CLK(opb_clk),				//32MHz
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK),					//80MHz
	 
		.PD_PWM({TP55,PD_PWM3}),
		.CUR_SAMP(TP66),
		.PD_ENABLE(PD_ENABLE[3]),
		.OVER_CURR(OVER_CURR[3]),
		.FAULTb(PD_FAULTb[3])  
	);
	
	BRIDGE_CONT coil2(
		.OPB_DO(opb_do),
 		.OPB_DI(opb_di),
		.OPB_ADDR(opb_addr[5:2]),
		.OPB_RE(coil2_re),
		.OPB_WE(coil2_we),
		.OPB_CLK(opb_clk),				//32MHz
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK),					//80MHz
	 
		.PD_PWM({TP67,PD_PWM4}),
		.CUR_SAMP(TP68),
		.PD_ENABLE(PD_ENABLE[4]),
		.OVER_CURR(OVER_CURR[4]),
		.FAULTb(PD_FAULTb[4])  
	);
	
	DAC_AD8803AR ilim_dac_mod(
		.ILIM_DAC_CLK(ILIM_DAC_CLK),
		.ILIM_DAC_SDI(ILIM_DAC_SDI),
		.ILIM_DAC_CS(ILIM_DAC_CS),	 
		.OPB_DO(opb_do),
		.OPB_DI(opb_di[15:0]),
		.OPB_ADDR(opb_addr[4:2]),
		.OPB_RE(ilim_dac_re),
		.OPB_WE(ilim_dac_we),
		.OPB_CLK(opb_clk),	
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK)
    );
	 
	 MEL mel_mod(
		.MEL_INT(MEL_INT),
		.MEL_ENABLE(MEL_ENABLE),
		.MEL_ERROR(MEL_ERROR),
		.MEL_ACK(MEL_ACK),
		.MEL_ERROR_RESET(MEL_ERROR_RESET),
		.MEL_XTRA(MEL_XTRA),
		.FPGA2CPLD(FPGA2CPLD),
		.FPGA2CPLD_CLK(FPGA2CPLD_CLK),		
		.OPB_DO(opb_do),
		.OPB_DI(opb_di[15:0]),
		.OPB_ADDR(opb_addr[6:2]),
		.OPB_RE(mel_re),
		.OPB_WE(mel_we),
		.OPB_CLK(opb_clk),	
		.OPB_RST(opb_rst),
		.SYSCLK(SYSCLK)

    );
	
/***************************************/
// Top Level Read / Write Access		   //	 
/***************************************/
	/* Top Level Read Access */
	assign opb_do = sp_re ? dev_sp : 32'bz;
	
	/* Top Level Write Access */
	always@(negedge opb_clk) begin
		if(opb_rst) begin
			dev_sp <= 32'hf5f5f5f5;
		end
		else if(sp_we) begin
			dev_sp <= opb_di;
		end
	end
endmodule
