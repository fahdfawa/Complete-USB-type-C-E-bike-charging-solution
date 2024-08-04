

#ifndef BMS_H_
#define BMS_H_

#include <stdint.h>
#include <stdbool.h>
#include <spi.h>



#define  SPI MXC_SPI0
extern uint8_t 	TOTAL_IC;


/* SPI0_SS */

//For BMS
#define MAX_GPIO_P2_26_SPI0_SS1_PORT_IN			MXC_GPIO2
#define MAX_GPIO_P2_26_SPI0_SS1_PIN_IN			MXC_GPIO_PIN_26

//For Charger
#define MAX_GPIO_P1_18_SPI0_SS2_PORT_IN			MXC_GPIO1
#define MAX_GPIO_P1_18_SPI0_SS2_PIN_IN			MXC_GPIO_PIN_18


/***** Initialization of MCU *****/
#define 	SPI_SPEED         	1000000  		// Clock speed for ADBMS6948 (MAX = 4MHz) -------- Practically we can go till 6MHz
#define		Master_Mode			1				//MCU in Master Mode
#define		Slave_Mode			0			    //MCU in Slave Mode (Target in Master Mode)
#define		Quad_Mode			1			    //4-bits per SCK cycle (Quad mode SPI).
#define		Mono_Mode			0			    //1-bit per SCK cycle (Three-wire half-duplex SPI and Four-wire full-duplex SPI)
#define 	SINGLE_SLAVE		1			    //1 Slave  connected to the MCU
#define 	DUAL_SLAVE			2			    //2 Slaves connected to the MCU
#define 	THREE_SLAVE			3			    //3 Slaves connected to the MCU
#define 	FOUR_SLAVE			4			    //4 Slaves connected to the MCU
#define		SS_Polarity			0         		//Slave Select at Active low polarity

/***** BMS IC *****/
/*Number of cells*/
#define N_CELLS 16U
/*Number of cells*/
#define N_CELLS_PER_REGISTERS 3U
/*Number of GPIOs*/
#define N_GPIOS 11U
/*Number of I ADC*/
#define N_IADC 2U
/*Number of bytes for a cell*/
#define BYTES_IN_CELL 2U
/*Number of bytes in data PEC*/
#define PEC_SIZE 2U
/*Number of registers in a group*/
#define REGISTER_CNT_IN_GRP 6U
/*Number of bytes receive for Cell, S, current, Aux, Status Read Commands*/
#define RECEIVE_RD_PACKET_SIZE ((N_CELLS_PER_REGISTERS * BYTES_IN_CELL) + PEC_SIZE)  //  6 + 2
/*Number of bytes receive for All Read Commands*/
#define RECEIVE_RD_ALL_PACKET_SIZE ((N_CELLS * BYTES_IN_CELL) + PEC_SIZE)            //  32 + 2
/*Number of bytes on the command packet 2byte Cmd and 2 byte PEC*/
#define COMMAND_PACKET_SIZE 4U
/*Number of byte receive for RDCVALL command*/
#define RDCVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE                                    //  32 + 2
/*Number of byte receive for RDACVALL command*/
#define RDACVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE                                   //  32 + 2
/*Number of byte receive for RDSVALL command*/
#define RDSVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE                                    //  32 + 2
/*Number of byte receive for RDFCVALL command*/
#define RDFCVALL_SIZE  RECEIVE_RD_ALL_PACKET_SIZE                                   //  32 + 2
/*Number of Bytes for Read All GPIOs*/
#define RDGPALL_SIZE ((N_GPIOS * BYTES_IN_CELL) + PEC_SIZE)                          //  22 + 2
#define RDAUXALL_SIZE ((N_GPIOS * BYTES_IN_CELL) + PEC_SIZE)                         //  22 + 2
/*Number of Bytes for Read All Redundant GPIOs*/
#define RDRGPALL_SIZE ((N_GPIOS * BYTES_IN_CELL) + PEC_SIZE)                         //  22 + 2
/*Number of Bytes for Read all C-ADC and S-ADC command*/
#define RDCSALL_SIZE (((N_CELLS * BYTES_IN_CELL) * 2) + PEC_SIZE)                    //  32*2 + 2
/*Number of Bytes for Read all Avg (C-ADC and S-ADC) command*/
#define RDACSALL_SIZE (((N_CELLS * BYTES_IN_CELL) * 2) + PEC_SIZE)                   //  32*2 + 2
/*Number of Bytes for Read all C-ADC, S-ADC and Current ADC command*/
#define RDCSIVALL_SIZE (((N_CELLS * BYTES_IN_CELL) * 2) + (N_CELLS_PER_REGISTERS * BYTES_IN_CELL) + PEC_SIZE)        //  32*2 + 6 + 2
/*Number of Bytes for Read all Avg (C-ADC, S-ADC and Current ADC) command*/
#define RDACSIVALL_SIZE (((N_CELLS * BYTES_IN_CELL) * 2) + (N_CELLS_PER_REGISTERS * BYTES_IN_CELL) + PEC_SIZE)       //  32*2 + 6 + 2
/*Number of Bytes for Read all C-ADC and Current ADC command*/
#define RDCIVALL_SIZE ((N_CELLS * BYTES_IN_CELL) + (N_CELLS_PER_REGISTERS * BYTES_IN_CELL) + PEC_SIZE)               //  32 + 6 + 2
/*Number of Bytes for Read all Avg (C-ADC and Current ADC) command*/
#define RDACIVALL_SIZE ((N_CELLS * BYTES_IN_CELL) + (N_CELLS_PER_REGISTERS * BYTES_IN_CELL) + PEC_SIZE)              //  32 + 6 + 2
/*Number of Bytes for Read all Avg (S-ADC and Current ADC) command*/
#define RDSVAIALL_SIZE ((N_CELLS * BYTES_IN_CELL) + (N_CELLS_PER_REGISTERS * BYTES_IN_CELL) + PEC_SIZE)              //  32 + 6 + 2
/*Number of Bytes for Read all Status Reg command*/
#define RDSTAALL_SIZE 0 // need to find
/*Number of Bytes for Read all Config Reg command*/
#define RDCCFGALL_SIZE 0 // need to find

///**** Definations *****///
#define Test_Print      0
#define ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x)    ((double)(x) * (double)(0.000150) + (double)(1.5))  //Cell Voltage
#define ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x)           ((double)(x) * (double)(0.000150) + (double)(1.5))  //GPIO(AUX) Voltage
#define ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(x)        ((double)(x) * (double)(0.000001))                  //Current
#define ADBMS_CONVERT_AVG_CURRENT_HEX_TO_VOLT(x)    ((double)(x) * (double)(0.000000125))               //Avg Current
#define ADBMS_CONVERT_Coulumb_Cntr_HEX_TO_VOLT(x)   ((double)(x) * (double)(0.000001))                  //Coulumb Counter
//STATA
#define ADBMS_CONVERT_VREF2_HEX_TO_VOLT(x)          ((double)(x) * (double)(0.000150) + (double)(1.5))  //VREF2 (2.988V to 3.012V)
#define ADBMS_CONVERT_ITEMP_HEX_TO_VOLT(x)          ((double)(x) * (double)(0.000150) + (double)(1.5))/(double)(0.0075) - (double)(273)  //Dia Temp of IC
//STATB
#define ADBMS_CONVERT_VD_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))  //Digital power supply (2.7 to 3.6V)
#define ADBMS_CONVERT_VA_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))  //Analog Power Supply (4.5V to 5.5V)
#define ADBMS_CONVERT_VR4K_HEX_TO_VOLT(x)           ((double)(x) * (double)(0.000150) + (double)(1.5))  //Voltage Accross 4K Resistor (2.9V to 3.1V)
//STATF
#define ADBMS_CONVERT_VM_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))  //Voltage Accross -V and Exposed PAD (~0V)
#define ADBMS_CONVERT_VP_HEX_TO_VOLT(x)             ((double)(x) * (double)(0.000150) + (double)(1.5))*(double)(25)//Total Cell Voltage(12V to 72V) (+v to -V)
#define ADBMS_CONVERT_VREF1_HEX_TO_VOLT(x)          ((double)(x) * (double)(0.000150) + (double)(1.5))  //VREF1

///***** Set ADCV CMD *****/
//ADBMS6948_ADCV(CONT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, &cmd[0]);
#define CONT_ADCV					 	0x1
#define SINGLESHOT_ADCV					0x0
#define RD_OFF_ADCV					  	0x0
#define RD_ON_ADCV					  	0x1
#define NO_RESET_ADCV				  	0x0
#define RESET_ADCV					  	0x0
#define DISCHARGE_ADCV					0x1
#define NODISCHARGE_ADCV				0x0
#define DISCHARGE_ADCV					0x1
#define ALL_CH_OW_OFF_ADCV				0x0
#define EVEN_CH_OW_ADCV					0x1
#define ODD_CH_OW_ADCV					0x2
#define ALL_CH_OW_OFF2_ADCV				0x3

///***** Set ADSV CMD *****/
//ADBMS6948_ADSV(CONT_ADSV, NODISCHARGE_ADSV, ALL_CH_OW_OFF_ADSV, &cmd[0]);
#define CONT_ADSV					    0x1
#define Singleshot_ADSV					0x0
#define DISCHARGE_ADSV					0x1
#define NODISCHARGE_ADSV				0x0
#define ALL_CH_OW_OFF_ADSV				0x0
#define EVEN_CH_OW_ADSV					0x1
#define ODD_CH_OW_ADSV					0x2
#define ALL_CH_OW_OFF2_ADSV				0x3

///***** Set ADAX amd ADAX2 CMD *****/
// void ADBMS6948_ADAX(uint8_t AUX_OW, uint8_t CH_ADAX, uint8_t PUP, uint8_t *cmd);
// void ADBMS6948_ADAX2(uint8_t CH_ADAX2, uint8_t *cmd);
#define OW_AUX_OFF_ADAX       			0x0
#define OW_AUX_ON_ADAX        			0x1
#define PULLDOWN_ADAX         			0x0
#define PULLUP_ADAX           			0x1
/*! \enum   CH_ADAX_t
    \brief  Channel Select
*/
typedef enum
{
  ALL_ADAX = 0x0,	            /*!<All Channel Select*/
  GPIO1TO7_ADAX = 0xE,	        /*!<GPIO 1-7*/
  GPIO8TO10_VP_VM_ADAX = 0xF,	/*!<GPIO 8-10, V+, V-*/
  GPIO1_ADAX = 0x1,	            /*!<Select GPIO 1*/
  GPIO2_ADAX = 0x2,	            /*!<Select GPIO2*/
  GPIO3_ADAX = 0x3,	            /*!<Select GPIO3*/
  GPIO4_ADAX = 0x4,	            /*!<Select GPIO4*/
  GPIO5_ADAX = 0x5,	            /*!<Select GPIO5*/
  GPIO6_ADAX = 0x6,	            /*!<Select GPIO6*/
  GPIO7_ADAX = 0x7,	            /*!<Selet GPIO7*/
  GPIO8_ADAX = 0x8,	            /*!<Select GPIO8*/
  GPIO9_ADAX = 0x9,	            /*!<Select GPIO9*/
  GPIO10_ADAX = 0xA,	        /*!<Select GPIO10*/
  VREF2_ADAX = 0x10,	        /*!<Select Vref2*/
  LDO3V_ADAX = 0x11,	        /*!<Select LDO3V*/
  LDO5V_ADAX = 0x12,	        /*!<Select LDO5V*/
  TEMP_ADAX = 0x13,	            /*!<Select TEMP*/
  VP2M_ADAX = 0x14,	            /*!<Select VP2M*/
  VM_ADAX = 0x15,	            /*!<Select VM*/
  VR4K_ADAX = 0x16,	            /*!<Select VR4K*/
  VREF1_DIV_ADAX = 0x17	        /*!<Select VREF1_DIV*/
}CH_ADAX_t;

/*! \enum   CH_ADAX2_t
    \brief  Channel Select
*/
typedef enum
{
  ALL_ADAX2 = 0x0,	        /*!<All Channel Select*/
  GPIO8TO10_ADAX2 = 0xF,	/*!<GPIO 8-10*/
  GPIO1TO7_ADAX2 = 0xE,	    /*!<GPIO 1-7*/
  GPIO1_ADAX2 = 0x1,	    /*!<Select GPIO 1*/
  GPIO2_ADAX2 = 0x2,	    /*!<Select GPIO2*/
  GPIO3_ADAX2 = 0x3,	    /*!<Select GPIO3*/
  GPIO4_ADAX2 = 0x4,	    /*!<Select GPIO4*/
  GPIO5_ADAX2 = 0x5,	    /*!<Select GPIO5*/
  GPIO6_ADAX2 = 0x6,	    /*!<Select GPIO6*/
  GPIO7_ADAX2 = 0x7,	    /*!<Selet GPIO7*/
  GPIO8_ADAX2 = 0x8,	    /*!<Select GPIO8*/
  GPIO9_ADAX2 = 0x9,	    /*!<Select GPIO9*/
  GPIO10_ADAX2 = 0xA	    /*!<Select GPIO10*/
}CH_ADAX2_t;

///***** Set I1-ADC and I2-ADC2 CMD *****/
//ADBMS6948_ADI1(CONT_ADCV, RD_OFF_ADCV, NO_RESET, NODISCHARGE, ALL_CH_OW_OFF, &cmd[0]);
//ADBMS6948_ADI2(CONT_ADCV, NODISCHARGE, ALL_CH_OW_OFF, &cmd[0]);
/*! \enum   I1_CONT_t
    \brief  Continuous
*/
typedef enum
{
  SINGLE_ADI1 = 0x0,	/*!<Single Measurement,then Standby*/
  CONT_ADI1 = 0x1	/*!<Continuous Measurement*/
}I1_CONT_t;

/*! \enum   I1_DIAGSEL_t
    \brief  Open Wire or Diagnostic Modes Select on I1 Channel
*/
typedef enum
{
  NORMAL_OPERATION_ADI1 = 0x0,	/*!<Normal Operation - Ix Conversion*/
  OPEN_WIRE_ENABLED_POS_SAT_ADI1 = 0x1,	/*!<Open-wire Enabled - OW_ON_001*/
  OPEN_WIRE_ENABLED_NEG_SAT_ADI1 = 0x2,	/*!<Open-wire Enabled - OW_ON_010*/
  OPEN_WIRE_SHIELD_ADI1 = 0x3,	/*!<Open-wire Enabled - OW_ON_011*/
  VOS_ADI1 = 0x4,	/*!<Vos Measurement*/
  VFS1_ADI1 = 0x5,	/*!<Vfs1 Measurement*/
  VFS2_ADI1 = 0x6,	/*!<Vfs2 Measurement*/
  SHIELD_SHORTED_IXA_IXB_ADI1 = 0x7	/*!<Open-wire Enabled - OW_ON_111*/
}I1_DIAGSEL_t;

/*! \enum   I1_DIAG_OW_t
    \brief  Open Wire or Diagnostic Mode Enable on I1 Channel
*/
typedef enum
{
  NORMAL_ADI1 = 0x0,	/*!<Normal Operation - I1 Conversion*/
  DIAGNOSTIC_ADI1 = 0x1	/*!<Diagnostic/Open Wire Operation as per I1_DIAGSEL*/
}I1_DIAG_OW_t;

/*! \enum   I1_RD_t
    \brief  Redundancy
*/
typedef enum
{
  RD_OFF_ADI1 = 0x0,	/*!<Disable Redundancy*/
  RD_ON_ADI1 = 0x1	/*!<Enable Redundancy*/
}I1_RD_t;

/*! \enum   I2_CONT_t
    \brief  Continuous
*/
typedef enum
{
  SINGLE_ADI2 = 0x0,	/*!<Single Measurement,then Standby*/
  CONT_ADI2 = 0x1	/*!<Continuous Measurement*/
}I2_CONT_t;

/*! \enum   I2_DIAGSEL_t
    \brief  Open Wire or Diagnostic Modes Select on I2 Channel
*/
typedef enum
{
  NORMAL_OPERATION_ADI2 = 0x0,	/*!<Normal Operation - Ix Conversion*/
  OPEN_WIRE_ENABLED_POS_SAT_ADI2 = 0x1,	/*!<Open-wire Enabled - OW_ON_001*/
  OPEN_WIRE_ENABLED_NEG_SAT_ADI2 = 0x2,	/*!<Open-wire Enabled - OW_ON_010*/
  OPEN_WIRE_SHIELD_ADI2 = 0x3,	/*!<Open-wire Enabled - OW_ON_011*/
  VOS_ADI2 = 0x4,	/*!<Vos Measurement*/
  VFS1_ADI2 = 0x5,	/*!<Vfs1 Measurement*/
  VFS2_ADI2 = 0x6,	/*!<Vfs2 Measurement*/
  SHIELD_SHORTED_IXA_IXB_ADI2 = 0x7	/*!<Open-wire Enabled - OW_ON_111*/
}I2_DIAGSEL_t;

/*! \enum   I2_DIAG_OW_t
    \brief  Open Wire or Diagnostic Mode Enable on I2 Channel
*/
typedef enum
{
  NORMAL_ADI2 = 0x0,	/*!<Normal Operation - I1 Conversion*/
  DIAGNOSTIC_ADI2 = 0x1	/*!<Diagnostic/Open Wire Operation as per I2_DIAGSEL*/
}I2_DIAG_OW_t;

//typedef enum
//{
//  CFGI2_ACCI_ACC_4 =  E_CFGI2_ACCI_ACC_4,	/*!<Accumulate 4 Current Values*/
//  CFGI2_ACCI_ACC_8 =  E_CFGI2_ACCI_ACC_8,	/*!<Accumulate 8 Current Values*/
//  CFGI2_ACCI_ACC_12 =  E_CFGI2_ACCI_ACC_12,	/*!<Accumulate 12 Current Values*/
//  CFGI2_ACCI_ACC_16 =  E_CFGI2_ACCI_ACC_16,	/*!<Accumulate 16 Current Values*/
//  CFGI2_ACCI_ACC_20 =  E_CFGI2_ACCI_ACC_20,	/*!<Accumulate 20 Current Values*/
//  CFGI2_ACCI_ACC_24 =  E_CFGI2_ACCI_ACC_24,	/*!<Accumulate 24 Current Values*/
//  CFGI2_ACCI_ACC_28 =  E_CFGI2_ACCI_ACC_28,	/*!<Accumulate 28 Current Values*/
//  CFGI2_ACCI_ACC_32 =  E_CFGI2_ACCI_ACC_32	/*!<Accumulate 32 Current Values*/
//}ACCI_bit_t;

/*! \enum   CI_CONT_t
    \brief  Continuous
*/
typedef enum
{
  SINGLE_ADCIV = 0x0,	/*!<Single Measurement,then Standby*/
  CONT_ADCIV = 0x1	/*!<Continuous Measurement*/
}CI_CONT_t;

/*! \enum   CI_OW_t
    \brief  Open Wire on C/I
*/
typedef enum
{
  ALL_CH_OW_OFF_ADCIV = 0x0,	/*!<OPEN Wire Detect OFF on All Channels*/
  EVEN_CH_ADCIV = 0x1,	/*!<OPEN Wire Detect ON for Even Channels OFF for Odd Channels*/
  ODD_CH_ADCIV = 0x2,	/*!<OPEN Wire Detect OFF for Even Channels, ON for Odd Channels*/
  ALL_CH_OW_OFF2_ADCIV = 0x3	/*!<OPEN Wire Detect OFF on All Channels*/
}CI_OW_t;

/*! \enum   CI_RD_t
    \brief  Redundancy
*/
typedef enum
{
  RD_OFF_ADCIV = 0x0,	/*!<Disable Redundancy*/
  RD_ON_ADCIV = 0x1	/*!<Enable Redundancy*/
}CI_RD_t;

/*! \enum   CI_RSTF_t
    \brief  Reset Filter
*/
typedef enum
{
  NO_RESET_ADCIV = 0x0,	/*!<Do Not Reset IIR Filter*/
  RESET_ADCIV = 0x1	/*!<Reset IIR Filter*/
}CI_RSTF_t;

/*! \enum   CI_SSDP_t
    \brief  Single Shot Discharge Permitted
*/
typedef enum
{
  NODISCHARGE_ADCIV = 0x0,	/*!<Discharge Not Permitted During A/D Conversion*/
  DISCHARGEPERMIT_ADCIV = 0x1	/*!<Discharge Permitted During A/D Conversion*/
}CI_SSDP_t;

/*! \enum   CCEN_CONT_t
    \brief  Continuous
*/
typedef enum
{
  SINGLE_CCEN = 0x0,	/*!<Single Measurement,then Standby*/
  CONT_CCEN = 0x1	/*!<Continuous Measurement*/
}CCEN_CONT_t;

/*! \enum   CCEN_EN_t
    \brief  Coulomb Counter Enable
*/
typedef enum
{
  STOP_CCEN = 0x0,	/*!<Stop CC Operation*/
  START_CCEN = 0x1	/*!<Start CC Operation*/
}CCEN_EN_t;

//ADBMS6948_ADCIV(CONT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, &cmd[0]);
//ADBMS6948_CMEN(CONT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, &cmd[0]);
//ADBMS6948_RDSTATC(CONT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, &cmd[0]);

typedef enum
{
RD_CV_ALL,        /*!< Read All Cell Voltage Result Registers*/
RD_AC_ALL,        /*!< Read All Averaged Cell Voltage Result Registers*/
RD_S_ALL,         /*!< Read All S-Voltage Result Registers*/
RD_FC_ALL,        /*!< Read All Filtered Cell Voltage Result Registers*/
RD_AUX_ALL,
RD_C_S_ALL,      /*!< Read all C & S Results*/
RD_AC_S_ALL,      /*!< Read all Avg (C & S) Results*/
RD_CIV_ALL,       /*!< Read All C I Registers Group*/
RD_ACIV_ALL,      /*!< Read All Avg (C & I) Registers Group*/
RD_SVAI_ALL,      /*!< Read All S and Avg I Registers Group*/
RD_CSIVALL,       /*!< Read All C & S I Results Group*/
RD_ACSIVALL,      /*!< Read All Avg (C & S I) Results Group*/
RD_STA_ALL,		/*!< Read All Status Registers Group*/
RD_C_CFG_ALL,		/*!< Read All Configuration Registers Group*/
NOT_ALL,
GRP_NONE,
GRPA,
GRPB,
GRPC,
GRPD,
GRPE,
GRPF,
GRPG,
RD_I_ADC,
}RD_DATA_SIZE_ALL_TYPE;

/*=============P U B L I C P R O T O T Y P E S =============*/

/*============= Function Declaration  =============*/
//main.c
void SPI_Transaction(uint8_t *tx_data, uint8_t *rx_data, uint8_t data_size);
int WakeupIC(mxc_spi_regs_t *spi);
void WakeupBMSIC(void);

//ADBMSMAIN.C
void adbmsmain(void);
void SPI_Initialization();
void ADBMS_Init(uint8_t TOTAL_IC);
void ADBMS_Write_Read_Config(uint8_t TOTAL_IC);
void Read_Battery_Voltage_Current(float *Voltage, float *Current);
unsigned int Detect_Battery(float Voltage);
void ADC_Cell_Voltage_Measurement(uint8_t TOTAL_IC);
void ADC_Current_Measurement(uint8_t TOTAL_IC);
void ADC_Singlehot_Current_Measurement(uint8_t TOTAL_IC);
float ADC_Cont_Current_Measurement(uint8_t TOTAL_IC);
void ADC_S_Voltage_Measurement(uint8_t TOTAL_IC);
void ADC_AVG_Cell_Voltage_Measurement(uint8_t TOTAL_IC);
void ADC_GPIO_Voltage_Measurement(uint8_t TOTAL_IC);
float ADBMS_Status_Reg_voltage_measurment(uint8_t TOTAL_IC);
void ADC_Cont_Sync_V_and_I_Measurement(uint8_t TOTAL_IC);
void ADC_Coulumb_Counter_Measurement(uint8_t TOTAL_IC);
void ADC_Coulumb_Counter_Measurement_debugg(uint8_t TOTAL_IC);
void ADSV_PWM_Discharge_Measurement(uint8_t tIC , uint8_t Timer_value);
void ADBMS_Print_WRCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_RDCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
float ADBMS_Print_Current_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_AUX_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
float ADBMS_Print_Status_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_All_Reg_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMS_Print_Coulumb_Counter_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type);
void ADBMSPollADC(uint8_t tIC, uint8_t cmd_arg[2]);
void pollVoltCTSconversion(void);
void poll8msCtsVconversion(void);
void pollCurntCTSconversion(void);
void GPIO_Init_BMS_Charger();
//void pollAvgCurntCTSconversion(const ACCI_bit_t acci);

//ADBMSCOMMON.c
void SPI_Write(uint8_t tIC, uint8_t cmd_arg[0]);
void ADBMS_Write_Cmd(uint8_t tIC, uint8_t cmd_arg[2]);
void ADBMS_Write_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Write);
void ADBMS_Read_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Read, RD_DATA_SIZE_ALL_TYPE type);
uint16_t Pec15_Calc( uint8_t len, uint8_t *data);
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data);
int16_t twos_complement_to_int(uint16_t value, uint8_t num_bits);
uint16_t SetUnderVoltageThreshold(double voltage);
uint16_t SetOverVoltageThreshold(double voltage);
void WRCFGB_data_Set(uint8_t tIC, uint8_t cmd_arg[], float UV_THSD, float OV_THSD, uint8_t *data_Write);

//ADBMSCMDLIST.h
void ADBMS_ADCV(uint8_t CV_CONT, uint8_t CV_RD, uint8_t CV_RSTF, uint8_t CV_SSDP, uint8_t C_OW, uint8_t *cmd);
void ADBMS_ADSV(uint8_t SV_CONT, uint8_t SV_SSDP, uint8_t S_OW, uint8_t *cmd);
void ADBMS_ADI1(uint8_t I1_CONT, uint8_t I1_DIAGSEL, uint8_t I1_DIAG_OW, uint8_t I1_RD, uint8_t *cmd);
void ADBMS_ADI2(uint8_t I2_CONT, uint8_t I2_DIAGSEL, uint8_t I2_DIAG_OW, uint8_t *cmd);
void ADBMS_ADCIV(uint8_t CI_CONT, uint8_t CI_RD, uint8_t CIV_RSTF, uint8_t CI_SSDP, uint8_t CI_OW, uint8_t *cmd);
void ADBMS_ADAX(uint8_t AUX_OW, CH_ADAX_t CH_ADAX, uint8_t PUP, uint8_t *cmd);
void ADBMS_ADAX2(CH_ADAX2_t CH_ADAX2, uint8_t *cmd);
void ADBMS_CMEN(uint8_t I_MON_EN, uint8_t *cmd);
void ADBMS_RDSTATC(uint8_t ERR, uint8_t *cmd);
void ADBMS_CCEN(uint8_t CCEN_CONT, uint8_t CCEN_EN, uint8_t *cmd);

#endif /* BMS_H_ */
