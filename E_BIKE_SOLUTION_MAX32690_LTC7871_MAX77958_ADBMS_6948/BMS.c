
/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "BMScmdlist.h"
#include "BMS.h"
#include "mxc_delay.h"



mxc_gpio_cfg_t l_s_spi_ss1_config;

mxc_gpio_cfg_t 			  l_s_r_LED_config;
mxc_gpio_cfg_t 			  l_s_g_LED_config;
mxc_gpio_cfg_t 			  l_s_fet_en_config;
mxc_gpio_cfg_t 			  l_s_usr_btn_config;
mxc_gpio_cfg_t 			  l_s_spi_ss1_config;
mxc_gpio_cfg_t 			  l_s_spi_ss2_config;
mxc_gpio_cfg_t 			  l_s_pchg_fet_config;

/***** Initialization of BMS IC *****/

float I1 = 0.0;
float Pack_Voltage = 0.0;

/* Set Under Voltage and Over Voltage Thresholds */
double OV_THRESHOLD = 4.2;                 /* Volt */
double UV_THRESHOLD = 3.7;                 /* Volt */

//Refon = 1 and All GPIO are OFF
uint8_t CFGA_data[6] = {0x81, 0x00, 0x00, 0xff, 0x03, 0x00};// 0x02, 0x8E
uint8_t CFGB_data[6];
uint8_t CFGC_data[6] = {0x00, 0x05, 0x00, 0x00, 0x00, 0x00};
uint8_t PWMA_data[6] = {0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD}; //set 48% PWM for cell
uint8_t PWMB_data[6] = {0xDD, 0xDD, 0x00, 0x00, 0x00, 0x00}; //set 48% PWM for cell

uint8_t 	TOTAL_IC	=	  1;


void SPI_Initialization()
{
	//Make every ss pins false for taking out the control
	mxc_spi_pins_t spi_pins;
	spi_pins.ss0 = FALSE;
	spi_pins.ss1 = FALSE;
	spi_pins.ss2 = FALSE;
	MXC_SPI_Init(SPI, Master_Mode, Mono_Mode, DUAL_SLAVE, SS_Polarity, SPI_SPEED, spi_pins);
	MXC_SPI_SetDataSize(SPI, 8);
	MXC_SPI_SetMode(SPI, SPI_MODE_0);
	MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);
}



/*!
  @brief Initialized  BMS IC
*/
void ADBMS_Init(uint8_t TOTAL_IC)
{
  int Reg_Size =RECEIVE_RD_PACKET_SIZE;
	uint8_t *Data_Read;
	Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC();
	ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
	ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], GRPA); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  WRCFGB_data_Set(TOTAL_IC, WRCFGB, UV_THRESHOLD, OV_THRESHOLD, &CFGB_data[0]);
	ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGB_data[0]); // Set UV and OV Threshold for all Cells
  ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPB);
  free(Data_Read);
}

/*!
  @brief Set Refon = 1, UV = 3V and OV = 4.2V in CFG Ref
*/
void ADBMS_Write_Read_Config(uint8_t TOTAL_IC)
{
  int Reg_Size =RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
  WakeupBMSIC();
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], GRPA); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  printf("\nWRCFGA\n");
  ADBMS_Print_WRCFG_Data(TOTAL_IC,  WRCFGA, CFGA_data, GRPA);
  printf("\nRDCFGA\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDCFGA, Data_Read, GRPA);
  WRCFGB_data_Set(TOTAL_IC, WRCFGB, UV_THRESHOLD, OV_THRESHOLD, &CFGB_data[0]);
  ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGB_data[0]); // Set UV and OV Threshold for all Cells
  ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPB);
  printf("\nWRCFGB\n");
  ADBMS_Print_WRCFG_Data(TOTAL_IC,  WRCFGB, CFGB_data, GRPB);
  printf("\nRDCFGB\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDCFGB, Data_Read, GRPB);
  free(Data_Read);
}

/*!
  @brief Measure C-ADC Voltage in Single shot mode
*/

void Read_Battery_Voltage_Current(float *Voltage, float *Current)
{
	*Current = ADC_Cont_Current_Measurement(TOTAL_IC);

	*Voltage = ADBMS_Status_Reg_voltage_measurment(TOTAL_IC);

}

unsigned int Detect_Battery(float Voltage)
{
    if (Voltage < 40.0)
    {
        return 0;
    }
    else if (Voltage >= 40.0 && Voltage <= 58.8)
    {
        return 1;
    }
    else
    {
        printf("Unexpected Battery Voltage\n");
        return 0; // or handle the unexpected voltage accordingly
    }
}



void ADC_Cell_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
  WakeupBMSIC(); //Wakeup BMS IC
  ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
  WakeupBMSIC(); ////Wakeup BMS IC after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADCV(SINGLESHOT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, ADCV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCV); //Send ADCV command with Single shot, No Redundancy, no discharge
  ADBMSPollADC(TOTAL_IC, PLCADC);  //Send poll ADC command to Check ADC Conversion
  //MXC_Delay(1100);               //We can use delay as C-ADC conversion completed after 1msec
  WakeupBMSIC();
  ADBMS_Read_Data(TOTAL_IC,   RDCVA, &Data_Read[0], GRPA); // Read Cell Voltages
  ADBMS_Print_Data(TOTAL_IC,  RDCVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDCVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDCVB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDCVC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDCVC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDCVD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDCVD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDCVE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDCVE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDCVF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDCVF, Data_Read, GRPF);
  printf("\nCLR Command\n");
//  ADBMS_Write_Cmd(TOTAL_IC, CLRCELL);                       //Clear cell Voltages
//  ADBMS_Read_Data(TOTAL_IC,   RDCVA, &Data_Read[0], GRPA);  //read default value after clearcell command
//  ADBMS_Print_Data(TOTAL_IC,  RDCVA, Data_Read, GRPA);
//  ADBMS_Read_Data(TOTAL_IC,   RDCVB, &Data_Read[0], GRPB);
//  ADBMS_Print_Data(TOTAL_IC,  RDCVB, Data_Read, GRPB);
  free(Data_Read);
}

/*!
  @brief Measure Current in I-ADC in Single shot mode
*/
void ADC_Singlehot_Current_Measurement(uint8_t TOTAL_IC)
{
	int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
	uint8_t *Data_Read;
	Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC
	ADBMS_Write_Cmd(TOTAL_IC, SRST); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC after Reset to bring ADBMS from SLEEP to STANDBY
	ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
	ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
	ADBMS_ADI1(SINGLE_ADI1, NORMAL_OPERATION_ADI1, NORMAL_ADI1, RD_ON_ADI1, ADI1);
	ADBMS_Write_Cmd(TOTAL_IC, ADI1);
  ADBMSPollADC(TOTAL_IC, PLCIADC);
	MXC_Delay(8800);
  ADBMS_Read_Data(TOTAL_IC, RDI, &Data_Read[0], RD_I_ADC);
  ADBMS_Print_Current_Data(TOTAL_IC, RDI, Data_Read, RD_I_ADC);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRCI);  //CLR I1ADC
  ADBMS_Write_Cmd(TOTAL_IC, CLRSI);  //CLR I2ADC
  ADBMS_Read_Data(TOTAL_IC, RDI, &Data_Read[0], RD_I_ADC);
  ADBMS_Print_Current_Data(TOTAL_IC, RDI, Data_Read, RD_I_ADC);
  free(Data_Read);
}

/*!
  @brief Measure Current in I-ADC in Continuous mode
*/
float ADC_Cont_Current_Measurement(uint8_t TOTAL_IC)
{
	int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
	uint8_t *Data_Read;
	Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC
	ADBMS_Write_Cmd(TOTAL_IC, SRST); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC after Reset to bring ADBMS from SLEEP to STANDBY
	ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
	ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
	ADBMS_ADI1(CONT_ADI1, NORMAL_OPERATION_ADI1, NORMAL_ADI1, RD_ON_ADI1, ADI1);
	ADBMS_Write_Cmd(TOTAL_IC, ADI1);
	pollCurntCTSconversion();
  //MXC_Delay(8800);
  ADBMS_Read_Data(TOTAL_IC, RDI, &Data_Read[0], RD_I_ADC);
  float Load_Curr = ADBMS_Print_Current_Data(TOTAL_IC, RDI, Data_Read, RD_I_ADC);
  ADBMS_ADI1(SINGLE_ADI1, NORMAL_OPERATION_ADI1, NORMAL_ADI1, RD_ON_ADI1, ADI1);
	ADBMS_Write_Cmd(TOTAL_IC, ADI1);
//  printf("\nCLR Command\n");
//  ADBMS_Write_Cmd(TOTAL_IC, CLRCI);  //CLR I1ADC
//  ADBMS_Write_Cmd(TOTAL_IC, CLRSI);  //CLR I2ADC
  free(Data_Read);
  return Load_Curr;

}

float ADBMS_Status_Reg_voltage_measurment(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
	uint8_t *Data_Read;
	Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
	ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
	//ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGA_data[0]); // Set UV and OV Threshold for all Cells
	ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
	ADBMS_ADAX(OW_AUX_OFF_ADAX, ALL_ADAX, PULLDOWN_ADAX, ADAX);
	ADBMS_Write_Cmd(TOTAL_IC, ADAX);
	ADBMSPollADC(TOTAL_IC, PLAUX1);
  //MXC_Delay(20000); // 20msec delay to get data of all GPIO Regs
  WakeupBMSIC();
//  ADBMS_Read_Data(TOTAL_IC, RDSTATA, &Data_Read[0], GRPA);
//  ADBMS_Print_Status_Data(TOTAL_IC, RDSTATA, Data_Read, GRPA);
//  ADBMS_Read_Data(TOTAL_IC, RDSTATB, &Data_Read[0], GRPB);
//  ADBMS_Print_Status_Data(TOTAL_IC, RDSTATB, Data_Read, GRPB);
//  ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
//  ADBMS_Read_Data(TOTAL_IC, RDSTATD, &Data_Read[0], GRPD);
//  ADBMS_Read_Data(TOTAL_IC, RDSTATE, &Data_Read[0], GRPE);
  ADBMS_Read_Data(TOTAL_IC, RDSTATF, &Data_Read[0], GRPF);
  float Pack_Volt = ADBMS_Print_Status_Data(TOTAL_IC, RDSTATF, Data_Read, GRPF);
//  ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
//  printf("\nCLR Command\n");
//  ADBMS_Write_Cmd(TOTAL_IC, CLRFLAG);
  free(Data_Read);
  return Pack_Volt;
}

/*!
  @brief Measure S-ADC Voltage in Single shot mode
*/
void ADC_S_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  //ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGA_data[0]); // Set UV and OV Threshold for all Cells
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADSV(Singleshot_ADSV, NODISCHARGE_ADSV, ALL_CH_OW_OFF_ADSV, ADSV);
  ADBMS_Write_Cmd(TOTAL_IC, ADSV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  //MXC_Delay(15000);
  ADBMS_Read_Data(TOTAL_IC,   RDSVA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDSVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDSVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDSVB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDSVC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDSVC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDSVD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDSVD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDSVE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDSVE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDSVF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDSVF, Data_Read, GRPF);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRSPIN);
  ADBMS_Read_Data(TOTAL_IC,   RDSVA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDSVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDSVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDSVB, Data_Read, GRPB);
  free(Data_Read);
}

/*!
  @brief Measure Averge C-ADC Voltage
*/
void ADC_AVG_Cell_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  //ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGA_data[0]); // Set UV and OV Threshold for all Cells
  //MXC_Delay(5000);
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADCV(CONT_ADCV, RD_OFF_ADCV, NO_RESET_ADCV, NODISCHARGE_ADCV, ALL_CH_OW_OFF_ADCV, ADCV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCV);
  pollVoltCTSconversion();
  ADBMS_Read_Data(TOTAL_IC,   RDACA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDACA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDACB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDACB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDACC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDACC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDACD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDACD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDACE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDACE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDACF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDACF, Data_Read, GRPF);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRCELL);
  free(Data_Read);
}

/*!
  @brief Measure GPIO Voltage
*/
void ADC_GPIO_Voltage_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
	WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
	ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
	WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  //ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGA_data[0]); // Set UV and OV Threshold for all Cells
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADAX(OW_AUX_OFF_ADAX, ALL_ADAX, PULLDOWN_ADAX, ADAX);
  ADBMS_Write_Cmd(TOTAL_IC, ADAX);
  ADBMSPollADC(TOTAL_IC, PLAUX1);
  MXC_Delay(50000); // 20msec delay to get data of all GPIO Regs
  ADBMS_Read_Data(TOTAL_IC,       RDAUXA, &Data_Read[0], GRPA);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,       RDAUXB, &Data_Read[0], GRPB);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,       RDAUXC, &Data_Read[0], GRPC);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,       RDAUXD, &Data_Read[0], GRPD);
  ADBMS_Print_AUX_Data(TOTAL_IC,  RDAUXD, Data_Read, GRPD);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRAUX);
  free(Data_Read);
}

/*!
  @brief Measure Status Voltage
*/


void ADC_Cont_Sync_V_and_I_Measurement(uint8_t TOTAL_IC)
{
  int   Reg_Size=RDCSIVALL_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
  WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC
  ADBMS_Write_Cmd(TOTAL_IC, SRST); // Reset ADBMS (ADBMS will goes into sleep state)
  WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], GRPA); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_ADCIV(SINGLE_ADCIV, RD_ON_ADCIV, NO_RESET_ADCIV, NODISCHARGE_ADCIV, ALL_CH_OW_OFF_ADCIV, ADCIV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCIV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  //MXC_Delay(8800);
  ADBMS_Read_Data(TOTAL_IC, RDACIVALL, &Data_Read[0], RD_ACIV_ALL);
  ADBMS_Print_All_Reg_Data(TOTAL_IC, RDACIVALL, Data_Read, RD_ACIV_ALL);
  printf("\nCLR Command\n");
  ADBMS_Write_Cmd(TOTAL_IC, CLRCELL); //CLR C-ADC
  ADBMS_Write_Cmd(TOTAL_IC, CLRCI);   //CLR I1ADC
  ADBMS_Write_Cmd(TOTAL_IC, CLRSI);   //CLR I2ADC
  ADBMS_Read_Data(TOTAL_IC, RDACIVALL, &Data_Read[0], RD_ACIV_ALL);
  ADBMS_Print_All_Reg_Data(TOTAL_IC, RDACIVALL, Data_Read, RD_ACIV_ALL);
  free(Data_Read);
}

void ADC_Coulumb_Counter_Measurement(uint8_t TOTAL_IC)
{
  int  Reg_Size=12;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
  WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC
  ADBMS_Write_Cmd(TOTAL_IC, SRST); // Reset ADBMS (ADBMS will goes into sleep state)
  WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); //Fault CC and TB set to 0 // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond to back to MCU on MISO (SDO) line
  WakeupBMSIC();
  ADBMS_Write_Data(TOTAL_IC, WRCFGC, &CFGC_data[0]); // Set TCC
  printf("\nWRCFGC\n");
  ADBMS_Print_WRCFG_Data(TOTAL_IC,  WRCFGC, CFGC_data, GRPC);

  ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
  printf("\nRDSTATC\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATC, Data_Read, GRPC);

  ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
  printf("\nRDSTATG\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATG, Data_Read, GRPG);

  ADBMS_Write_Cmd(TOTAL_IC, CLRCC);
  ADBMS_Write_Cmd(TOTAL_IC, CLRFLAG);
  ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
  printf("\nRDSTATC after CLRCC\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATC, Data_Read, GRPC);

  ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
  printf("\nRDSTATG after CLRCC\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATG, Data_Read, GRPG);
  ADBMS_ADCIV(CONT_ADCIV, RD_ON_ADCIV, NO_RESET_ADCIV, NODISCHARGE_ADCIV, ALL_CH_OW_OFF_ADCIV, ADCIV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCIV);
  MXC_Delay(10000);

  ADBMS_CCEN(SINGLE_CCEN, START_CCEN, CCEN);
  ADBMS_Write_Cmd(TOTAL_IC, CCEN);

  for(int i=0;i<5;i++)
  {
  //pollCurntCTSconversion(); //not workimg getting 0 continuously
  MXC_Delay(1000);
  printf("\nCount:%d\n",i);
  ADBMS_Read_Data(TOTAL_IC, RDI, &Data_Read[0], RD_I_ADC);
  ADBMS_Print_Current_Data(TOTAL_IC, RDI, Data_Read, RD_I_ADC);
  ADBMS_Read_Data(TOTAL_IC, RDCT, &Data_Read[0], GRP_NONE);
  ADBMS_Print_Coulumb_Counter_Data(TOTAL_IC, RDCT, Data_Read, GRP_NONE);
  ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
  printf("\nRDSTATC\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
  printf("\nRDSTATG\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATG, Data_Read, GRPG);
  ADBMS_Write_Cmd(TOTAL_IC, CLRCC);
  ADBMS_Write_Cmd(TOTAL_IC, CLRCI);
  }
  /*Read Coulomb counter and time base register and validate that values are 0*/
  ADBMS_Read_Data(TOTAL_IC, RDCT, &Data_Read[0], GRP_NONE);
  ADBMS_Print_Coulumb_Counter_Data(TOTAL_IC, RDCT, Data_Read, GRP_NONE);

   // ADBMS_CCEN(CONT_CCEN, STOP_CCEN, CCEN);//Stop CC
   // ADBMS_Write_Cmd(TOTAL_IC, CCEN);

  ADBMS_ADCIV(CONT_ADCIV, RD_ON_ADCIV, NO_RESET_ADCIV, NODISCHARGE_ADCIV, ALL_CH_OW_OFF_ADCIV, ADCIV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCIV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  free(Data_Read);
}

void ADC_Coulumb_Counter_Measurement_debugg(uint8_t TOTAL_IC)
{
  int  Reg_Size=12;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
  WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC
  ADBMS_Write_Cmd(TOTAL_IC, SRST); // Reset ADBMS (ADBMS will goes into sleep state)
  WakeupBMSIC(); //Toggle CS high and low to wakeup BMS IC after Reset to bring ADBMS from SLEEP to STANDBY

  printf("\nRDSTATC\n"); //CCFLT, TBOF
  ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATC, Data_Read, GRPC);

  printf("\nRDSTATE\n"); // OC1, OC2, FAULTB
  ADBMS_Read_Data(TOTAL_IC, RDSTATE, &Data_Read[0], GRPE);
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATE, Data_Read, GRPE);

  printf("\nRDSTATD\n"); //CCFLT, TBOF
  ADBMS_Read_Data(TOTAL_IC, RDSTATD, &Data_Read[0], GRPD);
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATD, Data_Read, GRPD);

  printf("\nRDSTATG\n");
  ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATG, Data_Read, GRPG);

  ADBMS_Write_Cmd(TOTAL_IC, CLRCC);
  ADBMS_Write_Cmd(TOTAL_IC, CLRCI);
  ADBMS_Write_Cmd(TOTAL_IC, CLRFLAG);

  WakeupBMSIC();
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); //Fault CC and TB set to 0 // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS respond back to MCU on MISO (SDO) line
  WakeupBMSIC();

  ADBMS_ADI1(CONT_ADI1, NORMAL_OPERATION_ADI1, NORMAL_ADI1, RD_ON_ADI1, ADI1);
  ADBMS_Write_Cmd(TOTAL_IC, ADI1);
  pollCurntCTSconversion();
  //MXC_Delay(2000);

  ADBMS_CCEN(CONT_CCEN, START_CCEN, CCEN);
  ADBMS_Write_Cmd(TOTAL_IC, CCEN);

  for(int i=0;i<5;i++)
  {
  printf("\nCount:%d\n",i);
  pollCurntCTSconversion();
  //ADBMS_Read_Data(TOTAL_IC, RDI, &Data_Read[0], RD_I_ADC);
  //ADBMS_Print_Current_Data(TOTAL_IC, RDI, Data_Read, RD_I_ADC);
  ADBMS_Read_Data(TOTAL_IC, RDCT, &Data_Read[0], GRP_NONE);
  ADBMS_Print_Coulumb_Counter_Data(TOTAL_IC, RDCT, Data_Read, GRP_NONE);
  }
  /*Read Coulomb counter and time base register and validate that values are 0*/
  ADBMS_Write_Cmd(TOTAL_IC, CLRCC);
  ADBMS_Write_Cmd(TOTAL_IC, CLRCI);
  ADBMS_Read_Data(TOTAL_IC, RDCT, &Data_Read[0], GRP_NONE);
  ADBMS_Print_Coulumb_Counter_Data(TOTAL_IC, RDCT, Data_Read, GRP_NONE);

  ADBMS_ADCIV(SINGLE_ADCIV, RD_ON_ADCIV, NO_RESET_ADCIV, NODISCHARGE_ADCIV, ALL_CH_OW_OFF_ADCIV, ADCIV);
  ADBMS_Write_Cmd(TOTAL_IC, ADCIV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  ADBMS_Read_Data(TOTAL_IC, RDSTATC, &Data_Read[0], GRPC);
  printf("\nRDSTATC\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
  printf("\nRDSTATG\n");
  ADBMS_Print_RDCFG_Data(TOTAL_IC,  RDSTATG, Data_Read, GRPG);

  free(Data_Read);
}

void ADSV_PWM_Discharge_Measurement(uint8_t tIC , uint8_t Timer_value)
{
  int   Reg_Size=RECEIVE_RD_PACKET_SIZE;
  uint8_t *Data_Read;
  Data_Read = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));
  WakeupBMSIC(); //Wakeup BMS IC and IsoSPI
  ADBMS_Write_Cmd(TOTAL_IC, &SRST[0]); // Reset ADBMS (ADBMS will goes into sleep state)
  WakeupBMSIC(); ////Wakeup BMS IC and IsoSPI after Reset to bring ADBMS from SLEEP to STANDBY
  ADBMS_Write_Data(TOTAL_IC, WRCFGA, &CFGA_data[0]); // Set REFON = 1 to bring ADBMS from Standby to REFUP State
  ADBMS_Read_Data(TOTAL_IC, RDCFGA, &Data_Read[0], NOT_ALL); //RDCFGA data should be match with WRCFGA data. RDCFGA helps to make sure that ADBMS responds back to MCU on MISO (SDO) line

  //Send ADSV with DCP = 0 and DCTO = 0 and check discharge should not happen
  printf("\nSend ADSV with DCP = 0 and DCTO = 0 and check discharge should not happen\n");
  WakeupBMSIC();
  ADBMS_ADSV(Singleshot_ADSV, NODISCHARGE_ADSV, ALL_CH_OW_OFF_ADSV, ADSV);
  ADBMS_Write_Cmd(TOTAL_IC, ADSV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  //MXC_Delay(15000);
  ADBMS_Read_Data(TOTAL_IC,   RDSVA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDSVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDSVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDSVB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDSVC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDSVC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDSVD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDSVD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDSVE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDSVE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDSVF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDSVF, Data_Read, GRPF);
  ADBMS_Write_Cmd(TOTAL_IC, CLRSPIN);

  //DCTO = 3min (1 min per bit) in WRCFGB and Send ADSV with DCP = 0 and \ncheck discharge should not happen
  printf("\nSet DCTO = 3min (1 min per bit) in WRCFGB and Send ADSV with DCP = 0 and \ncheck discharge should not happen\n");
  WakeupBMSIC();
  WRCFGB_data_Set(TOTAL_IC, WRCFGB, UV_THRESHOLD, OV_THRESHOLD, &CFGB_data[0]);
  ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGB_data[0]); // Set UV and OV Threshold for all Cells and set DCTO for 3min
  printf("CFGBR0: %x\n", CFGB_data[0]);
  printf("CFGBR1: %x\n", CFGB_data[1]);
  printf("CFGBR2: %x\n", CFGB_data[2]);
  printf("CFGBR3: %x\n", CFGB_data[3]);
  printf("CFGBR4: %x\n", CFGB_data[4]);
  printf("CFGBR5: %x\n", CFGB_data[5]);

  ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPB);

  ADBMS_ADSV(Singleshot_ADSV, NODISCHARGE_ADSV, ALL_CH_OW_OFF_ADSV, ADSV);
  ADBMS_Write_Cmd(TOTAL_IC, ADSV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  //MXC_Delay(15000);
  ADBMS_Read_Data(TOTAL_IC,   RDSVA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDSVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDSVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDSVB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDSVC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDSVC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDSVD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDSVD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDSVE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDSVE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDSVF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDSVF, Data_Read, GRPF);
  ADBMS_Write_Cmd(TOTAL_IC, CLRSPIN);

  //DCTO = 3min (1 min per bit. so, set 00011 for 3min) in WRCFGB and Send ADSV with DCP = 1 and check discharge should happen now
  printf("\nSet DCTO = 3min (1 min per bit. so, set 00011 for 3min) in WRCFGB\nSend WRPWM1 command with 100 percent duty cycle at cell 1 and \nSend ADSV with DCP = 1 and check discharge should happen now\n");
  //WRPWM to discharge the Cell
  WakeupBMSIC();
  WRCFGB_data_Set(TOTAL_IC, WRCFGB, UV_THRESHOLD, OV_THRESHOLD, &CFGB_data[0]);
  CFGB_data[3] = 0b11000000 | (0x3F & Timer_value);  //DCTO Timer setting, DTRNG Setting , DTMEN bit has to enabled to stop cell discharge when UV is hit

  ADBMS_Write_Data(TOTAL_IC, WRCFGB, &CFGB_data[0]); // Set UV and OV Threshold for all Cells and set DCTO for 3min
  ADBMS_Read_Data(TOTAL_IC, RDCFGB, &Data_Read[0], GRPB);

  printf("CFGBR0: %x\n", CFGB_data[0]);
  printf("CFGBR1: %x\n", CFGB_data[1]);
  printf("CFGBR2: %x\n", CFGB_data[2]);
  printf("CFGBR3: %x\n", CFGB_data[3]);
  printf("CFGBR4: %x\n", CFGB_data[4]);
  printf("CFGBR5: %x\n", CFGB_data[5]);

  ADBMS_Write_Data(TOTAL_IC, WRPWMA, &PWMA_data[0]); // Set 100% PWM for cell 1
  ADBMS_Read_Data(TOTAL_IC, RDPWMA, &Data_Read[0], GRPA);

  ADBMS_Write_Data(TOTAL_IC, WRPWMB, &PWMB_data[0]); // Set 100% PWM for cell 1
  ADBMS_Read_Data(TOTAL_IC, RDPWMB, &Data_Read[0], GRPB);

  ADBMS_ADSV(Singleshot_ADSV, DISCHARGE_ADSV, ALL_CH_OW_OFF_ADSV, ADSV);
  ADBMS_Write_Cmd(TOTAL_IC, ADSV);
  ADBMSPollADC(TOTAL_IC, PLADC);
  //MXC_Delay(15000);
  ADBMS_Read_Data(TOTAL_IC,   RDSVA, &Data_Read[0], GRPA);
  ADBMS_Print_Data(TOTAL_IC,  RDSVA, Data_Read, GRPA);
  ADBMS_Read_Data(TOTAL_IC,   RDSVB, &Data_Read[0], GRPB);
  ADBMS_Print_Data(TOTAL_IC,  RDSVB, Data_Read, GRPB);
  ADBMS_Read_Data(TOTAL_IC,   RDSVC, &Data_Read[0], GRPC);
  ADBMS_Print_Data(TOTAL_IC,  RDSVC, Data_Read, GRPC);
  ADBMS_Read_Data(TOTAL_IC,   RDSVD, &Data_Read[0], GRPD);
  ADBMS_Print_Data(TOTAL_IC,  RDSVD, Data_Read, GRPD);
  ADBMS_Read_Data(TOTAL_IC,   RDSVE, &Data_Read[0], GRPE);
  ADBMS_Print_Data(TOTAL_IC,  RDSVE, Data_Read, GRPE);
  ADBMS_Read_Data(TOTAL_IC,   RDSVF, &Data_Read[0], GRPF);
  ADBMS_Print_Data(TOTAL_IC,  RDSVF, Data_Read, GRPF);
  ADBMS_Write_Cmd(TOTAL_IC, CLRSPIN);

//  ADBMS_Write_Cmd(TOTAL_IC, UNMUTE);

  free(Data_Read);
}

/**************************Print Function********************************/

/*!
  @brief Print Write CNFG Reg Data
*/
void ADBMS_Print_WRCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int Data_Size = 6;
  for(int IC=1; IC<=tIC; IC++)
  {
    for(int i=0; i<(Data_Size); i++)
    {
      printf("0x%x\t",buff[i]);
    }
    printf("\n");
  }
}

/*!
  @brief Print Read CNFG Reg Data
*/
void ADBMS_Print_RDCFG_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int Data_Size = 6;
  int buffdata = 0;
  for(int IC=1; IC<=tIC; IC++)
  {
    for(int i=0; i<(Data_Size); i++)
    {
      printf("0x%x\t",buff[buffdata]);
      buffdata++;
    }
    printf("\n");
  }
}

/*!
  @brief Print Cell , S, Avg , Fillterced cell voltages
*/
void ADBMS_Print_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int16_t x, y, z; //to hold raw data
  uint8_t i = 0;
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    x = twos_complement_to_int((buff[i+1]<< 8)|buff[i],16);//Combine lower and upper bytes
    y = twos_complement_to_int((buff[i+3]<< 8)|buff[i+2],16);
    z = twos_complement_to_int((buff[i+5]<< 8)|buff[i+4],16);
    if			  ( type == GRPA){printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if		( type == GRPB){printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPC){printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPD){printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPE){printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(y), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(z));}
    else if 	( type == GRPF){printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(x));}
    else	    {printf("error");}
    i +=6;
  }
}

/*!
  @brief Print Current Reg data
*/
float ADBMS_Print_Current_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  double a,b, I1;
  uint32_t x, y; //to hold raw data
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    x = (((0x00) << 24)|((buff[2] & 0x07)<< 16)|(buff[1]<< 8)|buff[0]);
    y = (((0x00) << 24)|((buff[5] & 0x07)<< 16)|(buff[4]<< 8)|buff[3]);
    a = (double)x;
    b = (double)y;
    #if Test_Print
        printf("\n");
        printf("%x\t ", x);
        printf("%x\t ", y);
        printf("\n");
        printf("\n");
        printf("%lf\t ", a);
        printf("%lf\t ", b);
        printf("\n");
    #endif
    if (x > 0x3ffff){a = (double)x - (double)(0x7ffff);}
    if (y > 0x3ffff){b = (double)y - (double)(0x7ffff);}
    I1	=	ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(a) / ((double)200 * (double)0.000001);
//    I2	=	ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(b) / ((double)200 * (double)0.000001);
//    printf("\nIC:0%x->\t I1ADC  =%1.9lf\t I2ADC =%1.9lf\n",(IC+1), ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(a), ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(b));
//    printf("\nIC:0%x->\t Current1(A)  =%1.9lf\t Current2(A) =%1.9lf\n",(IC+1), I1, I2);

//    printf("Load_Curr: %f\n",I1);
    return I1;
  }
}

/*!
  @brief Print GPIO voltages
*/
void ADBMS_Print_AUX_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int16_t x, y, z; //to hold raw data
  uint8_t i = 0;
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    x = twos_complement_to_int((buff[i+1]<< 8)|buff[i],16);//Combine lower and upper bytes
    y = twos_complement_to_int((buff[i+3]<< 8)|buff[i+2],16);
    z = twos_complement_to_int((buff[i+5]<< 8)|buff[i+4],16);
    if			  ( type == GRPA){printf("\nIC:0%x->\t  GPIO1  =%1.3lf\t GPIO2 =%1.3lf\t GPIO3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(y),  ADBMS_CONVERT_GPIO_HEX_TO_VOLT(z));}
    else if		( type == GRPB){printf("\nIC:0%x->\t  GPIO4  =%1.3lf\t GPIO5 =%1.3lf\t GPIO6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(y),  ADBMS_CONVERT_GPIO_HEX_TO_VOLT(z));}
    else if 	( type == GRPC){printf("\nIC:0%x->\t  GPIO7  =%1.3lf\t GPIO8 =%1.3lf\t GPIO9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(y),  ADBMS_CONVERT_GPIO_HEX_TO_VOLT(z));}
    else if 	( type == GRPD){printf("\nIC:0%x->\t  GPIO10 =%1.3lf\t GPIO11=%1.3lf\n",(IC+1), ADBMS_CONVERT_GPIO_HEX_TO_VOLT(x), ADBMS_CONVERT_VM_HEX_TO_VOLT(y));}
    else	    {printf("error");}
    i +=6;
  }
}

/*!
  @brief Print Status Reg data
*/
float ADBMS_Print_Status_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  int16_t x, y, z; //to hold raw data
  uint8_t i = 0;
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    x = twos_complement_to_int((buff[i+1]<< 8)|buff[i],16);//Combine lower and upper bytes
    y = twos_complement_to_int((buff[i+3]<< 8)|buff[i+2],16);
    z = twos_complement_to_int((buff[i+5]<< 8)|buff[i+4],16);
    if			  ( type == GRPA){printf("\nIC:0%x->\t  VREF2 = %1.3lf\t IC Dia Temperature = %1.3lf\n",              (IC+1), ADBMS_CONVERT_VREF2_HEX_TO_VOLT(x),   ADBMS_CONVERT_ITEMP_HEX_TO_VOLT(y));}
    else if		( type == GRPB){printf("\nIC:0%x->\t  VDigital = %1.3lf\t VAnalog = %1.3lf\t VR4K = %1.3lf\n",      (IC+1), ADBMS_CONVERT_VD_HEX_TO_VOLT(x),      ADBMS_CONVERT_VA_HEX_TO_VOLT(y),  ADBMS_CONVERT_VR4K_HEX_TO_VOLT(z));}
    else if		( type == GRPF){
								float Pack_Voltage = ADBMS_CONVERT_VP_HEX_TO_VOLT(y);
							    return Pack_Voltage;}
    else	    {printf("error");}
    i +=6;

  }

}

/*!
  @brief Print All Reg data
*/
void ADBMS_Print_All_Reg_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
  uint8_t  RDDataSize;
  if      (type == RD_CV_ALL)		{RDDataSize = RDCVALL_SIZE;}
  else if (type == RD_AC_ALL)		{RDDataSize = RDACVALL_SIZE;}
  else if (type == RD_S_ALL)		{RDDataSize = RDSVALL_SIZE;}
  else if (type == RD_FC_ALL)		{RDDataSize = RDFCVALL_SIZE;}
  else if (type == RD_AUX_ALL)	{RDDataSize = RDAUXALL_SIZE;}
  else if (type == RD_C_S_ALL)	{RDDataSize = RDCSALL_SIZE;}
  else if (type == RD_AC_S_ALL)	{RDDataSize = RDACSALL_SIZE;}
  else if ((type == RD_CIV_ALL)  || (type == RD_ACIV_ALL) ||(type == RD_SVAI_ALL))
  									            {RDDataSize = RDCIVALL_SIZE;}
  else if ((type == RD_CSIVALL) || (type == RD_ACSIVALL) )
  									            {RDDataSize = RDCSIVALL_SIZE;}
  else if (type == RD_STA_ALL)	{RDDataSize = RDSTAALL_SIZE;}
  else if (type == RD_C_CFG_ALL){RDDataSize = RDCCFGALL_SIZE;}
  else								          {RDDataSize = 8;}//6 Byte data + 2byte data pec
  //uint8_t CountofSADC = 16, CountofCSADC =32;
  int16_t a[N_CELLS];
  uint32_t x[N_IADC];
  double y[N_IADC];
  uint8_t CurrADCSize = ((N_CELLS_PER_REGISTERS * BYTES_IN_CELL))*tIC;
  uint8_t VoltADCSize = ((N_CELLS * BYTES_IN_CELL))*tIC;

  uint8_t i=0, j=0;
  for(uint8_t IC=0;IC<tIC;IC++)
  {
    do
      {
    	a[j] = ((buff[i+1]<< 8)|buff[i]);
        if (a[j] > 0x3fff){a[j] = a[j] - (0x7fff);}
        j=j+1;
        i=i+2;
      }while(i < VoltADCSize);
    i = (RDDataSize-CurrADCSize-PEC_SIZE); j=0;
    do
      {
        x[j] = (((0x00) << 24)|((buff[i+2] & 0x07)<< 16)|(buff[i+1]<< 8)|buff[i]);
        y[j] = (double)x[j];
        if (x[j] > 0x3ffff){y[j] =  (double)x[j] - (double)(0x7ffff);}
        j = j + 1;
        i = i + 3;
      }while(i < (RDDataSize-PEC_SIZE));

    if (type == RD_CV_ALL || type == RD_AC_ALL || type == RD_S_ALL || type == RD_FC_ALL)
    {
      printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[0]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[1]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[2]));
      printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[3]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[4]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[5]));
      printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[6]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[7]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[8]));
      printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[9]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[10]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[11]));
      printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[12]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[13]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[14]));
      printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[15]));
    }
    else if (type == RD_AUX_ALL)
    {
      break;
    }
    else if (type == RD_C_S_ALL || type == RD_AC_S_ALL)
    {
      printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[0]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[1]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[2]));
      printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[3]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[4]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[5]));
      printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[6]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[7]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[8]));
      printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[9]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[10]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[11]));
      printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[12]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[13]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[14]));
      printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[15]));
      printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[16]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[17]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[18]));
      printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[19]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[20]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[21]));
      printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[22]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[23]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[24]));
      printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[25]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[26]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[27]));
      printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[28]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[29]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[30]));
      printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[31]));
    }

    else if (type == RD_CIV_ALL || type == RD_ACIV_ALL || type == RD_SVAI_ALL)
    {
      printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[0]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[1]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[2]));
      printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[3]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[4]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[5]));
      printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[6]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[7]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[8]));
      printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[9]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[10]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[11]));
      printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[12]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[13]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[14]));
      printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[15]));
      printf("\nIC:0%x->\t I1ADC  =%1.9lf\t I2ADC =%1.9lf\n",(IC+1), ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(y[0]),ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(y[1]));
    }

    else if ((type == RD_CSIVALL) || (type == RD_ACSIVALL) )
    {
      printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[0]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[1]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[2]));
      printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[3]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[4]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[5]));
      printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[6]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[7]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[8]));
      printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[9]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[10]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[11]));
      printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[12]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[13]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[14]));
      printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[15]));
      printf("\nIC:0%x->\t CELL1  =%1.3lf\t CELL2 =%1.3lf\t CELL3 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[16]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[17]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[18]));
      printf("\nIC:0%x->\t CELL4  =%1.3lf\t CELL5 =%1.3lf\t CELL6 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[19]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[20]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[21]));
      printf("\nIC:0%x->\t CELL7  =%1.3lf\t CELL8 =%1.3lf\t CELL9 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[22]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[23]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[24]));
      printf("\nIC:0%x->\t CELL10 =%1.3lf\t CELL11=%1.3lf\t CELL12=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[25]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[26]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[27]));
      printf("\nIC:0%x->\t CELL13 =%1.3lf\t CELL14=%1.3lf\t CELL15=%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[28]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[29]), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[30]));
      printf("\nIC:0%x->\t CELL16 =%1.3lf\n",(IC+1), ADBMS_CONVERT_CELLVOLTAGE_HEX_TO_VOLT(a[31]));
      printf("\nIC:0%x->\t I1ADC  =%1.9lf\t I2ADC =%1.9lf\n",(IC+1), ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(y[0]),ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(x[1]));
    }
  }
}

/*!
  @brief Print Coulumb Counter Data
*/
void ADBMS_Print_Coulumb_Counter_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
{
 double a,b;
 int32_t x; //to hold raw data
 int16_t y;
 x = ((buff[3] << 24)|(buff[2]<< 16)|(buff[1]<< 8)|buff[0]);
 y = ((buff[5]<< 8)|buff[4]);
 a = (double)x;
 b = (double)y;
  printf("\n");
  printf("%x\t ", x);
  printf("%x\t ", y);
  printf("\n");
  printf("\n");
  printf("%lf\t ", a);
  printf("%lf\t ", b);
  printf("\n");

 if (x > 0x3fffff){a = (double)x - (double)(0x7fffff);}
 if (y > 0x3fff  ){b = (double)y - (double)(0x7fff  );}

 {printf("\n Coulumb Count  =%1.9lf\t Time Base =%1.9lf\n",ADBMS_CONVERT_Coulumb_Cntr_HEX_TO_VOLT(a), b);}
}

/*!
  @brief Check Singleshot ADC conversion
*/
//To check ADC(Single shot) conversion
//Single shot mode only
//Not applicable to Continuous mode
void ADBMSPollADC(uint8_t tIC, uint8_t cmd_arg[2])
{
  int Pec_Size = 2, Cmd_Size =2, Check_ADC_Conversion = 1;
  int WRCmdSize = Cmd_Size + Pec_Size + Check_ADC_Conversion;
  uint16_t cmd_pec;
  uint8_t tx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  uint8_t rx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  //WRCmdPec(cmd_arg, tx_data);
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Pec_Size, cmd_arg); //2byte cmd pec
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  WakeupBMSIC();
  do
  {
  SPI_Transaction(tx_data, rx_data, WRCmdSize);
  }while(rx_data[4]<0x03); //It make sure that SDO will go high after ADC conversion will done.
  #if Test_Print
      printf("\nCommand\n");
      printf("0x%x\t0x%x\t0x%x\t0x%x\t0x%x\n",tx_data[0], tx_data[1],tx_data[2], tx_data[3], tx_data[4]);
  #endif
}

/*!
  @brief This function polls the STATG register until CTS_V bit is reset back to 0
*/
//Only for C ADC
//Single shot and Cont
//RDCVA/RDACA/RDFCA
void pollVoltCTSconversion(void)
{
  uint8_t Data_Read[8];

  do
  {
    /*Read Status Register*/
    ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
    //printf("\n%x\t", (Data_Read[1] & 3U));
  } while (((Data_Read[1] & 3U) != 3U));

  do
  {
    /*Read Status Register*/
    ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
    //printf("\t%x\n", (Data_Read[1] & 3U));
  } while (Data_Read[1] & 3U); // It will come out when CTS_V become 0
}

/*!
  @brief This function polls the STATG register until CTS_V bit is reset back to 0, 8 times
*/
//Only for C ADC and S ADC
//Single shot and Cont
//RDCVA/RDACA/RDFCA/RDSVA
void poll8msCtsVconversion(void)
{
  uint8_t vpoll_count = 8;
  for (uint8_t i = 0; i < vpoll_count; i++)
  {
    pollVoltCTSconversion();
  }
}

/*!
  @brief This function polls the STATG register until CTS_I bit is reset back to 0
*/
void pollCurntCTSconversion(void)
{
	uint8_t Data_Read[8];
  do
  {
    /*Read Status Register*/
    ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
    //printf("\n%x\t", (Data_Read[3] & 3U));
  } while (((Data_Read[3] & 3U) != 3U));

  do
  {
    /*Read Status Register*/
    ADBMS_Read_Data(TOTAL_IC, RDSTATG, &Data_Read[0], GRPG);
    //printf("\t%x\n", (Data_Read[3] & 3U));
  } while (Data_Read[3] & 3U); // It will come out when CTS_V become 0
}

/*!
 @brief This function polls the STATG register until CTS_I bit is reset back to 0, 8 times
*/
//void pollAvgCurntCTSconversion(const ACCI_bit_t acci)
//{
// for (uint8_t i = 0; i < (4*(uint8_t)acci + 4); i++)
// {
//   pollCurntCTSconversion();
// }
//}

// void ADBMS_Print_Current_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *buff, RD_DATA_SIZE_ALL_TYPE type)
// {
//     uint32_t x, y; //to hold raw data
//     //uint8_t cmd;
//     x = twos_complement_to_int((0x00 << 24)|((buff[2] & 0x07)<< 16)|(buff[1]<< 8)|buff[0],32);//Combine lower and upper bytes
//     y = twos_complement_to_int((0x00 << 24)|((buff[5] & 0x07)<< 16)|(buff[4]<< 8)|buff[3],32);
//     printf("\n");
//     printf("%x\t ", buff[0]);
//     printf("%x\t ", buff[1]);
//     printf("%x\t ", buff[2]);
//     printf("\n");
//     printf("%x\t ", x);
//     printf("%x\t ", y);
//     printf("\n");
//     printf("\nI1ADC  =%1.9lf\t I2ADC =%1.9lf\n",ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(x), ADBMS_CONVERT_CURRENT_HEX_TO_VOLT(y));
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*!
  @brief Write Dummy data Function to wakeup ISOSPI
*/
void SPI_Write(uint8_t tIC, uint8_t cmd_arg[0])
{
  uint8_t tx_data[1];//2 byte cmd + 2byte cmd pec
  uint8_t rx_data[1];//2 byte cmd + 2byte cmd pec
  tx_data[0] = cmd_arg[0];
  for(int i=0; i<tIC; i++)
  {
   SPI_Transaction(tx_data, rx_data, 1);
   #if Test_Print
        printf("\n%x\n",tx_data[0]);
   #endif
  }
}

/*!
  @brief Write Command Function
*/
void ADBMS_Write_Cmd(uint8_t tIC, uint8_t cmd_arg[2])
{
  int Pec_Size = 2, Cmd_Size =2;
  int WRCmdSize = Cmd_Size + Pec_Size;
  uint16_t cmd_pec;
  uint8_t tx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  uint8_t rx_data[WRCmdSize];//2 byte cmd + 2byte cmd pec
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Pec_Size, cmd_arg); //2byte cmd pec
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  WakeupBMSIC();
  SPI_Transaction(tx_data, rx_data, WRCmdSize);
  #if Test_Print
      printf("\nCommand\n");
      printf("0x%x\t0x%x\t0x%x\t0x%x\n",tx_data[0], tx_data[1],tx_data[2], tx_data[3]);
  #endif
}

/*!
  @brief Write data Function
*/
void ADBMS_Write_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Write)
{
  int Cmd_Size = 2, Cmd_Pec_Size = 2, Data_Size = 6, Data_Pec_Size = 2;
  int WRCmdSize = Cmd_Size + Cmd_Pec_Size;
  int txdatabuff = WRCmdSize;
  int WRDataSize = (Data_Size + Data_Pec_Size)*tIC;
  int Reg_Size = WRCmdSize + WRDataSize;
  uint16_t cmd_pec, data_pec;
  uint8_t *tx_data;
  tx_data = (uint8_t*)calloc(Reg_Size, sizeof(uint8_t));//2 byte cmd + 2byte cmd pec + (x Byte data+ 2byte data pec)*tIC
  uint8_t rx_data[Reg_Size];//2 byte cmd + 2byte cmd pec + 6 Byte data + 2byte data pec
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Cmd_Size, cmd_arg); //2byte cmd
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  /*Executes for each ADBMS6xxx, loop starts with the last IC on the stack
    The first configuration written is received by the last IC in the chain*/
  for(int IC=tIC; IC>0; IC--)
  {
   for(int i=0; i<(Data_Size); i++)
   {
     tx_data[(txdatabuff)] = data_Write[i];
     txdatabuff++;
   }
   data_pec = pec10_calc(true, Data_Size, data_Write);
   tx_data[(txdatabuff)] = (uint8_t)(data_pec >> 8);
   txdatabuff++;
   tx_data[(txdatabuff)] = (uint8_t)(data_pec);
   txdatabuff++;
  }
  WakeupBMSIC();
  SPI_Transaction(tx_data, rx_data, Reg_Size);
  MXC_Delay(1000);
  #if Test_Print
  	  	txdatabuff = WRCmdSize;
        printf("\nWrite Data\n");
        for(int IC=1; IC<=tIC; IC++)
        {
          printf("IC:0%x\t",IC);
          for(int i=0; i<(Data_Size + Data_Pec_Size); i++)
          {
            printf("0x%x\t",tx_data[txdatabuff]);
            txdatabuff++;
          }
          printf("\n");
        }
  #endif
  free(tx_data);
}

/*!
  @brief Read Function of Command of BMS IC
*/
void ADBMS_Read_Data(uint8_t tIC, uint8_t cmd_arg[], uint8_t *data_Read, RD_DATA_SIZE_ALL_TYPE type)
{
  int  RDDataSize;//6 Byte data + 2byte data pec
  if (type == RD_CV_ALL)			{RDDataSize = RDCVALL_SIZE;}
  else if (type == RD_AC_ALL)		{RDDataSize = RDACVALL_SIZE;}
  else if (type == RD_S_ALL)		{RDDataSize = RDSVALL_SIZE;}
  else if (type == RD_FC_ALL)		{RDDataSize = RDFCVALL_SIZE;}
  else if (type == RD_AUX_ALL)		{RDDataSize = RDAUXALL_SIZE;}
  else if (type == RD_C_S_ALL)		{RDDataSize = RDCSALL_SIZE;}
  else if (type == RD_AC_S_ALL)	{RDDataSize = RDACSALL_SIZE;}
  else if ((type == RD_CIV_ALL)  || (type == RD_ACIV_ALL) ||(type == RD_SVAI_ALL))
  									{RDDataSize = 40U;}
  else if ((type == RD_CSIVALL) || (type == RD_ACSIVALL) )
  									{RDDataSize = 72U;}
  else if (type == RD_STA_ALL)		{RDDataSize = RDSTAALL_SIZE;}
  else if (type == RD_C_CFG_ALL)	{RDDataSize = RDCCFGALL_SIZE;}
  else								{RDDataSize = 8;}//6 Byte data + 2byte data pec
  int Cmd_Size = 2, Cmd_Pec_Size = 2, Data_Pec_Size =2;
  int WRCmdSize = Cmd_Size + Cmd_Pec_Size;
  int Reg_Size = WRCmdSize + RDDataSize*tIC;
  uint16_t cmd_pec;
  uint8_t *rx_data, *copyArray;
  copyArray = (uint8_t *)calloc(RDDataSize*tIC, sizeof(uint8_t)); //For Data pec Calculation and Verification
  rx_data = (uint8_t *)calloc(Reg_Size, sizeof(uint8_t));         //2 byte cmd + 2byte cmd pec + x Byte data + 2byte data pec
  uint8_t tx_data[Reg_Size];                                      //2 byte cmd + 2byte cmd pec + x Byte data + 2byte data pec
  tx_data[0] = cmd_arg[0];
  tx_data[1] = cmd_arg[1];
  cmd_pec = Pec15_Calc(Cmd_Size, cmd_arg); //2byte cmd
  tx_data[2] = (uint8_t)(cmd_pec >> 8);
  tx_data[3] = (uint8_t)(cmd_pec);
  WakeupBMSIC();  // To wakeup IsoSPI from IDLE to active
  SPI_Transaction(tx_data, rx_data, Reg_Size);
  MXC_Delay(1000);  //ADBMS will take 3.5ms time to read config register //This dalay in not required for maxim microcontroller due to it's spi driver code. but it may be important for other microcontroller.
  #if Test_Print
  	  	 int rxdatabuff = WRCmdSize;
     	 printf("\nData Read From BMS IC\n");
     	 for(int IC=1; IC<=tIC; IC++)
     	 {
     		 printf("IC:0%x\t",IC);
     		 for (int i=0; i<(RDDataSize); i++)
     		 {
     			 printf("0x%x\t", rx_data[rxdatabuff]); // (inital 4 byte - invalid data - SPI Full Duplex)
     			 rxdatabuff++;
     		 }
     		 printf("\n");
     	 }
  #endif
  /* executes for each ic in the daisy chain and packs the data */
  for (uint8_t current_ic = 0; current_ic < tIC; current_ic++)
  {
    for (uint8_t data_byte = 0; data_byte < (RDDataSize-2); data_byte++)
     {
       data_Read[data_byte + (current_ic*(RDDataSize-2))] = rx_data[data_byte + WRCmdSize + (current_ic)*RDDataSize]; // Copy received Data only
     }
    for (uint8_t data_byte = 0; data_byte < (RDDataSize); data_byte++)
     {
       copyArray[data_byte + (current_ic*RDDataSize)] = rx_data[data_byte + WRCmdSize + (current_ic)*RDDataSize]; // Copy received Data + Data pec for data pec calc and Verification
     }

    uint16_t Received_Data_Pec, Cal_Data_Pec;
    int Reg_Curr_Size = WRCmdSize + (current_ic+1)*RDDataSize;
    #if Test_Print
        uint8_t Received_Command_Cntr;
        Received_Command_Cntr 	= rx_data[Reg_Curr_Size - 2U] >> 2U;
    #endif
    Received_Data_Pec 		= ((rx_data[( Reg_Curr_Size - 2U)] & 3U) << 8U) | (rx_data[( Reg_Curr_Size - 1U)]);
    Cal_Data_Pec				  =  pec10_calc(true, (RDDataSize-Data_Pec_Size), &copyArray[(current_ic*RDDataSize)]);
    #if Test_Print
        printf("Received_Command_Cntr=%x\t Received_Data_Pec=%x\t Cal_Data_Pec=%x\n", Received_Command_Cntr, Received_Data_Pec, Cal_Data_Pec);
    #endif
    if (Received_Data_Pec != Cal_Data_Pec)
    {
      printf("\nError in data Received From the ADBMS Board\n");
    }
  }
  free(copyArray);
  free(rx_data);
}

/*!
  @brief Precalculated CRC Table
*/
const uint16_t Crc15Table[256] =
{
  0x0000,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
  0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
  0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
  0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
  0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
  0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
  0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
  0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
  0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
  0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
  0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
  0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
  0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
  0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
  0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
  0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
  0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
  0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
  0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
  0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
  0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
  0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
  0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

/*!
  @brief CRC15 Pec Calculation Function
*/
uint16_t Pec15_Calc( uint8_t len, uint8_t *data)
{
  uint16_t remainder,addr;
  remainder = 16; /* initialize the PEC */
  for (uint8_t i = 0; i<len; i++) /* loops for each byte in data array */
  {
    addr = (((remainder>>7)^data[i])&0xff);/* calculate PEC table address */
    remainder = ((remainder<<8)^Crc15Table[addr]);
  }
  return(remainder*2);/* The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}

/*!
  @brief CRC10 Pec Calculation Function
*/
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data)
{
  uint16_t remainder = 16; /* PEC_SEED;   0000010000 */
  uint16_t polynom = 0x8F; /* x10 + x7 + x3 + x2 + x + 1 <- the CRC15 polynomial         100 1000 1111   48F */

  /* Perform modulo-2 division, a byte at a time. */
  for (uint8_t pbyte = 0; pbyte < len; ++pbyte)
  {
    /* Bring the next byte into the remainder. */
    remainder ^= (uint16_t)(data[pbyte] << 2);
    /* Perform modulo-2 division, a bit at a time.*/
    for (uint8_t bit_ = 8; bit_ > 0; --bit_)
    {
      /* Try to divide the current data bit. */
      if ((remainder & 0x200) > 0)//equivalent to remainder & 2^14 simply check for MSB
      {
        remainder = (uint16_t)((remainder << 1));
        remainder = (uint16_t)(remainder ^ polynom);
      }
      else
      {
        remainder = (uint16_t)(remainder << 1);
      }
    }
  }
  if (rx_cmd == true)
  {
    remainder ^= (uint16_t)((data[len] & 0xFC) << 2);
    /* Perform modulo-2 division, a bit at a time */
    for (uint8_t bit_ = 6; bit_ > 0; --bit_)
    {
      /* Try to divide the current data bit */
      if ((remainder & 0x200) > 0)//equivalent to remainder & 2^14 simply check for MSB
      {
        remainder = (uint16_t)((remainder << 1));
        remainder = (uint16_t)(remainder ^ polynom);
      }
      else
      {
        remainder = (uint16_t)((remainder << 1));
      }
    }
  }
  return ((uint16_t)(remainder & 0x3FF));
}

/*!
  @brief Combine Cell Voltage in 16 bit Hex format
*/
int16_t twos_complement_to_int(uint16_t value, uint8_t num_bits)
{
  int16_t result = value; //result is returned as-is if MSB is 0
  if (value & (1 <<(num_bits-1)))//if MSB is 1(negative)
  {
    result = value -(1 <<num_bits); //subtract the overflow from 2's complement value
  }
    return result;
}

/*!
  @brief Function: SetUnderVoltageThreshold fo ADBMS
*/
uint16_t SetUnderVoltageThreshold(double voltage)
{
  uint16_t vuv_value;
  uint8_t rbits = 12;
  voltage = (voltage - (double)1.5);
  voltage = voltage / ((double)16 * (double)0.000150);
  vuv_value = (uint16_t )(voltage + 2 * (1 << (rbits - 1)));
  vuv_value &= 0xFFF;
  return vuv_value;
}

/*!
  @brief Function: SetOverVoltageThreshold for ADBMS
*/
uint16_t SetOverVoltageThreshold(double voltage)
{
  uint16_t vov_value;
  uint8_t rbits = 12;
  voltage = (voltage - (double)1.5);
  voltage = voltage / ((double)16 * (double)0.000150);
  vov_value = (uint16_t )(voltage + 2 * (1 << (rbits - 1)));
  vov_value &= 0xFFF;
  return vov_value;
}

/*!
  @brief Write CFGB data
*/
void WRCFGB_data_Set(uint8_t tIC, uint8_t cmd_arg[], float UV_THSD, float OV_THSD, uint8_t *data_Write)
{
  uint16_t UV_data, OV_data;
  UV_data = SetUnderVoltageThreshold(UV_THSD);
  data_Write[0] = (uint8_t)(UV_data);
  data_Write[1] = (uint8_t)((UV_data | 0xFF)>>8);
  OV_data = SetOverVoltageThreshold(OV_THSD);
  data_Write[1] |= (uint8_t)(OV_data<<4);
  data_Write[2] = (uint8_t)(OV_data>>4);
  data_Write[3] = 0x00;
  data_Write[4] = 0x00;
  data_Write[5] = 0x00;
}


void SPI_Transaction(uint8_t *tx_dat, uint8_t *rx_dat, uint8_t length)
{
  mxc_spi_req_t req;
  req.spi = SPI;
  req.txData = tx_dat; 		//Tx_Buffer
  req.rxData = rx_dat; 		//Rx_Buffer
  req.txLen = length;
  req.rxLen = length;
  //IF USING MANUAL GPIO AS CS, then spi_pins.ss1 should be False which makes ssIdx as don't care
  req.ssIdx = 0; 			// Chip select index = 1 (if spi_pins.ss1 is TRUE;)
  req.txCnt = 0; 			// Initialize transmit counter to track no. of bytes transmitted
  req.rxCnt = 0; 			// Initialize receive counter to track no. of bytes received
  req.ssDeassert = 0; 		// If using manual GPIO as chip select, then put ssDessert as zero
  MXC_GPIO_OutClr(MAX_GPIO_P2_26_SPI0_SS1_PORT_IN, MAX_GPIO_P2_26_SPI0_SS1_PIN_IN);
  MXC_SPI_MasterTransaction(&req);
  MXC_GPIO_OutSet(MAX_GPIO_P2_26_SPI0_SS1_PORT_IN, MAX_GPIO_P2_26_SPI0_SS1_PIN_IN);
}

void GPIO_Init_BMS_Charger()
{
	/* User R_LED */
//	mxc_gpio_cfg_t 			  l_s_r_LED_config;
	l_s_r_LED_config.port	= MXC_GPIO0;
	l_s_r_LED_config.mask	= MXC_GPIO_PIN_14;
	l_s_r_LED_config.pad	= MXC_GPIO_PAD_NONE;
	l_s_r_LED_config.func	= MXC_GPIO_FUNC_OUT;
	l_s_r_LED_config.vssel	= MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_r_LED_config);

	/* User G_LED */
//	mxc_gpio_cfg_t 			  l_s_g_LED_config;
	l_s_g_LED_config.port	= MXC_GPIO0;
	l_s_g_LED_config.mask	= MXC_GPIO_PIN_23;
	l_s_g_LED_config.pad	= MXC_GPIO_PAD_NONE;
	l_s_g_LED_config.func	= MXC_GPIO_FUNC_OUT;
	l_s_g_LED_config.vssel	= MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_g_LED_config);


	/* User MCU_FET_EN */
//	mxc_gpio_cfg_t 			  l_s_fet_en_config;
	l_s_fet_en_config.port	= MXC_GPIO2;
	l_s_fet_en_config.mask	= MXC_GPIO_PIN_8;
	l_s_fet_en_config.pad	= MXC_GPIO_PAD_NONE;
	l_s_fet_en_config.func	= MXC_GPIO_FUNC_OUT;
	l_s_fet_en_config.vssel	= MXC_GPIO_VSSEL_VDDIOH;
	MXC_GPIO_Config(&l_s_fet_en_config);

	/* User PCHG_FET_EN */
//	mxc_gpio_cfg_t 			  l_s_fet_en_config;
	l_s_pchg_fet_config.port= MXC_GPIO1;
	l_s_pchg_fet_config.mask= MXC_GPIO_PIN_19;
	l_s_pchg_fet_config.pad	= MXC_GPIO_PAD_NONE;
	l_s_pchg_fet_config.func= MXC_GPIO_FUNC_OUT;
	l_s_pchg_fet_config.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_pchg_fet_config);

	/* User_BTN */
//	mxc_gpio_cfg_t 			  l_s_usr_btn_config;
	l_s_usr_btn_config.port	= MXC_GPIO4;
	l_s_usr_btn_config.mask	= MXC_GPIO_PIN_0;
	l_s_usr_btn_config.pad	= MXC_GPIO_PAD_PULL_UP;
	l_s_usr_btn_config.func	= MXC_GPIO_FUNC_IN;
	l_s_usr_btn_config.vssel= MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_usr_btn_config);

	/* SPI0_SS1  */
	l_s_spi_ss1_config.port	= MAX_GPIO_P2_26_SPI0_SS1_PORT_IN;
	l_s_spi_ss1_config.mask	= MAX_GPIO_P2_26_SPI0_SS1_PIN_IN;
	l_s_spi_ss1_config.pad	= MXC_GPIO_PAD_PULL_UP;
	l_s_spi_ss1_config.func	= MXC_GPIO_FUNC_OUT;
	l_s_spi_ss1_config.vssel= MXC_GPIO_VSSEL_VDDIOH;
	MXC_GPIO_Config(&l_s_spi_ss1_config);

	//Charger slave select
	l_s_spi_ss2_config.port	= MAX_GPIO_P1_18_SPI0_SS2_PORT_IN;
	l_s_spi_ss2_config.mask	= MAX_GPIO_P1_18_SPI0_SS2_PIN_IN;
	l_s_spi_ss2_config.pad	= MXC_GPIO_PAD_PULL_UP;
	l_s_spi_ss2_config.func	= MXC_GPIO_FUNC_OUT;
	l_s_spi_ss2_config.vssel= MXC_GPIO_VSSEL_VDDIOH;
	MXC_GPIO_Config(&l_s_spi_ss2_config);


#if MAX32690_UI_DEF_ZERO
	/* Prech_En */
	mxc_gpio_cfg_t 			  l_s_prch_en_config;
	l_s_prch_en_config.port	= MAX_GPIO_P1_19_PRECHARGE_EN_PORT_OUT;
	l_s_prch_en_config.mask	= MAX_GPIO_P1_19_PRECHARGE_EN_PIN_OUT;
	l_s_prch_en_config.pad	= MXC_GPIO_PAD_NONE;
	l_s_prch_en_config.func	= MXC_GPIO_FUNC_IN;
	l_s_prch_en_config.vssel= MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_prch_en_config);

	/* MCU Status */
	mxc_gpio_cfg_t 			  l_s_mcu_sts_config;
	l_s_mcu_sts_config.port	= MAX_GPIO_P1_7_MCU_STATUS_PORT_OUT;
	l_s_mcu_sts_config.mask	= MAX_GPIO_P1_7_MCU_STATUS_PIN_OUT;
	l_s_mcu_sts_config.pad	= MXC_GPIO_PAD_NONE;
	l_s_mcu_sts_config.func	= MXC_GPIO_FUNC_IN;
	l_s_mcu_sts_config.vssel= MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_mcu_sts_config);

	/* FaultB Pin */
	mxc_gpio_cfg_t 			  l_s_faultB_config;
	l_s_faultB_config.port	= MAX_GPIO_P1_14_FAULTB_AFE_PORT_IN;
	l_s_faultB_config.mask	= MAX_GPIO_P1_14_FAULTB_AFE_PIN_IN;
	l_s_faultB_config.pad	= MXC_GPIO_PAD_PULL_UP;
	l_s_faultB_config.func	= MXC_GPIO_FUNC_IN;
	l_s_faultB_config.vssel	= MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_faultB_config);

	/* KL_15 */
	mxc_gpio_cfg_t 			  l_s_kl15_config;
	l_s_kl15_config.port	= MAX_GPIO_P1_15_KL_15_MCU_PORT_IN;
	l_s_kl15_config.mask	= MAX_GPIO_P1_15_KL_15_MCU_PIN_IN;
	l_s_kl15_config.pad		= MXC_GPIO_PAD_PULL_UP;
	l_s_kl15_config.func	= MXC_GPIO_FUNC_IN;
	l_s_kl15_config.vssel	= MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&l_s_kl15_config);
#endif
//	MXC_GPIO_OutSet(MAX_GPIO_P1_18_SPI0_SS2_PORT_IN, MAX_GPIO_P1_18_SPI0_SS2_PIN_IN);
	printf("MAX32690_GPIO->\tMAX32690_GPIO_Init-> GPIO Init: Success\n");
}
/* 	ISO SPI will go to IDLE after 4.3ms. So, Wake isoSPI up from IDlE state and enters the READY state and
	BMS IC will go to sleep state after 2s. So, Wake BMS IC from Sleep state to Standby state					*/
int WakeupIC(mxc_spi_regs_t *spi)
{
	int TOTAL_IC = 1;
	for(int i=0; i<TOTAL_IC; i++)
	{
		/* Wake BMS IC from Sleep state to Standby state */
		spi->ctrl2 |= (MXC_S_SPI_CTRL2_SS_POL_SS0_HIGH);
		MXC_Delay(1000);
		spi->ctrl2 &= ~(MXC_S_SPI_CTRL2_SS_POL_SS0_HIGH);
		MXC_Delay(1000);
		/* Wake isoSPI up from IDlE state and enters the READY state */
		uint8_t a[1] = {0xff};
		SPI_Write(TOTAL_IC, &a[0]);	//Guarantees the isoSPI will be in ready state
		MXC_Delay(200);
	}
		return E_NO_ERROR;
}

void WakeupBMSIC(void)
{
	WakeupIC(SPI);
}


