

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_pins.h"
#include "spi.h"
#include "i2c.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "uart.h"
#include "board.h"
#include "BMS.h"
#include "max32690.h"
#include "Charger_Functions.h"
#include "led.h"


//#define I2C_SLAVE MXC_I2C1

#define I2C_FREQ 100000
#define I2C_BYTES 100



/***** Globals *****/

//static uint8_t txdata[I2C_BYTES];
//static uint8_t rxdata[I2C_BYTES];
uint8_t tx_buffer[50];
volatile uint8_t DMA_FLAG = 0;
volatile int I2C_FLAG;
volatile int txnum = 0;
volatile int txcnt = 0;
volatile int rxnum = 0;
volatile int num;

float Batt_Voltage;
float Batt_Current;

//Ideally inside a function
uint8_t CC_CV = 0;

state chg_state = BAT_DET;

uint8_t Bat_Det = 0;

float Batt_Term_Volt = 58.8;
float Chg_Volt;



int main(void)
{


	Current_Src_Cap_union curr_src_cap_combined;


	//MXC_GPIO_OutSet(MXC_GPIO2, MXC_GPIO_PIN_8);  //for turning on the FETs at the battery side
	int error = 0;

	error = MXC_I2C_Init(I2C_MASTER, 1, 0);

	if (error != E_NO_ERROR) {
		printf("-->Failed master\n");
		return error;
	} else {
		printf("\n-->I2C Master Initialization Complete\n");
	}

	NVIC_EnableIRQ(I2C2_IRQn);

	MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

	/* ADBMS6948*/ //Don't change the below code sequence

	GPIO_Init_BMS_Charger();

	SPI_Initialization();  //BMS and Charger

	ADBMS_Init(TOTAL_IC);

	ADSV_PWM_Discharge_Measurement(TOTAL_IC, 63); //Timer value should not be greater than 63

	printf("BMS Initialization Done\n");

//	Set_Output_Voltage_LTC7871(HIGH, Batt_Term_Volt);
//
//	printf("Output Voltage Setting Done\n");





//  	 Set_Output_Voltage_LTC7871(HIGH, 37);
//  	 Write_Register_LT7871(MFR_IDAC_SETCUR , Data)
//       printf("Output voltage setted\n");


	/*MAX77958*/
	GPIO_Init_PD_Negotiator();

	MAX77958_Configuration();

    Mask_Interrupts_PD_Negotiator();

    Read_and_Clear_Interrupts_PD_Negotiator();

    printf("MAX77958 Initialization Done\n");

	// Reading Source Capabilities of the charger
	uint32_t src_PDO1, src_PDO2, src_PDO3, src_PDO4, src_PDO5, src_PDO6, src_PDO7, src_PDO8;
	uint8_t Number_of_PDOs;
	uint8_t Selected_PDO;

	Current_Source_Cap(&curr_src_cap_combined, &Number_of_PDOs, &Selected_PDO, &src_PDO1, &src_PDO2, &src_PDO3, &src_PDO4, &src_PDO5, &src_PDO6, &src_PDO7, &src_PDO8);

	printf("Selected Source PDOs: %X\n", Selected_PDO);
	printf("Source_PDO1: %X\n", src_PDO1);
	printf("Source_PDO2:%X\n", src_PDO2);
	printf("Source_PDO3:%X\n", src_PDO3);
	printf("Source_PDO4:%X\n", src_PDO4);
	printf("Source_PDO5:%X\n", src_PDO5);
	printf("Source_PDO6:%X\n", src_PDO6);
	printf("Source_PDO7:%X\n", src_PDO7);
	printf("Source_PDO8:%X\n", src_PDO8);




	MXC_GPIO_OutSet(MXC_GPIO2, MXC_GPIO_PIN_8);


	while(1)
		 {

		   //MXC_Delay(5000000);



		   switch(chg_state)
		   {
	       case BAT_DET: Read_Battery_Voltage_Current(&Batt_Voltage, &Batt_Current);
					     printf("Batt_Voltage:  %f\n",Batt_Voltage);
					     printf("Batt_Current:  %f\n",Batt_Current);
					     Bat_Det = Detect_Battery(Batt_Voltage);
			             if (Bat_Det == 1)
							 {
			            	  printf("Battery Detected\n");
			            	  chg_state = CHG_DET;
							 }
			             else{
			            	  printf("Battery Detection Failed\n");
			                 }
		   break;

		   case CHG_DET: if(Chg_Det == 1)
		                 {
			               Read_Battery_Voltage_Current(&Batt_Voltage, &Batt_Current);
			               printf("Charger Detected\n");
			               chg_state = SET_BB_OUT;
		                 }
		                 else
		                 {
		                	 //MXC_Delay(1000000);
		                   printf("Charger De-attached\n");
		                 }
		   break;


		   case SET_BB_OUT: Set_Output_Voltage_LTC7871(HIGH, Batt_Voltage+0.4);
                          Chg_Volt = Batt_Voltage;
		                    chg_state = SWITCH_EN;
		                    chg_state = SWITCH_EN;


		   break;

		   case SWITCH_EN:  MXC_GPIO_OutSet(MXC_GPIO2, MXC_GPIO_PIN_8);
		                    //chg_state = CC_CV_CHG;
		                    chg_state = CHG_STOP;
		   break;

		   case CC_CV_CHG:  CC_CV_Charging(CC_CV_ON);

		                    if (Chg_Det == 0)//check charger is removed?? then turn off the switch and goto BAT_DET
		                    {
		                    	chg_state = CHG_STOP;
		                    }
		                    chg_state = CC_CV_CHG;
		   break;

		   case CHG_STOP: //Reduce the charging current and then FET OFF,
			   // MXC_GPIO_OutSet(MXC_GPIO2, MXC_GPIO_PIN_8); //Batter
//			    chg_state = CHG_STOP;
			    if (Chg_Det == 0)
			    {
			    	printf("Turn off\n");
			    	MXC_GPIO_OutClr(MXC_GPIO2, MXC_GPIO_PIN_8);
			    	chg_state = BAT_DET;

			    }
			    else
			    {
			    	printf("Turn on\n");
			    	chg_state = CHG_STOP;
			    }


           break;

		   default: printf("Unknown state\n");
		            MXC_GPIO_OutClr(MXC_GPIO2, MXC_GPIO_PIN_8);
		   break;
		   }

	 }
    return 0;
}


