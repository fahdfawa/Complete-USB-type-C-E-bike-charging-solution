/****************************************************ABOUT E_BIKE_FUNCTIONS.C********************************************************/
// CREATED BY FAHAD AHAMMAD (AE at ADI) //


/****************************************************HEADER DECLARATIONS********************************************************/

#include "Charger_Functions.h"
#include "Charger_Fields.h"
#include "i2c.h"
#include "Charger_Registers.h"
#include "BMS.h"

/****************************************************INSTANCES********************************************************/
mxc_i2c_req_t reqMaster;
mxc_spi_req_t req;


/****************************************************VARIABLE DEFINES********************************************************/
#define DATA_LEN        100         // Words
/***** Globals *****/
uint8_t rx_data[DATA_LEN]={0};
uint8_t tx_data[DATA_LEN]={0};


const float Term_volt         = 58.8;       //in volt
const float Prechg_volt       = 35.0;
const float Tckle_volt        = 43.7;       //Programmable in a normal charger IC
const float CC_Curr           = 1.0;        //in amps
const float Prechg_Curr       = 0.05;
const float Tckle_Curr        = 0.30;       //can be 100mA, 200mA, 300mA, 400mA; default set @ 300mA
const float Del_volt_PC       = 0.5;        //need to change based on LTC7106 accuracy
const float Del_volt_TC       = 0.5;        //need to change based on LTC7106 accuracy
const float Del_volt_CC_CV    = 0.2;        //in volt
const float Chg_volt_high_lmt = 59.5;       //in volt
const float Chg_volt_low_lmt  = 40.0;       //in volt

float Chg_volt      = 25.0;             //in volt


int Bi = 0;
int Br = 0;
volatile int Chg_Det = 0;

uint8_t Vcell[2];
uint8_t Current[2];
uint8_t USB_STATUS_1_Read[1];
uint8_t CC_Istat[1];
uint8_t Voltage_Read;
uint8_t Current_Read;

uint8_t CC_STATUS0_return[1];
uint8_t UIC_Interrupt[1];
uint8_t CC_Interrupt[1];
uint8_t PD_Interrupt[1];
uint8_t Action_Interrupt[1];

const float Rsense = 0.01;
const float Voltage_factor = 78.125e-6;
const float Current_factor = (1.5625e-6)/Rsense;


/****************************************************FUNCTION DEFINITIONS********************************************************/


void Register_Read(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t* Read_value)
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x25;
	reqMaster.tx_buf = &reg;                                     //tx_buf is a pointer
	reqMaster.tx_len = sizeof(reg);
	reqMaster.rx_buf = Read_value;                               //
	reqMaster.rx_len = 1;                                        //rx_len !=0 fir read operation
	MXC_I2C_MasterTransaction(&reqMaster);

}


void Register_Write(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t Write_value0_LSB) //
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x25;
	uint8_t buf[2] = {reg, Write_value0_LSB}; // is write data is more than 16bit, add Write_value1_LSB, Write_value1_MSB and so on...
	reqMaster.tx_buf = buf;
	reqMaster.tx_len = sizeof(buf);
	reqMaster.rx_len = 0;                                        //rx_len = 0 for write operation
	MXC_I2C_MasterTransaction(&reqMaster);
}

void Register_Read_LTC7106(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t* Read_value)
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x2A;
	reqMaster.tx_buf = &reg;                                     //tx_buf is a pointer
	reqMaster.tx_len = sizeof(reg);
	reqMaster.rx_buf = Read_value;                               //
	reqMaster.rx_len = 1;                                        //rx_len !=0 fir read operation
	MXC_I2C_MasterTransaction(&reqMaster);

}

void Register_Write_LTC7106(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t Write_value0_LSB) //
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x2A;
	uint8_t buf[2] = {reg, Write_value0_LSB}; // is write data is more than 16bit, add Write_value1_LSB, Write_value1_MSB and so on...
	reqMaster.tx_buf = buf;
	reqMaster.tx_len = sizeof(buf);
	reqMaster.rx_len = 0;                                        //rx_len = 0 for write operation
	MXC_I2C_MasterTransaction(&reqMaster);
}


void Register_Read_FG(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t* Read_value)
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x36;
	reqMaster.tx_buf = &reg;                                     //tx_buf is a pointer
	reqMaster.tx_len = sizeof(reg);
	reqMaster.rx_buf = Read_value;                               //
	reqMaster.rx_len = 2;                                        //rx_len !=0 fir read operation
	MXC_I2C_MasterTransaction(&reqMaster);

}

void Register_Write_FG(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t Write_value0_LSB, uint8_t Write_value0_MSB) //
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x36;
	uint8_t buf[3] = {reg, Write_value0_LSB, Write_value0_MSB}; // is write data is more than 16bit, add Write_value1_LSB, Write_value1_MSB and so on...
	reqMaster.tx_buf = buf;
	reqMaster.tx_len = sizeof(buf);
	reqMaster.rx_len = 0;                                        //rx_len = 0 for write operation
	MXC_I2C_MasterTransaction(&reqMaster);
}


void Register_Multi_Read(mxc_i2c_regs_t* i2c_master, uint8_t start_reg, uint8_t num_regs, uint8_t* Read_values)
{

    for (uint8_t i = 0; i < num_regs; i++) {
        Register_Read(i2c_master, start_reg + i, &Read_values[i]);
    }
}

void Multi_Register_4byte_Write(mxc_i2c_regs_t* i2c_master, uint8_t start_reg, uint32_t Write_Value_32_bit){
	    uint8_t write_buff[4];

	    write_buff[0] = (uint8_t)(0xFF & Write_Value_32_bit);        // Least significant byte (LSB)
	    write_buff[1] = (uint8_t)(0xFF & Write_Value_32_bit >> 8);
	    write_buff[2] = (uint8_t)(0xFF & Write_Value_32_bit >> 16);
	    write_buff[3] = (uint8_t)(0xFF & Write_Value_32_bit >> 24);

	for (uint8_t i = 0; i < 4; i++) {
	     Register_Write(i2c_master, start_reg + i, write_buff[i]);
	}
}





void BC_CTRL1_Write(uint8_t DCDCpt, State Nikon_det, State CHGDetMan, State CHGDetEn, BC_CTRL_union *comb_bc_ctrl1 ){

	comb_bc_ctrl1->bc_ctrl1.CHGDetEn  = CHGDetEn;
	comb_bc_ctrl1->bc_ctrl1.CHGDetMan = CHGDetMan;
	comb_bc_ctrl1->bc_ctrl1.Nikon_Detection = Nikon_det;
	comb_bc_ctrl1->bc_ctrl1.DCDCpl   = DCDCpt;

	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, BC_CTRL1_CONFIG_WRITE);
	Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0, comb_bc_ctrl1->BC_CTRL1_combined);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
}


void BC_CTRL1_Read(BC_CTRL_union *comb_bc_ctrl1, uint8_t* BC_CTRL1_value){

	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, BC_CTRL1_CONFIG_WRITE);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);

	Register_Read(I2C_MASTER, COMMAND_READ_ADDRESS ,&comb_bc_ctrl1->BC_CTRL1_combined);
	*BC_CTRL1_value =  comb_bc_ctrl1->BC_CTRL1_combined;

}

void Source_Cap_Req(PDO_position Position,  Src_Cap_Req_union *comb_src_cap_req){

	 comb_src_cap_req->src_cap_req.Req_PDO_Pos = Position;
	 Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, SRC_CAP_REQ);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0, comb_src_cap_req->Src_Cap_Req_combined);
	 Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
}


void Current_Source_Cap(Current_Src_Cap_union *comb_current_src_cap, uint8_t *No_of_PDOs, uint8_t *Selected_Pos, uint32_t* SRC_PDO1, uint32_t* SRC_PDO2, uint32_t* SRC_PDO3, uint32_t* SRC_PDO4, uint32_t* SRC_PDO5, uint32_t* SRC_PDO6, uint32_t* SRC_PDO7, uint32_t* SRC_PDO8){

	uint8_t NO_OF_READ_ADDRESS = 36;
	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, CUR_SEL_SRC_CAP);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
	Register_Multi_Read(I2C_MASTER, COMMAND_READ_ADDRESS, NO_OF_READ_ADDRESS, comb_current_src_cap->Current_Src_Cap_combined);
	*No_of_PDOs = comb_current_src_cap->cur_src_cap.Conf & 0b00000111;
	*Selected_Pos = (comb_current_src_cap->cur_src_cap.Conf >> 3) & 0b00000111;
    *SRC_PDO1 = comb_current_src_cap->cur_src_cap.PDO1_1 | comb_current_src_cap->cur_src_cap.PDO1_2 << 8 | comb_current_src_cap->cur_src_cap.PDO1_3 << 16 | comb_current_src_cap->cur_src_cap.PDO1_4 << 24 ;
    *SRC_PDO2 = comb_current_src_cap->cur_src_cap.PDO2_1 | comb_current_src_cap->cur_src_cap.PDO2_2 << 8 | comb_current_src_cap->cur_src_cap.PDO2_3 << 16 | comb_current_src_cap->cur_src_cap.PDO2_4 << 24 ;
    *SRC_PDO3 = comb_current_src_cap->cur_src_cap.PDO3_1 | comb_current_src_cap->cur_src_cap.PDO3_2 << 8 | comb_current_src_cap->cur_src_cap.PDO3_3 << 16 | comb_current_src_cap->cur_src_cap.PDO3_4 << 24 ;
    *SRC_PDO4 = comb_current_src_cap->cur_src_cap.PDO4_1 | comb_current_src_cap->cur_src_cap.PDO4_2 << 8 | comb_current_src_cap->cur_src_cap.PDO4_3 << 16 | comb_current_src_cap->cur_src_cap.PDO4_4 << 24 ;
    *SRC_PDO5 = comb_current_src_cap->cur_src_cap.PDO5_1 | comb_current_src_cap->cur_src_cap.PDO5_2 << 8 | comb_current_src_cap->cur_src_cap.PDO5_3 << 16 | comb_current_src_cap->cur_src_cap.PDO5_4 << 24 ;
	*SRC_PDO6 = comb_current_src_cap->cur_src_cap.PDO6_1 | comb_current_src_cap->cur_src_cap.PDO6_2 << 8 | comb_current_src_cap->cur_src_cap.PDO6_3 << 16 | comb_current_src_cap->cur_src_cap.PDO6_4 << 24 ;
	*SRC_PDO7 = comb_current_src_cap->cur_src_cap.PDO7_1 | comb_current_src_cap->cur_src_cap.PDO7_2 << 8 | comb_current_src_cap->cur_src_cap.PDO7_3 << 16 | comb_current_src_cap->cur_src_cap.PDO7_4 << 24 ;
	*SRC_PDO8 = comb_current_src_cap->cur_src_cap.PDO8_1 | comb_current_src_cap->cur_src_cap.PDO8_2 << 8 | comb_current_src_cap->cur_src_cap.PDO8_3 << 16 | comb_current_src_cap->cur_src_cap.PDO8_4 << 24 ;

}

void Set_Sink_PDOs(Memory_Write mry_write, PDO_position No_of_PDOs, uint32_t Snk_PDO1, uint32_t Snk_PDO2, uint32_t Snk_PDO3, uint32_t Snk_PDO4, uint32_t Snk_PDO5, uint32_t Snk_PDO6){

	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, SNK_PDO_SET);
	uint8_t command_write_data_0 = 0b10000111 & ((mry_write << 7) | No_of_PDOs);
	Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0,command_write_data_0) ;
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_1,  Snk_PDO1);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_5,  Snk_PDO2);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_9,  Snk_PDO3);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_13, Snk_PDO4);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_17, Snk_PDO5);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_21, Snk_PDO6);

	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);

}

void Sink_PDO_Req(SNK_PDO_union *comb_snk_PDO_req, uint8_t* No_of_PDOs, uint32_t* SNK_PDO1, uint32_t* SNK_PDO2, uint32_t* SNK_PDO3, uint32_t* SNK_PDO4, uint32_t* SNK_PDO5, uint32_t* SNK_PDO6){

	uint8_t NO_OF_READ_ADDRESS = 36;
	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, SNK_PDO_REQUEST_READ);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
	Register_Multi_Read(I2C_MASTER, COMMAND_READ_ADDRESS, NO_OF_READ_ADDRESS, comb_snk_PDO_req->SNK_PDO_Combined);
    *No_of_PDOs = comb_snk_PDO_req->snk_pdo.No_of_PDOs;
	*SNK_PDO1 = comb_snk_PDO_req->snk_pdo.PDO1_1 | comb_snk_PDO_req->snk_pdo.PDO1_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO1_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO1_4 << 24 ;
	*SNK_PDO2 = comb_snk_PDO_req->snk_pdo.PDO2_1 | comb_snk_PDO_req->snk_pdo.PDO2_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO2_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO2_4 << 24 ;
	*SNK_PDO3 = comb_snk_PDO_req->snk_pdo.PDO3_1 | comb_snk_PDO_req->snk_pdo.PDO3_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO3_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO3_4 << 24 ;
	*SNK_PDO4 = comb_snk_PDO_req->snk_pdo.PDO4_1 | comb_snk_PDO_req->snk_pdo.PDO4_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO4_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO4_4 << 24 ;
	*SNK_PDO5 = comb_snk_PDO_req->snk_pdo.PDO5_1 | comb_snk_PDO_req->snk_pdo.PDO5_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO5_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO5_4 << 24 ;
	*SNK_PDO6 = comb_snk_PDO_req->snk_pdo.PDO6_1 | comb_snk_PDO_req->snk_pdo.PDO6_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO6_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO6_4 << 24 ;


}

//void Set_Interrupt_Mask(){
//  Register_Write(I2C_MASTER, UIC_INT_M, 0b10000000);
//}

void gpio_isr(void *cbdata)
{
	MXC_Delay(500000); //Delay for eliminating false interrupt trigger during type C connection

    Charger_Mode_Selection();  //Forward or Reverse OTG mode based on CCStat

    Read_and_Clear_Interrupts_PD_Negotiator();  //Clearing MAX77958 Interrupt will make INT pin de-asserted

}

void GPIO_Init_PD_Negotiator(){
    mxc_gpio_cfg_t gpio_interrupt;
	mxc_gpio_cfg_t gpio_interrupt_status;

	/* Setup interrupt status pin as an output so we can toggle it on each interrupt. */
	gpio_interrupt_status.port = LED_PORT;
	gpio_interrupt_status.mask = LED_PIN;
	gpio_interrupt_status.pad = MXC_GPIO_PAD_NONE;
	gpio_interrupt_status.func = MXC_GPIO_FUNC_OUT;
	gpio_interrupt_status.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_interrupt_status);
	/*
	 *   Set up interrupt the gpio.
	 *   Switch on EV kit is open when non-pressed, and grounded when pressed.
	 *   Use an internal pull-up so pin reads high when button is not pressed.
	 */

	gpio_interrupt.port = MAX77958_INT_PORT;
	gpio_interrupt.mask = MAX77958_INT_PIN;
	gpio_interrupt.pad = MXC_GPIO_PAD_PULL_UP;
	gpio_interrupt.func = MXC_GPIO_FUNC_IN;
	gpio_interrupt.vssel = MXC_GPIO_VSSEL_VDDIOH;
	MXC_GPIO_Config(&gpio_interrupt);
	MXC_GPIO_RegisterCallback(&gpio_interrupt, gpio_isr, &gpio_interrupt_status);
	MXC_GPIO_IntConfig(&gpio_interrupt, MXC_GPIO_INT_FALLING);
	MXC_GPIO_EnableInt(gpio_interrupt.port, gpio_interrupt.mask);
	NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MAX77958_INT_PORT)));


}


void VBUS_ADC_Voltage_print(uint8_t* Voltage_int){
	switch(*Voltage_int)
	{
	case 0x00: printf("VBUS: 0x00 = VBUS < 3.5V\n");
	break;

	case 0x01: printf("VBUS: 0x01 = 3.5V < VBUS < 4.5V\n");
	break;

	case 0x02: printf("VBUS: 0x02 = 4.5V < VBUS < 5.5V\n");
	break;

	case 0x03: printf("VBUS: 0x03 = 5.5V < VBUS < 6.5V\n");
	break;

	case 0x04: printf("VBUS: 0x04 = 6.5V < VBUS < 7.5V\n");
	break;

	case 0x05: printf("VBUS: 0x05 = 7.5V < VBUS < 8.5V\n");
	break;

	case 0x06: printf("VBUS: 0x06 = 8.5V < VBUS < 9.5V\n");
	break;

	case 0x07: printf("VBUS: 0x07 = 9.5V < VBUS < 10.5V\n");
	break;

	case 0x08: printf("VBUS: 0x08 = 10.5V < VBUS < 11.5V\n");
	break;

	case 0x09: printf("VBUS: 0x09 = 11.5V < VBUS < 12.5V\n");
	break;

	case 0x0A: printf("VBUS: 0x0A = 12.5V < VBUS < 13.5V\n");
	break;

	case 0x0B: printf("VBUS: 0x0B = 13.5V < VBUS < 14.5V\n");
	break;

	case 0x0C: printf("VBUS: 0x0C = 14.5V < VBUS < 15.5V\n");
	break;

	case 0x0D: printf("VBUS: 0x0D = 15.5V < VBUS < 16.5V\n");
	break;

	case 0x0E: printf("VBUS: 0x0E = 16.5V < VBUS < 17.5V\n");
	break;

	case 0x0F: printf("VBUS: 0x0F = 17.5V < VBUS < 18.5V\n");
	break;

	case 0x10: printf("VBUS: 0x10 = 18.5V < VBUS < 19.5V\n");
	break;

	case 0x11: printf("VBUS: 0x11 = 19.5V < VBUS < 20.5V\n");
	break;

	case 0x12: printf("VBUS: 0x12 = 19.5V < VBUS < 20.5V\n");
	break;

	case 0x13: printf("VBUS: 0x13 = 21.5V < VBUS < 22.5V\n");
	break;

	case 0x14: printf("VBUS: 0x14 = 22.5V < VBUS < 23.5V\n");
	break;

	case 0x15: printf("VBUS: 0x15 = 23.5V < VBUS < 24.5V\n");
	break;

	case 0x16: printf("VBUS: 0x16 = 24.5V < VBUS < 25.5V \n");
	break;

	case 0x17: printf("VBUS: 0x17 = 25.5V < VBUS < 26.5V\n");
	break;

	case 0x18: printf("VBUS: 0x18 = 26.5V < VBUS < 27.5V\n");
	break;

	case 0x19: printf("VBUS: 0x19 = 27.5V < VBUS\n");
	break;

	case 0x1A: printf("VBUS: 0x1A = Reserved\n");
	break;

	default: printf("Unknown VBUS");
	break;
	}
}

void Port_current_print(uint8_t* Current_Int){
	switch(*Current_Int){

	case 0x00: printf("CC detected current limit: Not in UFP mode\n");
	break;

	case 0x01: printf("CC detected current limit: 500mA\n");
	break;

	case 0x02: printf("CC detected current limit: 1500mA\n");
	break;

	case 0x03: printf("CC detected current limit: 3000mA\n");
	break;

	default: printf("Unknown");
	break;
	}
}

void PD_Message_Print(uint8_t* PD_Mssg){
	switch(*PD_Mssg)
	{
	case 0x00: printf("PD Message: Nothing happened\n");
	break;

	case 0x01: printf("PD Message: Sink_PD_PSRdy_Received\n");
	break;

	case 0x02: printf("PD Message: Sink_PD_Error_Recovery\n");
	break;

	case 0x03: printf("PD Message: Sink_PD_SenderResponseTimer_Timeout\n");
	break;

	case 0x04: printf("PD Message: Source_PSRdy_Sent\n");
	break;

	case 0x05: printf("PD Message: Source_PD_Error_Recovery\n");
	break;

	case 0x06: printf("PD Message: Source_PD_SenderResponseTimer_Timeout\n");
	break;

	case 0x07: printf("PD Message: PD_DR_Swap_Request_Received\n");
	break;

	case 0x08: printf("PD Message: PD_PR_Swap_Request_Received\n");
	break;

	case 0x09: printf("PD Message: PD_VCONN_Swap_Request_Received\n");
	break;

	case 0x0A: printf("PD Message: Received PD Message in illegal state\n");
	break;

	case 0x0B: printf("PD Message: Sink_PD_Evaluate_State, SrcCap_Received\n");
	break;

	case 0x11: printf("PD Message: VDM Attention Message Received\n");
	break;

	case 0x12: printf("PD Message: Reject_Received\n");
	break;

	case 0x13: printf("PD Message: Not_Supported_Received\n");
	break;

	case 0x14: printf("PD_PR_Swap_SNKTOSRC_Cleanup\n");
	break;

	case 0x15: printf("PD Message: PD_PR_Swap_SRCTOSNK_Cleanup\n");
	break;

	case 0x16: printf("PD Message: HardReset_Received\n");
	break;

	case 0x17: printf("PD Message: PD_PowerSupply_VbusEnable\n");
	break;

	case 0x18: printf("PD Message: PD_PowerSupply_VbusDisable\n");
	break;

	case 0x19: printf("PD Message: HardReset_Sent\n");
	break;

	case 0x1A: printf("PD Message: PD_PR_Swap_SRCTOSWAP\n");
	break;

	case 0x1B: printf("PD Message: PD_PR_Swap_SWAPTOSNK\n");
	break;

	case 0x1C: printf("PD Message: PD_PR_Swap_SNKTOSWAP\n");
	break;

	case 0x1D: printf("PD_PR_Swap_SWAPTOSRC\n");
	break;

	case 0x20: printf("PD Message: Sink_PD_Disabled\n");
	break;

	case 0x21: printf("PD Message: Source_PD_Disabled\n");
	break;

	case 0x30: printf("PD Message: Get_Source_Capabilities_Extended_Received\n");
	break;

	case 0x31: printf("PD Message: Get_Status_Received\n");
	break;

	case 0x32: printf("PD Message: Get_Battery_Cap_Received\n");
	break;

	case 0x33: printf("PD Message: Get_Battery_Status_Received\n");
	break;

	case 0x34: printf("PD Message: Get_Manufacturer_Info_Received\n");
	break;

	case 0x35: printf("PD Message: Source_Capabilities_Extended_Received\n");
	break;

	case 0x36: printf("PD Message: Status_Received\n");
	break;

	case 0x37: printf("PD Message: Battery_Capabilities_Received\n");
	break;

	case 0x38: printf("PD Message: Battery_Status_Received\n");
	break;

	case 0x39: printf("PD Message: Manufacturer_Info_Received\n");
	break;

	case 0x3A: printf("PD Message: Security_Request_Received\n");
	break;

	case 0x3B: printf("PD Message: Security_Response_Received\n");
	break;

	case 0x3C: printf("PD Message: Firmware_Update_Request_Received\n");
	break;

	case 0x3D: printf("PD Message: Firmware_Update_Response_Received\n");
	break;

	case 0x3E: printf("PD Message: Alert_Received\n");
	break;

	case 0x40: printf("PD Message: VDM_NAK_Received\n");
	break;

	case 0x41: printf("PD Message: VDM_BUSY_Received\n");
	break;

	case 0x42: printf("PD Message: VDM_ACK_Received\n");
	break;

	case 0x43: printf("PD Message: VDM_REQ_Received\n");
	break;

	case 0x63: printf("PD Message: DiscoverMode_Received\n");
	break;

	case 0x65: printf("PD Message: PD_Status_Received\n");
	break;

	default: printf("Unknown\n");
	break;

	}
}



void Port_Detection_Status_Voltage_current_Read(uint8_t* Voltage_Read, uint8_t* Current_Read, uint8_t* PD_Status0){
	uint8_t BC_STATUS_return[1];
	Port_type Charger_type;
	uint8_t CC_Status1_ret[1];
	uint8_t PD_Status0_ret[1];
	uint8_t PD_Status1_ret[1];


	Register_Read(I2C_MASTER, BC_STATUS, BC_STATUS_return);


	Charger_type = (0b00000011 & BC_STATUS_return[0]);

	switch(Charger_type){

	case Nothing_Attached:
		 printf("Port type: Nothing Attached\n");
		 Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		 Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		 *Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		 *Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
		 break;

	case SDP:
		 printf("Port type: SDP\n");
		 Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		 Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		 *Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		 *Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
		 break;

	case CDP:
		printf("Port type: CDP\n");
		Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		*Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		*Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
		break;

	case DCP:
		printf("Port type: DCP\n");
		Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		*Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		*Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
        break;

	default:
		printf("Port Error\n");
		break;
	}

	VBUS_ADC_Voltage_print(Voltage_Read);
	Port_current_print(Current_Read);

	uint8_t VBUS_det = 0b00000001 & (BC_STATUS_return[0] >> 7);

	switch(VBUS_det)
	{

	case 0x0: printf("VBUS < VBDET\n");
	break;

	case 0x1: printf("VBUS > VBDET\n");
    break;

	default: printf("Unknown");
	break;
	}

	uint8_t CC_pin_state_mac = 0b00000111 & (CC_Istat[0]);

	switch(CC_pin_state_mac)
	{
	case 0x00: printf("CC_pin_machine: No connection\n");
	break;

	case 0x01: printf("CC_pin_machine: SINK\n");
    break;

	case 0x02: printf("CC_pin_machine: SOURCE\n");
	break;

	case 0x03: printf("CC_pin_machine: Audio accessory\n");
	break;

	case 0x04: printf("CC_pin_machine: DebugSrc accessory\n");
	break;

	case 0x05: printf("CC_pin_machine: Error\n");
	break;

	case 0x06: printf("CC_pin_machine: Disabled\n");
	break;

	case 0x07: printf("CC_pin_machine: DebugSnk accessory\n");
	break;

	default: printf("Unknown\n");
    break;
	}

	uint8_t CC_pin_stat = 0b00000011 & (CC_Istat[0] >> 6);

	switch(CC_pin_stat)
	{
	case 0x00: printf("No determination\n");
	break;

	case 0x01: printf("Port orientation: CC1 Active\n");
	break;

	case 0x02: printf("Port orientation: CC2 Active\n");
	break;

	case 0x03: printf("Port orientation: RFU\n");
	break;

	default: printf("Unknown\n");
	break;
	}

	Register_Read(I2C_MASTER, CC_STATUS1, CC_Status1_ret);
	uint8_t VSAFE_OV = 0b00000001 & (CC_Status1_ret[0] >> 3);

	switch(VSAFE_OV)
	{
	case 0x00: printf("VBUS < VSAFE0V\n");
	break;

	case 0x01: printf("VBUS > VSAFE0V\n");
	break;

	default: printf("Unknown\n");
	break;
	}


	Register_Read(I2C_MASTER, PD_STATUS0, PD_Status0_ret);
    *PD_Status0 = PD_Status0_ret[0];
    PD_Message_Print(PD_Status0);


    Register_Read(I2C_MASTER, PD_STATUS1, PD_Status1_ret);
    uint8_t PSRDY = 0b00000001 & (PD_Status1_ret[0] >> 4);

    switch(PSRDY)
    {
    case 0x00: printf("Nothing happened\n");
    break;

    case 0x01: printf("PSRDY received\n");
    break;

    default: printf("Unknown\n");
    break;
    }

    uint8_t Power_Role = 0b00000001 & (PD_Status1_ret[0] >> 6);

    switch(Power_Role)
    {
    case 0x00: printf("Power role: Sink\n");
    break;

    case 0x01: printf("Power role: Source\n");
    break;

    default: printf("Unknown\n");
    break;
    }

    uint8_t Data_Role = 0b00000001 & (PD_Status1_ret[0] >> 7);

    switch(Data_Role)
    {
    case 0x00: printf("Data Role: UFP\n");
    break;

    case 0x01: printf("Data Role: DFP\n");
    break;

    default: printf("Unknown\n");
    break;
    }
}

void MAX77958_Configuration()  //Make sure VBUS of MAX77958 is powered, otherwise the code will get stuck here
{
	Set_Sink_PDOs(MTP_Write, Position5, 0x1401912c, 0x0002d12c, 0x0003c12c, 0x0004b12c, 0x0006412c, 0x00000000);
}

//void SPI_ReadWrite(uint8_t *tx_data, uint8_t *rx_data, uint8_t TotalNoOfBytes)
//{
//
//	// SPI Request to change register values
//	req.spi = SPI;
//	req.txData = (uint8_t*) tx_data;
//	req.rxData = (uint8_t*) rx_data;
//	req.txLen = TotalNoOfBytes;
//	req.rxLen = TotalNoOfBytes;
//	req.ssIdx = 2;
//	req.txCnt = 0;
//	req.rxCnt = 0;
//	req.ssDeassert = 1; //Chip select to high after transaction
//
//	MXC_SPI_MasterTransaction(&req); //Maxim inbuilt function for SPI Transfer
//
//}


void Write_Register_LT7871(uint8_t Addr, uint8_t Data)
{
  uint8_t PEC_Return;
  PEC_Return = Pec_Calculation(Addr, Data);
  printf("PEC_Return:  %x\n",PEC_Return);
  uint8_t Byte_Count = 3;
  tx_data[0] = 0xFF & ((Addr << 1) | 0x00);
  tx_data[1] = 0xFF & Data;
  tx_data[2] = 0xFF & PEC_Return;

	req.spi = SPI;
	req.txData = (uint8_t*) tx_data;
	req.rxData = (uint8_t*) rx_data;
	req.txLen = Byte_Count;
	req.rxLen = Byte_Count;
	req.ssIdx = 0;
	req.txCnt = 0;
	req.rxCnt = 0;
	req.ssDeassert = 0; //Set Deassert to Zero for Manual CS control
	MXC_GPIO_OutClr(MAX_GPIO_P1_18_SPI0_SS2_PORT_IN, MAX_GPIO_P1_18_SPI0_SS2_PIN_IN);
	MXC_SPI_MasterTransaction(&req); //Maxim inbuilt function for SPI Transfer
	MXC_GPIO_OutSet(MAX_GPIO_P1_18_SPI0_SS2_PORT_IN, MAX_GPIO_P1_18_SPI0_SS2_PIN_IN);
}

uint8_t Read_Register_LT7871(uint8_t Addr)
{
  uint32_t Return_value = 0;
  uint8_t Byte_Count = 2;
  tx_data[0] = 0xff & ((Addr << 1) | 0x01);
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;

    req.spi = SPI;
  	req.txData = (uint8_t*) tx_data;
  	req.rxData = (uint8_t*) rx_data;
  	req.txLen = Byte_Count;
  	req.rxLen = Byte_Count;
  	req.ssIdx = 0;
  	req.txCnt = 0;
  	req.rxCnt = 0;
  	req.ssDeassert = 0; //Chip select to high after transaction

  	MXC_GPIO_OutClr(MAX_GPIO_P1_18_SPI0_SS2_PORT_IN, MAX_GPIO_P1_18_SPI0_SS2_PIN_IN);
  	MXC_SPI_MasterTransaction(&req); //Maxim inbuilt function for SPI Transfer
  	MXC_GPIO_OutSet(MAX_GPIO_P1_18_SPI0_SS2_PORT_IN, MAX_GPIO_P1_18_SPI0_SS2_PIN_IN);

  	return Return_value =  (0xFF & rx_data[1]);
}

uint8_t get_bit(uint16_t num, int bit_position) {
    return (num >> bit_position) & 0x01;
}


uint8_t Pec_Calculation(uint8_t addr, uint8_t data)
{
	uint8_t PEC_Array[8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};  //0x41
	uint8_t IN[3] = {0x00,0x00,0x00};

    uint16_t addr_data = 0xFFFF & (data | (addr << 9));
    printf("addr_data: %x\n",addr_data);
	for (int i = 0; i < 16; i++)
	{
	  uint8_t DATA[16];
	  DATA[i] = 0xFF & get_bit(addr_data,(15 - i));

	  IN[0] =  0xFF & (DATA[i] ^ PEC_Array[7]);
	  IN[1] =  0xFF & (PEC_Array[0] ^ IN[0]);
	  IN[2] =  0xFF & (PEC_Array[1] ^ IN[0]);

      PEC_Array[7] = PEC_Array[6];
      PEC_Array[6] = PEC_Array[5];
      PEC_Array[5] = PEC_Array[4];
      PEC_Array[4] = PEC_Array[3];
      PEC_Array[3] = PEC_Array[2];
	  PEC_Array[2] = IN[2];
	  PEC_Array[1] = IN[1];
	  PEC_Array[0] = IN[0];

	}
	uint8_t PEC_Return = 0xFF & (PEC_Array[0] | PEC_Array[1] << 1 | PEC_Array[2] << 2 | PEC_Array[3] << 3 | PEC_Array[4] << 4 | PEC_Array[5] << 5 | PEC_Array[6] << 6 | PEC_Array[7] << 7);
	return  PEC_Return;
}

void Set_Output_Voltage_LTC7871(Out_Side side, float Out_Volt_in_V)
{

	uint8_t DAC_Code = 0x00;
	float Rfb1 = 0.0;
	float Rfb2 = 0.0;

	switch(side)
	{
	case LOW: 	Rfb1 = 107.0;
	            Rfb2 = 10.0;
	break;

	case HIGH:   Rfb1 = 184.5;
	             Rfb2 = 4.64;
	break;

	default: printf("side unknown\n");
	break;

	}

	//Calculating DAC current
	float DAC_curr = (1200.0*((Rfb1+Rfb2)/(Rfb1*Rfb2)) - (1000.0*Out_Volt_in_V/Rfb1));

	//Scaling DAC current
	float DAC_curr_scaled = DAC_curr;
	int DAC_curr_rounded = (roundToInteger(DAC_curr_scaled));  //1000 for converting to uA

	//Adding Protection
	if (DAC_curr_rounded > 63)
	{
		DAC_curr_rounded = 63;
	}
	else if(DAC_curr_rounded < -64)
	{
		DAC_curr_rounded = -64;
	}

	//Converting to 2's complement
	if (DAC_curr > 0)
	{
	    DAC_Code = DAC_curr_rounded;
	}
	else
	{
        DAC_Code = (DAC_curr_rounded+128);
	}

//	printf("DAC_curr = %f\n",(double)DAC_curr);

	printf("DAC_Code: %x\n",DAC_Code);



	switch(side)
	{
	case LOW: 	Write_Register_LT7871(MFR_IDAC_VLOW, DAC_Code);
	break;

	case HIGH:   Write_Register_LT7871(MFR_IDAC_VHIGH, DAC_Code);
	break;

	default: printf("side unknown\n");
	break;

	}
}


//void Set_Charging_Current()
//{
//   switch (side)
//   {
//    case LOW: Write_Register_LT7871(MFR_IDAC_SETCUR, Data);
//    break;
//
//
//    case HIGH: Write_Register_LT7871(MFR_IDAC_SETCUR, Data);
//    break;
//
//    default: printf("side unknown\n");
//    break;
//   }
//}






void CC_CV_Charging(CC_CV_state chg_state)
{

	if (chg_state == CC_CV_ON)
    {
		float Pack_volt, Pack_curr;
        Read_Battery_Voltage_Current(&Pack_volt, &Pack_curr);


		// Pre-charge Mode
		if (Pack_volt <= Prechg_volt)
		{
			if(Pack_curr <= Prechg_Curr)
			{
			  Chg_volt  = Chg_volt + Del_volt_PC;
			}
		    else if(Pack_curr > Prechg_Curr)
		 	{
			  Chg_volt = Chg_volt - Del_volt_PC;
			}
		}

		// Trickle Charge Mode
		else if (Prechg_volt < Pack_volt && Pack_volt<= Tckle_volt)
		{
			if(Pack_curr <= Tckle_Curr)
			{
			  Chg_volt  = Chg_volt + Del_volt_TC;
			}
		    else if(Pack_curr > Tckle_Curr)
		 	{
			  Chg_volt = Chg_volt - Del_volt_TC;
			}
		}

        // CC/CV Mode
		else if (Tckle_volt < Pack_volt && Pack_volt <= Term_volt)
    	{
    	  if(Pack_curr <= CC_Curr)
    	    {
    		  Chg_volt  = Chg_volt + Del_volt_CC_CV;
    	    }
    	  else if(Pack_curr > CC_Curr)
		    {
    		  Chg_volt = Chg_volt - Del_volt_CC_CV;
		    }

    	  if(Chg_volt > Chg_volt_high_lmt)
			{
    		  Chg_volt = Chg_volt_high_lmt;
			}

    	  if(Chg_volt < Chg_volt_low_lmt)
			{
    		  Chg_volt = Chg_volt_low_lmt;
			}
        }
    	else if(Term_volt < Pack_volt)
    	{
    		Chg_volt = Chg_volt - Del_volt_CC_CV;
    	}

    	printf("Charging_Voltage: %f\n",(double)Chg_volt);

    	Set_Output_Voltage_LTC7871(HIGH, Chg_volt);

    }
	else if(chg_state == CC_CV_OFF)
	{
		printf("CC_CV_Charging off\n");
	}
	MXC_Delay(600000);  //Delay needs to be optimized with Del_volt
}

void Mask_Interrupts_PD_Negotiator()
{
	Register_Write(I2C_MASTER, UIC_INT_M, 0x84);  //AP command and Stop Mode interrupt masked
	Register_Write(I2C_MASTER, CC_INT_M, 0x00);
	Register_Write(I2C_MASTER, PD_INT_M, 0x00);
	Register_Write(I2C_MASTER, ACTION_INT_M, 0x00);

}

void Read_and_Clear_Interrupts_PD_Negotiator()
{
	Register_Read(I2C_MASTER, UIC_INT, UIC_Interrupt);
	Register_Read(I2C_MASTER, CC_INT, CC_Interrupt);
	Register_Read(I2C_MASTER, PD_INT, PD_Interrupt);
	Register_Read(I2C_MASTER, ACTION_INT, Action_Interrupt);

//	printf("UIC_Interrupt: %x\n",UIC_Interrupt[0]);
//	printf("CC_Interrupt: %x\n",CC_Interrupt[0]);
//	printf("PD_Interrupt: %x\n",PD_Interrupt[0]);
//	printf("Action_Interrupt: %x\n",Action_Interrupt[0]);

}



void Charger_Mode_Selection()
{
	Register_Read(I2C_MASTER, CC_STATUS0, CC_STATUS0_return);
	if((CC_STATUS0_return[0] & 0b00000111) == 0b00000001)     //Source connected
	{

	    //MAX77986_Forward_Charging_ON(Termination_Voltage, Input_curr_lim, Fast_chg_curr);

	    Chg_Det = 1;  //go inside while loop and do led blinking
	    printf("Source Connected\n");
	}
	else if((CC_STATUS0_return[0] & 0b00000111) == 0b00000010)  //Sink device connected
	{
		//Forward_Chg_OFF_Reverse_OTG_OFF_Boost_on(5000, 3000);
		//Enable_Reverse_OTG(5000,3000); //VBYP:  5V; Current_limit: 3A
		printf("Sink Connected\n");
		Chg_Det = 0;  //stop LED blinking when the charger is unplugged
	}
	else if((CC_STATUS0_return[0] & 0b00000111) == 0b00000000)  //Nothing Attached
	{
		//MAX77986_ALL_MODES_OFF();  //un-comment this line and comment the next line for getting the full power output when a sink device is connected at type C receptacle
		//Forward_Chg_OFF_Reverse_OTG_OFF_Boost_on(5000, 3000); //charger and Reverse OTG mode
		printf("Nothing attached\n");
		Chg_Det = 0;  //stop LED blinking when the charger is unplugged
	}
}
double roundToInteger(double value) {
    // Round to the nearest integer
    double roundedValue = round(value);
    return roundedValue;
}

uint16_t Twos_Comp_to_unsigned_value(int16_t twosCompValue)
{
	uint16_t unsigned_value = 0;
	if ((twosCompValue & 0x8000) != 0) {
        unsigned_value = ~twosCompValue + 1;
    }
	else if((twosCompValue & 0x8000) == 0){
		unsigned_value = twosCompValue;
	}
	return unsigned_value;
}




