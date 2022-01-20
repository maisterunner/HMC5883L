#ifndef HMC5883_H
#define HMC5883_H

#include "main.h"

// I2C Address

#define HMC_ADDR_W										0x3C
#define HMC_ADDR_R										0x3D


/* ----------------
*
*	Register map - address defines
*
 ----------------- */
 

// Configuration

#define HMC_CRA											0x00
#define HMC_CRB											0x01
#define HMC_MODE_REG									0x02

// Data output

#define HMC_DA_X_MSB									0x03
#define HMC_DA_X_LSB									0x04
#define HMC_DA_Z_MSB									0x05
#define HMC_DA_Z_LSB									0x06
#define HMC_DA_Y_MSB									0x07
#define HMC_DA_Y_LSB									0x08

// Status / ID

#define HMC_STATUS										0x09
#define HMC_ID_A										0x0A
#define HMC_ID_B										0x0B
#define HMC_ID_C										0x0C

/* ----------------
*
*	Settings defines
*
*   Only non-default settings are defined
*
 ----------------- */
 

// REG 0:	 		Configuration Register A
//			 		CRA (def 0x70)

// Reserved bit
#define HMC_CRA_CLEAR									0x00 << 7

// Measurement samples averaged
#define HMC_CRA_MEAS_AVG_1								0x00 << 5
#define HMC_CRA_MEAS_AVG_2								0x01 << 5
#define HMC_CRA_MEAS_AVG_4								0x02 << 5
#define HMC_CRA_MEAS_AVG_8								0x03 << 5

// Data output rate
#define HMC_CRA_DATA_OR_0_75							0x00 << 2
#define HMC_CRA_DATA_OR_1_5								0x01 << 2
#define HMC_CRA_DATA_OR_3								0x02 << 2
#define HMC_CRA_DATA_OR_7								0x03 << 2
#define HMC_CRA_DATA_OR_15								0x04 << 2
#define HMC_CRA_DATA_OR_30								0x05 << 2
#define HMC_CRA_DATA_OR_75								0x06 << 2

// Measurement mode
#define HMC_CRA_MEAS_MODE_NORM							0x00
#define HMC_CRA_MEAS_MODE_POS							0x01
#define HMC_CRA_MEAS_MODE_NEG							0x02


// REG 1:	 		Configuration Register B
//			 		CRB (def 0x20)

// Gain configuration bits
#define HMC_CRB_GAIN0									0x00 << 5
#define HMC_CRB_GAIN1									0x01 << 5
#define HMC_CRB_GAIN2									0x02 << 5
#define HMC_CRB_GAIN3									0x03 << 5
#define HMC_CRB_GAIN4									0x04 << 5
#define HMC_CRB_GAIN5									0x05 << 5
#define HMC_CRB_GAIN6									0x06 << 5
#define HMC_CRB_GAIN7									0x07 << 5
#define HMC_CRB_GAIN8									0x08 << 5


// REG 2:	 		Mode Register
//			 		MR (def 0x01)

// Operating mode
#define HMC_MR_CONT										0x00
#define HMC_MR_SINGLE									0x01
#define HMC_MR_IDLE										0x02

// Identification Register A
#define HMC_ID_A_VAL									0x48
// Identification Register B
#define HMC_ID_B_VAL									0x34
// Identification Register C
#define HMC_ID_C_VAL									0x33


/* ----------------
*
*
*
 ----------------- */

typedef struct{
		// Conditioning
	uint8_t id[3];
		// conditioned measurements
	int16_t mag_X;
	int16_t mag_Y;
	int16_t mag_Z;
		// float calc data
	float mag_Xf;
	float mag_Yf;
	float mag_Zf;
		// Interface
	I2C_HandleTypeDef *hi2c;
	GPIO_TypeDef *int_PinPort;
	uint16_t int_Pin;
		// raw data
	uint8_t raw[6];
		// I2C reading
	uint8_t reading_data;
	uint8_t data_ready;
	uint8_t meas_triggered;
		// data regs
	uint8_t reg_address[6];
} HMC_data;


/* ----------------
*
*	Function prototypes
*
 ----------------- */

uint8_t HMC_Init_Struct(HMC_data *input,
		I2C_HandleTypeDef *hi2c,
		GPIO_TypeDef *HMC_INT_Port, uint16_t HMC_IntPin);

uint8_t HMC_Read_ID(HMC_data *input);

uint8_t HMC_Config(HMC_data *input,
						uint8_t average, uint8_t data_or, uint8_t meas_mode,
						uint8_t gain, uint8_t mode);

void HMC_Read_Rdy(HMC_data *input);

uint8_t HMC_Trigger_Single_Meas(HMC_data *input);

uint8_t HMC_Read_Poll(HMC_data *input);

uint8_t HMC_SendAddr_DMA(HMC_data *input);

uint8_t HMC_ReadMeas_DMA(HMC_data *input);

void HMC_ReadCplt_DMA(HMC_data *input);


#endif
