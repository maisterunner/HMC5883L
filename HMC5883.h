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

// Gain cofiguration bits
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
*	Function prototypes
*
 ----------------- */

uint8_t HMC_Read_ID(I2C_HandleTypeDef *hi2c);
uint8_t HMC_Write_Config(I2C_HandleTypeDef *hi2c);
uint8_t HMC_Read_Out(I2C_HandleTypeDef *hi2c, uint16_t* result);





#endif
