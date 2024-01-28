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
 * ----------------- */
 

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


// CORTEX-M - DWT TIMER
#define  ARM_CM_DEMCR      								(*(uint32_t *)0xE000EDFC)
#define  ARM_CM_DWT_CTRL   								(*(uint32_t *)0xE0001000)
#define  ARM_CM_DWT_CYCCNT 								(*(uint32_t *)0xE0001004)



/* ----------------
 *
 *	Structures
 *
 * ----------------- */

enum HMC_state{
	HMC_INIT,
	HMC_IDLE,
	HMC_TRIGGERED,
	HMC_ADDR_SENT,
	HMC_DATA_RDY,
	HMC_READING,
	HMC_READ_DONE,
	HMC_TIMEOUT,
	HMC_RESET
};


enum HMC_ret{
	HMC_OK,
	HMC_ID_ERR,
	HMC_TX_ERR,
	HMC_TX_DMA_ERR,
	HMC_RX_ERR,
	HMC_RX_DMA_ERR,
	HMC_ERR_TIMEOUT,
	HMC_ERR_DMA_TIMEOUT
};


enum HMC_bool{
	HMC_False,
	HMC_True
};


enum HMC_mode{
	HMC_Polling,
	HMC_OneShot,
	HMC_Cont
};


typedef union{
	uint8_t raw[6];
	struct{
		int16_t X;
		int16_t Y;
		int16_t Z;
	} meas;
} meas_raw;


typedef struct{
	float X;
	float Y;
	float Z;
} meas_f;


typedef struct{
	float xOffs;
	float yOffs;
} offset;


typedef struct{
	uint32_t currTicks;
	uint32_t elapsed;
	uint32_t systemClock;
	enum HMC_state prevState;
	uint32_t cycleTicks;
} timer;


typedef struct{
		// Conditioning
	uint8_t id[3];
		// conditioned measurements
	meas_raw measurement;
	meas_f measFloat;
	float sensitivity;
	offset Offs;
		// Clocking, mode
	timer TimeElapsed;
	enum HMC_mode mode;
		// Interface
	I2C_HandleTypeDef *hi2c;
	GPIO_TypeDef *int_PinPort;
	uint16_t int_Pin;
		// I2C reading
	enum HMC_state state;
		// data regs
	uint8_t reg_address[6];
} HMC_data;


/*
 * Struct in use
 */
static HMC_data HMC;


/* ----------------
 *
 *	Function prototypes
 *
 * ----------------- */

enum HMC_ret HMC_Init_Struct(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *HMC_INT_Port, uint16_t HMC_IntPin);
enum HMC_ret HMC_Read_ID(void);
void HMC_IntPinConfig(void);
void HMC_SetSensitivity(uint8_t gain);
enum HMC_ret HMC_Config_Cont(void);
enum HMC_ret HMC_Config_Single(void);
void HMC_Read_Rdy(void);
void HMC_DmaTxCpltHandler(I2C_HandleTypeDef *hi2c);
void HMC_DmaRxCpltHandler(I2C_HandleTypeDef *hi2c);
void HMC_DataRdyPinHandler(void);
enum HMC_ret HMC_Trigger_Single_Meas(void);
enum HMC_ret HMC_Read_Poll(void);
enum HMC_ret HMC_SendAddr_DMA(void);
enum HMC_ret HMC_ReadMeas_DMA(void);
void HMC_ReadCplt_DMA(void);
void HMC_StateMachine(void);
void HMC_Int(void);
enum HMC_bool HMC_CheckTimeOut(void);
void HMC_ResetTimer(void);
void HMC_SetCycleTicks(uint8_t meas_mode, uint8_t op_mode, uint8_t data_or);


#endif
