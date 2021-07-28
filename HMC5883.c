#include "HMC5883.h"
#include "main.h"

/* --------

	Functions

-------- */

// Read HMC ID
uint8_t HMC_Read_ID(I2C_HandleTypeDef *hi2c){
	
	HAL_StatusTypeDef res;
	uint8_t buf[3];
	
	buf[0] = HMC_ID_A;
	buf[1] = HMC_ID_B;
	buf[2] = HMC_ID_C;

	for( uint8_t i=0; i <= 2; i++ ){

		res = HAL_I2C_Master_Transmit(hi2c, (uint16_t)HMC_ADDR_W, &buf[i], 1, HAL_MAX_DELAY); // write the address of the registers to be read
	
		if ( res != HAL_OK ){
			return 0;
		}
		else{
			res = HAL_I2C_Master_Receive(hi2c, (uint16_t)HMC_ADDR_R, &buf[i], 1, HAL_MAX_DELAY); // read the value of regs

			if ( res != HAL_OK ){
				return 0;
			}
		}
	}

	if ( buf[0] != HMC_ID_A_VAL || buf[1] != HMC_ID_B_VAL || buf[2] != HMC_ID_C_VAL ){	// check whether the id was received correctly
		return 0;
	}
	else{
		return 1;
	}
}

uint8_t HMC_Write_Config(I2C_HandleTypeDef *hi2c){
	
	HAL_StatusTypeDef res;
	uint8_t buf[4];
	
	// Config register A setting
	// No averaging (raw data)
	// 75Hz Data rate
	// Normal measuring mode
	buf[0] = HMC_CRA;
	buf[1] = HMC_CRA_CLEAR | HMC_CRA_MEAS_AVG_1 | HMC_CRA_DATA_OR_75 | HMC_CRA_MEAS_MODE_NORM;
	
	// Config register B setting
	// Left default (1.3 Gauss)
	
	// Mode register
	// Continous
	buf[2] = HMC_MODE_REG;
	buf[3] = HMC_MR_CONT;
	
	// Write configuration
	
	for( uint8_t i=0; i <= 2; i+=2 ){

		res = HAL_I2C_Master_Transmit(hi2c, (uint16_t)HMC_ADDR_W, &buf[i], 2, HAL_MAX_DELAY);
	
		if ( res != HAL_OK ){
			return 0;
		}
	}
	
	return 1;
	
}


uint8_t HMC_Read_Out(I2C_HandleTypeDef *hi2c, uint16_t* result){
	
	HAL_StatusTypeDef res;
	uint8_t buf[6];
	
	// write the address of the registers to be read
	res = HAL_I2C_Master_Transmit(hi2c, (uint16_t)HMC_ADDR_W, (uint8_t*)HMC_DA_X_MSB, 1, HAL_MAX_DELAY);
	
	if ( res != HAL_OK ){
		return 0;
	}
	else{
		// read the value of regs
		res = HAL_I2C_Master_Receive(hi2c, (uint16_t)HMC_ADDR_R, buf, 6, HAL_MAX_DELAY);
		if ( res != HAL_OK ){
			return 0;
		}
	}
	
	// connect msb and lsb bits
	for(uint8_t i = 0; i<=2; i++){
		result[i] = buf[2*i] << 8 | buf[2*i+1];
	}
	
	return 1;
}


