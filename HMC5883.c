#include "HMC5883.h"
#include "main.h"

/* --------

	Functions

-------- */


/* ------- SETUP ------- */

uint8_t HMC_Init_Struct(HMC_data *input,
		I2C_HandleTypeDef *hi2c,
		GPIO_TypeDef *HMC_INT_Port, uint16_t HMC_IntPin){

	/*
	*
	*	Initialize data struct
	*
	*/
	
	// Store ID
	input->id[0] = HMC_ID_A_VAL;
	input->id[1] = HMC_ID_B_VAL;
	input->id[3] = HMC_ID_C_VAL;
	
	// Zero data
	input->mag_X = 0;
	input->mag_Y = 0;
	input->mag_Z = 0;
	
	input->mag_Xf = 0;
	input->mag_Yf = 0;
	input->mag_Zf = 0;
	
	// Interface parameters
	input->hi2c = hi2c;
	input->int_PinPort = HMC_INT_Port;
	input->int_Pin = HMC_IntPin;
	
	// Busy flag
	input->reading_data = 0;
	input->data_ready = 0;
	
	// Data regs
	input->reg_address[0] 	= 		HMC_DA_X_MSB;
	input->reg_address[1] 	= 		HMC_DA_X_LSB;
	input->reg_address[2] 	= 		HMC_DA_Z_MSB;
	input->reg_address[3] 	= 		HMC_DA_Z_LSB;
	input->reg_address[4] 	= 		HMC_DA_Y_MSB;
	input->reg_address[5] 	= 		HMC_DA_Y_LSB;
	
	// Read id and check if it is received correcly
	if( HMC_Read_ID(input) ){
		return 1;
	}
	else{
		return 0;
	}
}


uint8_t HMC_Read_ID(HMC_data *input){
	
	/*
	*
	*	Read ID
	*
	*/
	
	// Init vars
	HAL_StatusTypeDef res;
	uint8_t buf[3];
	
	// Fill buff 
	buf[0] = HMC_ID_A;
	buf[1] = HMC_ID_B;
	buf[2] = HMC_ID_C;

	// Send addr, and receive id-s
	for( uint8_t i=0; i <= 2; i++ ){
		
		// write the address of the registers to be read
		res = HAL_I2C_Master_Transmit(input->hi2c, (uint16_t)HMC_ADDR_W, &buf[i], 1, HAL_MAX_DELAY); 
	
		// check if comm is ok
		if ( res != HAL_OK ){
			return 1;
		}
		else{
			// read the value of regs
			res = HAL_I2C_Master_Receive(input->hi2c, (uint16_t)HMC_ADDR_R, &buf[i], 1, HAL_MAX_DELAY);

			// check if comm is ok
			if ( res != HAL_OK ){
				return 2;
			}
		}
	}

	// check whether the id was received correctly
	if ( buf[0] != HMC_ID_A_VAL || buf[1] != HMC_ID_B_VAL || buf[2] != HMC_ID_C_VAL ){	
		return 3;
	}
	else{
		return 0;
	}
}



uint8_t HMC_Config(HMC_data *input,
						uint8_t average, uint8_t data_or, uint8_t meas_mode,	// config reg A
						uint8_t gain, uint8_t mode){							// config reg B
	
	/*
	* 
	* Setup recommendation:
	*
	* Config reg A:
	* 	- No averaging / HMC_CRA_MEAS_AVG_1 
	*	- 75 Hz data rate / HMC_CRA_DATA_OR_75
	*	- Normal meas mode / HMC_CRA_MEAS_MODE_NORM
	*
	* Config reg B:
	*	- Gain 1 (1.3 Gauss) / HMC_CRB_GAIN1
	*	- Continous / HMC_MR_CONT
	* 
	*	- Single shot alternatively (max possible DR) / HMC_MR_SINGLE
	*
	*/
	
	// Init vars
	HAL_StatusTypeDef res;
	uint8_t buf[4];
	
	// Config register A setting
	buf[0] = HMC_CRA;
	buf[1] = HMC_CRA_CLEAR | average | data_or | meas_mode;
	
	// Config register B setting
	buf[2] = HMC_MODE_REG;
	buf[3] = gain | mode;
	
	// Write configuration
	for( uint8_t i=0; i <= 2; i+=2 ){

		// Comm
		res = HAL_I2C_Master_Transmit(input->hi2c, (uint16_t)HMC_ADDR_W, &buf[i], 2, HAL_MAX_DELAY);
	
		// Comm check
		if ( res != HAL_OK ){
			return 1;
		}
	}
	
	// All setup OK
	return 0;
	
}

/* ------- OPERATION ------- */

void HMC_Read_Rdy(HMC_data *input){
	
	/*
	*
	*	Set read ready flag
	*
	*/
	
	// Set flag
	input->data_ready = 1;
	input->meas_triggered = 0;

}


uint8_t HMC_Trigger_Single_Meas(HMC_data *input){
	
	/*
	*
	*	Trigger single measurement
	*
	*	Used with single shot mode
	*	Mode is set to single shot
	*	When conversion is ready mode resets to idle
	*
	*/
	
	// Init vars
	HAL_StatusTypeDef res;
	uint8_t buf[2];
	
	// Fill buffer for triggering	
	buf[0] = HMC_MODE_REG;
	buf[1] = HMC_MR_SINGLE;
	
	// Comm
	res = HAL_I2C_Master_Transmit_DMA(input->hi2c, (uint16_t)HMC_ADDR_W, buf, 2);
	
	// Check comm
	if ( res != HAL_OK ){
		return 1;
	}
	
	// Set flag for succesful trigger
	input->meas_triggered = 1;
	return 0;
}


uint8_t HMC_Read_Poll(HMC_data *input){
	
	/*
	*
	*	Read measurement in polling mode
	*
	*/
	
	// Init vars
	HAL_StatusTypeDef res;
	uint8_t buf[6];
	
	// write the address of the registers to be read
	res = HAL_I2C_Master_Transmit(input->hi2c, (uint16_t)HMC_ADDR_W, (uint8_t*)HMC_DA_X_MSB, 1, HAL_MAX_DELAY);
	
	// check comm
	if ( res != HAL_OK ){
		return 1;
	}
	else{
		// read the value of regs
		res = HAL_I2C_Master_Receive(input->hi2c, (uint16_t)HMC_ADDR_R, buf, 6, HAL_MAX_DELAY);
		if ( res != HAL_OK ){
			return 2;
		}
	}
	
	// connect msb and lsb bits
	input->mag_X = buf[0] << 8 | buf[1];
	input->mag_Y = buf[2] << 8 | buf[3];
	input->mag_Z = buf[4] << 8 | buf[5];
	
	return 0;
}



uint8_t HMC_SendAddr_DMA(HMC_data *input){
	
	/*
	*
	*	Send addr for read with DMA (non-blocking)
	*
	*/
	
	// If the data is not ready to be read return
	if(input->data_ready == 0){
		return 4;
	}
	// If the data is ready clear the flag flag before it is read
	else{
		input->data_ready = 0;
	}
		
	// write the address of the registers to be read
	if(HAL_I2C_Master_Transmit_DMA(input->hi2c, (uint16_t)HMC_ADDR_W, (uint8_t*)HMC_DA_X_MSB, 1) == HAL_OK){
		input->reading_data = 1;
		return 0;
	}
	else{
		return 1;
	}
	
}

 
uint8_t HMC_ReadMeas_DMA(HMC_data *input){
	
	/*
	*
	*	Read measurement with DMA (non-blocking)
	*
	*/
	
	// If the addr is not sent
	if(input->reading_data == 0){
		return 4;
	}
	
	// read from the sent addr
	if( HAL_I2C_Master_Receive_DMA(input->hi2c, (uint16_t)HMC_ADDR_R, &input->raw[0], 6) == HAL_OK ){
		return 0;
	}
	else{
		return 1;
	}
}


void HMC_ReadCplt_DMA(HMC_data *input){
	
	/*
	*
	*	Read measurement complete with DMA (non-blocking)
	*
	*/
	
	// Reset read flag with completition
	input->reading_data = 0;
	
	// connect msb and lsb bits
	input->mag_X = input->raw[0] << 8 | input->raw[1];
	input->mag_Y = input->raw[2] << 8 | input->raw[3];
	input->mag_Z = input->raw[4] << 8 | input->raw[5];
}




