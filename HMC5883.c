#include "HMC5883.h"
#include "main.h"

/* --------
 *
 *	Functions
 *
 *-------- */


/* 
 *------- SETUP -------
 */

enum HMC_ret HMC_Init_Struct(I2C_HandleTypeDef *hi2c,
		GPIO_TypeDef *HMC_INT_Port, uint16_t HMC_IntPin) {
	/*
	 *
	 *	Initialize data struct
	 *
	 */
	
	// Store ID
	HMC.id[0] = HMC_ID_A_VAL;
	HMC.id[1] = HMC_ID_B_VAL;
	HMC.id[3] = HMC_ID_C_VAL;
	
	// Zero data
	HMC.measurement.raw[0] = 0;
	HMC.measurement.raw[1] = 0;
	HMC.measurement.raw[2] = 0;
	HMC.measurement.raw[3] = 0;
	HMC.measurement.raw[4] = 0;
	HMC.measurement.raw[5] = 0;

	HMC.measFloat.X = 0;
	HMC.measFloat.Y = 0;
	HMC.measFloat.Z = 0;

	HMC.Offs.xOffs = 0;
	HMC.Offs.yOffs = 0;
		
	// Interface parameters
	HMC.hi2c = hi2c;
	HMC.int_PinPort = HMC_INT_Port;
	HMC.int_Pin = HMC_IntPin;
	
	// Busy flag
	HMC.state = HMC_INIT;
	
	// Data regs
	HMC.reg_address[0] 	= 	HMC_DA_X_MSB;
	HMC.reg_address[1] 	= 	HMC_DA_X_LSB;
	HMC.reg_address[2] 	= 	HMC_DA_Z_MSB;
	HMC.reg_address[3] 	= 	HMC_DA_Z_LSB;
	HMC.reg_address[4] 	= 	HMC_DA_Y_MSB;
	HMC.reg_address[5] 	= 	HMC_DA_Y_LSB;
	
	// set the clocking
	if (ARM_CM_DWT_CTRL != 0){					// See if DWT is available
		// See if the DWT has been set by something else
		if ((((ARM_CM_DEMCR & (0b1 << 24)) != 0)) || ((ARM_CM_DWT_CTRL & 0b1) != 0)) {
			ARM_CM_DEMCR      |= 1 << 24;       // Set bit 24
			ARM_CM_DWT_CYCCNT  = 0;
			ARM_CM_DWT_CTRL   |= 1 << 0;		// Set bit 0
		}
	}

	HMC.TimeElapsed.currTicks = ARM_CM_DWT_CYCCNT;
	HMC.TimeElapsed.elapsed = 0;
	HMC.TimeElapsed.systemClock = HAL_RCC_GetSysClockFreq();

	// Config int pin
	HMC_IntPinConfig();

	// Read id and check if it is received correctly
	return HMC_Read_ID();
}


enum HMC_ret HMC_Read_ID(void) {
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
		res = HAL_I2C_Master_Transmit(HMC.hi2c, (uint16_t)HMC_ADDR_W, &buf[i], 1, HAL_MAX_DELAY);
	
		// check if comm is ok
		if ( res != HAL_OK ){
			return HMC_TX_ERR;
		}
		else{
			// read the value of regs
			res = HAL_I2C_Master_Receive(HMC.hi2c, (uint16_t)HMC_ADDR_R, &buf[i], 1, HAL_MAX_DELAY);

			// check if comm is ok
			if ( res != HAL_OK ){
				return HMC_RX_ERR;
			}
		}
	}

	// check whether the id was received correctly
	if ( buf[0] != HMC_ID_A_VAL || buf[1] != HMC_ID_B_VAL || buf[2] != HMC_ID_C_VAL ){	
		return HMC_ID_ERR;
	}
	else{
		return HMC_OK;
	}
}


enum HMC_ret HMC_Config(uint8_t average, uint8_t data_or,
						uint8_t meas_mode,
				    	uint8_t gain, uint8_t op_mode) {
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
	 *	- Continuous / HMC_MR_CONT
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
	buf[3] = gain | op_mode;
	
	// Write configuration
	for( uint8_t i=0; i <= 2; i+=2 ){

		// Comm
		res = HAL_I2C_Master_Transmit(HMC.hi2c, (uint16_t)HMC_ADDR_W, &buf[i], 2, HAL_MAX_DELAY);
	
		// Comm check
		if ( res != HAL_OK ){
			return HMC_TX_ERR;
		}
	}

	// Set sensitivity
	HMC_SetSensitivity(gain);

	// set cycle ticks for timer
	HMC_SetCycleTicks(meas_mode, op_mode, data_or);

	// All setup OK
	HMC.state = HMC_IDLE;
	
	// All setup OK
	return HMC_OK;	
}


void HMC_SetCycleTicks(uint8_t meas_mode, uint8_t op_mode, uint8_t data_or){
	/*
	 *
	 * Set up the number of cycles to timeout
	 *
	 */

	// measurement is only performed in normal mode
	if (meas_mode != HMC_CRA_MEAS_MODE_NORM){
		return;
	}

	// set the ticks / cycle
	if (op_mode == HMC_MR_IDLE) {
		HMC.TimeElapsed.cycleTicks = 0;
	} else if(op_mode == HMC_MR_SINGLE) {
		// approx. 10ms is a timeout
		HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock / 100;
	} else if(op_mode == HMC_MR_CONT){
		switch (data_or){
		case HMC_CRA_DATA_OR_0_75:
			HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock * 4 / 3;
			break;
		case HMC_CRA_DATA_OR_1_5:
			HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock / 3 * 2;
			break;
		case HMC_CRA_DATA_OR_3:
			HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock / 3;
			break;
		case HMC_CRA_DATA_OR_7:
			HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock / 7;
			break;
		case HMC_CRA_DATA_OR_15:
			HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock / 15;
			break;
		case HMC_CRA_DATA_OR_30:
			HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock / 30;
			break;
		case HMC_CRA_DATA_OR_75:
			HMC.TimeElapsed.cycleTicks = HMC.TimeElapsed.systemClock / 75;
			break;
		}
	}
}


void HMC_SetSensitivity(uint8_t gain) {
	/*
	 *
	 * Write the sensitivity of the sensor from the gain
	 *
	 */

	switch(gain){
		case HMC_CRB_GAIN0:
			HMC.sensitivity = 0.73f;
			break;

		case HMC_CRB_GAIN1:
			HMC.sensitivity = 0.92f;
			break;

		case HMC_CRB_GAIN2:
			HMC.sensitivity = 1.22f;
			break;

		case HMC_CRB_GAIN3:
			HMC.sensitivity = 1.52f;
			break;

		case HMC_CRB_GAIN4:
			HMC.sensitivity = 2.27f;
			break;

		case HMC_CRB_GAIN5:
			HMC.sensitivity = 2.53f;
			break;

		case HMC_CRB_GAIN6:
			HMC.sensitivity = 3.03f;
			break;

		case HMC_CRB_GAIN7:
			HMC.sensitivity = 4.35f;
			break;

		default:
			break;
	}
}


void HMC_IntPinConfig(void) {
	/*
	 *
	 * Configure the interrupt pin
	 *
	 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Determine which port is to be used
	if (HMC.int_PinPort == GPIOA){ __HAL_RCC_GPIOA_CLK_ENABLE(); }
	if (HMC.int_PinPort == GPIOB){ __HAL_RCC_GPIOB_CLK_ENABLE(); }
	if (HMC.int_PinPort == GPIOC){ __HAL_RCC_GPIOC_CLK_ENABLE(); }
	if (HMC.int_PinPort == GPIOD){ __HAL_RCC_GPIOD_CLK_ENABLE(); }
	if (HMC.int_PinPort == GPIOE){ __HAL_RCC_GPIOE_CLK_ENABLE(); }

	// configure the pin
	GPIO_InitStruct.Pin = HMC.int_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(HMC.int_PinPort, &GPIO_InitStruct);
}


/* 
 *	------- OPERATION ------- 
 */

enum HMC_ret HMC_Read_Poll(void) {
	/*
	 *
	 *	Read measurement in polling mode
	 *
	 */
	
	// Init vars
	HAL_StatusTypeDef res;
	// uint8_t buf[6];
	
	// write the address of the registers to be read
	res = HAL_I2C_Master_Transmit(HMC.hi2c, (uint16_t)HMC_ADDR_W, (uint8_t*)HMC_DA_X_MSB, 1, HAL_MAX_DELAY);
	
	// check comm
	if ( res != HAL_OK ){
		return HMC_TX_ERR;
	}
	else{
		// read the value of regs
		res = HAL_I2C_Master_Receive(HMC.hi2c, (uint16_t)HMC_ADDR_R, HMC.measurement.raw, 6, HAL_MAX_DELAY);
		if ( res != HAL_OK ){
			return HMC_RX_ERR;
		}
	}
	
	return HMC_OK;
}


enum HMC_ret HMC_Trigger_Single_Meas(void) {
	/*
	 *
	 *	Trigger single measurement
	 *
	 *	Used with single shot mode
	 *	Mode is set to single shot
	 *	Trigger when data was received
	 *	Either in state machine or rx_done
	 *
	 */
	
	// Init vars
	HAL_StatusTypeDef res;
	uint8_t buf[2];
	
	// Fill buffer for triggering	
	buf[0] = HMC_MODE_REG;
	buf[1] = HMC_MR_SINGLE;
	
	// Comm
	res = HAL_I2C_Master_Transmit_DMA(HMC.hi2c, (uint16_t)HMC_ADDR_W, buf, 2);
	
	// Check comm
	if ( res != HAL_OK ){
		return HMC_TX_DMA_ERR;
	}
	
	return HMC_OK;
}


enum HMC_ret HMC_SendAddr_DMA(void) {
	/*
	 *
	 *	Send addr for read with DMA (non-blocking)
	 *	(data ready pin)
	 *
	 */
	
	// If the data is not ready to be read return
	if(HMC.state != HMC_IDLE){
		return HMC_TX_DMA_ERR;
	}
		
	// write the address of the registers to be read
	if(HAL_I2C_Master_Transmit_DMA(HMC.hi2c, (uint16_t)HMC_ADDR_W, (uint8_t*)HMC_DA_X_MSB, 1) == HAL_OK){
		return HMC_OK;
	}
	else{
		return HMC_TX_DMA_ERR;
	}
}


enum HMC_ret HMC_ReadMeas_DMA(void) {
	/*
	 *
	 *	Read measurement with DMA (non-blocking)
	 *	(tx done)
	 *
	 */
	
	// If the addr is not sent
	if(HMC.state != HMC_ADDR_SENT){
		return HMC_RX_DMA_ERR;
	}
	
	// read from the sent addr
	if( HAL_I2C_Master_Receive_DMA(HMC.hi2c, (uint16_t)HMC_ADDR_R, HMC.measurement.raw, 6) == HAL_OK ){
		return HMC_OK;
	}
	else{
		return HMC_RX_DMA_ERR;
	}
}


void HMC_ReadCplt_DMA(void) {
	/*
	 *
	 *	Read measurement complete
	 *  located in void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
	 *
	 */
	
	HMC.state = HMC_IDLE;
}


/* 
 * ------- STATE MACHINE -------
 */

void HMC_StateMachine(void) {
	/*
	 *
	 *	State machine for reading data ONE SHOT
	 *
	 */
		
	// State machine
	switch(HMC.state){
		
		// Idle
		case HMC_IDLE:
			break;

		// Reading
		case HMC_TRIGGERED:
			break;
		case HMC_DATA_RDY:
			break;
		
		// Read ready
		case HMC_READING:
			break;
		
		// Error
		default:
			break;
	}

	if (HMC_CheckTimeOut() == HMC_True) {
		HMC.state = HMC_TIMEOUT;
	}
}


/*
 *
 * Interrupt handlers
 *
 */

void HMC_DmaTxCpltHandler(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == HMC.hi2c->Instance) {
		enum HMC_ret ret;
		ret = HMC_ReadMeas_DMA();

		if (ret == HMC_OK) {
			HMC.state = HMC_READING;
		} else {
			HMC.state = HMC_RESET;
		}
	}
}


void HMC_DmaRxCpltHandler(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == HMC.hi2c->Instance) {
		HMC_ReadCplt_DMA();
	}
}


void HMC_DataRdyPinHandler(void) {
	/*
	 *
	 *	Interrupt handler
	 *
	 */
	
	// Check if the interrupt is from the HMC
	if(HAL_GPIO_ReadPin(HMC.int_PinPort, HMC.int_Pin) == GPIO_PIN_RESET) {
		
		// Check if the data is ready to be read
		switch (HMC.state) {
		case HMC_IDLE:
			if (HMC.mode != HMC_Cont){
				HMC.state = HMC_RX_ERR;
				return;
			}
			break;

		case HMC_TRIGGERED:
			if (HMC.mode != HMC_OneShot) {
				HMC.state = HMC_RX_ERR;
				return;
			}
			break;
		
		default:
			break;
		}

		enum HMC_ret ret;
		ret = HMC_SendAddr_DMA();

		if (ret == HMC_OK) {
			HMC.state = HMC_ADDR_SENT;
		} else {
			HMC.state = HMC_RESET;
		}
	}
}


/*
 *
 * Condition measurements
 *
 */

void HMC_ConditionMeasAll(void) {
	/*
	 *
	 * Condition all measurement in HMC struct
	 *
	 */
	HMC.measFloat.X = ((float)HMC.measurement.meas.X - HMC.Offs.xOffs) * HMC.sensitivity;
	HMC.measFloat.Y = ((float)HMC.measurement.meas.Y - HMC.Offs.xOffs) * HMC.sensitivity;
	HMC.measFloat.Z = (float)HMC.measurement.meas.Z * HMC.sensitivity;
}


float HMC_ConditionMeasSingle(int16_t meas_raw, float offset) {
	/*
	 *
	 * Condition any individual measurement
	 *
	 */
	return ((float)meas_raw - offset) * HMC.sensitivity;
}


/*
 *
 * Check timeout
 *
 */

enum HMC_bool HMC_CheckTimeOut(void) {
	/*
	 *
	 * Check if timeout has occurred
	 *
	 */

	// calculate elapsed time
	HMC.TimeElapsed.elapsed = ARM_CM_DWT_CYCCNT - HMC.TimeElapsed.currTicks;

	// check whether the state has changed
	if(HMC.state != HMC.TimeElapsed.prevState) {
		HMC_ResetTimer();
		return HMC_False;
	}

	// check whether timeout has occurred
	if (HMC.TimeElapsed.elapsed > HMC.TimeElapsed.cycleTicks) {
		return HMC_True;
	} else {
		return HMC_False;
	}
}


void HMC_ResetTimer(void) {
	/*
	 *
	 * Reset the timer
	 *
	 */

	HMC.TimeElapsed.elapsed = 0;
	HMC.TimeElapsed.currTicks = ARM_CM_DWT_CYCCNT;
	HMC.TimeElapsed.prevState = HMC.state;
}


/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
}
 */
