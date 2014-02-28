/*
 ******************************************************************************
 * File    		measurement.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Measurement functions.
 * Peripherals	GPIOA, GPIOB, ADC1, SPI1, DMA1, TIM3
 ******************************************************************************
 */

// Includes
#define INLINE_MEASUREMENT
#include "measurement.h"

// Global variables
volatile uint16_t rawPosition;
volatile uint8_t rawStatus;
volatile uint16_t rawADC[3];
volatile uint8_t bufferDO;
volatile uint8_t bufferDI[3];
volatile uint16_t motorTemperatureToleranceCounter;
volatile uint16_t controllerTemperatureToleranceCounter;

// Private function prototypes
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void TIM3_IRQHandler(void);

// Initialization of measurement
void MEASUREMENT_Initialize(void) {

	// Private variables
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	// Initialize variables
	rawPosition = 0;
	rawStatus = 0;
	rawADC[0] = 0;
	rawADC[1] = 0;
	rawADC[2] = 0;
	bufferDO = 255;
	bufferDI[0] = 0;
	bufferDI[1] = 0;
	bufferDI[2] = 0;
	motorTemperatureToleranceCounter = 0;
	controllerTemperatureToleranceCounter = 0;

	// Initialize sensor inputs and outputs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// SENCS
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	// SENCLK, SENDO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// TEMP
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// VOLT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Initialize TIM3 as timer with compare interrupt and output trigger
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = CYCLE_TIME;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_Pulse = ANALOG_MEASUREMENT_TIME;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = TIMEOUT_TIME;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = CYCLE_TIME;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Initialize DMA1 for ADC1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x4001244C;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rawADC;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_TE, ENABLE);

	// Initialize ADC1 for measuring channel 8, 16 and 3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 3;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_239Cycles5);
	ADC_Cmd(ADC1, ENABLE);

	// Calibrate ADC1
	MEASUREMENT_CalibrateAnalogConverter();

	// Initialize DMA1 for SPI1 reception
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x4001300C;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)bufferDI;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC | DMA_IT_TE, ENABLE);

	// Initialize DMA1 for SPI1 transmission
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)bufferDO;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    // Initialize SPI1 with 281.25 kHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_CalculateCRC(SPI1, DISABLE);

}

// DMA interrupt handler
void DMA1_Channel1_IRQHandler(void) {

	// DMA transfer complete
	if (DMA_GetITStatus(DMA1_IT_TC1)) {

		// Clear interrupt flag
		DMA_ClearITPendingBit(DMA1_IT_TC1);

		// Deactivate DMA
		DMA_Cmd(DMA1_Channel1, DISABLE);

		// Deactivate ADC DMA mode
		ADC_DMACmd(ADC1, DISABLE);

		// Evaluate analog measurement values
		if (MEMORY_CheckFlag(FLAG_ANALOG_CONVERTER_INITIALIZATION)) {

			// Reset flag for analog converter initialization
			MEMORY_ResetFlag(FLAG_ANALOG_CONVERTER_INITIALIZATION);

			// Save raw values
			MEMORY_SetActualVoltage(voltageScaling[rawADC[0]]);
			MEMORY_SetActualControllerTemperature(controllerTemperatureScaling[rawADC[1]] + MEMORY_GetControllerTemperatureOffset());
			MEMORY_SetActualMotorTemperature(motorTemperatureScaling[rawADC[2]]);

		}
		else {

			// Apply a moving average filters on raw values and save filtered values
			MEMORY_SetActualVoltage((fxp16_t)((((fxp32_t)MEMORY_GetActualVoltage() * 63) + voltageScaling[rawADC[0]]) >> 6));
			MEMORY_SetActualControllerTemperature((fxp16_t)((((fxp32_t)MEMORY_GetActualControllerTemperature() * 4095) + controllerTemperatureScaling[rawADC[1]] + MEMORY_GetControllerTemperatureOffset()) >> 12));
			MEMORY_SetActualMotorTemperature((fxp16_t)((((fxp32_t)MEMORY_GetActualMotorTemperature() * 4095) + motorTemperatureScaling[rawADC[2]]) >> 12));
		}

		// Check voltage limits and set voltage limit error
		if ((((MEMORY_GetActualVoltage() < MEMORY_GetVoltageLimitLow()) || (MEMORY_GetActualVoltage() > MEMORY_GetVoltageLimitHigh()))) && !MEMORY_CheckError(ERROR_VOLTAGE_LIMIT)) {

			// Set voltage limit error
			MEMORY_SetError(ERROR_VOLTAGE_LIMIT);

		}

		// Check controller temperature limit and set controller temperature limit error
		if ((MEMORY_GetActualControllerTemperature() > MEMORY_GetControllerTemperatureLimit()) && !MEMORY_CheckError(ERROR_CONTROLLER_TEMPERATURE_LIMIT)) {

			if (controllerTemperatureToleranceCounter < TOLERANCE_CONTROLLER_TEMPERATURE) {

				// Increment tolerance counter
				controllerTemperatureToleranceCounter++;

			} else {

				// Set controller temperature limit error
				MEMORY_SetError(ERROR_CONTROLLER_TEMPERATURE_LIMIT);

				// Reset tolerance counter
				controllerTemperatureToleranceCounter = 0;

			}

		}

		// Check motor temperature limit and set motor temperature limit error
		if ((MEMORY_GetActualMotorTemperature() > MEMORY_GetMotorTemperatureLimit()) && !MEMORY_CheckError(ERROR_MOTOR_TEMPERATURE_LIMIT)) {

			if (motorTemperatureToleranceCounter < TOLERANCE_MOTOR_TEMPERATURE) {

				// Increment tolerance counter
				motorTemperatureToleranceCounter++;

			} else {

				// Set motor temperature limit error
				MEMORY_SetError(ERROR_MOTOR_TEMPERATURE_LIMIT);

				// Reset tolerance counter
				motorTemperatureToleranceCounter = 0;

			}

		}

		// Configure and activate DMA for analog conversion
		DMA1_Channel1->CNDTR = 3;
		ADC_DMACmd(ADC1, ENABLE);
		DMA_Cmd(DMA1_Channel1, ENABLE);

	}

	// DMA transmission error
	else if (DMA_GetITStatus(DMA1_IT_TE1)) {

		// Clear interrupt flag
		DMA_ClearITPendingBit(DMA1_IT_TE1);

		// Set analog converter error flag
		MEMORY_SetError(ERROR_ANALOG_CONVERTER);

	}

}

// DMA interrupt handler
void DMA1_Channel2_IRQHandler(void) {

	// Private variables
	int16_t transformedPosition;

	// DMA transfer complete
	if (DMA_GetITStatus(DMA1_IT_TC2)) {

		// Clear interrupt flag
		DMA_ClearITPendingBit(DMA1_IT_TC2);

		// Set SENCS to idle high
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

		// Deactivate timeout TIM compare
		TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);

		// Deactivate DMA
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);

		// Deactivate SPI DMA mode
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

		// Evaluate position measurement values
		rawPosition = (bufferDI[0] & 0x7F) << 5;
		rawPosition |= (bufferDI[1] & 0xF8) >> 3;
		rawStatus = (bufferDI[1] & 0x07) << 3;
		rawStatus |= (bufferDI[2] & 0xE0) >> 5;
		if (MEASUREMENT_CheckParity(rawPosition, rawStatus)) {

			// Transform position sensor measurement direction and offset
			transformedPosition = (4096 - MEMORY_GetPositionOffset() - rawPosition) % 4096;
			if (transformedPosition < 0) {

				transformedPosition += 4096;

			}

			// Save position sensor measurement and status
			MEMORY_SetActualPositionMeasurement(transformedPosition);
			MEMORY_SetActualPositionStatus(rawStatus);

			// Update logging values
			if (MEMORY_CheckFlag(FLAG_LOGGING_ACTIVATED)) {

				// Update performance value
				LOGGING_UpdatePerformance();

			}

		}

		// Deactivate position sensor measurements
		if (MEMORY_CheckFlag(FLAG_POSITION_SENSOR_DEACTIVATED)) {

			// Reset flag for sensor deactivation
			MEMORY_ResetFlag(FLAG_POSITION_SENSOR_DEACTIVATED);

			// Deactivate TIM3
			if (!MEMORY_CheckFlag(FLAG_ANALOG_CONVERTER_ACTIVATED)) {

				TIM_Cmd(TIM3, DISABLE);

			}

			// Deactivate SPI1
			SPI_Cmd(SPI1, DISABLE);

			// Reset flag for activated position sensor
			MEMORY_ResetFlag(FLAG_POSITION_SENSOR_ACTIVATED);

		}

	}

	// DMA transmission error
	else if (DMA_GetITStatus(DMA1_IT_TE2)) {

		// Clear interrupt flag
		DMA_ClearITPendingBit(DMA1_IT_TE2);

		// Set SENCS to idle high
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

		// Deactivate DMA
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);

		// Deactivate SPI DMA mode
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

		// Set position sensor error flag
		MEMORY_SetError(ERROR_POSITION_SENSOR);

		// Deactivate position sensor measurements
		if (MEMORY_CheckFlag(FLAG_POSITION_SENSOR_DEACTIVATED)) {

			// Reset flag for sensor deactivation
			MEMORY_ResetFlag(FLAG_POSITION_SENSOR_DEACTIVATED);

			// Deactivate TIM3
			if (!MEMORY_CheckFlag(FLAG_ANALOG_CONVERTER_ACTIVATED)) {

				TIM_Cmd(TIM3, DISABLE);

			}

			// Deactivate SPI1
			SPI_Cmd(SPI1, DISABLE);

			// Reset flag for activated position sensor
			MEMORY_ResetFlag(FLAG_POSITION_SENSOR_ACTIVATED);

		}

	}

}

// TIM interrupt handler
void TIM3_IRQHandler(void) {

	// TIM compare: Sensor initialization ended
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {

		// Clear interrupt flag
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		// Reset DMA
		DMA1_Channel2->CNDTR = 3;
		DMA1_Channel3->CNDTR = 3;

		// Clear SPI data register
		SPI_I2S_ReceiveData(SPI1);

		// Activate SPI DMA mode
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

		// Activate DMA
		DMA_Cmd(DMA1_Channel2, ENABLE);
		DMA_Cmd(DMA1_Channel3, ENABLE);

	}
	// TIM compare: Sensor transmission timed out
	if ((TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) && (TIM3->DIER & TIM_DIER_CC2IE)) {

		// Clear interrupt flag
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		// Set SENCS to idle high
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

		// Deactivate DMA
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);

		// Deactivate SPI DMA mode
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

		// Set position sensor error flag
		MEMORY_SetError(ERROR_POSITION_SENSOR);

		// Deactivate position sensor measurements
		if (MEMORY_CheckFlag(FLAG_POSITION_SENSOR_DEACTIVATED)) {

			// Reset flag for sensor deactivation
			MEMORY_ResetFlag(FLAG_POSITION_SENSOR_DEACTIVATED);

			// Deactivate TIM3
			if (!MEMORY_CheckFlag(FLAG_ANALOG_CONVERTER_ACTIVATED)) {

				TIM_Cmd(TIM3, DISABLE);

			}

			// Deactivate SPI1
			SPI_Cmd(SPI1, DISABLE);

			// Reset flag for activated position sensor
			MEMORY_ResetFlag(FLAG_POSITION_SENSOR_ACTIVATED);

		}

	}
	// TIM compare: Sensor cycle ended
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {

		// Clear interrupt flag
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		// Set SENCS to active low
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);

		// Activate timeout TIM compare
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

	}

}
