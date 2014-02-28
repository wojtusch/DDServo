/*
 ******************************************************************************
 * File    		measurement.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for measurement functions.
 * Peripherals	GPIOA, GPIOB, ADC1, SPI1, DMA1, TIM3
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __MEASUREMENT_H
#define __MEASUREMENT_H

// Defines
#ifndef INLINE_MEASUREMENT
#define INLINE_MEASUREMENT extern inline
#endif

// Type includes
#include "stm32f10x.h"
#include "fixpoint.h"

// Defines
#define ANALOG_MEASUREMENT_TIME 90 	// Analog measurement time and position sensor initialisation time in µs
#define TIMEOUT_TIME 380 			// Position sensor timeout in µs
#define CYCLE_TIME 400				// Cycle time in µs

// Global function prototypes
void MEASUREMENT_Initialize(void);
void MEASUREMENT_CalibrateAnalogConverter(void);
void MEASUREMENT_CalibrateTemperatureSensor(void);
void MEASUREMENT_ActivatePositionSensor(void);
void MEASUREMENT_DeactivatePositionSensor(void);
void MEASUREMENT_ActivateAnalogConverter(void);
void MEASUREMENT_DeactivateAnalogConverter(void);
uint8_t MEASUREMENT_CheckParity(uint16_t, uint8_t);

// Module includes
#include "common.h"
#include "logging.h"
#include "measurementLookup.h"
#include "memory.h"
#include "task.h"

// Calibrate analog converter
INLINE_MEASUREMENT void MEASUREMENT_CalibrateAnalogConverter(void) {

	// Run calibration with deactivated analog converter only
	if(MEMORY_CheckFlag(!FLAG_ANALOG_CONVERTER_ACTIVATED)) {

		// Reset calibration
		ADC_ResetCalibration(ADC1);
		while(ADC_GetResetCalibrationStatus(ADC1));
		// Start calibration
		ADC_StartCalibration(ADC1);
		while(ADC_GetCalibrationStatus(ADC1));

	}

}

// Calibrate controller temperature sensor
INLINE_MEASUREMENT void MEASUREMENT_CalibrateTemperatureSensor(void) {

	// Run calibration with activated analog converter only
	if(MEMORY_CheckFlag(FLAG_ANALOG_CONVERTER_ACTIVATED)) {

		// Calculate and save controller temperature sensor offset
		MEMORY_SetControllerTemperatureOffset(MEMORY_GetActualMotorTemperature() - MEMORY_GetActualControllerTemperature());

	}

}

// Activate position sensor measurements
INLINE_MEASUREMENT void MEASUREMENT_ActivatePositionSensor(void) {

	// Set SENCS to active low
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);

	// Activate SPI1
	SPI_Cmd(SPI1, ENABLE);

	// Activate all compares and TIM3
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	// Set flag for activated position sensor
	MEMORY_SetFlag(FLAG_POSITION_SENSOR_ACTIVATED);

}

// Deactivate position sensor measurements
INLINE_MEASUREMENT void MEASUREMENT_DeactivatePositionSensor(void) {

	// Set flag for sensor deactivation
	MEMORY_SetFlag(FLAG_POSITION_SENSOR_DEACTIVATED);

}

// Activate analog converter measurements
INLINE_MEASUREMENT void MEASUREMENT_ActivateAnalogConverter(void) {

	// Set flag for analog converter initialization
	MEMORY_SetFlag(FLAG_ANALOG_CONVERTER_INITIALIZATION);

	// Activate ADC DMA mode
	ADC_DMACmd(ADC1, ENABLE);

	// Activate DMA
	DMA_Cmd(DMA1_Channel1, ENABLE);

	// Activate external trigger
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);

	// Activate TIM3
	TIM_Cmd(TIM3, ENABLE);

	// Set flag for activated analog converter
	MEMORY_SetFlag(FLAG_ANALOG_CONVERTER_ACTIVATED);

}

// Deactivate analog converter measurements
INLINE_MEASUREMENT void MEASUREMENT_DeactivateAnalogConverter(void) {

	// Deactivate external trigger
	ADC_ExternalTrigConvCmd(ADC1, DISABLE);

	// Deactivate DMA
	DMA_Cmd(DMA1_Channel1, DISABLE);

	// Deactivate ADC DMA mode
	ADC_DMACmd(ADC1, DISABLE);

	// Reset DMA
	DMA1_Channel1->CNDTR = 3;

	// Deactivate TIM3
	if (!MEMORY_CheckFlag(FLAG_POSITION_SENSOR_ACTIVATED)) {

		TIM_Cmd(TIM3, DISABLE);

	}

	// Reset flag for activated analog converter
	MEMORY_ResetFlag(FLAG_ANALOG_CONVERTER_ACTIVATED);

}

// Calculate and check parity of raw sensor values
INLINE_MEASUREMENT uint8_t MEASUREMENT_CheckParity(uint16_t position, uint8_t status) {

	uint8_t parity = status & 0x01;
	uint8_t numberOfOnes = 0;
	uint32_t data = (position << 5) | (status >> 1);

	// Count ones in data set
	while (data != 0) {

		numberOfOnes++;
		data &= (data - 0x01);

	}

	return (parity == (numberOfOnes & 0x01));

}

#endif
