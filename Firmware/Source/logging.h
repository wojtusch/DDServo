/*
 ******************************************************************************
 * File    		logging.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for logging functions.
 * Peripherals
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __LOGGING_H
#define __LOGGING_H

// Type includes
#include "stm32f10x.h"

// Defines
#ifndef INLINE_LOGGING
#define INLINE_LOGGING extern inline
#endif

// Global variables
extern uint32_t endurance;
extern uint32_t performance;
extern uint16_t overcurrentTime;
extern uint16_t overtemperatureTime;
extern uint16_t oldPositionMeasurement;

// Global function prototypes
void LOGGING_Initialize(void);
void LOGGING_Save(void);
void LOGGING_UpdateEndurance(void);
void LOGGING_UpdatePerformance(void);
void LOGGING_UpdateOvercurrentTime(void);
void LOGGING_UpdateOvertemperatureTime(void);
uint32_t LOGGING_GetEndurance(void);
uint32_t LOGGING_GetPerformance(void);
uint16_t LOGGING_GetOvercurrentTime(void);
uint16_t LOGGING_GetOvertemperatureTime(void);

// Module includes
#include "common.h"
#include "eeprom1.h"
#include "memory.h"

// Save logging values to eeprom
INLINE_LOGGING void LOGGING_Save(void) {

	if (MEMORY_CheckFlag(FLAG_LOGGING_ACTIVATED)) {

		// Save endurance value
		EEPROM1_SetVariable(EEPROM1_ENDURANCE_LOW, ((uint16_t)(endurance & 0x00FF)));
		EEPROM1_SetVariable(EEPROM1_ENDURANCE_HIGH, ((uint16_t)(endurance >> 16)));

		// Save performance value
		EEPROM1_SetVariable(EEPROM1_PERFORMANCE_LOW, ((uint16_t)(performance & 0x00FF)));
		EEPROM1_SetVariable(EEPROM1_PERFORMANCE_HIGH, ((uint16_t)(performance >> 16)));

		// Save overcurrent time value
		EEPROM1_SetVariable(EEPROM1_OVERCURRENT_TIME, overcurrentTime);

		// Save overtemperature time value
		EEPROM1_SetVariable(EEPROM1_OVERTEMPERATURE_TIME, overtemperatureTime);

	}

}

// Update endurance value
INLINE_LOGGING void LOGGING_UpdateEndurance(void) {

	// Increment endurance by 1
	endurance += 1;

}

// Update performance value
INLINE_LOGGING void LOGGING_UpdatePerformance(void) {

	uint16_t currentPosition;

	// Get current position
	currentPosition = MEMORY_GetActualPositionMeasurement();

	// Increment performance by position measurement difference
	performance += COMMON_ABSOLUTE(currentPosition - oldPositionMeasurement);

	// Update old position value
	oldPositionMeasurement = currentPosition;

}

// Update overcurrent time value
INLINE_LOGGING void LOGGING_UpdateOvercurrentTime(void) {

	// Increment overcurrent time by 1
	overcurrentTime += 1;

}

// Update overtemperature time value
INLINE_LOGGING void LOGGING_UpdateOvertemperatureTime(void) {

	// Increment overtemperature time by 1
	overtemperatureTime += 1;

}

// Get endurance
INLINE_LOGGING uint32_t LOGGING_GetEndurance(void) {

	return endurance;

}

// Get performance
INLINE_LOGGING uint32_t LOGGING_GetPerformance(void) {

	return performance;

}

// Get overcurrent time
INLINE_LOGGING uint16_t LOGGING_GetOvercurrentTime(void) {

	return overcurrentTime;

}

// Get overtemperature time
INLINE_LOGGING uint16_t LOGGING_GetOvertemperatureTime(void) {

	return overtemperatureTime;

}

#endif
