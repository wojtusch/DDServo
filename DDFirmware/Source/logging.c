/*
 ******************************************************************************
 * File    		logging.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Logging functions.
 * Peripherals
 ******************************************************************************
 */

// Includes
#define INLINE_LOGGING
#include "logging.h"

// Global variables
uint32_t endurance;					// 0 ... 4294967295 in s | total endurance of the servo motor
uint32_t performance;				// 0 ... 4294967295 in encoder steps | total angular performance of the servo motor
uint16_t overcurrentTime;			// 0 ... 65535 in s | total time with a current greater than specified current limit
uint16_t overtemperatureTime;		// 0 ... 65535 in s | total time with a motor temperature greater than specified motor temperature limit
uint16_t oldPositionMeasurement;	// 0 ... 4095 in encoder steps | old position measurement value

// Initialize logging functions
void LOGGING_Initialize(void) {

	// Initialize eeprom emulation for logging
	EEPROM1_Initalize();

	// Initialize logging values with zero or load saved values from eeprom
	if (EEPROM1_GetVariable(((int32_t)EEPROM1_GetVariable(EEPROM1_ENDURANCE_HIGH) << 16) | EEPROM1_GetVariable(EEPROM1_ENDURANCE_LOW)) == EEPROM_ERROR) {

		// Initialize logging values with zero
		endurance = 0;
		performance = 0;
		overcurrentTime = 0;
		overtemperatureTime = 0;

	} else {

		// Initialize logging values with saved values from eeprom
		endurance = ((int32_t)EEPROM1_GetVariable(EEPROM1_ENDURANCE_HIGH) << 16) | EEPROM1_GetVariable(EEPROM1_ENDURANCE_LOW);
		performance = ((int32_t)EEPROM1_GetVariable(EEPROM1_PERFORMANCE_HIGH) << 16) | EEPROM1_GetVariable(EEPROM1_PERFORMANCE_LOW);
		overcurrentTime = EEPROM1_GetVariable(EEPROM1_OVERCURRENT_TIME);
		overtemperatureTime = EEPROM1_GetVariable(EEPROM1_OVERTEMPERATURE_TIME);

	}

	// Initialize global variables
	oldPositionMeasurement = MEMORY_GetActualPositionMeasurement();

}
