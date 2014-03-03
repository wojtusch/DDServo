/*
 ******************************************************************************
 * File    		control.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Position and speed control functions.
 ******************************************************************************
 */

// Includes
#define INLINE_CONTROL
#include "control.h"

// Global variables
fxp64_t controlSignalFF;
fxp64_t controlSignalP;
fxp64_t controlSignalI;
fxp64_t controlSignalD;
fxp64_t controlSignal;
fxp32_t controlError[3];
fxp32_t actualVoltage;
fxp32_t referencePositionValue;
fxp32_t referenceSpeedValue;
fxp32_t referenceAccelerationValue;
uint8_t controlMode;

// Initialization of control states
void CONTROL_Initialize(void) {

	// Initialize variables
	controlSignalP = 0;
	controlSignalI = 0;
	controlSignalD = 0;
	controlSignalFF = 0;
	controlSignal = 0;
	controlError[0] = 0;
	controlError[1] = 0;
	controlError[2] = 0;
	actualVoltage = 0;
	controlMode = MODE_POSITION_CONTROL_WITHOUT_TRAJECTORY;

}

// Reset of control states
void CONTROL_ResetControlStates(void) {

	// Reset variables
	controlSignalFF = 0;
	controlSignalP = 0;
	controlSignalI = 0;
	controlSignalD = 0;
	controlSignal = 0;

}

// Reset of desired values
void CONTROL_ResetDesiredValues(void) {

	// Reset variables
	switch ((MEMORY_CheckFlag(FLAG_CONTROL_MODE_2) << 2) | (MEMORY_CheckFlag(FLAG_CONTROL_MODE_1) << 1) | MEMORY_CheckFlag(FLAG_CONTROL_MODE_0)) {

	case MODE_POSITION_CONTROL_WITHOUT_TRAJECTORY:

		MEMORY_SetDesiredPosition(MEMORY_GetActualPosition());

		break;

	case MODE_POSITION_CONTROL_WITH_TRAJECTORY:

		MEMORY_SetDesiredPosition(MEMORY_GetActualPosition());

		break;

	case MODE_SPEED_CONTROL_WITHOUT_TRAJECTORY:

		MEMORY_SetDesiredSpeed(0);

		break;

	case MODE_SPEED_CONTROL_WITH_TRAJECTORY:

		MEMORY_SetDesiredSpeed(0);

		break;

	case MODE_TORQUE_CONTROL:

		//MEMORY_SetDesiredTorque(0);

		break;

	case MODE_PULSE_WIDTH_CONTROL:

		MEMORY_SetDesiredPulseWidth(0);

		break;

	default:

		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}
