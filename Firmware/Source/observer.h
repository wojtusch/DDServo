/*
 ******************************************************************************
 * File    		observer.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for state observer functions.
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __OBSERVER_H
#define __OBSERVER_H

// Type includes
#include "stm32f10x.h"
#include "fixpoint.h"

// Defines
#ifndef INLINE_OBSERVER
#define INLINE_OBSERVER extern inline
#endif
#define MULTITURN_LIMIT_1 1365			// First third of 4095
#define MULTITURN_LIMIT_2 2731			// Second third of 4095
#if (CONFIGURATION_MODEL_NUMBER  == 128)
#define OBSERVER_CONSTANT_1 32			// Matrix element A_o12
#define OBSERVER_CONSTANT_2 -2			// Matrix element A_o13
#define OBSERVER_CONSTANT_3 61289		// Matrix element A_o22
#define OBSERVER_CONSTANT_4 -9649		// Matrix element A_o23
#define OBSERVER_CONSTANT_5 2221		// Vector element B_o2
#define OBSERVER_CONSTANT_6 12329		// Vector element (K_o1 * 4096 / (2 * pi))
#define OBSERVER_CONSTANT_7 1965250		// Vector element (K_o2 * 4096 / (2 * pi))
#define OBSERVER_CONSTANT_8 -1230417	// Vector element (K_o3 * 4096 / (2 * pi))
#define OBSERVER_CONSTANT_9 125295		// Factor (r_g / k_w)
#define OBSERVER_CONSTANT_10 543949		// Factor R_m
#elif (CONFIGURATION_MODEL_NUMBER  == 164)
#define OBSERVER_CONSTANT_1 32			// Matrix element A_o12
#define OBSERVER_CONSTANT_2 -1			// Matrix element A_o13
#define OBSERVER_CONSTANT_3 61473		// Matrix element A_o22
#define OBSERVER_CONSTANT_4 -3608		// Matrix element A_o23
#define OBSERVER_CONSTANT_5 1524		// Vector element B_o2
#define OBSERVER_CONSTANT_6 8789		// Vector element (K_o1 * 4096 / (2 * pi))
#define OBSERVER_CONSTANT_7 892907		// Vector element (K_o2 * 4096 / (2 * pi))
#define OBSERVER_CONSTANT_8 -1260323	// Vector element (K_o3 * 4096 / (2 * pi))
#define OBSERVER_CONSTANT_9 174679		// Factor (r_g / k_w)
#define OBSERVER_CONSTANT_10 414843		// Factor R_m
#endif

// Global variables
extern fxp32_t estimatedPosition[2];
extern fxp32_t estimatedSpeed[2];
extern fxp32_t estimatedTorque[2];
extern uint16_t torqueToleranceCounter;
extern fxp32_t estimatedCurrent;
extern uint16_t currentToleranceCounter;
extern fxp32_t appliedVoltage;
extern fxp32_t measuredPosition;
extern uint16_t rawPositionMeasurement[2];
extern int32_t multiturnOffset;

// Global function prototypes
void OBSERVER_Initialize(void);
void OBSERVER_UpdateEstimation(void);
void OBSERVER_ResetEstimation(void);

// Module includes
#include "memory.h"
#include "motor.h"

// Update estimation of observed variables
INLINE_OBSERVER void OBSERVER_UpdateEstimation(void) {

	// Private variables
	fxp32_t deviation;

	// Get new applied armature voltage
	appliedVoltage = MOTOR_GetArmatureVoltage();

	// Get new raw position
	rawPositionMeasurement[0] = MEMORY_GetActualPositionMeasurement();

	// Multiturn detection
	if ((rawPositionMeasurement[0] < MULTITURN_LIMIT_1) && (rawPositionMeasurement[1] > MULTITURN_LIMIT_2)) {

		// Counterclockwise overflow
		multiturnOffset += 4096;

	} else if ((rawPositionMeasurement[0] > MULTITURN_LIMIT_2) && (rawPositionMeasurement[1] < MULTITURN_LIMIT_1)) {

		// Clockwise overflow
		multiturnOffset -= 4096;

	}
	measuredPosition = ((fxp32_t)(((int64_t)(rawPositionMeasurement[0] + multiturnOffset) * TWO_PI32) >> 12));

	// Update old raw position
	rawPositionMeasurement[1] = rawPositionMeasurement[0];

	// Prediction phase
	estimatedPosition[0] = estimatedPosition[1] + (fxp32_t)((((int64_t)estimatedSpeed[1] * OBSERVER_CONSTANT_1) + ((int64_t)estimatedTorque[1] * OBSERVER_CONSTANT_2)) >> FXP32_Q);
	estimatedSpeed[0] = (fxp32_t)((((int64_t)estimatedSpeed[1] * OBSERVER_CONSTANT_3) + ((int64_t)estimatedTorque[1] * OBSERVER_CONSTANT_4) + ((int64_t)appliedVoltage * OBSERVER_CONSTANT_5)) >> FXP32_Q);

	// Update phase
	deviation = measuredPosition - estimatedPosition[0];
	estimatedPosition[0] += FXP32_Mul(deviation, OBSERVER_CONSTANT_6);
	estimatedSpeed[0] += FXP32_Mul(deviation, OBSERVER_CONSTANT_7);
	estimatedTorque[0] += FXP32_Mul(deviation, OBSERVER_CONSTANT_8);

	// Calculate estimated current
	estimatedCurrent = (fxp32_t)(((int64_t)appliedVoltage << FXP32_Q) - ((int64_t)estimatedSpeed[0] * OBSERVER_CONSTANT_9)) / OBSERVER_CONSTANT_10;

	// Save estimated values
	MEMORY_SetActualPosition(estimatedPosition[0]);
	MEMORY_SetActualSpeed(estimatedSpeed[0]);
	MEMORY_SetActualTorque(estimatedTorque[0]);
	MEMORY_SetActualCurrent(estimatedCurrent);

	// Check torque limit and set torque limit error
	if (((COMMON_ABSOLUTE(estimatedTorque[0]) > MEMORY_GetTorqueLimit())) && !MEMORY_CheckError(ERROR_TORQUE_LIMIT)) {

		if (torqueToleranceCounter < TOLERANCE_CURRENT) {

			// Increment tolerance counter
			torqueToleranceCounter++;

		} else {

			// Set torque limit error
			MEMORY_SetError(ERROR_TORQUE_LIMIT);

			// Reset tolerance counter
			torqueToleranceCounter = 0;

		}

	} else if (((COMMON_ABSOLUTE(estimatedTorque[0]) <= MEMORY_GetTorqueLimit())) && MEMORY_CheckError(ERROR_TORQUE_LIMIT)) {

		// Reset torque limit error
		MEMORY_ResetError(ERROR_TORQUE_LIMIT);

		// Reset tolerance counter
		torqueToleranceCounter = 0;

	}

	// Check torque limit and set torque limit error
	if (((COMMON_ABSOLUTE(estimatedCurrent) > MEMORY_GetCurrentLimit())) && !MEMORY_CheckError(ERROR_CURRENT_LIMIT)) {

		if (currentToleranceCounter < TOLERANCE_CURRENT) {

			// Increment tolerance counter
			currentToleranceCounter++;

		} else {

			// Set current limit error
			MEMORY_SetError(ERROR_CURRENT_LIMIT);

			// Reset tolerance counter
			currentToleranceCounter = 0;

		}

	} else if (((COMMON_ABSOLUTE(estimatedCurrent) <= MEMORY_GetCurrentLimit())) && MEMORY_CheckError(ERROR_CURRENT_LIMIT)) {

		// Reset current limit error
		MEMORY_ResetError(ERROR_CURRENT_LIMIT);

		// Reset tolerance counter
		currentToleranceCounter = 0;

	}

	// Update old values
	estimatedPosition[1] = estimatedPosition[0];
	estimatedSpeed[1] = estimatedSpeed[0];
	estimatedTorque[1] = estimatedTorque[0];

}

// Reset estimation of observed variables
INLINE_OBSERVER void OBSERVER_ResetEstimation(void) {

	estimatedPosition[0] = 0;
	estimatedPosition[1] = estimatedPosition[0];
	estimatedSpeed[0] = 0;
	estimatedSpeed[1] = 0;
	estimatedTorque[0] = 0;
	estimatedTorque[1] = 0;
	torqueToleranceCounter = 0;
	estimatedCurrent = 0;
	currentToleranceCounter = 0;
	appliedVoltage = 0;
	measuredPosition = 0;
	rawPositionMeasurement[0] = 0;
	rawPositionMeasurement[1] = 2048;
	multiturnOffset = 0;

}

#endif
