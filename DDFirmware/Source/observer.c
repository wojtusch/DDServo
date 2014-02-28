/*
 ******************************************************************************
 * File    		observer.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		State observer functions.
 ******************************************************************************
 */

// Includes
#define INLINE_OBSERVER
#include "observer.h"

// Global variables
fxp32_t estimatedPosition[2];
fxp32_t estimatedSpeed[2];
fxp32_t estimatedTorque[2];
uint16_t torqueToleranceCounter;
fxp32_t estimatedCurrent;
uint16_t currentToleranceCounter;
fxp32_t appliedVoltage;
fxp32_t measuredPosition;
uint16_t rawPositionMeasurement[2];
int32_t multiturnOffset;

// Initialization of state observer states
void OBSERVER_Initialize(void) {

	// Private variables
	uint16_t actualPositionMeasurement = MEMORY_GetActualPositionMeasurement();

	// Initialize variables
	if (actualPositionMeasurement > 2048) {

		estimatedPosition[0] = ((fxp32_t)(((int64_t)(actualPositionMeasurement - 4096) * TWO_PI32) >> 12));
		multiturnOffset = -4096;

	} else {

		estimatedPosition[0] = ((fxp32_t)(((int64_t)actualPositionMeasurement * TWO_PI32) >> 12));
		multiturnOffset = 0;

	}
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

}
