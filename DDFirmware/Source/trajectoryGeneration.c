/*
 ******************************************************************************
 * File    		trajectoryGeneration.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Trajectory generation.
 ******************************************************************************
 */

// Includes
#define INLINE_TRAJECTORY_GENERATION
#include "trajectoryGeneration.h"

// Global variables
fxpTI_t trajectoryTime;
fxpTI_t trajectoryFinalTime;
fxp32_t initialPosition;
fxp32_t initialSpeed;
fxp32_t desiredPosition;
fxp32_t desiredSpeed;
fxp32_t desiredAcceleration;
fxp32_t referencePosition;
fxp32_t referenceSpeed;
fxp32_t referenceAcceleration;

// Initialization of the state observer
void TRAJECTORY_GENERATION_Initialize(void) {

	// Initialize variables
	TRAJECTORY_GENERATION_ResetPositionTrajectory();

}

void TRAJECTORY_GENERATION_UpdatePositionTrajectory(void) {

	// Calculate position trajectory
	if ((trajectoryTime + TRAJECTORY_TIME_INTERVAL) < trajectoryFinalTime) {

		// Update trajectory time
		trajectoryTime += TRAJECTORY_TIME_INTERVAL;

		// Calculate reference values
		referencePosition = initialPosition + (fxp32_t) ((FXPTIto64(trajectoryTime) * desiredSpeed) >> FXP32_Q);
		referenceSpeed = desiredSpeed;

	} else {

		referencePosition = desiredPosition;
		referenceSpeed = 0;

	}

}

void TRAJECTORY_GENERATION_UpdateSpeedTrajectory(void) {

	// Calculate speed trajectory
	if ((trajectoryTime + TRAJECTORY_TIME_INTERVAL) < trajectoryFinalTime) {

		// Update trajectory time
		trajectoryTime += TRAJECTORY_TIME_INTERVAL;

		// Calculate reference values
		referenceSpeed = initialSpeed + (fxp32_t) ((FXPTIto64(trajectoryTime) * desiredAcceleration) >> FXP32_Q);
		referenceAcceleration = desiredAcceleration;

	} else {

		referenceSpeed = desiredSpeed;
		referenceAcceleration = 0;

	}

}
