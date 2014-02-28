/*
 ******************************************************************************
 * File    		trajectoryGeneration.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for trajectory generation.
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __TRAJECTORY_GENERATION_H
#define __TRAJECTORY_GENERATION_H

// Type includes
#include "stm32f10x.h"
#include "fixpoint.h"

// Defines
#ifndef INLINE_TRAJECTORY_GENERATION
#define INLINE_TRAJECTORY_GENERATION extern inline
#endif

// Global variables
extern fxpTI_t trajectoryTime;
extern fxpTI_t trajectoryFinalTime;
extern fxp32_t initialPosition;
extern fxp32_t initialSpeed;
extern fxp32_t desiredPosition;
extern fxp32_t desiredSpeed;
extern fxp32_t desiredAcceleration;
extern fxp32_t referencePosition;
extern fxp32_t referenceSpeed;
extern fxp32_t referenceAcceleration;

// Global function prototypes
void TRAJECTORY_GENERATION_Initialize(void);
void TRAJECTORY_GENERATION_UpdatePositionTrajectory(void);
void TRAJECTORY_GENERATION_UpdateSpeedTrajectory(void);
void TRAJECTORY_GENERATION_ResetPositionTrajectory(void);
void TRAJECTORY_GENERATION_ResetSpeedTrajectory(void);
fxp32_t TRAJECTORY_GENERATION_GetReferencePosition(void);
fxp32_t TRAJECTORY_GENERATION_GetReferenceSpeed(void);
fxp32_t TRAJECTORY_GENERATION_GetReferenceAcceleration(void);

// Module includes
#include "common.h"
#include "memory.h"
#include "task.h"

// Reset of position trajectory states
INLINE_TRAJECTORY_GENERATION void TRAJECTORY_GENERATION_ResetPositionTrajectory(void) {

	// Set variables
	trajectoryTime = 0;
	initialPosition = MEMORY_GetActualPosition();
	initialSpeed = MEMORY_GetActualSpeed();
	referencePosition = initialPosition;
	referenceSpeed = initialSpeed;
	referenceAcceleration = 0;
	desiredPosition = MEMORY_GetDesiredPosition();
	desiredSpeed = MEMORY_GetDesiredSpeed();
	if (desiredSpeed == 0) {

		desiredSpeed = DEFAULT_DESIRED_SPEED;

	}
	if (desiredPosition < initialPosition) {

		desiredSpeed = -COMMON_ABSOLUTE(desiredSpeed);

	} else {

		desiredSpeed = COMMON_ABSOLUTE(desiredSpeed);

	}
	desiredAcceleration = COMMON_ABSOLUTE(MEMORY_GetDesiredAcceleration());

	// Calculate final time
	trajectoryFinalTime = ((fxp64_t)COMMON_ABSOLUTE(desiredPosition - initialPosition) << (FXP32_Q + FXPTI_Q)) / COMMON_ABSOLUTE(desiredSpeed);

}

// Reset of speed trajectory states
INLINE_TRAJECTORY_GENERATION void TRAJECTORY_GENERATION_ResetSpeedTrajectory(void) {

	// Set variables
	trajectoryTime = 0;
	initialSpeed = MEMORY_GetActualSpeed();
	referenceSpeed = initialSpeed;
	referenceAcceleration = 0;
	desiredSpeed = MEMORY_GetDesiredSpeed();
	desiredAcceleration = MEMORY_GetDesiredAcceleration();
	if (desiredAcceleration == 0) {

		desiredAcceleration = DEFAULT_DESIRED_ACCELERATION;

	}
	if (desiredSpeed < initialSpeed) {

		desiredAcceleration = -COMMON_ABSOLUTE(desiredAcceleration);

	} else {

		desiredAcceleration = COMMON_ABSOLUTE(desiredAcceleration);

	}

	// Calculate final time
	trajectoryFinalTime = FXP64toTI(((fxp64_t)COMMON_ABSOLUTE(desiredSpeed - initialSpeed) << FXP32_Q) / COMMON_ABSOLUTE(desiredAcceleration));

}

// Get reference position
INLINE_TRAJECTORY_GENERATION fxp32_t TRAJECTORY_GENERATION_GetReferencePosition(void) {

	return referencePosition;

}

// Get reference speed
INLINE_TRAJECTORY_GENERATION fxp32_t TRAJECTORY_GENERATION_GetReferenceSpeed(void) {

	return referenceSpeed;

}

// Get reference acceleration
INLINE_TRAJECTORY_GENERATION fxp32_t TRAJECTORY_GENERATION_GetReferenceAcceleration(void) {

	return referenceAcceleration;

}

#endif
