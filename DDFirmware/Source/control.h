/*
 ******************************************************************************
 * File    		control.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for position and speed control functions.
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __CONTROL_H
#define __CONTROL_H

// Type includes
#include "stm32f10x.h"
#include "fixpoint.h"

// Defines
#ifndef INLINE_CONTROL
#define INLINE_CONTROL extern inline
#endif
#if (CONFIGURATION_MODEL_NUMBER == 128)
#define FEED_FORWARD_FACTOR_SPEED32 124518
#define FEED_FORWARD_FACTOR_ACCELERATION32 937
#define FEED_FORWARD_FACTOR_TORQUE32 281805
#elif (CONFIGURATION_MODEL_NUMBER == 164)
#define FEED_FORWARD_FACTOR_SPEED32 176947
#define FEED_FORWARD_FACTOR_ACCELERATION32 1363
#define FEED_FORWARD_FACTOR_TORQUE32 157286
#endif

// Global variables
extern fxp64_t controlSignalFF;
extern fxp64_t controlSignalP;
extern fxp64_t controlSignalI;
extern fxp64_t controlSignalD;
extern fxp64_t controlSignal;
extern fxp32_t controlError[3];
extern fxp32_t actualVoltage;
extern fxp32_t referencePositionValue;
extern fxp32_t referenceSpeedValue;
extern fxp32_t referenceAccelerationValue;
extern uint8_t controlMode;

// Global function prototypes
void CONTROL_Initialize(void);
void CONTROL_Update(void);
void CONTROL_ResetControlStates(void);
void CONTROL_ResetDesiredValues(void);

// Module includes
#include "motor.h"
#include "task.h"
#include "trajectoryGeneration.h"

// Update control signal
INLINE_CONTROL void CONTROL_UpdateControlSignal(void) {

	// Get actual voltage
	actualVoltage = FXP16to32(MEMORY_GetActualVoltage());

	// Distinction of controlled values
	switch ((MEMORY_CheckFlag(FLAG_CONTROL_MODE_2) << 2) | (MEMORY_CheckFlag(FLAG_CONTROL_MODE_1) << 1) | MEMORY_CheckFlag(FLAG_CONTROL_MODE_0)) {

	case MODE_POSITION_CONTROL_WITHOUT_TRAJECTORY:

		// Reset control errors and signals after switching control mode
		if (controlMode != MODE_POSITION_CONTROL_WITHOUT_TRAJECTORY) {

			controlSignalP = 0;
			controlSignalI = 0;
			controlSignalD = 0;
			controlSignalFF = 0;
			controlSignal = 0;
			controlError[0] = 0;
			controlError[1] = 0;
			controlError[2] = 0;

			// Initialize desired position
			MEMORY_SetDesiredPosition(MEMORY_GetActualPosition());

			// Set control mode
			controlMode = MODE_POSITION_CONTROL_WITHOUT_TRAJECTORY;

		}

		// Set desired values
		referencePositionValue = MEMORY_GetDesiredPosition();
		referenceSpeedValue = MEMORY_GetDesiredSpeed();
		referenceAccelerationValue = MEMORY_GetDesiredAcceleration();

		break;

	case MODE_POSITION_CONTROL_WITH_TRAJECTORY:

		// Reset control errors and signals after switching control mode
		if (controlMode != MODE_POSITION_CONTROL_WITH_TRAJECTORY) {

			controlSignalP = 0;
			controlSignalI = 0;
			controlSignalD = 0;
			controlSignalFF = 0;
			controlSignal = 0;
			controlError[0] = 0;
			controlError[1] = 0;
			controlError[2] = 0;

			// Initialize desired position
			MEMORY_SetDesiredPosition(MEMORY_GetActualPosition());

			// Set control mode
			controlMode = MODE_POSITION_CONTROL_WITH_TRAJECTORY;

		}

		// Update trajectory generation and set desired values
		TRAJECTORY_GENERATION_UpdatePositionTrajectory();
		referencePositionValue = TRAJECTORY_GENERATION_GetReferencePosition();
		referenceSpeedValue = TRAJECTORY_GENERATION_GetReferenceSpeed();
		referenceAccelerationValue = TRAJECTORY_GENERATION_GetReferenceAcceleration();

		break;

	case MODE_SPEED_CONTROL_WITHOUT_TRAJECTORY:

		// Reset control errors and signals after switching control mode
		if (controlMode != MODE_SPEED_CONTROL_WITHOUT_TRAJECTORY) {

			controlSignalP = 0;
			controlSignalI = 0;
			controlSignalD = 0;
			controlSignalFF = 0;
			controlSignal = 0;
			controlError[0] = 0;
			controlError[1] = 0;
			controlError[2] = 0;

			// Initialize desired speed
			MEMORY_SetDesiredSpeed(0);

			// Set control mode
			controlMode = MODE_SPEED_CONTROL_WITHOUT_TRAJECTORY;

		}

		// Set desired values
		referenceSpeedValue = MEMORY_GetDesiredSpeed();
		referenceAccelerationValue = MEMORY_GetDesiredAcceleration();

		break;

	case MODE_SPEED_CONTROL_WITH_TRAJECTORY:

		// Reset control errors and signals after switching control mode
		if (controlMode != MODE_SPEED_CONTROL_WITH_TRAJECTORY) {

			controlSignalP = 0;
			controlSignalI = 0;
			controlSignalD = 0;
			controlSignalFF = 0;
			controlSignal = 0;
			controlError[0] = 0;
			controlError[1] = 0;
			controlError[2] = 0;

			// Initialize desired speed
			MEMORY_SetDesiredSpeed(0);

			// Set control mode
			controlMode = MODE_SPEED_CONTROL_WITH_TRAJECTORY;

		}

		// Update trajectory generation and set desired values
		TRAJECTORY_GENERATION_UpdateSpeedTrajectory();
		referenceSpeedValue = TRAJECTORY_GENERATION_GetReferenceSpeed();
		referenceAccelerationValue = TRAJECTORY_GENERATION_GetReferenceAcceleration();

		break;

	case MODE_TORQUE_CONTROL:

		// Reset control errors and signals after switching control mode
		if (controlMode != MODE_TORQUE_CONTROL) {

			controlSignalP = 0;
			controlSignalI = 0;
			controlSignalD = 0;
			controlSignal = 0;
			controlError[0] = 0;
			controlError[1] = 0;
			controlError[2] = 0;

			// Initialize desired torque
			//MEMORY_SetDesiredTorque(0);

			// Set control mode
			controlMode = MODE_TORQUE_CONTROL;

		}

		// Set desired values
		//desiredTorqueValue = MEMORY_GetDesiredTorque();

		break;

	case MODE_PULSE_WIDTH_CONTROL:

		// Reset control errors and signals after switching control mode
		if (controlMode != MODE_PULSE_WIDTH_CONTROL) {

			controlSignalP = 0;
			controlSignalI = 0;
			controlSignalD = 0;
			controlSignal = 0;
			controlError[0] = 0;
			controlError[1] = 0;
			controlError[2] = 0;

			// Initialize desired pulse width
			MEMORY_SetDesiredPulseWidth(0);

			// Set control mode
			controlMode = MODE_PULSE_WIDTH_CONTROL;

			break;

		}

		// Apply pulse width to motor
		MOTOR_Run(MEMORY_GetDesiredPulseWidth());

	default:

		MEMORY_SetError(ERROR_PARAMETER_RANGE);
		MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);
		return;

	}

	// Calculate control signal
	if ((controlMode == MODE_POSITION_CONTROL_WITHOUT_TRAJECTORY) || (controlMode == MODE_POSITION_CONTROL_WITH_TRAJECTORY)) {

		// Get new control error and control error derivative
		controlError[0] = FXP32_Mul((referencePositionValue - MEMORY_GetActualPosition()), MEMORY_GetPositionComplianceParameter());
		controlError[2] = (fxp32_t)((((fxp64_t)controlError[2] * 63) + FXPTI_Div((controlError[0] - controlError[1]), CONTROL_TIME_INTERVAL)) >> 6);

		// Calculate feed-forward control signal
		controlSignalFF = FXP64_Mul(referenceSpeedValue, FEED_FORWARD_FACTOR_SPEED32)
				+ FXP64_Mul(referenceAccelerationValue, FEED_FORWARD_FACTOR_ACCELERATION32)
				+ FXP64_Mul(MEMORY_GetActualTorque(), FEED_FORWARD_FACTOR_TORQUE32);
		controlSignalFF = (controlSignalFF * MEMORY_GetFeedForwardControlParameter()) >> FXP32_Q;

		// Calculate proportional position control signal
		controlSignalP = FXP64_Mul(controlError[0], MEMORY_GetProportionalPositionControlParameter());

		// Calculate integral and derivative position control signal
		if (COMMON_ABSOLUTE((controlSignalFF + controlSignalP + controlSignalI)) < actualVoltage) {

			controlSignalI += FXP64_Mul(controlError[0], MEMORY_GetIntegralPositionControlParameter());
			controlSignalD = FXP64_Mul(controlError[2], MEMORY_GetDerivativePositionControlParameter());

		}

	} else if ((controlMode == MODE_SPEED_CONTROL_WITHOUT_TRAJECTORY) || (controlMode == MODE_SPEED_CONTROL_WITH_TRAJECTORY)) {

		// Get new control error and filtered control error derivative
		controlError[0] = FXP32_Mul((referenceSpeedValue - MEMORY_GetActualSpeed()), MEMORY_GetSpeedComplianceParameter());
		controlError[2] = (fxp32_t)((((fxp64_t)controlError[2] * 63) + FXPTI_Div((controlError[0] - controlError[1]), CONTROL_TIME_INTERVAL)) >> 6);

		controlSignalFF = FXP64_Mul(referenceSpeedValue, FEED_FORWARD_FACTOR_SPEED32)
				+ FXP64_Mul(referenceAccelerationValue, FEED_FORWARD_FACTOR_ACCELERATION32)
				+ FXP64_Mul(MEMORY_GetActualTorque(), FEED_FORWARD_FACTOR_TORQUE32);
		controlSignalFF = (controlSignalFF * MEMORY_GetFeedForwardControlParameter()) >> FXP32_Q;

		// Calculate proportional speed control signal
		controlSignalP = FXP64_Mul(controlError[0], MEMORY_GetProportionalSpeedControlParameter());

		// Calculate integral and derivative speed control signal
		if (COMMON_ABSOLUTE((controlSignalFF + controlSignalP + controlSignalI)) < actualVoltage) {

			controlSignalI += FXP64_Mul(controlError[0], MEMORY_GetIntegralSpeedControlParameter());
			controlSignalD = FXP64_Mul(controlError[2], MEMORY_GetDerivativeSpeedControlParameter());

		}

	} else if (controlMode == MODE_TORQUE_CONTROL) {

		// ToDo: Implement torque control
		controlSignalFF = 0;
		controlSignalP = 0;
		controlSignalI = 0;
		controlSignalD = 0;

	}

	// Apply control signal
	if (controlMode != MODE_PULSE_WIDTH_CONTROL) {

		// Update old control errors
		controlError[1] = controlError[0];

		// Compose and convert control signal
		controlSignal = controlSignalFF + controlSignalP + controlSignalI + controlSignalD;
		if (controlSignal > actualVoltage) {

			controlSignal = MOTOR_DUTY_CYCLE_LIMIT;

		} else if (controlSignal < -actualVoltage) {

			controlSignal = -MOTOR_DUTY_CYCLE_LIMIT;

		} else {

			controlSignal = (controlSignal * MOTOR_DUTY_CYCLE_LIMIT) / actualVoltage;

		}

		// Apply control signal to motor
        if (((COMMON_ABSOLUTE(controlSignal) << FXP32_Q) / MOTOR_DUTY_CYCLE_LIMIT) > MEMORY_GetSkipZoneParameter()) {

            MOTOR_Run((int32_t)controlSignal);

        } else {

            MOTOR_Run(0);

        }

	}

}

#endif
