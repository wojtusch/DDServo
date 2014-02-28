 /*
 ******************************************************************************
 * File    		motor.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for motor functions.
 * Peripherals	GPIOA, TIM2
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __MOTOR_H
#define __MOTOR_H

// Type includes
#include "stm32f10x.h"
#include "fixpoint.h"

// Defines
#ifndef INLINE_MOTOR
#define INLINE_MOTOR extern inline
#endif
#define MOTOR_DUTY_CYCLE_LIMIT 4600

// Global variables
extern fxp32_t armatureVoltage;
extern int16_t dutyCycle;

// Global function prototypes
void MOTOR_Initialize(void);
void MOTOR_Activate(void);
void MOTOR_Deactivate(void);
void MOTOR_Run(int32_t);
void MOTOR_Brake(void);
fxp32_t MOTOR_GetArmatureVoltage(void);
int16_t MOTOR_GetDutyCycle(void);

// Module includes
#include "common.h"
#include "memory.h"

// Activate motor
INLINE_MOTOR void MOTOR_Activate(void) {

	// Reset armature voltage and duty cycle
	armatureVoltage = 0;
	dutyCycle = 0;

	// Set MOTE
	GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);

}

// Deactivate motor
INLINE_MOTOR void MOTOR_Deactivate(void) {

	// Reset armature voltage and duty cycle
	armatureVoltage = 0;
	dutyCycle = 0;

	// Reset MOTE
	GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);

}

// Run motor with specified duty cycle (positive values result in a clockwise
// turns, negative values result in counterclockwise turns, absolute maximum
// value is MOTOR_DUTY_CYCLE_LIMIT)
INLINE_MOTOR void MOTOR_Run(int32_t rawDutyCycle) {

	// Run motor only, if motor power is activated
	if (MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		if (rawDutyCycle < 0) {

			// Limit duty cycle value
			if (rawDutyCycle < -MOTOR_DUTY_CYCLE_LIMIT) {

				dutyCycle = -MOTOR_DUTY_CYCLE_LIMIT;

			} else {

				dutyCycle = (int16_t)rawDutyCycle;

			}

			// Set PWM values to turn motor counterclockwise
			if (CONFIGURATION_MODEL_NUMBER == 128) {

				TIM_SetCompare1(TIM2, COMMON_ABSOLUTE(dutyCycle));
				TIM_SetCompare2(TIM2, 0);

			} else if (CONFIGURATION_MODEL_NUMBER == 164) {

				TIM_SetCompare1(TIM2, 0);
				TIM_SetCompare2(TIM2, COMMON_ABSOLUTE(dutyCycle));

			}

		} else {

			// Limit duty cycle value
			if (rawDutyCycle > MOTOR_DUTY_CYCLE_LIMIT) {

				dutyCycle = MOTOR_DUTY_CYCLE_LIMIT;

			} else {

				dutyCycle = (int16_t)rawDutyCycle;

			}

			// Set PWM values to turn motor clockwise
			if (CONFIGURATION_MODEL_NUMBER == 128) {

				TIM_SetCompare1(TIM2, 0);
				TIM_SetCompare2(TIM2, dutyCycle);

			} else if (CONFIGURATION_MODEL_NUMBER == 164) {

				TIM_SetCompare1(TIM2, dutyCycle);
				TIM_SetCompare2(TIM2, 0);

			}

		}

	} else {

		// Deactivate motor
		MOTOR_Deactivate();
		dutyCycle = 0;

	}

	// Set actual pulse width
	MEMORY_SetActualPulseWidth(dutyCycle);

	// Calculate armature voltage
	armatureVoltage = (fxp32_t)(((int64_t)dutyCycle * FXP16to32(MEMORY_GetActualVoltage())) / MOTOR_DUTY_CYCLE_LIMIT);

}

// Brake motor
INLINE_MOTOR void MOTOR_Brake(void) {

	// Set PWM values
	TIM_SetCompare1(TIM2, 0);
	TIM_SetCompare2(TIM2, 0);

}

// Get armature voltage
INLINE_MOTOR fxp32_t MOTOR_GetArmatureVoltage(void) {

	return armatureVoltage;

}

// Get duty cycle
INLINE_MOTOR int16_t MOTOR_GetDutyCycle(void) {

	return dutyCycle;

}

#endif
