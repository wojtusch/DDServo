/*
 ******************************************************************************
 * File    		task.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for task management functions.
 * Peripherals	TIM1
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __TASK_H
#define __TASK_H

// Type includes
#include "stm32f10x.h"

// Defines
#ifndef INLINE_TASK
#define INLINE_TASK extern inline
#endif
#define SYSTEM_TIME_MULTIPLIER 4
#define OBSERVER_MULTIPLIER 20
#define CONTROL_MULTIPLIER 40
#define SIGNAL_MULTIPLIER 500
#define LOGGING_UPDATE_MULTIPLIER 40000
#define LOGGING_SAVING_MULTIPLIER 10
#define OBSERVER_TIME_INTERVAL (fxpTI_t)(((((25LL << (FXP32_Q + FXPTI_Q + 1)) * OBSERVER_MULTIPLIER) / 1000000) + 1) >> 1)
#define CONTROL_TIME_INTERVAL (fxpTI_t)(((((25LL << (FXP32_Q + FXPTI_Q + 1)) * CONTROL_MULTIPLIER) / 1000000) + 1) >> 1)
#define TRAJECTORY_TIME_INTERVAL CONTROL_TIME_INTERVAL

// Global variables
extern uint8_t tasks;
extern uint8_t timers;
extern uint16_t statusReturnDelayCounter;
extern uint16_t receptionTimemoutCounter;
extern uint16_t synchronizedReadTimeoutCounter;

// Global function prototypes
void TASK_Initialize(void);
void TASK_Activate(void);
void TASK_Deactivate(void);
void TASK_SetStatusReturnDelay(uint16_t);
void TASK_SetReceptionTimeout(uint16_t);
void TASK_ResetReceptionTimeout(void);
void TASK_SetSynchronizedReadTimeout(uint16_t);
void TASK_ResetSynchronizedReadTimeout(void);
void TASK_SetTask(uint8_t);
void TASK_ResetTask(uint8_t);
uint8_t TASK_CheckTask(uint8_t);
void TASK_SetTimer(uint8_t);
void TASK_ResetTimer(uint8_t);
uint8_t TASK_CheckTimer(uint8_t);

// Module includes
#include "communication.h"
#include "logging.h"
#include "memory.h"

// Activate task management
INLINE_TASK void TASK_Activate(void) {

	// Activate TIM1
	TIM_Cmd(TIM1, ENABLE);

}

// Deactivate task management
INLINE_TASK void TASK_Deactivate(void) {

	// Deactivate TIM1
	TIM_Cmd(TIM1, DISABLE);

}

// Set status return delay
INLINE_TASK void TASK_SetStatusReturnDelay(uint16_t counter) {

	statusReturnDelayCounter = counter;

}

// Set reception timeout
INLINE_TASK void TASK_SetReceptionTimeout(uint16_t counter) {

	receptionTimemoutCounter = counter;

}

// Reset reception timeout
INLINE_TASK void TASK_ResetReceptionTimeout(void) {

	receptionTimemoutCounter = 0;

}

// Set synchronized read timeout
INLINE_TASK void TASK_SetSynchronizedReadTimeout(uint16_t counter) {

	synchronizedReadTimeoutCounter = counter;

}

// Reset synchronized read timeout
INLINE_TASK void TASK_ResetSynchronizedReadTimeout(void) {

	synchronizedReadTimeoutCounter = 0;

}

// Set the task status specified by task pointer
INLINE_TASK void TASK_SetTask(uint8_t taskPointer) {

	tasks |= (1 << taskPointer);

}

// Reset the task status specified by task pointer
INLINE_TASK void TASK_ResetTask(uint8_t taskPointer) {

	tasks &= ~(1 << taskPointer);

}

// Check the task status specified by task pointer
INLINE_TASK uint8_t TASK_CheckTask(uint8_t taskPointer) {

	if(tasks & (1 << taskPointer)) {

		return 1;

	} else {

		return 0;

	}

}

// Set the timer status specified by timer pointer
INLINE_TASK void TASK_SetTimer(uint8_t timerPointer) {

	timers |= (1 << timerPointer);

}

// Reset the timer status specified by timer pointer
INLINE_TASK void TASK_ResetTimer(uint8_t timerPointer) {

	timers &= ~(1 << timerPointer);

}

// Check the timer status specified by timer pointer
INLINE_TASK uint8_t TASK_CheckTimer(uint8_t timerPointer) {

	if(timers & (1 << timerPointer)) {

		return 1;

	} else {

		return 0;

	}

}

#endif
