/*
 ******************************************************************************
 * File    		task.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Task management functions.
 * Peripherals	TIM1
 ******************************************************************************
 */

// Includes
#define INLINE_TASK
#include "task.h"

// Global variables
uint8_t tasks;
uint8_t timers;
uint8_t systemTimeCounter;
uint16_t signalCounter;
uint8_t observerCounter;
uint8_t controlCounter;
uint16_t loggingUpdateCounter;
uint8_t loggingSavingCounter;
uint16_t statusReturnDelayCounter;
uint16_t receptionTimemoutCounter;
uint16_t synchronizedReadTimeoutCounter;

// Private function prototypes
void TIM1_UP_IRQHandler(void);

// Initialization of task management
void TASK_Initialize(void) {

	// Private variables
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Initialize variables
	tasks = 0;
	timers = 0;
	systemTimeCounter = 0;
	observerCounter = 0;
	controlCounter = 0;
	signalCounter = 0;
	loggingUpdateCounter = 0;
	loggingSavingCounter = 0;
	statusReturnDelayCounter = 0;
	receptionTimemoutCounter = 0;
	synchronizedReadTimeoutCounter = 0;

	// Initialize TIM1 as timer a period of 25µs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 1799;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

}

// TIM interrupt handler
void TIM1_UP_IRQHandler(void) {

	// TIM update
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {

		// Clear interrupt flag
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		// System time task
		if (systemTimeCounter == (SYSTEM_TIME_MULTIPLIER - 1)) {

			// Increment actual system time
			MEMORY_IncrementActualSystemTime();

		}
		COMMON_ADVANCE(systemTimeCounter, SYSTEM_TIME_MULTIPLIER);

		// Observer task
		if (observerCounter == (OBSERVER_MULTIPLIER - 1)) {

			// Check for task management timeout
			if (TASK_CheckTask(TASK_OBSERVER)) {

				MEMORY_SetError(ERROR_TASK_MANAGEMENT);

			}

			// Set observer task
			TASK_SetTask(TASK_OBSERVER);

		}
		COMMON_ADVANCE(observerCounter, OBSERVER_MULTIPLIER);

		// Control task
		if ((controlCounter == (CONTROL_MULTIPLIER - 1)) && (MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED) == MODE_MOTOR_POWER_ON)) {

			// Check for task management timeout
			if (TASK_CheckTask(TASK_CONTROL)) {

				MEMORY_SetError(ERROR_TASK_MANAGEMENT);

			}

			// Set control task
			TASK_SetTask(TASK_CONTROL);

		}
		COMMON_ADVANCE(controlCounter, CONTROL_MULTIPLIER);

		// Signal task
		if ((signalCounter == (SIGNAL_MULTIPLIER - 1)) && !(MEMORY_GetErrors() & MEMORY_GetAlarmSignal()) && !MEMORY_CheckFlag(FLAG_COMMUNICATION_COLOR_PING_REQUEST)) {

			// Check for task management timeout
			if (TASK_CheckTask(TASK_SIGNAL)) {

				MEMORY_SetError(ERROR_TASK_MANAGEMENT);

			}

			// Set signal task
			TASK_SetTask(TASK_SIGNAL);

		}
		COMMON_ADVANCE(signalCounter, SIGNAL_MULTIPLIER);

		// Logging task
		if ((loggingUpdateCounter == (LOGGING_UPDATE_MULTIPLIER - 1)) && MEMORY_CheckFlag(FLAG_LOGGING_ACTIVATED)) {

			// Check for task management timeout
			if (TASK_CheckTask(TASK_LOGGING_UPDATE)) {

				MEMORY_SetError(ERROR_TASK_MANAGEMENT);

			}

			// Set logging update task
			TASK_SetTask(TASK_LOGGING_UPDATE);

			if (loggingSavingCounter == (LOGGING_SAVING_MULTIPLIER - 1)) {


				// Check for task management timeout
				if (TASK_CheckTask(TASK_LOGGING_SAVING)) {

					MEMORY_SetError(ERROR_TASK_MANAGEMENT);

				}

				// Set logging saving task
				TASK_SetTask(TASK_LOGGING_SAVING);

			}
			COMMON_ADVANCE(loggingSavingCounter, LOGGING_SAVING_MULTIPLIER);

		}
		COMMON_ADVANCE(loggingUpdateCounter, LOGGING_UPDATE_MULTIPLIER);

		// Status return delay
		if (statusReturnDelayCounter > 0) {

			if (--statusReturnDelayCounter == 0) {

				// Set status return delay timer
				TASK_SetTimer(TIMER_STATUS_RETURN_DELAY);

			}

		}

		// Reception timeout
		if (receptionTimemoutCounter > 0) {

			if (--receptionTimemoutCounter == 0) {

				// Set reception timeout timer
				TASK_SetTimer(TIMER_RECEPTION_TIMEOUT);

			}

		}

		// Synchronized read timeout
		if (synchronizedReadTimeoutCounter > 0) {

			if (--synchronizedReadTimeoutCounter == 0) {

				// Set synchronized read timeout timer
				TASK_SetTimer(TIMER_SYNCHRONIZED_READ_TIMEOUT);

			}

		}

	}

}
