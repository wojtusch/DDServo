/*
 ******************************************************************************
 * File    		common.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Common functions.
 * Peripherals	GPIOB, IWDG, SysTick
 ******************************************************************************
 */

// Includes
#include "common.h"
#include "measurement.h"
#include "memory.h"
#include "signal.h"
#include "task.h"

// Global variables
volatile uint32_t delayCounter;

// Private function prototypes
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);

// Initialize common functions
void COMMON_Initialize(void) {

	// Private variables
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Initialize variables
	delayCounter = 0;

	// Initialize interrupt priority grouping
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// Initialize bootloader output
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Initialize system tick timer with 1ms
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	if (SysTick_Config(SystemCoreClock / 1000)) {

		// Catch initialization error
		SIGNAL_Error();
		MOTOR_Deactivate();
		while (1) {

			// Do nothing

		}

	}

	// Initialize watchdog timer with 100ms
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_8);
	IWDG_SetReload(500);
	IWDG_ReloadCounter();

}

// Reboot controller
void COMMON_Reboot(void) {

	// Deactivate servo functions
	MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);
	TASK_Deactivate();
	MEASUREMENT_DeactivatePositionSensor();
	MEASUREMENT_DeactivateAnalogConverter();

	// Reboot controller
	NVIC_SystemReset();

}

// Reboot controller from system memory and start the bootloader
void COMMON_Bootloader(void) {

	// Deactivate servo functions
	MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);
	TASK_Deactivate();
	MEASUREMENT_DeactivatePositionSensor();
	MEASUREMENT_DeactivateAnalogConverter();

	// Start charging the external capacitor
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);

	// Wait until capacity is fully charged
	COMMON_Delay(50);

	// Reboot controller
	NVIC_SystemReset();

}

// Activate watchdog timer
void COMMON_ActivateWatchdog(void) {

	IWDG_Enable();

}

// Reload watchdog timer
void COMMON_ReloadWatchdog(void) {

	IWDG_ReloadCounter();

}

// Delay execution for the specified time in ms
void COMMON_Delay(uint32_t milliSeconds) {

	delayCounter = milliSeconds;
	while (delayCounter != 0);

}

// Non-maskable interrupt
void NMI_Handler(void) {
}

// Hard fault exception
void HardFault_Handler(void) {

	// Catch hard fault exception
	SIGNAL_Error();
	MOTOR_Deactivate();
	while (1) {

		// Reload watchdog timer
		COMMON_ReloadWatchdog();

	}

}

// Memory manage exception.
void MemManage_Handler(void) {

	// Catch hard fault exception
	SIGNAL_Error();
	MOTOR_Deactivate();
	while (1) {

		// Reload watchdog timer
		COMMON_ReloadWatchdog();

	}

}

// Bus fault exception
void BusFault_Handler(void) {

	// Catch hard fault exception
	SIGNAL_Error();
	MOTOR_Deactivate();
	while (1) {

		// Reload watchdog timer
		COMMON_ReloadWatchdog();

	}

}

// Usage fault exception
void UsageFault_Handler(void) {

	// Catch hard fault exception
	SIGNAL_Error();
	MOTOR_Deactivate();
	while (1) {

		// Reload watchdog timer
		COMMON_ReloadWatchdog();

	}

}

// System service call
void SVC_Handler(void) {
}

// Pending system service call
void PendSV_Handler(void) {
}

// Debug monitor exception
void DebugMon_Handler(void) {
}

// SysTick Handler
void SysTick_Handler(void) {

	// Decrement delay counter
	if(delayCounter > 0)
		delayCounter--;

}
