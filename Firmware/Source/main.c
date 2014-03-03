/*
 ******************************************************************************
 * File    main.c
 * Author  wojtusch@sim.tu-darmstadt.de
 * Brief   Main program.
 ******************************************************************************
 */

// Includes
#include "stm32f10x.h"
#include "common.h"
#include "communication.h"
#include "communicationProtocol.h"
#include "control.h"
#include "logging.h"
#include "measurement.h"
#include "memory.h"
#include "motor.h"
#include "observer.h"
#include "signal.h"
#include "task.h"
#include "trajectoryGeneration.h"

// Main program
int main(void) {

	// Initialize modules
	COMMON_Initialize();
	SIGNAL_Initialize();
	MEASUREMENT_Initialize();
	MOTOR_Initialize();
	TASK_Initialize();
	MEMORY_Initialize();
	COMMUNICATION_Initialize();
	//LOGGING_Initialize();

	// Activate measurements
	MEASUREMENT_ActivateAnalogConverter();
	MEASUREMENT_ActivatePositionSensor();

	// Wait for stabilized measurements
    COMMON_Delay(5);

	// Calibrate controller temperature sensor
	if (MEMORY_CheckFlag(FLAG_TEMPERATURE_SENSOR_CALIBRATION)) {

		// Reset flag for controller temperature sensor calibration
		MEMORY_ResetFlag(FLAG_TEMPERATURE_SENSOR_CALIBRATION);

		// Calibrate controller temperature sensor
		MEASUREMENT_CalibrateTemperatureSensor();

	}

	// Initialize state observer and control
	OBSERVER_Initialize();
	TRAJECTORY_GENERATION_Initialize();
	CONTROL_Initialize();

	// Activate signal
	SIGNAL_Activate();

	// Activate task management
	TASK_Activate();

	// Activate watchdog timer
	COMMON_ActivateWatchdog();

	// Clear communication status register
	COMMUNICATION_ClearStatusRegister();

	while (1) {

		// Observer task
		if (TASK_CheckTask(TASK_OBSERVER)) {

			// Update estimation
			OBSERVER_UpdateEstimation();

			// Reset observer task
			TASK_ResetTask(TASK_OBSERVER);

		}

		// Control task
		if (TASK_CheckTask(TASK_CONTROL)) {

			// Update control signal
			CONTROL_UpdateControlSignal();

			// Reset control task
			TASK_ResetTask(TASK_CONTROL);

		}

		// Signal task
		if (TASK_CheckTask(TASK_SIGNAL)) {

			// Update signal
			SIGNAL_Update();

			// Reset signal task
			TASK_ResetTask(TASK_SIGNAL);

		}

		// Logging update task
		if (TASK_CheckTask(TASK_LOGGING_UPDATE)) {

			// Update logging values
			/*LOGGING_UpdateEndurance();
			if (MEMORY_CheckError(ERROR_CURRENT_LIMIT)) {

				LOGGING_UpdateOvercurrentTime();

			}
			if (MEMORY_CheckError(ERROR_MOTOR_TEMPERATURE_LIMIT)) {

				LOGGING_UpdateOvertemperatureTime();

			}*/

			// Reset logging update task
			TASK_ResetTask(TASK_LOGGING_UPDATE);

		}

		// Logging saving task
		if (TASK_CheckTask(TASK_LOGGING_SAVING) && (EEPROM0_GetStatus() != EEPROM_BUSY)) {

			// Save logging values
			//LOGGING_Save();

			// Reset logging saving task
			TASK_ResetTask(TASK_LOGGING_SAVING);

		}

		// Status return delay timer
		if (TASK_CheckTimer(TIMER_STATUS_RETURN_DELAY)) {

			// Process end of status return delay
			COMMUNICATION_ProcessStatusReturnDelay();

			// Reset status return delay timer
			TASK_ResetTimer(TIMER_STATUS_RETURN_DELAY);

		}

		// Reception timeout timer
		if (TASK_CheckTimer(TIMER_RECEPTION_TIMEOUT)) {

			// Process reception timeout
			COMMUNICATION_ProcessReceptionTimeout();

			// Reset reception timeout timer
			TASK_ResetTimer(TIMER_RECEPTION_TIMEOUT);

		}

		// Synchronized read timeout timer
		if (TASK_CheckTimer(TIMER_SYNCHRONIZED_READ_TIMEOUT)) {

			// Process synchronized read timeout
			COMMUNICATION_PROTOCOL_ProcessSynchronizedReadTimeout();

			// Reset synchronized read timeout timer
			TASK_ResetTimer(TIMER_SYNCHRONIZED_READ_TIMEOUT);

		}

		// Process communication packets
		if (COMMUNICATION_CheckPacketStatus() == COMMUNICATION_PROCESS) {

			// Process communication packet
			COMMUNICATION_PROTOCOL_ProcessPacket();

		}

		// Reload watchdog timer
		COMMON_ReloadWatchdog();

	}
}
