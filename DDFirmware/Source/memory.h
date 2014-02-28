/*
 ******************************************************************************
 * File    		memory.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for memory functions.
 * Peripherals
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __MEMORY_H
#define __MEMORY_H

// Type includes
#include "stm32f10x.h"
#include "fixpoint.h"

// Defines
#ifndef INLINE_MEMORY
#define INLINE_MEMORY extern inline
#endif

// Type defines
typedef struct {

	// This structure represents all relevant servo actuator variables that can be set or read via the communication
	// interface. The variables are organized in blocks of 32 bit without overlapping to ensure an access of each value
	// with one memory access only. When new variables are added to the structure, this scheme should be maintained.

	// Volatile copy of non-volatile values
	uint16_t modelNumber; 							// 0 ... 65535 | DD-28: 29, DD-64: 65
	uint16_t firmwareVersion; 						// 0 ... 65535 | read-only
	uint8_t servoId; 								// 0 ... 253
	uint8_t statusReturnLevel; 						// 0 ... 2 | respond to PING only: 0, respond to READ, SYNC_READ and PING: 1, respond to all instructions: 2
	uint16_t statusReturnDelayTime; 				// 0 ... 65535 in 25µs
	uint16_t baudrateDivider; 						// 16 ... 65535 | baudrateDivider as Q12.4 = 72MHz / baudrate in bit/s
	uint16_t positionOffset;						// 0 ... 4095 in encoder steps
	fxp32_t positionLimitClockwise; 				// 0 ... 32768* in rad as Q16.16
	fxp32_t positionLimitCounterclockwise; 			// 0 ... 32768* in rad as Q16.16
	fxp32_t torqueLimit;	 						// 0 ... 32768* in Nm as Q16.16
	fxp32_t currentLimit;	 						// 0 ... 32768* in A as Q16.16
	fxp16_t voltageLimitLow; 						// 11.5 ... 25.5 in V as Q10.6
	fxp16_t voltageLimitHigh; 						// 11.5 ... 25.5 in V as Q10.6
	fxp16_t motorTemperatureLimit; 					// 30 ... 150 in °C as Q10.6
	fxp16_t controllerTemperatureLimit; 			// 30 ... 150 in °C as Q10.6
	fxp16_t controllerTemperatureOffset; 			// -512 ... 512* in °C as Q10.6
	uint16_t statusSignal; 							// 0 ... 65535 | bit 0 to bit 7: color mode, bit 8 to bit 15: operation mode
	uint16_t alarmSignal; 							// 0 ... 65535
	uint16_t alarmShutdown; 						// 0 ... 65535
    fxp32_t skipZoneParameter;          			// 0 ... 1 as Q16.16
	fxp32_t feedForwardControlParameter;			// -32768 ... 32768* as Q16.16
	fxp32_t proportionalPositionControlParameter;	// -32768 ... 32768* as Q16.16
	fxp32_t integralPositionControlParameter;		// -32768 ... 32768* as Q16.16
	fxp32_t derivativePositionControlParameter;		// -32768 ... 32768* as Q16.16
	fxp32_t positionComplianceParameter;			// 0 ... 1 as Q16.16
	fxp32_t proportionalSpeedControlParameter;		// -32768 ... 32768* as Q16.16
	fxp32_t integralSpeedControlParameter;			// -32768 ... 32768* as Q16.16
	fxp32_t derivativeSpeedControlParameter;		// -32768 ... 32768* as Q16.16
	fxp32_t speedComplianceParameter;				// 0 ... 1 as Q16.16

	// Volatile values
	volatile fxp32_t desiredPosition; 				// -32768 ... 32768* in rad as Q16.16
	volatile fxp32_t desiredSpeed; 					// -32768 ... 32768* in rad/s as Q16.16
	volatile fxp32_t desiredAcceleration;			// -32768 ... 32768* in rad/s as Q16.16
	volatile int16_t desiredPulseWidth;				// -4600 ... 4600 in timer steps
	volatile int16_t actualPulseWidth;				// -4600 ... 4600 in timer steps | read-only
	volatile uint16_t actualPositionMeasurement;	// 0 ... 4095 in encoder steps | read-only
	volatile uint16_t actualPositionStatus;			// 0 ... 31 | bit 0: even parity, bit 1: magnitude increase, bit 2: magnitude decrease, bit 3: linearity alarm, bit 4: cordic overflow
													// bit 5: offset compensation finished | read-only
	volatile uint32_t actualSystemTime; 			// 0 ... 4294967295 in 0.1ms | read-only
	volatile fxp32_t actualPosition; 				// -32768 ... 32768* in rad as Q16.16 | read-only
	volatile fxp32_t actualSpeed; 					// -32768 ... 32768* in rad/s as Q16.16 | read-only
	volatile fxp32_t actualTorque; 					// -32768 ... 32768* in Nm as Q16.16 | read-only
	volatile fxp32_t actualCurrent; 				// -32768 ... 32768* in A as Q16.16 | read-only
	volatile fxp16_t actualVoltage; 				// -512 ... 512* in V as Q10.6 | read-only
	volatile fxp16_t actualMotorTemperature; 		// -512 ... 512* in °C as Q10.6 | read-only
	volatile fxp16_t actualControllerTemperature; 	// -512 ... 512* in °C as Q10.6 | read-only
	volatile uint16_t flags; 						// 0 ... 65535 | read-only
	volatile uint16_t errors; 						// 0 ... 65535 | read-only

	// Non-volatile values
	uint16_t checksum; 								// 0 ... 65535 | read-only

} __attribute__((packed)) structMemoryFields;
typedef union {

	structMemoryFields fields;
	uint8_t data[sizeof(structMemoryFields)];

} unionMemoryFields;

// Global variables
extern unionMemoryFields memoryData;
extern uint8_t controlMode;

// Global function prototypes
void MEMORY_Initialize(void);
void MEMORY_Reset(void);
uint16_t MEMORY_GetFlags(void);
void MEMORY_SetFlags(uint16_t);
void MEMORY_SetFlag(uint8_t);
void MEMORY_ResetFlag(uint8_t);
uint8_t MEMORY_CheckFlag(uint8_t);
uint16_t MEMORY_GetErrors(void);
uint8_t MEMORY_GetCompressedErrors(void);
uint8_t MEMORY_TranslateCompressedErrors(void);
void MEMORY_SetErrors(uint16_t);
void MEMORY_SetError(uint8_t);
void MEMORY_ResetError(uint8_t);
uint8_t MEMORY_CheckError(uint8_t);
uint16_t MEMORY_GetChecksum(void);
uint8_t MEMORY_GetMemoryData(uint8_t);
uint16_t MEMORY_GetModelNumber(void);
void MEMORY_SetModelNumber(uint16_t);
uint16_t MEMORY_GetFirmwareVersion(void);
void MEMORY_SetFirmwareVersion(uint16_t);
uint8_t MEMORY_GetFirmwareChecksum(void);
void MEMORY_SetFirmwareChecksum(uint8_t);
uint8_t MEMORY_GetServoId(void);
void MEMORY_SetServoId(uint8_t);
uint8_t MEMORY_GetStatusReturnLevel(void);
void MEMORY_SetStatusReturnLevel(uint8_t);
uint16_t MEMORY_GetStatusReturnDelayTime(void);
void MEMORY_SetStatusReturnDelayTime(uint16_t);
uint16_t MEMORY_GetBaudrateDivider(void);
void MEMORY_SetBaudrateDivider(uint16_t);
uint32_t MEMORY_GetBaudrate(void);
void MEMORY_SetBaudrate(uint32_t);
uint16_t MEMORY_GetPositionOffset(void);
void MEMORY_SetPositionOffset(uint16_t);
fxp32_t MEMORY_GetPositionLimitClockwise(void);
void MEMORY_SetPositionLimitClockwise(fxp32_t);
fxp32_t MEMORY_GetPositionLimitCounterclockwise(void);
void MEMORY_SetPositionLimitCounterclockwise(fxp32_t);
fxp32_t MEMORY_GetTorqueLimit(void);
void MEMORY_SetTorqueLimit(fxp32_t);
fxp32_t MEMORY_GetCurrentLimit(void);
void MEMORY_SetCurrentLimit(fxp32_t);
fxp16_t MEMORY_GetVoltageLimitLow(void);
void MEMORY_SetVoltageLimitLow(fxp16_t);
fxp16_t MEMORY_GetVoltageLimitHigh(void);
void MEMORY_SetVoltageLimitHigh(fxp16_t);
fxp16_t MEMORY_GetMotorTemperatureLimit(void);
void MEMORY_SetMotorTemperatureLimit(fxp16_t);
fxp16_t MEMORY_GetControllerTemperatureLimit(void);
void MEMORY_SetControllerTemperatureLimit(fxp16_t);
fxp16_t MEMORY_GetControllerTemperatureOffset(void);
void MEMORY_SetControllerTemperatureOffset(fxp16_t);
uint16_t MEMORY_GetStatusSignal(void);
uint8_t MEMORY_GetStatusSignalOperationMode(void);
uint8_t MEMORY_GetStatusSignalColorMode(void);
void MEMORY_SetStatusSignal(uint16_t);
void MEMORY_SetStatusSignalColorMode(uint8_t);
void MEMORY_SetStatusSignalOperationMode(uint8_t);
uint16_t MEMORY_GetAlarmSignal(void);
void MEMORY_SetAlarmSignal(uint16_t);
uint16_t MEMORY_GetAlarmShutdown(void);
void MEMORY_SetAlarmShutdown(uint16_t);
fxp32_t MEMORY_GetSkipZoneParameter(void);
void MEMORY_SetSkipZoneParameter(fxp32_t);
fxp32_t MEMORY_GetFeedForwardControlParameter(void);
void MEMORY_SetFeedForwardControlParameter(fxp32_t);
fxp32_t MEMORY_GetProportionalPositionControlParameter(void);
void MEMORY_SetProportionalPositionControlParameter(fxp32_t);
fxp32_t MEMORY_GetIntegralPositionControlParameter(void);
void MEMORY_SetIntegralPositionControlParameter(fxp32_t);
fxp32_t MEMORY_GetDerivativePositionControlParameter(void);
void MEMORY_SetDerivativePositionControlParameter(fxp32_t);
fxp32_t MEMORY_GetPositionComplianceParameter(void);
void MEMORY_SetPositionComplianceParameter(fxp32_t);
fxp32_t MEMORY_GetProportionalSpeedControlParameter(void);
void MEMORY_SetProportionalSpeedControlParameter(fxp32_t);
fxp32_t MEMORY_GetIntegralSpeedControlParameter(void);
void MEMORY_SetIntegralSpeedControlParameter(fxp32_t);
fxp32_t MEMORY_GetDerivativeSpeedControlParameter(void);
void MEMORY_SetDerivativeSpeedControlParameter(fxp32_t);
fxp32_t MEMORY_GetSpeedComplianceParameter(void);
void MEMORY_SetSpeedComplianceParameter(fxp32_t);
fxp32_t MEMORY_GetDesiredPosition(void);
void MEMORY_SetDesiredPosition(fxp32_t);
fxp32_t MEMORY_GetDesiredSpeed(void);
void MEMORY_SetDesiredSpeed(fxp32_t);
fxp32_t MEMORY_GetDesiredAcceleration(void);
void MEMORY_SetDesiredAcceleration(fxp32_t);
int16_t MEMORY_GetDesiredPulseWidth(void);
void MEMORY_SetDesiredPulseWidth(int16_t);
int16_t MEMORY_GetActualPulseWidth(void);
void MEMORY_SetActualPulseWidth(int16_t);
uint16_t MEMORY_GetActualPositionMeasurement(void);
void MEMORY_SetActualPositionMeasurement(uint16_t);
uint16_t MEMORY_GetActualPositionStatus(void);
void MEMORY_SetActualPositionStatus(uint16_t);
uint32_t MEMORY_GetActualSystemTime(void);
void MEMORY_SetActualSystemTime(uint32_t);
void MEMORY_IncrementActualSystemTime(void);
fxp32_t MEMORY_GetActualPosition(void);
void MEMORY_SetActualPosition(fxp32_t);
fxp32_t MEMORY_GetActualSpeed(void);
void MEMORY_SetActualSpeed(fxp32_t);
fxp32_t MEMORY_GetActualTorque(void);
void MEMORY_SetActualTorque(fxp32_t);
fxp32_t MEMORY_GetActualCurrent(void);
void MEMORY_SetActualCurrent(fxp32_t);
fxp16_t MEMORY_GetActualVoltage(void);
void MEMORY_SetActualVoltage(fxp16_t);
fxp16_t MEMORY_GetActualMotorTemperature(void);
void MEMORY_SetActualMotorTemperature(fxp16_t);
fxp16_t MEMORY_GetActualControllerTemperature(void);
void MEMORY_SetActualControllerTemperature(fxp16_t);
void MEMORY_SetChecksum(uint16_t);

// Module includes
#include "common.h"
#include "control.h"
#include "communication.h"
#include "eeprom0.h"
#include "eeprom1.h"
#include "motor.h"
#include "signal.h"
#include "trajectoryGeneration.h"

// Get memory data from defined address
INLINE_MEMORY uint8_t MEMORY_GetMemoryData(uint8_t address) {

	return memoryData.data[address];

}

// Get model number
INLINE_MEMORY uint16_t MEMORY_GetModelNumber(void) {

	return memoryData.fields.modelNumber;

}

// Set model number
INLINE_MEMORY void MEMORY_SetModelNumber(uint16_t modelNumber) {

	// Save value to ram and eeprom
	memoryData.fields.modelNumber = modelNumber;
	// Write value to eeprom only, if motor power is deactivated
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_MODEL_NUMBER, modelNumber);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get firmware version
INLINE_MEMORY uint16_t MEMORY_GetFirmwareVersion(void) {

	return memoryData.fields.firmwareVersion;

}

// Set firmware version
INLINE_MEMORY void MEMORY_SetFirmwareVersion(uint16_t firmwareVersion) {

	// Save value to ram
	memoryData.fields.firmwareVersion = firmwareVersion;

}

// Get servo id
INLINE_MEMORY uint8_t MEMORY_GetServoId(void) {

	return memoryData.fields.servoId;

}

// Set servo id
INLINE_MEMORY void MEMORY_SetServoId(uint8_t servoId) {

	if (servoId <= LIMIT_SERVO_ID) {

		// Save value to ram and eeprom
		memoryData.fields.servoId = servoId;
		// Write value to eeprom only, if motor power is deactivated
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_SERVO_ID, servoId);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get status return level
INLINE_MEMORY uint8_t MEMORY_GetStatusReturnLevel(void) {

	return memoryData.fields.statusReturnLevel;

}

// Set status return level
INLINE_MEMORY void MEMORY_SetStatusReturnLevel(uint8_t statusReturnLevel) {

	if (statusReturnLevel <= LIMIT_STATUS_RETURN_LEVEL) {

		// Save value to ram and eeprom
		memoryData.fields.statusReturnLevel = statusReturnLevel;
		// Write value to eeprom only, if motor power is deactivated
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_STATUS_RETURN_LEVEL, statusReturnLevel);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get status return delay time
INLINE_MEMORY uint16_t MEMORY_GetStatusReturnDelayTime(void) {

	return memoryData.fields.statusReturnDelayTime;

}

// Set status return delay time
INLINE_MEMORY void MEMORY_SetStatusReturnDelayTime(uint16_t statusReturnDelayTime) {

	// Save value to ram and eeprom
	memoryData.fields.statusReturnDelayTime = statusReturnDelayTime;
	// Write value to eeprom only, if motor power is deactivated
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_STATUS_RETURN_DELAY_TIME, statusReturnDelayTime);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get baudrate divider
INLINE_MEMORY uint16_t MEMORY_GetBaudrateDivider(void) {

	return memoryData.fields.baudrateDivider;

}

// Set baudrate divider
INLINE_MEMORY void MEMORY_SetBaudrateDivider(uint16_t baudrateDivider) {

	if (baudrateDivider >= LIMIT_BAUDRATE_DIVIDER) {

		// Save value to ram and eeprom and update communication interface
		memoryData.fields.baudrateDivider = baudrateDivider;
		// Write value to eeprom only, if motor power is deactivated
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_BAUDRATE_DIVIDER, baudrateDivider);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}
		COMMUNICATION_UpdateBaudrate();

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get baudrate in bit/s
INLINE_MEMORY uint32_t MEMORY_GetBaudrate(void) {

	if (MEMORY_GetBaudrateDivider() >= LIMIT_BAUDRATE_DIVIDER) {

		return 72000000 / MEMORY_GetBaudrateDivider();

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

		// Set to default value
		MEMORY_SetBaudrateDivider(DEFAULT_BAUDRATE_DIVIDER);
		return (72000000 / DEFAULT_BAUDRATE_DIVIDER);

	}

}

// Set baudrate in bit/s
INLINE_MEMORY void MEMORY_SetBaudrate(uint32_t baudrate) {

	if ((baudrate > LIMIT_BAUDRATE_LOW) && (baudrate <= LIMIT_BAUDRATE_HIGH)) {

		MEMORY_SetBaudrateDivider((((72000000 << 1) / baudrate) + 1) >> 1);

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get position sensor offset
INLINE_MEMORY uint16_t MEMORY_GetPositionOffset(void) {

	return memoryData.fields.positionOffset;

}

// Set position sensor offset
INLINE_MEMORY void MEMORY_SetPositionOffset(uint16_t positionOffset) {

	if (positionOffset <= LIMIT_POSITION_OFFSET) {

		// Save value to ram and eeprom only, if motor power is deactivated
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			memoryData.fields.positionOffset = positionOffset;
			EEPROM0_SetVariable(EEPROM0_POSITION_OFFSET, positionOffset);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get angle limit clockwise
INLINE_MEMORY fxp32_t MEMORY_GetPositionLimitClockwise(void) {

	return memoryData.fields.positionLimitClockwise;

}

// Set angle limit clockwise
INLINE_MEMORY void MEMORY_SetPositionLimitClockwise(fxp32_t positionLimitClockwise) {

	if ((positionLimitClockwise >= LIMIT_POSITION_LIMIT_CLOCKWISE_LOW) && (positionLimitClockwise <= LIMIT_POSITION_LIMIT_CLOCKWISE_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.positionLimitClockwise = positionLimitClockwise;
		// Write value to eeprom only, if motor power is deactivated
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_POSITION_LIMIT_CLOCKWISE_LOW, ((uint16_t)(positionLimitClockwise & 0xFFFF)));
			EEPROM0_SetVariable(EEPROM0_POSITION_LIMIT_CLOCKWISE_HIGH, ((uint16_t)(positionLimitClockwise >> 16)));

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get angle limit counterclockwise
INLINE_MEMORY fxp32_t MEMORY_GetPositionLimitCounterclockwise(void) {

	return memoryData.fields.positionLimitCounterclockwise;

}

// Set angle limit counterclockwise
INLINE_MEMORY void MEMORY_SetPositionLimitCounterclockwise(fxp32_t positionLimitCounterclockwise) {

	if ((positionLimitCounterclockwise >= LIMIT_POSITION_LIMIT_COUNTERCLOCKWISE_LOW) && (positionLimitCounterclockwise <= LIMIT_POSITION_LIMIT_COUNTERCLOCKWISE_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.positionLimitCounterclockwise = positionLimitCounterclockwise;
		// Write value to eeprom only, if motor power is deactivated
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_POSITION_LIMIT_COUNTERCLOCKWISE_LOW, ((uint16_t)(positionLimitCounterclockwise & 0xFFFF)));
			EEPROM0_SetVariable(EEPROM0_POSITION_LIMIT_COUNTERCLOCKWISE_HIGH, ((uint16_t)(positionLimitCounterclockwise >> 16)));

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get torque limit
INLINE_MEMORY fxp32_t MEMORY_GetTorqueLimit(void) {

	return memoryData.fields.torqueLimit;

}

// Set torque limit
INLINE_MEMORY void MEMORY_SetTorqueLimit(fxp32_t torqueLimit) {

	if ((torqueLimit >= LIMIT_TORQUE_LIMIT_LOW) && (torqueLimit <= LIMIT_TORQUE_LIMIT_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.torqueLimit = torqueLimit;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_TORQUE_LIMIT_LOW, ((uint16_t)(torqueLimit & 0xFFFF)));
			EEPROM0_SetVariable(EEPROM0_TORQUE_LIMIT_HIGH, ((uint16_t)(torqueLimit >> 16)));

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get current limit
INLINE_MEMORY fxp32_t MEMORY_GetCurrentLimit(void) {

	return memoryData.fields.currentLimit;

}

// Set current limit
INLINE_MEMORY void MEMORY_SetCurrentLimit(fxp32_t currentLimit) {

	if ((currentLimit >= LIMIT_CURRENT_LIMIT_LOW) && (currentLimit <= LIMIT_CURRENT_LIMIT_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.currentLimit = currentLimit;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_CURRENT_LIMIT_LOW, ((uint16_t)(currentLimit & 0xFFFF)));
			EEPROM0_SetVariable(EEPROM0_CURRENT_LIMIT_HIGH, ((uint16_t)(currentLimit >> 16)));

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get lower voltage limit
INLINE_MEMORY fxp16_t MEMORY_GetVoltageLimitLow(void) {

	return memoryData.fields.voltageLimitLow;

}

// Set lower voltage limit
INLINE_MEMORY void MEMORY_SetVoltageLimitLow(fxp16_t voltageLimitLow) {

	if ((voltageLimitLow >= LIMIT_VOLTAGE_LIMIT_LOW) && (voltageLimitLow <= LIMIT_VOLTAGE_LIMIT_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.voltageLimitLow = voltageLimitLow;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_VOLTAGE_LIMIT_LOW, voltageLimitLow);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get higher voltage limit
INLINE_MEMORY fxp16_t MEMORY_GetVoltageLimitHigh(void) {

	return memoryData.fields.voltageLimitHigh;

}

// Set higher voltage Limit
INLINE_MEMORY void MEMORY_SetVoltageLimitHigh(fxp16_t voltageLimitHigh)  {

	if ((voltageLimitHigh >= LIMIT_VOLTAGE_LIMIT_LOW) && (voltageLimitHigh <= LIMIT_VOLTAGE_LIMIT_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.voltageLimitHigh = voltageLimitHigh;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_VOLTAGE_LIMIT_HIGH, voltageLimitHigh);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get motor temperature limit
INLINE_MEMORY fxp16_t MEMORY_GetMotorTemperatureLimit(void) {

	return memoryData.fields.motorTemperatureLimit;

}

// Set motor temperature limit
INLINE_MEMORY void MEMORY_SetMotorTemperatureLimit(fxp16_t motorTemperatureLimit) {

	if ((motorTemperatureLimit >= LIMIT_MOTOR_TEMPERATURE_LOW) && (motorTemperatureLimit <= LIMIT_MOTOR_TEMPERATURE_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.motorTemperatureLimit = motorTemperatureLimit;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_MOTOR_TEMPERATURE_LIMIT, motorTemperatureLimit);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get controller temperature limit
INLINE_MEMORY fxp16_t MEMORY_GetControllerTemperatureLimit(void) {

	return memoryData.fields.controllerTemperatureLimit;

}

// Set controller temperature limit
INLINE_MEMORY void MEMORY_SetControllerTemperatureLimit(fxp16_t controllerTemperatureLimit) {

	if ((controllerTemperatureLimit >= LIMIT_CONTROLLER_TEMPERATURE_LOW) && (controllerTemperatureLimit <= LIMIT_CONTROLLER_TEMPERATURE_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.controllerTemperatureLimit = controllerTemperatureLimit;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_CONTROLLER_TEMPERATURE_LIMIT, controllerTemperatureLimit);

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get controller temperature sensor offset
INLINE_MEMORY fxp16_t MEMORY_GetControllerTemperatureOffset(void) {

	return memoryData.fields.controllerTemperatureOffset;

}

// Set controller temperature sensor offset
INLINE_MEMORY void MEMORY_SetControllerTemperatureOffset(fxp16_t controllerTemperatureOffset) {

	// Save value to ram and eeprom
	memoryData.fields.controllerTemperatureOffset = controllerTemperatureOffset;
	// Write value to eeprom only, if motor power is deactivated
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_CONTROLLER_TEMPERATURE_OFFSET, controllerTemperatureOffset);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get status signal setting
INLINE_MEMORY uint16_t MEMORY_GetStatusSignal(void) {

	return memoryData.fields.statusSignal;

}

// Set status signal setting
INLINE_MEMORY void MEMORY_SetStatusSignal(uint16_t statusSignal) {

	// Save value to ram and eeprom
	memoryData.fields.statusSignal = statusSignal;
	// Write value to eeprom only, if motor power is deactivated
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_STATUS_SIGNAL, statusSignal);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get status signal operation mode
INLINE_MEMORY uint8_t MEMORY_GetStatusSignalOperationMode(void) {

	return ((uint8_t)(memoryData.fields.statusSignal >> 8));

}

// Set status signal operation mode
INLINE_MEMORY void MEMORY_SetStatusSignalOperationMode(uint8_t statusSignalOperationMode) {

	// Save value to ram and eeprom
	memoryData.fields.statusSignal &= 0x00FF;
	memoryData.fields.statusSignal |= ((uint16_t)(statusSignalOperationMode << 8));
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_STATUS_SIGNAL, memoryData.fields.statusSignal);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get status signal color mode
INLINE_MEMORY uint8_t MEMORY_GetStatusSignalColorMode(void) {

	return ((uint8_t)(memoryData.fields.statusSignal & 0x00FF));

}

// Set status signal color mode
INLINE_MEMORY void MEMORY_SetStatusSignalColorMode(uint8_t statusSignalColorMode) {

	// Save value to ram and eeprom
	memoryData.fields.statusSignal &= 0xFF00;
	memoryData.fields.statusSignal |= ((uint16_t)statusSignalColorMode);
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_STATUS_SIGNAL, memoryData.fields.statusSignal);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get alarm signal setting
INLINE_MEMORY uint16_t MEMORY_GetAlarmSignal(void) {

	return memoryData.fields.alarmSignal;

}

// Set alarm signal setting
INLINE_MEMORY void MEMORY_SetAlarmSignal(uint16_t alarmSignal) {

	// Save value to ram and eeprom
	memoryData.fields.alarmSignal = alarmSignal;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_ALARM_SIGNAL, alarmSignal);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get alarm shutdown setting
INLINE_MEMORY uint16_t MEMORY_GetAlarmShutdown(void) {

	return memoryData.fields.alarmShutdown;

}

// Set alarm shutdown setting
INLINE_MEMORY void MEMORY_SetAlarmShutdown(uint16_t alarmShutdown) {

	// Save value to ram and eeprom
	memoryData.fields.alarmShutdown = alarmShutdown;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_ALARM_SHUTDOWN, alarmShutdown);

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get skip zone parameter
INLINE_MEMORY fxp32_t MEMORY_GetSkipZoneParameter(void) {

    return memoryData.fields.skipZoneParameter;

}

// Set skip zone parameter
INLINE_MEMORY void MEMORY_SetSkipZoneParameter(fxp32_t skipZoneParameter) {

    if ((skipZoneParameter >= LIMIT_SKIP_ZONE_PARAMETER_LOW) && (skipZoneParameter <= LIMIT_SKIP_ZONE_PARAMETER_HIGH)) {

        // Save value to ram and eeprom
        memoryData.fields.skipZoneParameter = skipZoneParameter;
        if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

            EEPROM0_SetVariable(EEPROM0_SKIP_ZONE_PARAMETER_LOW, ((uint16_t)(skipZoneParameter & 0xFFFF)));
            EEPROM0_SetVariable(EEPROM0_SKIP_ZONE_PARAMETER_HIGH, ((uint16_t)(skipZoneParameter >> 16)));

        } else {

            MEMORY_SetError(ERROR_DATA_LOSS);

        }

    } else {

        // Set parameter range error flag
        MEMORY_SetError(ERROR_PARAMETER_RANGE);

    }

}

// Get feed-forward control parameter
INLINE_MEMORY fxp32_t MEMORY_GetFeedForwardControlParameter(void) {

	return memoryData.fields.feedForwardControlParameter;

}

// Set feed-forward control parameter
INLINE_MEMORY void MEMORY_SetFeedForwardControlParameter(fxp32_t feedForwardControlParameter) {

	// Save value to ram and eeprom
	memoryData.fields.feedForwardControlParameter = feedForwardControlParameter;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_FEED_FORWARD_CONTROL_PARAMETER_LOW, ((uint16_t)(feedForwardControlParameter & 0xFFFF)));
		EEPROM0_SetVariable(EEPROM0_FEED_FORWARD_CONTROL_PARAMETER_HIGH, ((uint16_t)(feedForwardControlParameter >> 16)));

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get proportional position control parameter
INLINE_MEMORY fxp32_t MEMORY_GetProportionalPositionControlParameter(void) {

	return memoryData.fields.proportionalPositionControlParameter;

}

// Set proportional position control parameter
INLINE_MEMORY void MEMORY_SetProportionalPositionControlParameter(fxp32_t proportionalPositionControlParameter) {

	// Save value to ram and eeprom
	memoryData.fields.proportionalPositionControlParameter = proportionalPositionControlParameter;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_PROPORTIONAL_POSITION_CONTROL_PARAMETER_LOW, ((uint16_t)(proportionalPositionControlParameter & 0xFFFF)));
		EEPROM0_SetVariable(EEPROM0_PROPORTIONAL_POSITION_CONTROL_PARAMETER_HIGH, ((uint16_t)(proportionalPositionControlParameter >> 16)));

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get integral position control parameter
INLINE_MEMORY fxp32_t MEMORY_GetIntegralPositionControlParameter(void) {

	return memoryData.fields.integralPositionControlParameter;

}

// Set integral position control parameter
INLINE_MEMORY void MEMORY_SetIntegralPositionControlParameter(fxp32_t integralPositionControlParameter) {

	// Save value to ram and eeprom
	memoryData.fields.integralPositionControlParameter = integralPositionControlParameter;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_INTEGRAL_POSITION_CONTROL_PARAMETER_LOW, ((uint16_t)(integralPositionControlParameter & 0xFFFF)));
		EEPROM0_SetVariable(EEPROM0_INTEGRAL_POSITION_CONTROL_PARAMETER_HIGH, ((uint16_t)(integralPositionControlParameter >> 16)));

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get derivative position control parameter
INLINE_MEMORY fxp32_t MEMORY_GetDerivativePositionControlParameter(void) {

	return memoryData.fields.derivativePositionControlParameter;

}

// Set derivative position control parameter
INLINE_MEMORY void MEMORY_SetDerivativePositionControlParameter(fxp32_t derivativePositionControlParameter) {

	// Save value to ram and eeprom
	memoryData.fields.derivativePositionControlParameter = derivativePositionControlParameter;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_DERIVATIVE_POSITION_CONTROL_PARAMETER_LOW, ((uint16_t)(derivativePositionControlParameter & 0xFFFF)));
		EEPROM0_SetVariable(EEPROM0_DERIVATIVE_POSITION_CONTROL_PARAMETER_HIGH, ((uint16_t)(derivativePositionControlParameter >> 16)));

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get position compliance parameter
INLINE_MEMORY fxp32_t MEMORY_GetPositionComplianceParameter(void) {

	return memoryData.fields.positionComplianceParameter;

}

// Set position compliance parameter
INLINE_MEMORY void MEMORY_SetPositionComplianceParameter(fxp32_t positionComplianceParameter) {

	if ((positionComplianceParameter >= LIMIT_POSITION_COMPLIANCE_PARAMETER_LOW) && (positionComplianceParameter <= LIMIT_POSITION_COMPLIANCE_PARAMETER_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.positionComplianceParameter = positionComplianceParameter;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_POSITION_COMPLIANCE_PARAMETER_LOW, ((uint16_t)(positionComplianceParameter & 0xFFFF)));
			EEPROM0_SetVariable(EEPROM0_POSITION_COMPLIANCE_PARAMETER_HIGH, ((uint16_t)(positionComplianceParameter >> 16)));

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get proportional speed control parameter
INLINE_MEMORY fxp32_t MEMORY_GetProportionalSpeedControlParameter(void) {

	return memoryData.fields.proportionalSpeedControlParameter;

}

// Set proportional speed control parameter
INLINE_MEMORY void MEMORY_SetProportionalSpeedControlParameter(fxp32_t proportionalSpeedControlParameter) {

	// Save value to ram and eeprom
	memoryData.fields.proportionalSpeedControlParameter = proportionalSpeedControlParameter;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_PROPORTIONAL_SPEED_CONTROL_PARAMETER_LOW, ((uint16_t)(proportionalSpeedControlParameter & 0xFFFF)));
		EEPROM0_SetVariable(EEPROM0_PROPORTIONAL_SPEED_CONTROL_PARAMETER_HIGH, ((uint16_t)(proportionalSpeedControlParameter >> 16)));

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get integral speed control parameter
INLINE_MEMORY fxp32_t MEMORY_GetIntegralSpeedControlParameter(void) {

	return memoryData.fields.integralSpeedControlParameter;

}

// Set integral speed control parameter
INLINE_MEMORY void MEMORY_SetIntegralSpeedControlParameter(fxp32_t integralSpeedControlParameter) {

	// Save value to ram and eeprom
	memoryData.fields.integralSpeedControlParameter = integralSpeedControlParameter;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_INTEGRAL_SPEED_CONTROL_PARAMETER_LOW, ((uint16_t)(integralSpeedControlParameter & 0xFFFF)));
		EEPROM0_SetVariable(EEPROM0_INTEGRAL_SPEED_CONTROL_PARAMETER_HIGH, ((uint16_t)(integralSpeedControlParameter >> 16)));

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get derivative speed control parameter
INLINE_MEMORY fxp32_t MEMORY_GetDerivativeSpeedControlParameter(void) {

	return memoryData.fields.derivativeSpeedControlParameter;

}

// Set derivative speed control parameter
INLINE_MEMORY void MEMORY_SetDerivativeSpeedControlParameter(fxp32_t derivativeSpeedControlParameter) {

	// Save value to ram and eeprom
	memoryData.fields.derivativeSpeedControlParameter = derivativeSpeedControlParameter;
	if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		EEPROM0_SetVariable(EEPROM0_DERIVATIVE_SPEED_CONTROL_PARAMETER_LOW, ((uint16_t)(derivativeSpeedControlParameter & 0xFFFF)));
		EEPROM0_SetVariable(EEPROM0_DERIVATIVE_SPEED_CONTROL_PARAMETER_HIGH, ((uint16_t)(derivativeSpeedControlParameter >> 16)));

	} else {

		MEMORY_SetError(ERROR_DATA_LOSS);

	}

}

// Get speed compliance parameter
INLINE_MEMORY fxp32_t MEMORY_GetSpeedComplianceParameter(void) {

	return memoryData.fields.speedComplianceParameter;

}

// Set speed compliance parameter
INLINE_MEMORY void MEMORY_SetSpeedComplianceParameter(fxp32_t speedComplianceParameter) {

	if ((speedComplianceParameter >= LIMIT_SPEED_COMPLIANCE_PARAMETER_LOW) && (speedComplianceParameter <= LIMIT_SPEED_COMPLIANCE_PARAMETER_HIGH)) {

		// Save value to ram and eeprom
		memoryData.fields.speedComplianceParameter = speedComplianceParameter;
		if (!MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

			EEPROM0_SetVariable(EEPROM0_SPEED_COMPLIANCE_PARAMETER_LOW, ((uint16_t)(speedComplianceParameter & 0xFFFF)));
			EEPROM0_SetVariable(EEPROM0_SPEED_COMPLIANCE_PARAMETER_HIGH, ((uint16_t)(speedComplianceParameter >> 16)));

		} else {

			MEMORY_SetError(ERROR_DATA_LOSS);

		}

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get desired position
INLINE_MEMORY fxp32_t MEMORY_GetDesiredPosition(void) {

	return memoryData.fields.desiredPosition;

}

// Set desired position
INLINE_MEMORY void MEMORY_SetDesiredPosition(fxp32_t desiredPosition) {

	fxp32_t positionLimit = 0;

	// Limit desired position value to clockwise limit value and set position limit error
	positionLimit = MEMORY_GetPositionLimitClockwise();
	if ((positionLimit > 0) && (desiredPosition > positionLimit)) {

		desiredPosition = positionLimit;
		MEMORY_SetError(ERROR_POSITION_LIMIT);

	}

	// Limit desired position value to counterclockwise limit value and set position limit error
	positionLimit = MEMORY_GetPositionLimitCounterclockwise();
	if ((positionLimit > 0) && (desiredPosition < positionLimit)) {

		desiredPosition = MEMORY_GetPositionLimitCounterclockwise();
		MEMORY_SetError(ERROR_POSITION_LIMIT);

	}

	// Save value to ram
	memoryData.fields.desiredPosition = desiredPosition;

	// Reset position trajectory generation
	if (((MEMORY_CheckFlag(FLAG_CONTROL_MODE_2) << 2) | (MEMORY_CheckFlag(FLAG_CONTROL_MODE_1) << 1) | MEMORY_CheckFlag(FLAG_CONTROL_MODE_0)) == MODE_POSITION_CONTROL_WITH_TRAJECTORY) {

		TRAJECTORY_GENERATION_ResetPositionTrajectory();

	}

}

// Get desired speed
INLINE_MEMORY fxp32_t MEMORY_GetDesiredSpeed(void) {

	return memoryData.fields.desiredSpeed;

}

// Set desired speed
INLINE_MEMORY void MEMORY_SetDesiredSpeed(fxp32_t desiredSpeed) {

	// Save value to ram
	memoryData.fields.desiredSpeed = desiredSpeed;

	// Reset speed trajectory generation
	if (((MEMORY_CheckFlag(FLAG_CONTROL_MODE_2) << 2) | (MEMORY_CheckFlag(FLAG_CONTROL_MODE_1) << 1) | MEMORY_CheckFlag(FLAG_CONTROL_MODE_0)) == MODE_SPEED_CONTROL_WITH_TRAJECTORY) {

		TRAJECTORY_GENERATION_ResetSpeedTrajectory();

	}

}

// Get desired acceleration
INLINE_MEMORY fxp32_t MEMORY_GetDesiredAcceleration(void) {

	return memoryData.fields.desiredAcceleration;

}

// Set desired acceleration
INLINE_MEMORY void MEMORY_SetDesiredAcceleration(fxp32_t desiredAcceleration) {

	// Save value to ram
	memoryData.fields.desiredAcceleration = desiredAcceleration;

}

// Get desired pulse width
INLINE_MEMORY int16_t MEMORY_GetDesiredPulseWidth(void) {

	return memoryData.fields.desiredPulseWidth;

}

// Set desired pulse width
INLINE_MEMORY void MEMORY_SetDesiredPulseWidth(int16_t desiredPulseWidth) {

	if ((desiredPulseWidth >= LIMIT_PULSE_WIDTH_LOW) && (desiredPulseWidth <= LIMIT_PULSE_WIDTH_HIGH)) {

		// Save value to ram
		memoryData.fields.desiredPulseWidth = desiredPulseWidth;

	} else {

		// Set parameter range error flag
		MEMORY_SetError(ERROR_PARAMETER_RANGE);

	}

}

// Get actual pulse width
INLINE_MEMORY int16_t MEMORY_GetActualPulseWidth(void) {

	return memoryData.fields.actualPulseWidth;

}

// Set actual pulse width
INLINE_MEMORY void MEMORY_SetActualPulseWidth(int16_t actualPulseWidth) {

	// Save value to ram
	memoryData.fields.actualPulseWidth = actualPulseWidth;

}

// Get actual position sensor measurement
INLINE_MEMORY uint16_t MEMORY_GetActualPositionMeasurement(void) {

	return memoryData.fields.actualPositionMeasurement;

}

// Set actual position sensor measurement
INLINE_MEMORY void MEMORY_SetActualPositionMeasurement(uint16_t actualPositionMeasurement) {

	// Save value to ram
	memoryData.fields.actualPositionMeasurement = actualPositionMeasurement;

}

// Get actual position sensor status
INLINE_MEMORY uint16_t MEMORY_GetActualPositionStatus(void) {

	return memoryData.fields.actualPositionStatus;

}

// Set actual position sensor status
INLINE_MEMORY void MEMORY_SetActualPositionStatus(uint16_t actualPositionStatus) {

	// Save value to ram
	memoryData.fields.actualPositionStatus = actualPositionStatus;

}

// Get actual system time
INLINE_MEMORY uint32_t MEMORY_GetActualSystemTime(void) {

	return memoryData.fields.actualSystemTime;

}

// Set actual system time
INLINE_MEMORY void MEMORY_SetActualSystemTime(uint32_t actualSystemTime) {

	// Save value to ram
	memoryData.fields.actualSystemTime = actualSystemTime;

}

// Increment system time
INLINE_MEMORY void MEMORY_IncrementActualSystemTime(void) {

	memoryData.fields.actualSystemTime++;

}

// Get actual position
INLINE_MEMORY fxp32_t MEMORY_GetActualPosition(void) {

	return memoryData.fields.actualPosition;

}

// Set actual position
INLINE_MEMORY void MEMORY_SetActualPosition(fxp32_t actualPosition) {

	// Save value to ram
	memoryData.fields.actualPosition = actualPosition;

}

// Get actual speed
INLINE_MEMORY fxp32_t MEMORY_GetActualSpeed(void) {

	return memoryData.fields.actualSpeed;

}

// Set actual speed
INLINE_MEMORY void MEMORY_SetActualSpeed(fxp32_t actualSpeed) {

	// Save value to ram
	memoryData.fields.actualSpeed = actualSpeed;

}

// Get actual torque
INLINE_MEMORY fxp32_t MEMORY_GetActualTorque(void) {

	return memoryData.fields.actualTorque;

}

// Set actual torque
INLINE_MEMORY void MEMORY_SetActualTorque(fxp32_t actualTorque) {

	// Save value to ram
	memoryData.fields.actualTorque = actualTorque;

}

// Get actual current
INLINE_MEMORY fxp32_t MEMORY_GetActualCurrent(void) {

	return memoryData.fields.actualCurrent;

}

// Set actual current
INLINE_MEMORY void MEMORY_SetActualCurrent(fxp32_t actualCurrent) {

	// Save value to ram
	memoryData.fields.actualCurrent = actualCurrent;

}

// Get actual voltage
INLINE_MEMORY fxp16_t MEMORY_GetActualVoltage(void) {

	return memoryData.fields.actualVoltage;

}

// Set actual voltage
INLINE_MEMORY void MEMORY_SetActualVoltage(fxp16_t actualVoltage) {

	// Save value to ram
	memoryData.fields.actualVoltage = actualVoltage;

}

// Get actual motor temperature
INLINE_MEMORY fxp16_t MEMORY_GetActualMotorTemperature(void) {

	return memoryData.fields.actualMotorTemperature;

}

// Set actual motor temperature
INLINE_MEMORY void MEMORY_SetActualMotorTemperature(fxp16_t actualMotorTemperature) {

	// Save value to ram
	memoryData.fields.actualMotorTemperature = actualMotorTemperature;

}

// Get actual controller temperature
INLINE_MEMORY fxp16_t MEMORY_GetActualControllerTemperature(void) {

	return memoryData.fields.actualControllerTemperature;

}

// Set actual controller temperature
INLINE_MEMORY void MEMORY_SetActualControllerTemperature(fxp16_t actualControllerTemperature) {

	// Save value to ram
	memoryData.fields.actualControllerTemperature = actualControllerTemperature;

}

// Get checksum
INLINE_MEMORY uint16_t MEMORY_GetChecksum(void) {

	return memoryData.fields.checksum;

}

// Set checksum
INLINE_MEMORY void MEMORY_SetChecksum(uint16_t checksum) {

	// Save value to ram
	memoryData.fields.checksum = checksum;

}

#endif
