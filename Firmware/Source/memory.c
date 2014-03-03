/*
 ******************************************************************************
 * File    		memory.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Memory functions.
 * Peripherals
 ******************************************************************************
 */

// Includes
#define INLINE_MEMORY
#include "memory.h"

// Global variables
unionMemoryFields memoryData;

// Private function prototypes
void MEMORY_CalculateChecksum(void);

// Initialize memory functions
void MEMORY_Initialize(void) {

	// Initialize eeprom emulation for system settings
	EEPROM0_Initalize();

	// Initialize volatile values with default values
	MEMORY_SetFlags(DEFAULT_FLAGS);
	MEMORY_SetErrors(DEFAULT_ERRORS);
	MEMORY_SetChecksum(DEFAULT_FIRMWARE_CHECKSUM);
	MEMORY_SetDesiredPosition(DEFAULT_DESIRED_POSITION);
	MEMORY_SetDesiredSpeed(DEFAULT_DESIRED_SPEED);
	MEMORY_SetDesiredAcceleration(DEFAULT_DESIRED_ACCELERATION);
	MEMORY_SetDesiredPulseWidth(DEFAULT_DESIRED_PULSE_WIDTH);
	MEMORY_SetActualPulseWidth(DEFAULT_ACTUAL_PULSE_WIDTH);
	MEMORY_SetActualPositionMeasurement(DEFAULT_ACTUAL_POSITION_MEASUREMENT);
	MEMORY_SetActualPositionStatus(DEFAULT_ACTUAL_POSITION_STATUS);
	MEMORY_SetActualSystemTime(DEFAULT_ACTUAL_SYSTEM_TIME);
	MEMORY_SetActualPosition(DEFAULT_ACTUAL_POSITION);
	MEMORY_SetActualSpeed(DEFAULT_ACTUAL_SPEED);
	MEMORY_SetActualTorque(DEFAULT_ACTUAL_TORQUE);
	MEMORY_SetActualCurrent(DEFAULT_ACTUAL_CURRENT);
	MEMORY_SetActualVoltage(DEFAULT_ACTUAL_VOLTAGE);

	// Initialize non-volatile values with default values or load saved values from eeprom
	MEMORY_SetFirmwareVersion(DEFAULT_FIRMWARE_VERSION);
	if (EEPROM0_GetVariable(EEPROM0_SERVO_ID) == EEPROM_ERROR) {

		// Initialize non-volatile values with default values
		MEMORY_Reset();
		MEMORY_SetServoId(DEFAULT_SERVO_ID);
		MEMORY_SetBaudrateDivider(DEFAULT_BAUDRATE_DIVIDER);

		// Set flag for controller temperature sensor calibration
		MEMORY_SetFlag(FLAG_TEMPERATURE_SENSOR_CALIBRATION);

	} else {

		// Initialize non-volatile values with saved values from eeprom
		MEMORY_SetModelNumber(EEPROM0_GetVariable(EEPROM0_MODEL_NUMBER));
		MEMORY_SetServoId((uint8_t)EEPROM0_GetVariable(EEPROM0_SERVO_ID));
		MEMORY_SetStatusReturnLevel((uint8_t)EEPROM0_GetVariable(EEPROM0_STATUS_RETURN_LEVEL));
		MEMORY_SetStatusReturnDelayTime(EEPROM0_GetVariable(EEPROM0_STATUS_RETURN_DELAY_TIME));
		MEMORY_SetBaudrateDivider(EEPROM0_GetVariable(EEPROM0_BAUDRATE_DIVIDER));
		MEMORY_SetPositionOffset(EEPROM0_GetVariable(EEPROM0_POSITION_OFFSET));
		MEMORY_SetPositionLimitClockwise(((fxp32_t)EEPROM0_GetVariable(EEPROM0_POSITION_LIMIT_CLOCKWISE_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_POSITION_LIMIT_CLOCKWISE_LOW));
		MEMORY_SetPositionLimitCounterclockwise(((fxp32_t)EEPROM0_GetVariable(EEPROM0_POSITION_LIMIT_COUNTERCLOCKWISE_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_POSITION_LIMIT_COUNTERCLOCKWISE_LOW));
		MEMORY_SetTorqueLimit(((fxp32_t)EEPROM0_GetVariable(EEPROM0_TORQUE_LIMIT_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_TORQUE_LIMIT_LOW));
		MEMORY_SetCurrentLimit(((fxp32_t)EEPROM0_GetVariable(EEPROM0_CURRENT_LIMIT_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_CURRENT_LIMIT_LOW));
		MEMORY_SetVoltageLimitLow(EEPROM0_GetVariable(EEPROM0_VOLTAGE_LIMIT_LOW));
		MEMORY_SetVoltageLimitHigh(EEPROM0_GetVariable(EEPROM0_VOLTAGE_LIMIT_HIGH));
		MEMORY_SetMotorTemperatureLimit(EEPROM0_GetVariable(EEPROM0_MOTOR_TEMPERATURE_LIMIT));
		MEMORY_SetControllerTemperatureLimit(EEPROM0_GetVariable(EEPROM0_CONTROLLER_TEMPERATURE_LIMIT));
		MEMORY_SetControllerTemperatureOffset(EEPROM0_GetVariable(EEPROM0_CONTROLLER_TEMPERATURE_OFFSET));
		MEMORY_SetStatusSignal(EEPROM0_GetVariable(EEPROM0_STATUS_SIGNAL));
		MEMORY_SetAlarmSignal(EEPROM0_GetVariable(EEPROM0_ALARM_SIGNAL));
		MEMORY_SetAlarmShutdown(EEPROM0_GetVariable(EEPROM0_ALARM_SHUTDOWN));
        MEMORY_SetSkipZoneParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_SKIP_ZONE_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_SKIP_ZONE_PARAMETER_LOW));
        MEMORY_SetFeedForwardControlParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_FEED_FORWARD_CONTROL_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_FEED_FORWARD_CONTROL_PARAMETER_LOW));
		MEMORY_SetProportionalPositionControlParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_PROPORTIONAL_POSITION_CONTROL_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_PROPORTIONAL_POSITION_CONTROL_PARAMETER_LOW));
		MEMORY_SetIntegralPositionControlParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_INTEGRAL_POSITION_CONTROL_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_INTEGRAL_POSITION_CONTROL_PARAMETER_LOW));
		MEMORY_SetDerivativePositionControlParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_DERIVATIVE_POSITION_CONTROL_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_DERIVATIVE_POSITION_CONTROL_PARAMETER_LOW));
		MEMORY_SetPositionComplianceParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_POSITION_COMPLIANCE_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_POSITION_COMPLIANCE_PARAMETER_LOW));
		MEMORY_SetProportionalSpeedControlParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_PROPORTIONAL_SPEED_CONTROL_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_PROPORTIONAL_SPEED_CONTROL_PARAMETER_LOW));
		MEMORY_SetIntegralSpeedControlParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_INTEGRAL_SPEED_CONTROL_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_INTEGRAL_SPEED_CONTROL_PARAMETER_LOW));
		MEMORY_SetDerivativeSpeedControlParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_DERIVATIVE_SPEED_CONTROL_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_DERIVATIVE_SPEED_CONTROL_PARAMETER_LOW));
		MEMORY_SetSpeedComplianceParameter(((fxp32_t)EEPROM0_GetVariable(EEPROM0_SPEED_COMPLIANCE_PARAMETER_HIGH) << 16) | EEPROM0_GetVariable(EEPROM0_SPEED_COMPLIANCE_PARAMETER_LOW));

	}

	// Calculate firmware checksum
	MEMORY_CalculateChecksum();

}

// Reset non-volatile values to default values, except servo id and broadcast divider
void MEMORY_Reset(void) {

	MEMORY_SetModelNumber(DEFAULT_MODEL_NUMBER);
	MEMORY_SetStatusReturnLevel(DEFAULT_STATUS_RETURN_LEVEL);
	MEMORY_SetStatusReturnDelayTime(DEFAULT_STATUS_RETURN_DELAY_TIME);
	MEMORY_SetPositionOffset(DEFAULT_POSITION_OFFSET);
	MEMORY_SetPositionLimitClockwise(DEFAULT_POSITION_LIMIT_CLOCKWISE);
	MEMORY_SetPositionLimitCounterclockwise(DEFAULT_POSITION_LIMIT_COUNTERCLOCKWISE);
	MEMORY_SetTorqueLimit(DEFAULT_TORQUE_LIMIT);
	MEMORY_SetCurrentLimit(DEFAULT_CURRENT_LIMIT);
	MEMORY_SetVoltageLimitLow(DEFAULT_VOLTAGE_LIMIT_LOW);
	MEMORY_SetVoltageLimitHigh(DEFAULT_VOLTAGE_LIMIT_HIGH);
	MEMORY_SetMotorTemperatureLimit(DEFAULT_MOTOR_TEMPERATURE_LIMIT);
	MEMORY_SetControllerTemperatureLimit(DEFAULT_CONTROLLER_TEMPERATURE_LIMIT);
	MEMORY_SetControllerTemperatureOffset(DEFAULT_CONTROLLER_TEMPERATURE_OFFSET);
	MEMORY_SetStatusSignal(DEFAULT_STATUS_SIGNAL);
	MEMORY_SetAlarmSignal(DEFAULT_ALARM_SIGNAL);
	MEMORY_SetAlarmShutdown(DEFAULT_ALARM_SHUTDOWN);
    MEMORY_SetSkipZoneParameter(DEFAULT_SKIP_ZONE_PARAMETER);
	MEMORY_SetFeedForwardControlParameter(DEFAULT_FEED_FORWARD_CONTROL_PARAMETER);
	MEMORY_SetProportionalPositionControlParameter(DEFAULT_PROPORTIONAL_POSITION_CONTROL_PARAMETER);
	MEMORY_SetIntegralPositionControlParameter(DEFAULT_INTEGRAL_POSITION_CONTROL_PARAMETER);
	MEMORY_SetDerivativePositionControlParameter(DEFAULT_DERIVATIVE_POSITION_CONTROL_PARAMETER);
	MEMORY_SetPositionComplianceParameter(DEFAULT_POSITION_COMPLIANCE_PARAMETER);
	MEMORY_SetProportionalSpeedControlParameter(DEFAULT_PROPORTIONAL_SPEED_CONTROL_PARAMETER);
	MEMORY_SetIntegralSpeedControlParameter(DEFAULT_INTEGRAL_SPEED_CONTROL_PARAMETER);
	MEMORY_SetDerivativeSpeedControlParameter(DEFAULT_DERIVATIVE_SPEED_CONTROL_PARAMETER);
	MEMORY_SetSpeedComplianceParameter(DEFAULT_SPEED_COMPLIANCE_PARAMETER);

}

// Calculate the checksum of the currently stored firmware
void MEMORY_CalculateChecksum(void) {

	// Private variables
	uint8_t* memoryAddress = (uint8_t*)FLASH_START_ADDRESS;
	uint16_t firmwareChecksum = 0;

	// Calculate and save checksum
	while (memoryAddress < (uint8_t*)EEPROM1_START_ADDRESS) {

		firmwareChecksum += *memoryAddress;
		memoryAddress++;

	}
	firmwareChecksum = ~firmwareChecksum;
	MEMORY_SetChecksum(firmwareChecksum);

}

// Get flag bit field
uint16_t MEMORY_GetFlags(void) {

	return memoryData.fields.flags;

}

// Set flag bit field
void MEMORY_SetFlags(uint16_t flags) {

	memoryData.fields.flags = flags;

	// Evaluate motor power flag
	if (MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

		// Reset desired values and activate motor power
		CONTROL_ResetDesiredValues();
		MOTOR_Activate();

	} else {

		// Deactivate motor
		MOTOR_Deactivate();

	}

}

// Set the flag specified by flag pointer
void MEMORY_SetFlag(uint8_t flagPointer) {

	memoryData.fields.flags |= (1 << flagPointer);

	// If motor power flag is set, activate motor power
	if (flagPointer == FLAG_MOTOR_POWER_ACTIVATED) {

		// Reset desired values and activate motor power
		CONTROL_ResetDesiredValues();
		MOTOR_Activate();

	}

}

// Reset the flag specified by flag pointer
void MEMORY_ResetFlag(uint8_t flagPointer) {

	memoryData.fields.flags &= ~(1 << flagPointer);

	// If motor power is deactivated, shut motor down
	if (flagPointer == FLAG_MOTOR_POWER_ACTIVATED) {

		// Deactivate motor
		MOTOR_Deactivate();

	}

}

// Check the flag status specified by flag pointer
uint8_t MEMORY_CheckFlag(uint8_t flagPointer) {

	if (memoryData.fields.flags & (1 << flagPointer)) {

		return 1;

	} else {

		return 0;

	}

}

// Get error bit field
uint16_t MEMORY_GetErrors(void) {

	return memoryData.fields.errors;

}

// Get a summary of errors
uint8_t MEMORY_GetCompressedErrors(void) {

	uint8_t compressedErrors = 0;

	// Voltage limit error
	if (MEMORY_CheckError(ERROR_VOLTAGE_LIMIT)) {

		compressedErrors |= 0x01;

	}

	// Position limit error
	if (MEMORY_CheckError(ERROR_POSITION_LIMIT)) {

		compressedErrors |= 0x02;

	}

	// Torque or current limit error
	if (MEMORY_CheckError(ERROR_TORQUE_LIMIT) || MEMORY_CheckError(ERROR_CURRENT_LIMIT)) {

		compressedErrors |= 0x04;

	}

	// Temperature limit error
	if (MEMORY_CheckError(ERROR_CONTROLLER_TEMPERATURE_LIMIT) || MEMORY_CheckError(ERROR_MOTOR_TEMPERATURE_LIMIT)) {

		compressedErrors |= 0x08;

	}

	// Parameter range or instruction error
	if (MEMORY_CheckError(ERROR_PARAMETER_RANGE) || MEMORY_CheckError(ERROR_INSTRUCTION)) {

		compressedErrors |= 0x10;

	}

	// Checksum error
	if (MEMORY_CheckError(ERROR_CHECKSUM)) {

		compressedErrors |= 0x20;

	}

	// Task management timeout error
	if (MEMORY_CheckError(ERROR_TASK_MANAGEMENT)) {

		compressedErrors |= 0x40;

	}

	// Hardware error
	if (MEMORY_CheckError(ERROR_COMMUNICATION) || MEMORY_CheckError(ERROR_ANALOG_CONVERTER) || MEMORY_CheckError(ERROR_POSITION_SENSOR) || MEMORY_CheckError(ERROR_EEPROM)) {

		compressedErrors |= 0x80;

	}

	return compressedErrors;

}

// Translate a summary of errors from DD format to Robotis format
uint8_t MEMORY_TranslateCompressedErrors(void) {

	uint8_t compressedErrors = 0;

	// Voltage limit error
	if (MEMORY_CheckError(ERROR_VOLTAGE_LIMIT)) {

		compressedErrors |= 0x01;

	}

	// Position limit error
	if (MEMORY_CheckError(ERROR_POSITION_LIMIT)) {

		compressedErrors |= 0x02;

	}

	// Temperature limit error
	if (MEMORY_CheckError(ERROR_MOTOR_TEMPERATURE_LIMIT)) {

		compressedErrors |= 0x04;

	}

	// Parameter range error
	if (MEMORY_CheckError(ERROR_PARAMETER_RANGE)) {

		compressedErrors |= 0x08;

	}

	// Checksum error
	if (MEMORY_CheckError(ERROR_CHECKSUM)) {

		compressedErrors |= 0x10;

	}

	// Torque limit error
	if (MEMORY_CheckError(ERROR_TORQUE_LIMIT)) {

		compressedErrors |= 0x20;

	}

	// Instruction error
	if (MEMORY_CheckError(ERROR_INSTRUCTION)) {

		compressedErrors |= 0x40;

	}

	return compressedErrors;

}

// Set error bit field
void MEMORY_SetErrors(uint16_t errors) {

	memoryData.fields.errors = errors;

	// If any predefined error has occurred, shut servo down
	if (memoryData.fields.errors & MEMORY_GetAlarmShutdown()) {

		// Deactivate motor power and motor
		MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);

	}

	// If any predefined error has occurred, turn signal to error signal
	if (memoryData.fields.errors & MEMORY_GetAlarmSignal()) {

		// Turn signal to error color
		SIGNAL_Error();

	}

}

// Set the error specified by error pointer
void MEMORY_SetError(uint8_t errorPointer) {

	memoryData.fields.errors |= (1 << errorPointer);

	// If any predefined error has occurred, shut motor down
	if (memoryData.fields.errors & MEMORY_GetAlarmShutdown()) {

		// Deactivate motor power and motor
		MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);

	}

	// If any predefined error has occurred, turn signal to error signal
	if (memoryData.fields.errors & MEMORY_GetAlarmSignal()) {

		// Turn signal to error color
		SIGNAL_Error();

	}

}

// Reset the error specified by error pointer
void MEMORY_ResetError(uint8_t errorPointer) {

	memoryData.fields.errors &= ~(1 << errorPointer);

}

// Check the error status specified by error pointer
uint8_t MEMORY_CheckError(uint8_t errorPointer) {

	if (memoryData.fields.errors & (1 << errorPointer)) {

		return 1;

	} else {

		return 0;

	}

}
