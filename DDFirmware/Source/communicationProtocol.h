 /*
 ******************************************************************************
 * File    		communication.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for communication protocol functions.
 * Peripherals	TIM8
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __COMMUNICATION_PROTOCOL_H
#define __COMMUNICATION_PROTOCOL_H

// Type includes
#include "stm32f10x.h"

// Defines
#ifndef INLINE_COMMUNICATION_PROTOCOL
#define INLINE_COMMUNICATION_PROTOCOL extern inline
#endif

// Type defines
typedef enum {

	INSTRUCTION_START_BOOTLOADER_DD = 0x00,
	INSTRUCTION_PING_SERVO_ROBOTIS = 0x01,
	INSTRUCTION_READ_DATA_ROBOTIS = 0x02,
	INSTRUCTION_WRITE_DATA_ROBOTIS = 0x03,
	INSTRUCTION_WRITE_PENDING_DATA_ROBOTIS = 0x04,
	INSTRUCTION_TRIGGER_PENDING_DATA_ROBOTIS = 0x05,
	INSTRUCTION_RESET_SETTINGS_ROBOTIS = 0x06,
	INSTRUCTION_SYNCHRONIZED_WRITE_DATA_ROBOTIS = 0x83,
	INSTRUCTION_PING_SERVO_DD = 0x11,
	INSTRUCTION_READ_DATA_DD = 0x12,
	INSTRUCTION_WRITE_DATA_DD = 0x13,
	INSTRUCTION_SET_MOTOR_POWER_DD = 0x14,
	INSTRUCTION_SET_LOGGING_DD = 0x15,
	INSTRUCTION_SET_CONTROL_MODE_DD = 0x16,
	INSTRUCTION_WRITE_PENDING_DATA_DD = 0x17,
	INSTRUCTION_TRIGGER_PENDING_DATA_DD = 0x18,
	INSTRUCTION_SYNCHRONIZED_READ_DATA_DD = 0x19,
	INSTRUCTION_SYNCHRONIZED_WRITE_DATA_DD = 0x1A,
	INSTRUCTION_READ_LOGGING_DATA_DD = 0x1B,
	INSTRUCTION_READ_CHECKSUM_DD = 0x1C,
	INSTRUCTION_SET_POSITION_OFFSET_DD = 0x1D,
	INSTRUCTION_REBOOT_SERVO_DD = 0x1E,
	INSTRUCTION_RESET_ERRORS_DD = 0x1F,
	INSTRUCTION_RESET_SETTINGS_DD = 0x20

} instruction_t;
typedef enum {

	ADDRESS_EEPROM_MODEL_NUMBER_LOW,
	ADDRESS_EEPROM_MODEL_NUMBER_HIGH,
	ADDRESS_EEPROM_VERSION_OF_FIRMWARE,
	ADDRESS_EEPROM_ID,
	ADDRESS_EEPROM_BAUDRATE,
	ADDRESS_EEPROM_RETURN_DELAY_TIME,
	ADDRESS_EEPROM_CLOCKWISE_ANGLE_LIMIT_LOW,
	ADDRESS_EEPROM_CLOCKWISE_ANGLE_LIMIT_HIGH,
	ADDRESS_EEPROM_COUNTERCLOCKWISE_ANGLE_LIMIT_LOW,
	ADDRESS_EEPROM_COUNTERCLOCKWISE_ANGLE_LIMIT_HIGH,
	ADDRESS_EEPROM_RESERVERD_1,
	ADDRESS_EEPROM_TEMPERATURE_LIMIT,
	ADDRESS_EEPROM_LOWER_VOLTAGE_LIMIT,
	ADDRESS_EEPROM_UPPER_VOLTAGE_LIMIT,
	ADDRESS_EEPROM_TORQUE_LIMIT_LOW,
	ADDRESS_EEPROM_TORQUE_LIMIT_HIGH,
	ADDRESS_EEPROM_STATUS_RETURN_LEVEL,
	ADDRESS_EEPROM_ALARM_LED,
	ADDRESS_EEPROM_ALERM_SHUTDOWN,
	ADDRESS_EEPROM_RESERVED_2,
	ADDRESS_EEPROM_DOWN_CALIBRATION_LOW,
	ADDRESS_EEPROM_DOWN_CALIBRATION_HIGH,
	ADDRESS_EEPROM_UP_CALIBRATION_LOW,
	ADDRESS_EEPROM_UP_CALIBRATION_HIGH,
	ADDRESS_RAM_TORQUE_ENABLE,
	ADDRESS_RAM_LED,
	ADDRESS_RAM_CLOCKWISE_COMPLIANCE_MARGIN,
	ADDRESS_RAM_COUNTERCLOCKWISE_COMPLIANCE_MARGIN,
	ADDRESS_RAM_CLOCKWISE_COMPLIANCE_SLOPE,
	ADDRESS_RAM_COUNTERCLOCKWISE_COMPLIANCE_SLOPE,
	ADDRESS_RAM_GOAL_POSITION_LOW,
	ADDRESS_RAM_GOAL_POSITION_HIGH,
	ADDRESS_RAM_MOVING_SPEED_LOW,
	ADDRESS_RAM_MOVING_SPEED_HIGH,
	ADDRESS_RAM_TORQUE_LIMIT_LOW,
	ADDRESS_RAM_TORQUE_LIMIT_HIGH,
	ADDRESS_RAM_PRESENT_POSITION_LOW,
	ADDRESS_RAM_PRESENT_POSITION_HIGH,
	ADDRESS_RAM_PRESENT_SPEED_LOW,
	ADDRESS_RAM_PRESENT_SPEED_HIGH,
	ADDRESS_RAM_PRESENT_LOAD_LOW,
	ADDRESS_RAM_PRESENT_LOAD_HIGH,
	ADDRESS_RAM_PRESENT_VOLTAGE,
	ADDRESS_RAM_PRESENT_TEMPERATURE,
	ADDRESS_RAM_REGISTERED_INSTRUCTION,
	ADDRESS_RAM_RESERVED_3,
	ADDRESS_RAM_MOVING,
	ADDRESS_RAM_LOCK,
	ADDRESS_RAM_PUNCH_LOW,
	ADDRESS_RAM_PUNCH_HIGH

} address_t;

// Global variables
extern uint8_t pendingDataBuffer[256];

// Global function prototypes
void COMMUNICATION_PROTOCOL_ProcessPacket(void);
void COMMUNICATION_PROTOCOL_ProcessSynchronizedReadTimeout(void);
void COMMUNICATION_PROTOCOL_CreateStatusPacket(uint8_t, uint8_t);
uint8_t COMMUNICATION_PROTOCOL_GetValue8(uint8_t*, uint8_t*, uint8_t*, uint8_t);
uint16_t COMMUNICATION_PROTOCOL_GetValue16(uint16_t*, uint8_t*, uint8_t*, uint8_t);
uint32_t COMMUNICATION_PROTOCOL_GetValue32(uint32_t*, uint8_t*, uint8_t*, uint8_t);
void COMMUNICATION_PROTOCOL_CopyBufferToBuffer(uint8_t, uint8_t);
uint8_t COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(uint8_t);
uint16_t COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(uint8_t);
uint32_t COMMUNICATION_PROTOCOL_GetPendingDataBuffer32(uint8_t);
uint8_t COMMUNICATION_PROTOCOL_CalculateChecksum();

// Module includes
#include <stddef.h>
#include "communication.h"
#include "control.h"
#include "logging.h"
#include "memory.h"
#include "motor.h"
#include "observer.h"
#include "signal.h"
#include "task.h"

// Create status packet and write memory values to transmit buffer
INLINE_COMMUNICATION_PROTOCOL void COMMUNICATION_PROTOCOL_CreateStatusPacket(uint8_t memoryAddress, uint8_t memoryLength) {

	// Add servo id to transmission buffer
	COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetServoId());

	// Add length to transmission buffer
	COMMUNICATION_AddTransmissionBuffer8(memoryLength + 2);

	// Add errors to transmission buffer
	COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetCompressedErrors());

	// Add memory values to transmission buffer
	for (; memoryLength > 0; memoryLength--) {

		COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetMemoryData(memoryAddress));
		memoryAddress++;

	}

}

// Get one byte at assigned address from reception or data pending buffer
INLINE_COMMUNICATION_PROTOCOL uint8_t COMMUNICATION_PROTOCOL_GetValue8(uint8_t *value, uint8_t *memoryLength, uint8_t *bufferAddress, uint8_t mode) {

	if (*memoryLength < 1) {

		return 0;

	}

	if (mode == MODE_RECEPTION_BUFFER) {

		*value = COMMUNICATION_GetReceptionBuffer8(*bufferAddress);

	} else {

		*value = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(*bufferAddress);

	}

	// Update address and length
	*bufferAddress += 1;
	*memoryLength -= 1;

	return 1;

}

// Get two bytes at assigned address from reception or data pending buffer
INLINE_COMMUNICATION_PROTOCOL uint16_t COMMUNICATION_PROTOCOL_GetValue16(uint16_t *value, uint8_t *memoryLength, uint8_t *bufferAddress, uint8_t mode) {

	if (*memoryLength < 2) {

		return 0;

	}

	if (mode == MODE_RECEPTION_BUFFER) {

		*value = COMMUNICATION_GetReceptionBuffer16(*bufferAddress);

	} else {

		*value = COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(*bufferAddress);

	}

	// Update address and length
	*bufferAddress += 2;
	*memoryLength -= 2;

	return 1;

}

// Get four bytes at assigned address from reception or data pending buffer
INLINE_COMMUNICATION_PROTOCOL uint32_t COMMUNICATION_PROTOCOL_GetValue32(uint32_t *value, uint8_t *memoryLength, uint8_t *bufferAddress, uint8_t mode) {

	if (*memoryLength < 4) {

		return 0;

	}

	if (mode == MODE_RECEPTION_BUFFER) {

		*value = COMMUNICATION_GetReceptionBuffer32(*bufferAddress);

	} else {

		*value = COMMUNICATION_PROTOCOL_GetPendingDataBuffer32(*bufferAddress);

	}

	// Update address and length
	*bufferAddress += 4;
	*memoryLength -= 4;

	return 1;

}

// Copy reception buffer values to pending data buffer
INLINE_COMMUNICATION_PROTOCOL void COMMUNICATION_PROTOCOL_CopyBufferToBuffer(uint8_t bufferAddress, uint8_t bufferLength) {

	// Private variables
	uint8_t bufferAddressOffset = 0;

	// Copy reception buffer values to data pending buffer
	for (; bufferLength > 0; bufferLength--) {

		pendingDataBuffer[bufferAddressOffset] = COMMUNICATION_GetReceptionBuffer8(bufferAddress + bufferAddressOffset);
		bufferAddressOffset++;

	}

}

// Get one byte at assigned address from data pending buffer
INLINE_COMMUNICATION_PROTOCOL uint8_t COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(uint8_t address) {

	uint8_t data = pendingDataBuffer[address];

	return data;

}

// Get two bytes at assigned address from data pending buffer
INLINE_COMMUNICATION_PROTOCOL uint16_t COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(uint8_t address) {

	uint16_t data = *((uint16_t*)&pendingDataBuffer[address]);

	return data;

}

// Get four bytes at assigned address from data pending buffer
INLINE_COMMUNICATION_PROTOCOL uint32_t COMMUNICATION_PROTOCOL_GetPendingDataBuffer32(uint8_t address) {

	uint32_t data = *((uint32_t*)&pendingDataBuffer[address]);

	return data;

}

// Calculate checksum
INLINE_COMMUNICATION_PROTOCOL uint8_t COMMUNICATION_PROTOCOL_CalculateChecksum() {

	uint8_t index;
	uint8_t checksum = 0;

	// Calculate checksum
	for (index = 2; index < (COMMUNICATION_GetReceptionBuffer8(3) + 3); index++) {

		checksum += COMMUNICATION_GetReceptionBuffer8(index);

	}
	checksum = ~checksum;

	return checksum;

}

#endif
