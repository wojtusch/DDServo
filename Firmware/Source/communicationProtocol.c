/*
 ******************************************************************************
 * File    		communicationProtocol.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Communication protocol functions.
 * Peripherals	TIM8
 ******************************************************************************
 */

// Includes
#define INLINE_COMMUNICATION_PROTOCOL
#include "communicationProtocol.h"

// Global variables
uint8_t pendingDataBuffer[256];
uint8_t pendingDataAdress;
uint8_t pendingDataLength;
uint8_t pendingServoId;
uint8_t lastServoId;
uint16_t synchronizedReadTimeout;

// Private function prototypes
static void COMMUNICATION_PROTOCOL_CopyBufferToMemory(uint8_t, uint8_t, uint8_t, uint8_t);
static void COMMUNICATION_PROTOCOL_TranslateFormat(uint8_t, uint8_t, uint8_t, uint8_t);

// Process communication packet
void COMMUNICATION_PROTOCOL_ProcessPacket(void) {

	// Private variables
	uint8_t index;
	uint8_t checksum;

	// Calculate checksum
	checksum = COMMUNICATION_PROTOCOL_CalculateChecksum();

	// If servo id does not match, servo id is not broadcast id and synchronized read
	// is not in progress, refuse packet and clear reception buffer
	if ((COMMUNICATION_GetReceptionBuffer8(2) != MEMORY_GetServoId()) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID) && !MEMORY_CheckFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS)) {

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		return;

	}

	// If checksum does not match, refuse packet and reset reception buffer
	if (checksum != COMMUNICATION_GetReceptionBuffer8(COMMUNICATION_GetReceptionBuffer8(3) + 3)) {

		// Set checksum error flag
		MEMORY_SetError(ERROR_CHECKSUM);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		return;

	}

	// If synchronized read is in progress, process all packets
	if (MEMORY_CheckFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS)) {

		// If servo id does not equal to pending servo id, update pending servo id
		if (COMMUNICATION_GetReceptionBuffer8(2) != pendingServoId) {

			// Set pending servo id
			pendingServoId = COMMUNICATION_GetReceptionBuffer8(2);

		}

		// Set pending servo id
		if (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(7) == BROADCAST_ID) {

			// Set pending servo id
			pendingServoId++;

		} else {

			// Set last servo id
			lastServoId = pendingServoId;

			// Set pending servo id
			pendingServoId = BROADCAST_ID;

			// Search for servo id and pending servo id
			for (index = 7; index < (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(3) + 3); index++) {

				if ((COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(index) < pendingServoId) && (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(index) > lastServoId)) {

					// Set pending servo id
					pendingServoId = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(index);

				}

			}

		}

		// If no new pending servo id was found, deactivate synchronized read processing
		if (pendingServoId == BROADCAST_ID) {

			// Reset synchronized read in progress flag
			MEMORY_ResetFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS);

			return;

		}

		// Initiate transmission of transmission buffer and deactivate synchronized
		// read processing or activate synchronized read timeout
		if (pendingServoId == MEMORY_GetServoId()) {

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

			// Reset synchronized read in progress flag
			MEMORY_ResetFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS);

		} else {

			// Activate synchronized read timeout
			TASK_SetSynchronizedReadTimeout(synchronizedReadTimeout);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		return;

	}

	//Procession according to instruction
	switch (COMMUNICATION_GetReceptionBuffer8(4)) {

	case INSTRUCTION_START_BOOTLOADER_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x06) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) != 0x01) || (COMMUNICATION_GetReceptionBuffer8(6) != 0x02) || (COMMUNICATION_GetReceptionBuffer8(7) != 0x03) || (COMMUNICATION_GetReceptionBuffer8(8) != 0x04)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Start bootloader
		COMMON_Bootloader();

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_PING_SERVO_ROBOTIS:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x02) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Create status packet
		COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

		// Add checksum and initiate transmission of transmission buffer
		if (COMMUNICATION_GetReceptionBuffer8(2) == BROADCAST_ID) {

			// Initiate transmission of transmission buffer with a delay calculated
			// from the servo id and the baudrate divider divided by 16
			COMMUNICATION_TransmitTransmissionBuffer((MEMORY_GetServoId() * (MEMORY_GetBaudrateDivider() >> 4)) + 1);

		} else {

			// Initiate transmission of transmission buffer without a delay
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_READ_DATA_ROBOTIS:

		// If servo id is broadcast id, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(2) == BROADCAST_ID) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x04) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level does not match, refuse packet and clear reception buffer
		if (MEMORY_GetStatusReturnLevel() < 1) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Create status packet with translating between DD and Robotis memory formats
		COMMUNICATION_PROTOCOL_TranslateFormat(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6), 0, MODE_READ_DATA);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_WRITE_DATA_ROBOTIS:

		// If length does not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(3) < 0x04)  || (COMMUNICATION_GetReceptionBuffer8(3) > 0x34)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Copy data from reception buffer to memory with translating between DD and Robotis memory formats
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			COMMUNICATION_PROTOCOL_TranslateFormat(COMMUNICATION_GetReceptionBuffer8(5), (COMMUNICATION_GetReceptionBuffer8(3) - 3), 6, MODE_WRITE_DATA_WITH_STATUS_PACKET);

		} else {

			COMMUNICATION_PROTOCOL_TranslateFormat(COMMUNICATION_GetReceptionBuffer8(5), (COMMUNICATION_GetReceptionBuffer8(3) - 3), 6, MODE_WRITE_DATA_WITHOUT_STATUS_PACKET);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_WRITE_PENDING_DATA_ROBOTIS:

		// If length does not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(3) < 0x04)  || (COMMUNICATION_GetReceptionBuffer8(3) > 0xFC)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Save pending data address and pending data length
		pendingDataAdress = COMMUNICATION_GetReceptionBuffer8(5);
		pendingDataLength = (COMMUNICATION_GetReceptionBuffer8(3) - 3);

		// Copy receive buffer values to pending data buffer
		COMMUNICATION_PROTOCOL_CopyBufferToBuffer(6, pendingDataLength);

		// Set pending data registered flag
		MEMORY_SetFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_TRIGGER_PENDING_DATA_ROBOTIS:

		// If pending data registered flag is not set, set instruction error flag, refuse
		// packet and clear reception buffer
		if (!MEMORY_CheckFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED)) {

			// Set instruction error flag
			MEMORY_SetError(ERROR_INSTRUCTION);

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x02) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Copy data from reception buffer to memory with translating between DD and Robotis memory formats
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			COMMUNICATION_PROTOCOL_TranslateFormat(pendingDataAdress, pendingDataLength, 0, MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET);

		} else {

			COMMUNICATION_PROTOCOL_TranslateFormat(pendingDataAdress, pendingDataLength, 0, MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET);

		}

		// Reset pending data registered flag
		MEMORY_ResetFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_RESET_SETTINGS_ROBOTIS:

		// Not implemented for reasons of safety

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_SYNCHRONIZED_WRITE_DATA_ROBOTIS:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) < 0x06) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Search servo id and copy reception buffer values to memory
		for (index = 7; index < (COMMUNICATION_GetReceptionBuffer8(3) + 3); index += (COMMUNICATION_GetReceptionBuffer8(6) + 1)) {

			if (COMMUNICATION_GetReceptionBuffer8(index) == MEMORY_GetServoId()) {

				// Copy data from reception buffer to memory with translating between DD and Robotis memory formats
				if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

					COMMUNICATION_PROTOCOL_TranslateFormat(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6), (index + 1), MODE_WRITE_DATA_WITH_STATUS_PACKET);

				} else {

					COMMUNICATION_PROTOCOL_TranslateFormat(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6), (index + 1), MODE_WRITE_DATA_WITHOUT_STATUS_PACKET);

				}

				break;

			}

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_PING_SERVO_DD:

		// If length does not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(3) != 0x02) && (COMMUNICATION_GetReceptionBuffer8(3) != 0x05)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If color parameters are set, set status signal
		if (COMMUNICATION_GetReceptionBuffer8(3) == 0x05) {

			// Activate or deactiavte color ping
			if ((COMMUNICATION_GetReceptionBuffer8(5) != 0x00) || (COMMUNICATION_GetReceptionBuffer8(6) != 0x00) || (COMMUNICATION_GetReceptionBuffer8(7) != 0x00)) {

				// Set color ping request flag
				MEMORY_SetFlag(FLAG_COMMUNICATION_COLOR_PING_REQUEST);

				// Set status signal to requested color
				SIGNAL_SetRGBColor(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6), COMMUNICATION_GetReceptionBuffer8(7));

			} else {

				// Reset color ping request flag
				MEMORY_ResetFlag(FLAG_COMMUNICATION_COLOR_PING_REQUEST);

			}

		}

		// Create status packet
		COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

		// Add checksum and initiate transmission of transmission buffer
		if (COMMUNICATION_GetReceptionBuffer8(2) == BROADCAST_ID) {

			// Initiate transmission of transmission buffer with a delay calculated
			// from the servo id and the baudrate divider divided by 16
			COMMUNICATION_TransmitTransmissionBuffer(MEMORY_GetServoId() * ((MEMORY_GetBaudrateDivider() >> 4) + 1));

		} else {

			// Initiate transmission of transmission buffer without a delay
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_READ_DATA_DD:

		// If servo id is broadcast id, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(2) == BROADCAST_ID) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x04) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) >= sizeof(structMemoryFields)) || (COMMUNICATION_GetReceptionBuffer8(6) > (sizeof(structMemoryFields) - COMMUNICATION_GetReceptionBuffer8(5))) || (COMMUNICATION_GetReceptionBuffer8(6) > 0xFA)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level does not match, refuse packet and clear reception buffer
		if (MEMORY_GetStatusReturnLevel() < 1) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Create status packet
		COMMUNICATION_PROTOCOL_CreateStatusPacket(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6));

		// Add checksum and initiate transmission of transmission buffer
		COMMUNICATION_TransmitTransmissionBuffer(0);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_WRITE_DATA_DD:

		// If length does not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(3) < 0x04)  || (COMMUNICATION_GetReceptionBuffer8(3) > 0xFC)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) >= sizeof(structMemoryFields)) || (((uint8_t)(COMMUNICATION_GetReceptionBuffer8(3) - 3)) > (sizeof(structMemoryFields) - COMMUNICATION_GetReceptionBuffer8(5)))) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Copy reception buffer values to memory
		COMMUNICATION_PROTOCOL_CopyBufferToMemory(COMMUNICATION_GetReceptionBuffer8(5), (COMMUNICATION_GetReceptionBuffer8(3) - 3), 6, MODE_RECEPTION_BUFFER);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_SET_MOTOR_POWER_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x03) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(5) > MODE_MOTOR_POWER_ON) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Set motor power according to parameter
		if ((COMMUNICATION_GetReceptionBuffer8(5) == MODE_MOTOR_POWER_OFF) && (MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED) == MODE_MOTOR_POWER_ON)) {

			// Reset motor power activated flag
			MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);
			CONTROL_ResetControlStates();
			MOTOR_Run(0);

		} else if ((COMMUNICATION_GetReceptionBuffer8(5) == MODE_MOTOR_POWER_ON) && (MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED) == MODE_MOTOR_POWER_OFF)) {

			// Set motor power activated flag
			CONTROL_ResetControlStates();
			MEMORY_SetFlag(FLAG_MOTOR_POWER_ACTIVATED);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_SET_LOGGING_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x03) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(5) > MODE_LOGGING_ON) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Set logging according to parameter
		if (COMMUNICATION_GetReceptionBuffer8(5) == MODE_LOGGING_OFF) {

			// Reset logging activated flag
			MEMORY_ResetFlag(FLAG_LOGGING_ACTIVATED);

		} else if (COMMUNICATION_GetReceptionBuffer8(5) == MODE_LOGGING_ON) {

			// Set logging activated flag
			MEMORY_SetFlag(FLAG_LOGGING_ACTIVATED);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_SET_CONTROL_MODE_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x03) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(5) > 0x05) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Set control mode according to parameter
		if (COMMUNICATION_GetReceptionBuffer8(5) & 0x01) {

			MEMORY_SetFlag(FLAG_CONTROL_MODE_0);

		} else {

			MEMORY_ResetFlag(FLAG_CONTROL_MODE_0);

		}
		if ((COMMUNICATION_GetReceptionBuffer8(5) >> 1) & 0x01) {

			MEMORY_SetFlag(FLAG_CONTROL_MODE_1);

		} else {

			MEMORY_ResetFlag(FLAG_CONTROL_MODE_1);

		}
		if ((COMMUNICATION_GetReceptionBuffer8(5) >> 2) & 0x01) {

			MEMORY_SetFlag(FLAG_CONTROL_MODE_2);

		} else {

			MEMORY_ResetFlag(FLAG_CONTROL_MODE_2);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_WRITE_PENDING_DATA_DD:

		// If length does not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(3) < 0x04)  || (COMMUNICATION_GetReceptionBuffer8(3) > 0xFC)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) >= sizeof(structMemoryFields)) || (((uint8_t)(COMMUNICATION_GetReceptionBuffer8(3) - 3)) > (sizeof(structMemoryFields) - COMMUNICATION_GetReceptionBuffer8(5)))) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Save pending data address and pending data length
		pendingDataAdress = COMMUNICATION_GetReceptionBuffer8(5);
		pendingDataLength = (COMMUNICATION_GetReceptionBuffer8(3) - 3);

		// Copy receive buffer values to pending data buffer
		COMMUNICATION_PROTOCOL_CopyBufferToBuffer(6, pendingDataLength);

		// Set pending data registered flag
		MEMORY_SetFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_TRIGGER_PENDING_DATA_DD:

		// If pending data registered flag is not set, set instruction error flag, refuse
		// packet and clear reception buffer
		if (!MEMORY_CheckFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED)) {

			// Set instruction error flag
			MEMORY_SetError(ERROR_INSTRUCTION);

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x02) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Copy pending data buffer values to memory
		COMMUNICATION_PROTOCOL_CopyBufferToMemory(pendingDataAdress, pendingDataLength, 0, MODE_PENDING_DATA_BUFFER);

		// Reset pending data registered flag
		MEMORY_ResetFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_SYNCHRONIZED_READ_DATA_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) < 0x05) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) >= sizeof(structMemoryFields)) || (COMMUNICATION_GetReceptionBuffer8(6) > (sizeof(structMemoryFields) - COMMUNICATION_GetReceptionBuffer8(5)))) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Search servo id, set pending servo id, activate synchronized read
		// processing and copy memory values to transmission buffer
		if (COMMUNICATION_GetReceptionBuffer8(7) == BROADCAST_ID) {

			// Set pending servo id
			pendingServoId = 0;

			// Set synchronized read in progress flag
			MEMORY_SetFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS);

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6));

			// Copy receive buffer values to pending data buffer
			COMMUNICATION_PROTOCOL_CopyBufferToBuffer(0, (COMMUNICATION_GetReceptionBuffer8(3) + 4));

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;


		} else {

			// Set pending servo id
			pendingServoId = BROADCAST_ID;

			// Search for servo id and pending servo id
			for (index = 7; index < (COMMUNICATION_GetReceptionBuffer8(3) + 3); index++) {

				if (COMMUNICATION_GetReceptionBuffer8(index) < pendingServoId) {

					// Set pending servo id
					pendingServoId = COMMUNICATION_GetReceptionBuffer8(index);

				}

				if (COMMUNICATION_GetReceptionBuffer8(index) == MEMORY_GetServoId()) {

					// Set synchronized read in progress flag
					MEMORY_SetFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS);

					// Create status packet
					COMMUNICATION_PROTOCOL_CreateStatusPacket(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6));

					// Copy receive buffer values to pending data buffer
					COMMUNICATION_PROTOCOL_CopyBufferToBuffer(0, (COMMUNICATION_GetReceptionBuffer8(3) + 4));

					// Clear reception buffer
					COMMUNICATION_ClearReceptionBuffer();

					return;

				}

			}

			// If servo id could not be found, refuse packet and clear reception buffer
			if (!MEMORY_CheckFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS)) {

				// Clear reception buffer
				COMMUNICATION_ClearReceptionBuffer();

				return;

			}

		}

		// Initiate transmission of transmission buffer and deactivate synchronized
		// read processing or activate synchronized read timeout
		if (pendingServoId == MEMORY_GetServoId()) {

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

			// Reset synchronized read in progress flag
			MEMORY_ResetFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS);

		} else {

			// Calculate synchronized read timeout
			synchronizedReadTimeout = COMMON_MAXIMUM((uint16_t)(((((uint32_t)(6 << 3) * 40000) / MEMORY_GetBaudrate()) + 1) + MEMORY_GetStatusReturnDelayTime()), 2);

			// Activate synchronized read timeout
			TASK_SetSynchronizedReadTimeout(synchronizedReadTimeout);

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_SYNCHRONIZED_WRITE_DATA_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) < 0x06) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) >= sizeof(structMemoryFields)) || (COMMUNICATION_GetReceptionBuffer8(6) > (sizeof(structMemoryFields) - COMMUNICATION_GetReceptionBuffer8(5)))) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Search servo id and copy reception buffer values to memory
		for (index = 7; index < (COMMUNICATION_GetReceptionBuffer8(3) + 3); index += (COMMUNICATION_GetReceptionBuffer8(6) + 1)) {

			if (COMMUNICATION_GetReceptionBuffer8(index) == MEMORY_GetServoId()) {

				// Copy reception buffer values to memory
				COMMUNICATION_PROTOCOL_CopyBufferToMemory(COMMUNICATION_GetReceptionBuffer8(5), COMMUNICATION_GetReceptionBuffer8(6), (index + 1), MODE_RECEPTION_BUFFER);

				break;

			}

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_READ_LOGGING_DATA_DD:

		// If servo id is broadcast id, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(2) == BROADCAST_ID) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x02) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Add servo id to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetServoId());

		// Add length to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(0x0E);

		// Add errors to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetCompressedErrors());

		// Add logging values to transmission buffer
		COMMUNICATION_AddTransmissionBuffer32(LOGGING_GetEndurance());
		COMMUNICATION_AddTransmissionBuffer32(LOGGING_GetPerformance());
		COMMUNICATION_AddTransmissionBuffer16(LOGGING_GetOvercurrentTime());
		COMMUNICATION_AddTransmissionBuffer16(LOGGING_GetOvercurrentTime());

		// Add checksum and initiate transmission of transmission buffer
		COMMUNICATION_TransmitTransmissionBuffer(0);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_READ_CHECKSUM_DD:

		// If servo id is broadcast id, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(2) == BROADCAST_ID) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x02) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// Add servo id to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetServoId());

		// Add length to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(0x04);

		// Add errors to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetCompressedErrors());

		// Add firmware checksum to transmission buffer
		COMMUNICATION_AddTransmissionBuffer16(MEMORY_GetChecksum());

		// Add packet checksum and initiate transmission of transmission buffer
		COMMUNICATION_TransmitTransmissionBuffer(0);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_SET_POSITION_OFFSET_DD:

		// If length does not match or servo id is broadcast id in case of an applied
		// position value, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(3) != 0x06) && ((COMMUNICATION_GetReceptionBuffer8(3) != 0x08) || (COMMUNICATION_GetReceptionBuffer8(2) == BROADCAST_ID))) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) != 0x01) || (COMMUNICATION_GetReceptionBuffer8(6) != 0x02) || (COMMUNICATION_GetReceptionBuffer8(7) != 0x03) || (COMMUNICATION_GetReceptionBuffer8(8) != 0x04)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Set position offset
		if (COMMUNICATION_GetReceptionBuffer8(3) == 0x06) {

			MEMORY_SetPositionOffset((MEMORY_GetActualPositionMeasurement() + MEMORY_GetPositionOffset()) % 4096);
			OBSERVER_ResetEstimation();

		} else if (COMMUNICATION_GetReceptionBuffer8(3) == 0x08) {

			MEMORY_SetPositionOffset((MEMORY_GetPositionOffset() + COMMUNICATION_GetReceptionBuffer16(9)) % 4096);
			OBSERVER_ResetEstimation();

		}

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_REBOOT_SERVO_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x06) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) != 0x01) || (COMMUNICATION_GetReceptionBuffer8(6) != 0x02) || (COMMUNICATION_GetReceptionBuffer8(7) != 0x03) || (COMMUNICATION_GetReceptionBuffer8(8) != 0x04)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Reboot controller
		COMMON_Reboot();

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_RESET_ERRORS_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x06) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) != 0x01) || (COMMUNICATION_GetReceptionBuffer8(6) != 0x02) || (COMMUNICATION_GetReceptionBuffer8(7) != 0x03) || (COMMUNICATION_GetReceptionBuffer8(8) != 0x04)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Reset errors
		MEMORY_SetErrors(0);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	case INSTRUCTION_RESET_SETTINGS_DD:

		// If length does not match, refuse packet and clear reception buffer
		if (COMMUNICATION_GetReceptionBuffer8(3) != 0x06) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If parameters do not match, refuse packet and clear reception buffer
		if ((COMMUNICATION_GetReceptionBuffer8(5) != 0x01) || (COMMUNICATION_GetReceptionBuffer8(6) != 0x02) || (COMMUNICATION_GetReceptionBuffer8(7) != 0x03) || (COMMUNICATION_GetReceptionBuffer8(8) != 0x04)) {

			// Clear reception buffer
			COMMUNICATION_ClearReceptionBuffer();

			return;

		}

		// If status return level permits and servo id is not broadcast id, create and
		// transmit status packet
		if ((MEMORY_GetStatusReturnLevel() >= 2) && (COMMUNICATION_GetReceptionBuffer8(2) != BROADCAST_ID)) {

			// Create status packet
			COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

		}

		// Reset motor power activated flag
		MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);

		// Reset non-volatile values to default values, except servo id and baudrate divider
		MEMORY_Reset();

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

		break;

	default:

		// Set instruction error flag
		MEMORY_SetError(ERROR_INSTRUCTION);

		// Clear reception buffer
		COMMUNICATION_ClearReceptionBuffer();

	}

}

// Process synchronized read timeout
void COMMUNICATION_PROTOCOL_ProcessSynchronizedReadTimeout(void) {

	// Private variables
	uint8_t index;

	if (MEMORY_CheckFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS)) {

		// Set pending servo id
		if (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(7) == BROADCAST_ID) {

			// Set pending servo id
			pendingServoId++;

		} else {

			// Set last servo id
			lastServoId = pendingServoId;

			// Set pending servo id
			pendingServoId = BROADCAST_ID;

			// Search for servo id and pending servo id
			for (index = 7; index < (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(3) + 3); index++) {

				if ((COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(index) < pendingServoId) && (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(index) > lastServoId)) {

					// Set pending servo id
					pendingServoId = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(index);

				}

			}

		}

		// If no new pending servo id was found, deactivate synchronized read processing
		if (pendingServoId == BROADCAST_ID) {

			// Reset synchronized read in progress flag
			MEMORY_ResetFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS);

			return;

		}

		// Initiate transmission of transmission buffer and deactivate synchronized
		// read processing or activate synchronized read timeout
		if (pendingServoId == MEMORY_GetServoId()) {

			// Add checksum and initiate transmission of transmission buffer
			COMMUNICATION_TransmitTransmissionBuffer(0);

			// Reset synchronized read in progress flag
			MEMORY_ResetFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS);

		} else {

			// Activate synchronized read timeout
			TASK_SetSynchronizedReadTimeout(synchronizedReadTimeout);

		}

	}

}

// Copy reception buffer or pending data values to memory
static void COMMUNICATION_PROTOCOL_CopyBufferToMemory(uint8_t memoryAddress, uint8_t memoryLength, uint8_t bufferAddress, uint8_t mode) {

	// Private variables
	uint8_t value8;
	uint16_t value16;
	uint32_t value32;

	switch (memoryAddress) {

	// Model number
	case offsetof(structMemoryFields, modelNumber):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetModelNumber(value16);

	// Firmware version
	case offsetof(structMemoryFields, firmwareVersion):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetFirmwareVersion(value16);

	// Servo id
	case offsetof(structMemoryFields, servoId):

		if (!COMMUNICATION_PROTOCOL_GetValue8(&value8, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetServoId(value8);

	// Status return level
	case offsetof(structMemoryFields, statusReturnLevel):

		if (!COMMUNICATION_PROTOCOL_GetValue8(&value8, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetStatusReturnLevel(value8);

	// Status return delay time
	case offsetof(structMemoryFields, statusReturnDelayTime):

		if (!COMMUNICATION_PROTOCOL_GetValue8(&value8, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetStatusReturnDelayTime(value8);

	// Baudrate divider
	case offsetof(structMemoryFields, baudrateDivider):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetBaudrateDivider(value16);

	// Position offset
	case offsetof(structMemoryFields, positionOffset):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		// Not implemented for reasons of safety. Data bytes are refused. Position offset has to be written with a special command.
		// MEMORY_SetPositionOffset(value16);

	// Position limit clockwise
	case offsetof(structMemoryFields, positionLimitClockwise):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetPositionLimitClockwise(value32);

	// Position limit counterclockwise
	case offsetof(structMemoryFields, positionLimitCounterclockwise):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetPositionLimitCounterclockwise(value32);

	// Torque limit
	case offsetof(structMemoryFields, torqueLimit):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetTorqueLimit(value32);

	// Current limit
	case offsetof(structMemoryFields, currentLimit):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetCurrentLimit(value32);

	// Lower voltage limit
	case offsetof(structMemoryFields, voltageLimitLow):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetVoltageLimitLow(value16);

	// Higher voltage limit
	case offsetof(structMemoryFields, voltageLimitHigh):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetVoltageLimitHigh(value16);

	// Motor temperature limit
	case offsetof(structMemoryFields, motorTemperatureLimit):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetMotorTemperatureLimit(value16);

	// Controller temperature limit
	case offsetof(structMemoryFields, controllerTemperatureLimit):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetControllerTemperatureLimit(value16);

	// Controller temperature offset
	case offsetof(structMemoryFields, controllerTemperatureOffset):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetControllerTemperatureOffset(value16);

	// Status signal
	case offsetof(structMemoryFields, statusSignal):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetStatusSignal(value16);

	// Alarm signal
	case offsetof(structMemoryFields, alarmSignal):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetAlarmSignal(value16);

	// Alarm shutdown
	case offsetof(structMemoryFields, alarmShutdown):

		if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetAlarmShutdown(value16);

    // Skip zone parameter
    case offsetof(structMemoryFields, skipZoneParameter):

        if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

            break;

        }
        MEMORY_SetSkipZoneParameter(value32);

	// Feed-forward control parameter
	case offsetof(structMemoryFields, feedForwardControlParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetFeedForwardControlParameter(value32);

	// Proportional position control parameter
	case offsetof(structMemoryFields, proportionalPositionControlParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetProportionalPositionControlParameter(value32);

	// Integral position control parameter
	case offsetof(structMemoryFields, integralPositionControlParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetIntegralPositionControlParameter(value32);

	// Derivative position control parameter
	case offsetof(structMemoryFields, derivativePositionControlParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetDerivativePositionControlParameter(value32);

	// Position compliance parameter
	case offsetof(structMemoryFields, positionComplianceParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetPositionComplianceParameter(value32);

	// Proportional speed control parameter
	case offsetof(structMemoryFields, proportionalSpeedControlParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetProportionalSpeedControlParameter(value32);

	// Integral speed control parameter
	case offsetof(structMemoryFields, integralSpeedControlParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetIntegralSpeedControlParameter(value32);

	// Derivative speed control parameter
	case offsetof(structMemoryFields, derivativeSpeedControlParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetDerivativeSpeedControlParameter(value32);

	// Speed compliance parameter
	case offsetof(structMemoryFields, speedComplianceParameter):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetSpeedComplianceParameter(value32);

	// Desired position
	case offsetof(structMemoryFields, desiredPosition):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetDesiredPosition(value32);

	// Desired speed
	case offsetof(structMemoryFields, desiredSpeed):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetDesiredSpeed(value32);

	// Desired acceleration
	case offsetof(structMemoryFields, desiredAcceleration):

		if (!COMMUNICATION_PROTOCOL_GetValue32(&value32, &memoryLength, &bufferAddress, mode)) {

			break;

		}
		MEMORY_SetDesiredAcceleration(value32);

		// Desired acceleration
		case offsetof(structMemoryFields, desiredPulseWidth):

			if (!COMMUNICATION_PROTOCOL_GetValue16(&value16, &memoryLength, &bufferAddress, mode)) {

				break;

			}
			MEMORY_SetDesiredPulseWidth(value16);

	}

}

// Translate between DD and Robotis memory formats
static void COMMUNICATION_PROTOCOL_TranslateFormat(uint8_t memoryAddress, uint8_t memoryLength, uint8_t bufferAddress, uint8_t mode) {

	// Private variables
	uint8_t processableData = 1;

	if (mode == MODE_READ_DATA) {

		// Add servo id to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetServoId());

		// Add length to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(memoryLength + 2);

		// Add errors to transmission buffer
		COMMUNICATION_AddTransmissionBuffer8(MEMORY_TranslateCompressedErrors());

		// Reset errors
		MEMORY_SetErrors(0);

	}

	// Translate memory formats
	while (processableData) {

		// Remap memory addresses
		switch (memoryAddress) {

		case ADDRESS_EEPROM_MODEL_NUMBER_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer16(MEMORY_GetModelNumber());

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetModelNumber(COMMUNICATION_GetReceptionBuffer16(bufferAddress));

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetModelNumber(COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(bufferAddress));

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_MODEL_NUMBER_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_VERSION_OF_FIRMWARE:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8((uint8_t)MEMORY_GetFirmwareVersion());

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetFirmwareVersion((uint16_t)COMMUNICATION_GetReceptionBuffer8(bufferAddress));

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetFirmwareVersion((uint16_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress));

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_ID:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetServoId());

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetServoId(COMMUNICATION_GetReceptionBuffer8(bufferAddress));

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetServoId(COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress));

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_BAUDRATE:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8((uint8_t)(2000000 / MEMORY_GetBaudrate()) - 1);

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetBaudrate((uint32_t)(2000000 / (COMMUNICATION_GetReceptionBuffer8(bufferAddress) + 1)));

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetServoId((uint32_t)(2000000 / (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress) + 1)));

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_RETURN_DELAY_TIME:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetStatusReturnDelayTime() > 20) {

						COMMUNICATION_AddTransmissionBuffer8(255);

					} else {

						COMMUNICATION_AddTransmissionBuffer8((uint8_t)(MEMORY_GetStatusReturnDelayTime() * 25) >> 1);

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetStatusReturnDelayTime(((uint16_t)COMMUNICATION_GetReceptionBuffer8(bufferAddress) << 1) / 25);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetStatusReturnDelayTime(((uint16_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress) << 1) / 25);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_CLOCKWISE_ANGLE_LIMIT_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetPositionLimitClockwise() >= HALF_POSITION_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if ((MEMORY_GetPositionLimitClockwise() == 0) || (MEMORY_GetPositionLimitClockwise() <= -HALF_POSITION_RANGE32)) {

						COMMUNICATION_AddTransmissionBuffer16(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer16((uint16_t)(((MEMORY_GetPositionLimitClockwise() << 10) / FULL_POSITION_RANGE32) + 511));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetPositionLimitClockwise((((fxp32_t)COMMUNICATION_GetReceptionBuffer16(bufferAddress) - 511) * FULL_POSITION_RANGE32) >> 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetPositionLimitClockwise((((fxp32_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(bufferAddress) - 511) * FULL_POSITION_RANGE32) >> 10);

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_CLOCKWISE_ANGLE_LIMIT_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_COUNTERCLOCKWISE_ANGLE_LIMIT_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if ((MEMORY_GetPositionLimitCounterclockwise() == 0) || (MEMORY_GetPositionLimitCounterclockwise() >= HALF_POSITION_RANGE32)) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if (MEMORY_GetPositionLimitCounterclockwise() <= -HALF_POSITION_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer16((uint16_t)(((MEMORY_GetPositionLimitCounterclockwise() << 10) / FULL_POSITION_RANGE32) + 511));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetPositionLimitCounterclockwise((((fxp32_t)COMMUNICATION_GetReceptionBuffer16(bufferAddress) - 511) * FULL_POSITION_RANGE32) >> 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetPositionLimitCounterclockwise((((fxp32_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(bufferAddress) - 511) * FULL_POSITION_RANGE32) >> 10);

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_COUNTERCLOCKWISE_ANGLE_LIMIT_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_RESERVERD_1:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_TEMPERATURE_LIMIT:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetMotorTemperatureLimit() >= 16320) {

						COMMUNICATION_AddTransmissionBuffer8(255);

					} else if (MEMORY_GetMotorTemperatureLimit() <= 0) {

						COMMUNICATION_AddTransmissionBuffer8(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer8((uint8_t)FXP16_Int(MEMORY_GetMotorTemperatureLimit()));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetMotorTemperatureLimit(FXP16_Init(COMMUNICATION_GetReceptionBuffer8(bufferAddress)));

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetMotorTemperatureLimit(FXP16_Init(COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress)));

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_LOWER_VOLTAGE_LIMIT:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetVoltageLimitLow() >= 1632) {

						COMMUNICATION_AddTransmissionBuffer8(255);

					} else if (MEMORY_GetVoltageLimitLow() <= 0) {

						COMMUNICATION_AddTransmissionBuffer8(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer8((uint8_t)FXP16_Int(MEMORY_GetVoltageLimitLow() * 10));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetVoltageLimitLow(FXP16_Init(COMMUNICATION_GetReceptionBuffer8(bufferAddress)) / 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetVoltageLimitLow(FXP16_Init(COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress)) / 10);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_UPPER_VOLTAGE_LIMIT:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetVoltageLimitHigh() >= 1632) {

						COMMUNICATION_AddTransmissionBuffer8(255);

					} else if (MEMORY_GetVoltageLimitHigh() <= 0) {

						COMMUNICATION_AddTransmissionBuffer8(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer8((uint8_t)FXP16_Int(MEMORY_GetVoltageLimitHigh() * 10));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetVoltageLimitHigh(FXP16_Init(COMMUNICATION_GetReceptionBuffer8(bufferAddress)) / 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetVoltageLimitHigh(FXP16_Init(COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress)) / 10);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_TORQUE_LIMIT_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetTorqueLimit() >= FULL_TORQUE_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if (MEMORY_GetTorqueLimit() <= 0) {

						COMMUNICATION_AddTransmissionBuffer16(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer16((uint16_t)((MEMORY_GetTorqueLimit() << 10) / FULL_TORQUE_RANGE32));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetTorqueLimit(((fxp32_t)COMMUNICATION_GetReceptionBuffer16(bufferAddress) * FULL_TORQUE_RANGE32) >> 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetTorqueLimit(((fxp32_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(bufferAddress) * FULL_TORQUE_RANGE32) >> 10);

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_TORQUE_LIMIT_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_STATUS_RETURN_LEVEL:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					COMMUNICATION_AddTransmissionBuffer8(MEMORY_GetStatusReturnLevel());

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetStatusReturnLevel(COMMUNICATION_GetReceptionBuffer8(bufferAddress));

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetStatusReturnLevel(COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress));

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_ALARM_LED:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Private variables
					uint8_t dataRobotis = 0;

					if (MEMORY_GetAlarmSignal() & (1 << ERROR_VOLTAGE_LIMIT)) {

						dataRobotis |= 0x01;

					}
					if (MEMORY_GetAlarmSignal() & (1 << ERROR_POSITION_LIMIT)) {

						dataRobotis |= 0x02;

					}
					if (MEMORY_GetAlarmSignal() & (1 << ERROR_MOTOR_TEMPERATURE_LIMIT)) {

						dataRobotis |= 0x04;

					}
					if (MEMORY_GetAlarmSignal() & (1 << ERROR_PARAMETER_RANGE)) {

						dataRobotis |= 0x08;

					}
					if (MEMORY_GetAlarmSignal() & (1 << ERROR_CHECKSUM)) {

						dataRobotis |= 0x10;

					}
					if (MEMORY_GetAlarmSignal() & (1 << ERROR_TORQUE_LIMIT)) {

						dataRobotis |= 0x20;

					}
					if (MEMORY_GetAlarmSignal() & (1 << ERROR_INSTRUCTION)) {

						dataRobotis |= 0x40;

					}
					COMMUNICATION_AddTransmissionBuffer8(dataRobotis);

				} else {

					// Private variables
					uint8_t dataRobotis = 0;
					uint16_t dataDD = 0;

					// Save translated data to memory
					if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

						dataRobotis = COMMUNICATION_GetReceptionBuffer8(bufferAddress);

					} else {

						dataRobotis = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress);

					}
					if (dataRobotis & 0x01) {

						dataDD |= (1 << ERROR_VOLTAGE_LIMIT);

					}
					if (dataRobotis & 0x02) {

						dataDD |= (1 << ERROR_POSITION_LIMIT);

					}
					if (dataRobotis & 0x04) {

						dataDD |= (1 << ERROR_MOTOR_TEMPERATURE_LIMIT);

					}
					if (dataRobotis & 0x08) {

						dataDD |= (1 << ERROR_PARAMETER_RANGE);

					}
					if (dataRobotis & 0x10) {

						dataDD |= (1 << ERROR_CHECKSUM);

					}
					if (dataRobotis & 0x20) {

						dataDD |= (1 << ERROR_TORQUE_LIMIT);

					}
					if (dataRobotis & 0x40) {

						dataDD |= (1 << ERROR_INSTRUCTION);

					}
					MEMORY_SetAlarmSignal(dataDD);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_ALERM_SHUTDOWN:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Private variables
					uint8_t dataRobotis = 0;

					if (MEMORY_GetAlarmShutdown() & (1 << ERROR_VOLTAGE_LIMIT)) {

						dataRobotis |= 0x01;

					}
					if (MEMORY_GetAlarmShutdown() & (1 << ERROR_POSITION_LIMIT)) {

						dataRobotis |= 0x02;

					}
					if (MEMORY_GetAlarmShutdown() & (1 << ERROR_MOTOR_TEMPERATURE_LIMIT)) {

						dataRobotis |= 0x04;

					}
					if (MEMORY_GetAlarmShutdown() & (1 << ERROR_PARAMETER_RANGE)) {

						dataRobotis |= 0x08;

					}
					if (MEMORY_GetAlarmShutdown() & (1 << ERROR_CHECKSUM)) {

						dataRobotis |= 0x10;

					}
					if (MEMORY_GetAlarmShutdown() & (1 << ERROR_TORQUE_LIMIT)) {

						dataRobotis |= 0x20;

					}
					if (MEMORY_GetAlarmShutdown() & (1 << ERROR_INSTRUCTION)) {

						dataRobotis |= 0x40;

					}
					COMMUNICATION_AddTransmissionBuffer8(dataRobotis);

				} else {

					// Private variables
					uint8_t dataRobotis = 0;
					uint16_t dataDD = 0;

					// Save translated data to memory
					if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

						dataRobotis = COMMUNICATION_GetReceptionBuffer8(bufferAddress);

					} else {

						dataRobotis = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress);

					}
					if (dataRobotis & 0x01) {

						dataDD |= (1 << ERROR_VOLTAGE_LIMIT);

					}
					if (dataRobotis & 0x02) {

						dataDD |= (1 << ERROR_POSITION_LIMIT);

					}
					if (dataRobotis & 0x04) {

						dataDD |= (1 << ERROR_MOTOR_TEMPERATURE_LIMIT);

					}
					if (dataRobotis & 0x08) {

						dataDD |= (1 << ERROR_PARAMETER_RANGE);

					}
					if (dataRobotis & 0x10) {

						dataDD |= (1 << ERROR_CHECKSUM);

					}
					if (dataRobotis & 0x20) {

						dataDD |= (1 << ERROR_TORQUE_LIMIT);

					}
					if (dataRobotis & 0x40) {

						dataDD |= (1 << ERROR_INSTRUCTION);

					}
					MEMORY_SetAlarmShutdown(dataDD);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_RESERVED_2:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_DOWN_CALIBRATION_LOW:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_DOWN_CALIBRATION_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_UP_CALIBRATION_LOW:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_EEPROM_UP_CALIBRATION_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_TORQUE_ENABLE:

			if (memoryLength >= 1) {

				// Private variables
				uint8_t data = 0;

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED));

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					data = COMMUNICATION_GetReceptionBuffer8(bufferAddress);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					data = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress);

				}

				if (mode != MODE_READ_DATA) {

					// Save translated data to memory
					if (data == MODE_MOTOR_POWER_OFF) {

						MEMORY_ResetFlag(FLAG_MOTOR_POWER_ACTIVATED);
						CONTROL_ResetControlStates();
						MOTOR_Run(0);

					} else if (data == MODE_MOTOR_POWER_ON) {

						CONTROL_ResetControlStates();
						MEMORY_SetFlag(FLAG_MOTOR_POWER_ACTIVATED);

					}

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_LED:

			if (memoryLength >= 1) {

				// Private variables
				uint8_t data = 0;

				if (mode == MODE_READ_DATA) {

					if (MEMORY_GetStatusSignal() >> 8) {

						// Add translated data to transmission buffer
						COMMUNICATION_AddTransmissionBuffer8(1);

					} else {

						// Add translated data to transmission buffer
						COMMUNICATION_AddTransmissionBuffer8(0);

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					data = COMMUNICATION_GetReceptionBuffer8(bufferAddress);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					data = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress);

				}

				if (mode != MODE_READ_DATA) {

					// Save translated data to memory
					if (data) {

						MEMORY_SetStatusSignal(DEFAULT_STATUS_SIGNAL);

					} else {

						MEMORY_SetStatusSignal(0);

					}

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_CLOCKWISE_COMPLIANCE_MARGIN:
		case ADDRESS_RAM_COUNTERCLOCKWISE_COMPLIANCE_MARGIN:

            if (memoryLength >= 1) {

                if (mode == MODE_READ_DATA) {

                    COMMUNICATION_AddTransmissionBuffer8((uint8_t)((MEMORY_GetSkipZoneParameter() * 254) / LIMIT_SKIP_ZONE_PARAMETER_HIGH));

                } else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

                    // Save translated data to memory
                    MEMORY_SetSkipZoneParameter(((fxp32_t)COMMUNICATION_GetReceptionBuffer8(bufferAddress) * LIMIT_SKIP_ZONE_PARAMETER_HIGH) / 254);

                } else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

                    // Save translated pending data to memory
                    MEMORY_SetSkipZoneParameter(((fxp32_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress) * LIMIT_SKIP_ZONE_PARAMETER_HIGH) / 254);

                }

                // Update addresses and length
                memoryAddress += 1;
                memoryLength -= 1;
                bufferAddress += 1;

            } else {

                processableData = 0;

            }
            break;

		case ADDRESS_RAM_CLOCKWISE_COMPLIANCE_SLOPE:
		case ADDRESS_RAM_COUNTERCLOCKWISE_COMPLIANCE_SLOPE:

            if (memoryLength >= 1) {

                if (mode == MODE_READ_DATA) {

                    COMMUNICATION_AddTransmissionBuffer8((uint8_t)((253 - ((MEMORY_GetPositionComplianceParameter() * 253) / LIMIT_POSITION_COMPLIANCE_PARAMETER_HIGH)) + 1));

                } else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

                    // Save translated data to memory
                    MEMORY_SetPositionComplianceParameter(((fxp32_t)(253 - (COMMUNICATION_GetReceptionBuffer8(bufferAddress) - 1)) * LIMIT_POSITION_COMPLIANCE_PARAMETER_HIGH) / 253);

                } else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

                    // Save translated pending data to memory
                    MEMORY_SetPositionComplianceParameter(((fxp32_t)(253 - (COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress) - 1)) * LIMIT_POSITION_COMPLIANCE_PARAMETER_HIGH) / 254);

                }

                // Update addresses and length
                memoryAddress += 1;
                memoryLength -= 1;
                bufferAddress += 1;

            } else {

                processableData = 0;

            }
            break;

		case ADDRESS_RAM_GOAL_POSITION_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetDesiredPosition() >= HALF_POSITION_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if (MEMORY_GetDesiredPosition() <= -HALF_POSITION_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer16((uint16_t)(((MEMORY_GetDesiredPosition() << 10) / FULL_POSITION_RANGE32) + 511));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// ToDo: Delete me! (Workaround)
                    //if (!(MEMORY_GetErrors() & MEMORY_GetAlarmShutdown())) {

                        MEMORY_SetFlag(FLAG_MOTOR_POWER_ACTIVATED);

                    //}

                    // Save translated data to memory
                    MEMORY_SetDesiredPosition((((fxp32_t)COMMUNICATION_GetReceptionBuffer16(bufferAddress) - 511) * FULL_POSITION_RANGE32) >> 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

                    // ToDo: Delete me! (Workaround)
                    //if (!(MEMORY_GetErrors() & MEMORY_GetAlarmShutdown())) {

                        MEMORY_SetFlag(FLAG_MOTOR_POWER_ACTIVATED);

                    //}

                    // Save translated pending data to memory
                    MEMORY_SetDesiredPosition((((fxp32_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(bufferAddress) - 511) * FULL_POSITION_RANGE32) >> 10);

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_GOAL_POSITION_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_MOVING_SPEED_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (COMMON_ABSOLUTE(MEMORY_GetDesiredSpeed()) >= FULL_SPEED_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else {

						COMMUNICATION_AddTransmissionBuffer16((uint16_t)((COMMON_ABSOLUTE(MEMORY_GetDesiredSpeed()) << 10) / FULL_SPEED_RANGE32));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

                    // ToDo: Delete me! (Workaround)
                    //if (!(MEMORY_GetErrors() & MEMORY_GetAlarmShutdown())) {

                        MEMORY_SetFlag(FLAG_MOTOR_POWER_ACTIVATED);

                    //}

                    // Save translated data to memory
                    MEMORY_SetDesiredSpeed(((fxp32_t)COMMUNICATION_GetReceptionBuffer16(bufferAddress) * FULL_SPEED_RANGE32) >> 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

                    // ToDo: Delete me! (Workaround)
                    //if (!(MEMORY_GetErrors() & MEMORY_GetAlarmShutdown())) {

                        MEMORY_SetFlag(FLAG_MOTOR_POWER_ACTIVATED);

                    //}

                    // Save translated pending data to memory
                    MEMORY_SetDesiredSpeed(((fxp32_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(bufferAddress) * FULL_SPEED_RANGE32) >> 10);

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_MOVING_SPEED_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_TORQUE_LIMIT_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetTorqueLimit() >= FULL_TORQUE_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if (MEMORY_GetTorqueLimit() <= 0) {

						COMMUNICATION_AddTransmissionBuffer16(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer16((uint16_t)((MEMORY_GetTorqueLimit() << 10) / FULL_TORQUE_RANGE32));

					}

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated data to memory
					MEMORY_SetTorqueLimit(((fxp32_t)COMMUNICATION_GetReceptionBuffer16(bufferAddress) * FULL_TORQUE_RANGE32) >> 10);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					// Save translated pending data to memory
					MEMORY_SetTorqueLimit(((fxp32_t)COMMUNICATION_PROTOCOL_GetPendingDataBuffer16(bufferAddress) * FULL_TORQUE_RANGE32) >> 10);

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_TORQUE_LIMIT_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_POSITION_LOW:

			if (memoryLength >= 2) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetActualPosition() >= HALF_POSITION_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if (MEMORY_GetActualPosition() <= -HALF_POSITION_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer16((uint16_t)(((MEMORY_GetActualPosition() << 10) / FULL_POSITION_RANGE32) + 511));

					}

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_POSITION_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_SPEED_LOW:

			if (memoryLength >= 2) {

				// Private variables
				uint16_t dataRobotis = 0;

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetActualSpeed() >= FULL_SPEED_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if (MEMORY_GetActualSpeed() <= -FULL_SPEED_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(2047);

					} else {

						dataRobotis = (uint16_t)((COMMON_ABSOLUTE(MEMORY_GetActualSpeed()) << 10) / FULL_SPEED_RANGE32);
						if (MEMORY_GetActualSpeed() < 0) {

							dataRobotis |= (1 << 10);

						}
						COMMUNICATION_AddTransmissionBuffer16(dataRobotis);

					}

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_SPEED_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_LOAD_LOW:

			if (memoryLength >= 2) {

				// Private variables
				uint16_t dataRobotis = 0;

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetActualTorque() >= FULL_TORQUE_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(1023);

					} else if (MEMORY_GetActualTorque() <= -FULL_TORQUE_RANGE32) {

						COMMUNICATION_AddTransmissionBuffer16(2047);

					} else {

						dataRobotis = (uint16_t)((COMMON_ABSOLUTE(MEMORY_GetActualTorque()) << 10) / FULL_TORQUE_RANGE32);
						if (MEMORY_GetActualTorque() < 0) {

							dataRobotis |= (1 << 10);

						}
						COMMUNICATION_AddTransmissionBuffer16(dataRobotis);

					}

				}

				// Update addresses and length
				memoryAddress += 2;
				memoryLength -= 2;
				bufferAddress += 2;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_LOAD_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_VOLTAGE:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetActualVoltage() >= 1632) {

						COMMUNICATION_AddTransmissionBuffer8(255);

					} else if (MEMORY_GetActualVoltage() <= 0) {

						COMMUNICATION_AddTransmissionBuffer8(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer8((uint8_t)FXP16_Int(MEMORY_GetActualVoltage() * 10));

					}

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PRESENT_TEMPERATURE:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					if (MEMORY_GetActualMotorTemperature() >= 16320) {

						COMMUNICATION_AddTransmissionBuffer8(255);

					} else if (MEMORY_GetActualMotorTemperature() <= 0) {

						COMMUNICATION_AddTransmissionBuffer8(0);

					} else {

						COMMUNICATION_AddTransmissionBuffer8((uint8_t)FXP16_Int(MEMORY_GetActualMotorTemperature()));

					}

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_REGISTERED_INSTRUCTION:

			if (memoryLength >= 1) {

				// Private variables
				uint8_t data = 0;

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(MEMORY_CheckFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED));

				} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_DATA_WITHOUT_STATUS_PACKET)) {

					data = COMMUNICATION_GetReceptionBuffer8(bufferAddress);

				} else if ((mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET)) {

					data = COMMUNICATION_PROTOCOL_GetPendingDataBuffer8(bufferAddress);

				}

				if (mode != MODE_READ_DATA) {

					// Save translated data to memory
					if (data == 0) {

						MEMORY_ResetFlag(FLAG_COMMUNICATION_PENDING_DATA_REGISTERED);

					}

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_RESERVED_3:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_MOVING:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					if (MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED) && ((MEMORY_GetDesiredPosition() >> 10) != (MEMORY_GetActualPosition() >> 10))) {

						// Add translated data to transmission buffer
						COMMUNICATION_AddTransmissionBuffer8(1);

					} else {

						// Add translated data to transmission buffer
						COMMUNICATION_AddTransmissionBuffer8(0);

					}

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_LOCK:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add translated data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED));

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PUNCH_LOW:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		case ADDRESS_RAM_PUNCH_HIGH:

			if (memoryLength >= 1) {

				if (mode == MODE_READ_DATA) {

					// Add dummy data to transmission buffer
					COMMUNICATION_AddTransmissionBuffer8(0);

				}

				// Update addresses and length
				memoryAddress += 1;
				memoryLength -= 1;
				bufferAddress += 1;

			} else {

				processableData = 0;

			}
			break;

		default:

			processableData = 0;

		}

	}

	// If status return level permits and servo id is not broadcast id, create and
	// transmit status packet
	if (mode == MODE_READ_DATA) {

		// Fill unused transmission buffer with dummy data
		for (; memoryLength > 0; memoryLength--) {

			COMMUNICATION_AddTransmissionBuffer8(0);

		}

		// Add checksum and initiate transmission of transmission buffer
		COMMUNICATION_TransmitTransmissionBuffer(0);

	} else if ((mode == MODE_WRITE_DATA_WITH_STATUS_PACKET) || (mode == MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET)) {

		// Create status packet
		COMMUNICATION_PROTOCOL_CreateStatusPacket(0, 0);

		// Add checksum and initiate transmission of transmission buffer
		COMMUNICATION_TransmitTransmissionBuffer(0);

	}

}
