 /*
 ******************************************************************************
 * File    		communication.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for communication functions.
 * Peripherals	GPIOA, USART1, DMA1
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

// Type includes
#include "stm32f10x.h"

// Defines
#ifndef INLINE_COMMUNICATION
#define INLINE_COMMUNICATION extern inline
#endif
#define RING_BUFFER_SIZE 4
#define TRANSMISSION_MODE 0
#define RECEPTION_MODE 1
#define BROADCAST_ID 0xFE

// Type defines
typedef struct {

	volatile uint8_t buffer[256];
	volatile uint8_t length;

} structPacketBuffer;

// Global variables
extern volatile structPacketBuffer receptionBuffer[];
extern volatile structPacketBuffer transmissionBuffer;
extern volatile uint8_t bufferReadIndex;
extern volatile uint8_t bufferWriteIndex;
extern uint8_t checksum;
extern uint16_t receptionTimeout;

// Global function prototypes
void COMMUNICATION_Initialize(void);
void COMMUNICATION_ClearStatusRegister(void);
void COMMUNICATION_UpdateBaudrate(void);
uint8_t COMMUNICATION_CheckPacketStatus(void);
uint8_t COMMUNICATION_GetReceptionBuffer8(uint8_t);
uint16_t COMMUNICATION_GetReceptionBuffer16(uint8_t);
uint32_t COMMUNICATION_GetReceptionBuffer32(uint8_t);
void COMMUNICATION_ClearReceptionBuffer(void);
void COMMUNICATION_AddTransmissionBuffer8(uint8_t);
void COMMUNICATION_AddTransmissionBuffer16(uint16_t);
void COMMUNICATION_AddTransmissionBuffer32(uint32_t);
void COMMUNICATION_TransmitTransmissionBuffer(uint16_t);
void COMMUNICATION_ProcessStatusReturnDelay(void);
void COMMUNICATION_ProcessReceptionTimeout(void);

// Module includes
#include "common.h"
#include "memory.h"
#include "task.h"

// Get one byte at assigned address from reception buffer
INLINE_COMMUNICATION uint8_t COMMUNICATION_GetReceptionBuffer8(uint8_t address) {

	uint8_t data = receptionBuffer[bufferReadIndex].buffer[address];

	return data;

}

// Get two bytes at assigned address from reception buffer
INLINE_COMMUNICATION uint16_t COMMUNICATION_GetReceptionBuffer16(uint8_t address) {

	uint16_t data = *((uint16_t*)&receptionBuffer[bufferReadIndex].buffer[address]);

	return data;

}

// Get four bytes at assigned address from reception buffer
INLINE_COMMUNICATION uint32_t COMMUNICATION_GetReceptionBuffer32(uint8_t address) {

	uint32_t data = *((uint32_t*)&receptionBuffer[bufferReadIndex].buffer[address]);

	return data;

}

// Clear reception buffer
INLINE_COMMUNICATION void COMMUNICATION_ClearReceptionBuffer(void) {

	// Switch ring buffer to next reception buffer
	COMMON_ADVANCE(bufferReadIndex, RING_BUFFER_SIZE);

}

// Add one byte to transmission buffer and update checksum
INLINE_COMMUNICATION void COMMUNICATION_AddTransmissionBuffer8(uint8_t data) {

	checksum += data;
	transmissionBuffer.buffer[transmissionBuffer.length++] = data;

}

// Add two bytes to transmission buffer and update checksum
INLINE_COMMUNICATION void COMMUNICATION_AddTransmissionBuffer16(uint16_t data) {

	uint8_t data1 = *((uint8_t*)&data);
	uint8_t data2 = *(((uint8_t*)&data) + 1);

	checksum += data1 + data2;
	transmissionBuffer.buffer[transmissionBuffer.length++] = data1;
	transmissionBuffer.buffer[transmissionBuffer.length++] = data2;

}

// Add four bytes to transmission buffer and update checksum
INLINE_COMMUNICATION void COMMUNICATION_AddTransmissionBuffer32(uint32_t data) {

	uint8_t data1 = *((uint8_t*)&data);
	uint8_t data2 = *(((uint8_t*)&data) + 1);
	uint8_t data3 = *(((uint8_t*)&data) + 2);
	uint8_t data4 = *(((uint8_t*)&data) + 3);

	checksum += data1 + data2 + data3 + data4;
	transmissionBuffer.buffer[transmissionBuffer.length++] = data1;
	transmissionBuffer.buffer[transmissionBuffer.length++] = data2;
	transmissionBuffer.buffer[transmissionBuffer.length++] = data3;
	transmissionBuffer.buffer[transmissionBuffer.length++] = data4;

}

#endif
