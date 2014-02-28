/*
 ******************************************************************************
 * File    		communication.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Communication functions.
 * Peripherals	GPIOA, USART1, DMA1
 ******************************************************************************
 */

// Includes
#define INLINE_COMMUNICATION
#include "communication.h"

// Global variables
volatile structPacketBuffer receptionBuffer[RING_BUFFER_SIZE];	// Ring buffer with multiple 256 byte reception buffers
volatile structPacketBuffer transmissionBuffer;					// Single 256 byte transmission buffer
volatile uint8_t bufferReadIndex;								// Ring buffer index for read access
volatile uint8_t bufferWriteIndex;								// Ring buffer index for write access
uint8_t checksum;												// Transmission buffer checksum
uint16_t receptionTimeout;										// Reception timeout in 25µs

// Private function prototypes
static void COMMUNICATION_SetCommunicationMode(uint8_t mode);
static void COMMUNICATION_SetBaudrate(void);
void USART1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);

// Initialize communication
void COMMUNICATION_Initialize(void) {

	// Private variables
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	uint8_t index;

	// Initialize variables
	for (index = 0; index < RING_BUFFER_SIZE; index++) {

		receptionBuffer[index].length = 0;

	}
	transmissionBuffer.buffer[0] = 0xFF;
	transmissionBuffer.buffer[1] = 0xFF;
	transmissionBuffer.length = 2;
	bufferReadIndex = 0;
	bufferWriteIndex = 0;
	checksum = 0;
	receptionTimeout = MEMORY_GetBaudrateDivider() << 1;

	// Initialize USART1 inputs and outputs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// COMTX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// COMRX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// COMT/NR
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	COMMUNICATION_SetCommunicationMode(RECEPTION_MODE);

	// Initialize DMA1 for USART1 transmission
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40013804;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)transmissionBuffer.buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Initialize DMA1 for USART1 reception
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)receptionBuffer[bufferWriteIndex].buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Initialize USART1 as 8N1 with specified baudrate
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_InitStructure.USART_BaudRate = MEMORY_GetBaudrate();
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
	USART_Cmd(USART1, ENABLE);

}

// Clear status register with unwanted interrupts
void COMMUNICATION_ClearStatusRegister(void) {

	// Read status register
    USART1->SR;

	// Read data register
    USART_ReceiveData(USART1);

}

// Update baudrate setting
void COMMUNICATION_UpdateBaudrate(void) {

	if (!MEMORY_CheckFlag(FLAG_COMMUNICATION_TRANSMISSION_IN_PROGRESS) && !MEMORY_CheckFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS)) {

		COMMUNICATION_SetBaudrate();


	} else {

		// Set flag for baudrate update
		MEMORY_SetFlag(FLAG_COMMUNICATION_BAUDRATE_UPDATE);

	}

}

// Check for a new packet in reception buffer
uint8_t COMMUNICATION_CheckPacketStatus(void) {

	if ((bufferReadIndex != bufferWriteIndex) && !MEMORY_CheckFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS) && !MEMORY_CheckFlag(FLAG_COMMUNICATION_TRANSMISSION_IN_PROGRESS)) {

		return COMMUNICATION_PROCESS;

	} else {

		return COMMUNICATION_SKIP;

	}

}

// Add checksum and initiate transmission of transmission buffer with settable delay
void COMMUNICATION_TransmitTransmissionBuffer(uint16_t delay) {

	// Set flag for transmission in progress
	MEMORY_SetFlag(FLAG_COMMUNICATION_TRANSMISSION_IN_PROGRESS);

	// Add checksum to transmission buffer
	transmissionBuffer.buffer[transmissionBuffer.length++] = ~checksum;

	// Reset checksum
	checksum = 0;

	// Initiate transmission immediately or wait for status return delay
	if ((MEMORY_GetStatusReturnDelayTime() == 0) && (delay == 0)) {

		// Disable USART interrupts
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		USART_ITConfig(USART1, USART_IT_ERR, DISABLE);

		// Change communication mode
		COMMUNICATION_SetCommunicationMode(TRANSMISSION_MODE);

		// Configure and activate DMA
		DMA1_Channel4->CMAR = (uint32_t)transmissionBuffer.buffer;
		DMA1_Channel4->CNDTR = transmissionBuffer.length;
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		DMA_Cmd(DMA1_Channel4, ENABLE);

	} else if (delay != 0) {

		// Activate status return delay
		TASK_SetStatusReturnDelay(delay);

	}

	else {

		// Activate status return delay
		TASK_SetStatusReturnDelay(MEMORY_GetStatusReturnDelayTime());

	}

}

// Process end of status return delay
void COMMUNICATION_ProcessStatusReturnDelay(void) {

	// Disable USART interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART1, USART_IT_ERR, DISABLE);

	// Change communication mode
	COMMUNICATION_SetCommunicationMode(TRANSMISSION_MODE);

	// Configure and activate DMA
	DMA1_Channel4->CMAR = (uint32_t)transmissionBuffer.buffer;
	DMA1_Channel4->CNDTR = transmissionBuffer.length;
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);

}

// Process reception timeout
void COMMUNICATION_ProcessReceptionTimeout(void) {

	// Clear interrupt flags and data register
	USART_GetITStatus(USART1, USART_IT_ORE);
	USART_GetITStatus(USART1, USART_IT_NE);
	USART_GetITStatus(USART1, USART_IT_FE);
	USART_GetITStatus(USART1, USART_IT_PE);
	USART_ReceiveData(USART1);

	// Enable USART1 interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

	// Deactivate DMA for reception
	DMA_Cmd(DMA1_Channel5, DISABLE);

	// Deactivate USART DMA mode
	USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);

	// Refuse packet and reset reception buffer
	receptionBuffer[bufferWriteIndex].length = 0;

	// Set transmission error flag
	MEMORY_SetError(ERROR_COMMUNICATION);

	// Reset flag for reception in progress
	MEMORY_ResetFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS);

	// Update baudrate setting
	if (MEMORY_CheckFlag(FLAG_COMMUNICATION_BAUDRATE_UPDATE)) {

		COMMUNICATION_SetBaudrate();

	}

}

// Set half duplex communication mode
static void COMMUNICATION_SetCommunicationMode(uint8_t mode) {

	if (mode == TRANSMISSION_MODE) {

		// Set COMT/NR to transmission mode
		GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_SET);

	} else if (mode == RECEPTION_MODE) {

		// Update baudrate setting
		if (MEMORY_CheckFlag(FLAG_COMMUNICATION_BAUDRATE_UPDATE)) {

			COMMUNICATION_SetBaudrate();

		}

		// Set COMT/NR to reception mode
		GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_RESET);

	}

}

// Set changed baudrate
static void COMMUNICATION_SetBaudrate(void) {

	// Reset flag for baudrate update
	MEMORY_ResetFlag(FLAG_COMMUNICATION_BAUDRATE_UPDATE);

	// Deactivate USART1
	USART_Cmd(USART1, DISABLE);

	// Calculate timeout from the baudrate divider multiplied by 2
	receptionTimeout = MEMORY_GetBaudrateDivider() << 1;

	// Update baudrate register
	USART1->BRR = MEMORY_GetBaudrateDivider();

	// Activate USART1
	USART_Cmd(USART1, ENABLE);

}

// USART1 interrupt handler
void USART1_IRQHandler(void) {

	// Private variables
	uint8_t receivedByte;

	// USART receive data register is not empty
	if ((USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) || USART_GetITStatus(USART1, USART_IT_ORE) != RESET) {

		// Get received byte
		receivedByte = USART_ReceiveData(USART1);

		// If packet starts with start byte, start reception
		if ((receptionBuffer[bufferWriteIndex].length == 0) && (receivedByte == 0xFF)) {

			// Set flag for reception in progress
			MEMORY_SetFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS);

			// Activate reception timeout
			TASK_SetReceptionTimeout(receptionTimeout);

		}

		// If packet does not start with two start bytes, refuse packet and reset
		// reception buffer
		if ((receptionBuffer[bufferWriteIndex].length < 2) && (receivedByte != 0xFF)) {

			// Deactivate reception timeout
			TASK_ResetReceptionTimeout();

			// Reset reception buffer
			receptionBuffer[bufferWriteIndex].length = 0;

			// Reset flag for reception in progress
			MEMORY_ResetFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS);

			return;

		}

		// If servo id does not match or does not equal broadcast id or synchronized
		// read is not in progress, refuse packet and reset reception buffer
		if ((receptionBuffer[bufferWriteIndex].length == 2) && ((receivedByte != MEMORY_GetServoId()) && (receivedByte != BROADCAST_ID)) && !MEMORY_CheckFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS)) {

			// Deactivate reception timeout
			TASK_ResetReceptionTimeout();

			// Reset reception buffer
			receptionBuffer[bufferWriteIndex].length = 0;

			// Reset flag for reception in progress
			MEMORY_ResetFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS);

			return;

		}

		// If packet length is not valid, refuse packet and reset reception buffer
		if ((receptionBuffer[bufferWriteIndex].length == 3) && ((receivedByte < 2) || (receivedByte > 250))) {

			// Deactivate reception timeout
			TASK_ResetReceptionTimeout();

			// Reset reception buffer
			receptionBuffer[bufferWriteIndex].length = 0;

			// Reset flag for reception in progress
			MEMORY_ResetFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS);

			return;

		}

		// Copy received byte to buffer and increase buffer length
		receptionBuffer[bufferWriteIndex].buffer[receptionBuffer[bufferWriteIndex].length++] = receivedByte;

		// If packet length is known, configure and activate DMA for reception and
		// deactivate synchronized read timeout
		if (receptionBuffer[bufferWriteIndex].length == 4) {

			// Configure and activate DMA for reception
			DMA_Cmd(DMA1_Channel5, DISABLE);
			DMA1_Channel5->CMAR = (uint32_t)&(receptionBuffer[bufferWriteIndex].buffer[4]);
			DMA1_Channel5->CNDTR = receptionBuffer[bufferWriteIndex].buffer[3];
			USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
			DMA_Cmd(DMA1_Channel5, ENABLE);

			// Disable USART interrupts
			USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
			USART_ITConfig(USART1, USART_IT_ERR, DISABLE);

			// If synchronized read is in progress, deactivate synchronized read timeout
			if (MEMORY_CheckFlag(FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS)) {

				// Deactivate synchronized read timeout
				TASK_ResetSynchronizedReadTimeout();

			}

		}

	}

	// USART transfer complete
	else if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) {

		if (!MEMORY_CheckFlag(FLAG_COMMUNICATION_TRANSMISSION_IN_PROGRESS)) {

			// Deactivate USART transmission complete interrupt
			USART_ITConfig(USART1, USART_IT_TC, DISABLE);

			// Clear interrupt flags and data register
			USART_GetITStatus(USART1, USART_IT_ORE);
			USART_GetITStatus(USART1, USART_IT_NE);
			USART_GetITStatus(USART1, USART_IT_FE);
			USART_GetITStatus(USART1, USART_IT_PE);
			USART_ReceiveData(USART1);

			// Enable USART interrupts
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

			// Reset transmission buffer
			transmissionBuffer.length = 2;

			// Change communication mode
			COMMUNICATION_SetCommunicationMode(RECEPTION_MODE);

		}

	}

	// USART transmission error
	else if (USART_GetITStatus(USART1, USART_IT_ERR) != RESET) {

		// Deactivate USART transmission complete interrupt
		USART_ITConfig(USART1, USART_IT_ERR, DISABLE);

		// Clear interrupt flags and data register
		USART_GetITStatus(USART1, USART_IT_ORE);
		USART_GetITStatus(USART1, USART_IT_NE);
		USART_GetITStatus(USART1, USART_IT_FE);
		USART_GetITStatus(USART1, USART_IT_PE);
		USART_ReceiveData(USART1);

		// Enable USART interrupts
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

		// Set transmission error flag
		MEMORY_SetError(ERROR_COMMUNICATION);

	}

}

// DMA interrupt handler
void DMA1_Channel4_IRQHandler(void) {

	// DMA transfer complete
	if (DMA_GetITStatus(DMA1_IT_TC4)) {

		// Clear interrupt flag
		DMA_ClearITPendingBit(DMA1_IT_TC4);

		// Deactivate DMA
		DMA_Cmd(DMA1_Channel4, DISABLE);

		// Deactivate USART DMA mode
		USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);

		// Reset flag for transmission in progress
		MEMORY_ResetFlag(FLAG_COMMUNICATION_TRANSMISSION_IN_PROGRESS);

		// Activate USART transmission complete interrupt
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);

	}

}

// DMA interrupt handler
void DMA1_Channel5_IRQHandler(void) {

	// DMA transfer complete
	if (DMA_GetITStatus(DMA1_IT_TC5)) {

		// Clear interrupt flag
		DMA_ClearITPendingBit(DMA1_IT_TC5);

		// Clear interrupt flags and data register
		USART_GetITStatus(USART1, USART_IT_ORE);
		USART_GetITStatus(USART1, USART_IT_NE);
		USART_GetITStatus(USART1, USART_IT_FE);
		USART_GetITStatus(USART1, USART_IT_PE);
		USART_ReceiveData(USART1);

		// Enable USART1 interrupts
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

		// Deactivate DMA
		DMA_Cmd(DMA1_Channel5, DISABLE);

		// Deactivate USART DMA mode
		USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);

		// Deactivate reception timeout
		TASK_ResetReceptionTimeout();

		// Update reception buffer length and switch ring buffer to next reception buffer
		receptionBuffer[bufferWriteIndex].length += receptionBuffer[bufferWriteIndex].buffer[3];
		COMMON_ADVANCE(bufferWriteIndex, RING_BUFFER_SIZE);
		receptionBuffer[bufferWriteIndex].length = 0;

		// Reset flag for reception in progress
		MEMORY_ResetFlag(FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS);

		// Update baudrate setting
		if (MEMORY_CheckFlag(FLAG_COMMUNICATION_BAUDRATE_UPDATE)) {

			COMMUNICATION_SetBaudrate();

		}

	}

}
