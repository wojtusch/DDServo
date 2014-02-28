/*
 *
 ******************************************************************************
 * File    		eeprom0.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Eeprom emulation functions for system settings.
 * Peripherals
 ******************************************************************************
 */

// Includes
#include "eeprom0.h"
#include "common.h"
#include "memory.h"

// Global variables
uint16_t variableData0 = 0;
uint16_t virtualVariableAddresses0[EEPROM0_NUMBER_OF_VARIABLES] = {

		EEPROM0_MODEL_NUMBER,
		EEPROM0_SERVO_ID,
		EEPROM0_BAUDRATE_DIVIDER,
		EEPROM0_STATUS_RETURN_DELAY_TIME,
		EEPROM0_STATUS_RETURN_LEVEL,
		EEPROM0_POSITION_OFFSET,
		EEPROM0_POSITION_LIMIT_CLOCKWISE_LOW,
		EEPROM0_POSITION_LIMIT_CLOCKWISE_HIGH,
		EEPROM0_POSITION_LIMIT_COUNTERCLOCKWISE_LOW,
		EEPROM0_POSITION_LIMIT_COUNTERCLOCKWISE_HIGH,
		EEPROM0_CONTROLLER_TEMPERATURE_OFFSET,
		EEPROM0_CONTROLLER_TEMPERATURE_LIMIT,
		EEPROM0_MOTOR_TEMPERATURE_LIMIT,
		EEPROM0_VOLTAGE_LIMIT_LOW,
		EEPROM0_VOLTAGE_LIMIT_HIGH,
		EEPROM0_TORQUE_LIMIT_LOW,
		EEPROM0_TORQUE_LIMIT_HIGH,
		EEPROM0_CURRENT_LIMIT_LOW,
		EEPROM0_CURRENT_LIMIT_HIGH,
		EEPROM0_STATUS_SIGNAL,
		EEPROM0_ALARM_SIGNAL,
		EEPROM0_ALARM_SHUTDOWN,
        EEPROM0_SKIP_ZONE_PARAMETER_LOW,
        EEPROM0_SKIP_ZONE_PARAMETER_HIGH,
		EEPROM0_FEED_FORWARD_CONTROL_PARAMETER_LOW,
		EEPROM0_FEED_FORWARD_CONTROL_PARAMETER_HIGH,
		EEPROM0_PROPORTIONAL_POSITION_CONTROL_PARAMETER_LOW,
		EEPROM0_PROPORTIONAL_POSITION_CONTROL_PARAMETER_HIGH,
		EEPROM0_INTEGRAL_POSITION_CONTROL_PARAMETER_LOW,
		EEPROM0_INTEGRAL_POSITION_CONTROL_PARAMETER_HIGH,
		EEPROM0_DERIVATIVE_POSITION_CONTROL_PARAMETER_LOW,
		EEPROM0_DERIVATIVE_POSITION_CONTROL_PARAMETER_HIGH,
		EEPROM0_POSITION_COMPLIANCE_PARAMETER_LOW,
		EEPROM0_POSITION_COMPLIANCE_PARAMETER_HIGH,
		EEPROM0_PROPORTIONAL_SPEED_CONTROL_PARAMETER_LOW,
		EEPROM0_PROPORTIONAL_SPEED_CONTROL_PARAMETER_HIGH,
		EEPROM0_INTEGRAL_SPEED_CONTROL_PARAMETER_LOW,
		EEPROM0_INTEGRAL_SPEED_CONTROL_PARAMETER_HIGH,
		EEPROM0_DERIVATIVE_SPEED_CONTROL_PARAMETER_LOW,
		EEPROM0_DERIVATIVE_SPEED_CONTROL_PARAMETER_HIGH,
		EEPROM0_SPEED_COMPLIANCE_PARAMETER_LOW,
		EEPROM0_SPEED_COMPLIANCE_PARAMETER_HIGH

};

// Private function prototypes
static void EEPROM_Unlock(void);
static uint8_t EEPROM_WaitForLastOperation(uint32_t);
static uint8_t EEPROM_ErasePageAndWait(uint32_t);
static uint8_t EEPROM_ProgramHalfWordAndWait(uint32_t, uint16_t);
static uint8_t EEPROM_FormatAndWait(void);
static uint16_t EEPROM_FindValidPage(uint8_t);
static uint16_t EEPROM_VerifyPageFullWriteVariableAndWait(uint16_t, uint16_t);
static uint16_t EEPROM_PageTransferAndWait(uint16_t, uint16_t);

// Restore the pages to a known state and wait for completion
void EEPROM0_Initalize(void) {

	// Unlock the flash program erase controller for eeprom emulation
	EEPROM_Unlock();

	uint16_t pageStatus0 = 6;
	uint16_t pageStatus1 = 6;
	uint16_t variableIndex = 0;
	uint16_t status = 0;
	int16_t index = -1;
	uint16_t eepromStatus;

	// Get page0 status
	pageStatus0 = (*(volatile uint16_t*)EEPROM0_PAGE0_BASE_ADDRESS);
	// Get page1 status
	pageStatus1 = (*(volatile uint16_t*)EEPROM0_PAGE1_BASE_ADDRESS);
	// Check for invalid header states and repair if necessary
	switch (pageStatus0) {

	case EEPROM_ERASED:

		if (pageStatus1 == EEPROM_VALID_PAGE) {
			// Erase page0
			eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE0_BASE_ADDRESS);
			// If erase operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		} else if (pageStatus1 == EEPROM_RECEIVE_DATA) {

			// Erase page0
			eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE0_BASE_ADDRESS);
			// If erase operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}
			// Mark page1 as valid
			eepromStatus = EEPROM_ProgramHalfWordAndWait(EEPROM0_PAGE1_BASE_ADDRESS, EEPROM_VALID_PAGE);
			// If program operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;
			}

		} else {

			// Erase both Page0 and Page1 and set Page0 as valid page
			eepromStatus = EEPROM_FormatAndWait();
			// If erase/program operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		}
		break;

	case EEPROM_RECEIVE_DATA:

		if (pageStatus1 == EEPROM_VALID_PAGE) {

			// Transfer data from page1 to page0
			for (variableIndex = 0; variableIndex < EEPROM0_NUMBER_OF_VARIABLES; variableIndex++) {

				if (( *(volatile uint16_t*)(EEPROM0_PAGE0_BASE_ADDRESS + 6)) == virtualVariableAddresses0[variableIndex]) {

					index = variableIndex;

				}
				if (variableIndex != index) {

					// Read the last variable updates
					variableData0 = EEPROM0_GetVariable(virtualVariableAddresses0[variableIndex]);
					if (!MEMORY_CheckError(ERROR_EEPROM)) {

						// Transfer the variable to the page0
						status = EEPROM_VerifyPageFullWriteVariableAndWait(virtualVariableAddresses0[variableIndex], variableData0);
						// If program operation failed, eeprom error flag is set
						if (status != EEPROM_COMPLETE) {

							// Set eeprom error flag
							MEMORY_SetError(ERROR_EEPROM);
							return;

						}

					}

				}

			}
			// Mark page0 as valid
			eepromStatus = EEPROM_ProgramHalfWordAndWait(EEPROM0_PAGE0_BASE_ADDRESS, EEPROM_VALID_PAGE);
			// If program operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}
			// Erase page1
			eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE1_BASE_ADDRESS);
			// If erase operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		} else if (pageStatus1 == EEPROM_ERASED) {

			// Erase page1
			eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE1_BASE_ADDRESS);
			// If erase operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}
			// Mark page0 as valid
			eepromStatus = EEPROM_ProgramHalfWordAndWait(EEPROM0_PAGE0_BASE_ADDRESS, EEPROM_VALID_PAGE);
			// If program operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		} else {

			// Erase both page0 and page1 and set page0 as valid page
			eepromStatus = EEPROM_FormatAndWait();
			// If erase/program operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		}
		break;

	case EEPROM_VALID_PAGE:

		if (pageStatus1 == EEPROM_VALID_PAGE) {

			// Erase both page0 and page1 and set page0 as valid page
			eepromStatus = EEPROM_FormatAndWait();
			// If erase/program operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		} else if (pageStatus1 == EEPROM_ERASED) {

			// Erase page1
			eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE1_BASE_ADDRESS);
			// If erase operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		} else {

			// Transfer data from page0 to page1
			for (variableIndex = 0; variableIndex < EEPROM0_NUMBER_OF_VARIABLES; variableIndex++) {

				if ((*(volatile uint16_t*)(EEPROM0_PAGE1_BASE_ADDRESS + 6)) == virtualVariableAddresses0[variableIndex]) {

					index = variableIndex;

				}
				if (variableIndex != index) {

					// Read the last variable updates
					variableData0 = EEPROM0_GetVariable(virtualVariableAddresses0[variableIndex]);
					if (!MEMORY_CheckError(ERROR_EEPROM)) {

						// Transfer the variable to the page1
						status = EEPROM_VerifyPageFullWriteVariableAndWait(virtualVariableAddresses0[variableIndex], variableData0);
						// If program operation failed, eeprom error flag is set
						if (status != EEPROM_COMPLETE) {

							// Set eeprom error flag
							MEMORY_SetError(ERROR_EEPROM);
							return;

						}

					}

				}

			}
			// Mark page1 as valid
			eepromStatus = EEPROM_ProgramHalfWordAndWait(EEPROM0_PAGE1_BASE_ADDRESS, EEPROM_VALID_PAGE);
			// If program operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

			// Erase page0
			eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE0_BASE_ADDRESS);
			// If erase operation failed, eeprom error flag is set
			if (eepromStatus != EEPROM_COMPLETE) {

				// Set eeprom error flag
				MEMORY_SetError(ERROR_EEPROM);
				return;

			}

		}
		break;

	default:

		// Erase both page0 and page1 and set page0 as valid page
		eepromStatus = EEPROM_FormatAndWait();
		// If erase/program operation failed, eeprom error flag is set
		if (eepromStatus != EEPROM_COMPLETE) {

			// Set eeprom error flag
			MEMORY_SetError(ERROR_EEPROM);
			return;

		}
		break;

	}

}

// Check current eeprom status
uint8_t EEPROM0_GetStatus(void) {

	uint8_t status = EEPROM_COMPLETE;

	if((FLASH->SR & EEPROM_FLAG_BUSY) == EEPROM_FLAG_BUSY) {

		status = EEPROM_BUSY;

	} else {

		if((FLASH->SR & EEPROM_FLAG_PROGRAM_ERROR) != 0) {

			status = EEPROM_PROGRAM_ERROR;

		} else {

			if((FLASH->SR & EEPROM_FLAG_WRITE_PROTECTED_ERROR) != 0 ) {

				status = EEPROM_WRITE_PROTECTED_ERROR;

			} else {

				status = EEPROM_COMPLETE;

			}

		}

	}
	return status;

}

// Get last stored variable data
uint16_t EEPROM0_GetVariable(uint16_t virtualAddress) {

	uint16_t validPage = EEPROM0_PAGE0;
	uint16_t addressValue = 0;
	uint16_t readStatus = 1;
	uint32_t address = 0;
	uint32_t pageStartAddress = 0;
	uint16_t data;

	// Get active page for read operation
	validPage = EEPROM_FindValidPage(EEPROM_READ_FROM_VALID_PAGE);

	// Check if there is no valid page
	if (validPage == EEPROM_NO_VALID_PAGE) {

		// Set eeprom error flag
		MEMORY_SetError(ERROR_EEPROM);
		return EEPROM_ERROR;

	}

	// Get the valid page start address
	pageStartAddress = (uint32_t)(EEPROM0_START_ADDRESS + (uint32_t)(validPage * EEPROM0_PAGE_SIZE));

	// Get the valid page end address
	address = (uint32_t)((EEPROM0_START_ADDRESS - 2) + (uint32_t)((1 + validPage) * EEPROM0_PAGE_SIZE));

	// Check each active page address starting from end
	while (address > (pageStartAddress + 2)) {

		// Get the current location content to be compared with virtual address
		addressValue = (*(volatile uint16_t*)address);

		// Compare the read address with the virtual address
		if (addressValue == virtualAddress) {

			// Get content of address - 2 which is variable value
			data = (*(volatile uint16_t*)(address - 2));

			// In case variable value is read, reset read status flag
			readStatus = 0;

			break;

		} else {

			// Next address location
			address = address - 4;

		}

	}

	// If variable does not exist, return eeprom error
	if (readStatus == 1) {

		return EEPROM_ERROR;

	}
	else {

		return data;

	}

}

// Set and updates variable data and wait for completion
void EEPROM0_SetVariable(uint16_t virtualAddress, uint16_t data) {

	uint16_t status = 0;

	// Write the variable virtual address and value in the eeprom
	status = EEPROM_VerifyPageFullWriteVariableAndWait(virtualAddress, data);

	// In case the eeprom active page is full
	if (status == EEPROM_PAGE_FULL) {

		// Perform page transfer
		status = EEPROM_PageTransferAndWait(virtualAddress, data);

	}

}

// Unlock the flash program erase controller for eeprom emulation
static void EEPROM_Unlock(void) {

	// Authorize the fpec bank1 access
	FLASH->KEYR = EEPROM_KEY1;
	FLASH->KEYR = EEPROM_KEY2;

}

// Wait for last operation to be completed
static uint8_t EEPROM_WaitForLastOperation(uint32_t timeout) {

	uint8_t status = EEPROM_COMPLETE;
	// Check for the eeprom status
	status = EEPROM0_GetStatus();
	// Wait for an operation to be completed or a timeout to occur
	while ((status == EEPROM_BUSY) && (timeout != 0x00)) {

		status = EEPROM0_GetStatus();
		timeout--;

	}
	if (timeout == 0x00 ) {

		status = EEPROM_TIMEOUT;

	}
	return status;

}

// Erase a page at the specified address and wait for completion
static uint8_t EEPROM_ErasePageAndWait(uint32_t address) {

	uint8_t status = EEPROM_COMPLETE;
	// Wait for last operation to be completed
	status = EEPROM_WaitForLastOperation(EEPROM_ERASE_TIMEOUT);
	if (status == EEPROM_COMPLETE) {

		// If the previous operation is completed, erase the page
	    FLASH->CR|= EEPROM_CR_PER_Set;
	    FLASH->AR = address;
	    FLASH->CR |= EEPROM_CR_STRT_Set;

	    // Wait for last operation to be completed
	    status = EEPROM_WaitForLastOperation(EEPROM_ERASE_TIMEOUT);
	    if (status != EEPROM_TIMEOUT) {

	    	// If the erase operation is completed, disable the PER bit
	    	FLASH->CR &= EEPROM_CR_PER_Reset;

	    }

	}
	return status;

}

// Program a half word at the specified address and wait for completion
static uint8_t EEPROM_ProgramHalfWordAndWait(uint32_t address, uint16_t data) {

	uint8_t status = EEPROM_COMPLETE;
	// Wait for last operation to be completed
	status = EEPROM_WaitForLastOperation(EEPROM_PROGRAM_TIMEOUT);
	if (status == EEPROM_COMPLETE) {

	    // If the previous operation is completed, program new data
	    FLASH->CR |= EEPROM_CR_PG_Set;
	    *(volatile uint16_t*)address = data;
	    // Wait for last operation to be completed
	    status = EEPROM_WaitForLastOperation(EEPROM_PROGRAM_TIMEOUT);
	    if (status != EEPROM_TIMEOUT) {

	    	// If the program operation is completed, disable the PG bit
	    	FLASH->CR &= EEPROM_CR_PG_Reset;

	    }

	}
	return status;

}

// Erase pages and mark first page as valid and wait for completion
static uint8_t EEPROM_FormatAndWait(void) {

	uint8_t eepromStatus = EEPROM_COMPLETE;

	// Erase Page0
	eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE0_BASE_ADDRESS);

	// If erase operation failed, error code is returned
	if (eepromStatus != EEPROM_COMPLETE) {

		return eepromStatus;

	}

	// Set page0 as valid page
	eepromStatus = EEPROM_ProgramHalfWordAndWait(EEPROM0_PAGE0_BASE_ADDRESS, EEPROM_VALID_PAGE);

	// If program operation failed, error code is returned
	if (eepromStatus != EEPROM_COMPLETE) {

		return eepromStatus;

	}

	// Erase page1
	eepromStatus = EEPROM_ErasePageAndWait(EEPROM0_PAGE1_BASE_ADDRESS);

	// Return page1 erase operation status
	return eepromStatus;

}

// Find valid page for write or read operation
static uint16_t EEPROM_FindValidPage(uint8_t operation) {

	uint16_t pageStatus0 = 6;
	uint16_t pageStatus1 = 6;

	// Get page0 actual status
	pageStatus0 = (*(volatile uint16_t*)EEPROM0_PAGE0_BASE_ADDRESS);

	// Get page1 actual status
	pageStatus1 = (*(volatile uint16_t*)EEPROM0_PAGE1_BASE_ADDRESS);

	// Write or read operation
	switch (operation) {

	case EEPROM_WRITE_IN_VALID_PAGE:

		if (pageStatus1 == EEPROM_VALID_PAGE) {

			// Page0 receiving data
			if (pageStatus0 == EEPROM_RECEIVE_DATA) {

				return EEPROM0_PAGE0;

			} else {

				return EEPROM0_PAGE1;

			}

		} else if (pageStatus0 == EEPROM_VALID_PAGE) {

			// Page1 receiving data
			if (pageStatus1 == EEPROM_RECEIVE_DATA) {

				return EEPROM0_PAGE1;

			} else {

				return EEPROM0_PAGE0;

			}

		} else {

			return EEPROM_NO_VALID_PAGE;

		}

	case EEPROM_READ_FROM_VALID_PAGE:

		if (pageStatus0 == EEPROM_VALID_PAGE) {

			return EEPROM0_PAGE0;

		} else if (pageStatus1 == EEPROM_VALID_PAGE) {

			return EEPROM0_PAGE1;

		} else {

			return EEPROM_NO_VALID_PAGE;

		}

	default:

		return EEPROM0_PAGE0;

	}

}

// Verify if active page is full and write variable and wait for completion
static uint16_t EEPROM_VerifyPageFullWriteVariableAndWait(uint16_t virtualAddress, uint16_t data) {

	uint8_t eepromStatus = EEPROM_COMPLETE;
	uint16_t validPage = EEPROM0_PAGE0;
	uint32_t address = 0;
	uint32_t pageEndAddress = 0;

	// Get valid page for write operation
	validPage = EEPROM_FindValidPage(EEPROM_WRITE_IN_VALID_PAGE);

	// Check if there is no valid page
	if (validPage == EEPROM_NO_VALID_PAGE) {

		return EEPROM_NO_VALID_PAGE;

	}

	// Get the valid page start address
	address = (uint32_t)(EEPROM0_START_ADDRESS + (uint32_t)(validPage * EEPROM0_PAGE_SIZE));

	// Get the valid page end address
	pageEndAddress = (uint32_t)((EEPROM0_START_ADDRESS - 2) + (uint32_t)((1 + validPage) * EEPROM0_PAGE_SIZE));

	// Check each active page address starting from beginning
	while (address < pageEndAddress) {

		// Verify if address and address + 2 contents are 0xFFFFFFFF
		if ((*(volatile uint32_t*)address) == 0xFFFFFFFF) {

			// Set variable data
			eepromStatus = EEPROM_ProgramHalfWordAndWait(address, data);
			// If program operation failed, error code is returned
			if (eepromStatus != EEPROM_COMPLETE) {

				return eepromStatus;

			}
			// Set variable virtual address
			eepromStatus = EEPROM_ProgramHalfWordAndWait(address + 2, virtualAddress);
			// Return program operation status
			return eepromStatus;

		} else {

			// Next address location
			address = address + 4;

		}

	}

	// Return page full status in case the valid page is full
	return EEPROM_PAGE_FULL;

}

// Transfer last updated variables data from full page to empty page and wait for completion
static uint16_t EEPROM_PageTransferAndWait(uint16_t virtualAddress, uint16_t data) {

	uint8_t eepromStatus = EEPROM_COMPLETE;
	uint32_t newPageAddress = 0;
	uint32_t oldPageAddress = 0;
	uint16_t validPage = EEPROM0_PAGE0;
	uint16_t variableIndex = 0;
	uint16_t status = 0;

	// Get active page for read operation
	validPage = EEPROM_FindValidPage(EEPROM_READ_FROM_VALID_PAGE);

	if (validPage == EEPROM0_PAGE1) {

		// New page address where variable will be moved to
		newPageAddress = EEPROM0_PAGE0_BASE_ADDRESS;

		// Old page address where variable will be taken from
		oldPageAddress = EEPROM0_PAGE1_BASE_ADDRESS;

	} else if (validPage == EEPROM0_PAGE0) {

		// New page address where variable will be moved to
		newPageAddress = EEPROM0_PAGE1_BASE_ADDRESS;

		// Old page address where variable will be taken from
		oldPageAddress = EEPROM0_PAGE0_BASE_ADDRESS;

	} else {

		return EEPROM_NO_VALID_PAGE;

	}

	// Set the new page status to receive data status
	eepromStatus = EEPROM_ProgramHalfWordAndWait(newPageAddress, EEPROM_RECEIVE_DATA);
	// If program operation failed, error code is returned
	if (eepromStatus != EEPROM_COMPLETE) {

		return eepromStatus;

	}

	// Write the variable passed as parameter in the new active page
	status = EEPROM_VerifyPageFullWriteVariableAndWait(virtualAddress, data);
	// If program operation failed, error code is returned
	if (status != EEPROM_COMPLETE) {

		return status;

	}

	// Transfer process
	for (variableIndex = 0; variableIndex < EEPROM0_NUMBER_OF_VARIABLES; variableIndex++) {

		if (virtualVariableAddresses0[variableIndex] != virtualAddress) {

			// Read the other last variable updates
			variableData0 = EEPROM0_GetVariable(virtualVariableAddresses0[variableIndex]);

			if (!MEMORY_CheckError(ERROR_EEPROM)) {

				// Transfer the variable to the new active page
				status = EEPROM_VerifyPageFullWriteVariableAndWait(virtualVariableAddresses0[variableIndex], variableData0);
				// If program operation failed, error code is returned
				if (status != EEPROM_COMPLETE) {

					return status;

				}

			}

		}

	}

	// Erase the old page:
	eepromStatus = EEPROM_ErasePageAndWait(oldPageAddress);
	// If erase operation failed, error code is returned
	if (eepromStatus != EEPROM_COMPLETE) {

		return eepromStatus;

	}

	// Set new page status to valid page status
	eepromStatus = EEPROM_ProgramHalfWordAndWait(newPageAddress, EEPROM_VALID_PAGE);
	// If program operation failed, error code is returned
	if (eepromStatus != EEPROM_COMPLETE) {

		return eepromStatus;

	}
	return eepromStatus;

}
