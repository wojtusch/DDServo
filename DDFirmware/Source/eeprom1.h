/*
 ******************************************************************************
 * File    		eeprom1.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for eeprom emulation functions for logging.
 * Peripherals
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __EEPROM1_H
#define __EEPROM1_H

// Includes
#include "stm32f10x.h"

// Defines
#define EEPROM1_START_ADDRESS 0x0801F000
#define EEPROM1_NUMBER_OF_VARIABLES 6
#define EEPROM1_ENDURANCE_LOW 0x0000
#define EEPROM1_ENDURANCE_HIGH 0x0001
#define EEPROM1_PERFORMANCE_LOW 0x0002
#define EEPROM1_PERFORMANCE_HIGH 0x0003
#define EEPROM1_OVERCURRENT_TIME 0x0004
#define EEPROM1_OVERTEMPERATURE_TIME 0x0005
#define EEPROM1_PAGE_SIZE 0x400
#define EEPROM1_PAGE0_BASE_ADDRESS EEPROM1_START_ADDRESS
#define EEPROM1_PAGE0_END_ADDRESS (EEPROM1_START_ADDRESS + EEPROM1_PAGE_SIZE - 1)
#define EEPROM1_PAGE1_BASE_ADDRESS (EEPROM1_START_ADDRESS + EEPROM1_PAGE_SIZE)
#define EEPROM1_PAGE1_END_ADDRESS (EEPROM1_PAGE1_BASE_ADDRESS + EEPROM1_PAGE_SIZE - 1)
#define EEPROM1_PAGE0 0
#define EEPROM1_PAGE1 1

// Global function prototypes
void EEPROM1_Initalize(void);
uint8_t EEPROM1_GetStatus(void);
uint16_t EEPROM1_GetVariable(uint16_t);
void EEPROM1_SetVariable(uint16_t, uint16_t);

#endif
