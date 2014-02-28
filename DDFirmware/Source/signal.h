 /*
 ******************************************************************************
 * File    		signal.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for signal functions.
 * Peripherals	GPIOB, TIM4
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __SIGNAL_H
#define __SIGNAL_H

// Type includes
#include "stm32f10x.h"

// Defines
#ifndef INLINE_SIGNAL
#define INLINE_SIGNAL extern inline
#endif
#define STARTUP_SIGNAL_DURATION 75

// Type defines
typedef enum {

	COLOR_AQUA,
	COLOR_BLACK,
	COLOR_BLUE,
	COLOR_FUCHSIA,
	COLOR_GREEN,
	COLOR_ORANGE,
	COLOR_RED,
	COLOR_WHITE,
	COLOR_YELLOW

} signalColor_t;
typedef enum {

	OPERATION_OFF,
	OPERATION_ENERGY,
	OPERATION_POWER,
	OPERATION_CONTROLLER_TEMPERATURE,
	OPERATION_MOTOR_TEMPERATURE,
	OPERATION_SPEED,
	OPERATION_TORQUE,
	OPERATION_CURRENT,
	OPERATION_VOLTAGE

} operationMode_t;
typedef enum {

	GRADIENT_BLACK_RED,
	GRADIENT_BLACK_GREEN,
	GRADIENT_BLACK_BLUE,
	GRADIENT_BLUE_RED,
	GRADIENT_GREEN_BLUE,
	GRADIENT_GREEN_RED,
	GRADIENT_WHITE_RED,
	GRADIENT_WHITE_GREEN,
	GRADIENT_WHITE_BLUE

} gradientMode_t;

// Global variables
extern uint16_t startupSignalCounter;
extern uint8_t colorTable[16][3];

// Global function prototypes
void SIGNAL_Initialize(void);
void SIGNAL_Activate(void);
void SIGNAL_Deactivate(void);
void SIGNAL_Error(void);
void SIGNAL_SetColor(signalColor_t);
void SIGNAL_SetRGBColor(uint8_t, uint8_t, uint8_t);
void SIGNAL_Update(void);

// Module includes
#include "common.h"
#include "memory.h"
#include "signalLookup.h"

// Activate signal
INLINE_SIGNAL void SIGNAL_Activate(void) {

	// Start TIM4
	TIM_Cmd(TIM4, ENABLE);

}

// Deactivate signal
INLINE_SIGNAL void SIGNAL_Deactivate(void) {

	// Stop TIM4
	TIM_Cmd(TIM4, DISABLE);

}

// Set signal to error signal
INLINE_SIGNAL void SIGNAL_Error(void) {

	SIGNAL_SetColor(COLOR_ORANGE);

}

// Set signal color according to the color table
INLINE_SIGNAL void SIGNAL_SetColor(signalColor_t color) {

	TIM_SetCompare1(TIM4, powerLaw[colorTable[color][0]]);
	TIM_SetCompare2(TIM4, powerLaw[colorTable[color][1]]);
	TIM_SetCompare3(TIM4, powerLaw[colorTable[color][2]]);

}

// Set signal color according to RGB color parameters
INLINE_SIGNAL void SIGNAL_SetRGBColor(uint8_t red, uint8_t green, uint8_t blue) {

	TIM_SetCompare1(TIM4, powerLaw[red]);
	TIM_SetCompare2(TIM4, powerLaw[green]);
	TIM_SetCompare3(TIM4, powerLaw[blue]);

}

#endif
