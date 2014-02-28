/*
 ******************************************************************************
 * File    		signal.c
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Signal functions.
 * Peripherals	GPIOB, TIM4
 ******************************************************************************
 */

// Includes
#define INLINE_SIGNAL
#include "signal.h"

// Global variables
uint16_t startupSignalCounter;
uint8_t colorTable[16][3] = {

	{0, 230, 230},		// Aqua
	{0, 0, 0},			// Black
	{0, 0, 240},		// Blue
	{210, 0, 230},		// Fuchsia
	{0, 240, 0},		// Green
	{230, 220, 0},		// Orange
	{230, 0, 0},		// Red
	{240, 240, 240},	// White
	{230, 230, 0},		// Yellow
	{0, 0, 0},			// Empty
	{0, 0, 0},			// Empty
	{0, 0, 0},			// Empty
	{0, 0, 0},			// Empty
	{0, 0, 0},			// Empty
	{0, 0, 0},			// Empty
	{0, 0, 0}			// Empty

};

// Private function prototypes
static void SIGNAL_UpdateGradient(gradientMode_t, uint8_t);

// Initialize signal
void SIGNAL_Initialize(void) {

	// Private variables
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

    // Initialize variables
    startupSignalCounter = 0;

	// Initialize TIM4 for PWM with 1.1kHz on channel 1, 2 and 3
	SIGNAL_SetColor(COLOR_BLACK);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);

	// Initialize signal inputs and outputs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// LEDRED, LEDGREEN and LEDBLUE
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Set startup signal
    SIGNAL_SetColor(COLOR_WHITE);

}

// Update signal according to signal settings
void SIGNAL_Update(void) {

	uint8_t graduationValue = 0;
	int32_t actualValue = 0;
	int32_t limitValue = 0;

    if (startupSignalCounter <= STARTUP_SIGNAL_DURATION) {

        // Increment startup signal counter
        startupSignalCounter++;


    } else {

        switch(MEMORY_GetStatusSignalOperationMode()) {

        case OPERATION_OFF:

            // Turn signal off, if no error has occurred
            SIGNAL_SetColor(COLOR_BLACK);
            break;

        case OPERATION_ENERGY:

            // Turn signal on, if power is activated
            SIGNAL_SetColor((signalColor_t) MEMORY_GetStatusSignalColorMode());
            break;

        case OPERATION_POWER:

            // Turn signal on, if motor power is activated
            if (MEMORY_CheckFlag(FLAG_MOTOR_POWER_ACTIVATED)) {

                SIGNAL_SetColor((signalColor_t) MEMORY_GetStatusSignalColorMode());

            } else {

                SIGNAL_SetColor(COLOR_BLACK);

            }

            break;

        case OPERATION_CONTROLLER_TEMPERATURE:

            // Graduate signal according to controller temperature
            actualValue = MEMORY_GetActualControllerTemperature() - NORMAL_CONTROLLER_TEMPERATURE;
            limitValue = MEMORY_GetControllerTemperatureLimit() - NORMAL_CONTROLLER_TEMPERATURE;
            if (actualValue <= 0) {

                // Controller temperature is less than or equal to normal temperature
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 0);

            } else if (actualValue >= limitValue) {

                // Controller temperature is greater than or equal to temperature limit
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 255);

            } else {

                // Controller temperature lies in between
                graduationValue = (uint8_t)(((((int32_t)actualValue * CHAR_MAX16)) / limitValue) >> FXP16_Q);
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), graduationValue);

            }

            break;

        case OPERATION_MOTOR_TEMPERATURE:

            // Graduate signal according to motor temperature
            actualValue = MEMORY_GetActualMotorTemperature() - NORMAL_MOTOR_TEMPERATURE;
            limitValue = MEMORY_GetMotorTemperatureLimit() - NORMAL_MOTOR_TEMPERATURE;
            if (actualValue <= 0) {

                // Motor temperature is less than or equal to normal temperature
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 0);

            } else if (actualValue >= limitValue) {

                // Motor temperature is greater than or equal to temperature limit
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 255);

            } else {

                // Motor temperature lies in between
                graduationValue = (uint8_t)(((((int32_t)actualValue * CHAR_MAX16)) / limitValue) >> FXP16_Q);
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), graduationValue);

            }

            break;

        case OPERATION_SPEED:

            // Graduate signal according to speed
            actualValue = MEMORY_GetActualSpeed();
            actualValue = COMMON_ABSOLUTE(actualValue);
            limitValue = LIMIT_SPEED;
            if (actualValue >= limitValue) {

                // Speed is greater than or equal to normal speed
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 255);

            } else {

                // Speed is less than normal speed
                graduationValue = (uint8_t)(((((int64_t)actualValue * CHAR_MAX32)) / limitValue) >> FXP32_Q);
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), graduationValue);

            }

            break;

        case OPERATION_TORQUE:

            // Graduate signal according to torque
            actualValue = MEMORY_GetActualTorque();
            actualValue = COMMON_ABSOLUTE(actualValue);
            limitValue = MEMORY_GetTorqueLimit();
            if (actualValue >= limitValue) {

                // Torque is greater than or equal to torque limit
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 255);

            } else {

                // Torque is less than torque limit
                graduationValue = (uint8_t)(((((int64_t)actualValue * CHAR_MAX32)) / limitValue) >> FXP32_Q);
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), graduationValue);

            }

            break;

        case OPERATION_CURRENT:

            // Graduate signal according to current
            actualValue = MEMORY_GetActualCurrent();
            actualValue = COMMON_ABSOLUTE(actualValue);
            limitValue = MEMORY_GetCurrentLimit();
            if (actualValue >= limitValue) {

                // Current is greater than or equal to current limit
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 255);

            } else {

                // Current is less than current limit
                graduationValue = (uint8_t)(((((int64_t)actualValue * CHAR_MAX32)) / limitValue) >> FXP32_Q);
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), graduationValue);

            }

            break;

        case OPERATION_VOLTAGE:

            // Graduate signal according to voltage
            actualValue = MEMORY_GetActualVoltage() - MEMORY_GetVoltageLimitLow();
            limitValue = MEMORY_GetVoltageLimitHigh() - MEMORY_GetVoltageLimitLow();
            if (actualValue <= 0) {

                // Voltage is less than or equal to lower voltage limit
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 0);

            } else if (actualValue >= limitValue) {

                // Voltage is greater than or equal to higher voltage limit
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), 255);

            } else {

                // Voltage lies in between
                graduationValue = (uint8_t)(((((int32_t)actualValue * CHAR_MAX16)) / limitValue) >> FXP16_Q);
                SIGNAL_UpdateGradient(MEMORY_GetStatusSignalColorMode(), graduationValue);

            }

            break;

        default:

            // Turn signal off
            SIGNAL_SetColor(COLOR_BLACK);

        }

    }

}

// Update signal gradient
static void SIGNAL_UpdateGradient(gradientMode_t gradientMode, uint8_t graduationValue) {

	switch(gradientMode) {

	case GRADIENT_BLACK_RED:

		SIGNAL_SetRGBColor(graduationValue, 0, 0);
		break;

	case GRADIENT_BLACK_GREEN:

		SIGNAL_SetRGBColor(0, graduationValue, 0);
		break;

	case GRADIENT_BLACK_BLUE:

		SIGNAL_SetRGBColor(0, 0, graduationValue);
		break;

	case GRADIENT_BLUE_RED:

		SIGNAL_SetRGBColor(graduationValue, 0, (255 - graduationValue));
		break;

	case GRADIENT_GREEN_BLUE:

		SIGNAL_SetRGBColor(0, (255 - graduationValue), graduationValue);
		break;

	case GRADIENT_GREEN_RED:

		SIGNAL_SetRGBColor(graduationValue, (255 - graduationValue), 0);
		break;

	case GRADIENT_WHITE_RED:

		SIGNAL_SetRGBColor(255, (255 - graduationValue), (255 - graduationValue));
		break;

	case GRADIENT_WHITE_GREEN:

		SIGNAL_SetRGBColor((255 - graduationValue), 255, (255 - graduationValue));
		break;

	case GRADIENT_WHITE_BLUE:

		SIGNAL_SetRGBColor((255 - graduationValue), (255 - graduationValue), 255);
		break;

	default:

		SIGNAL_SetColor(COLOR_BLACK);

	}

}
