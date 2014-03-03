/*
 ******************************************************************************
 * File    		fixpoint.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for fix-point calculations.
 * Peripherals
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __FIXPOINT_H
#define __FIXPOINT_H

// Defines
#define FXP16_Q 6                     	// Fix-point format: Q(16 - FIXPOINT16_Q).FIXPOINT16_Q
#define FXP32_Q 16                    	// Fix-point format: Q(32 - FIXPOINT32_Q).FIXPOINT32_Q
#define FXP64_Q 16                    	// Fix-point format: Q(64 - FIXPOINT64_Q).FIXPOINT64_Q
#define FXPTI_Q 16                    	// Fix-point format: Q(64 - FIXPOINT32_Q - FIXPOINTTI_Q).(FIXPOINT32_Q + FIXPOINTTI_Q)
#define ONE 65536                     	// Constant for pi in Q16.16
#define PI32 102944                   	// Constant for pi in Q16.16
#define TWO_PI32 411775               	// Constant for 2pi in Q16.16
#define HALF_POSITION_RANGE32 171573	// Constant for 150 degrees in Q16.16
#define FULL_POSITION_RANGE32 343146	// Constant for 300 degrees in Q16.16
#define FULL_SPEED_RANGE32 779304     	// Constant for 4280.847 degrees per second in Q16.16
#if (CONFIGURATION_MODEL_NUMBER == 128)
#define FULL_TORQUE_RANGE32 237795
#elif (CONFIGURATION_MODEL_NUMBER == 164)
#define FULL_TORQUE_RANGE32 334198
#endif
#define CHAR_MAX16 16320              	// Constant for 255 in Q10.6
#define CHAR_MAX32 16711680           	// Constant for 255 in Q16.16

// Type defines
typedef int16_t fxp16_t;
typedef int32_t fxp32_t;
typedef int64_t fxp64_t;
typedef int64_t fxpTI_t;

// Macros
#define FXP16_Init(a) ((fxp16_t)((a) * (1LL << FXP16_Q)))
#define FXP16_Int(a) ((a) >> FXP16_Q)
#define FXP16_Mul(a, b) ((fxp16_t)(((int32_t)(a) * (b)) >> FXP16_Q))
#define FXP16_Div(a, b) ((fxp16_t)(((int32_t)(a) << FXP16_Q) / (b)))
#define FXP32_Init(a) ((fxp32_t)((a) * (1LL << FXP32_Q)))
#define FXP32_Int(a) ((a) >> FXP32_Q)
#define FXP32_Mul(a, b) ((fxp32_t)(((fxp64_t)(a) * (b)) >> FXP32_Q))
#define FXP32_Div(a, b) ((fxp32_t)(((fxp64_t)(a) << FXP32_Q) / (b)))
#define FXP64_Mul(a, b) ((((fxp64_t)(a) * (b)) >> FXP64_Q))
#define FXP64_Div(a, b) ((((fxp64_t)(a) << FXP64_Q) / (b)))
#define FXPTI_Mul(a, b) (((a) * (b)) >> (FXP32_Q + FXPTI_Q))
#define FXPTI_Div(a, b) ((((fxp64_t)(a)) << (FXP32_Q + FXPTI_Q)) / (b))
#define FXP16to32(a) ((fxp32_t)(a) << (FXP32_Q - FXP16_Q))
#define FXP32to16(a) ((fxp16_t)((a) >> (FXP32_Q - FXP16_Q)))
#define FXP32toTI(a) ((fxpTI_t)((a) * (1LL << FXPTI_Q)))
#define FXP64toTI(a) ((a) * (1LL << FXPTI_Q))
#define FXPTIto32(a) ((fxp32_t)(a) >> FXPTI_Q)
#define FXPTIto64(a) ((a) >> FXPTI_Q)

#endif
