 /*
 ******************************************************************************
 * File    configuration.h
 * Author  wojtusch@sim.tu-darmstadt.de
 * Brief   Header for configuration of used peripherals.
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

// Defines
#define CONFIGURATION_FIRMWARE_VERSION 9

// Includes
#include "stm32f10x_adc.h"
// #include "stm32f10x_bkp.h"
// #include "stm32f10x_can.h"
// #include "stm32f10x_cec.h"
// #include "stm32f10x_crc.h"
// #include "stm32f10x_dac.h"
// #include "stm32f10x_dbgmcu.h"
#include "stm32f10x_dma.h"
// #include "stm32f10x_exti.h"
// #include "stm32f10x_flash.h"
// #include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
// #include "stm32f10x_i2c.h"
#include "stm32f10x_iwdg.h"
// #include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
// #include "stm32f10x_rtc.h"
// #include "stm32f10x_sdio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
// #include "stm32f10x_wwdg.h"
#include "misc.h"

// Macros
#define assert_param(expr) ((void)0)

#endif
