cmake_minimum_required(VERSION 2.6)
project(STM32F10xDriver)

set(BINARY_NAME ${PROJECT_NAME})

set(SRCS
  Source/misc.c
  Source/stm32f10x_adc.c
  Source/stm32f10x_bkp.c
  Source/stm32f10x_can.c
  Source/stm32f10x_cec.c
  Source/stm32f10x_crc.c
  Source/stm32f10x_dac.c
  Source/stm32f10x_dbgmcu.c
  Source/stm32f10x_dma.c
  Source/stm32f10x_exti.c
  Source/stm32f10x_flash.c
  Source/stm32f10x_fsmc.c
  Source/stm32f10x_gpio.c
  Source/stm32f10x_i2c.c
  Source/stm32f10x_iwdg.c
  Source/stm32f10x_pwr.c
  Source/stm32f10x_rcc.c
  Source/stm32f10x_rtc.c
  Source/stm32f10x_sdio.c
  Source/stm32f10x_spi.c
  Source/stm32f10x_tim.c
  Source/stm32f10x_usart.c
  Source/stm32f10x_wwdg.c
  Include/misc.h
  Include/stm32f10x_adc.h
  Include/stm32f10x_bkp.h
  Include/stm32f10x_can.h
  Include/stm32f10x_cec.h
  Include/stm32f10x_crc.h
  Include/stm32f10x_dac.h
  Include/stm32f10x_dbgmcu.h
  Include/stm32f10x_dma.h
  Include/stm32f10x_exti.h
  Include/stm32f10x_flash.h
  Include/stm32f10x_fsmc.h
  Include/stm32f10x_gpio.h
  Include/stm32f10x_i2c.h
  Include/stm32f10x_iwdg.h
  Include/stm32f10x_pwr.h
  Include/stm32f10x_rcc.h
  Include/stm32f10x_rtc.h
  Include/stm32f10x_sdio.h
  Include/stm32f10x_spi.h
  Include/stm32f10x_tim.h
  Include/stm32f10x_usart.h
  Include/stm32f10x_wwdg.h
)

add_library(${BINARY_NAME} ${SRCS})

target_link_libraries(${BINARY_NAME})
