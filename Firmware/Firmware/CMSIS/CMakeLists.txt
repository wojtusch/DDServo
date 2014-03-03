cmake_minimum_required(VERSION 2.6)
project(CMSIS)

set(BINARY_NAME ${PROJECT_NAME})

set(SRCS
  CM3/CoreSupport/core_cm3.c
  CM3/CoreSupport/core_cm3.h
  CM3/DeviceSupport/ST/STM32F10x/stm32f10x.h
  CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
  CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.h
)

add_library(${BINARY_NAME} ${SRCS})

target_link_libraries(${BINARY_NAME})
