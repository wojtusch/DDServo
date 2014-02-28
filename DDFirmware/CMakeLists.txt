project(DDFirmware)
cmake_minimum_required(VERSION 2.6)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include (CMake/CMakeSetCompilerFlags.cmake)
include (CMake/CMakePrintCompilerFlags.cmake)

if (CMAKE_SYSTEM_NAME MATCHES "ARM")

  include_directories(Source)
  include_directories(Firmware/CMSIS/CM3/CoreSupport)
  include_directories(Firmware/CMSIS/CM3/DeviceSupport/ST/STM32F10x)
  include_directories(Firmware/STM32F10xDriver/Include)

  add_subdirectory(Source Source)
  add_subdirectory(Firmware/CMSIS CMSIS)
  add_subdirectory(Firmware/STM32F10xDriver STM32F10xDriver)

  if (CONFIGURATION_MODEL_NUMBER MATCHES "128")

    set(BINARY_NAME "DD28Firmware")

  elseif (CONFIGURATION_MODEL_NUMBER MATCHES "164")

    set(BINARY_NAME "DD64Firmware")

  else ()

    message( FATAL_ERROR "CONFIGURATION_MODEL_NUMBER is not set. Please use -DCONFIGURATION_MODEL_NUMBER=###.")

  endif ()

  set(CMAKE_EXE_LINKER_FLAGS "-static -Wl,-cref,-u,Reset_Handler -T\"${CMAKE_SOURCE_DIR}/linker.ld\" -Wl,--gc-sections -Xlinker -Map=\"${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${BINARY_NAME}.map\"")

  add_custom_command (
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/startup.o
    COMMAND ${CMAKE_ASM_COMPILER} -mthumb -mcpu=cortex-m3 -o${CMAKE_CURRENT_BINARY_DIR}/startup.o ${CMAKE_CURRENT_SOURCE_DIR}/startup.S
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/startup.S
  )
  
  add_executable(${BINARY_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/startup.o)
  
  set_target_properties(${BINARY_NAME}.elf PROPERTIES LINKER_LANGUAGE C)
  
  target_link_libraries(${BINARY_NAME}.elf Source CMSIS STM32F10xDriver)

  add_custom_target(
    ${BINARY_NAME}.bin
    ALL
    COMMAND ${CMAKE_OBJCOPY} -R .eeprom -O binary ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${BINARY_NAME}.elf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${BINARY_NAME}.bin
    DEPENDS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${BINARY_NAME}.elf
  )

else ()

  message("Target not supported yet!")
  message("Please specify valid toolchain with -DCMAKE_TOOLCHAIN_FILE=filename option.")

endif ()
