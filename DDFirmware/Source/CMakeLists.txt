cmake_minimum_required(VERSION 2.6)
project(Source)

set(BINARY_NAME ${PROJECT_NAME})

add_definitions(-DCONFIGURATION_MODEL_NUMBER=${CONFIGURATION_MODEL_NUMBER})

set(SRCS
  common.c
  common.h
  communication.c
  communication.h
  communicationProtocol.c
  communicationProtocol.h
  configuration.h
  control.c
  control.h
  eeprom0.c
  eeprom0.h
  eeprom1.c
  eeprom1.h
  fixpoint.h
  logging.c
  logging.h
  main.c
  measurement.c
  measurement.h
  measurementLookup.c
  measurementLookup.h
  memory.c
  memory.h
  motor.c
  motor.h
  observer.c
  observer.h
  signal.c
  signal.h
  signalLookup.c
  signalLookup.h
  task.c
  task.h
  trajectoryGeneration.c
  trajectoryGeneration.h
)

add_library(${BINARY_NAME} ${SRCS})

target_link_libraries(${BINARY_NAME})
