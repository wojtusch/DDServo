 /*
 ******************************************************************************
 * File    		common.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for common functions.
 * Peripherals	GPIOB, IWDG, SysTick
 ******************************************************************************
 */

// Define to prevent recursive inclusion
#ifndef __COMMON_H
#define __COMMON_H

// Includes
#include "stm32f10x.h"

// Defines
#if (CONFIGURATION_MODEL_NUMBER  == 128)
#define DEFAULT_TORQUE_LIMIT 327680
#define DEFAULT_CURRENT_LIMIT 114688
#define DEFAULT_SKIP_ZONE_PARAMETER 0
#define DEFAULT_FEED_FORWARD_CONTROL_PARAMETER 0 //65536
#define DEFAULT_PROPORTIONAL_POSITION_CONTROL_PARAMETER 4587520 //5242880
#define DEFAULT_INTEGRAL_POSITION_CONTROL_PARAMETER 0 //6554
#define DEFAULT_DERIVATIVE_POSITION_CONTROL_PARAMETER 0
#define DEFAULT_POSITION_COMPLIANCE_PARAMETER 65536
#define DEFAULT_PROPORTIONAL_SPEED_CONTROL_PARAMETER 327680
#define DEFAULT_INTEGRAL_SPEED_CONTROL_PARAMETER 6554
#define DEFAULT_DERIVATIVE_SPEED_CONTROL_PARAMETER 0
#define DEFAULT_SPEED_COMPLIANCE_PARAMETER 65536
#define LIMIT_SPEED 655360
#elif (CONFIGURATION_MODEL_NUMBER  == 164)
#define DEFAULT_TORQUE_LIMIT 327680
#define DEFAULT_CURRENT_LIMIT 180224
#define DEFAULT_SKIP_ZONE_PARAMETER 0
#define DEFAULT_FEED_FORWARD_CONTROL_PARAMETER 0 //65536
#define DEFAULT_PROPORTIONAL_POSITION_CONTROL_PARAMETER 4587520 //5242880
#define DEFAULT_INTEGRAL_POSITION_CONTROL_PARAMETER 0 //6554
#define DEFAULT_DERIVATIVE_POSITION_CONTROL_PARAMETER 0
#define DEFAULT_POSITION_COMPLIANCE_PARAMETER 65536
#define DEFAULT_PROPORTIONAL_SPEED_CONTROL_PARAMETER 327680
#define DEFAULT_INTEGRAL_SPEED_CONTROL_PARAMETER 6554
#define DEFAULT_DERIVATIVE_SPEED_CONTROL_PARAMETER 0
#define DEFAULT_SPEED_COMPLIANCE_PARAMETER 65536
#define LIMIT_SPEED 524288
#endif
#define DEFAULT_MODEL_NUMBER CONFIGURATION_MODEL_NUMBER
#define DEFAULT_FIRMWARE_VERSION CONFIGURATION_FIRMWARE_VERSION
#define DEFAULT_FIRMWARE_CHECKSUM 0
#define DEFAULT_SERVO_ID 0
#define DEFAULT_STATUS_RETURN_LEVEL 1
#define DEFAULT_STATUS_RETURN_DELAY_TIME 1
#define DEFAULT_BAUDRATE_DIVIDER 72
#define DEFAULT_POSITION_OFFSET 0
#define DEFAULT_POSITION_LIMIT_CLOCKWISE 0
#define DEFAULT_POSITION_LIMIT_COUNTERCLOCKWISE 0
#define DEFAULT_VOLTAGE_LIMIT_LOW 736
#define DEFAULT_VOLTAGE_LIMIT_HIGH 1632
#define DEFAULT_MOTOR_TEMPERATURE_LIMIT 6080
#define DEFAULT_CONTROLLER_TEMPERATURE_LIMIT 5760
#define DEFAULT_CONTROLLER_TEMPERATURE_OFFSET 0
#define DEFAULT_STATUS_SIGNAL 0x0403
#define DEFAULT_ALARM_SIGNAL (1 << ERROR_CURRENT_LIMIT | 1 << ERROR_VOLTAGE_LIMIT | 1 << ERROR_MOTOR_TEMPERATURE_LIMIT | 1 << ERROR_CONTROLLER_TEMPERATURE_LIMIT)
#define DEFAULT_ALARM_SHUTDOWN (1 << ERROR_MOTOR_TEMPERATURE_LIMIT | 1 << ERROR_CONTROLLER_TEMPERATURE_LIMIT)
#define DEFAULT_DESIRED_POSITION 0
#define DEFAULT_DESIRED_SPEED 655360
#define DEFAULT_DESIRED_ACCELERATION 13107200
#define DEFAULT_DESIRED_PULSE_WIDTH 0
#define DEFAULT_ACTUAL_PULSE_WIDTH 0
#define DEFAULT_ACTUAL_POSITION_MEASUREMENT 0
#define DEFAULT_ACTUAL_POSITION_STATUS 0
#define DEFAULT_ACTUAL_SYSTEM_TIME 0
#define DEFAULT_ACTUAL_POSITION 0
#define DEFAULT_ACTUAL_SPEED 0
#define DEFAULT_ACTUAL_TORQUE 0
#define DEFAULT_ACTUAL_CURRENT 0
#define DEFAULT_ACTUAL_VOLTAGE 0
#define DEFAULT_FLAGS 0 // ToDo: Activate logging
#define DEFAULT_ERRORS 0
#define LIMIT_SERVO_ID 253
#define LIMIT_BAUDRATE_DIVIDER 16
#define LIMIT_BAUDRATE_LOW 0
#define LIMIT_BAUDRATE_HIGH 4500000
#define LIMIT_STATUS_RETURN_LEVEL 2
#define LIMIT_POSITION_OFFSET 4095
#define LIMIT_POSITION_LIMIT_CLOCKWISE_LOW 0
#define LIMIT_POSITION_LIMIT_CLOCKWISE_HIGH 2147483647
#define LIMIT_POSITION_LIMIT_COUNTERCLOCKWISE_LOW 0
#define LIMIT_POSITION_LIMIT_COUNTERCLOCKWISE_HIGH 2147483647
#define LIMIT_TORQUE_LIMIT_LOW 0
#define LIMIT_TORQUE_LIMIT_HIGH 2147483647
#define LIMIT_CURRENT_LIMIT_LOW 0
#define LIMIT_CURRENT_LIMIT_HIGH 2147483647
#define LIMIT_VOLTAGE_LIMIT_LOW 736
#define LIMIT_VOLTAGE_LIMIT_HIGH 1632
#define LIMIT_MOTOR_TEMPERATURE_LOW 1920
#define LIMIT_MOTOR_TEMPERATURE_HIGH 9600
#define LIMIT_CONTROLLER_TEMPERATURE_LOW 1920
#define LIMIT_CONTROLLER_TEMPERATURE_HIGH 9600
#define LIMIT_SKIP_ZONE_PARAMETER_LOW 0
#define LIMIT_SKIP_ZONE_PARAMETER_HIGH 65536
#define LIMIT_POSITION_COMPLIANCE_PARAMETER_LOW 0
#define LIMIT_POSITION_COMPLIANCE_PARAMETER_HIGH 65536
#define LIMIT_SPEED_COMPLIANCE_PARAMETER_LOW 0
#define LIMIT_SPEED_COMPLIANCE_PARAMETER_HIGH 65536
#define LIMIT_PULSE_WIDTH_LOW -MOTOR_DUTY_CYCLE_LIMIT
#define LIMIT_PULSE_WIDTH_HIGH MOTOR_DUTY_CYCLE_LIMIT
#define TOLERANCE_TORQUE 500
#define TOLERANCE_CURRENT 500
#define TOLERANCE_MOTOR_TEMPERATURE 500
#define TOLERANCE_CONTROLLER_TEMPERATURE 500
#define NORMAL_CONTROLLER_TEMPERATURE 3200
#define NORMAL_MOTOR_TEMPERATURE 3200
#define TIMER_STATUS_RETURN_DELAY 0
#define TIMER_RECEPTION_TIMEOUT 1
#define TIMER_SYNCHRONIZED_READ_TIMEOUT 2
#define COMMUNICATION_SKIP 0
#define COMMUNICATION_PROCESS 1
#define MODE_MOTOR_POWER_OFF 0
#define MODE_MOTOR_POWER_ON 1
#define MODE_LOGGING_OFF 0
#define MODE_LOGGING_ON 1
#define MODE_READ_DATA 0
#define MODE_WRITE_DATA_WITH_STATUS_PACKET 1
#define MODE_WRITE_DATA_WITHOUT_STATUS_PACKET 2
#define MODE_WRITE_PENDING_DATA_WITH_STATUS_PACKET 3
#define MODE_WRITE_PENDING_DATA_WITHOUT_STATUS_PACKET 4
#define MODE_RECEPTION_BUFFER 0
#define MODE_PENDING_DATA_BUFFER 1
#define MODE_POSITION_CONTROL_WITHOUT_TRAJECTORY 0
#define MODE_POSITION_CONTROL_WITH_TRAJECTORY 1
#define MODE_SPEED_CONTROL_WITHOUT_TRAJECTORY 2
#define MODE_SPEED_CONTROL_WITH_TRAJECTORY 3
#define MODE_TORQUE_CONTROL 4
#define MODE_PULSE_WIDTH_CONTROL 5
#define EEPROM_BUSY 1
#define EEPROM_PROGRAM_ERROR 2
#define EEPROM_WRITE_PROTECTED_ERROR 3
#define EEPROM_COMPLETE 4
#define EEPROM_TIMEOUT 5
#define EEPROM_NO_VALID_PAGE 0x00AB
#define EEPROM_ERASED 0xFFFF
#define EEPROM_RECEIVE_DATA 0xEEEE
#define EEPROM_VALID_PAGE 0x0000
#define EEPROM_READ_FROM_VALID_PAGE 0x00
#define EEPROM_WRITE_IN_VALID_PAGE 0x01
#define EEPROM_PAGE_FULL 0x80
#define EEPROM_ERROR 0xFFFF
#define EEPROM_KEY1 0x45670123
#define EEPROM_KEY2 0xCDEF89AB
#define EEPROM_CR_PG_Set 0x00000001
#define EEPROM_CR_PG_Reset 0x00001FFE
#define EEPROM_CR_PER_Set 0x00000002
#define EEPROM_CR_PER_Reset 0x00001FFD
#define EEPROM_CR_STRT_Set 0x00000040
#define EEPROM_ERASE_TIMEOUT 0x000B0000
#define EEPROM_PROGRAM_TIMEOUT 0x00002000
#define EEPROM_FLAG_BUSY 0x00000001
#define EEPROM_FLAG_PROGRAM_ERROR 0x00000004
#define EEPROM_FLAG_WRITE_PROTECTED_ERROR 0x00000010
#define FLASH_START_ADDRESS 0x08000000

// Macros
#define COMMON_ABSOLUTE(a) (((a) < 0) ? -(a) : (a))
#define COMMON_ADVANCE(a, max) (a = (a + 1) % (max))
#define COMMON_MAXIMUM(a, b) (((a) < (b)) ?  (b) : (a))

// Type defines
typedef enum {

	FLAG_MOTOR_POWER_ACTIVATED,
	FLAG_LOGGING_ACTIVATED,
	FLAG_CONTROL_MODE_0,
	FLAG_CONTROL_MODE_1,
	FLAG_CONTROL_MODE_2,
	FLAG_COMMUNICATION_RECEPTION_IN_PROGRESS,
	FLAG_COMMUNICATION_TRANSMISSION_IN_PROGRESS,
	FLAG_COMMUNICATION_BAUDRATE_UPDATE,
	FLAG_COMMUNICATION_PROTOCOL_SYNCHRONIZED_READ_IN_PROGRESS,
	FLAG_COMMUNICATION_COLOR_PING_REQUEST,
	FLAG_COMMUNICATION_PENDING_DATA_REGISTERED,
	FLAG_POSITION_SENSOR_DEACTIVATED,
	FLAG_POSITION_SENSOR_ACTIVATED,
	FLAG_TEMPERATURE_SENSOR_CALIBRATION,
	FLAG_ANALOG_CONVERTER_INITIALIZATION,
	FLAG_ANALOG_CONVERTER_ACTIVATED

} flag_t;
typedef enum {

	ERROR_POSITION_LIMIT,
	ERROR_TORQUE_LIMIT,
	ERROR_CURRENT_LIMIT,
	ERROR_VOLTAGE_LIMIT,
	ERROR_MOTOR_TEMPERATURE_LIMIT,
	ERROR_CONTROLLER_TEMPERATURE_LIMIT,
	ERROR_PARAMETER_RANGE,
	ERROR_CHECKSUM,
	ERROR_INSTRUCTION,
	ERROR_COMMUNICATION,
	ERROR_ANALOG_CONVERTER,
	ERROR_POSITION_SENSOR,
	ERROR_EEPROM,
	ERROR_TASK_MANAGEMENT,
	ERROR_DATA_LOSS

} error_t;
typedef enum {

	TASK_SYSTEM_TIME,
	TASK_OBSERVER,
	TASK_CONTROL,
	TASK_SIGNAL,
	TASK_LOGGING_UPDATE,
	TASK_LOGGING_SAVING

} task_t;

// Global function prototypes
void COMMON_Initialize(void);
void COMMON_Reboot(void);
void COMMON_Bootloader(void);
void COMMON_ActivateWatchdog(void);
void COMMON_ReloadWatchdog(void);
void COMMON_Delay(uint32_t);

#endif
