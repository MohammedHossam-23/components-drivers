/*
 * drive_motor.h
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */

#ifndef ACTUATOR_DRIVER__API_ACT_HEADER_DRIVE_MOTOR_H_
#define ACTUATOR_DRIVER__API_ACT_HEADER_DRIVE_MOTOR_H_

#include "stm32f4xx_hal.h"

// Motor Identifiers
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} DriveMotorId_t;

// Maximum PWM based ARR Value
#define DRIVE_MAX_PWM 5000

/**
 * @brief Initializes PWM timers and Direction GPIOs.
 */
void DriveMotor_Init(void);

/**
 * @brief Sets the motor speed and direction.
 * @param motor MOTOR_LEFT or MOTOR_RIGHT
 * @param pwm_value Range: -DRIVE_MAX_PWM to +DRIVE_MAX_PWM. 0 stops the motor.
 */
void DriveMotor_SetPWM(DriveMotorId_t motor, int32_t pwm_value);

/**
 * @brief Instantly cuts power to both motors (0 PWM). Used for cliff detection.
 */
void DriveMotor_EmergencyStop(void);

#endif /* ACTUATOR_DRIVER__API_ACT_HEADER_DRIVE_MOTOR_H_ */
