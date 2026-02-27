/*
 * drive_motor.c
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */


#include "drive_motor.h"
#include <stdlib.h> // for abs()

// Extern your STM32 timers configured in CubeMX
extern TIM_HandleTypeDef htim1; //  Left & Right Motor Timer

// CubeMX Channels
#define LEFT_TIM_CHANNEL  TIM_CHANNEL_1
#define RIGHT_TIM_CHANNEL TIM_CHANNEL_2

// CubeMX Direction GPIOs (Adjust ports/pins to your wiring)
#define LEFT_DIR_PORT  GPIOA
#define LEFT_DIR_PIN   GPIO_PIN_8
#define RIGHT_DIR_PORT GPIOA
#define RIGHT_DIR_PIN  GPIO_PIN_9

void DriveMotor_Init(void) {
    HAL_TIM_PWM_Start(&htim1, LEFT_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim1, RIGHT_TIM_CHANNEL);
    DriveMotor_EmergencyStop();
}

void DriveMotor_SetPWM(DriveMotorId_t motor, int32_t pwm_value) {
    // 1. Clamp the PWM value to the absolute maximum
    if (pwm_value > DRIVE_MAX_PWM) pwm_value = DRIVE_MAX_PWM;
    if (pwm_value < -DRIVE_MAX_PWM) pwm_value = -DRIVE_MAX_PWM;

    // 2. Determine Direction
    GPIO_PinState dir_state = (pwm_value >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // 3. Get absolute value for the Timer Compare Register
    uint32_t abs_pwm = abs(pwm_value);

    // 4. Apply to Hardware
    if (motor == MOTOR_LEFT) {
        HAL_GPIO_WritePin(LEFT_DIR_PORT, LEFT_DIR_PIN, dir_state);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_TIM_CHANNEL, abs_pwm);
    } else {
        HAL_GPIO_WritePin(RIGHT_DIR_PORT, RIGHT_DIR_PIN, dir_state);
        __HAL_TIM_SET_COMPARE(&htim1, RIGHT_TIM_CHANNEL, abs_pwm);
    }
}

void DriveMotor_EmergencyStop(void) {
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_TIM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_TIM_CHANNEL, 0);
}
