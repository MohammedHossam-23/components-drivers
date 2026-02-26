/*
 * ENC_DRIVE.h
 *
 *  Created on: Feb 17, 2026
 *      Author: Mohammed Hossam
 */

#ifndef STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ROBOT_DRIVER_H_
#define STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ROBOT_DRIVER_H_

#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include "MPU_6050.h"
#include <math.h>

// --- Configuration ---
#define WHEEL_DIA_M       0.08f// or 0.065f هنشوف هترسى على ايه
#define ENCODER_PPR       11
#define GEAR_RATIO        90
#define ROBOT_WIDTH_M     0.15f // (distance between the 2 motors)to be measured
#define TICKS_PER_REV     (ENCODER_PPR * 4 * GEAR_RATIO)
#define METERS_PER_TICK   ((M_PI * WHEEL_DIA_M) / TICKS_PER_REV)

// --- Structs ---

// Encoder struct: Only handles ticks and speed
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t current_counter_value;
    uint32_t last_counter_value;
    int64_t total_ticks;
    float velocity_m_s;
    float delta_distance_m;
    uint8_t is_32bit;
    int32_t last_delta;
} Encoder_t;

// Robot struct: Handles global position (Odometry)
typedef struct {
    Encoder_t left_enc;
    Encoder_t right_enc;

    // Global Pose (Belongs to the Robot, not the wheels)
    float x_m;
    float y_m;
    float theta_rad;
} Robot_Odometry_t;

void Robot_Init(Robot_Odometry_t *robot, TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right, MPU6050_t *MPUcfg);
void Robot_Update(Robot_Odometry_t *robot,I2C_HandleTypeDef *I2Cx ,MPU6050_t *MPUcfg ,float dt);
#endif /* STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ROBOT_DRIVER_H_ */
