/*
 * MPU_6050.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Mohammed Hossam
 */

#ifndef STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_MPU_6050_H_
#define STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_MPU_6050_H_

/*
 * MPU6050.h
 *
 * Description: Driver for MPU6050 with Complementary Filter
 * Platform: STM32 HAL
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"  // To include HAL libraries and typedefs
#include <math.h>  // For atan2 and sqrt functions

// ============================================================================
// 1. I2C Address & Registers
// ============================================================================
// Address is 0x68. Shifted left by 1 for HAL Read/Write operations -> 0xD0
#define MPU6050_ADDR         0xD0

#define SMPLRT_DIV_REG       0x19
#define CONFIG_REG			 0x1A
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define ACCEL_XOUT_H_REG     0x3B
#define PWR_MGMT_1_REG       0x6B
#define WHO_AM_I_REG         0x75

// ============================================================================
// 2. Constants
// ============================================================================
#define RAD_TO_DEG           57.2957795131 // 180/3.14
#define ALPHA  				 0.96  // Filter coefficient (Trust 96% Gyro, 4% Accel)

// ============================================================================
// 3. Data Structure
// ============================================================================
typedef struct {
    // Raw Data (Directly from registers)
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    // Converted Data (Physical values)
    double Ax;  // Units: g
    double Ay;
    double Az;
    double Gx;  // Units: deg/s
    double Gy;
    double Gz;


    //calc gyro and Accel Roll (X) and Pitch (Y)

    double AccAngleRoll;
    double AccAnglePitch;

    // Final Filtered Angles
    double FinalAngleRoll;
    double FinalAnglePitch;
    double MPU_Yaw;
    //delta time
    double dt;

} MPU6050_t;

// ============================================================================
// 4. Function Prototypes
// ============================================================================

/**
 * @brief  Initialize the MPU6050 sensor.
 * @param  I2Cx Pointer to I2C handle.
 * @return 0 if initialization is successful, 1 if failed.
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

/**
 * @brief  Read raw data from sensor and convert to physical units.
 * @param  I2Cx Pointer to I2C handle.
 * @param  DataStruct Pointer to MPU6050 structure to store data.
 */
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,double dt);

/**
 * @brief  Calculate Pitch and Roll angles using a Complementary Filter.
 * @param  DataStruct Pointer to MPU6050 structure.
 * @param  dt Loop time in seconds (e.g., 0.01 for 10ms loop).
 */
void MPU6050_Process_Complementary(MPU6050_t *DataStruct, double dt);

#endif /* INC_MPU6050_H_ */

#endif /* STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_MPU_6050_H_ */
