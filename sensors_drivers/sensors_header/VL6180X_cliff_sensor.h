/*
 * VL6180X_cliff_sensor.h
 *
 *  Created on: Feb 25, 2026
 *      Author: Mohammed Hossam
 */

#ifndef SENSORS_DRIVERS_SENSORS_HEADER_VL6180X_CLIFF_SENSOR_H_
#define SENSORS_DRIVERS_SENSORS_HEADER_VL6180X_CLIFF_SENSOR_H_



#include "stm32f4xx_hal.h" // Or include your custom I2C library here


/* I2C 7-bit Address */
#define VL6180X_I2C_ADDR 0x29
//#define VL6180X_2_I2C_ADDR 0xXX
//#define VL6180X_3_I2C_ADDR 0xXX

/* Essential registers only for maximum speed cliff detection */
#define VL6180X_REG_SYSTEM_INTERRUPT_CLEAR                  0x0015
#define VL6180X_REG_SYSRANGE_START                          0x0018
#define VL6180X_REG_SYSRANGE_MAX_CONVERGENCE_TIME           0x001C
#define VL6180X_REG_SYSRANGE_INTERMEASUREMENT_PERIOD        0x001B
#define VL6180X_REG_RESULT_RANGE_STATUS                     0x004D
#define VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO            0x004F
#define VL6180X_REG_RESULT_RANGE_VAL                        0x0062
#define VL6180X_REG_READOUT_AVERAGING_SAMPLE_PERIOD         0x010A


/* Cliff Settings */
#define CLIFF_THRESHOLD_MM 60  // Distance in millimeters to be considered a cliff


#define bool	_Bool
#define true	1
#define false	0


/* Function Prototypes */
void VL6180X_Init(I2C_HandleTypeDef *hi2c);
bool VL6180X_IsCliffDetected(I2C_HandleTypeDef *hi2c);



#endif /* SENSORS_DRIVERS_SENSORS_HEADER_VL6180X_CLIFF_SENSOR_H_ */
