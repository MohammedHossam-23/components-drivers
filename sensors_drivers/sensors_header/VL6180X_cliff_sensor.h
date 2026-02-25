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


/* Essential registers only for maximum speed cliff detection */
#define VL6180X_REG_SYSTEM_INTERRUPT_CLEAR                  0x0015
#define VL6180X_REG_SYSRANGE_START                          0x0018
#define VL6180X_REG_SYSRANGE_MAX_CONVERGENCE_TIME           0x001C
#define VL6180X_REG_SYSRANGE_INTERMEASUREMENT_PERIOD        0x001B
#define VL6180X_REG_RESULT_RANGE_STATUS                     0x004D
#define VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO            0x004F
#define VL6180X_REG_RESULT_RANGE_VAL                        0x0062
#define VL6180X_REG_READOUT_AVERAGING_SAMPLE_PERIOD         0x010A
#define VL6180X_REG_I2C_SLAVE_DEVICE_ADDRESS                0x0212

/* Cliff Settings */
#define CLIFF_THRESHOLD_MM 60  // Distance in millimeters to be considered a cliff

#define bool	_Bool
#define true	1
#define false	0

typedef struct{
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_address;
	bool last_cliff_state ;
}VL6180X_t;



/* Function Prototypes */

void VL6180X_ChangeAddress(VL6180X_t *cliff_sensor,uint8_t new_address);
void VL6180X_Init(VL6180X_t *cliff_sensor);
bool VL6180X_IsCliffDetected(VL6180X_t *cliff_sensor);



#endif /* SENSORS_DRIVERS_SENSORS_HEADER_VL6180X_CLIFF_SENSOR_H_ */
