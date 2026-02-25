/*
 * VL6180X_cliff_sensor.c
 *
 *  Created on: Feb 25, 2026
 *      Author: Mohammed Hossam
 */


#include "VL6180X_cliff_sensor.h"


static bool last_cliff_state = false;

// --- Custom I2C Wrapper Functions ---
static void WriteReg16(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t value) {
	HAL_I2C_Mem_Write(hi2c, (VL6180X_I2C_ADDR << 1), reg, I2C_MEMADD_SIZE_16BIT, &value, 1, 100);
}

static uint8_t ReadReg16(I2C_HandleTypeDef *hi2c, uint16_t reg) {
	uint8_t value = 0;
	HAL_I2C_Mem_Read(hi2c, (VL6180X_I2C_ADDR << 1), reg, I2C_MEMADD_SIZE_16BIT, &value, 1, 100);
	return value;
}
// -------------------------------------------------------------------------

void VL6180X_Init(I2C_HandleTypeDef *hi2c) {
	// 1. Wait for the device to be ready
	// Bit 0 of RESULT_RANGE_STATUS indicates if the device is ready
	while ((ReadReg16(hi2c, VL6180X_REG_RESULT_RANGE_STATUS) & 0x01) == 0);

	// 2. Configure for maximum speed
	// Reduce readout averaging sample period to decrease measurement time
	//each 1 bit = 64.5 mu s
	//16 * 64.5mu s = 1.03 ms
	WriteReg16(hi2c, VL6180X_REG_READOUT_AVERAGING_SAMPLE_PERIOD, 0x10);

	// Set max convergence time to 8ms for faster response.
	// This prevents the sensor from hanging too long if no target is found.
	WriteReg16(hi2c, VL6180X_REG_SYSRANGE_MAX_CONVERGENCE_TIME, 8);

	// 3. Clear any pending interrupts to start fresh
	WriteReg16(hi2c, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
	// ==========================================
	// (Continuous Mode)
	// ==========================================
	//sysrange__intermeasurement_period: Time delay between measurements in Ranging
	//continuous mode. Range 0-254 (0 = 10ms). Step size = 10ms.
	// (Period). 10 ms
	WriteReg16(hi2c, VL6180X_REG_SYSRANGE_INTERMEASUREMENT_PERIOD, 1);

	// 4. Start a continuous mode measurement
	//set both -> bit [0] : sysrange__startstop ,bit [1]: Ranging Mode Continuous
	WriteReg16(hi2c, VL6180X_REG_SYSRANGE_START, 0x03);

}

bool VL6180X_IsCliffDetected(I2C_HandleTypeDef *hi2c) {

	// 1. read if the measurement is ready
	// The lowest 3 bits [2:0] will be equal to 4 when a new sample is ready
	uint8_t status = ReadReg16(hi2c, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x07;
	if (status == 0x04) {
		// 2. Read error code and measured distance
		// The error code is located in the upper 4 bits [7:4] of the status register
		uint8_t error_code = ReadReg16(hi2c, VL6180X_REG_RESULT_RANGE_STATUS) >> 4;
		uint8_t distance = ReadReg16(hi2c, VL6180X_REG_RESULT_RANGE_VAL);

		// 3. Clear the interrupt to allow the sensor to prepare for the next measurement
		WriteReg16(hi2c, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

		// 4. Determine if a cliff is detected
		// A cliff is detected if the sensor returns an error (like "no target" due to infinite depth)
		// OR if the measured distance is strictly greater than our safety threshold
		if (error_code != 0 || distance > CLIFF_THRESHOLD_MM) {
			return last_cliff_state = true;  // DANGER! Cliff detected
		}
		else {
			return last_cliff_state = false; // SAFE, ground is detected within limits

		}

	}

	return last_cliff_state;
}
