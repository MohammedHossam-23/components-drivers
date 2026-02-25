/*
 * ACS758.h
 *
 *  Created on: Feb 25, 2026
 *      Author: Mohammed Hossam
 */

#ifndef SENSORS_DRIVERS_SENSORS_HEADER_ACS758_H_
#define SENSORS_DRIVERS_SENSORS_HEADER_ACS758_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/**
 * @brief ACS758LCB Configuration Constants
 * Based on ACS758LCB-050B (Bidirectional 50A version)
 */
#define ACS758_SENSITIVITY      0.040f   // 40mV/A = 0.040 V/A
#define V_REF                   3.3f     // STM32 ADC Reference Voltage
#define ADC_RES                 4095.0f  // 12-bit ADC Resolution
#define VOLTAGE_DIVIDER_RATIO   1.5f     // Ratio if using (10k/20k) to step down 5V to 3.3V
#define EMA_ALPHA               0.1f     // Filter coefficient (0.0 to 1.0). Lower = Smoother

typedef struct {
    ADC_HandleTypeDef *hadc;
    float zero_current_voltage; // Calibrated voltage at 0 Amps
    float current_amps;         // Instantaneous raw current
    float filtered_amps;        // Smoothed current after EMA filter
    float battery_capacity_ah;  // Total battery capacity in Ampere-hours
    float remaining_charge_ah;  // Current charge state in Ampere-hours
    uint32_t last_tick;         // Timestamp for SoC integration
    bool is_calibrated;
} ACS758_Handle;

/* Function Prototypes */
void ACS758_Init(ACS758_Handle *sensor, ADC_HandleTypeDef *hadc, float capacity_ah);
void ACS758_Calibrate(ACS758_Handle *sensor, uint32_t raw_adc_avg);
void ACS758_Update(ACS758_Handle *sensor, uint32_t raw_adc);
float ACS758_GetSoC(ACS758_Handle *sensor);
float Get_Battery_Voltage(uint32_t raw_adc);
#endif /* SENSORS_DRIVERS_SENSORS_HEADER_ACS758_H_ */
