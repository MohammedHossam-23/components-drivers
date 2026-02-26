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

// ==========================================
// Hardware & ADC Configuration
// ==========================================
#define V_REF                   3.3f     // STM32 ADC Reference Voltage in Volts
#define ADC_RES                 4095.0f  // 12-bit ADC Resolution (2^12 - 1)

// ==========================================
// ACS758 Current Sensor Configuration
// ==========================================
#define ACS758_SENSITIVITY      0.040f   // Sensitivity for ACS758LCB-050B (40mV/A)
#define VOLTAGE_DIVIDER_RATIO   1.5f     // Ratio for 5V to 3.3V step-down (10k and 20k resistors) -> (10+20)/20 = 1.5
#define EMA_ALPHA               0.1f     // Exponential Moving Average filter coefficient (0.0 to 1.0)

// ==========================================
// Battery Monitoring Configuration (OCV)
// ==========================================
#define BATTERY_CELL_COUNT      3        // Number of Li-ion cells in series (e.g., 3 for 3S, 4 for 4S)
#define VOLTAGE_DIVIDER_VBAT    11.0f    // Ratio for Battery Voltage step-down (100k and 10k resistors) -> (100+10)/10 = 11.0

// Automatically derived voltage thresholds based on cell count
#define V_MAX_BATT              (BATTERY_CELL_COUNT * 4.2f)  // Max voltage (100% SoC)
#define V_MIN_BATT              (BATTERY_CELL_COUNT * 3.0f)  // Min voltage (0% SoC)

// ==========================================
// Sensor Handle Structure
// ==========================================
typedef struct {
    ADC_HandleTypeDef *hadc;
    float battery_capacity_ah;  // Total designed capacity of the battery in Ampere-hours (Ah)
    float remaining_charge_ah;  // Current remaining charge in Ah

    float zero_current_voltage; // Calibrated voltage output of the sensor at 0 Amps
    float current_amps;         // Instantaneous raw current reading
    float filtered_amps;        // Smoothed current reading after applying EMA filter

    uint32_t last_tick;         // Timestamp for calculating time delta (dt) in Coulomb Counting
    bool is_calibrated;         // Flag to ensure the sensor is calibrated before usage
} ACS758_Handle;

// ==========================================
// Function Prototypes
// ==========================================
void ACS758_Init(ACS758_Handle *sensor, ADC_HandleTypeDef *hadc, float capacity_ah);
void ACS758_Calibrate(ACS758_Handle *sensor, uint32_t raw_adc_avg);
void ACS758_Update(ACS758_Handle *sensor, uint32_t raw_adc_current);
void ACS758_UpdateSoCWithVoltage(ACS758_Handle *sensor, uint32_t raw_adc_voltage);
float ACS758_GetSoC(ACS758_Handle *sensor);

#endif /* SENSORS_DRIVERS_SENSORS_HEADER_ACS758_H_ */
