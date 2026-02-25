/*
 * ACS758.c
 *
 *  Created on: Feb 25, 2026
 *      Author: Mohammed Hossam
 */


#include "ACS758.h"

/**
 * @brief Initializes the sensor handle and sets starting capacity.
 */
void ACS758_Init(ACS758_Handle *sensor, ADC_HandleTypeDef *hadc, float capacity_ah) {
    sensor->hadc = hadc;
    sensor->battery_capacity_ah = capacity_ah;
    sensor->remaining_charge_ah = capacity_ah; // Assume full battery at start
    sensor->filtered_amps = 0.0f;
    sensor->is_calibrated = false;
    sensor->last_tick = HAL_GetTick();
}

/**
 * @brief Calibrates the 0A offset (Quiescent Voltage).
 * Run this when NO current is flowing through the sensor.
 */
void ACS758_Calibrate(ACS758_Handle *sensor, uint32_t raw_adc_avg) {
    // Convert average raw ADC to voltage
    float v_pin = (raw_adc_avg / ADC_RES) * V_REF;
    // Scale up to sensor output voltage (undoing the voltage divider)
    sensor->zero_current_voltage = v_pin * VOLTAGE_DIVIDER_RATIO;
    sensor->is_calibrated = true;
}

/**
 * @brief Processes a new ADC reading. Designed to be called in the 100Hz Task.
 * @param raw_adc: The value directly from the DMA buffer.
 */
void ACS758_Update(ACS758_Handle *sensor, uint32_t raw_adc) {
    if (!sensor->is_calibrated) return;

    // 1. Convert ADC value to Voltage at the Sensor Output
    float v_pin = (raw_adc / ADC_RES) * V_REF;
    float v_sensor = v_pin * VOLTAGE_DIVIDER_RATIO;

    // 2. Calculate Current: I = (V_out - V_quiescent) / Sensitivity
    sensor->current_amps = (v_sensor - sensor->zero_current_voltage) / ACS758_SENSITIVITY;

    // 3. Apply Exponential Moving Average (EMA) Filter to reduce noise
    sensor->filtered_amps = (EMA_ALPHA * sensor->current_amps) +
                            ((1.0f - EMA_ALPHA) * sensor->filtered_amps);

    // 4. Coulomb Counting Integration (Ah += I * dt)
    uint32_t now = HAL_GetTick();
    float dt = (now - sensor->last_tick) / 1000.0f; // Time delta in seconds
    sensor->last_tick = now;

    // Convert Amps*Seconds to Amp-Hours: (A * s) / 3600
    float consumed_ah = (sensor->filtered_amps * dt) / 3600.0f;
    sensor->remaining_charge_ah -= consumed_ah;
}

/**
 * @brief Returns the State of Charge (SoC) as a percentage.
 */
float ACS758_GetSoC(ACS758_Handle *sensor) {
    float soc_percent = (sensor->remaining_charge_ah / sensor->battery_capacity_ah) * 100.0f;

    // Constraint within 0% - 100%
    if (soc_percent > 100.0f) soc_percent = 100.0f;
    if (soc_percent < 0.0f)   soc_percent = 0.0f;

    return soc_percent;
}

// Calculate Real Battery Voltage based on your divider
float Get_Battery_Voltage(uint32_t raw_adc) {
    float v_pin = (raw_adc / 4095.0f) * 3.3f;
    return v_pin * 11.0f; // Multiplier (R1+R2)/R2
}
