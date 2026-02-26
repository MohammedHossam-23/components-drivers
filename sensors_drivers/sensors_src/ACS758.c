/*
 * ACS758.c
 *
 * Created on: Feb 26, 2026
 * Author: Mohammed Hossam
 */

#include "ACS758.h"

// ==========================================
// Private Helper Functions
// ==========================================

/**
 * @brief Converts actual battery voltage to a State of Charge (SoC) percentage
 * using linear interpolation between V_MIN_BATT and V_MAX_BATT instead of look up table.
 * @param voltage Actual battery voltage.
 * @return SoC percentage (0.0 to 100.0).
 */
static float Voltage_To_SoC(float voltage) {
    // Cap values at 100% and 0%
    if (voltage >= V_MAX_BATT) return 100.0f;
    if (voltage <= V_MIN_BATT) return 0.0f;

    // Linear mapping equation: SoC = ((V_current - V_min) / (V_max - V_min)) * 100
    return ((voltage - V_MIN_BATT) / (V_MAX_BATT - V_MIN_BATT)) * 100.0f;
}

// ==========================================
// Public Functions
// ==========================================

/**
 * @brief Initializes the sensor handle and sets starting capacity.
 * @param sensor Pointer to the ACS758 handle structure.
 * @param hadc Pointer to the ADC handle used for DMA.
 * @param capacity_ah Total capacity of the battery in Ampere-hours.
 */
void ACS758_Init(ACS758_Handle *sensor, ADC_HandleTypeDef *hadc, float capacity_ah) {
    sensor->hadc = hadc;
    sensor->battery_capacity_ah = capacity_ah;
    sensor->remaining_charge_ah = capacity_ah; // Assume full battery at start (will be corrected by OCV later)
    sensor->filtered_amps = 0.0f;
    sensor->is_calibrated = false;
    sensor->last_tick = HAL_GetTick();
}

/**
 * @brief Calibrates the 0A offset (Quiescent Voltage).
 * IMPORTANT: Run this during startup when NO current is flowing to the motors.
 * @param sensor Pointer to the ACS758 handle structure.
 * @param raw_adc_avg Average of several raw ADC readings taken at 0 Amps.
 */
void ACS758_Calibrate(ACS758_Handle *sensor, uint32_t raw_adc_avg) {
    // 1. Convert average raw ADC to voltage seen at the STM32 pin
    float v_pin = (raw_adc_avg / ADC_RES) * V_REF;

    // 2. Scale back up to the actual sensor output voltage (undoing the voltage divider)
    sensor->zero_current_voltage = v_pin * VOLTAGE_DIVIDER_RATIO;

    sensor->is_calibrated = true;
}

/**
 * @brief Processes a new ADC current reading and integrates it (Coulomb Counting).
 * Designed to be called continuously in a fast task (e.g., 100Hz / 10ms).
 * @param sensor Pointer to the ACS758 handle structure.
 * @param raw_adc_current The raw ADC value directly from the DMA buffer (Rank 1).
 */
void ACS758_Update(ACS758_Handle *sensor, uint32_t raw_adc_current) {
    if (!sensor->is_calibrated) return;

    // 1. Convert ADC value to Voltage at the Sensor Output
    float v_pin = (raw_adc_current / ADC_RES) * V_REF;
    float v_sensor = v_pin * VOLTAGE_DIVIDER_RATIO;

    // 2. Calculate Instantaneous Current: I = (V_out - V_quiescent) / Sensitivity
    sensor->current_amps = (v_sensor - sensor->zero_current_voltage) / ACS758_SENSITIVITY;

    // 3. Apply Exponential Moving Average (EMA) Filter to reduce PWM/Motor noise
    sensor->filtered_amps = (EMA_ALPHA * sensor->current_amps) +
                            ((1.0f - EMA_ALPHA) * sensor->filtered_amps);

    // 4. Coulomb Counting Integration (Ah = Ah - (I * dt))
    uint32_t now = HAL_GetTick();
    float dt = (now - sensor->last_tick) / 1000.0f; // Calculate time delta in seconds
    sensor->last_tick = now;

    // Convert Amps * Seconds into Amp-Hours (divide by 3600s)
    float consumed_ah = (sensor->filtered_amps * dt) / 3600.0f;

    // Subtract consumed energy from remaining charge
    sensor->remaining_charge_ah -= consumed_ah;
}

/**
 * @brief Reads the battery voltage and corrects the SoC to fix Coulomb Counting drift.
 * Designed to be called in a slow task (e.g., 1Hz / 1000ms).
 * @param sensor Pointer to the ACS758 handle structure.
 * @param raw_adc_voltage The raw ADC value from the battery voltage divider (Rank 2).
 */
void ACS758_UpdateSoCWithVoltage(ACS758_Handle *sensor, uint32_t raw_adc_voltage) {
    // 1. Convert raw ADC reading to actual total Battery Voltage
    float v_pin = (raw_adc_voltage / ADC_RES) * V_REF;
    float battery_voltage = v_pin * VOLTAGE_DIVIDER_VBAT;

    // 2. Correct SoC only if the robot is resting (Current is near zero)
    //    We do this to avoid "Voltage Sag" errors when motors are drawing heavy current.
    if (sensor->filtered_amps < 0.1f && sensor->filtered_amps > -0.1f) {

        // Map voltage to percentage
        float soc_percentage_from_voltage = Voltage_To_SoC(battery_voltage);

        // Overwrite the integrated remaining charge with the reality of the OCV
        sensor->remaining_charge_ah = (soc_percentage_from_voltage / 100.0f) * sensor->battery_capacity_ah;
    }
}

/**
 * @brief Returns the State of Charge (SoC) as a safe, constrained percentage.
 * @param sensor Pointer to the ACS758 handle structure.
 * @return SoC percentage (0.0 to 100.0).
 */
float ACS758_GetSoC(ACS758_Handle *sensor) {
    // Calculate percentage based on current remaining capacity vs total capacity
    float soc_percent = (sensor->remaining_charge_ah / sensor->battery_capacity_ah) * 100.0f;

    // Constraint within 0% - 100% to avoid logic errors in other modules
    if (soc_percent > 100.0f) soc_percent = 100.0f;
    if (soc_percent < 0.0f)   soc_percent = 0.0f;

    return soc_percent;
}
