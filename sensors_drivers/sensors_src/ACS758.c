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

// Li-ion single cell discharge Look-Up Table (3.7V Nominal)
// Maps Cell Voltage to State of Charge (SoC) percentage
static const float SoC_LookupTable[10][2] = {
    {3.00f, 0.0f},   {3.30f, 5.0f},   {3.45f, 10.0f}, {3.60f, 20.0f},
    {3.70f, 40.0f},  {3.80f, 60.0f},  {3.90f, 70.0f}, {4.00f, 80.0f},
    {4.10f, 90.0f},  {4.20f, 100.0f}
};

/**
 * @brief Converts actual battery voltage to a State of Charge (SoC) percentage
 * using a Look-Up Table (LUT) with linear interpolation for Li-ion discharge curve.
 * @param battery_voltage Actual total battery voltage.
 * @return SoC percentage (0.0 to 100.0).
 */
static float Voltage_To_SoC(float battery_voltage) {
    // 1. Calculate single cell voltage so the LUT works seamlessly with any battery pack (3S, 4S, etc.)
    float cell_voltage = battery_voltage / BATTERY_CELL_COUNT;

    // 2. Cap values at 0% and 100% bounds
    if (cell_voltage <= SoC_LookupTable[0][0]) return 0.0f;
    if (cell_voltage >= SoC_LookupTable[9][0]) return 100.0f;

    // 3. Search the LUT and perform linear interpolation between the two closest points
    for (int i = 0; i < 9; i++) {
        if (cell_voltage >= SoC_LookupTable[i][0] && cell_voltage <= SoC_LookupTable[i+1][0]) {
            float v0 = SoC_LookupTable[i][0];
            float v1 = SoC_LookupTable[i+1][0];
            float s0 = SoC_LookupTable[i][1];
            float s1 = SoC_LookupTable[i+1][1];

            // Straight-line equation between the two found data points
            return s0 + (s1 - s0) * ((cell_voltage - v0) / (v1 - v0));
        }
    }
    return 0.0f; // Fallback in case of an unexpected error
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
void ACS758_Init(ACS758_Handle *sensor, ADC_HandleTypeDef *hadc, double capacity_ah) {
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
void ACS758_Update(ACS758_Handle *sensor, uint32_t raw_adc_current , double dt) {
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
//    uint32_t now = HAL_GetTick();
//    float dt = (now - sensor->last_tick) / 1000.0f; // Calculate time delta in seconds
//    sensor->last_tick = now;

    // Convert Amps * Seconds into Amp-Hours (divide by 3600s)
    double consumed_ah = (sensor->filtered_amps * dt) / 3600.0;

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





//============================================================
//					impelementaion
//============// ==========================================
// Inside main.c
// ==========================================
//
//// 1. Define a buffer array to receive DMA data
//// Assuming Rank 1 is the Current Sensor, and Rank 2 is the Voltage Sensor
//uint32_t adc_buffer[2];
//
//// Define the sensor handle structure
//ACS758_Handle myBattery;
//
//int main(void) {
//    // 1. Basic Initialization (HAL_Init, SystemClock_Config, MX_GPIO, MX_ADC1, ...)
//
//    // 2. Start the ADC in DMA mode once before starting FreeRTOS
//    // The DMA will continuously update adc_buffer[0] and adc_buffer[1] in the background
//    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);
//
//    // 3. Initialize the sensor variables
//    ACS758_Init(&myBattery, &hadc1, 10.0); // e.g., 10.0 Ah battery capacity
//
//    // 4. Calibrate the current sensor (assuming the robot is stationary and drawing zero motor current)
//    // We take the value from adc_buffer[0] which is currently being populated by the DMA
//    HAL_Delay(100); // Wait briefly for DMA readings to stabilize
//    ACS758_Calibrate(&myBattery, adc_buffer[0]);
//
//    // 5. Start the FreeRTOS scheduler
//    osKernelStart();
//
//    while (1) {
//        // Empty loop, execution is now handled by FreeRTOS tasks
//    }
//}
//
//
//void vSensingAndControlTask(void * pvParameters) {
//    TickType_t xLastWakeTime = xTaskGetTickCount();
//    TickType_t previous_time = xLastWakeTime; // Added this to prevent compilation error
//    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Target loop time: 10ms
//
//    for(;;) {
//        // 1. Block the task until exactly 10ms have passed
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//
//        // 2. Calculate the actual time delta (dt) in case FreeRTOS was delayed
//        TickType_t current_time = xTaskGetTickCount();
//        double actual_dt = (double)(current_time - previous_time) / 1000.0;
//        previous_time = current_time;
//
//        // 3. Pass the current reading directly from DMA (Index 0) and the precise dt
//        ACS758_Update(&myBattery, adc_buffer[0], actual_dt);
//
//        // 4. Update the OCV State of Charge.
//        // You can run this continuously since it is internally protected by the "near-zero current" condition.
//        ACS758_UpdateSoCWithVoltage(&myBattery, adc_buffer[1]);
//
//        // 5. MPU and Odometry updates go here...
//        // MPU6050_Read_And_Process(&hi2c1, &myMPUStruct, actual_dt);
//        // Calculate_Odometry(&myOdomStruct, actual_dt);
//    }
//}
