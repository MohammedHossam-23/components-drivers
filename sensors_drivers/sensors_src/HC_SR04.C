/*
 * HC_SR04.C
 *
 *  Created on: Feb 24, 2026
 *      Author: Mohammed Hossam
 */


//
// Created by ashkore
// Modified to use EXTI and a Free-Running Timer
//

#include "HC_SR04.h"

#define DISTANCE_LIMIT_CM 400.0f  // Maximum reliable distance for HC-SR04 is usually 400 cm (4 meters)

/**
 * @brief Helper function to create a precise microsecond delay using the hardware timer.
 * This avoids blocking the entire system (unlike HAL_Delay which is millisecond based).
 * @param htim Pointer to the timer handle (configured to tick every 1 us).
 * @param us   Number of microseconds to delay.
 */
static void delay_us(TIM_HandleTypeDef *htim, uint32_t us) {
    uint32_t start_val = __HAL_TIM_GET_COUNTER(htim);
    // Wait until the difference between current counter and start value reaches 'us'
    while ((__HAL_TIM_GET_COUNTER(htim) - start_val) < us);
}

void HC_SR04_init(HC_SR04_t *sr04_struct, TIM_HandleTypeDef *htim) {
    // Link the free-running timer to the sensor structure
    sr04_struct->htim = htim;

    // Ensure the trigger pin starts in a LOW state
    HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_RESET);

    // Initialize state variables
    sr04_struct->start_time_us = 0;
    sr04_struct->distance_cm = 0.0f;
    sr04_struct->last_distance_cm = 0.0f;
}

void HC_SR04_trigger(HC_SR04_t *sr04_struct) {
    // 1. Send a HIGH pulse to the trigger pin
    HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_SET);

    // 2. Wait for exactly 10 microseconds (hardware timer delay, no RTOS disruption)
    delay_us(sr04_struct->htim, 10);

    // 3. Set the trigger pin back to LOW
    HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_RESET);
}

/**
 * @brief EXTI processing function for the HC-SR04 Echo pin.
 * This function MUST be called inside the HAL_GPIO_EXTI_Callback function.
 * @param sr04_struct Pointer to the sensor structure.
 */
void HC_SR04_process_EXTI(HC_SR04_t *sr04_struct) {
    // 1. Immediately read the current timer value for maximum accuracy
    uint32_t current_time = __HAL_TIM_GET_COUNTER(sr04_struct->htim);

    // 2. Check the state of the Echo pin to determine if this is a Rising or Falling edge
    if (HAL_GPIO_ReadPin(sr04_struct->echo_port, sr04_struct->echo_pin) == GPIO_PIN_SET) {
        // --- Rising Edge Detected ---
        // The sensor just sent the ping; record the start time.
        sr04_struct->start_time_us = current_time;
    }
    else {
        // --- Falling Edge Detected ---
        // The echo pulse has returned. Calculate the pulse width.
        uint32_t diff_us;

        // Calculate elapsed time while handling potential 32-bit timer overflow
        if (current_time >= sr04_struct->start_time_us) {
            diff_us = current_time - sr04_struct->start_time_us;
        } else {
            diff_us = (0xFFFFFFFF - sr04_struct->start_time_us) + current_time;
        }

        // Calculate distance in cm (Pulse width in microseconds divided by 58)
        float current_distance = (float)diff_us / 58.0f;

        // Basic Noise Filter / Bounds Check
        if (current_distance > DISTANCE_LIMIT_CM || current_distance <= 0.0f) {
            // If the reading is out of bounds (glitch/noise), keep the last valid distance
            sr04_struct->distance_cm = sr04_struct->last_distance_cm;
        } else {
            // Valid reading: update current and last distance values
            sr04_struct->last_distance_cm = sr04_struct->distance_cm;
            sr04_struct->distance_cm = current_distance;
        }
    }
}
