/*
 * HC_SR04.h
 *
 *  Created on: Feb 24, 2026
 *      Author: Mohammed Hossam
 */

#ifndef STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_HC_SR04_H_
#define STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_HC_SR04_H_


#include "stm32f4xx_hal.h" // Note: Change to stm32f1xx_hal.h if using STM32F1

// HC-SR04 configuration and state structure
typedef struct {
    GPIO_TypeDef *trig_port;  // Trigger pin port
    uint16_t trig_pin;        // Trigger pin number
    GPIO_TypeDef *echo_port;  // Echo pin port (Must be configured as EXTI in CubeMX)
    uint16_t echo_pin;        // Echo pin number
    TIM_HandleTypeDef *htim;  // Pointer to the free-running timer handle (e.g., &htim5)

    volatile uint32_t start_time_us; // Timer counter value at the rising edge
    volatile float distance_cm;      // Calculated distance in centimeters
    float last_distance_cm;          // Last valid distance (used for basic noise filtering)
} HC_SR04_t;

// Function Prototypes
void HC_SR04_init(HC_SR04_t *sr04_struct, TIM_HandleTypeDef *htim);
void HC_SR04_trigger(HC_SR04_t *sr04_struct);
void HC_SR04_process_EXTI(HC_SR04_t *sr04_struct); // Must be called inside HAL_GPIO_EXTI_Callback
#endif /* STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_HC_SR04_H_ */
