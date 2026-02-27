/*
 * mopping_driver.h
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */

#ifndef ACTUATOR_DRIVER__API_ACT_HEADER_MOPPING_DRIVER_H_
#define ACTUATOR_DRIVER__API_ACT_HEADER_MOPPING_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h" // Adjust to your STM32 family

// --- Mop Motor Speeds ---
typedef enum {
    MOP_SPEED_OFF = 0,
    MOP_SPEED_LOW,
    MOP_SPEED_HIGH
} MopSpeed_t;

// --- Pump States ---
typedef enum {
    PUMP_OFF = 0,
    PUMP_ON = 1
} PumpState_t;

/**
 * @brief Initializes the timer for Mop Motor PWM and the GPIO for the Pump Relay.
 */
void Mopping_Init(void);

/**
 * @brief Sets the spinning speed of the mop motors.
 * @param speed MOP_SPEED_OFF, MOP_SPEED_LOW, or MOP_SPEED_HIGH.
 */
void MopMotors_SetSpeed(MopSpeed_t speed);

/**
 * @brief Turns the water pump relay ON or OFF.
 * @param state PUMP_ON or PUMP_OFF.
 */
void WaterPump_SetState(PumpState_t state);

#endif /* ACTUATOR_DRIVER__API_ACT_HEADER_MOPPING_DRIVER_H_ */
