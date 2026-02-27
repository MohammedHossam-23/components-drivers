/*
 * vacuum_esc.h
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */

#ifndef ACTUATOR_DRIVER__API_ACT_HEADER_VACUUM_ESC_H_
#define ACTUATOR_DRIVER__API_ACT_HEADER_VACUUM_ESC_H_

#include "stm32f4xx_hal.h" // Change to your specific STM32 family (e.g., stm32g4xx_hal.h)

// --- Operating States ---
typedef enum {
    VACUUM_OFF = 0,
    VACUUM_ON  = 1
} VacuumState_t;

// --- API Functions ---
/**
 * @brief Initializes the timer for the ESC PWM and performs the ESC arming sequence.
 */
void Vacuum_ESC_Init(void);

/**
 * @brief Turns the vacuum BLDC motor ON (100%) or OFF (0%).
 * @param state VACUUM_ON or VACUUM_OFF.
 */
void Vacuum_ESC_SetState(VacuumState_t state);

/**
 * @brief Retrieves the current operating state.
 * @return VACUUM_ON or VACUUM_OFF.
 */
VacuumState_t Vacuum_ESC_GetState(void);

#endif /* ACTUATOR_DRIVER__API_ACT_HEADER_VACUUM_ESC_H_ */
