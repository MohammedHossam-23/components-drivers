/*
 * mopping_driver.c
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */


#include "mopping_driver.h"

// --- Hardware Configuration ---
extern TIM_HandleTypeDef htim1; // Example: Timer for Mop Motors
#define MOP_LEFT_CHANNEL  TIM_CHANNEL_3
#define MOP_RIGHT_CHANNEL TIM_CHANNEL_4

// Maximum ARR for your 16-bit timer (matches CubeMX)
#define MOP_TIMER_ARR 4294967295

// Pump Relay GPIO (Matches CubeMX labels)
#define PUMP_RELAY_PORT GPIOA
#define PUMP_RELAY_PIN  GPIO_PIN_15

void Mopping_Init(void) {
    // Start PWM for both mop motors
    HAL_TIM_PWM_Start(&htim1, MOP_LEFT_CHANNEL);
    HAL_TIM_PWM_Start(&htim1, MOP_RIGHT_CHANNEL);

    // Ensure everything is off at startup
    MopMotors_SetSpeed(MOP_SPEED_OFF);
    WaterPump_SetState(PUMP_OFF);
}

void MopMotors_SetSpeed(MopSpeed_t speed) {
    uint32_t pwm_val = 0;

    switch(speed) {
        case MOP_SPEED_LOW:
            pwm_val = (MOP_TIMER_ARR * 50) / 100; // 50% Speed
            break;
        case MOP_SPEED_HIGH:
            pwm_val = MOP_TIMER_ARR;              // 100% Speed
            break;
        case MOP_SPEED_OFF:
        default:
            pwm_val = 0;                          // 0% Speed
            break;
    }

    // Apply to both mop motor channels
    __HAL_TIM_SET_COMPARE(&htim1, MOP_LEFT_CHANNEL, pwm_val);
    __HAL_TIM_SET_COMPARE(&htim1, MOP_RIGHT_CHANNEL, pwm_val);
}

void WaterPump_SetState(PumpState_t state) {
    if (state == PUMP_ON) {
        // Depending on your relay module, it might be active-low or active-high.
        // Assuming Active-High here:
        HAL_GPIO_WritePin(PUMP_RELAY_PORT, PUMP_RELAY_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(PUMP_RELAY_PORT, PUMP_RELAY_PIN, GPIO_PIN_RESET);
    }
}










/*
 * // Mopping State Trackers
static uint8_t pump_cycle_counter = 0;
static bool is_mopping_active = false;

void vAuxiliaryTask(void *pvParameters) {
    HardwareCommand_t received_cmd;
    Mopping_Init();

    while(1) {
        // Wait up to 1 second (1000ms) for a new command
        if (xQueueReceive(xHardwareCommandQueue, &received_cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {

            if (received_cmd.type == CMD_TYPE_MOPPING) {
                if (received_cmd.value == 0) {
                    // Turn Mopping OFF
                    is_mopping_active = false;
                    MopMotors_SetSpeed(MOP_SPEED_OFF);
                    WaterPump_SetState(PUMP_OFF);
                }
                else if (received_cmd.value == 1) {
                    // Turn Mopping ON
                    is_mopping_active = true;
                    pump_cycle_counter = 0; // Reset pulse cycle
                    MopMotors_SetSpeed(MOP_SPEED_HIGH);
                }
            }

        } else {
            // --- 1-Second Timeout Occurred ---
            // This block runs exactly once every second if no commands arrive.

            // 1. Handle Battery SOC (Your existing logic)
            Perform_Battery_SOC_Calculation();

            // 2. Handle Water Pump Pulsing
            if (is_mopping_active) {
                pump_cycle_counter++;

                // Example: Pulse the pump ON for 1 second, then OFF for 4 seconds.
                // This gives a 20% water flow rate without needing a PWM pump.
                if (pump_cycle_counter == 1) {
                    WaterPump_SetState(PUMP_ON);
                } else if (pump_cycle_counter >= 2) {
                    WaterPump_SetState(PUMP_OFF);
                }

                // Reset the cycle every 5 seconds
                if (pump_cycle_counter >= 5) {
                    pump_cycle_counter = 0;
                }
            }
        }
    }
}*/
