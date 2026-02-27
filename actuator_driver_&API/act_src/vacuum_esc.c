/*
 * vacuum_esc.c
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */

#include "vacuum_esc.h"

// --- Hardware Configuration ---
extern TIM_HandleTypeDef htim3;
#define VACUUM_TIM_CHANNEL TIM_CHANNEL_1

// --- 16-bit ARR Configuration ---
// Must match the "Counter Period" in CubeMX
#define VACUUM_TIMER_ARR 65535

static VacuumState_t current_state = VACUUM_OFF;

void Vacuum_ESC_Init(void) {
    // Start the PWM generation
    HAL_TIM_PWM_Start(&htim3, VACUUM_TIM_CHANNEL);

    // Arm the ESC with a 0% duty cycle
    __HAL_TIM_SET_COMPARE(&htim3, VACUUM_TIM_CHANNEL, 0);

    // Give the ESC time to recognize the arming signal before RTOS starts
    HAL_Delay(2000);

    current_state = VACUUM_OFF;
}

void Vacuum_ESC_SetState(VacuumState_t state) {
    if (state == VACUUM_ON) {
        // Set to 100% Duty Cycle (Full Speed)
        __HAL_TIM_SET_COMPARE(&htim3, VACUUM_TIM_CHANNEL, VACUUM_TIMER_ARR);
        current_state = VACUUM_ON;
    } else {
        // Set to 0% Duty Cycle (Off)
        __HAL_TIM_SET_COMPARE(&htim3, VACUUM_TIM_CHANNEL, 0);
        current_state = VACUUM_OFF;
    }
}

VacuumState_t Vacuum_ESC_GetState(void) {
    return current_state;
}

//task logic:
//When vMicroROSTask receives a /vacuum_cmd (which can now just be a boolean true/false),
//it sends it to the queue, and your auxiliary task handles it instantly.

/*void vAuxiliaryTask(void *pvParameters) {
    HardwareCommand_t received_cmd;

    while(1) {
        // Wait up to 1 second for a command
        if (xQueueReceive(xHardwareCommandQueue, &received_cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {

            if (received_cmd.type == CMD_TYPE_VACUUM) {
                // received_cmd.value is now just 1 (ON) or 0 (OFF)
                Vacuum_ESC_SetState((VacuumState_t)received_cmd.value);
            }

        } else {
            // 1-Second Timeout: Run slow background tasks
            Perform_Battery_SOC_Calculation();
        }
    }
}*/
