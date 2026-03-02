/*
 * vacuum_esc.c
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */

#include "vacuum_esc.h"

// --- Hardware Configuration ---
extern TIM_HandleTypeDef htim4;
#define VACUUM_TIM_CHANNEL TIM_CHANNEL_3

// --- 16-bit ARR Configuration ---
// 16 bit "Counter Period" in CubeMX
#define VACUUM_TIMER_ARR   50000 	//20 ms
#define VACUUM_SPEED_OFF   2500 	// (1/20 * ARR) : 1 ms
#define VACUUM_SPEED_LOW   3750		//(3/40 * ARR) :  1.25 ms
#define VACUUM_SPEED_MED   4400		//(7/80 * ARR) :  1.5 ms
#define VACUUM_SPEED_HIGH  5000 	//(1/10 * ARR) :  2 ms

static VacuumState_t current_state = VACUUM_OFF;

void Vacuum_ESC_Init(void) {
	// Start the PWM generation
	HAL_TIM_PWM_Start(&htim4, VACUUM_TIM_CHANNEL);

	// Arm the ESC with a 0% duty cycle
	__HAL_TIM_SET_COMPARE(&htim4, VACUUM_TIM_CHANNEL, VACUUM_SPEED_OFF);

	// Give the ESC time to recognize the arming signal before RTOS starts
	HAL_Delay(2000);

	current_state = VACUUM_OFF;
}

void Vacuum_ESC_SetState(VacuumState_t state) {
	switch (state) {
	// Set to 100% Speed (Full Speed)
	case VACUUM_HIGH:
		__HAL_TIM_SET_COMPARE(&htim4, VACUUM_TIM_CHANNEL, VACUUM_SPEED_HIGH);
		current_state = VACUUM_HIGH;
		break;

		// Set to 75% Speed
	case VACUUM_MED:
		__HAL_TIM_SET_COMPARE(&htim4, VACUUM_TIM_CHANNEL, VACUUM_SPEED_MED);
		current_state = VACUUM_MED;
		break;

		// Set to 50% Speed
	case VACUUM_LOW:
		__HAL_TIM_SET_COMPARE(&htim4, VACUUM_TIM_CHANNEL, VACUUM_SPEED_LOW);
		current_state = VACUUM_LOW;
		break;

		// Set to Arming signal
	case VACUUM_OFF:
	default:
		__HAL_TIM_SET_COMPARE(&htim4, VACUUM_TIM_CHANNEL, VACUUM_SPEED_OFF);
		current_state = VACUUM_OFF;
		break;
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
