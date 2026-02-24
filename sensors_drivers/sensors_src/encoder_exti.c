/*
 * encoder_exti.c
 *
 *  Created on: Feb 24, 2026
 *      Author: Mohammed Hossam
 */




#include "encoder_exti.h"
/**
 * @brief Initializes the encoder structure and starts the hardware timer.
 */
void Encoder_Init(HallEncoder_t *encoder, TIM_HandleTypeDef *htim) {
	encoder->timer_handle = htim;
	encoder->last_timer_counter = 0;
	encoder->pulse_interval_us = 0;
	encoder->output_shaft_rpm = 0.0f;
	encoder->linear_velocity_cm_s = 0.0f;
	encoder->is_stalled = 0;
	encoder->last_pulse_timestamp_ms = HAL_GetTick();

	/* Start the 32-bit timer in base mode (Internal Clock) */
	HAL_TIM_Base_Start(htim);
}

/**
 * @brief Must be called within HAL_GPIO_EXTI_Callback when the encoder pin triggers.
 */
void Encoder_Process_ISR(HallEncoder_t *encoder) {
	/* Read the 32-bit counter value of TIM5 */
	uint32_t current_counter = __HAL_TIM_GET_COUNTER(encoder->timer_handle);//ask

	/* Calculate time elapsed since the last pulse (Handling 32-bit overflow) */
	if (current_counter > encoder->last_timer_counter) {
		encoder->pulse_interval_us = current_counter - encoder->last_timer_counter;
	} else {
		/* Rare case where the 32-bit timer wraps around (Approx. every 71 minutes at 1MHz) */
		encoder->pulse_interval_us = (0xFFFFFFFF - encoder->last_timer_counter) + current_counter;//ask
	}

	/* Update tracking variables */
	encoder->last_timer_counter = current_counter;
	encoder->last_pulse_timestamp_ms = HAL_GetTick();//ask
}

/**
 * @brief Calculates RPM and Velocity. Call this inside your periodic FreeRTOS Task.
 */
void Encoder_Update_Calculations(HallEncoder_t *encoder) {
	uint32_t current_time_ms = HAL_GetTick();
	uint32_t local_interval_us;

	/* 1. Safety Logic: Check if motor is stalled */
	if ((current_time_ms - encoder->last_pulse_timestamp_ms) > STALL_THRESHOLD_MS) {
		encoder->output_shaft_rpm = 0.0f;
		encoder->linear_velocity_cm_s = 0.0f;
		encoder->is_stalled = 1; // Trigger safety flag
		return;
	}

	/* 2. Atomic Read of the pulse interval to prevent data corruption during ISR */
	taskENTER_CRITICAL();
	local_interval_us = encoder->pulse_interval_us;
	taskEXIT_CRITICAL();

	/* 3. Perform Calculations if a pulse was recently detected */
	if (local_interval_us > 0) {
		encoder->is_stalled = 0;

		/* RPM = (Seconds in minute * Microseconds in second) / (Total Pulses * Pulse Interval) */
		encoder->output_shaft_rpm = (60.0f * 1000000.0f) / (TOTAL_PPR * local_interval_us);

		/* Velocity = (2 * PI * R * RPM) / 60 seconds */
		encoder->linear_velocity_cm_s = (2.0f * 3.14159f * MOP_WHEEL_RADIUS_CM * encoder->output_shaft_rpm) / 60.0f;
	}
}



//===================================================================================
//										Example
//===================================================================================
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == ENC_MOP_R)
//    {
//        // This links the hardware interrupt to your driver logic
//        Encoder_Process_ISR(&right_mop_encoder);
//    }
//	  else if (GPIO_Pin == ENC_MOP_L)
//	  {
//		  Encoder_Process_ISR(&left_mop_encoder);
//    }
//}
