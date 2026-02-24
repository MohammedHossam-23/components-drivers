/*
 * encoder_exti.h
 *
 *  Created on: Feb 24, 2026
 *      Author: Mohammed Hossam
 */


#ifndef STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ENCODER_EXTI_H_
#define STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ENCODER_EXTI_H_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
/* --- Motor Configuration Constants --- */

#define ENCODER_PULSES_PER_REV 		11.0f //Pulses per internal motor revolution
#define MOTOR_GEAR_RATIO 			90.0f // Gearbox reduction ratio (e.g., 1:45)
#define MOP_WHEEL_RADIUS_CM			7.0f  // Radius of the mopping disk
#define STALL_THRESHOLD_MS 			300   // Time without pulses to consider motor stalled

/* Total pulses seen by STM32 for one full output shaft rotation */
#define TOTAL_PPR 					(ENCODER_PULSES_PER_REV * MOTOR_GEAR_RATIO)

typedef struct {
	TIM_HandleTypeDef *timer_handle;	// Handle to TIM5 (32-bit timer)
	uint32_t last_timer_counter;			// Previous reading of the timer CNT register
	uint32_t last_pulse_timestamp_ms;		// System tick (ms) of the last received pulse
	volatile uint32_t pulse_interval_us;	// Measured time between two pulses in microseconds
	float output_shaft_rpm;					// Calculated RPM of the mopping disk
	float linear_velocity_cm_s;				// Calculated speed in cm/s
	int8_t is_stalled;						// Safety flag: 1 if motor is stuck, 0 otherwise
} HallEncoder_t;

/* Function Prototypes */
void Encoder_Init(HallEncoder_t *encoder, TIM_HandleTypeDef *htim);
void Encoder_Process_ISR(HallEncoder_t *encoder);
void Encoder_Update_Calculations(HallEncoder_t *encoder);


#endif /* STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ENCODER_EXTI_H_ */
