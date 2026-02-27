/*
 * pid_controller.h
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */

#ifndef ACTUATOR_DRIVER__API_ACT_SRC_PID_CONTROLLER_H_
#define ACTUATOR_DRIVER__API_ACT_SRC_PID_CONTROLLER_H_

typedef struct {
    // Tuning Parameters
    float Kp;
    float Ki;
    float Kd;

    // State Variables
    float integral_sum;
    float previous_error;

    // Limits
    float max_output;     // e.g., DRIVE_MAX_PWM
    float max_integral;   // Anti-windup limit
} PID_Controller_t;

/**
 * @brief Initializes the PID controller struct with gains and limits.
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_out);

/**
 * @brief Computes the new PWM output. Call this at exactly 100Hz.
 * @param pid Pointer to the specific wheel's PID instance.
 * @param setpoint_rpm The target speed.
 * @param measured_rpm The actual speed (calculated from encoders).
 * @return The new PWM command (-max_output to +max_output).
 */
int32_t PID_Compute(PID_Controller_t *pid, float setpoint_rpm, float measured_rpm);

/**
 * @brief Resets the integral sum to 0. Crucial after a cliff detection/emergency stop.
 */
void PID_ResetIntegrator(PID_Controller_t *pid);


#endif /* ACTUATOR_DRIVER__API_ACT_SRC_PID_CONTROLLER_H_ */
