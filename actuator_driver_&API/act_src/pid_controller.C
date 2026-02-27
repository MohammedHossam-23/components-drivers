/*
 * pid_controller.C
 *
 *  Created on: Feb 27, 2026
 *      Author: Mohammed Hossam
 */


#include "pid_controller.h"

// Since your loop runs strictly at 100Hz (10ms), dt is a constant 0.01 seconds.
#define DT_SECONDS 0.01f

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_out) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral_sum = 0.0f;
    pid->previous_error = 0.0f;
    pid->max_output = max_out;
    pid->max_integral = max_out; // Anti-windup threshold
}

int32_t PID_Compute(PID_Controller_t *pid, float setpoint_rpm, float measured_rpm) {
    float error = setpoint_rpm - measured_rpm;

    // Proportional
    float p_term = pid->Kp * error;

    // Integral with Anti-Windup
    pid->integral_sum += (error * DT_SECONDS);

    // Clamp the integrator to prevent it from spooling up to infinity if the wheels get stuck
    if (pid->integral_sum > pid->max_integral) pid->integral_sum = pid->max_integral;
    if (pid->integral_sum < -pid->max_integral) pid->integral_sum = -pid->max_integral;

    float i_term = pid->Ki * pid->integral_sum;

    // Derivative (prevent high frequency noise by calculating derivative of error)
    float d_term = pid->Kd * ((error - pid->previous_error) / DT_SECONDS);

    // Save error for next loop
    pid->previous_error = error;

    // Compute Total Output
    float output = p_term + i_term + d_term;

    // Clamp Final Output to Maximum PWM
    if (output > pid->max_output) output = pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;

    return (int32_t)output;
}

void PID_ResetIntegrator(PID_Controller_t *pid) {
    pid->integral_sum = 0.0f;
    pid->previous_error = 0.0f;
}




/*
 * // Globals inside your task file
PID_Controller_t pid_left;
PID_Controller_t pid_right;

void vSensingAndControlTask(void *pvParameters) {
    // 1. Initialization
    DriveMotor_Init();

    // Tune these Kp, Ki, Kd values on the physical robot later
    PID_Init(&pid_left,  10.0f, 5.0f, 0.1f, DRIVE_MAX_PWM);
    PID_Init(&pid_right, 10.0f, 5.0f, 0.1f, DRIVE_MAX_PWM);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // ... (Sense Encoders, Calculate measured_rpm_l and measured_rpm_r) ...

        if (CliffSensor_IsTriggered()) {
            // SAFETY FIRST: Instantly stop motors and clear PID memory
            DriveMotor_EmergencyStop();
            PID_ResetIntegrator(&pid_left);
            PID_ResetIntegrator(&pid_right);
        } else {
            // NORMAL CONTROL: Compute PID and drive
            int32_t pwm_l = PID_Compute(&pid_left,  target_rpm_l, measured_rpm_l);
            int32_t pwm_r = PID_Compute(&pid_right, target_rpm_r, measured_rpm_r);

            DriveMotor_SetPWM(MOTOR_LEFT, pwm_l);
            DriveMotor_SetPWM(MOTOR_RIGHT, pwm_r);
        }

        // Wait for exact 10ms cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}*/
