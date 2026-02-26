/*
 * ENC_DRIVE.c
 *
 *  Created on: Feb 17, 2026
 *      Author: Mohammed Hossam
 */

#include "robot_driver.h"



void Robot_Init(Robot_Odometry_t *robot, TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right, MPU6050_t *MPUcfg) {

	// 1. Initialize Left Encoder (Distance/Speed only)
	robot->left_enc.htim = htim_left;
	robot->left_enc.last_counter_value = __HAL_TIM_GET_COUNTER(htim_left);
	robot->left_enc.total_ticks = 0;
	robot->left_enc.velocity_m_s = 0;
	robot->left_enc.delta_distance_m = 0;
	robot->left_enc.is_32bit = 1;

	// 2. Initialize Right Encoder (Distance/Speed only)
	robot->right_enc.htim = htim_right;
	robot->right_enc.last_counter_value = __HAL_TIM_GET_COUNTER(htim_right);
	robot->right_enc.total_ticks = 0;
	robot->right_enc.velocity_m_s = 0;
	robot->right_enc.delta_distance_m = 0;
	robot->right_enc.is_32bit= 0;

	// 3. Initialize Global Robot Pose
	robot->x_m = 0.0f;
	robot->y_m = 0.0f;
	robot->theta_rad = 0.0f;
	MPUcfg->MPU_Yaw=0;

	// 4. Start Hardware Timers
	// Important: We use 0 to MAX_ARR in CubeMX so the counter loops naturally.
	HAL_TIM_Encoder_Start(htim_left, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_ALL);
}

// Helper to update one wheel
static void Update_Single_Encoder(Encoder_t *enc, float dt) {
	int32_t delta = 0;
	enc->current_counter_value= __HAL_TIM_GET_COUNTER(enc->htim);

	// Handle wrapping (overflow/underflow)
	// We cast to int16_t if using 16-bit timer, or int32_t for 32-bit timer.
	// Assuming 32-bit TIM2/TIM5 here. Use (int16_t) if using TIM3/TIM4!
	if(enc->is_32bit){
		delta = (int32_t)(enc->current_counter_value- enc->last_counter_value);
	}
	else {
		delta = (int16_t)(enc->current_counter_value- enc->last_counter_value);
	}
	enc->last_counter_value = enc->current_counter_value;
	enc->total_ticks += delta;
	enc->last_delta = delta;
	enc->delta_distance_m = delta * METERS_PER_TICK;

	if (dt > 0.0001f) {
		enc->velocity_m_s = enc->dist_traveled_m / dt;
	}
}

void Robot_Update(Robot_Odometry_t *robot,I2C_HandleTypeDef *I2Cx ,MPU6050_t *MPUcfg ,float dt) {

	// 1. Read Hardware
	//Encoder
	Update_Single_Encoder(&robot->left_enc, dt);
	Update_Single_Encoder(&robot->right_enc, dt);
	//MPU
	MPU6050_Read_All(I2Cx, MPUcfg, dt);


	// 2. Calculate
	// Kinematics
	float d_L = robot->left_enc.delta_distance_m;
	float d_R = robot->right_enc.delta_distance_m;
	float d_center = (d_L + d_R) / 2.0f;
	// 3. Calculate Delta Yaw from Encoders (in radians)
	// Formula: (Right_Distance - Left_Distance) / Robot_Track_Width
	float delta_yaw_enc = (d_R - d_L) / ROBOT_WIDTH_M;

	// 4. Calculate Delta Yaw from Gyroscope
	// Convert Gz from degrees/sec to radians/sec, then multiply by dt
	// Note: Gz must be the rotation around the Z-axis (perpendicular to the ground)
	float gyro_z_rad_s = MPUcfg->Gz * (M_PI / 180.0f);
	float delta_yaw_gyro = gyro_z_rad_s * dt;

	// 5. YAW SENSOR FUSION (Delta Fusion via Complementary Filter)
	// High trust in Gyro (0.95): It is fast and highly accurate in the short term.
	// Low trust in Encoders (0.05): It acts as a reference to pull the Gyro back and prevent long-term drift.
	float fused_delta_yaw = (0.95f * delta_yaw_gyro) + (0.05f * delta_yaw_enc);

	// 6. Update Global Robot Pose (Odometry)
	robot->theta_rad += fused_delta_yaw;
	robot->x_m += d_center * cosf(robot->theta_rad);
	robot->y_m += d_center * sinf(robot->theta_rad);

	// 7. Normalize the heading angle to keep it strictly between -PI and PI
	if (robot->theta_rad > M_PI) {
		robot->theta_rad -= 2.0f * M_PI;
	}
	else if (robot->theta_rad < -M_PI) {
		robot->theta_rad += 2.0f * M_PI;
	}
}
