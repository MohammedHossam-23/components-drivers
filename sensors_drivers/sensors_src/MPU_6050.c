/*
 * MPU_6050.c
 *
 *  Created on: Feb 15, 2026
 *      Author: Mohammed Hossam
 */


/*
 * Description: Implementation of MPU6050 Driver with English Comments
 */

#include "MPU_6050.h"

// ============================================================================
// 1. Initialization Function
// ============================================================================
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
	uint8_t check;
	uint8_t Data;

	// 1. Check Device ID (WHO_AM_I)
	// We read the register 0x75. It should return 0x68.
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);

	if (check == 0x68) // 0x68 is the correct ID for MPU6050
	{
		// 2. Wake up the sensor (Power Management)
		// Write 0 to PWR_MGMT_1 register to wake it up from sleep mode
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 100);

		// 3. set DLPF  to 5
		// Accelerometer Bandwidth :10 Hz delay:13.8 ms
		// Gyroscope	 Bandwidth :10 Hz delay:13.4 ms
		//Fs : 1 KHz
		Data = 0x05;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 100);


		// 4. Set Sample Rate Divider each 10 ms at 100Hz
		Data = 0x09;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 100);

		// 5. Set Accelerometer Configuration -> ± 8g
		// Bit 4:3 set to 10 binary (2 decimal) corresponds to ±8g
		Data = 0x10;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 100);

		// 6. Set Gyroscope Configuration -> ± 500 °/s
		// Bit 4:3 set to 01 binary (1 decimal) corresponds to ±500 dps
		Data = 0x08;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 100);

		// 7. Wake up the sensor (Power Management)
		// Write 0 to PWR_MGMT_1 register to wake it up from sleep mode
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 100);

		return 0; // Initialization Successful
	}

	return 1; // Initialization Failed (Sensor not found or wiring issue)
}

// ============================================================================
// 2. Read Data Function
// ============================================================================
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *MPUcfg , double dt)
{
	uint8_t check, Data;
	uint8_t Rec_Data[14];

	// Read 14 bytes starting from ACCEL_XOUT_H register
	// Order: Accel(6 bytes) -> Temp(2 bytes) -> Gyro(6 bytes)
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 100);

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &check, 1, 10);
	if (check != 0x00) {
	    MPU6050_Init(I2Cx); // Re-init if sensor is in sleep mode
	}


	// --- Process Accelerometer ---

	// Init the Accelerometer
	// Wake up the sensor (Power Management)
	// Write 0 to PWR_MGMT_1 register to wake it up from sleep mode
	Data = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 100);

	// set DLPF  to 5
	// Accelerometer Bandwidth :10 Hz delay:13.8 ms
	// Gyroscope	 Bandwidth :10 Hz delay:13.4 ms
	//Fs : 1 KHz
	Data = 0x05;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 100);


	// Set Sample Rate Divider each 10 ms at 100Hz
	Data = 0x09;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 100);

	// Set Accelerometer Configuration -> ± 8g
	// Bit 4:3 set to 10 binary (2 decimal) corresponds to ±8g
	Data = 0x10;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 100);


	// Combine High Byte and Low Byte
	MPUcfg->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	MPUcfg->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	MPUcfg->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// Convert to physical 'g' values
	// We divide by 4096.0 because we selected ±8g range (LSB Sensitivity)
	MPUcfg->Ax = MPUcfg->Accel_X_RAW / 4096.0;
	MPUcfg->Ay = MPUcfg->Accel_Y_RAW / 4096.0;
	MPUcfg->Az = MPUcfg->Accel_Z_RAW / 4096.0;

	// to Calculate pitch/roll from Accelerometer
	// Using atan to get angles in radians, then convert to degrees
	// AccX (Roll)
	MPUcfg->AccAngleRoll = (atan(MPUcfg->Accel_Y_RAW / sqrt(pow(MPUcfg->Accel_X_RAW, 2) + pow(MPUcfg->Accel_Z_RAW, 2))) * RAD_TO_DEG) - 0.58;

	// AccY (Pitch)
	MPUcfg->AccAnglePitch= (atan(-1 * MPUcfg->Accel_X_RAW / sqrt(pow(MPUcfg->Accel_Y_RAW, 2) + pow(MPUcfg->Accel_Z_RAW, 2))) * RAD_TO_DEG) + 1.58;


	//===============================================================================================



	// --- Process Gyroscope ---

	// Set Gyroscope Configuration -> ± 500 °/s
	// Bit 4:3 set to 01 binary (1 decimal) corresponds to ±500 dps
	Data = 0x08;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 100);



	// Skipping bytes 6 and 7 (Temperature data)
	MPUcfg->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	MPUcfg->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	MPUcfg->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	// Convert to physical 'deg/s' values
	// We divide by 65.5 because we selected ±500dps range (LSB Sensitivity)
	MPUcfg->Gx = MPUcfg->Gyro_X_RAW / 65.5 + 2.7;
	MPUcfg->Gy = MPUcfg->Gyro_Y_RAW / 65.5 - 0.86;
	MPUcfg->Gz = MPUcfg->Gyro_Z_RAW / 65.5 + 0.40;



	//Complementary Filter (Sensor Fusion)
	// Formula: Angle = Alpha * (Old_Angle + Gyro_Rate * dt) + (1 - Alpha) * Accel_Angle
	MPUcfg->FinalAngleRoll  = ALPHA * (MPUcfg->FinalAngleRoll + MPUcfg->Gx * dt) + (1.0 - ALPHA) * (MPUcfg->AccAngleRoll - 0.58);
	MPUcfg->FinalAnglePitch = ALPHA * (MPUcfg->FinalAnglePitch + MPUcfg->Gy * dt) + (1.0 - ALPHA) * (MPUcfg->AccAnglePitch + 1.58);

	// Yaw Calculation (Gyro Integration Only)
	MPUcfg->MPU_Yaw += MPUcfg->Gz *dt;
}






//===================================================
//			dt (Detla Time) sudo implemention
//===================================================
/*
 *void vSensingAndControlTask(void * pvParameters) {
    // 1. التوقيت الثابت (كما هو في مخططك)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
    const double dt = 0.01; // القيمة الثابتة التي ستستخدمها في كل المعادلات

    for(;;) {
        // انتظر حتى يحين موعد الدورة القادمة بدقة
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 2. Read Sensors (MPU, Encoders, etc.)
        // استدعِ دالة القراءة التي كتبناها ومرر لها dt الثابت
        MPU6050_Read_And_Process(&hi2c1, &myMPUStruct, dt);

        // 3. Odometry & PID
        // استخدم نفس الـ dt لحساب المسافة من الانكودر والـ PID
        Calculate_Odometry(&myOdomStruct, dt);
        Run_PID_Controllers(dt);

        // 4. Share Data (Update Global Vars)
        Update_Global_Variables();
    }
}
 *
 * /

// another method
/*
 * void vSensingAndControlTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t previous_time = xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // الهدف: 10ms

    for(;;) {
        // 1. نام لحد ما الـ 10ms تخلص
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 2. احسب الـ dt الفعلي اللي عدى بجد (عشان لو الـ RTOS اتأخر ملي ثانية ولا حاجة)
        TickType_t current_time = xTaskGetTickCount();
        float actual_dt = (float)(current_time - previous_time) / 1000.0f;
        previous_time = current_time;

        // 3. مرر الـ dt الحقيقي والموحد لكل الروبوت
        ACS758_Update(&myBattery, adc_buffer[0], actual_dt);
        MPU6050_Read_And_Process(&hi2c1, &myMPUStruct, actual_dt);
        Calculate_Odometry(&myOdomStruct, actual_dt);
        Run_PID_Controllers(actual_dt);
        Update_Global_Variables();
    }
}
 * */






//
//// ============================================================================
//// 3. Complementary Filter Function
//// ============================================================================
//void MPU6050_Process_Complementary(MPU6050_t *DataStruct, double dt) {
//	// 1. Calculate pitch/roll from Accelerometer (Noisy but accurate over long term)
//	// Using atan2 to get angles in radians, then convert to degrees
//	double accel_roll = atan2(DataStruct->Ay, DataStruct->Az) * RAD_TO_DEG;
//	double accel_pitch = atan2(-DataStruct->Ax, sqrt(DataStruct->Ay * DataStruct->Ay + DataStruct->Az * DataStruct->Az)) * RAD_TO_DEG;
//
//	// 2. Apply Complementary Filter
//	// Formula: Angle = alpha * (old_angle + gyro_rate * dt) + (1 - alpha) * accel_angle
//	// We trust Gyroscope for short term changes, and Accelerometer for long term drift correction
//
//	// Calculate Roll
//	DataStruct->AngleRoll = COMPLEMENTARY_ALPHA * (DataStruct->AngleRoll + DataStruct->Gx * dt)
//                        														  + (1.0 - COMPLEMENTARY_ALPHA) * accel_roll;
//
//	// Calculate Pitch (This is the main angle for Self-Balancing Robot)
//	DataStruct->AnglePitch = COMPLEMENTARY_ALPHA * (DataStruct->AnglePitch + DataStruct->Gy * dt)
//                        														   + (1.0 - COMPLEMENTARY_ALPHA) * accel_pitch;
//}
*/
