/**
 * @brief IMU 姿态角 C 语言接口实现
 */

/***************example*****************/
/*
#include "imu_wrapper.h"

void my_module_update(void)
{
    // === 方式1：一次性获取所有姿态角 ===
    imu_angles_t angles;
    imu_get_angles(&angles);

    // angles.roll      — 横滚角 (°)
    // angles.pitch     — 俯仰角 (°)
    // angles.yaw       — 偏航角 (°)
    // angles.yaw_total — 累积偏航角 (°)
    // angles.yaw_round — 偏航圈数

    // === 方式2：单独获取某个值 ===
    float roll  = imu_get_roll();        // °
    float pitch = imu_get_pitch();       // °
    float yaw   = imu_get_yaw();         // °

    float gyro_x = imu_get_gyro_x();     // rad/s
    float gyro_y = imu_get_gyro_y();     // rad/s
    float gyro_z = imu_get_gyro_z();     // rad/s

    float acc_x = imu_get_accel_x();     // m/s²
    float acc_y = imu_get_accel_y();     // m/s²
    float acc_z = imu_get_accel_z();     // m/s²
}
*/


#include "imu_wrapper.h"
#include "imu_ekf.hpp"

extern "C" {

void imu_get_angles(imu_angles_t *angles)
{
	if (!angles) return;

	const auto &gimbal = breeze::imu_sensor.gimbal_info;
	angles->roll      = gimbal.G_Roll;
	angles->pitch     = gimbal.G_Pitch;
	angles->yaw       = gimbal.G_Yaw;
	angles->yaw_total = gimbal.YawTotalAngle;
	angles->yaw_round = gimbal.YawRoundCount;
}

float imu_get_roll(void)
{
	return breeze::imu_sensor.gimbal_info.G_Roll;
}

float imu_get_pitch(void)
{
	return breeze::imu_sensor.gimbal_info.G_Pitch;
}

float imu_get_yaw(void)
{
	return breeze::imu_sensor.gimbal_info.G_Yaw;
}

float imu_get_yaw_total(void)
{
	return breeze::imu_sensor.gimbal_info.YawTotalAngle;
}

float imu_get_gyro_x(void)
{
	return breeze::imu_sensor.Gyro[0];
}

float imu_get_gyro_y(void)
{
	return breeze::imu_sensor.Gyro[1];
}

float imu_get_gyro_z(void)
{
	return breeze::imu_sensor.Gyro[2];
}

float imu_get_accel_x(void)
{
	return breeze::imu_sensor.Accel[0];
}

float imu_get_accel_y(void)
{
	return breeze::imu_sensor.Accel[1];
}

float imu_get_accel_z(void)
{
	return breeze::imu_sensor.Accel[2];
}

} /* extern "C" */
