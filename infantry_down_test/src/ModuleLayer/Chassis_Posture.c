#include "Chassis_Posture.h"
#include "imu_wrapper.h"

static void Chassis_Posture_Update(Chassis_Posture_t *My_Chassis_Posture);

Chassis_Posture_info_t Chassis_Posture_info;

Chassis_Posture_t Chassis_Posture = {
	.data_update = Chassis_Posture_Update,
	.info = &Chassis_Posture_info,
};

float roll_offset = 0.0f;
float pitch_offset = 0.0f;

static void Chassis_Posture_Update(Chassis_Posture_t *My_Chassis_Posture)
{
	Chassis_Posture_info_t *info = My_Chassis_Posture->info;

	//角度更新
	info->pitch = imu_get_pitch() * Degree_to_rad + pitch_offset * Degree_to_rad;
	info->roll  = -imu_get_roll() * Degree_to_rad;
	info->yaw   = imu_get_yaw() * Degree_to_rad;

	//角速度更新（IMU陀螺仪输出已是 rad/s，不用乘 Degree_to_rad）
	info->roll_v  = -imu_get_gyro_x();
	info->pitch_v =  imu_get_gyro_y();
	info->yaw_v   =  imu_get_gyro_z();

	//加速度更新
	info->a_x =  imu_get_accel_x();
	info->a_y = -imu_get_accel_y();
	info->a_z =  imu_get_accel_z();

	// TODO: 世界加速度更新 — 待 IMU 驱动完善后补充
	// Zephyr IMU 驱动尚未实现此功能，待后续补齐后取消注释
	// info->x_world = ...;
	// info->y_world = ...;
	// info->z_world = ...;
}
