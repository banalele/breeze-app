/**
 * @brief IMU 姿态角 C 语言接口
 */
#ifndef IMU_WRAPPER_H
#define IMU_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取 IMU 姿态角结构体
 */
typedef struct {
	float roll;          //度
	float pitch;        
	float yaw;           
	float yaw_total;     
	int16_t yaw_round;   
} imu_angles_t;

/**
 * @brief 获取 IMU 当前姿态角
 * @param angles 输出姿态角结构体指针
 */
void imu_get_angles(imu_angles_t *angles);

/**
 * @brief 获取单个横滚角
 * @return 横滚角（度）
 */
float imu_get_roll(void);

/**
 * @brief 获取单个俯仰角
 * @return 俯仰角（度）
 */
float imu_get_pitch(void);

/**
 * @brief 获取单个偏航角
 * @return 偏航角（度，-180°~180°）
 */
float imu_get_yaw(void);

/**
 * @brief 获取累积偏航角
 * @return 累积偏航角（度，可超过360°）
 */
float imu_get_yaw_total(void);

/**
 * @brief 获取陀螺仪 X 轴角速度
 * @return 角速度（rad/s）
 */
float imu_get_gyro_x(void);

/**
 * @brief 获取陀螺仪 Y 轴角速度
 * @return 角速度（rad/s）
 */
float imu_get_gyro_y(void);

/**
 * @brief 获取陀螺仪 Z 轴角速度
 * @return 角速度（rad/s）
 */
float imu_get_gyro_z(void);

/**
 * @brief 获取原始加速度 X 轴
 * @return 加速度（m/s²）
 */
float imu_get_accel_x(void);

/**
 * @brief 获取原始加速度 Y 轴
 * @return 加速度（m/s²）
 */
float imu_get_accel_y(void);

/**
 * @brief 获取原始加速度 Z 轴
 * @return 加速度（m/s²）
 */
float imu_get_accel_z(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_WRAPPER_H */
