#include "zephyr/device.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_basic, LOG_LEVEL_INF);

const struct device *accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));
typedef struct {
	float raw_acc_x;
    float raw_acc_y;
    float raw_acc_z;
    float raw_gyr_x;
    float raw_gyr_y;
    float raw_gyr_z;

} raw_info_t;

typedef struct {
	float yaw;
	float pitch;
	float roll;

	float rate_yaw;
	float rate_pitch;
	float rate_roll;
} base_info_t;

typedef enum {
	DEV_ONLINE,
	DEV_OFFLINE,
} dev_status_e;

typedef struct{
    dev_status_e dev_status;
    uint8_t offline_cnt;
    uint8_t offline_max_cnt;

}state_info_t;


typedef struct {
    raw_info_t raw_info;
    base_info_t base_info;
    state_info_t state_info;
	int16_t temp;
} imu_sensor_t;

imu_sensor_t imu_sensor = {
	.raw_info = {0},
	.base_info = {0},
	.state_info = {
	.dev_status = DEV_OFFLINE,
	.offline_cnt = 0,
	.offline_max_cnt = 10, // 可以在这里设置默认值
	},
	.temp = 0,
};

static void imu_init(imu_sensor_t *imu)
{
    imu->state_info.dev_status = DEV_OFFLINE;

	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Accelerometer device %s is not ready", accel_dev->name);
		imu->state_info.dev_status = DEV_OFFLINE;
		return;
	}

	if (!device_is_ready(gyro_dev)) {
		LOG_ERR("Gyroscope device %s is not ready", gyro_dev->name);
		imu->state_info.dev_status = DEV_OFFLINE;
		return;
	}

	LOG_INF("Accelerometer device %p name is %s", accel_dev, accel_dev->name);
	LOG_INF("Gyroscope device %p name is %s", gyro_dev, gyro_dev->name);
	imu->state_info.dev_status = DEV_ONLINE;
    imu->state_info.offline_cnt = 0;
    imu->state_info.offline_max_cnt = 10;
}


static void imu_update(imu_sensor_t *imu)
{
	struct sensor_value acc[3], gyr[3];
	if (imu->state_info.dev_status == DEV_OFFLINE) {
		LOG_ERR("IMU device is offline");
		imu->state_info.offline_cnt++;
		if (imu->state_info.offline_cnt >= imu->state_info.offline_max_cnt) {
			imu->state_info.dev_status = DEV_OFFLINE;
		}
		return ;
	} else {
		
		// 读取加速度计数据
		sensor_sample_fetch(accel_dev);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);

		// 读取陀螺仪数据
		sensor_sample_fetch(gyro_dev);
		sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
        imu->raw_info.raw_acc_x = sensor_value_to_float(&acc[0]);
        imu->raw_info.raw_acc_y = sensor_value_to_float(&acc[1]);  
        imu->raw_info.raw_acc_z = sensor_value_to_float(&acc[2]);
        imu->raw_info.raw_gyr_x = sensor_value_to_float(&gyr[0]);
        imu->raw_info.raw_gyr_y = sensor_value_to_float(&gyr[1]);
        imu->raw_info.raw_gyr_z = sensor_value_to_float(&gyr[2]);

		LOG_INF("Accelerometer: X=%.2f, Y=%.2f, Z=%.2f", (double)imu->raw_info.raw_acc_x, (double)imu->raw_info.raw_acc_y, (double)imu->raw_info.raw_acc_z);
		LOG_INF("Gyroscope: X=%.2f, Y=%.2f, Z=%.2f", (double)imu->raw_info.raw_gyr_x, (double)imu->raw_info.raw_gyr_y, (double)imu->raw_info.raw_gyr_z);
	    imu->state_info.offline_cnt = 0;
	}
}
int main()
{
	imu_init(&imu_sensor);
	while (1)
	{
		LOG_INF("Updating IMU data...");
		imu_update(&imu_sensor);
		k_msleep(500);
	}
}
