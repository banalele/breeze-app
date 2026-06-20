#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>


typedef struct
{
    float raw_acc_x;
    float raw_acc_y;
    float raw_acc_z;
    float raw_gyr_x;
    float raw_gyr_y;
    float raw_gyr_z;

} raw_info_t;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float yaw_total_angle;

    float accx;
    float accy;
    float accz;

    float rate_yaw;
    float rate_pitch;
    float rate_roll;

    float ave_rate_yaw;
    float ave_rate_pitch;
    float ave_rate_roll;

} base_info_t;

typedef enum
{
    DEV_ONLINE,
    DEV_OFFLINE,
} dev_status_e;

typedef enum
{
    IMU_DATA_ERR,
    IMU_DATA_CALI,
    IMU_NONE_ERR,
    
} imu_err_e;

typedef struct
{
    dev_status_e dev_status;

    int8_t init_code;
    uint8_t init_flag;
    uint8_t err_cnt;
    imu_err_e err_code;
    uint8_t offline_cnt;
    uint8_t offline_max_cnt;

} state_info_t;

typedef struct
{
    float gx_offset;
    float gy_offset;
    float gz_offset;
} offset_info_t;

typedef struct imu_info_struct
{
    raw_info_t raw_info;
    base_info_t base_info;
    offset_info_t offset_info;
} imu_info_t;
typedef struct
{
    imu_info_t info;
    state_info_t state_info;
} imu_sensor_t;



#endif
