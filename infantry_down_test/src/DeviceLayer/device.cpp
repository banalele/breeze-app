#include "device.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(device, LOG_LEVEL_INF);

//------------------------------获取设备节点---------------------------------------
const struct device *accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));
// 定义触发模式为当传感器数据到达水印即一定数量，要在设备树中定义，触发时包含传感器数据
#define STREAM_TRIGGERS \
    {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE}
// 创建 RTIO I/O 设备实例，关联到加速度计设备节点
SENSOR_DT_STREAM_IODEV(accel_iodev, DT_NODELABEL(bmi08x_accel), STREAM_TRIGGERS);
SENSOR_DT_STREAM_IODEV(gyro_iodev, DT_NODELABEL(bmi08x_gyro), STREAM_TRIGGERS);
// 内存池定义，提供给异步读取使用
RTIO_DEFINE_WITH_MEMPOOL(ctx, 16, 16, 64, 128, sizeof(void *));
/* 参数说明：名称, sq大小, cq大小, 内存池块数, 每块大小, 对齐方式 */

/* 遥控器 */
const struct device *remote_dev = DEVICE_DT_GET(DT_ALIAS(remote0));

/* 看门狗 */
const struct device *iwdg_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));

//-------------------------------------------------------------------------------

int Device_Init(void)
{
    int ret = 0;

    ret = IWDG_Init(iwdg_dev);
    if (ret != 0)
    {
        LOG_ERR("IWDG_Init failed with error code: %d", ret);
        return ret;
    }
    
    ret = Remote_Init(remote_dev);
    if (ret != 0)
    {
        LOG_ERR("Remote_Init failed with error code: %d", ret);
        return ret;
    }
    
    ret = Motor_Init();
    if (ret != 0)
    {
        LOG_ERR("Motor_Init failed with error code: %d", ret);
        return ret;
    }
    
    ret = breeze::Imu_Init(breeze::imu_sensor);
    if (ret != 0)
    {
        LOG_ERR("Imu_Init failed with error code: %d", ret);
        return ret;
    }
    chassis.init(&chassis);
    infantry.init(&infantry);
    vofa_rtt_init();
    return 0;
}