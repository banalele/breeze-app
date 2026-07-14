#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include "imu_ekf.hpp"
#include <stdint.h>				// 整型类型
#include <SEGGER_RTT.h>			// RTT
#include "conf_task.hpp"
#include "device.hpp"				// 设备

LOG_MODULE_REGISTER(infantry_down_test, LOG_LEVEL_INF);

// RTT 通道索引
#define RTT_CH_LOG 0	// 通道 0 RTT 日志输出通道
#define RTT_CH_VOFA_1 1 // 通道 1
#define RTT_CH_VOFA_2 2 // 通道 2
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

static uint8_t vofa_buf_1[1024]; // 用于通道 1
static uint8_t vofa_buf_2[1024]; // 用于通道 2
// VOFA+ JustFloat模式 帧尾
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

using namespace breeze;

// 配置 RTT 通道
void vofa_rtt_init(void)
{
	// 上行通道
	SEGGER_RTT_ConfigUpBuffer(
		RTT_CH_VOFA_1,				  // 通道索引
		"vofa_channel_1",			  // 通道名称（仅供调试器显示）
		vofa_buf_1,					  // 缓冲区地址
		sizeof(vofa_buf_1),			  // 缓冲区大小
		SEGGER_RTT_MODE_NO_BLOCK_SKIP // 无阻塞模式：空间不足时丢弃新数据，不等待
	);
	SEGGER_RTT_ConfigUpBuffer(
		RTT_CH_VOFA_2,
		"vofa_channel_2",
		vofa_buf_2,
		sizeof(vofa_buf_2),
		SEGGER_RTT_MODE_NO_BLOCK_SKIP);

	LOG_INF("VOFA RTT channel configured\r\n");
}


int main()
{

	//Imu_Init(imu_sensor);
	if (Device_Init() != 0)
	{
		LOG_ERR("Device initialization failed, halting system.");
		// 停止系统，避免运行任务，同时输出报错地方
		k_panic(); 
	}
	infantry_down_test::InitProcess();
	while (1)
	{
			k_sleep(K_FOREVER); // 永远挂起，不消耗 CPU
		
	}
}
