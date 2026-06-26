#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include "imu_ekf.hpp"
#include <stdint.h>				// 整型类型
#include <SEGGER_RTT.h>			// RTT

LOG_MODULE_REGISTER(imu_ekf_stream, LOG_LEVEL_INF);

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

// ===== 添加线程相关定义 =====
#define IMU_THREAD_STACK_SIZE 4096
#define IMU_THREAD_PRIORITY 5
struct k_thread imu_thread_data;
K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);

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

//---------
void imu_thread_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	while (1)
	{
		imu_process();
		k_usleep(500);
	}
}


void Create_Imu_Thread(void)
{
	k_thread_create(
		&imu_thread_data,						 // 线程控制块
		imu_thread_stack,						 // 栈空间
		K_THREAD_STACK_SIZEOF(imu_thread_stack), // 栈大小
		imu_thread_task,						 // 入口函数
		NULL, NULL, NULL,						 // 参数
		IMU_THREAD_PRIORITY,					 // 优先级（数字越小优先级越高）
		0,										 // 线程选项
		K_NO_WAIT								 // 启动
	);
	k_thread_name_set(&imu_thread_data, "imu_process");
	LOG_INF("System started, IMU thread running");
}

int main()
{
	vofa_rtt_init(); // 初始化 VOFA RTT 通道
	Imu_Init(imu_sensor);
	Create_Imu_Thread();
	while (1)
	{
		static uint32_t vofa_cnt = 0;
		vofa_cnt++;
		if (vofa_cnt % 20 == 0) {
			// 发送数据
			float vofa_data[] = {
				(float)k_uptime_get_32() / 1000.0f,
				(float)imu_sensor.gimbal_info.G_Yaw,
				(float)imu_sensor.gimbal_info.G_Pitch,
				(float)imu_sensor.gimbal_info.G_Roll,
			};
			SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_data, sizeof(vofa_data));
			SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_tail, sizeof(vofa_tail));
		}
		k_usleep(500);
	}
	return 0;
}
