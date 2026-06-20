#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include "imu_ekf.hpp"
#include <stdint.h>				// 整型类型
#include <SEGGER_RTT.h>			// RTT

LOG_MODULE_REGISTER(imu_ekf_sample, LOG_LEVEL_INF);

// RTT 通道索引
#define RTT_CH_LOG 0	// 通道 0 RTT 日志输出通道
#define RTT_CH_VOFA_1 1 // 通道 1
#define RTT_CH_VOFA_2 2 // 通道 2
const struct device *accel_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
const struct device *gyro_dev = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));

static uint8_t vofa_buf_1[4096]; // 用于通道 1
static uint8_t vofa_buf_2[4096]; // 用于通道 2
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
	vofa_rtt_init(); // 初始化 VOFA RTT 通道
	Imu_Init(imu_sensor);
	
	while (1)
	{
		//uint32_t start = k_cycle_get_32();
		Imu_Update(imu_sensor);
		//uint32_t cycles = k_cycle_get_32() - start;

		// 直接看周期数（最直观）
		//LOG_WRN("Imu_Update took %u cycles", cycles);

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
}
