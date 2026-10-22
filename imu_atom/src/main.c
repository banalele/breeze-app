#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>
#include <SEGGER_RTT.h>

#include "imu_uart.h"
#include "imu_xrobot.h"

// RTT 通道索引
#define RTT_CH_LOG 0	// 通道 0 RTT 日志输出通道
#define RTT_CH_VOFA_1 1 // 通道 1
#define RTT_CH_VOFA_2 2 // 通道 2

static uint8_t vofa_buf_1[4096]; // 用于通道 1
static uint8_t vofa_buf_2[4096]; // 用于通道 2
// VOFA+ JustFloat模式 帧尾
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

/* RTT 上报周期：10 ms = 100 Hz
 * 串口解析仍是 1 ms（必须快，否则 ring 会溢出），只是发送降频
 */
#define VOFA_PERIOD_MS 10

LOG_MODULE_REGISTER(imu_atom, LOG_LEVEL_INF); // 开启日志模块



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

int main(void)
{
	vofa_rtt_init(); // 初始化 VOFA RTT 通道

	if (imu_uart_start() < 0)
	{
		LOG_ERR("ATOM-IMU 串口初始化失败");
	}

	int64_t stat_tick = k_uptime_get();
	int64_t vofa_tick = 0;

	while (1)
	{
		/* 串口数据 -> 环形缓冲 -> 组帧解析 -> imu_data（每 1 ms 一次） */
		imu_update();

		/* RTT 发送降频到 VOFA_PERIOD_MS，避免日志链路拖慢主循环 */
		int64_t now = k_uptime_get();
		if ((now - vofa_tick) >= VOFA_PERIOD_MS)
		{
			vofa_tick = now;

			float vofa_data[] = {
				(float)k_uptime_get_32() / 1000.0f, // 时间戳（秒）
				imu_data.eulr_.yaw,					// 偏航角（度）
				imu_data.eulr_.pit,					// 俯仰角（度）
				imu_data.eulr_.rol,					// 横滚角（度）
			};
			// 将数据包写入 RTT 上行通道
			SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_data, sizeof(vofa_data));
			// 写入帧尾
			SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_tail, sizeof(vofa_tail));
		}

		/* 每 5 s 打印一次当前姿态 */
		if ((k_uptime_get() - stat_tick) >= 5000)
		{
			stat_tick = k_uptime_get();
			LOG_INF("yaw=%+6f, pitch=%+6f, roll=%+6f",
					(double)imu_data.eulr_.yaw, (double)imu_data.eulr_.pit, (double)imu_data.eulr_.rol);
		}

		k_sleep(K_MSEC(1));
	}
}
