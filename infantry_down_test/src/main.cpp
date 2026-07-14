#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include <stdint.h>				// 整型类型
#include <SEGGER_RTT.h>			// RTT
#include "conf_task.hpp"
#include "device.hpp"				// 设备

LOG_MODULE_REGISTER(infantry_down_test, LOG_LEVEL_INF);

// RTT 通道索引
#define RTT_CH_LOG 0	// 通道 0 RTT 日志输出通道
#define RTT_CH_VOFA_1 1 // 通道 1
#define RTT_CH_VOFA_2 2 // 通道 2

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
