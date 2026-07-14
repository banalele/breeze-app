#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include <stdint.h>				// 整型类型
#include <SEGGER_RTT.h>			// RTT
#include "conf_task.hpp"
#include "device.hpp"				// 设备

LOG_MODULE_REGISTER(infantry_down_test, LOG_LEVEL_INF);


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
