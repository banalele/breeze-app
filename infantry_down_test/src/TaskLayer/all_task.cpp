#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "conf_task.hpp"
#include "device.hpp"				// 设备

LOG_MODULE_REGISTER(all_task, LOG_LEVEL_INF);

extern "C" void StartSystemUpdateTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("System Update Task started");
    while (true)
    {
        breeze::Imu_Process();
        k_sleep(K_MSEC(1));
    }
}

extern "C" void StartUpdateTask(void *arg1, void *arg2, void *arg3)
{
    // 注意：参数名统一为 arg3，不要写 arg3t
    while (true)
    {
        k_sleep(K_MSEC(1));
    }
}

extern "C" void StartHeartbeatTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Heartbeat Task started");
    while (true)
    {
        k_sleep(K_MSEC(10));
    }
}

extern "C" void StartMonitorTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Monitor Task started");
    while (true)
    {
        //Remote_Print();
        k_sleep(K_MSEC(100));
    }
}
