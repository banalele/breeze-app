#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "conf_task.hpp"

LOG_MODULE_REGISTER(all_task, LOG_LEVEL_INF);


namespace infantry_down_test
{

    /**
     * @brief 系统层更新任务
     * @note 与其它层独立开来是因为频率不需要那么高(并非，目前为了能对应上设备层1KHz的更新频率，这里也由原来的500Hz改成了1KHz)
     */
    extern "C" void StartSystemUpdateTask(void *argument)
    {
        LOG_INF("System Update Task started");

        while (true)
        {
            k_sleep(K_MSEC(1));
        }
    }

    /**
     * @brief 其余层的更新任务以及系统核心更新任务（1000Hz）
     */
    extern "C" void StartUpdateTask(void *argument)
    {
        /* 初始化系统核心（等所有模块初始化完成之后再初始化） */

        while (true)
        {
        
            k_sleep(K_MSEC(1));
        }
    }

    /**
     * @brief 心跳任务以及看门狗（100Hz）
     */
    extern "C" void StartHeartbeatTask(void *argument)
    {
        LOG_INF("Heartbeat Task started");

        while (true)
        {
            
            k_sleep(K_MSEC(10));
        }
    }

    /**
     * @brief 监控任务，用于测试 RTT 的调试功能（100Hz）
     */
    extern "C" void StartMonitorTask(void *argument)
    {
        LOG_INF("Monitor Task started");

        while (true)
        {
            /* RTT 调试输出（使用 Zephyr LOG 代替） */
            k_sleep(K_MSEC(10));
        }
    }

} // namespace my_engineer