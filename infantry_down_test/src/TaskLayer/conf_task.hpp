#pragma once

#include "zephyr/kernel.h"

    /* 任务优先级定义 */
    #define proc_SystemTaskPriority 3    // 系统更新任务
    #define proc_UpdateTaskPriority 1    // 主更新任务（最高）
    #define proc_HeartbeatTaskPriority 4 // 心跳任务
    #define proc_MonitorTaskPriority 5   // 监控任务（最低）

    /* 任务栈大小定义 */
    #define STACK_SIZE_SYSTEM 2048
    #define STACK_SIZE_UPDATE 4096
    #define STACK_SIZE_HEARTBEAT 1024
    #define STACK_SIZE_MONITOR 2048

#ifdef __cplusplus
    extern "C"
    {
#endif

        // 任务回调函数（C链接，供Zephyr内核直接调用）
        void StartSystemUpdateTask(void *arg1, void *arg2, void *arg3);
        void StartUpdateTask(void *arg1, void *arg2, void *arg3);
        void StartHeartbeatTask(void *arg1, void *arg2, void *arg3);
        void StartMonitorTask(void *arg1, void *arg2, void *arg3);

#ifdef __cplusplus
    }
#endif

    namespace infantry_down_test
    {
        /* 线程控制块（替代 TaskHandle_t） */
        extern struct k_thread system_update_thread_ctrl;
        extern struct k_thread update_thread_ctrl;
        extern struct k_thread heartbeat_thread_ctrl;
        extern struct k_thread monitor_thread_ctrl;

        /* 初始化所有任务 */
        void InitProcess(void);

} // namespace infantry_down_test