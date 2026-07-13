#include "conf_task.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(conf_task, LOG_LEVEL_INF);

namespace infantry_down_test
{

    /* ---------- 定义线程控制块（实例化） ---------- */
    struct k_thread system_update_thread_ctrl;
    struct k_thread update_thread_ctrl;
    struct k_thread heartbeat_thread_ctrl;
    struct k_thread monitor_thread_ctrl;

    /* ---------- 定义栈空间（实例化） ---------- */
    K_KERNEL_STACK_DEFINE(system_update_stack, STACK_SIZE_SYSTEM);
    K_KERNEL_STACK_DEFINE(update_stack, STACK_SIZE_UPDATE);
    K_KERNEL_STACK_DEFINE(heartbeat_stack, STACK_SIZE_HEARTBEAT);
    K_KERNEL_STACK_DEFINE(monitor_stack, STACK_SIZE_MONITOR);

    /* ---------- 初始化所有任务 ---------- */
    void InitProcess(void)
    {
        LOG_INF("Creating all tasks...");

        /* 1. 创建 System Update Task */
        k_thread_create(&system_update_thread_ctrl,
                        system_update_stack,
                        K_KERNEL_STACK_SIZEOF(system_update_stack),
                        StartSystemUpdateTask,
                        nullptr, nullptr, nullptr,
                        proc_SystemTaskPriority,
                        0,          // 选项（0 表示默认）
                        K_NO_WAIT); // 立即启动
        LOG_INF("System Update Task created (priority %d)", proc_SystemTaskPriority);

        /* 2. 创建 Update Task（主任务） */
        k_thread_create(&update_thread_ctrl,
                        update_stack,
                        K_KERNEL_STACK_SIZEOF(update_stack),
                        StartUpdateTask,
                        nullptr, nullptr, nullptr,
                        proc_UpdateTaskPriority,
                        0,
                        K_NO_WAIT);
        LOG_INF("Update Task created (priority %d)", proc_UpdateTaskPriority);

        /* 3. 创建 Heartbeat Task */
        k_thread_create(&heartbeat_thread_ctrl,
                        heartbeat_stack,
                        K_KERNEL_STACK_SIZEOF(heartbeat_stack),
                        StartHeartbeatTask,
                        nullptr, nullptr, nullptr,
                        proc_HeartbeatTaskPriority,
                        0,
                        K_NO_WAIT);
        LOG_INF("Heartbeat Task created (priority %d)", proc_HeartbeatTaskPriority);

        /* 4. 创建 Monitor Task */
        k_thread_create(&monitor_thread_ctrl,
                        monitor_stack,
                        K_KERNEL_STACK_SIZEOF(monitor_stack),
                        StartMonitorTask,
                        nullptr, nullptr, nullptr,
                        proc_MonitorTaskPriority,
                        0,
                        K_NO_WAIT);
        LOG_INF("Monitor Task created (priority %d)", proc_MonitorTaskPriority);

        LOG_INF("All tasks created successfully!");
    }

} // namespace infantry_down_test