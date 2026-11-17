#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "conf_task.hpp"
#include "device.hpp"				// 设备
#include "module.h"

LOG_MODULE_REGISTER(all_task, LOG_LEVEL_INF);

extern "C" void StartSystemUpdateTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("System Task started");
    while (true)
    {
        k_sleep(K_MSEC(1));
    }
}

/* Update Task 每次循环的实际周期, 单位: 毫秒(ms), 供调试/监视使用 */
uint32_t cycle_time = 0;
float tourque0 = 0;
float tourque1 = 0;

/* 扭矩阶跃测试: 持续时间(ms) 与 扭矩值(N·m) */
static const int64_t TORQUE_TEST_MS = 1000;
static const float TORQUE_TEST_NM = 0.3f;

extern "C" void StartUpdateTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Update Task started");
    uint32_t cycle_last = k_cycle_get_32();
    const int64_t torque_test_start = k_uptime_get();
    while (true)
    {
        /* 等待 1ms 定时器节拍（中断里 k_sem_give 唤醒本线程） */
        k_sem_take(&infantry_down_test::update_timer_sem, K_FOREVER);

        /* 计算本次循环的实际周期(ms) */
        uint32_t cycle_now = k_cycle_get_32();
        cycle_time = k_cyc_to_ms_near32(cycle_now - cycle_last);
        cycle_last = cycle_now;

        breeze::Imu_Process();
        //gimbal_motor->tx_info->torque = 2;
        // for (int i = 0; i < GIMBAL_CNT; i++)
        // {
        //     gimbal_motor[i].single_set_torque(&gimbal_motor[i]);
        // }
        /* 测试: 任务启动后 500ms 内 motor0/motor1 同时输出 0.3N·m，之后回零 */
        // bool torque_test_on = (k_uptime_get() - torque_test_start) < TORQUE_TEST_MS;
        // tourque0 = torque_test_on ? TORQUE_TEST_NM : 0.0f;
        // tourque1 = tourque0;
        chassis_motor[0].tx_info->torque = tourque0;
        chassis_motor[1].tx_info->torque = tourque1;
        for (int i = 0; i < RMMOTOR_CNT; i++)
        {
            chassis_motor[i].single_set_torque(&chassis_motor[i]);
        }
    }
}

extern "C" void StartHeartbeatTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Heartbeat Task started");
    while (true)
    {
        Motor_Heartbeat();

        /* 喂狗 */
        IWDG_Feed(iwdg_dev, IWDG_Channel_ID);
        
        k_sleep(K_MSEC(100));
    }
}

// VOFA+ JustFloat模式 帧尾
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

extern "C" void StartMonitorTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Monitor Task started");
    while (true)
    {

        
		// 发送数据
        float vofa_data[] = {
            (float)k_uptime_get_32() / 1000.0f,
            (float)cycle_time,
            (float)chassis_motor[0].rx_info->encoder_speed,
            (float)chassis_motor[1].rx_info->encoder_speed,
        };
        SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_data, sizeof(vofa_data));
        SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_tail, sizeof(vofa_tail));

        k_sleep(K_MSEC(10));
    }
}
