#pragma once

#include <drivers/remote.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C"
{
#endif

    

    /* ---------- 定义应用数据结构 ---------- */
    struct app_data
    {
        uint32_t last_receive_time;
        uint32_t packet_count;
    };

    extern struct app_data my_app_data;
    extern rc_sensor_t *rc_sensor;

    int Remote_Init(void);
    void Remote_Print(void);

#ifdef __cplusplus
}
#endif

