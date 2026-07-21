#pragma once

#include <drivers/remote.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* 遥控器拨杆状态枚举 */
    typedef enum {
        keep_R,         //保持
        up_R,           //向上拨
        mid_R,          //向中拨
        down_R,         //向下拨
    }remote_switch_status_e;

    /* 拨杆信息 */
    typedef struct {
        uint8_t value_last;  //上一次值
        uint8_t value;       //新值
        remote_switch_status_e status;      //状态
    }remote_switch_info_t;

    /* 兼容扩展数据 */
    typedef struct rc_extend_data {
        /* 拨轮阶梯上升沿 */
        bool step_rising[4];
        uint8_t step_last[4];

        /* 开关状态 */
        remote_switch_info_t s1_info;
        remote_switch_info_t s2_info;
    }rc_extend_data_t;


    /* ---------- 定义应用数据结构 ---------- */
    typedef struct app_data {
        uint32_t last_receive_time;
        uint32_t packet_count;
    }app_data_t;

    extern rc_extend_data_t rc_extend_data;
    extern app_data_t rc_app_data;
    extern rc_sensor_t *rc_sensor;

    int Remote_Init(const struct device *remote_dev);
    int rc_extend_data_update(rc_sensor_t *rc_sensor, rc_extend_data_t *rc_extend_data);


#ifdef __cplusplus
}
#endif

