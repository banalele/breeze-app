#include "rc_sensor.h"

LOG_MODULE_REGISTER(remote, LOG_LEVEL_INF);

rc_sensor_t *rc_sensor;
rc_extend_data_t rc_extend_data = {
    .step_rising = {false, false, false, false},
    .step_last = {0, 0, 0, 0},

    .s1_info = {
        .value_last = 0,
        .value = 0,
        .status = keep_R,
    },
    .s2_info = {
        .value_last = 0,
        .value = 0,
        .status = keep_R,
    },
};
app_data_t rc_app_data = {
    .last_receive_time = 0,
    .packet_count = 0,
};

static void remote_data_ready(const struct device *dev, rc_sensor_t *sns, void *user_data)
{
    struct app_data *data = (struct app_data *)user_data;
    data->last_receive_time = k_uptime_get_32();
    data->packet_count++;
    rc_extend_data_update(sns, &rc_extend_data);
}

/* 初始化遥控器设备 */
int Remote_Init(const struct device *remote_dev)
{
    if (!device_is_ready(remote_dev))
    {
        LOG_ERR("Remote device not ready");
        return -1;
    }
    rc_sensor = remote_get_sensor(remote_dev);
    if (!rc_sensor)
    {
        LOG_ERR("Failed to get remote sensor");
        return -1;
    }

    remote_set_data_ready_cb(remote_dev, remote_data_ready, &rc_app_data);
    return 0;
}

/* 更新扩展数据 */
int rc_extend_data_update(rc_sensor_t *rc_sensor, rc_extend_data_t *rc_extend_data)
{
    if (!rc_sensor || !rc_sensor->info || !rc_extend_data) return -1;

    /* 拨轮阶梯上升沿 */
    for (int i = 0; i < 4; i++) {
        rc_extend_data->step_rising[i] = (rc_sensor->info->thumbwheel.step[i] != rc_extend_data->step_last[i]);
        rc_extend_data->step_last[i] = rc_sensor->info->thumbwheel.step[i];
    }

    /* 拨杆跳变 */
	/* 左拨杆判断 */
    rc_extend_data->s1_info.value = rc_sensor->info->s1;
    rc_extend_data->s2_info.value = rc_sensor->info->s2;
    if (rc_extend_data->s1_info.value != rc_extend_data->s1_info.value_last) {
		switch (rc_extend_data->s1_info.value)
		{
            case RC_SW_UP:
                rc_extend_data->s1_info.status = up_R;
                break;
            case RC_SW_MID:
                rc_extend_data->s1_info.status = mid_R;
                break;
            case RC_SW_DOWN:
                rc_extend_data->s1_info.status = down_R;
                break;
            default:
                break;
		}
	}
	else {
		rc_extend_data->s1_info.status = keep_R;
	}
    /* 右拨杆判断 */
    if (rc_extend_data->s2_info.value != rc_extend_data->s2_info.value_last) {
		switch (rc_extend_data->s2_info.value)
		{
            case RC_SW_UP:
                rc_extend_data->s2_info.status = up_R;
                break;
            case RC_SW_MID:
                rc_extend_data->s2_info.status = mid_R;
                break;
            case RC_SW_DOWN:
                rc_extend_data->s2_info.status = down_R;
                break;
            default:
                break;
		}
	}
	else {
		rc_extend_data->s2_info.status = keep_R;
	}
    rc_extend_data->s1_info.value_last = rc_extend_data->s1_info.value;
    rc_extend_data->s2_info.value_last = rc_extend_data->s2_info.value;

    return 0;
}
