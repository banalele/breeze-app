#include "device_remote.h"

LOG_MODULE_REGISTER(remote, LOG_LEVEL_INF);

#define DR16_REMOTE_NODE DT_ALIAS(remote0)
#if !DT_NODE_HAS_STATUS_OKAY(DR16_REMOTE_NODE)
#error "DT alias 'remote0' 未定义或禁用"
#endif

static const struct device *remote_dev;
rc_sensor_t *rc_sensor;

struct app_data my_app_data = {
    .last_receive_time = 0,
    .packet_count = 0,
};

static void remote_data_ready(const struct device *dev, rc_sensor_t *sns, void *user_data)
{
    struct app_data *data = (struct app_data *)user_data;
    data->last_receive_time = k_uptime_get_32();
    data->packet_count++;
}

int Remote_Init(void)
{
    remote_dev = DEVICE_DT_GET(DR16_REMOTE_NODE);
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

    remote_set_data_ready_cb(remote_dev, remote_data_ready, &my_app_data);
    return 0;
}

void Remote_Print(void)
{
    if (!rc_sensor->is_online)
    {
        printk("\033[2J\033[H");
        printk("====================================\n");
        printk("         DJI DR16 RECEIVER          \n");
        printk("====================================\n");
        printk(" STATUS: OFFLINE\n");
    }
    else
    {
        printk("\033[2J\033[H");
        printk("==============================================\n");
        printk("              DJI DR16 RECEIVER               \n");
        printk("==============================================\n");
        printk(" STATUS: ONLINE   |  PACKETS: %-6u\n", my_app_data.packet_count);
        printk(" UPTIME: %-8u |  S1: %d  S2: %d\n", my_app_data.last_receive_time, rc_sensor->info->s1, rc_sensor->info->s2);
        printk("----------------------------------------------\n");
        printk(" [ RIGHT STICK ]  |  [ LEFT STICK ] \n");
        printk("  CH0 (X): %-6d |   CH2 (X): %-6d\n", rc_sensor->info->ch0, rc_sensor->info->ch2);
        printk("  CH1 (Y): %-6d |   CH3 (Y): %-6d\n", rc_sensor->info->ch1, rc_sensor->info->ch3);
        printk("----------------------------------------------\n");
        printk(" [ THUMBWHEEL ]   :  %-6d  (Steps: %d%d%d%d)\n",
               rc_sensor->info->thumbwheel.value,
               rc_sensor->info->thumbwheel.step[0], rc_sensor->info->thumbwheel.step[1],
               rc_sensor->info->thumbwheel.step[2], rc_sensor->info->thumbwheel.step[3]);
        printk("----------------------------------------------\n");
        printk(" [ MOUSE AXES ]   |  [ MOUSE BUTTONS ]\n");
        printk("  VX: %-6d      |   LEFT:  %d (Cnt: %d)\n", rc_sensor->info->mouse_vx, rc_sensor->info->mouse_btn_l.value, rc_sensor->info->mouse_btn_l.cnt);
        printk("  VY: %-6d      |   RIGHT: %d (Cnt: %d)\n", rc_sensor->info->mouse_vy, rc_sensor->info->mouse_btn_r.value, rc_sensor->info->mouse_btn_r.cnt);
        printk("  VZ: %-6d      |\n", rc_sensor->info->mouse_vz);
        printk("----------------------------------------------\n");
        printk(" [ KEYBOARD MAP ] Raw Vector: 0x%04X\n", rc_sensor->info->key_v);
        printk("  W:%d S:%d A:%d D:%d | Q:%d E:%d R:%d F:%d | G:%d Z:%d X:%d C:%d\n",
               rc_sensor->info->W.value, rc_sensor->info->S.value, rc_sensor->info->A.value, rc_sensor->info->D.value,
               rc_sensor->info->Q.value, rc_sensor->info->E.value, rc_sensor->info->R.value, rc_sensor->info->F.value,
               rc_sensor->info->G.value, rc_sensor->info->Z.value, rc_sensor->info->X.value, rc_sensor->info->C.value);
        printk("  V:%d B:%d        | SHIFT:%d CTRL:%d\n",
               rc_sensor->info->V.value, rc_sensor->info->B.value, rc_sensor->info->Shift.value, rc_sensor->info->Ctrl.value);
        printk("==============================================\n");
    }
}