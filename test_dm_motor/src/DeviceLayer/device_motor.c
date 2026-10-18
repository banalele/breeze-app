#include "device_motor.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

//DM_MOTOR-----------------------------------------------------------
//pidInfo被放在了gimbal里
Motor_DM_Rx_Info_t Pitch_Rx_Info;
Motor_DM_Tx_Info_t Pitch_Tx_Info;
Motor_DM_State_t Pitch_State;
Motor_DM_Ctrl_Info_t Pitch_Ctrl;

Motor_DM_t gimbal_motor[GIMBAL_CNT] =
{
    [PITCH] =
    {
        .rx_info = &Pitch_Rx_Info,
        .tx_info = &Pitch_Tx_Info,
        .state = &Pitch_State,
        .ctrl = &Pitch_Ctrl,
        .motor = DEVICE_DT_GET(GIMBAL_PITCH_NODE),
        .single_init = &DM_Single_Motor_Init,
        .type = dm_4310,
    },
};

//MOTOR_INIT----------------------------------------------------
#ifdef CONFIG_CAN_RX_MANAGER
/**
 * @brief DM 云台电机接收监听器：由 CAN RX 线程在驱动解析完 rx_data 后立即调用（等价于轮询里的 gimbal_motor[i].rx()）。
 */
static void gimbal_motor_rx_handler(const struct can_frame *frame, void *user_data)
{
    Motor_DM_t *motor = (Motor_DM_t *)user_data;
    (void)frame;
    if ((motor != NULL) && (motor->rx != NULL))
    {
        motor->rx(motor);
    }
}

static const struct device *gimbal_motor_get_rx_mgr(int idx)
{
    switch (idx)
    {
    case PITCH:
        return DEVICE_DT_GET(DT_PHANDLE(GIMBAL_PITCH_NODE, rx_manager));
    default:
        return NULL;
    }
}

static uint16_t gimbal_motor_get_rx_id(int idx)
{
    switch (idx)
    {
    case PITCH:
        return DT_PROP(GIMBAL_PITCH_NODE, rx_id);
    default:
        return 0;
    }
}
#endif /* CONFIG_CAN_RX_MANAGER */

int Motor_Init()
{
#ifdef CONFIG_CAN_RX_MANAGER
    const struct device *rx_mgr1 = DEVICE_DT_GET(RX_MANAGER1_NODE);
    const struct device *rx_mgr2 = DEVICE_DT_GET(RX_MANAGER2_NODE);
#endif

    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        if (!gimbal_motor[i].motor)
        {
            LOG_ERR("Motor %d not found!", i);
            return -ENODEV;
        }
        if (!device_is_ready(gimbal_motor[i].motor))
        {
            LOG_ERR("Motor %d is not ready!", i);
            return -ENODEV;
        }
        gimbal_motor[i].single_init(&gimbal_motor[i]);
    }

#ifdef CONFIG_CAN_RX_MANAGER
    if (!rx_mgr1)
    {
        LOG_ERR("CAN RX manager not found");
        return -ENODEV;
    }
    if (!device_is_ready(rx_mgr1))
    {
        LOG_ERR("CAN RX manager not ready: %s", rx_mgr1->name);
        return -ENODEV;
    }
    if (!rx_mgr2)
    {
        LOG_ERR("CAN RX manager not found");
        return -ENODEV;
    }
    if (!device_is_ready(rx_mgr2))
    {
        LOG_ERR("CAN RX manager not ready: %s", rx_mgr2->name);
        return -ENODEV;
    }
#endif



    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        if (gimbal_motor[i].motor == NULL)
        {
            continue;
        }
        motor_disable(gimbal_motor[i].motor);          // 先失能因为注册会自动发送一个扭矩
        if (register_motor(gimbal_motor[i].motor) < 0) // 这是通用的多电机注册函数
        {
            LOG_ERR("Failed to register motor %d", i);
            return -ENODEV;
        }
    }

#ifdef CONFIG_CAN_RX_MANAGER
    /* 应用层追加 DM 云台电机接收监听器：必须在驱动 register_motor() 之后注册，
     * 保证同一帧内驱动先解析 rx_data，再由本监听器调用 gimbal_motor[i].rx() */
    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        const struct device *rx_mgr = gimbal_motor_get_rx_mgr(i);
        uint16_t rx_id = gimbal_motor_get_rx_id(i);
        if ((rx_mgr == NULL) || (rx_id == 0U))
        {
            LOG_ERR("Gimbal motor %d: rx manager or rx id missing", i);
            return -ENODEV;
        }
        struct can_filter filter = {
            .id = rx_id & CAN_STD_ID_MASK,
            .mask = CAN_STD_ID_MASK,
            .flags = 0,
        };
        int ret = can_rx_manager_register(rx_mgr, &filter, gimbal_motor_rx_handler, (void *)&gimbal_motor[i]);
        if (ret < 0)
        {
            LOG_ERR("Failed to register gimbal motor %d rx listener: %d", i, ret);
            return ret;
        }
    }
#endif /* CONFIG_CAN_RX_MANAGER */

    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        gimbal_motor[i].single_sleep(&gimbal_motor[i]);
    }

    

    return 0;
}

void Motor_Heartbeat()
{
    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        gimbal_motor[i].single_heart_beat(&gimbal_motor[i]);
    }
}