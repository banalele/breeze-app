#include "device_motor.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

//DM_MOTOR-----------------------------------------------------------
//pidInfo被放在了gimbal里
Motor_DM_Rx_Info_t Pitch_Rx_Info;
Motor_DM_Tx_Info_t Pitch_Tx_Info;
Motor_DM_State_t Pitch_State;
Motor_DM_Ctrl_Info_t Pitch_Ctrl;
Motor_RM_Rx_Info_t Chassis_Rx_Info[RMMOTOR_CNT];
Motor_RM_Tx_Info_t Chassis_Tx_Info[RMMOTOR_CNT];
Motor_RM_State_t Chassis_State[RMMOTOR_CNT];
Motor_RM_Ctrl_Info_t Chassis_Ctrl[] = {
    [CHASSIS_LF] =
        {
            .angle_ctrl_inner = NULL,
            .angle_ctrl_outer = NULL,
            .speed_ctrl = NULL,
        },
    [CHASSIS_LB] =
        {
            .angle_ctrl_inner = NULL,
            .angle_ctrl_outer = NULL,
            .speed_ctrl = NULL,
        },
    [CHASSIS_RF] =
        {
            .angle_ctrl_inner = NULL,
            .angle_ctrl_outer = NULL,
            .speed_ctrl = NULL,
        },
    [CHASSIS_RB] =
        {
            .angle_ctrl_inner = NULL,
            .angle_ctrl_outer = NULL,
            .speed_ctrl = NULL,
        },
};


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

Motor_RM_t chassis_motor[RMMOTOR_CNT] =
{
    [CHASSIS_LF] =
    {
        .rx_info = &Chassis_Rx_Info[CHASSIS_LF],
        .tx_info = &Chassis_Tx_Info[CHASSIS_LF],
        .state = &Chassis_State[CHASSIS_LF],
        .ctrl = &Chassis_Ctrl[CHASSIS_LF],
        .motor = DEVICE_DT_GET(CHASSIS_LF_NODE),
        .type = _3508_Single,
        .single_init = &RM_Motor_Init,
    },
    [CHASSIS_RF] =
    {
        .rx_info = &Chassis_Rx_Info[CHASSIS_RF],
        .tx_info = &Chassis_Tx_Info[CHASSIS_RF],
        .state = &Chassis_State[CHASSIS_RF],
        .ctrl = &Chassis_Ctrl[CHASSIS_RF],
        .motor = DEVICE_DT_GET(CHASSIS_RF_NODE),
        .type = _3508_Single,
        .single_init = &RM_Motor_Init,
    },
    [CHASSIS_LB] =
    {
        .rx_info = &Chassis_Rx_Info[CHASSIS_LB],
        .tx_info = &Chassis_Tx_Info[CHASSIS_LB],
        .state = &Chassis_State[CHASSIS_LB],
        .ctrl = &Chassis_Ctrl[CHASSIS_LB],
        .motor = DEVICE_DT_GET(CHASSIS_LB_NODE),
        .type = _3508_Single,
        .single_init = &RM_Motor_Init,
    },
    [CHASSIS_RB] =
    {
        .rx_info = &Chassis_Rx_Info[CHASSIS_RB],
        .tx_info = &Chassis_Tx_Info[CHASSIS_RB],
        .state = &Chassis_State[CHASSIS_RB],
        .ctrl = &Chassis_Ctrl[CHASSIS_RB],
        .motor = DEVICE_DT_GET(CHASSIS_RB_NODE),
        .type = _3508_Single,
        .single_init = &RM_Motor_Init,
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

/**
 * @brief RM 底盘电机接收监听器：由 CAN RX 线程在驱动解析完 rx_data 后立即调用（等价于轮询里的 chassis_motor[i].rx()）。
 */
static void chassis_motor_rx_handler(const struct can_frame *frame, void *user_data)
{
    Motor_RM_t *motor = (Motor_RM_t *)user_data;
    (void)frame;
    if ((motor != NULL) && (motor->rx != NULL))
    {
        motor->rx(motor);
    }
}

static const struct device *chassis_motor_get_rx_mgr(int idx)
{
    switch (idx)
    {
    case CHASSIS_LF:
        return DEVICE_DT_GET(DT_PHANDLE(CHASSIS_LF_NODE, rx_manager));
    case CHASSIS_LB:
        return DEVICE_DT_GET(DT_PHANDLE(CHASSIS_LB_NODE, rx_manager));
    case CHASSIS_RF:
        return DEVICE_DT_GET(DT_PHANDLE(CHASSIS_RF_NODE, rx_manager));
    case CHASSIS_RB:
        return DEVICE_DT_GET(DT_PHANDLE(CHASSIS_RB_NODE, rx_manager));
    default:
        return NULL;
    }
}

static uint16_t chassis_motor_get_rx_id(int idx)
{
    switch (idx)
    {
    case CHASSIS_LF:
        return DT_PROP(CHASSIS_LF_NODE, rx_id);
    case CHASSIS_LB:
        return DT_PROP(CHASSIS_LB_NODE, rx_id);
    case CHASSIS_RF:
        return DT_PROP(CHASSIS_RF_NODE, rx_id);
    case CHASSIS_RB:
        return DT_PROP(CHASSIS_RB_NODE, rx_id);
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
    for (int i = 0; i < RMMOTOR_CNT; i++)
    {
        if (!chassis_motor[i].motor)
        {
            LOG_ERR("Chassis motor %d not found!", i);
            return -ENODEV;
        }
        if (!device_is_ready(chassis_motor[i].motor))
        {
            LOG_ERR("Chassis motor %d is not ready!", i);
            return -ENODEV;
        }
        chassis_motor[i].single_init(&chassis_motor[i]);
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

    for (int i = 0; i < RMMOTOR_CNT; i++)
    {
        if (chassis_motor[i].motor == NULL)
        {
            continue;
        }
        if (register_motor(chassis_motor[i].motor) < 0) // 这是通用的多电机注册函数
        {
            LOG_ERR("Failed to register chassis motor %d", i);
            return -ENODEV;
        }
    }
#ifdef CONFIG_CAN_RX_MANAGER
    /* 应用层追加电机接收监听器（DM 云台 + RM 底盘）：必须在驱动 register_motor() 之后注册，
     * 保证同一帧内驱动先解析 rx_data，再由本监听器调用 xxx_motor[i].rx() */
    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        const struct device *rx_mgr = gimbal_motor_get_rx_mgr(i);
        uint16_t rx_id = gimbal_motor_get_rx_id(i);
        if ((rx_mgr == NULL))
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

    for (int i = 0; i < RMMOTOR_CNT; i++)
    {
        const struct device *rx_mgr = chassis_motor_get_rx_mgr(i);
        uint16_t rx_id = chassis_motor_get_rx_id(i);
        if ((rx_mgr == NULL) || (rx_id == 0))
        {
            LOG_ERR("Chassis motor %d: rx manager or rx id missing", i);
            return -ENODEV;
        }
        struct can_filter filter = {
            .id = rx_id & CAN_STD_ID_MASK,
            .mask = CAN_STD_ID_MASK,
            .flags = 0,
        };
        int ret = can_rx_manager_register(rx_mgr, &filter, chassis_motor_rx_handler, (void *)&chassis_motor[i]);
        if (ret < 0)
        {
            LOG_ERR("Failed to register chassis motor %d rx listener: %d", i, ret);
            return ret;
        }
    }
#endif /* CONFIG_CAN_RX_MANAGER */

    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        gimbal_motor[i].single_sleep(&gimbal_motor[i]);
    }

    for(int i = 0; i < RMMOTOR_CNT; i++)
    {
        chassis_motor[i].single_sleep(&chassis_motor[i]);
    }

    return 0;
}

void Motor_Heartbeat()
{
    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        gimbal_motor[i].single_heart_beat(&gimbal_motor[i]);
    }
    for (int i = 0; i < RMMOTOR_CNT; i++)
    {
        chassis_motor[i].single_heart_beat(&chassis_motor[i]);
    }
}