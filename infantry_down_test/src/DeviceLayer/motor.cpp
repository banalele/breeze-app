#include "motor.hpp"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

#if defined(__cplusplus)
extern "C" {
#endif

int Motor_Init()
{
    const struct device *motor_lf = DEVICE_DT_GET(CHASSIS_LF_NODE);
    const struct device *motor_rf = DEVICE_DT_GET(CHASSIS_RF_NODE);
    const struct device *motor_lb = DEVICE_DT_GET(CHASSIS_LB_NODE);
    const struct device *motor_rb = DEVICE_DT_GET(CHASSIS_RB_NODE);
#ifdef CONFIG_CAN_RX_MANAGER
    const struct device *rx_mgr = DEVICE_DT_GET(RX_MANAGER_NODE);
#endif

    if (!motor_lf)
    {
        LOG_ERR("motor LF not found");
        return -ENODEV;
    }
    if (!motor_rf)
    {
        LOG_ERR("motor RF not found");
        return -ENODEV;
    }
    if (!motor_lb)
    {
        LOG_ERR("motor LB not found");
        return -ENODEV;
    }
    if (!motor_rb)
    {
        LOG_ERR("motor RB not found");
        return -ENODEV;
    }

    if (!device_is_ready(motor_lf))
    {
        LOG_ERR("motor LF not ready: %s", motor_lf->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_rf))
    {
        LOG_ERR("motor RF not ready: %s", motor_rf->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_lb))
    {
        LOG_ERR("motor LB not ready: %s", motor_lb->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_rb))
    {
        LOG_ERR("motor RB not ready: %s", motor_rb->name);
        return -ENODEV;
    }

#ifdef CONFIG_CAN_RX_MANAGER
    if (!rx_mgr)
    {
        LOG_ERR("CAN RX manager not found");
        return -ENODEV;
    }
    if (!device_is_ready(rx_mgr))
    {
        LOG_ERR("CAN RX manager not ready: %s", rx_mgr->name);
        return -ENODEV;
    }
#endif

    register_motor(motor_lf);
    register_motor(motor_rf);
    register_motor(motor_lb);
    register_motor(motor_rb);

    motor_torque_control(motor_lf, 0); // 这是通用的多电机扭矩控制函数
    motor_torque_control(motor_rf, 0);
    motor_torque_control(motor_lb, 0);
    motor_torque_control(motor_rb, 0);

    return 0;
}

#if defined(__cplusplus)
}
#endif