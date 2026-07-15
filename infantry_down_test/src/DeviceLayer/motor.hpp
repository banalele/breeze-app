#pragma once

#include <drivers/motor.h>

#ifdef CONFIG_CAN_TX_MANAGER
#include <drivers/can_tx_manager.h>
#endif
#ifdef CONFIG_CAN_RX_MANAGER
#include <drivers/can_rx_manager.h>
#endif


#define MOTOR_3508_CURRENT_MAX 10000

#define CHASSIS_LF_NODE DT_NODELABEL(chassis_lf)
#define CHASSIS_LB_NODE DT_NODELABEL(chassis_lb)
#define CHASSIS_RF_NODE DT_NODELABEL(chassis_rf)
#define CHASSIS_RB_NODE DT_NODELABEL(chassis_rb)

#ifdef CONFIG_CAN_RX_MANAGER
#define RX_MANAGER_NODE DT_NODELABEL(can_rx_mgr1)
#endif

#ifdef __cplusplus
extern "C"
{
#endif
int Motor_Init(void);
#ifdef __cplusplus
}
#endif  
