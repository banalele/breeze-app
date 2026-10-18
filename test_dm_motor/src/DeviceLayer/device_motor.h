#pragma once

#include "RM_motor.h"
#include "DM_Motor.h"
#include "KT_motor.h"

#ifdef CONFIG_CAN_TX_MANAGER
#include <drivers/can_tx_manager.h>
#endif
#ifdef CONFIG_CAN_RX_MANAGER
#include <drivers/can_rx_manager.h>
#endif


#define GIMBAL_PITCH_NODE DT_NODELABEL(gimbal_pitch)

#ifdef CONFIG_CAN_RX_MANAGER
#define RX_MANAGER1_NODE DT_NODELABEL(can_rx_mgr1)
#define RX_MANAGER2_NODE DT_NODELABEL(can_rx_mgr2)
#endif

typedef enum
{
    PITCH = 0,
    GIMBAL_CNT,
}Gimbal_List_e;

#ifdef __cplusplus
extern "C"
{
#endif
    extern Motor_DM_t gimbal_motor[GIMBAL_CNT];

    int Motor_Init(void);
    void Motor_Heartbeat(void);
#ifdef __cplusplus
}
#endif