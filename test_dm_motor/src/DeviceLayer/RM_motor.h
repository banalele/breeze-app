/**
  ******************************************************************************
  * @file    RM_motor.h
  * @brief   RM电机驱动,保留rx、tx、ctrl结构体，方便chassis计算pid调用
  * 
  ******************************************************************************
  * @attention
  * 
  * 
  ******************************************************************************
  */
#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include <drivers/bldcm/bldcm.h>
#include "rp_device_config.h"
#include "pid.h"

/* Exported typedef ----------------------------------------------------------*/
#define _3508_TORQUE_CONSTANT     0.3f //3508加减速箱的扭矩常数，N*m/A
#define _2006_TORQUE_CONSTANT     0.18f //2006的扭矩常数，N*m/A
#define _3508_MAX_CURRENT         20.f    //3508输出最大电流，手册-20~20A
#define _2006_MAX_CURRENT     		10.f //2006输出最大电流，手册-10~10A

#define _6020_TORQUE_CONSTANT     1.f //6020的转速常数，rpm/V
#define _6020_MAX_CURRENT         25000.f    //3508输出最大电流，手册-20~20A

#define _3508_REDUCT_RATIO        (19.f/1.f)
#define _2006_REDUCT_RATIO        (36.f/1.f)
/*电机模式*/
typedef enum Motor_RM_Type
{
	_3508_Single,	 // 3508不加减速箱
	_3508_Reduction, // 3508加减速箱
	_6020_Single,	 // 单6020电机
	_2006_Single,	 // 单2006电机
	_2006_Reduction  // 2006减速箱
} Motor_RM_Type_e;

typedef struct Motor_RM_Rx_Info_struct_t
{
	float torque;

	float torque_current;

	int16_t torque_current_raw;

	int16_t encoder_speed; // rpm(r/min)

	float speed; // rad/s

	uint16_t encoder; // 0~8191

	int32_t encoder_sum;

	uint16_t encoder_last;

	float motor_angle;

	float motor_angle_last;
	
	float motor_angle_sum;

	int8_t temperature;
} Motor_RM_Rx_Info_t;

typedef struct Motor_RM_Ctrl_Info_struct_t
{
	bool Speed_Input_Flag;//使用外部传感器的速度标志位：0不使用，1使用
	pid_ctrl_t* angle_ctrl_inner;//角度环内环
	
	bool Angle_Input_Flag;//使用外部传感器的角度标志位：0不使用，1使用
	bool Nearest_Return;//半圈处理标志位：0不使用，1使用
	pid_ctrl_t* angle_ctrl_outer;//角度环外环
	
	pid_ctrl_t* speed_ctrl;//速度环
}Motor_RM_Ctrl_Info_t;

typedef struct Motor_RM_Tx_Info_struct_t
{
	float	torque;//需要发送的转矩
	
	float torque_current;
	
	int16_t torque_current_raw;
	
	
}Motor_RM_Tx_Info_t;

typedef struct Motor_RM_State_struct_t
{
	dev_work_state_t status;

} Motor_RM_State_t;

typedef struct Motor_RM_struct_t
{
	const struct device *motor;

	Motor_RM_Type_e type; // 电机类型

	Motor_RM_Rx_Info_t *rx_info;

	Motor_RM_Tx_Info_t *tx_info;

	Motor_RM_State_t *state;

	Motor_RM_Ctrl_Info_t *ctrl;

	void (*single_set_torque)(struct Motor_RM_struct_t *motor);

	void (*rx)(struct Motor_RM_struct_t *rm_motor);

	void (*single_sleep)(struct Motor_RM_struct_t *motor);

	void (*single_init)(struct Motor_RM_struct_t *motor);
	
	void (*single_heart_beat)(struct Motor_RM_struct_t *motor);

}Motor_RM_t;


/* Exported functions --------------------------------------------------------*/
void RM_Motor_Init(Motor_RM_t *motor);

#endif

