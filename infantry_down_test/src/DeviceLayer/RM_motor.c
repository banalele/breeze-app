/**
  ******************************************************************************
  * @file    RM_motor.c
  * @brief   电机控制
  * @version 
  * @date    
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "RM_motor.h"
#include "rp_math.h"

static void Torque_to_Raw_Current(Motor_RM_t *motor);
static void Angle_Sum_Cal(Motor_RM_t *motor);
static void Encoder_to_Motor_Angle(Motor_RM_t *motor);
static float RPM_to_Rads(Motor_RM_t *motor);
static void Raw_Current_to_Torque(Motor_RM_t* motor);
static void Encoder_Sum_Cal(Motor_RM_t *motor);
/*..........................................单电机..........................................*/
/**
  * @brief          单电机控制输出转矩
  * @param[in]      Motor_RM_t *motor     电机本体
  * @retval         none
  */
static void Motor_Set_Torque(Motor_RM_t *motor)
{
	Torque_to_Raw_Current(motor);
	motor_torque_control(motor->motor, motor->tx_info->torque_current_raw);
}


/**
  * @brief          单电机卸力
  * @param[in]      Motor_RM_t *motor     电机本体
  * @retval         none
  */
static void Single_Motor_Sleep(Motor_RM_t *motor)
{
	motor->tx_info->torque = 0;
	motor_torque_control(motor->motor, motor->tx_info->torque);
}

/**
 * @brief  电机心跳失联检测
 * @param  motor: 电机结构体
 * @retval 无
 */
static void rm_motor_heart_beat(Motor_RM_t *rm_motor)
{
	Motor_RM_State_t *motor_state = rm_motor->state;
	motor_state->status = get_motor_heartbeat_status(rm_motor->motor) ? DEV_ONLINE : DEV_OFFLINE;
}

/**
 *	@brief	解析RM标准电机的角度、速度、转矩电流与温度
 */
static void rm_motor_update(Motor_RM_t *rm_motor)
{
	const smotor_receive_data_t *dev_rx_info = get_motor_rxdata(rm_motor->motor);
	Motor_RM_Rx_Info_t *motor_info = rm_motor->rx_info;

	motor_info->encoder = dev_rx_info->encoder;
	Encoder_Sum_Cal(rm_motor);
	Encoder_to_Motor_Angle(rm_motor);
	motor_info->encoder_speed = dev_rx_info->speed;
	motor_info->speed = RPM_to_Rads(rm_motor);
	motor_info->torque_current_raw = dev_rx_info->iq;
	Raw_Current_to_Torque(rm_motor);
	motor_info->temperature = dev_rx_info->specific_data.m3508.temp;
}

/**
 * @brief  电机初始化
 * @param  motor: 电机结构体
 * @retval 无
 */
void RM_Motor_Init(Motor_RM_t *motor)
{
	motor->single_set_torque = Motor_Set_Torque;
	motor->single_sleep = Single_Motor_Sleep;
	motor->rx = rm_motor_update;
	motor->single_heart_beat = rm_motor_heart_beat;
	motor->type = _3508_Reduction; // 默认类型
}





/*..........................................工具函数..........................................*/
/**
  * @brief          将电机期望输出扭矩转为原始电流数据,用于发送数据的处理
  * @param[in]      Motor_RM_t *motor     电机本体
  * @retval         none
  */
static void Torque_to_Raw_Current(Motor_RM_t *motor)
{
		switch(motor->type)
		{
			//单3508电机没有稳定的转矩常数，实际输出扭矩并不是motor->tx_info->torque
			case _3508_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -16384, 16384);//最大电流限幅
			/*单3508转矩电流转化为电流数值*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
			case _3508_Reduction:
			motor->tx_info->torque_current = motor->tx_info->torque / _3508_TORQUE_CONSTANT;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_3508_MAX_CURRENT*0.9f, _3508_MAX_CURRENT*0.9f);//最大电流限幅
			/*3508减速箱转矩电流转化为电流数值*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current / _3508_MAX_CURRENT) * 16384.f);
			break;
			case _2006_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -10000, 10000);//最大电流限幅
			/*2006转矩电流转化为电流数值*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
			case _6020_Single:
			motor->tx_info->torque_current = motor->tx_info->torque / _6020_TORQUE_CONSTANT;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_6020_MAX_CURRENT, _6020_MAX_CURRENT);//最大电流限幅
			/*6020转矩电流转化为电流数值*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
		}
		

}


/**
 *	@brief	校验RM标准电机的数据(简化为计算转过的角度)
 */
static void Encoder_Sum_Cal(Motor_RM_t *motor)
{
	int16_t err;
	Motor_RM_Rx_Info_t *motor_info = motor->rx_info;
	
	/* 未初始化 */
	if(motor_info->motor_angle_last == 0 && motor_info->encoder_sum == 0)
	{
		err = 0;
	}
	else
	{
		err = motor_info->encoder - motor_info->encoder_last;
	}
	
	/* 过零点 */
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info->encoder_sum += -8191 + err;
		/* 8191↑ -> 0 */
		else
			motor_info->encoder_sum += 8191 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info->encoder_sum += err;
	}
	
	motor_info->encoder_last = motor_info->encoder;
}

/**
 * @brief          将编码器值转化为弧度制，并计算电机角度和,分9025和8016
 * @param[in]      Motor_RM_t *motor     电机本体
 * @retval         none
 */
static void Encoder_to_Motor_Angle(Motor_RM_t *motor)
{
	if (motor->type == _3508_Reduction)
		motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _3508_REDUCT_RATIO;
	else if (motor->type == _2006_Single)
		motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _2006_REDUCT_RATIO;
	else
		motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f;

	Angle_Sum_Cal(motor);
}

/**
 * @brief          计算电机旋转角度和
 * @param[in]      Motor_RM_t *motor     电机本体
 * @retval         none
 */
static void Angle_Sum_Cal(Motor_RM_t *motor)
{
	float err = 0.f;

	if (!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum) // 上一角度值为0且角度和为零时（电机启动），不计算误差
	{
		err = 0.f;
	}
	else
	{
		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
	}

	if ((abs(err) > ((float)PI)) || (abs(err) > ((float)PI / _3508_REDUCT_RATIO) && motor->type == _3508_Reduction)) // 过零点
	{
		if (err > 0.f)
		{
			if (motor->type == _3508_Reduction)
				motor->rx_info->motor_angle_sum += (-(float)PI * 2.f / _3508_REDUCT_RATIO + err);
			else
				motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) ;
		}
		else
		{
			if (motor->type == _3508_Reduction)
				motor->rx_info->motor_angle_sum += ((float)PI * 2.f / _3508_REDUCT_RATIO + err);
			else
				motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err);
		}
	}
	else
	{
		motor->rx_info->motor_angle_sum += err;
	}

	motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/**
  * @brief          转换电机旋转速度为rad/s
  * @param[in]      int16_t rpm     r/min
  * @retval         rad/s
  */
static float RPM_to_Rads(Motor_RM_t *motor)
{
	float ret;
	if(motor->type == _3508_Reduction)
	ret= motor->rx_info->encoder_speed/60.f*2*PI/_3508_REDUCT_RATIO;
	else if(motor->type == _2006_Single)
	ret= motor->rx_info->encoder_speed/60.f*2*PI/_2006_REDUCT_RATIO;
	else
	ret= motor->rx_info->encoder_speed/60.f*2*PI;
	return ret;
}


/**
  * @brief          将电机接收原始电流数据转为实际转矩,用于接收数据的处理
  * @param[in]      Motor_RM_t *motor     电机本体
  * @retval         none(目前未完善)
  */
static void Raw_Current_to_Torque(Motor_RM_t* motor)
{
		motor->rx_info->torque_current = (motor->rx_info->torque_current_raw / 16384.f)*20.f;
		motor->rx_info->torque = motor->rx_info->torque_current * _3508_TORQUE_CONSTANT;
}
