/**

 * @file        DM_Motor.c

 * @author      2025_YZJ

 * @Version     V1.0

 * @date        8-Febraruary-2025

 * @brief       达秒mit控制电机包

 */



/* Includes ------------------------------------------------------------------*/

#include "DM_Motor.h"
#include "rp_math.h"


static void Motor_Limit_Target(Motor_DM_t *motor);

static void Angle_Sum_Cal(Motor_DM_t *motor);

static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word);



/*..........................................单电机..........................................*/

/**

 * @brief          单电机卸力

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Sleep(Motor_DM_t *motor)

{

	if (motor != NULL && motor->motor != NULL)

	{

		motor->tx_info->torque = 0;

		motor->tx_info->Kd = 0;

		motor->tx_info->Kp = 0;

	}

}



/**

 * @brief          单电机置零编码器

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_ZeroPosSensor(Motor_DM_t *motor)

{

	if (motor != NULL && motor->motor != NULL)

	{

		motor->single_sleep(motor); // 先对电机卸力

		dm_save_zero(motor->motor);
	}

}



/**

 * @brief          单电机控制输出转矩,含有CAN发送操作

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Set_Torque(Motor_DM_t *motor)

{

	if (motor != NULL && motor->motor != NULL)

	{
		Motor_DM_Tx_Info_t *motor_tx_info = motor->tx_info;

		if(motor->state->motor_state != Motor_Enable)
		{
			if (motor->state->motor_state == Motor_Unenable)

			{
				motor->state->motor_state = Motor_Enable;

				motor_enable(motor->motor);
				dm_mit_control(motor->motor, 0.0f, 0.0f, 0.0f, 0.0f, motor_tx_info->torque);
			}

			else
			{
				motor_clear_error(motor->motor);
			}
		}
		else
		{

			dm_mit_control(motor->motor, 0.0f, 0.0f, 0.0f, 0.0f, motor_tx_info->torque);
			//motor->tx_info->torque = 0;

		}
	}

}



/**

 * @brief          单电机控制速度,需要自行设置kd\target_speed\torque,含有CAN发送操作

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Set_Speed(Motor_DM_t *motor)

{

	if (motor != NULL && motor->motor != NULL)

	{
		Motor_DM_Tx_Info_t *motor_tx_info = motor->tx_info;

		if (motor->state->motor_state == Motor_Unenable)

		{

			motor->state->motor_state = Motor_Enable;

			motor_enable(motor->motor);
			dm_vel_control(motor->motor, motor_tx_info->target_speed);
		}

		else

		{

			Motor_Limit_Target(motor);
			
			dm_vel_control(motor->motor,motor_tx_info->target_speed);
		}

	}

}

/**
 * @brief 达妙电机位置速度控制模式
 * @param motor 电机设备 
 */
void Motor_Set_Speed_Position_DM(Motor_DM_t *motor)

{
	if (motor != NULL && motor->motor != NULL)

	{
		Motor_DM_Tx_Info_t *motor_tx_info = motor->tx_info;

		if (motor->state->motor_state == Motor_Unenable)

		{

			motor->state->motor_state = Motor_Enable;

			motor_enable(motor->motor);

			dm_posvel_control(motor->motor, motor_tx_info->target_angle, motor_tx_info->target_speed);
		}

		else

		{

			Motor_Limit_Target(motor);

			dm_posvel_control(motor->motor, motor_tx_info->target_angle, motor_tx_info->target_speed);
		}
	}
}



void DM_Single_Motor_Set(Motor_DM_t *motor)

{

	if (motor != NULL && motor->motor != NULL)

	{

		if (motor->state->motor_state == Motor_Unenable)

		{

			motor->state->motor_state = Motor_Enable;

			motor_enable(motor->motor);
		}

	}

}



/**

 * @brief          电机CAN中断接收数据处理

 * @param[in]      Motor_DM_t *motor      电机本体

 * @retval         none

 */

static void Motor_ReceiveData(Motor_DM_t *motor)

{
	if(motor->motor != NULL)
	{
		const smotor_receive_data_t *receive_info = get_motor_rxdata(motor->motor);
		const smotor_dm_rxdata_t *dm_rxdata = &receive_info->specific_data.dm;

		Motor_DM_Rx_Info_t *motor_rx_info = motor->rx_info;

		Motor_ERR_Check(motor, dm_rxdata->errState);

		motor_rx_info->motor_angle = dm_rxdata->pos_real;

		motor_rx_info->speed = dm_rxdata->vel_real;

		motor_rx_info->torque = dm_rxdata->iq_real;

		Angle_Sum_Cal(motor);

		motor_rx_info->ERR = dm_rxdata->errState;

		motor_rx_info->T_MOS = dm_rxdata->mos_temp;

		motor_rx_info->T_Rotor = dm_rxdata->motor_temp;
	}
	
}



/**

 * @brief          单电机心跳包

 * @param[in]      Motor_HT_t *motor    电机本体

 * @retval         none

 */

static void DM_Motor_Hearbeat(Motor_DM_t *motor)

{
	if (motor->motor != NULL)
	{
		Motor_DM_State_t *motor_state = motor->state;
		motor_state->status = get_motor_heartbeat_status(motor->motor) ? DEV_ONLINE : DEV_OFFLINE;
	}
}
/**

 * @brief          单电机初始化

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Init(Motor_DM_t *motor)

{

	motor->single_sleep = DM_Single_Motor_Sleep;

	motor->single_set_torque = DM_Single_Motor_Set_Torque;

	motor->single_set_speed = DM_Single_Motor_Set_Speed;

	motor->single_set_angle = Motor_Set_Speed_Position_DM;

	motor->rx = Motor_ReceiveData;

	motor->single_heart_beat = DM_Motor_Hearbeat;

	/*开启电机控制*/

	motor->state->motor_state = Motor_Unenable;

	motor->state->last_motor_state = Motor_Unenable;

	motor->rx_info->motor_angle_sum = 0;

}

/*..........................................工具函数..........................................*/

/**

 * @brief          

 * @param          Motor_DM_t *motor

 * @retval         none

 */

static void Motor_Limit_Target(Motor_DM_t *motor)

{

	Motor_DM_Tx_Info_t *motor_tx_info = motor->tx_info;

	static float pmin, pmax, vmax, vmin, cmax, cmin, tmax, tmin;

	if (motor->type == dm_6006)

	{

		pmax = P_MAX_6006;

		pmin = P_MIN_6006;

		vmax = V_MAX_6006;

		vmin = V_MIN_6006;

		cmax = C_MAX_6006;

		cmin = C_MIN_6006;

		tmax = T_MAX_6006;

		tmin = T_MIN_6006;

	}

	else if (motor->type == dm_4310)

	{

		pmax = P_MAX_4310;

		pmin = P_MIN_4310;

		vmax = V_MAX_4310;

		vmin = V_MIN_4310;

		cmax = C_MAX_4310;

		cmin = C_MIN_4310;

		tmax = T_MAX_4310;

		tmin = T_MIN_4310;

	}

	else /*(motor->type == leg_8009)*/

	{

		pmax = P_MAX_8009;

		pmin = P_MIN_8009;

		vmax = V_MAX_8009;

		vmin = V_MIN_8009;

		cmax = C_MAX_8009;

		cmin = C_MIN_8009;

		tmax = T_MAX_8009;

		tmin = T_MIN_8009;

	}

	/* 限制输入的参数在定义的范围内 */

	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, pmin, pmax);

	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, vmin, vmax);

	motor_tx_info->Kp = constrain(motor_tx_info->Kp, KP_MIN, KP_MAX);

	motor_tx_info->Kd = constrain(motor_tx_info->Kd, KD_MIN, KD_MAX);

	motor_tx_info->torque = constrain(motor_tx_info->torque, tmin, tmax);

}


/**

 * @brief          计算电机旋转角度和

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

static void Angle_Sum_Cal(Motor_DM_t *motor)

{

	float err = 0.f;

	float order_correction = 1.f;

	if (!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum) // 上一角度值为0且角度和为零时（电机启动），不计算误差

	{

		err = 0.f;

	}

	else

	{

		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;

	}



	if (fabsf(err) > PI) // 过零点

	{

		if (err > 0.f)

		{

			motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;

		}

		else

		{

			motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;

		}

	}

	else

	{

		motor->rx_info->motor_angle_sum += err * order_correction;

	}



	motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;

}



/**

 * @brief          判断电机错误码

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word)

{

	Motor_DM_State_t *my_state = motor->state;

	switch (err_word)

	{

	case 0:

		my_state->motor_state = Motor_Unenable;

		break;

	case 1:

		my_state->motor_state = Motor_Enable;

		break;

	case 8:

		my_state->motor_state = Over_Voltage;

		break;

	case 9:

		my_state->motor_state = Lack_Voltage;

		break;

	case 10:

		my_state->motor_state = Over_Current;

		break;

	case 11:

		my_state->motor_state = MOS_OverTemp;

		break;

	case 12:

		my_state->motor_state = Motor_OverTemp;

		break;

	case 13:

		my_state->motor_state = Commun_Loss;

		break;

	default:

		my_state->motor_state = Unknow_Err;

		break;

	};

	/*获取上一次不同于当前的电机状态*/

	//	if(temp_state != my_state->motor_state)

	//	{

	//		if(temp_state != my_state->last_motor_state)

	//		{

	//			my_state->last_motor_state = temp_state;

	//		}

	//	}

	//	temp_state = my_state->motor_state;

	if (my_state->motor_state != Motor_Enable &&

		my_state->motor_state != Motor_Unenable)

	{

		my_state->last_motor_state = my_state->motor_state;

	}

}

