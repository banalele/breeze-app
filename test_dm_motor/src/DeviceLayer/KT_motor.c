#include "KT_motor.h"

/* Private functions ---------------------------------------------------------*/
#define OFFLINE_LINE_CNT_MAX 100
#define KT_TX_IQ_CONTROL_MAX 2000		   //-2048~2048   ← 被 Motor_Set_Torque 使用


void KT_motor_class_heartbeat(KT_motor_t *motor);

void get_kt_motor_info(KT_motor_t *motor);

static void KT_Encoder_Sum_Cal(KT_motor_t *motor);

void Motor_Set_Torque(KT_motor_t *motor);

/**

 *	@brief	电机初始化，负责一些参数赋值、函数指针赋值、清零发送数组、清零发送的结构体参数

 */

void KT_motor_class_init(KT_motor_t *motor)
{
	if (motor == NULL)

		return;

	memset((uint8_t *)motor->tx_buff, 0, 8);

	memset(&motor->KT_motor_info.tx_info, 0, 8);

	motor->KT_motor_info.state_info.offline_cnt_max = OFFLINE_LINE_CNT_MAX;
	motor->KT_motor_info.state_info.offline_cnt = 0;
	motor->KT_motor_info.state_info.work_state = M_OFFLINE;
	motor->heartbeat = KT_motor_class_heartbeat;
	motor->get_info = get_kt_motor_info;
	motor->single_set_torque = Motor_Set_Torque;
}

/**

 *	@brief	电机心跳，如果失联，下一次收到CAN数据时offline_cnt会被get_info清零

 *	        注：selfprotect_cnt/flag逻辑虽在旧代码中实现，但上层从未读取，已注释

 */

void KT_motor_class_heartbeat(KT_motor_t *motor)

{
	if (motor == NULL)
		return;

	KT_motor_state_info_t *state_info = &motor->KT_motor_info.state_info;
    state_info->work_state = get_motor_heartbeat_status(motor->lk_motor) ? M_ONLINE : M_OFFLINE;
}

/*---------------------------以下函数都是针对单电机控制----------------------------*/

/**

 *	@brief 给电机发送有关  写参数或者想要的控制模式的命令，内部已经有can的发送函数

 */

void Motor_Set_Torque(KT_motor_t *motor)
{
	if (motor == NULL)
		return;


	int16_t iqControl = constrain(motor->KT_motor_info.tx_info.iqControl, -KT_TX_IQ_CONTROL_MAX, KT_TX_IQ_CONTROL_MAX);

	motor->KT_motor_info.tx_info.iqControl = iqControl;
	/* 单电机模式(tx-id 0x141)：必须用 0xA1 单电机闭环转矩命令；
	 * motor_torque_control() 对应的是多电机模式(tx-id 0x280)，这里不能用 */
	single_closedloop_control(motor->lk_motor, iqControl);
}

/**
 *  @brief  计算KT电机多圈角度和，处理16位编码器(0-65535)的过零点
 *  @param  motor: KT电机结构体
 *  @retval none
 */
static void KT_Encoder_Sum_Cal(KT_motor_t *motor)
{
	int32_t err;
	KT_motor_rx_info_t *rx_info = &motor->KT_motor_info.rx_info;

	/* 未初始化：last_encoder为0且encoder_sum为0时，不计算误差 */
	if (rx_info->last_encoder == 0 && rx_info->encoder_sum == 0)
	{
		err = 0;
	}
	else
	{
		err = (int32_t)rx_info->encoder - (int32_t)rx_info->last_encoder;
	}

	/* 过零点判断：16位编码器半量程为32767 */
	if (err > 32767 || err < -32767)
	{
		/* 65535↓ -> 0 （编码器值从大向小跳变，如60000->2000） */
		if (err >= 0)
			rx_info->encoder_sum += (uint32_t)(-65535 + err);
		/* 0↑ -> 65535 （编码器值从小向大跳变，如2000->60000） */
		else
			rx_info->encoder_sum += (uint32_t)(65535 + err);
	}
	/* 未过零点 */
	else
	{
		rx_info->encoder_sum += (uint32_t)err;
	}

	rx_info->last_encoder = rx_info->encoder;
}

/**

 *	@brief	接收电机发来的信息并自动更新，需要注意，大部分发送给电机的指令，电机也会返回一些数据
						如果传入空指针，认为电机数据出错，并返回
						最后面会根据接收结构体的errorState判断是否要自我保护
 *  @return
 */
void get_kt_motor_info(KT_motor_t *motor)
{
	if (motor == NULL)
		return;

	KT_motor_rx_info_t *rx_info = &motor->KT_motor_info.rx_info;
	KT_motor_state_info_t *state_info = &motor->KT_motor_info.state_info;

	/* CAN 接收由 LK 驱动的 RX Manager 自动处理，这里只同步解析后的数据 */
	const smotor_receive_data_t *dev_rx = get_motor_rxdata(motor->lk_motor);
	if (dev_rx == NULL) {
		state_info->work_state = M_DATA_ERR;
		return;
	}
		state_info->work_state = M_ONLINE;

		/* 通用数据 */
		rx_info->temperature = dev_rx->specific_data.lk.temp;
		rx_info->voltage     = dev_rx->specific_data.lk.vol;
		rx_info->errorState  = dev_rx->specific_data.lk.errorState;
		rx_info->current     = dev_rx->iq;
		rx_info->speed       = dev_rx->speed;
		rx_info->encoder     = (uint16_t)dev_rx->encoder;
		rx_info->current_A   = dev_rx->specific_data.lk.iA;
		rx_info->current_B   = dev_rx->specific_data.lk.iB;
		rx_info->current_C   = dev_rx->specific_data.lk.iC;
		rx_info->powerControl = dev_rx->specific_data.lk.power;

		/* 单电机特有数据 */
		const motor_lk_single_data_t *single = get_single_data(motor->lk_motor);
		if (single != NULL) {
			rx_info->motorAngle    = single->motorAngle;
			rx_info->circleAngle   = single->circleAngle;
			rx_info->encoderRaw    = single->encoder_data.encoderRaw;
			rx_info->encoderOffset = single->encoder_data.encoderOffset;
		}

	KT_Encoder_Sum_Cal(motor);
}


