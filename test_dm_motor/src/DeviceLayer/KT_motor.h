#pragma once

#include <drivers/bldcm/bldcm_lk.h>
#include "rp_device_config.h"
#include "rp_math.h"
#include "pid.h"

#define ID_DIAL 0x141

typedef enum motor_state_e
{
	M_OFFLINE = 0,
	M_ONLINE,
	M_DATA_ERR,
}motor_state_e;

typedef enum motor_drive_e
{
	M_CAN1,
	M_CAN2,
}motor_drive_e;

typedef enum lk_motor_type_e
{
	KT9015 = 4,
	KT9025,
	KT4005,
}lk_motor_type_e;

// 填充力矩变量：kt_motor[0].W_iqControl(&kt_motor[0],kt_motor[0].base_info.motor_out);
// CAN发送：kt_motor[0].tx_W_cmd(&kt_motor[0],TORQUE_CLOSE_LOOP_ID);
typedef enum motor_kt9025_command_e
{
	PID_RX_ID = 0x30,
	PID_TX_RAM_ID = 0x31, // 断电失效
	PID_TX_ROM_ID = 0x32, // 断电有效
	ACCEL_RX_ID = 0x33,
	ACCEL_TX_ID = 0x34,
	ENCODER_RX_ID = 0x90,
	ZERO_ENCODER_TX_ID = 0x91,
	ZERO_POSNOW_TX_ID = 0x19,
	MOTOR_ANGLE_ID = 0x92,
	CIRCLE_ANGLE_ID = 0x94,
	STATE1_ID = 0x9A,
	CLEAR_ERROR_State_ID = 0x9B,
	STATE2_ID = 0x9C,
	STATE3_ID = 0x9D,
	MOTOR_CLOSE_ID = 0x80,
	MOTOR_STOP_ID = 0x81,
	MOTOR_RUN_ID = 0x88,
	TORQUE_OPEN_LOOP_ID = 0xA0,
	TORQUE_CLOSE_LOOP_ID = 0xA1,
	SPEED_CLOSE_LOOP_ID = 0XA2,
	POSI_CLOSE_LOOP_ID1 = 0XA3,
	POSI_CLOSE_LOOP_ID2 = 0XA4,
	POSI_CLOSE_LOOP_ID3 = 0XA5,
	POSI_CLOSE_LOOP_ID4 = 0XA6,
	POSI_CLOSE_LOOP_ID5 = 0XA7,
	POSI_CLOSE_LOOP_ID6 = 0XA8,

} motor_kt9025_command_e;

typedef struct KT_motor_rx_info_t
{
	int32_t accel;			// 加速度 1dps/s
	uint16_t encoder;		// 编码器位置             （0~16383 * 4）
	uint32_t encoder_sum;	// 角度和
	uint32_t last_encoder;	// 上一循环编码器位置，用于计算多圈角度
	uint16_t encoderRaw;	// 编码器原始位置         （0~16383 * 4）
	uint16_t encoderOffset; // 编码器零偏             （0~16383 * 4）
	int8_t temperature;		// 温度    1°C/LSB
	uint16_t voltage;		// 电压    1V
	uint8_t errorState;		// 高四位无效    低四位0xx0都正常；0xx1温度正常+低压保护；1xx0过温保护+电压正常；1001过温保护+低压保护
	int16_t current;		// 转矩电流    返回的值无单位说明   -2048 ~ 2048  （-33~33A）
	int16_t speed;			// 电机转速    1dps/LSB
	int16_t current_A;		// ABC三相电流数据     1A/64LSB
	int16_t current_B;
	int16_t current_C;
	int16_t powerControl; // 输出功率（-1000~1000）  无单位说明
	int64_t motorAngle;	  // 电机多圈角度   0.01°/LSB  （-2^63~2^63）    顺时针增加，逆时针减少
	uint32_t circleAngle; // 电机单圈角度   0.01°/LSB  （ 0~36000 * 减速比-1）
						  // 以编码零点作为起始点，顺时针增加，再次到达零点时数值变为0
	int16_t angle_add;	  //-4096~4096
} KT_motor_rx_info_t;

typedef struct KT_motor_tx_info_t
{
	int32_t accel;								// 加速度     1dps/s
	uint16_t encoderOffset;						// 编码器零偏（0~16383*4）
	int16_t powerControl;						// 输出功率 （-1000~1000）     不受上位机的Max Power限制
	int16_t iqControl;							// 扭矩电流 （-2000~2000，对应的实际扭矩电流-32A~32A） 不受上位机限制
	int32_t speedControl;						// 实际转速为0.01dps/LSB       最大值受上位机设置
	int32_t angle_sum_Control;					// 多圈角度，  0.01degree/LSB     36000代表360°      最大值受上位机设置
	uint16_t angle_sum_Control_maxSpeed;		// 多圈角度的最大速度   1dps/LSB   最大值受上位机设置
	uint16_t angle_single_Control;				// 单圈角度    0.01degree/LSB       0~35999对应实际角度0~359.99°   最大值受上位机设置
	uint16_t angle_single_Control_maxSpeed;		// 单圈角度的最大速度   1dps/LSB   最大值受上位机设置
	uint8_t angle_single_Control_spinDirection; // 单圈角度旋转方向，0x00顺时针，0x01逆时针
	int32_t angle_add_Control;					// 角度位置增量，转动方向由控制量符号决定   0.01degree/LSB     最大值受上位机设置
	uint16_t angle_add_Control_maxSpeed;		// 角度位置增量的最大速度  1dps/LSB   最大值受上位机设置

} KT_motor_tx_info_t;

typedef struct KT_motor_id_info_t
{
	uint32_t tx_id; // 发送id
	uint32_t rx_id; // 接收id

	lk_motor_type_e motor_type;

} KT_motor_id_info_t;

typedef struct KT_motor_state_info_t
{
	uint8_t offline_cnt_max;
	uint8_t offline_cnt;
	motor_state_e work_state;

} KT_motor_state_info_t;

typedef struct KT_motor_info_t
{
	KT_motor_rx_info_t rx_info;
	KT_motor_tx_info_t tx_info;
	KT_motor_id_info_t id;
	KT_motor_state_info_t state_info;

} KT_motor_info_t;

typedef struct KT_motor_class_t
{
	const struct device *lk_motor;

	KT_motor_info_t KT_motor_info;
	uint8_t tx_buff[8];
	void (*init)(struct KT_motor_class_t *motor);
	void (*heartbeat)(struct KT_motor_class_t *motor);

	// W表示写，R表示读，cmd是命令

	void (*get_info)(struct KT_motor_class_t *motor);
	void (*tx_W_cmd)(struct KT_motor_class_t *motor, uint8_t command); // 发送写命令
	void (*W_iqControl)(struct KT_motor_class_t *motor, int16_t iqControl);

} KT_motor_t;

void KT_motor_class_init(KT_motor_t *motor);
