/**
 ******************************************************************************
 * File Name          : gimbal.c
 * Description        : 云台控制实现 — 纯 pitch（rm_typec 板）
 *                      仅保留 rm_typec.overlay 中定义的 gimbal_pitch(DM) 电机，
 *                      已移除 yaw 电机与升降(lift, RM)机构相关逻辑。
 ******************************************************************************
 * @attention
 * Copyright (c) 2026 HMY
 ******************************************************************************
 */

#include "gimbal.h"
#include "imu_wrapper.h"
#include "rp_math.h"
#include <math.h>

#define M_PI 3.14159265358979323846f
//#define DEBUG
/* Private function prototypes -----------------------------------------------*/
static void Gimbal_info_update(gimbal_t *gimbal);//云台信息传入
static void Gimbal_state_change(gimbal_t *gimbal);//云台状态更新(视觉or板间)
static void Gimbal_angle_protect(gimbal_t *gimbal);//云台目标角度保护并传目标值到pid
static void gravity_f_cal(gimbal_t *gimbal);//重力补偿
static void Gimbal_Pid_cal(gimbal_t *gimbal);//pid更新
static void Gimbal_protect(gimbal_t *gimbal);//云台最高优先级保护
static void Gimbal_Init_proccess(gimbal_t *gimbal);//云台到位检查
static void Gimbal_Pid_Init(gimbal_t *gimbal);//pid初始化

/*pubilic functions-------------------------------------------------------------*/

void Gimbal_Init(gimbal_t *gimbal); // 初始化实现
void Gimbal_Work(gimbal_t *gimbal); // 主处理函数实现

/* Public variables ---------------------------------------------------------*/
gimbal_t Gimbal =
    {
        .pitch_motor = &gimbal_motor[PITCH],

        .init = Gimbal_Init,
        .work = Gimbal_Work,
        .all_pid_calc = all_pid_calc,

        .ctrl_type = BOARD_CTRL,
        .gimbal_reset_state = DEV_RESET_NO, // 云台复位

        .init_info = {
            .init_time = 0,
            .init_time_max = 6000.f,
            .pitchInitAngleTolerance = 2.0f,
            .pitchInitSpeedTolerance = 30.f,
        },
};

/* 无板间/视觉通信时的 pitch 目标值（mec: rad, imu: 度），测试时修改这两个变量 */
float gimbal_pitch_mec_target = 0.f;
float gimbal_pitch_imu_target = 0.f;

/* Private member functions ---------------------------------------------------*/
    void Gimbal_Work(gimbal_t *gimbal)
    {
        /*外部信息更新*/
        Gimbal_info_update(gimbal);

	/*模式切换改变目标值来源*/
    #ifndef DEBUG
    Gimbal_state_change(gimbal);
    Gimbal_angle_protect(gimbal);
    #endif

    Gimbal_Pid_cal(gimbal);

    gimbal->pitch_motor->tx_info->torque = gimbal->base_info.output_gimbal_p;

    Gimbal_protect(gimbal);
}

/*外部数据更新*/
static void Gimbal_info_update(gimbal_t *gimbal)
{
    // DM 云台电机数据已在 CAN RX 线程中通过 rx 监听器调用 gimbal_motor[i].rx() 更新，
    // 此处不再轮询处理，避免重复累加

    // 陀螺仪数据 (imu_wrapper返回rad/s, 原始代码使用°/s, 需转换)
    gimbal->base_info.pitch_imu_angle = imu_get_pitch();
    gimbal->base_info.pitch_imu_speed = imu_get_gyro_y() * 57.2958f;   // rad/s -> °/s

    //电机数据
    gimbal->base_info.pitch_motor_angle = gimbal->pitch_motor->rx_info->motor_angle - PITCH_MOTOR_ANGLE_MIDDLE;
    gimbal->base_info.pitch_motor_angle = motor_half_cycle(gimbal->base_info.pitch_motor_angle , 2*PI);
    gimbal->base_info.pitch_mec_360_angle = gimbal->base_info.pitch_motor_angle / (2 * PI) * 360.f;
    gimbal->base_info.pitch_mec_speed = gimbal->pitch_motor->rx_info->speed;

    /* 无板间/视觉通信：目标值来自本地变量 */
    gimbal->base_info.pitch_ctrl_mec_target = gimbal_pitch_mec_target;
    gimbal->base_info.pitch_ctrl_imu_target = gimbal_pitch_imu_target;

    /* 初始化完成前保持 G_INIT，之后进入机械角度控制 */
    if(gimbal->init_info.init_flag == 0)
        gimbal->gimbal_mode = G_INIT;
    else
        gimbal->gimbal_mode = G_MEC;

    gimbal->ctrl_type = BOARD_CTRL;
}

static void Gimbal_state_change(gimbal_t *gimbal)
{
    switch (gimbal->ctrl_type)
    {
    case  BOARD_CTRL:
        if(gimbal->gimbal_mode == G_MEC || gimbal->gimbal_mode == G_INIT)
        {
            gimbal->pid_info.pitch_target_raw = gimbal->base_info.pitch_ctrl_mec_target/M_PI*180.f;
        }
        else if(gimbal->gimbal_mode == G_GYRO)
        {
            gimbal->pid_info.pitch_target_raw = gimbal->base_info.pitch_ctrl_imu_target;
        }
        break;

    //没有设定目标值（自己改）
    case SELF_DEBUG:

        break;
    default:
        break;
    }
}

/**
 * @brief  云台最高优先级保护
 * @param  gimbal：云台句柄
 */
static void Gimbal_protect(gimbal_t *gimbal)
{
    if(gimbal->gimbal_mode == G_SLEEP)//遥控器失联
    {
        //直接睡大觉
        gimbal->pitch_motor->tx_info->torque = 0;
    }
}

static float pitch ;
static void Gimbal_angle_protect(gimbal_t *gimbal)
{
    pitch = gimbal->pid_info.pitch_target_raw;

    //多圈处理
    while (fabsf(pitch) >= 180.f)
        pitch -= sgn(pitch) * 360.f;

    //最大角度限制
    if(gimbal->gimbal_mode == G_MEC || gimbal->gimbal_mode == G_INIT)
    {
        if(pitch > GIMBAL_MAX_MEC_ANGEL)
            pitch = GIMBAL_MAX_MEC_ANGEL;
        else if(pitch < GIMBAL_MIN_MEC_ANGEL)
            pitch = GIMBAL_MIN_MEC_ANGEL;
    }
    else if(gimbal->gimbal_mode == G_GYRO)
    {
        if(pitch > GIMBAL_MAX_GYRO_ANGEL)
            pitch = GIMBAL_MAX_GYRO_ANGEL;
        else if(pitch < GIMBAL_MIN_GYRO_ANGEL)
            pitch = GIMBAL_MIN_GYRO_ANGEL;
    }

   switch (gimbal->gimbal_mode)
   {
	case G_INIT:
    case G_MEC:
          gimbal->pid_info.pitch_target = pitch;
          break;
     case G_GYRO:
          gimbal->pid_info.pitch_target = pitch;
          break;
     default:
          break;
   }
}

static void Gimbal_Init_proccess(gimbal_t *gimbal)
{
	// 检测追随结果：pitch 到位即认为复位完成
    if (fabsf(gimbal->base_info.pitch_mec_360_angle - 0.f) < gimbal->init_info.pitchInitAngleTolerance
        && fabsf(gimbal->base_info.pitch_mec_speed) < gimbal->init_info.pitchInitSpeedTolerance)
	{
		gimbal->gimbal_reset_state = DEV_RESET_OK;
		gimbal->init_info.init_flag = 1;
		gimbal->init_info.init_time = 0;
	}

	//超时仅用于测试
    if(gimbal->init_info.init_time > gimbal->init_info.init_time_max)
    {
		gimbal->gimbal_reset_state = DEV_RESET_NO;
        gimbal->init_info.init_time = 0;
		gimbal->init_info.init_flag = 1;
    }
    else if (gimbal->init_info.init_flag == 0)
    {
        gimbal->init_info.init_time++;
    }
}

static void Gimbal_Pid_cal(gimbal_t *gimbal)
{
    /*云台状态机*/
    gravity_f_cal(gimbal);

    switch (gimbal->gimbal_mode)
    {
    case G_INIT:
        Gimbal_Init_proccess(gimbal);

    case G_MEC:
        gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc(&gimbal->pid_info.pitch_mec_outer,
                                                                 &gimbal->pid_info.pitch_mec_inner, gimbal->pid_info.pitch_target,
                                                                 gimbal->base_info.pitch_mec_360_angle, gimbal->base_info.pitch_mec_speed, -1, 3)
											+gimbal->base_info.gravity_f;//重力补偿
        break;
    case G_GYRO:
        // 获取目标量
        gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc(&gimbal->pid_info.pitch_gyro_outer,
                                                                 &gimbal->pid_info.pitch_gyro_inner, gimbal->pid_info.pitch_target,
                                                                 gimbal->base_info.pitch_imu_angle, gimbal->base_info.pitch_imu_speed, -1, 3);
        break;
    case G_SLEEP:
        // 真正保险是最后给卸力can信息
        gimbal->base_info.output_gimbal_p = 0.f;
        break;
    default:
		gimbal->base_info.output_gimbal_p = 0.f;
        break;
    }
}

void Gimbal_Init(gimbal_t *gimbal)
{
    Gimbal_Pid_Init(gimbal);

    gimbal->pid_info.pitch_target_raw = 0.f;

    gimbal->init_info.init_flag = 0;
}

//重力补偿计算
static void gravity_f_cal(gimbal_t *gimbal)
{
	float middle_angle = 2.678706762f - PITCH_MOTOR_ANGLE_MIDDLE;
	float k = 4.2072f;
	float b = -2.496f;

	gimbal->base_info.gravity_f = cosf(gimbal->base_info.pitch_motor_angle - middle_angle) * k + b;
}

static void Gimbal_Pid_Init(gimbal_t *gimbal)
{
    // pitch 陀螺仪外环 PID
    gimbal->pid_info.pitch_gyro_outer.kp = 56.0f;
    gimbal->pid_info.pitch_gyro_outer.ki = 1.0f;
    gimbal->pid_info.pitch_gyro_outer.kd = 0.0f;
    gimbal->pid_info.pitch_gyro_outer.integral_max = 20.0f;//500.0f;
    gimbal->pid_info.pitch_gyro_outer.out_max = 100.0f;//100.0f;

    // pitch 陀螺仪内环 PID
    gimbal->pid_info.pitch_gyro_inner.kp = 0.03f;
    gimbal->pid_info.pitch_gyro_inner.ki = 0.0f;
    gimbal->pid_info.pitch_gyro_inner.kd = 0.0f;
    gimbal->pid_info.pitch_gyro_inner.integral_max = 0.0f;
    gimbal->pid_info.pitch_gyro_inner.out_max = 10.0f;

    // pitch 机械外环 PID
    gimbal->pid_info.pitch_mec_outer.kp = 1.6f;
    gimbal->pid_info.pitch_mec_outer.ki = 0.0f;
    gimbal->pid_info.pitch_mec_outer.kd = 0.0f;
    gimbal->pid_info.pitch_mec_outer.integral_max = 500.0f;
    gimbal->pid_info.pitch_mec_outer.out_max = 10.0f;

    // pitch 机械内环 PID
    gimbal->pid_info.pitch_mec_inner.kp = 1.2f;
    gimbal->pid_info.pitch_mec_inner.ki = 0.0f;
    gimbal->pid_info.pitch_mec_inner.kd = 0.0f;
    gimbal->pid_info.pitch_mec_inner.integral_max = 0.0f;
    gimbal->pid_info.pitch_mec_inner.out_max = 10.0f;
}
