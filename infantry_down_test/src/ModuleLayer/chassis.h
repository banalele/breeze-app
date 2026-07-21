#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "device_motor.h"

#define   CHASSIS_MAX_SPEED     60
#define   FRONT_MAX_SPEED       40
#define   LEFT_MAX_SPEED        40
#define   CYCLE_MAX_SPEED       40
#define   TURN_CYCLE_SPEED      40

#define  ROOM_ENOUGH_GIMBAL   0


typedef enum{
	SPEED_MODE,
	POSITION_MODE,
	
}Chassis_Pid_Mode_e;


typedef enum{
  C_SLEEP,
	C_INIT,
	C_BOSS,//云台跟底盘
	C_SLAVE,//底盘跟云台

}Chassis_Mode_e;


typedef struct{
	float  front_speed;
	float  left_speed;
	float  cycle_speed;
	
	float  front_location;
	float  left_location;
	float  cycle_location;
	
	float  motor_speed[WHEEL_CNT];
	float  motor_position[WHEEL_CNT];
	
}Chassis_Target_t;


typedef struct{
	
	float   front_speed;
	float   left_speed;
	float   cycle_speed;
	
	float  front_location;
	float  left_location;
	float  cycle_location;
	
}Chassis_Measure_t;



typedef struct{
  int16_t  w_s_now;// W,S键现在
	int16_t  a_d_now;// A,D键现在
	
	int16_t  w_s_last;
	int16_t  a_d_last;


}Chassis_Key_Info_t;



typedef struct{
	float  wheel_feed_out[WHEEL_CNT];// 重力前馈输出
	float  wheel_initial_out[WHEEL_CNT];//原先输出
	float  wheel_powerd_out[WHEEL_CNT];//功率限制输出
	float  wheel_end_out[WHEEL_CNT];//最终输出

}Chassis_Out_t;

/**
 * @brief  底盘运动学逆解算，车速算轮速
 * @note   
 */
typedef struct{
	bool  slip_flag;// 轮组打滑标志
	bool  is_allot;// 是否动态分配功率
	float wheel_speed_max_difference;// 轮组速度差阈值
  float slip_low_out;// 轮组打滑时的输出功率限制
}Chassis_Slip_t;


typedef struct Chassis_Struct_t{
	Motor_RM_t         *wheel[WHEEL_CNT];
	Chassis_Pid_Mode_e  pid_mode; 
	Chassis_Mode_e      mode;
  Chassis_Target_t    target;
	Chassis_Measure_t   measure;
	Chassis_Key_Info_t  key;
	Chassis_Slip_t      slip;
	float               power_coefficient[4][6];
	
  Chassis_Out_t       out;
	
	void (*init)(struct Chassis_Struct_t* chassis);
	void (*work)(struct Chassis_Struct_t* chassis);

}Chassis_t;


extern Chassis_t chassis;


#endif


