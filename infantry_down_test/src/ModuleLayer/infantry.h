#ifndef __INFANTRY_H
#define __INFANTRY_H

#include <stdbool.h>
#include <stdint.h>
#include "rc_sensor.h"

#define  WHEEL_UP_TO_ONCE        rc_info->thumbwheel.step[0] != last_thumbwheel_step[0] || rc_info->thumbwheel.step[1] != last_thumbwheel_step[1]    
#define  WHEEL_DOWN_TO_ONCE        rc_info->thumbwheel.step[2] != last_thumbwheel_step[2] || rc_info->thumbwheel.step[3] != last_thumbwheel_step[3]   


typedef enum{
	RC_CTRL,
	KEY_CTRL,

}Infantry_Ctrl_e;

typedef enum{
	I_SLEEP,
	I_INIT,
	I_MEC,
	I_IMU,
	I_TURN,//小陀螺
  I_HOLE,
}Infantry_Mode_e;




typedef enum{
	LOWING,
	RISING,
	HIGHING,
	FALLING,
	
}Signal_Form_e;

typedef struct{
	bool  value;
	bool  last_value;
	Signal_Form_e    form;
	uint16_t  tick;
	uint16_t  tick_max;

}Flag_Class_t;




typedef struct{
	Flag_Class_t  U_turn_flag;
	Flag_Class_t  L_turn_flag;
	Flag_Class_t  R_turn_flag;
//	Flag_Class_t    hole_flag;
	Flag_Class_t    chassis_reset;	
	
	bool    mec_flag;
	bool    imu_flag;
    bool    turn_flag;
	bool    hole_flag;
	uint8_t vision_flag;
	bool    broken_flag;

//  bool    U_turn_flag;
//	bool    L_turn_flag;
//	bool    R_turn_flag;
	
	bool    chassis_off;
	bool    gimbal_off;
	
//	bool    chassis_reset;
	bool    car_reset;
	
}Infantry_Flag_t;




typedef struct Infantry_Struct_t{
	Infantry_Ctrl_e          ctrl;
    Infantry_Mode_e          mode;
	Infantry_Mode_e          last_mode;
	Infantry_Flag_t          flag;

	void (* init)(struct Infantry_Struct_t* infantry);
    void (* work)(struct Infantry_Struct_t* infantry);

}Infantry_t;




extern Infantry_t  infantry;


#endif