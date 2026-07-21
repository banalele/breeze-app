#pragma once
#include "rc_sensor.h"
#include "device_motor.h"
#include "imu_ekf.hpp"
#include "rtt.h"
#include "infantry.h"
#include "chassis.h"
#include "iwdg.h"

int Device_Init(void);

extern const struct device *iwdg_dev;  /* 外部只读，初始化后在 device.cpp 中赋值 */