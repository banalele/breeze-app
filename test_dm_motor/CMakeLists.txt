cmake_minimum_required(VERSION 3.20)

if(NOT BOARD)
    set(BOARD damiao_mc02)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(infantry_dog_up)

# 直接添加所有源文件到 app
target_sources(app PRIVATE

    src/main.cpp
    src/TaskLayer/all_task.cpp
    src/TaskLayer/conf_task.cpp
    src/AlgorithmLayer/Algorithm.cpp
    src/AlgorithmLayer/algo_ekf_filter.cpp
    src/AlgorithmLayer/algo_kf_filter.cpp
    src/AlgorithmLayer/ave_filter.c
    src/AlgorithmLayer/crc.c
    src/AlgorithmLayer/rp_math.c
    src/AlgorithmLayer/pid.c
    src/DeviceLayer/bmi_stream.cpp
    src/DeviceLayer/device.cpp
    src/DeviceLayer/device_motor.c
    src/DeviceLayer/RM_motor.c
    src/DeviceLayer/DM_Motor.c
    src/DeviceLayer/KT_motor.c
    src/DeviceLayer/imu_wrapper.cpp
    src/DeviceLayer/iwdg.c
    src/DeviceLayer/rtt.c
    src/ModuleLayer/gimbal.c
    src/ModuleLayer/module.c
)
# 添加头文件路径
target_include_directories(app PRIVATE
    src/AlgorithmLayer/include      # 算法层头文件
    src/DeviceLayer                 # 设备层头文件（如 imu_ekf.hpp）
    src/TaskLayer                   # 任务层头文件（如 conf_task.hpp）
    src/ModuleLayer                 # 模块层头文件（如 Chassis_Posture.h）
    src/ConfigLayer                 # 配置层头文件（如 rp_device_config.h）
    src/ProtocolLayer               # 协议层头文件 (如 board_protocol.h)
)

