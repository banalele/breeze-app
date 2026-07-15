cmake_minimum_required(VERSION 3.20)

if(NOT BOARD)
    set(BOARD damiao_mc02)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(infantry_down_test)

# 直接添加所有源文件到 app
target_sources(app PRIVATE
    src/main.cpp
    src/DeviceLayer/bmi_stream.cpp
    src/AlgorithmLayer/Algorithm.cpp
    src/AlgorithmLayer/algo_ekf_filter.cpp
    src/AlgorithmLayer/algo_kf_filter.cpp
    src/TaskLayer/all_task.cpp
    src/TaskLayer/conf_task.cpp
    src/DeviceLayer/device.cpp
    src/DeviceLayer/device_remote.c
    src/DeviceLayer/rtt.c
    src/DeviceLayer/motor.cpp
)

# 添加头文件路径
target_include_directories(app PRIVATE
    src/AlgorithmLayer/include      # 算法层头文件
    src/DeviceLayer                 # 设备层头文件（如 imu_ekf.hpp）
    src/TaskLayer                   # 任务层头文件（如 conf_task.hpp）
)
