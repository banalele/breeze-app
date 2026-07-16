breeze-app/

├── infantry_down_test/ # 步兵测试程序
├── imu-ws2812-app/ # IMU 与 WS2812 灯带联动应用
├── imu_ekf_stream/ # EKF 滤波实时数据流处理
├── imu_ekf/ # IMU 扩展卡尔曼滤波器
├── imu_damiao/ # “大喵”系列 IMU 驱动或配置
├── imu_basic/ # IMU 基础读取与校准
└── imu/ # 通用 IMU 底层驱动/接口


infantry_down_test/
└── src/
    ├── main.cpp                         # 程序入口
    ├── AlgorithmLayer/                  # 算法层
    │   ├── Algorithm.cpp                # 算法统一接口/调度
    │   ├── algo_ekf_filter.cpp          # 扩展卡尔曼滤波实现
    │   ├── algo_kf_filter.cpp           # 卡尔曼滤波实现
    │   └── include/                     # 算法层头文件
    │       ├── algo_ekf_filter.hpp
    │       ├── algo_filter_common.hpp   # 滤波通用定义
    │       ├── algo_kf_filter.hpp
    │       ├── conf_algo.hpp            # 算法配置参数
    │       └── rp_matrix.hpp            # 矩阵运算工具（可能为 Roll-Pitch 或自定义矩阵）
    ├── DeviceLayer/                     # 设备驱动层
    │   ├── device.cpp / .hpp            # 设备抽象基类/管理
    │   ├── bmi_stream.cpp               # BMI 系列 IMU 数据流处理
    │   ├── imu_ekf.hpp                  # IMU EKF 设备级封装
    │   ├── motor.cpp / .hpp             # 电机驱动
    │   ├── device_remote.c / .h         # 远程通信设备（如遥控器接收）
    │   └── rtt.c / .h                   # 实时线程（RTT）相关
    ├── ModuleLayer/                     # 模块层（功能模块）
    │   ├── chassis.cpp / .hpp           # 底盘控制模块（运动解算、轮速分配等）
    └── TaskLayer/                       # 任务调度层
        ├── all_task.cpp                 # 所有任务的总入口/初始化
        ├── conf_task.cpp / .hpp         # 任务配置（周期、优先级、堆栈等）