# Infantry_dog

## 目录结构

```
Infantry_dog_up/
├── CMakeLists.txt
├── prj.conf
├── README.md
├── boards/
│   ├── rm_typec.overlay
│   └── damiao_mc02.overlay
└── src/
    ├── main.cpp
    ├── AlgorithmLayer/
    │   ├── include/
    │   │   ├── algo_ekf_filter.hpp
    │   │   ├── algo_filter_common.hpp
    │   │   ├── algo_kf_filter.hpp
    │   │   ├── ave_filter.h
    │   │   ├── conf_algo.hpp
    │   │   ├── crc.h
    │   │   ├── pid.h
    │   │   ├── rp_math.h
    │   │   ├── rp_matrix.hpp
    │   │   └── struct_typedef.h
    │   ├── Algorithm.cpp
    │   ├── algo_ekf_filter.cpp
    │   ├── algo_kf_filter.cpp
    │   ├── ave_filter.c
    │   ├── crc.c
    │   ├── pid.c   
    │   └── rp_math.c
    ├── ConfigLayer/
    │   ├── config_chassis.h
    │   ├── rp_config.h
    │   └── rp_device_config.h
    ├── DeviceLayer/
    │   ├── RM_motor.c
    │   ├── RM_motor.h
    │   ├── bmi_stream.cpp
    │   ├── device.cpp
    │   ├── device.hpp
    │   ├── device_motor.c
    │   ├── device_motor.h
    │   ├── imu_ekf.hpp
    │   ├── imu_wrapper.cpp
    │   ├── imu_wrapper.h
    │   ├── iwdg.h
    │   ├── iwdg.c
    │   ├── rc_sensor.c
    │   ├── rc_sensor.h
    │   ├── rtt.c
    │   └── rtt.h
    ├── ModuleLayer/
    │   
    └── TaskLayer/
        ├── all_task.cpp
        ├── conf_task.cpp
        └── conf_task.hpp
```