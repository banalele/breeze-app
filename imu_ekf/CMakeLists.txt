cmake_minimum_required(VERSION 3.20)

if(NOT BOARD)
    set(BOARD damiao_mc02)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(imu_ekf)

# 添加独立的 robotpilots_algorithm 库（在上层 algo 目录）
add_subdirectory(../algo ${CMAKE_CURRENT_BINARY_DIR}/algo_build)

# 直接添加所有源文件到 app
target_sources(app PRIVATE
    src/main.cpp
    src/bmi.cpp
    # src/algorithm/Algorithm.cpp
    # src/algorithm/algo_ekf_filter.cpp
    # src/algorithm/algo_kf_filter.cpp
    # src/algorithm/algo_ave_filter.cpp
)

# 添加头文件路径
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 链接算法库
target_link_libraries(app PRIVATE robotpilots_algorithm)