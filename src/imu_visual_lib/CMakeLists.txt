cmake_minimum_required(VERSION 2.8)
project(imu_visual_lib)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)


cs_add_library(${PROJECT_NAME} SHARED
src/configparam.cpp
src/g2otypes.cpp
src/imudata.cpp
src/IMUPreintegrator.cpp
src/NavState.cpp
src/so3.cpp
src/imu_tools.cpp
src/IMUConverter.cc
)

cs_install()
cs_export()
