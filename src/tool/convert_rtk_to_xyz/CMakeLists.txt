cmake_minimum_required(VERSION 3.1)
project(convert_rtk_to_xyz)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)
cs_add_executable(${PROJECT_NAME} src/main.cpp)

cs_install()
cs_export()
