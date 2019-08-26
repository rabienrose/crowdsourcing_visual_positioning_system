cmake_minimum_required(VERSION 2.8)
project(display_traj)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

cs_add_executable(${PROJECT_NAME}
src/main.cc
)

cs_install()
cs_export()
