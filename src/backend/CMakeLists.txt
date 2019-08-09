cmake_minimum_required(VERSION 3.1)
project(backend)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

cs_add_executable(${PROJECT_NAME}
    src/main.cc
    src/find_correspond.cc
    src/pose_graph_opti.cc
    src/ba.cc
    src/culling_frame.cc
    src/map_tool.cc
)

cs_install()
cs_export()
