cmake_minimum_required(VERSION 3.1)
project(sim3_ransac)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

cs_add_library(${PROJECT_NAME}
    src/sim3_match.cc
)

cs_install()
cs_export()
