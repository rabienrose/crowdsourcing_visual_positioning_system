cmake_minimum_required(VERSION 3.1)
project(global_match)
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
cs_add_library(${PROJECT_NAME}
    src/global_match.cc
)

cs_install()
cs_export()

