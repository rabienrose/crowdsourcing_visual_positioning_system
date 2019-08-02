cmake_minimum_required(VERSION 2.8)
project(inverted_multi_index)
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)
find_package(catkin_simple REQUIRED)
catkin_simple()

set(LIBRARY_NAME ${PROJECT_NAME})

cs_add_library(${LIBRARY_NAME} src/inverted-multi-index.cc)

cs_install()
cs_export()
