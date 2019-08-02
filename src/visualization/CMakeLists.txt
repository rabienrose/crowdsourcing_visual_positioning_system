cmake_minimum_required (VERSION 2.8)
project(visualization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

##########################
# EXECUTABLE AND LIBRARY #
##########################
cs_add_library(${PROJECT_NAME}
  src/common-rviz-visualization.cc
  src/rviz-visualization-sink.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
