cmake_minimum_required(VERSION 2.8.3)
project(show_pc)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#######
# APP #
#######
cs_add_executable(${PROJECT_NAME}
  src/main.cc
)
############
## EXPORT ##
############
cs_install()
cs_export()
