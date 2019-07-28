cmake_minimum_required(VERSION 3.1)
project(build_all)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export()
