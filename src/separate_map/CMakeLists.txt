cmake_minimum_required(VERSION 2.8)
project(separate_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++14 -Wno-unknown-pragmas)

set(SEPARATE_MAP
	src/separate_map.cc
	#src/separate_map_S2.cc
)

cs_add_library(separate_map ${SEPARATE_MAP})

############
## EXPORT ##
############
cs_install()
cs_export()
