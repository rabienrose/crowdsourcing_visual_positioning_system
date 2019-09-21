cmake_minimum_required(VERSION 3.1)
project(global_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++14 -Wno-unknown-pragmas -w)

cs_add_library(${PROJECT_NAME} ${PROTO_SRCS}
    src/global_map.cc
    src/frame.cc
    src/map_point.cc
    src/global_map_seri.cc
)

cs_add_executable(${PROJECT_NAME}_test
	src/main.cc
)

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
