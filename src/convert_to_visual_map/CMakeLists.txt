cmake_minimum_required(VERSION 3.1)
project(convert_to_visual_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)

cs_add_library(${PROJECT_NAME}_lib
    src/convert_to_visual_map.cc
)

cs_add_executable(${PROJECT_NAME}
    src/main.cc
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

cs_install()
cs_export()
