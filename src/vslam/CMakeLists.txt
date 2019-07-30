cmake_minimum_required(VERSION 2.8)
project(vslam)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

cs_add_library(${PROJECT_NAME}
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/System.cc
src/Initializer.cc
)

cs_add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

cs_add_executable(create_vol
Examples/Monocular/create_vol.cc)
target_link_libraries(create_vol ${PROJECT_NAME})

cs_install()
cs_export()
