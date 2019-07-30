cmake_minimum_required(VERSION 2.8)
project(dbow2)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp   
  DBoW2/FFREAK.cpp  
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

cs_add_library(dbow2 ${SRCS_DBOW2} ${SRCS_DUTILS})

############
## EXPORT ##
############
cs_install()
cs_export()
