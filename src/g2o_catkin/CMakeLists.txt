cmake_minimum_required(VERSION 2.8.3)
project(g2o_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(--std=c++11 -Wno-unknown-pragmas)
#############
# LIBRARIES #
#############
set(SOURCES
    src/types/types_sba.cpp
    src/types/types_six_dof_expmap.cpp
    src/types/types_seven_dof_expmap.cpp
    src/core/hyper_graph_action.cpp        
    src/core/hyper_graph.cpp        
    src/core/marginal_covariance_cholesky.cpp            
    src/core/matrix_structure.cpp
    src/core/batch_stats.cpp                         
    src/core/parameter.cpp                              
    src/core/cache.cpp                   
    src/core/optimizable_graph.cpp             
    src/core/solver.cpp                              
    src/core/optimization_algorithm_factory.cpp
    src/core/estimate_propagator.cpp           
    src/core/factory.cpp                 
    src/core/sparse_optimizer.cpp  
    src/core/hyper_dijkstra.cpp 
    src/core/parameter_container.cpp     
    src/core/optimization_algorithm.cpp 
    src/core/optimization_algorithm_with_hessian.cpp 
    src/core/optimization_algorithm_levenberg.cpp 
    src/core/jacobian_workspace.cpp 
    src/core/robust_kernel.cpp 
    src/core/robust_kernel_factory.cpp
    src/core/robust_kernel_impl.cpp 
    src/stuff/timeutil.cpp
    src/stuff/os_specific.c
    src/stuff/string_tools.cpp
    src/stuff/property.cpp       
)
include_directories(${PROJECT_SOURCE_DIR}/include/g2o/core)
include_directories(${PROJECT_SOURCE_DIR}/include/g2o/types)
include_directories(${PROJECT_SOURCE_DIR}/include/g2o/stuff)
include_directories(${PROJECT_SOURCE_DIR}/include/g2o/solvers)

cs_add_library(${PROJECT_NAME} ${SOURCES})

##########
# EXPORT #
##########
cs_install()
cs_export()
