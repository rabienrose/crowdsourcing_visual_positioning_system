cmake_minimum_required(VERSION 2.8)
project(sfm)
option(CUDA_ENABLED "Whether to enable CUDA, if available" ON)
set(CUDA_ARCHS "Auto" CACHE STRING "List of CUDA architectures for which to generate code, e.g., Auto, All, Maxwell, Pascal, ...")
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CUDA_MIN_VERSION "7.0")
if(CUDA_ENABLED)
    find_package(CUDA ${CUDA_MIN_VERSION} QUIET)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(FreeImage REQUIRED)
if(CUDA_FOUND)
    if(CUDA_ENABLED)
        add_definitions("-DCUDA_ENABLED")

        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/SelectCudaComputeArch.cmake)

        CUDA_SELECT_NVCC_ARCH_FLAGS(CUDA_ARCH_FLAGS ${CUDA_ARCHS})

        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH_FLAGS}")

        # Fix for some combinations of CUDA and GCC (e.g. under Ubuntu 16.04).
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -D_FORCE_INLINES")
        # Do not show warnings if the architectures are deprecated.
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Wno-deprecated-gpu-targets" -std=c++11)

        message(STATUS "Enabling CUDA support (version: ${CUDA_VERSION_STRING},"
                       " archs: ${CUDA_ARCH_FLAGS_readable})")
    else()
        set(CUDA_FOUND OFF)
        message(STATUS "Disabling CUDA support")
    endif()
else()
    set(CUDA_ENABLED OFF)
    if(CUDA_VERSION_STRING)
        message(STATUS "Disabling CUDA support (found version "
                "${CUDA_VERSION_STRING} but >= ${CUDA_MIN_VERSION} required)")
    else()
        message(STATUS "Disabling CUDA support")
    endif()
endif()
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)

set(CPP_SRC 
  feature/types.cc
  feature/utils.cc
  feature/extraction.cc
  feature/matching.cc
  feature/sift.cc
  estimators/absolute_pose.cc
  estimators/affine_transform.cc
  estimators/coordinate_frame.cc
  estimators/essential_matrix.cc
  estimators/fundamental_matrix.cc
  estimators/generalized_absolute_pose.cc
  estimators/generalized_absolute_pose_coeffs.cc
  estimators/generalized_relative_pose.cc
  estimators/homography_matrix.cc
  estimators/pose.cc
  estimators/triangulation.cc
  estimators/two_view_geometry.cc
  estimators/utils.cc
  base/camera.cc
  base/camera_database.cc
  base/camera_models.cc
  base/camera_rig.cc
  base/correspondence_graph.cc
  base/database.cc
  base/database_cache.cc
  base/essential_matrix.cc
  base/gps.cc
  base/graph_cut.cc
  base/homography_matrix.cc
  base/image.cc
  base/image_reader.cc
  base/line.cc
  base/point2d.cc
  base/point3d.cc
  base/polynomial.cc
  base/pose.cc
  base/projection.cc
  base/reconstruction.cc
  base/reconstruction_manager.cc
  base/scene_clustering.cc
  base/similarity_transform.cc
  base/track.cc
  base/triangulation.cc
  base/undistortion.cc
  base/visibility_pyramid.cc
  base/warp.cc
  util/bitmap.cc
  util/camera_specs.cc
  util/cuda.cc
  util/cudacc.cc
  util/logging.cc
  util/math.cc
  util/misc.cc
  util/option_manager.cc
  util/ply.cc
  util/random.cc
  util/string.cc
  util/threading.cc
  util/timer.cc
  util/version.cc
  optim/bundle_adjustment.cc
  optim/combination_sampler.cc
  optim/least_absolute_deviations.cc
  optim/progressive_sampler.cc
  optim/random_sampler.cc
  optim/sprt.cc
  optim/support_measurement.cc
  mvs/consistency_graph.cc
  mvs/depth_map.cc
  mvs/fusion.cc
  mvs/image.cc
  mvs/meshing.cc
  mvs/model.cc
  mvs/normal_map.cc
  mvs/workspace.cc
  sfm/incremental_mapper.cc
  sfm/incremental_triangulator.cc
  controllers/automatic_reconstruction.cc
  controllers/bundle_adjustment.cc
  controllers/hierarchical_mapper.cc
  controllers/incremental_mapper.cc
  retrieval/vote_and_verify.cc
  retrieval/geometry.cc
)

set(CUDA_SRC
  mvs/gpu_mat_prng.cu
  mvs/gpu_mat_ref_image.cu
  mvs/patch_match.cc
  mvs/patch_match_cuda.cu 
  
)

if(CUDA_ENABLED)
  cuda_add_library(${PROJECT_NAME}
    ${CPP_SRC} ${CUDA_SRC}
  )
else()
  cs_add_library(${PROJECT_NAME}
    ${CPP_SRC}
  )
endif()

cuda_add_executable(${PROJECT_NAME}_exe app/main.cc)
set (devel_addr /root/chamo_vps/devel/lib)
target_link_libraries(${PROJECT_NAME}_exe 
${devel_addr}/libgflags.so
${devel_addr}/libsfm.so
${devel_addr}/libsql_catkin.so
${devel_addr}/libglog.so
${devel_addr}/libceres.so
${devel_addr}/libPBA_catkin.so
${devel_addr}/libSiftGPU_catkin.so
${devel_addr}/libboost_filesystem.so
${devel_addr}/libboost_regex.so
${devel_addr}/libboost_system.so
${devel_addr}/libboost_program_options.so
${devel_addr}/libVLFeat_catkin.so
${devel_addr}/libLSD_catkin.so
${devel_addr}/libFLANN_catkin.so
${devel_addr}/libPoissonRecon_catkin.so
${devel_addr}/libGraclus_catkin.so
/usr/lib/x86_64-linux-gnu/libuuid.so
/opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so
/opt/ros/kinetic/lib/libnodeletlib.so
/opt/ros/kinetic/lib/libbondcpp.so
/opt/ros/kinetic/lib/librosbag.so
/opt/ros/kinetic/lib/librosbag_storage.so
/opt/ros/kinetic/lib/libroslz4.so
/opt/ros/kinetic/lib/libtopic_tools.so
/opt/ros/kinetic/lib/libtf.so
/opt/ros/kinetic/lib/libtf2_ros.so
/opt/ros/kinetic/lib/libactionlib.so
/opt/ros/kinetic/lib/libtf2.so
/opt/ros/kinetic/lib/libimage_transport.so
/opt/ros/kinetic/lib/libmessage_filters.so
/opt/ros/kinetic/lib/libclass_loader.so
/opt/ros/kinetic/lib/libroscpp.so
/opt/ros/kinetic/lib/librosconsole.so
/opt/ros/kinetic/lib/librosconsole_log4cxx.so
/opt/ros/kinetic/lib/librosconsole_backend_interface.so
/opt/ros/kinetic/lib/libxmlrpcpp.so
/opt/ros/kinetic/lib/libroslib.so
/opt/ros/kinetic/lib/librospack.so
/opt/ros/kinetic/lib/libroscpp_serialization.so
/opt/ros/kinetic/lib/librostime.so
/opt/ros/kinetic/lib/libcpp_common.so
${devel_addr}/libchamo_common.so
${devel_addr}/libvisualization.so
${FREEIMAGE_LIBRARIES}
#${PROJECT_NAME}
#${catkin_LIBRARIES}
${devel_addr}/libopencv_calib3d.so
${devel_addr}/libopencv_core.so
${devel_addr}/libopencv_features2d.so
${devel_addr}/libopencv_flann.so
${devel_addr}/libopencv_highgui.so
${devel_addr}/libopencv_imgcodecs.so
${devel_addr}/libopencv_imgproc.so
${devel_addr}/libopencv_ml.so
${devel_addr}/libopencv_objdetect.so
${devel_addr}/libopencv_phase_unwrapping.so
${devel_addr}/libopencv_photo.so
${devel_addr}/libopencv_rgbd.so
${devel_addr}/libopencv_saliency.so
${devel_addr}/libopencv_shape.so
${devel_addr}/libgps_tool.so
${devel_addr}/libsim3_ransac.so
)

message(STATUS "lib: ${catkin_LIBRARIES}")
cs_install()
cs_export()
