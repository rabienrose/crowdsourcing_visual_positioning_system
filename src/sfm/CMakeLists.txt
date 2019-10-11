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
  base/scene_clustering.cc
  base/similarity_transform.cc
  base/track.cc
  base/triangulation.cc
  base/undistortion.cc
  base/visibility_pyramid.cc
  base/warp.cc
  util/bitmap.cc
  util/camera_specs.cc
  util/logging.cc
  util/math.cc
  util/misc.cc
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

cs_install()
cs_export()
