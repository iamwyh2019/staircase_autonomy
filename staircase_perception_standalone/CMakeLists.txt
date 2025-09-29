cmake_minimum_required(VERSION 3.10)
project(staircase_perception_standalone)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting CMAKE_BUILD_TYPE to ${CMAKE_BUILD_TYPE}")
endif()

# Find required packages
find_package(Eigen3 REQUIRED)

# Try multiple methods to find PCL
set(PCL_FOUND FALSE)

# Method 1: Try standard find_package
find_package(PCL QUIET COMPONENTS common io filters)
if(PCL_FOUND)
    message(STATUS "Found PCL via find_package: ${PCL_VERSION}")
endif()

# Method 2: Try pkg-config if find_package failed
if(NOT PCL_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(PCL QUIET pcl_common pcl_io pcl_filters)
        if(PCL_FOUND)
            message(STATUS "Found PCL via pkg-config: ${PCL_VERSION}")
        endif()
    endif()
endif()

# Method 3: Manual library finding if others failed
if(NOT PCL_FOUND)
    message(STATUS "Attempting manual PCL discovery...")
    find_library(PCL_COMMON_LIBRARY pcl_common)
    find_library(PCL_IO_LIBRARY pcl_io)
    find_library(PCL_FILTERS_LIBRARY pcl_filters)
    find_path(PCL_INCLUDE_DIR pcl/point_cloud.h)

    if(PCL_COMMON_LIBRARY AND PCL_IO_LIBRARY AND PCL_FILTERS_LIBRARY AND PCL_INCLUDE_DIR)
        set(PCL_FOUND TRUE)
        set(PCL_LIBRARIES ${PCL_COMMON_LIBRARY} ${PCL_IO_LIBRARY} ${PCL_FILTERS_LIBRARY})
        set(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIR})
        message(STATUS "Found PCL manually: ${PCL_INCLUDE_DIR}")
    endif()
endif()

if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL not found! Please install PCL development packages.")
endif()

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Line extraction library
add_library(line_extraction
  src/utils/line_extraction/line.cpp
  src/utils/line_extraction/line_extractor.cpp
)

target_link_libraries(line_extraction
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Config parser library
add_library(config_parser
  src/utils/config_parser.cpp
)

target_link_libraries(config_parser
  Eigen3::Eigen
)

# StairDetector library
add_library(stair_detector
  src/core/stair_detector.cpp
)

target_link_libraries(stair_detector
  line_extraction
  config_parser
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Phase 1 test executable
add_executable(test_detector_only
  tests/test_detector_only.cpp
)

target_link_libraries(test_detector_only
  stair_detector
  config_parser
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# PLY to PCD converter tool
add_executable(ply_to_pcd_converter
  tools/ply_to_pcd_converter.cpp
)

target_link_libraries(ply_to_pcd_converter
  ${PCL_LIBRARIES}
)

# PCD inspection tool
add_executable(inspect_pcd
  tools/inspect_pcd.cpp
)

target_link_libraries(inspect_pcd
  ${PCL_LIBRARIES}
)

# PCD to PLY converter tool
add_executable(pcd_to_ply
  tools/pcd_to_ply.cpp
)

target_link_libraries(pcd_to_ply
  ${PCL_LIBRARIES}
)



# Install targets
install(TARGETS stair_detector line_extraction config_parser test_detector_only ply_to_pcd_converter inspect_pcd pcd_to_ply
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)