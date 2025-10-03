cmake_minimum_required(VERSION 3.10)
project(stair_detector_python)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific configuration
if(APPLE)
    # Force x86_64 architecture on macOS
    set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "Build architectures for macOS" FORCE)
elseif(WIN32)
    # Windows-specific settings
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    # Set runtime library to match Python (usually /MD for release)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
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

# Find Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")
message(STATUS "Python3_VERSION: ${Python3_VERSION}")

# Fetch pybind11
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# Include directories (use parent project's includes)
set(STANDALONE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
include_directories(
    ${STANDALONE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Build the existing standalone libraries from parent project
# Line extraction library
add_library(line_extraction
    ${STANDALONE_DIR}/src/utils/line_extraction/line.cpp
    ${STANDALONE_DIR}/src/utils/line_extraction/line_extractor.cpp
)
target_link_libraries(line_extraction
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)

# Config parser library
add_library(config_parser
    ${STANDALONE_DIR}/src/utils/config_parser.cpp
)
target_link_libraries(config_parser
    Eigen3::Eigen
)

# StairDetector library
add_library(stair_detector_lib
    ${STANDALONE_DIR}/src/core/stair_detector.cpp
)
target_link_libraries(stair_detector_lib
    line_extraction
    config_parser
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)

# Python module
pybind11_add_module(stair_detector bindings.cpp)

target_link_libraries(stair_detector PRIVATE
    stair_detector_lib
    config_parser
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)



# Windows-specific linking
if(WIN32)
    # Ensure proper symbol visibility
    target_compile_definitions(stair_detector PRIVATE PYBIND11_COMPILER_TYPE="_MSC_VER")

    # Try to link statically to reduce DLL dependencies
    set_property(TARGET stair_detector PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    target_compile_definitions(stair_detector PRIVATE PCL_STATIC)
endif()
