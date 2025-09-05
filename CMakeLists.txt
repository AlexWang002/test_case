cmake_minimum_required(VERSION 3.10)
project(lidar_robosense_SDK LANGUAGES CXX VERSION 3.02.01)

# Options
option(COMPILE_EXAMPLES "Build Lidar SDK examples" ON)
option(COMPILE_ARM_VERSION "Build arm version" OFF)
option(COMPILE_TABLE_HIDE "Hide interface or not" ON)

if(COMPILE_ARM_VERSION)
    message("Build arm version")
    add_definitions(-DARM)
    set(INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/release/arm_release)
else()
    message("Build pc version")
    set(INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/release/pc_release)
endif()

# Add submodules
add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/rs_logger)
add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/yaml-cpp)

# Include cmake files
include(${CMAKE_SOURCE_DIR}/cmake/module.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/compile.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/source.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/sdk_api.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/install.cmake)

if(COMPILE_EXAMPLES)
    add_subdirectory(${CMAKE_SOURCE_DIR}/examples)
endif()
