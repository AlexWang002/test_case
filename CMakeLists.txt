cmake_minimum_required(VERSION 3.20)
find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(lidar_robosense_SDK LANGUAGES CXX VERSION 3.02.01)

# Options
option(COMPILE_EXAMPLES "Build Lidar SDK examples" ON)
option(COMPILE_ARM_VERSION "Build arm version" OFF)
option(COMPILE_TABLE_HIDE "Hide interface or not" ON)
option(COMPILE_RELEASE_VERSION "Compile release version" ON)

if(COMPILE_ARM_VERSION)
    message("Build arm version")
    add_definitions(-DARM)
    set(INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/release/arm_release)
else()
    message("Build pc version")
    set(INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/release/pc_release)
endif()

if(COMPILE_RELEASE_VERSION)
    message("Compile release version 3040 mipi")
    add_compile_definitions(COMPILE_RELEASE_VERSION=1)
else()
    message("Compile local test version 3372 mipi")
    add_compile_definitions(COMPILE_RELEASE_VERSION=0)
endif()

# Add submodules
add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/rs_logger)

# Include cmake files
include(${CMAKE_SOURCE_DIR}/cmake/module.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/compile.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/source.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/sdk_api.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/install.cmake)

if(COMPILE_EXAMPLES)
    add_subdirectory(${CMAKE_SOURCE_DIR}/examples)
endif()
