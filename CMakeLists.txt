cmake_minimum_required(VERSION 3.20)
find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(lidar_robosense_SDK LANGUAGES CXX VERSION 2.01.00)

# Options
option(COMPILE_EXAMPLES "Build Lidar SDK examples" ON)
option(COMPILE_ARM_VERSION "Build arm version" OFF)
option(COMPILE_TABLE_HIDE "Hide interface or not" ON)
option(COMPILE_MIPI_10HZ "Compile the MIPI 10Hz version" OFF)
option(COMPILE_MIPI_30HZ "Compile the MIPI 30Hz version" OFF)

if(COMPILE_ARM_VERSION)
    message("Build arm version")
    add_definitions(-DARM)
    set(INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/release/arm_release)
else()
    message("Build pc version")
    set(INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/release/pc_release)
endif()

if(COMPILE_MIPI_10HZ)
    message("COMPILE the MIPI 10Hz version")
    add_compile_definitions(MIPI_10HZ=1)
elseif(COMPILE_MIPI_30HZ)
    message("COMPILE the MIPI 30Hz version")
    add_compile_definitions(MIPI_30HZ=1)
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
