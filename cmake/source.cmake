# Source files
cmake_minimum_required(VERSION 3.10)

file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/api/*.cpp
    ${CMAKE_SOURCE_DIR}/core/*.cpp
)

configure_file(
    ${CMAKE_SOURCE_DIR}/core/version/version.hpp.in
    ${CMAKE_SOURCE_DIR}/core/version/version.hpp
    @ONLY
)

message("=============================================================")
message("-- lidar_sdk Version : v${PROJECT_VERSION}")
message("=============================================================")
