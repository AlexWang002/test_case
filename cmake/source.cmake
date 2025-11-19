# Source files
cmake_minimum_required(VERSION 3.10)

file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/api/*.cpp
    ${CMAKE_SOURCE_DIR}/core/*.cpp
)

# 排除 Nvidia GPU 底软文件
list(REMOVE_ITEM SOURCES ${CMAKE_SOURCE_DIR}/core/impl/nvBufObjToCpuBuf.cpp)

if(COMPILE_ARM_VERSION)
    # 在 ARM 架构下添加特定文件
    list(APPEND SOURCES ${CMAKE_SOURCE_DIR}/core/impl/nvBufObjToCpuBuf.cpp)
endif()

configure_file(
    ${CMAKE_SOURCE_DIR}/core/version/version.hpp.in
    ${CMAKE_SOURCE_DIR}/core/version/version.hpp
    @ONLY
)

message("=============================================================")
message("-- lidar_sdk Version : v${PROJECT_VERSION}")
message("=============================================================")
