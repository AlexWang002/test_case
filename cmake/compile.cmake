# Compiler configuration
cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message("System Default C++ Compiler: ${CMAKE_CXX_COMPILER_ID}")

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Compile mode: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-O0 -g)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-O2 -g)
else()  # Release or MinSizRel
    add_compile_options(-Wall -O3)
    add_definitions(-DDEBUG_LEVEL=0)
    add_link_options(-s)
endif()

set(SDK_API_TYPE SHARED)
add_compile_options(-Wdeprecated-declarations -pthread)
if(SUPPORT_FPIC)
    add_compile_options(-fPIC)
else()
    message(WARNING "The compiler does not support the -fPIC flag.")
endif()

if(COMPILE_ARM_VERSION)
    add_compile_options(-march=armv8.1-a+crc)
    add_compile_options(-ffast-math -funsafe-math-optimizations)
endif()