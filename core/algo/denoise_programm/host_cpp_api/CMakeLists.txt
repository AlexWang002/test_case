cmake_minimum_required(VERSION 3.20)
find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(denoise)
add_library_pva(denoise_api
    HOST
        denoise.cpp
    DEVICE
        denoise_dev
    STATIC_HOST)

target_include_directories(denoise_api
                           PRIVATE ..
                           )
