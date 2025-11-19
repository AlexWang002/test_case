cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(upsample)

add_library_pva(upsample_api
    HOST
        upsample.cpp
    DEVICE
        upsample_dev
    STATIC_HOST)

target_include_directories(upsample_api
                           PRIVATE ..)
