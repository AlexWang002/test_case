cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(trail)

add_library_pva(trail_api
    HOST
        trail.cpp
    DEVICE
        trail_dev
    STATIC_HOST)


target_include_directories(trail_api
                           PRIVATE ..
                           )
