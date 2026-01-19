cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(spray)

add_library_pva(spray_api
    HOST
        spray.cpp
    DEVICE
        spray_dev
        enhance_dev
    STATIC_HOST)


target_include_directories(spray_api
                           PRIVATE ..
                           )
