cmake_minimum_required(VERSION 3.20)
find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(pva_utils)
add_library_pva(pva_utils_api
    HOST
        pva_utils.cpp
    DEVICE
        
    STATIC_HOST)

target_include_directories(pva_utils_api
                           PRIVATE ..
                           )
