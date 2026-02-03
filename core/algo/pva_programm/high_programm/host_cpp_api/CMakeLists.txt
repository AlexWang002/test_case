cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(highcalc)

add_library_pva(highcalc_api
    HOST
        highcalc.cpp
    DEVICE
        highcalc_dev
    STATIC_HOST)


target_include_directories(highcalc_api
                           PRIVATE ..
                           )
