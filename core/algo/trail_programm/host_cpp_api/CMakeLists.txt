#
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

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
