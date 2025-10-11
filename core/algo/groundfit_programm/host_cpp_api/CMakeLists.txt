#
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

# The PVA SDK includes CMake scripts to enable building an application for Tegra.
# PVA SDK build scripts can handle cross compilation of host code using the argument -DPVA_BUILD_MODE.
# PVA_BUILD_MODE can be set to QNX or L4T (Linux for Tegra).
# To build a PVA application for Linux, you can do:
# $ cmake -DPVA_BUILD_MODE=L4T ..
# $ make
# [cmake_project]
cmake_minimum_required(VERSION 3.20)
# We first call the find_package(pva-sdk) command to detect the installed PVA SDK package before setting the name of the project.
find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(groundfit)
# [cmake_project]

# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# Let's create an executable by building the host side application code.
# add_executable_pva is a custom build command PVA SDK provides.
# Host source files listed under the HOST keyword are added to the target.
# Device targets are embedded in the host target as data blobs.
# add_executable_pva command also handles linking against the cuPVA runtime libraries.
# [add_executable]
add_library_pva(groundfit_api
    HOST
        groundfit.cpp
    DEVICE
        groundfit_dev
    STATIC_HOST)
# [add_executable]

target_include_directories(groundfit_api
                           PRIVATE ..)
                           