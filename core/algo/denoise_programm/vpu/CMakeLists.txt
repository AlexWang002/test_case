#
# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA Corporation and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA Corporation is strictly prohibited.
#

# [cmake_project]
cmake_minimum_required(VERSION 3.20)
# We first call the find_package(pva-sdk) command to detect the installed PVA SDK package before setting the name of the project.
find_package(pva-sdk 2.5 REQUIRED)
project(denoise_programm)
# [cmake_project]

# pva_device command is used to specify target for the device-side code.
# The first argument is the name of the target, subsequent arguments are the list of files to build.
# [pva_device]

pva_device(denoise_dev
    denoise_top.c)
# [pva_device]
