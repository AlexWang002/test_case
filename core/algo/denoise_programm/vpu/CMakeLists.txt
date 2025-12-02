cmake_minimum_required(VERSION 3.20)
find_package(pva-sdk 2.5 REQUIRED)
project(denoise_programm)
pva_device(denoise_dev
    denoise_top.c)
