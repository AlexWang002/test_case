cmake_minimum_required(VERSION 3.20)
find_package(pva-sdk 2.5 REQUIRED)
project(upsample_programm)
pva_device(upsample_dev
    upsample_top.c)
