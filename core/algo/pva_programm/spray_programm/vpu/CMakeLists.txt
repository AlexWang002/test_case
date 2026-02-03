cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
project(spray_programm)

pva_device(spray_dev
        spray_top.c
)

pva_device(enhance_dev
        enhance_top.c
)