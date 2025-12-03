cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
project(trail_programm)

pva_device(trail_dev
    trail_top.c
    BCF
    vpu_stack_size.bcf)