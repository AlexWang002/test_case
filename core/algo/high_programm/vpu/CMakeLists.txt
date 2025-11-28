cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
project(high_programm)

pva_device(highcalc_dev
        highcalc_top.c
    BCF
        vpu_stack_size.bcf
)