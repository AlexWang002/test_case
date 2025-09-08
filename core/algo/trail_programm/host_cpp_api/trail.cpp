/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> /**< Main host-side C++-API header file */
#include <cupva_platform.h> /**< Header that includes macros for specifying PVA executables */
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

#include "trail.h"
using namespace cupva;

PVA_DECLARE_EXECUTABLE(trail_dev)

uint16_t *DistIn_d = nullptr;
uint16_t *DistIn_h = nullptr;

uint8_t *RefIn_d = nullptr;
uint8_t *RefIn_h = nullptr;

int *ValidOut_d = nullptr;
int *ValidOut_h = nullptr;

int TrailMask[VIEW_HEIGHT][VIEW_WIDTH] = {0};
namespace
{
    int8_t TrailParam_a[sizeof(TrailParam_t)];
    TrailParam_t *TrailParams = (TrailParam_t*) TrailParam_a;
}

void TrailDataAlloc()
{
    DistIn_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistIn_h = (uint16_t *)mem::GetHostPointer(DistIn_d);

    RefIn_d = (uint8_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint8_t));
    RefIn_h = (uint8_t *)mem::GetHostPointer(RefIn_d);

    ValidOut_d = (int *)mem::Alloc(VIEW_HEIGHT* VIEW_WIDTH * sizeof(int));
    ValidOut_h = (int *)mem::GetHostPointer(ValidOut_d);
}

void TrailDataFree()
{
    mem::Free(DistIn_d);
    mem::Free(RefIn_d);
    mem::Free(ValidOut_d);
}

void trail_main()
{
    try
    {
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(trail_dev),
                                             PVA_EXECUTABLE_SIZE(trail_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        TrailParams->D_H = 250;
        TrailParams->BypassDis = 3;
        TrailParams->dist_th_ratio = 1;
        TrailParams->DisThreRatio = DIS_THRE_RATIO_VALUE;
        TrailParams->near_cnt_th_h = 2400;
        TrailParams->near_cnt_th_v = 3;
        TrailParams->SlopDifThre = 5;

        prog["algorithmParams"].set(TrailParam_a, sizeof(TrailParam_t));

        RasterDataFlow &sourceDistDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler     = prog["sourceDistDataFlowHandler"];
        uint16_t *inputDistBufferVMEM   = prog["inputDistBufferVMEM"].ptr<uint16_t>();

        RasterDataFlow &sourceRefDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler     = prog["sourceRefDataFlowHandler"];
        uint16_t *inputRefBufferVMEM   = prog["inputRefBufferVMEM"].ptr<uint16_t>();

        RasterDataFlow &destinationDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler     = prog["destinationDataFlowHandler"];
        uint16_t *outputValidBufferVMEM       = prog["outputValidBufferVMEM"].ptr<uint16_t>();


        sourceDistDataFlow.handler(sourceDistDataFlowHandler)
            .src(DistIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        sourceRefDataFlow.handler(sourceRefDataFlowHandler)
            .src(RefIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        destinationDataFlow.handler(destinationDataFlowHandler)
            .dst(ValidOut_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputValidBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        /** 编译程序数据流 */
        prog.compileDataFlows();

        SetVPUPrintBufferSize(64 * 1024);

        /** 创建一个SyncObj对象，用于存储PVA与CPU同步所需的数据，是一个单调递增的整数，每次发出信号都会增加 */
        SyncObj sync = SyncObj::Create();
        /** 创建一个栅栏对象，用于PVA程序之间的同步 */
        Fence fence{sync};
        /** 将Fence封装在cmd中，与CmdProgram一起提交到PVA */
        CmdRequestFences rf{fence};
        /** 拖点算法绑定VPU1 */
        Stream stream = Stream::Create(cupva::PVA0, cupva::VPU1);
        CmdStatus status[2];
        stream.submit({&prog, &rf}, status);
        /** 等待Fence失效 */
        fence.wait();
        /** 将结果写入数组 */
        memcpy(&TrailMask[0][0], ValidOut_h, VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
        /** 检查cmd状态 */
        cupva::Error statusCode = CheckCommandStatus(status[0]);
        if (statusCode != Error::None)
        {
            std::cout << "VPU Program returned an Error Code: " << (int32_t)statusCode << std::endl;
        }
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
    }
}
