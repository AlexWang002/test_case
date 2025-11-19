/*******************************************************************************
 * \addtogroup spray_programm
 * \{
 * \file spray.cpp
 * \brief
 * \version 0.1
 * \date 2025-09-11
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-11 | Init version |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> /**< Main host-side C++-API header file */
#include <cupva_platform.h> /**< Header that includes macros for specifying PVA executables */
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "spray.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(spray_dev)
PVA_DECLARE_EXECUTABLE(enhance_dev)

uint16_t *DistIn0_d = nullptr;
uint16_t *DistIn0_h = nullptr;
uint16_t *DistIn1_d = nullptr;
uint16_t *DistIn1_h = nullptr;
uint16_t *RefIn0_d = nullptr;
uint16_t *RefIn0_h = nullptr;
uint16_t *RefIn1_d = nullptr;
uint16_t *RefIn1_h = nullptr;
uint16_t *GndIn0_d = nullptr;
uint16_t *GndIn0_h = nullptr;
uint16_t *GndIn1_d = nullptr;
uint16_t *GndIn1_h = nullptr;
uint16_t *AttIn0_d = nullptr;
uint16_t *AttIn0_h = nullptr;
uint16_t *AttIn1_d = nullptr;
uint16_t *AttIn1_h = nullptr;
uint16_t *Glink_d = nullptr;
uint16_t *Glink_h = nullptr;

uint16_t *RainOut0_d = nullptr;
uint16_t *RainOut0_h = nullptr;
uint16_t *RainOut1_d = nullptr;
uint16_t *RainOut1_h = nullptr;

uint16_t *FinalOut0_d = nullptr;
uint16_t *FinalOut0_h = nullptr;
uint16_t *FinalOut1_d = nullptr;
uint16_t *FinalOut1_h = nullptr;

Stream Spray_stream;
namespace
{
    SprayParam_t SprayParams = DEFAULT_SPRAY_PARAM;
}
/**
 * \brief Allocate memory for spray processing data structures
*/
void sprayDataAlloc()
{
    DistIn0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistIn0_h = (uint16_t *)mem::GetHostPointer(DistIn0_d);
    DistIn1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistIn1_h = (uint16_t *)mem::GetHostPointer(DistIn1_d);
    RefIn0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    RefIn0_h = (uint16_t *)mem::GetHostPointer(RefIn0_d);
    RefIn1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    RefIn1_h = (uint16_t *)mem::GetHostPointer(RefIn1_d);
    GndIn0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    GndIn0_h = (uint16_t *)mem::GetHostPointer(GndIn0_d);
    GndIn1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    GndIn1_h = (uint16_t *)mem::GetHostPointer(GndIn1_d);
    AttIn0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    AttIn0_h = (uint16_t *)mem::GetHostPointer(AttIn0_d);
    AttIn1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    AttIn1_h = (uint16_t *)mem::GetHostPointer(AttIn1_d);
    Glink_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * 4 * sizeof(uint16_t)); // 1,2回波
    Glink_h = (uint16_t *)mem::GetHostPointer(Glink_d);

    RainOut0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    RainOut0_h = (uint16_t *)mem::GetHostPointer(RainOut0_d);
    RainOut1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    RainOut1_h = (uint16_t *)mem::GetHostPointer(RainOut1_d);

    FinalOut0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    FinalOut0_h = (uint16_t *)mem::GetHostPointer(FinalOut0_d);
    FinalOut1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    FinalOut1_h = (uint16_t *)mem::GetHostPointer(FinalOut1_d);

    Spray_stream = Stream::Create(PVA0, VPU1);
}

/**
 * \brief Free memory for spray processing data structures
*/
void sprayDataFree()
{
    mem::Free(DistIn0_d);
    mem::Free(DistIn1_d);
    mem::Free(RefIn0_d);
    mem::Free(RefIn1_d);
    mem::Free(GndIn0_d);
    mem::Free(GndIn1_d);
    mem::Free(AttIn0_d);
    mem::Free(AttIn1_d);
    mem::Free(Glink_d);
    mem::Free(RainOut0_d);
    mem::Free(RainOut1_d);
    mem::Free(FinalOut0_d);
    mem::Free(FinalOut1_d);
}

/**
 * \brief Spray processing main function in host-side C++ API
*/
void sprayRemovePva()
{
    try
    {
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(spray_dev),
                                             PVA_EXECUTABLE_SIZE(spray_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        prog["algorithmParams"].set((int *)&SprayParams, sizeof(SprayParam_t));

        RasterDataFlow &sourceDistDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波距离
        auto sourceDistDataFlowHandler0 = prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(DistIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceDistDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波距离
        auto sourceDistDataFlowHandler1 = prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(DistIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波反射率
        auto sourceRefDataFlowHandler0 = prog["sourceRefDataFlowHandler0"];
        uint16_t *inputRefBufferVMEM0 = prog["inputRefBufferVMEM0"].ptr<uint16_t>();

        sourceRefDataFlow0.handler(sourceRefDataFlowHandler0)
            .src(RefIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波反射率
        auto sourceRefDataFlowHandler1 = prog["sourceRefDataFlowHandler1"];
        uint16_t *inputRefBufferVMEM1 = prog["inputRefBufferVMEM1"].ptr<uint16_t>();

        sourceRefDataFlow1.handler(sourceRefDataFlowHandler1)
            .src(RefIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceAttDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波属性
        auto sourceAttDataFlowHandler0 = prog["sourceAttDataFlowHandler0"];
        uint16_t *inputAttBufferVMEM0 = prog["inputAttBufferVMEM0"].ptr<uint16_t>();

        sourceAttDataFlow0.handler(sourceAttDataFlowHandler0)
            .src(AttIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceAttDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波属性
        auto sourceAttDataFlowHandler1 = prog["sourceAttDataFlowHandler1"];
        uint16_t *inputAttBufferVMEM1 = prog["inputAttBufferVMEM1"].ptr<uint16_t>();

        sourceAttDataFlow1.handler(sourceAttDataFlowHandler1)
            .src(AttIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceGlinkDataFlow = prog.addDataFlowHead<RasterDataFlow>();//输入单回波地面链接
        auto sourceGlkDataFlowHandler = prog["sourceGlkDataFlowHandler"];
        uint16_t *inputGlkBufferVMEM = prog["inputGlkBufferVMEM"].ptr<uint16_t>();

        sourceGlinkDataFlow.handler(sourceGlkDataFlowHandler)
            .src(Glink_d, VIEW_WIDTH, VIEW_HEIGHT * 4, VIEW_WIDTH)
            .tileBuffer(inputGlkBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT * 4);

        RasterDataFlow &destinationDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输出单回波标记
        auto destinationDataFlowHandler0 = prog["destinationDataFlowHandler0"];
        uint16_t *outputRainBufferVMEM0 = prog["outputRainBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(RainOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRainBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输出双回波标记
        auto destinationDataFlowHandler1 = prog["destinationDataFlowHandler1"];
        uint16_t *outputRainBufferVMEM1 = prog["outputRainBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(RainOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRainBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        prog.compileDataFlows();

        SetVPUPrintBufferSize(64 * 1024);
        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];
        Spray_stream.submit({&prog, &rf}, status);
        fence.wait();
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


void rainEnhancePva()
{
    try
    {
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(enhance_dev),
                                             PVA_EXECUTABLE_SIZE(enhance_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        prog["algorithmParams"].set((int *)&SprayParams, sizeof(SprayParam_t));

        /*pva input*/
        /*Dist wave0*/
        RasterDataFlow &sourceDistDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波距离
        auto sourceDistDataFlowHandler0 = prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(DistIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        /*Dist wave1*/
        RasterDataFlow &sourceDistDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波距离
        auto sourceDistDataFlowHandler1 = prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(DistIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);
        
        /*Ref wave0*/
        RasterDataFlow &sourceRefDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler0 = prog["sourceRefDataFlowHandler0"];
        uint16_t *inputRefBufferVMEM0 = prog["inputRefBufferVMEM0"].ptr<uint16_t>();

        sourceRefDataFlow0.handler(sourceRefDataFlowHandler0)
            .src(RefIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        /*Ref wave1*/
        RasterDataFlow &sourceRefDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler1 = prog["sourceRefDataFlowHandler1"];
        uint16_t *inputRefBufferVMEM1 = prog["inputRefBufferVMEM1"].ptr<uint16_t>();

        sourceRefDataFlow1.handler(sourceRefDataFlowHandler1)
            .src(RefIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);
        
        /*Gnd wave0 & wave1*/
        RasterDataFlow &sourceGlkDataFlow = prog.addDataFlowHead<RasterDataFlow>();//输入单回波距离
        auto sourceGlkDataFlowHandler = prog["sourceGlkDataFlowHandler"];
        uint16_t *inputGlkBufferVMEM = prog["inputGlkBufferVMEM"].ptr<uint16_t>();

        sourceGlkDataFlow.handler(sourceGlkDataFlowHandler)
            .src(Glink_d, VIEW_WIDTH, VIEW_HEIGHT * 4, VIEW_WIDTH)
            .tileBuffer(inputGlkBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT * 4);
        
        /*Rain wave0*/
        RasterDataFlow &sourceRainDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波反射率
        auto sourceRainDataFlowHandler0 = prog["sourceRainDataFlowHandler0"];
        uint16_t *inputRainBufferVMEM0 = prog["inputRainBufferVMEM0"].ptr<uint16_t>();

        sourceRainDataFlow0.handler(sourceRainDataFlowHandler0)
            .src(RainOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRainBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        /*Rain wave1*/
        RasterDataFlow &sourceRainDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波反射率
        auto sourceRainDataFlowHandler1 = prog["sourceRainDataFlowHandler1"];
        uint16_t *inputRainBufferVMEM1 = prog["inputRainBufferVMEM1"].ptr<uint16_t>();

        sourceRainDataFlow1.handler(sourceRainDataFlowHandler1)
            .src(RainOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRainBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        /*pva output*/
        RasterDataFlow &destinationDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输出单回波标记
        auto destinationDataFlowHandler0 = prog["destinationDataFlowHandler0"];
        uint16_t *outputFinalBufferVMEM0 = prog["outputFinalBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(FinalOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputFinalBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输出双回波标记
        auto destinationDataFlowHandler1 = prog["destinationDataFlowHandler1"];
        uint16_t *outputFinalBufferVMEM1 = prog["outputFinalBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(FinalOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputFinalBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        prog.compileDataFlows();

        SetVPUPrintBufferSize(64 * 1024);
        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];
        Spray_stream.submit({&prog, &rf}, status);
        fence.wait();
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