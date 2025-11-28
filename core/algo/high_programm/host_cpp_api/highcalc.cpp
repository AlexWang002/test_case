/*******************************************************************************
 * \addtogroup highcalc_programm
 * \{
 * \file highcalc.cpp
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
#include "highcalc.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(highcalc_dev)

uint16_t *h_dist_in0_d = nullptr;
uint16_t *h_dist_in0_h = nullptr;
uint16_t *h_dist_in1_d = nullptr;
uint16_t *h_dist_in1_h = nullptr;
uint16_t *h_high_in0_d = nullptr;
uint16_t *h_high_in0_h = nullptr;
uint16_t *h_high_in1_d = nullptr;
uint16_t *h_high_in1_h = nullptr;
uint16_t *h_proj_dist0_d = nullptr;
uint16_t *h_proj_dist0_h = nullptr;
uint16_t *h_proj_dist1_d = nullptr;
uint16_t *h_proj_dist1_h = nullptr;

uint16_t *h_gnd_out0_d = nullptr;
uint16_t *h_gnd_out0_h = nullptr;
uint16_t *h_gnd_out1_d = nullptr;
uint16_t *h_gnd_out1_h = nullptr;

Stream Highcalc_stream;
namespace
{
    HighcalcParam_t HighcalcParams = DEFAULT_HIGH_PARAM;
}
/**
 * \brief Allocate memory for highcalc processing data structures
*/
void highcalcDataAlloc()
{
    h_dist_in0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    h_dist_in0_h = (uint16_t *)mem::GetHostPointer(h_dist_in0_d);
    h_dist_in1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    h_dist_in1_h = (uint16_t *)mem::GetHostPointer(h_dist_in1_d);
    h_high_in0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(int16_t));
    h_high_in0_h = (uint16_t *)mem::GetHostPointer(h_high_in0_d);
    h_high_in1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(int16_t));
    h_high_in1_h = (uint16_t *)mem::GetHostPointer(h_high_in1_d);
    h_proj_dist0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    h_proj_dist0_h = (uint16_t *)mem::GetHostPointer(h_proj_dist0_d);
    h_proj_dist1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    h_proj_dist1_h = (uint16_t *)mem::GetHostPointer(h_proj_dist1_d);

    h_gnd_out0_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    h_gnd_out0_h = (uint16_t *)mem::GetHostPointer(h_gnd_out0_d);
    h_gnd_out1_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    h_gnd_out1_h = (uint16_t *)mem::GetHostPointer(h_gnd_out1_d);

    Highcalc_stream = Stream::Create(PVA0, VPU1);
}

/**
 * \brief Free memory for highcalc processing data structures
*/
void highcalcDataFree()
{
    mem::Free(h_dist_in0_d);
    mem::Free(h_dist_in1_d);
    mem::Free(h_high_in0_d);
    mem::Free(h_high_in1_d);
    mem::Free(h_proj_dist0_d);
    mem::Free(h_proj_dist1_d);
    mem::Free(h_gnd_out0_d);
    mem::Free(h_gnd_out1_d);
}

/**
 * \brief Spray processing main function in host-side C++ API
*/
void highcalcPva()
{
    try
    {
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(highcalc_dev),
                                             PVA_EXECUTABLE_SIZE(highcalc_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        prog["algorithmParams"].set((int *)&HighcalcParams, sizeof(HighcalcParam_t));

        RasterDataFlow &sourceDistDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波距离
        auto sourceDistDataFlowHandler0 = prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(h_dist_in0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceDistDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波距离
        auto sourceDistDataFlowHandler1 = prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(h_dist_in1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceHighDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波高度值
        auto sourceHighDataFlowHandler0 = prog["sourceHighDataFlowHandler0"];
        uint16_t *inputHighBufferVMEM0 = prog["inputHighBufferVMEM0"].ptr<uint16_t>();

        sourceHighDataFlow0.handler(sourceHighDataFlowHandler0)
            .src(h_high_in0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputHighBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceHighDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波高度值
        auto sourceHighDataFlowHandler1 = prog["sourceHighDataFlowHandler1"];
        uint16_t *inputHighBufferVMEM1 = prog["inputHighBufferVMEM1"].ptr<uint16_t>();

        sourceHighDataFlow1.handler(sourceHighDataFlowHandler1)
            .src(h_high_in1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputHighBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceProjDistDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输入单回波投影距离
        auto sourceProjDistDataFlowHandler0 = prog["sourceProjDistDataFlowHandler0"];
        uint16_t *inputProjDistBufferVMEM0 = prog["inputProjDistBufferVMEM0"].ptr<uint16_t>();

        sourceProjDistDataFlow0.handler(sourceProjDistDataFlowHandler0)
            .src(h_proj_dist0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputProjDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceProjDistDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输入双回波投影距离
        auto sourceProjDistDataFlowHandler1 = prog["sourceProjDistDataFlowHandler1"];
        uint16_t *inputProjDistBufferVMEM1 = prog["inputProjDistBufferVMEM1"].ptr<uint16_t>();

        sourceProjDistDataFlow1.handler(sourceProjDistDataFlowHandler1)
            .src(h_proj_dist1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputProjDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &destinationDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();//输出单回波地面标记
        auto destinationDataFlowHandler0 = prog["destinationDataFlowHandler0"];
        uint16_t *outputGndBufferVMEM0 = prog["outputGndBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(h_gnd_out0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputGndBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();//输出双回波地面标记
        auto destinationDataFlowHandler1 = prog["destinationDataFlowHandler1"];
        uint16_t *outputGndBufferVMEM1 = prog["outputGndBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(h_gnd_out1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputGndBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        prog.compileDataFlows();

        SetVPUPrintBufferSize(64 * 1024);
        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];
        Highcalc_stream.submit({&prog, &rf}, status);
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