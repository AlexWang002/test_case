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
#include "../../pva_utils.h"

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

namespace
{
    HighcalcParam_t HighcalcParams = DEFAULT_HIGH_PARAM;
}

/**
 * \brief Get an Executable object instance
 *
 * \return An Executable object instance
 */
Executable& getHighcalcExec() {
    static Executable highcalc_exec = Executable::Create(
        PVA_EXECUTABLE_DATA(highcalc_dev),
        PVA_EXECUTABLE_SIZE(highcalc_dev)
    );
    return highcalc_exec;
}

/**
 * \brief Get an CmdProgram object instance
 *
 * \return An CmdProgram object instance
 */
CmdProgram& getHighcalcProg() {
    static CmdProgram highcalc_prog = CmdProgram::Create(getHighcalcExec());
    return highcalc_prog;
}

/**
 * \brief Compile the pva dataflow
 *
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
 */
int pvaHighcalcCompile()
{
    try
    {
        CmdProgram& highcalc_prog = getHighcalcProg();
        highcalc_prog["algorithmParams"].set((int *)&HighcalcParams, sizeof(HighcalcParam_t));

        RasterDataFlow &sourceDistDataFlow0 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输入单回波距离
        auto sourceDistDataFlowHandler0 = highcalc_prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = highcalc_prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(h_dist_in0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceDistDataFlow1 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输入双回波距离
        auto sourceDistDataFlowHandler1 = highcalc_prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = highcalc_prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(h_dist_in1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceHighDataFlow0 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输入单回波高度值
        auto sourceHighDataFlowHandler0 = highcalc_prog["sourceHighDataFlowHandler0"];
        uint16_t *inputHighBufferVMEM0 = highcalc_prog["inputHighBufferVMEM0"].ptr<uint16_t>();

        sourceHighDataFlow0.handler(sourceHighDataFlowHandler0)
            .src(h_high_in0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputHighBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceHighDataFlow1 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输入双回波高度值
        auto sourceHighDataFlowHandler1 = highcalc_prog["sourceHighDataFlowHandler1"];
        uint16_t *inputHighBufferVMEM1 = highcalc_prog["inputHighBufferVMEM1"].ptr<uint16_t>();

        sourceHighDataFlow1.handler(sourceHighDataFlowHandler1)
            .src(h_high_in1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputHighBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceProjDistDataFlow0 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输入单回波投影距离
        auto sourceProjDistDataFlowHandler0 = highcalc_prog["sourceProjDistDataFlowHandler0"];
        uint16_t *inputProjDistBufferVMEM0 = highcalc_prog["inputProjDistBufferVMEM0"].ptr<uint16_t>();

        sourceProjDistDataFlow0.handler(sourceProjDistDataFlowHandler0)
            .src(h_proj_dist0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputProjDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceProjDistDataFlow1 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输入双回波投影距离
        auto sourceProjDistDataFlowHandler1 = highcalc_prog["sourceProjDistDataFlowHandler1"];
        uint16_t *inputProjDistBufferVMEM1 = highcalc_prog["inputProjDistBufferVMEM1"].ptr<uint16_t>();

        sourceProjDistDataFlow1.handler(sourceProjDistDataFlowHandler1)
            .src(h_proj_dist1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputProjDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &destinationDataFlow0 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输出单回波地面标记
        auto destinationDataFlowHandler0 = highcalc_prog["destinationDataFlowHandler0"];
        uint16_t *outputGndBufferVMEM0 = highcalc_prog["outputGndBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(h_gnd_out0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputGndBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = highcalc_prog.addDataFlowHead<RasterDataFlow>();//输出双回波地面标记
        auto destinationDataFlowHandler1 = highcalc_prog["destinationDataFlowHandler1"];
        uint16_t *outputGndBufferVMEM1 = highcalc_prog["outputGndBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(h_gnd_out1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputGndBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        highcalc_prog.compileDataFlows();
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Allocate memory for highcalc processing data structures
*/
int highcalcDataAlloc()
{
    try 
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

        if (pvaHighcalcCompile() != 0) {
            return 1;
        }
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Free memory for highcalc processing data structures
*/
int highcalcDataFree()
{
    try
    {
        mem::Free(h_dist_in0_d);
        mem::Free(h_dist_in1_d);
        mem::Free(h_high_in0_d);
        mem::Free(h_high_in1_d);
        mem::Free(h_proj_dist0_d);
        mem::Free(h_proj_dist1_d);
        mem::Free(h_gnd_out0_d);
        mem::Free(h_gnd_out1_d);

        h_dist_in0_h = nullptr;
        h_dist_in1_h = nullptr;
        h_high_in0_h = nullptr;
        h_high_in1_h = nullptr;
        h_proj_dist0_h = nullptr;
        h_proj_dist1_h = nullptr;
        h_gnd_out0_h = nullptr;
        h_gnd_out1_h = nullptr;
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Spray processing main function in host-side C++ API
*/
int highcalcPva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        CmdProgram& highcalc_prog = getHighcalcProg();

        SyncObj sync = SyncObj::Create(true);
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];

        auto time5 = std::chrono::steady_clock::now();

        Stream& algo_stream = getAlgoStream();
        algo_stream.submit({&highcalc_prog, &rf}, status);

        auto time6 = std::chrono::steady_clock::now();

        fence.wait(); // highcalc task timeout: 6.5ms

        auto time7 = std::chrono::steady_clock::now();

        submit_time = std::chrono::duration_cast<std::chrono::microseconds>(time6 - time5).count();
        wait_time = std::chrono::duration_cast<std::chrono::microseconds>(time7 - time6).count();

        cupva::Error statusCode = CheckCommandStatus(status[0]);
        if (statusCode != Error::None)
        {
            status_code = (int32_t)statusCode;
            return 2;
        }
    }
    catch (cupva::Exception const &e)
    {
        exception_msg = std::string(e.what());
        return 1;
    }
    return 0;
}