/*******************************************************************************
 * \addtogroup trail_programm
 * \{
 * \file trail.cpp
 * \brief
 * \version 0.2
 * \date 2025-11-28
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-11 | Init version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-11-28 | Add exception log messages |
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp>
#include <cupva_platform.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "trail.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(trail_dev)

uint16_t *DistIn_d = nullptr;
uint16_t *DistIn_h = nullptr;

uint16_t *ValidOut_d = nullptr;
uint16_t *ValidOut_h = nullptr;

Stream Trail_stream;
namespace
{
    TrailParam_t TrailParams = DEFAULT_TRAIL_PARAM;
}

Executable& getTrailExec() {
    thread_local Executable trail_exec = Executable::Create(
        PVA_EXECUTABLE_DATA(trail_dev),
        PVA_EXECUTABLE_SIZE(trail_dev)
    );
    return trail_exec;
}

CmdProgram& getTrailProg() {
    thread_local CmdProgram trail_prog = CmdProgram::Create(getTrailExec());
    return trail_prog;
}

// Executable trail_exec = Executable::Create(PVA_EXECUTABLE_DATA(trail_dev),
//                                         PVA_EXECUTABLE_SIZE(trail_dev));
// CmdProgram trail_prog = CmdProgram::Create(trail_exec);

int pvaTrailCompile()
{
    try
    {
        CmdProgram& trail_prog = getTrailProg();
        trail_prog["algorithmParams"].set((int *)&TrailParams, sizeof(TrailParam_t));
        RasterDataFlow &sourceDistDataFlow  = trail_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler      = trail_prog["sourceDistDataFlowHandler"];
        uint16_t *inputDistBufferVMEM       = trail_prog["inputDistBufferVMEM"].ptr<uint16_t>();

        RasterDataFlow &destinationDataFlow = trail_prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler     = trail_prog["destinationDataFlowHandler"];
        uint16_t *outputValidBufferVMEM     = trail_prog["outputValidBufferVMEM"].ptr<uint16_t>();

        sourceDistDataFlow.handler(sourceDistDataFlowHandler)
            .src(DistIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM)
            .tile(TILE_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        destinationDataFlow.handler(destinationDataFlowHandler)
            .dst(ValidOut_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputValidBufferVMEM)
            .tile(TILE_WIDTH, TILE_HEIGHT);

        trail_prog.compileDataFlows();
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

/**
 * \brief Allocate memory for trail processing data structures
*/
void TrailDataAlloc()
{
    DistIn_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistIn_h = (uint16_t *)mem::GetHostPointer(DistIn_d);

    ValidOut_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT* VIEW_WIDTH * sizeof(uint16_t));
    ValidOut_h = (uint16_t *)mem::GetHostPointer(ValidOut_d);

    Trail_stream = Stream::Create(PVA0, VPU1);

    pvaTrailCompile();
}

/**
 * \brief Free memory for trail processing data structures
*/
void TrailDataFree()
{
    mem::Free(DistIn_d);
    mem::Free(ValidOut_d);
}

/**
 * \brief Create and submit pva task for trail algo
 *
 * \param[in] exception_msg: Exception message
 *                 Range: NA. Accuracy: NA.
 *
 * \param[in] status_code: column index of the buffer
 *                 Range: 0-2. Accuracy: 1.
*/
int trail_main(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        CmdProgram& trail_prog = getTrailProg();

        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];

        auto time5 = std::chrono::steady_clock::now();

        Trail_stream.submit({&trail_prog, &rf}, status);

        auto time6 = std::chrono::steady_clock::now();

        fence.wait(); // trail task timeout: 3.5ms

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
