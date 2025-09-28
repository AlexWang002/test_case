/*******************************************************************************
 * \addtogroup trail_programm
 * \{
 * \file trail.cpp
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

uint16_t TrailMask[VIEW_HEIGHT][VIEW_WIDTH] = {0};
Stream Trail_stream;
namespace
{
    TrailParam_t TrailParams = DEFAULT_TRAIL_PARAM;
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

    Trail_stream = Stream::Create(PVA0, VPU0);
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
 * \brief Trail processing main function in host-side C++ API
*/
void trail_main()
{
    try
    {
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(trail_dev),
                                             PVA_EXECUTABLE_SIZE(trail_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        prog["algorithmParams"].set((int *)&TrailParams, sizeof(TrailParam_t));
        RasterDataFlow &sourceDistDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler     = prog["sourceDistDataFlowHandler"];
        uint16_t *inputDistBufferVMEM   = prog["inputDistBufferVMEM"].ptr<uint16_t>();

        RasterDataFlow &destinationDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler     = prog["destinationDataFlowHandler"];
        uint16_t *outputValidBufferVMEM       = prog["outputValidBufferVMEM"].ptr<uint16_t>();

        sourceDistDataFlow.handler(sourceDistDataFlowHandler)
            .src(DistIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        destinationDataFlow.handler(destinationDataFlowHandler)
            .dst(ValidOut_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputValidBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        prog.compileDataFlows();

        SetVPUPrintBufferSize(64 * 1024);
        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        /** Bind trail algorithm to VPU0 */
        
        CmdStatus status[2];
        Trail_stream.submit({&prog, &rf}, status);
        fence.wait();
        memcpy(&TrailMask[0][0], ValidOut_h, VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
        int cnt = 0;
        for (int i = 0; i < VIEW_HEIGHT; i++) {
            for (int j = 0; j < VIEW_WIDTH; j++) {
                if (TrailMask[i][j] == 1) {
                    cnt++;
                }
            }
        }
        std::cout << "trail count: " << cnt << std::endl;
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
