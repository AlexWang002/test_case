/*******************************************************************************
 * \addtogroup upsample_programm
 * \{
 * \file upsample.cpp
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
#include "upsample.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(upsample_dev)

uint16_t DistOutUp[VIEW_HEIGHT][VIEW_WIDTH] = {0};
uint8_t RefOutUp[VIEW_HEIGHT][VIEW_WIDTH] = {0};

uint16_t *DistDownIn_d = nullptr;
uint16_t *DistDownIn_h = nullptr;
uint16_t *DistRawIn_d = nullptr;
uint16_t *DistRawIn_h = nullptr;
uint8_t *RefDownIn_d = nullptr;
uint8_t *RefDownIn_h = nullptr;
uint8_t *RefRawIn_d = nullptr;
uint8_t *RefRawIn_h = nullptr;
uint16_t *DistOutUp_d = nullptr;
uint16_t *DistOutUp_h = nullptr;
uint8_t *RefOutUp_d = nullptr;
uint8_t *RefOutUp_h = nullptr;

Stream upsample_stream;
namespace {
    InsertParam_t Up_param = DEFAULT_UP_PARAM;
}
/**
 * \brief Allocate memory for upsample processing data structures
*/
void upsampleDataAlloc()
{
    DistDownIn_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistDownIn_h = (uint16_t *)mem::GetHostPointer(DistDownIn_d);

    DistRawIn_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistRawIn_h = (uint16_t *)mem::GetHostPointer(DistRawIn_d);

    RefDownIn_d = (uint8_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint8_t));
    RefDownIn_h = (uint8_t *)mem::GetHostPointer(RefDownIn_d);

    RefRawIn_d = (uint8_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint8_t));
    RefRawIn_h = (uint8_t *)mem::GetHostPointer(RefRawIn_d);

    DistOutUp_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistOutUp_h = (uint16_t *)mem::GetHostPointer(DistOutUp_d);

    RefOutUp_d = (uint8_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint8_t));
    RefOutUp_h = (uint8_t *)mem::GetHostPointer(RefOutUp_d);

    upsample_stream = Stream::Create(PVA0, VPU0);
}

/**
 * \brief Free memory for upsample processing data structures
*/
void upsampleDataFree()
{
    mem::Free(DistDownIn_d);
    mem::Free(DistRawIn_d);
    mem::Free(RefDownIn_d);
    mem::Free(RefRawIn_d);
    mem::Free(DistOutUp_d);
    mem::Free(RefOutUp_d);
}

/**
 * \brief Upsample processing main function in host-side C++ API
*/
void upsample_main()
{
    try
    {
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(upsample_dev),
                                             PVA_EXECUTABLE_SIZE(upsample_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        prog["algorithmParams"].set((int *)&Up_param, sizeof(InsertParam_t));

        RasterDataFlow &InputDistDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto InputDistDataFlowHandler     = prog["InputDistDataFlowHandler"];
        uint16_t *inputDistBufferVMEM   = prog["inputDistBufferVMEM"].ptr<uint16_t>();
        InputDistDataFlow.handler(InputDistDataFlowHandler)
            .src(DistDownIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &InputDistRawDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto InputDistRawDataFlowHandler     = prog["InputDistRawDataFlowHandler"];
        uint16_t *inputDistRawBufferVMEM   = prog["inputDistRawBufferVMEM"].ptr<uint16_t>();
        InputDistRawDataFlow.handler(InputDistRawDataFlowHandler)
            .src(DistRawIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistRawBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &InputRefDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto InputRefDataFlowHandler     = prog["InputRefDataFlowHandler"];
        uint8_t *inputRefBufferVMEM   = prog["inputRefBufferVMEM"].ptr<uint8_t>();
        InputRefDataFlow.handler(InputRefDataFlowHandler)
            .src(RefDownIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &InputRefRawDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto InputRefRawDataFlowHandler     = prog["InputRefRawDataFlowHandler"];
        uint8_t *inputRefRawBufferVMEM   = prog["inputRefRawBufferVMEM"].ptr<uint8_t>();
        InputRefRawDataFlow.handler(InputRefRawDataFlowHandler)
            .src(RefRawIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefRawBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &OutputDistUpDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto OutputDistUpDataFlowHandler     = prog["OutputDistUpDataFlowHandler"];
        uint16_t *outputDistUpBufferVMEM       = prog["outputDistUpBufferVMEM"].ptr<uint16_t>();
        OutputDistUpDataFlow.handler(OutputDistUpDataFlowHandler)
            .dst(DistOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputDistUpBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &OutputRefUpDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto OutputRefUpDataFlowHandler     = prog["OutputRefUpDataFlowHandler"];
        uint8_t *outputRefUpBufferVMEM       = prog["outputRefUpBufferVMEM"].ptr<uint8_t>();
        OutputRefUpDataFlow.handler(OutputRefUpDataFlowHandler)
            .dst(RefOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRefUpBufferVMEM)
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
        
        CmdStatus status[2];
        upsample_stream.submit({&prog, &rf}, status);
        /** 等待Fence失效 */
        fence.wait();
        /** 将结果写入数组 */

        memcpy(&DistOutUp[0][0], DistOutUp_h, VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
        memcpy(&RefOutUp[0][0], RefOutUp_h, VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint8_t));

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
