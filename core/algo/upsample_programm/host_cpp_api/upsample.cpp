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

uint16_t *DistDownIn_d = nullptr;
uint16_t *DistDownIn_h = nullptr;
uint16_t *DistRawIn_d = nullptr;
uint16_t *DistRawIn_h = nullptr;
uint16_t *RefDownIn_d = nullptr;
uint16_t *RefDownIn_h = nullptr;
uint16_t *RefRawIn_d = nullptr;
uint16_t *RefRawIn_h = nullptr;
uint16_t *AttrIn_d = nullptr;
uint16_t *AttrIn_h = nullptr;
uint16_t *DistOutUp_d = nullptr;
uint16_t *DistOutUp_h = nullptr;
uint16_t *RefOutUp_d = nullptr;
uint16_t *RefOutUp_h = nullptr;
uint16_t *AttrOutUp_d = nullptr;
uint16_t *AttrOutUp_h = nullptr;

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

    RefDownIn_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    RefDownIn_h = (uint16_t *)mem::GetHostPointer(RefDownIn_d);

    RefRawIn_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    RefRawIn_h = (uint16_t *)mem::GetHostPointer(RefRawIn_d);

    AttrIn_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    AttrIn_h = (uint16_t *)mem::GetHostPointer(AttrIn_d);

    DistOutUp_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    DistOutUp_h = (uint16_t *)mem::GetHostPointer(DistOutUp_d);

    RefOutUp_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    RefOutUp_h = (uint16_t *)mem::GetHostPointer(RefOutUp_d);

    AttrOutUp_d = (uint16_t *)mem::Alloc(VIEW_HEIGHT * VIEW_WIDTH * sizeof(uint16_t));
    AttrOutUp_h = (uint16_t *)mem::GetHostPointer(AttrOutUp_d);
    upsample_stream = Stream::Create(PVA0, VPU1);
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
    mem::Free(AttrIn_d);
    mem::Free(DistOutUp_d);
    mem::Free(RefOutUp_d);
    mem::Free(AttrOutUp_d);
}

/**
 * \brief Upsample processing main function in host-side C++ API
*/
int upsample_main(std::string& exception_msg, int32_t& status_code)
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
        uint16_t *inputRefBufferVMEM   = prog["inputRefBufferVMEM"].ptr<uint16_t>();
        InputRefDataFlow.handler(InputRefDataFlowHandler)
            .src(RefDownIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &InputRefRawDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto InputRefRawDataFlowHandler     = prog["InputRefRawDataFlowHandler"];
        uint16_t *inputRefRawBufferVMEM   = prog["inputRefRawBufferVMEM"].ptr<uint16_t>();
        InputRefRawDataFlow.handler(InputRefRawDataFlowHandler)
            .src(RefRawIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefRawBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &InputAttrDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto InputAttrDataFlowHandler     = prog["InputAttrDataFlowHandler"];
        uint16_t *inputAttrBufferVMEM   = prog["inputAttrBufferVMEM"].ptr<uint16_t>();
        InputAttrDataFlow.handler(InputAttrDataFlowHandler)
            .src(AttrIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttrBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &OutputDistUpDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto OutputDistUpDataFlowHandler     = prog["OutputDistUpDataFlowHandler"];
        uint16_t *outputDistUpBufferVMEM       = prog["outputDistUpBufferVMEM"].ptr<uint16_t>();
        OutputDistUpDataFlow.handler(OutputDistUpDataFlowHandler)
            .dst(DistOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputDistUpBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &OutputRefUpDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto OutputRefUpDataFlowHandler     = prog["OutputRefUpDataFlowHandler"];
        uint16_t *outputRefUpBufferVMEM       = prog["outputRefUpBufferVMEM"].ptr<uint16_t>();
        OutputRefUpDataFlow.handler(OutputRefUpDataFlowHandler)
            .dst(RefOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRefUpBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &OutputAttrUpDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto OutputAttrDataFlowHandler     = prog["OutputAttrDataFlowHandler"];
        uint16_t *outputAttrBufferVMEM       = prog["outputAttrBufferVMEM"].ptr<uint16_t>();
        OutputAttrUpDataFlow.handler(OutputAttrDataFlowHandler)
            .dst(AttrOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputAttrBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        /** 编译程序数据流 */
        prog.compileDataFlows();

        SetVPUPrintBufferSize(64 * 1024);
        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];
        upsample_stream.submit({&prog, &rf}, status);
        /** 等待Fence失效 */
        fence.wait();
        /** 检查cmd状态 */
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
