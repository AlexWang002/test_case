/*******************************************************************************
 * \addtogroup upsample_programm
 * \{
 * \file upsample.cpp
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
 * | 0.2 | 2025-11-28 | add exception log messages|
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
#include <chrono>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "upsample.h"
#include "../../pva_utils.h"

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

namespace {
    InsertParam_t Up_param = DEFAULT_UP_PARAM;
}

/**
 * \brief Get an Executable object instance
 *
 * \return An Executable object instance
 */
Executable& getUpsampleExec() {
    static Executable upsample_exec = Executable::Create(
        PVA_EXECUTABLE_DATA(upsample_dev),
        PVA_EXECUTABLE_SIZE(upsample_dev)
    );
    return upsample_exec;
}

/**
 * \brief Get an CmdProgram object instance
 *
 * \return An CmdProgram object instance
 */
CmdProgram& getUpsampleProg() {
    static CmdProgram upsample_prog = CmdProgram::Create(getUpsampleExec());
    return upsample_prog;
}

/**
 * \brief Compile the pva dataflow
 *
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
 */
int pvaUpsampleCompile()
{
    try
    {
        CmdProgram& upsample_prog = getUpsampleProg();
        upsample_prog["algorithmParams"].set((int *)&Up_param, sizeof(InsertParam_t));

        RasterDataFlow &InputDistDataFlow    = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto InputDistDataFlowHandler        = upsample_prog["InputDistDataFlowHandler"];
        uint16_t *inputDistBufferVMEM        = upsample_prog["inputDistBufferVMEM"].ptr<uint16_t>();
        InputDistDataFlow.handler(InputDistDataFlowHandler)
            .src(DistDownIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &InputDistRawDataFlow = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto InputDistRawDataFlowHandler     = upsample_prog["InputDistRawDataFlowHandler"];
        uint16_t *inputDistRawBufferVMEM     = upsample_prog["inputDistRawBufferVMEM"].ptr<uint16_t>();
        InputDistRawDataFlow.handler(InputDistRawDataFlowHandler)
            .src(DistRawIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistRawBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &InputRefDataFlow     = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto InputRefDataFlowHandler         = upsample_prog["InputRefDataFlowHandler"];
        uint16_t *inputRefBufferVMEM         = upsample_prog["inputRefBufferVMEM"].ptr<uint16_t>();
        InputRefDataFlow.handler(InputRefDataFlowHandler)
            .src(RefDownIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &InputRefRawDataFlow  = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto InputRefRawDataFlowHandler      = upsample_prog["InputRefRawDataFlowHandler"];
        uint16_t *inputRefRawBufferVMEM      = upsample_prog["inputRefRawBufferVMEM"].ptr<uint16_t>();
        InputRefRawDataFlow.handler(InputRefRawDataFlowHandler)
            .src(RefRawIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefRawBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &InputAttrDataFlow    = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto InputAttrDataFlowHandler        = upsample_prog["InputAttrDataFlowHandler"];
        uint16_t *inputAttrBufferVMEM        = upsample_prog["inputAttrBufferVMEM"].ptr<uint16_t>();
        InputAttrDataFlow.handler(InputAttrDataFlowHandler)
            .src(AttrIn_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttrBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &OutputDistUpDataFlow = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto OutputDistUpDataFlowHandler     = upsample_prog["OutputDistUpDataFlowHandler"];
        uint16_t *outputDistUpBufferVMEM     = upsample_prog["outputDistUpBufferVMEM"].ptr<uint16_t>();
        OutputDistUpDataFlow.handler(OutputDistUpDataFlowHandler)
            .dst(DistOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputDistUpBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &OutputRefUpDataFlow  = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto OutputRefUpDataFlowHandler      = upsample_prog["OutputRefUpDataFlowHandler"];
        uint16_t *outputRefUpBufferVMEM      = upsample_prog["outputRefUpBufferVMEM"].ptr<uint16_t>();
        OutputRefUpDataFlow.handler(OutputRefUpDataFlowHandler)
            .dst(RefOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRefUpBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &OutputAttrUpDataFlow = upsample_prog.addDataFlowHead<RasterDataFlow>();
        auto OutputAttrDataFlowHandler       = upsample_prog["OutputAttrDataFlowHandler"];
        uint16_t *outputAttrBufferVMEM       = upsample_prog["outputAttrBufferVMEM"].ptr<uint16_t>();
        OutputAttrUpDataFlow.handler(OutputAttrDataFlowHandler)
            .dst(AttrOutUp_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputAttrBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        upsample_prog.compileDataFlows();
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;

}

/**
 * \brief Allocate memory for upsample processing data structures
 * 
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
*/
int upsampleDataAlloc()
{
    try
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

        if (pvaUpsampleCompile() != 0) {
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
 * \brief Free memory for upsample processing data structures
 * 
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
*/
int upsampleDataFree()
{
    try 
    {
        mem::Free(DistDownIn_d);
        mem::Free(DistRawIn_d);
        mem::Free(RefDownIn_d);
        mem::Free(RefRawIn_d);
        mem::Free(AttrIn_d);
        mem::Free(DistOutUp_d);
        mem::Free(RefOutUp_d);
        mem::Free(AttrOutUp_d);

        DistDownIn_h = nullptr;
        DistRawIn_h = nullptr;
        RefDownIn_h = nullptr;
        RefRawIn_h = nullptr;
        AttrIn_h = nullptr;
        DistOutUp_h = nullptr;
        RefOutUp_h = nullptr;
        AttrOutUp_h = nullptr;
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Create and submit pva task for upsample algo
 *
 * \param[in] exception_msg: Exception message
 *                 Range: NA. Accuracy: NA.
 *
 * \param[in] status_code: column index of the buffer
 *                 Range: 0-2. Accuracy: 1.
 * 
 * \return Error code
 * \retval 0: PVA task submitted successed
 * \retval 1: Caught a cuPVA exceptions
 * \retval 2: VPU Program returned an Error Code
*/
int upsampleProcPva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        CmdProgram& upsample_prog = getUpsampleProg();

        SyncObj sync = SyncObj::Create(true);
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];

        auto time5 = std::chrono::steady_clock::now();

        Stream& algo_stream = getAlgoStream();
        algo_stream.submit({&upsample_prog, &rf}, status);

        auto time6 = std::chrono::steady_clock::now();

        fence.wait();

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
