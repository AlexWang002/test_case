/*******************************************************************************
 * \addtogroup spray_programm
 * \{
 * \file spray.cpp
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
#include "../../pva_utils.h"
#include <chrono>

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

namespace
{
    SprayParam_t SprayParams = DEFAULT_SPRAY_PARAM;
}

/**
 * \brief Get an Executable object instance
 *
 * \return An Executable object instance
 */
Executable& getSprayExec() {
    static Executable spray_exec = Executable::Create(
        PVA_EXECUTABLE_DATA(spray_dev),
        PVA_EXECUTABLE_SIZE(spray_dev)
    );
    return spray_exec;
}

/**
 * \brief Get an CmdProgram object instance
 *
 * \return An CmdProgram object instance
 */
CmdProgram& getSprayProg() {
    static CmdProgram spray_prog = CmdProgram::Create(getSprayExec());
    return spray_prog;
}

/**
 * \brief Get an Executable object instance
 *
 * \return An Executable object instance
 */
Executable& getEnhanceExec() {
    static Executable enhance_exec = Executable::Create(
        PVA_EXECUTABLE_DATA(enhance_dev),
        PVA_EXECUTABLE_SIZE(enhance_dev)
    );
    return enhance_exec;
}

/**
 * \brief Get an CmdProgram object instance
 *
 * \return An CmdProgram object instance
 */
CmdProgram& getEnhanceProg() {
    static CmdProgram enhance_prog = CmdProgram::Create(getEnhanceExec());
    return enhance_prog;
}

/**
 * \brief Compile the pva dataflow
 *
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
 */
int pvaSprayCompile()
{
    try
    {
        CmdProgram& spray_prog = getSprayProg();
        spray_prog["algorithmParams"].set((int *)&SprayParams, sizeof(SprayParam_t));

        RasterDataFlow &sourceDistDataFlow0 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler0 = spray_prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = spray_prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(DistIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceDistDataFlow1 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler1 = spray_prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = spray_prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(DistIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow0 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler0 = spray_prog["sourceRefDataFlowHandler0"];
        uint16_t *inputRefBufferVMEM0 = spray_prog["inputRefBufferVMEM0"].ptr<uint16_t>();

        sourceRefDataFlow0.handler(sourceRefDataFlowHandler0)
            .src(RefIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow1 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler1 = spray_prog["sourceRefDataFlowHandler1"];
        uint16_t *inputRefBufferVMEM1 = spray_prog["inputRefBufferVMEM1"].ptr<uint16_t>();

        sourceRefDataFlow1.handler(sourceRefDataFlowHandler1)
            .src(RefIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceAttDataFlow0 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceAttDataFlowHandler0 = spray_prog["sourceAttDataFlowHandler0"];
        uint16_t *inputAttBufferVMEM0 = spray_prog["inputAttBufferVMEM0"].ptr<uint16_t>();

        sourceAttDataFlow0.handler(sourceAttDataFlowHandler0)
            .src(AttIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceAttDataFlow1 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceAttDataFlowHandler1 = spray_prog["sourceAttDataFlowHandler1"];
        uint16_t *inputAttBufferVMEM1 = spray_prog["inputAttBufferVMEM1"].ptr<uint16_t>();

        sourceAttDataFlow1.handler(sourceAttDataFlowHandler1)
            .src(AttIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceGlinkDataFlow = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceGlkDataFlowHandler = spray_prog["sourceGlkDataFlowHandler"];
        uint16_t *inputGlkBufferVMEM = spray_prog["inputGlkBufferVMEM"].ptr<uint16_t>();

        sourceGlinkDataFlow.handler(sourceGlkDataFlowHandler)
            .src(Glink_d, VIEW_WIDTH, VIEW_HEIGHT * 4, VIEW_WIDTH)
            .tileBuffer(inputGlkBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT * 4);

        RasterDataFlow &destinationDataFlow0 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler0 = spray_prog["destinationDataFlowHandler0"];
        uint16_t *outputRainBufferVMEM0 = spray_prog["outputRainBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(RainOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRainBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = spray_prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler1 = spray_prog["destinationDataFlowHandler1"];
        uint16_t *outputRainBufferVMEM1 = spray_prog["outputRainBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(RainOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRainBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        spray_prog.compileDataFlows();
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Compile the pva dataflow
 *
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
 */
int pvaEnhanceCompile()
{
    try
    {
        CmdProgram& enhance_prog = getEnhanceProg();
        enhance_prog["algorithmParams"].set((int *)&SprayParams, sizeof(SprayParam_t));

        RasterDataFlow &sourceDistDataFlow0 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler0 = enhance_prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = enhance_prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(DistIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceDistDataFlow1 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler1 = enhance_prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = enhance_prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(DistIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow0 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler0 = enhance_prog["sourceRefDataFlowHandler0"];
        uint16_t *inputRefBufferVMEM0 = enhance_prog["inputRefBufferVMEM0"].ptr<uint16_t>();

        sourceRefDataFlow0.handler(sourceRefDataFlowHandler0)
            .src(RefIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &sourceRefDataFlow1 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler1 = enhance_prog["sourceRefDataFlowHandler1"];
        uint16_t *inputRefBufferVMEM1 = enhance_prog["inputRefBufferVMEM1"].ptr<uint16_t>();

        sourceRefDataFlow1.handler(sourceRefDataFlowHandler1)
            .src(RefIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &sourceGlkDataFlow = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceGlkDataFlowHandler = enhance_prog["sourceGlkDataFlowHandler"];
        uint16_t *inputGlkBufferVMEM = enhance_prog["inputGlkBufferVMEM"].ptr<uint16_t>();

        sourceGlkDataFlow.handler(sourceGlkDataFlowHandler)
            .src(Glink_d, VIEW_WIDTH, VIEW_HEIGHT * 4, VIEW_WIDTH)
            .tileBuffer(inputGlkBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT * 4);

        RasterDataFlow &sourceRainDataFlow0 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRainDataFlowHandler0 = enhance_prog["sourceRainDataFlowHandler0"];
        uint16_t *inputRainBufferVMEM0 = enhance_prog["inputRainBufferVMEM0"].ptr<uint16_t>();

        sourceRainDataFlow0.handler(sourceRainDataFlowHandler0)
            .src(RainOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRainBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRainDataFlow1 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRainDataFlowHandler1 = enhance_prog["sourceRainDataFlowHandler1"];
        uint16_t *inputRainBufferVMEM1 = enhance_prog["inputRainBufferVMEM1"].ptr<uint16_t>();

        sourceRainDataFlow1.handler(sourceRainDataFlowHandler1)
            .src(RainOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRainBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &destinationDataFlow0 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler0 = enhance_prog["destinationDataFlowHandler0"];
        uint16_t *outputFinalBufferVMEM0 = enhance_prog["outputFinalBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(FinalOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputFinalBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = enhance_prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler1 = enhance_prog["destinationDataFlowHandler1"];
        uint16_t *outputFinalBufferVMEM1 = enhance_prog["outputFinalBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(FinalOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputFinalBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        enhance_prog.compileDataFlows();
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Allocate memory for spray processing data structures
*/
int sprayDataAlloc()
{
    try
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

        if (pvaSprayCompile() != 0) {
            return 1;
        }

        if (pvaEnhanceCompile() != 0) {
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
 * \brief Free memory for spray processing data structures
*/
int sprayDataFree()
{
    try
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

        DistIn0_h = nullptr;
        DistIn1_h = nullptr;
        RefIn0_h = nullptr;
        RefIn1_h = nullptr;
        GndIn0_h = nullptr;
        GndIn1_h = nullptr;
        AttIn0_h = nullptr;
        AttIn1_h = nullptr;
        Glink_h = nullptr;
        RainOut0_h = nullptr;
        RainOut1_h = nullptr;
        FinalOut0_h = nullptr;
        FinalOut1_h = nullptr;
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Create and submit pva task for spray remove algo
 *
 * \param[in] exception_msg: Exception message
 *                 Range: NA. Accuracy: NA.
 *
 * \param[in] status_code: column index of the buffer
 *                 Range: 0-2. Accuracy: 1.
*/
int sprayRemovePva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        CmdProgram& spray_prog = getSprayProg();
        CmdStatus status[2];

        auto time5 = std::chrono::steady_clock::now();

        Stream& algo_stream = getAlgoStream();
        algo_stream.submit({&spray_prog}, status);

        auto time6 = std::chrono::steady_clock::now();

        //fence.wait(); // sprayremove task timeout: 6ms

        auto time7 = std::chrono::steady_clock::now();

        submit_time = std::chrono::duration_cast<std::chrono::microseconds>(time6 - time5).count();
        wait_time = std::chrono::duration_cast<std::chrono::microseconds>(time7 - time6).count();
    
        cupva::Error statusCode = CheckCommandStatus(status[0]);
        if (statusCode != Error::None && statusCode != Error::OperationPending)
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

/**
 * \brief Create and submit pva task for rain enhance algo
 *
 * \param[in] exception_msg: Exception message
 *                 Range: NA. Accuracy: NA.
 *
 * \param[in] status_code: column index of the buffer
 *                 Range: 0-2. Accuracy: 1.
*/
int rainEnhancePva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        CmdProgram& enhance_prog = getEnhanceProg();

        SyncObj sync = SyncObj::Create(true);
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];

        auto time5 = std::chrono::steady_clock::now();

        Stream& algo_stream = getAlgoStream();
        algo_stream.submit({&enhance_prog, &rf}, status);

        auto time6 = std::chrono::steady_clock::now();

        fence.wait(); // rainenhance task timeout: 3.5ms

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