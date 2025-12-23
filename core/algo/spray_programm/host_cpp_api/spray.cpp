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
 * \brief Create and submit pva task for spray remove algo
 *
 * \param[in] exception_msg: Exception message
 *                 Range: NA. Accuracy: NA.
 *
 * \param[in] status_code: column index of the buffer
 *                 Range: 0-2. Accuracy: 1.
*/
int sprayRemovePva(std::string& exception_msg, int32_t& status_code, 
    uint32_t& stage1, uint32_t& stage2, uint32_t& stage3, uint32_t& stage4,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        auto time1 = std::chrono::steady_clock::now();

        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(spray_dev),
                                             PVA_EXECUTABLE_SIZE(spray_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        auto time2 = std::chrono::steady_clock::now();

        prog["algorithmParams"].set((int *)&SprayParams, sizeof(SprayParam_t));

        RasterDataFlow &sourceDistDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler0 = prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(DistIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceDistDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler1 = prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(DistIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler0 = prog["sourceRefDataFlowHandler0"];
        uint16_t *inputRefBufferVMEM0 = prog["inputRefBufferVMEM0"].ptr<uint16_t>();

        sourceRefDataFlow0.handler(sourceRefDataFlowHandler0)
            .src(RefIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler1 = prog["sourceRefDataFlowHandler1"];
        uint16_t *inputRefBufferVMEM1 = prog["inputRefBufferVMEM1"].ptr<uint16_t>();

        sourceRefDataFlow1.handler(sourceRefDataFlowHandler1)
            .src(RefIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceAttDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceAttDataFlowHandler0 = prog["sourceAttDataFlowHandler0"];
        uint16_t *inputAttBufferVMEM0 = prog["inputAttBufferVMEM0"].ptr<uint16_t>();

        sourceAttDataFlow0.handler(sourceAttDataFlowHandler0)
            .src(AttIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceAttDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceAttDataFlowHandler1 = prog["sourceAttDataFlowHandler1"];
        uint16_t *inputAttBufferVMEM1 = prog["inputAttBufferVMEM1"].ptr<uint16_t>();

        sourceAttDataFlow1.handler(sourceAttDataFlowHandler1)
            .src(AttIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputAttBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceGlinkDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceGlkDataFlowHandler = prog["sourceGlkDataFlowHandler"];
        uint16_t *inputGlkBufferVMEM = prog["inputGlkBufferVMEM"].ptr<uint16_t>();

        sourceGlinkDataFlow.handler(sourceGlkDataFlowHandler)
            .src(Glink_d, VIEW_WIDTH, VIEW_HEIGHT * 4, VIEW_WIDTH)
            .tileBuffer(inputGlkBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT * 4);

        RasterDataFlow &destinationDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler0 = prog["destinationDataFlowHandler0"];
        uint16_t *outputRainBufferVMEM0 = prog["outputRainBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(RainOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRainBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler1 = prog["destinationDataFlowHandler1"];
        uint16_t *outputRainBufferVMEM1 = prog["outputRainBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(RainOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputRainBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        auto time3 = std::chrono::steady_clock::now();
        
        prog.compileDataFlows();

        auto time4 = std::chrono::steady_clock::now();

        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];
        
        auto time5 = std::chrono::steady_clock::now();

        Spray_stream.submit({&prog, &rf}, status);
        
        auto time6 = std::chrono::steady_clock::now();

        fence.wait(); // sprayremove task timeout: 6ms
        
        auto time7 = std::chrono::steady_clock::now();

        stage1 = std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count();
        stage2 = std::chrono::duration_cast<std::chrono::microseconds>(time3 - time2).count();
        stage3 = std::chrono::duration_cast<std::chrono::microseconds>(time4 - time3).count();
        stage4 = std::chrono::duration_cast<std::chrono::microseconds>(time5 - time4).count();
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
    uint32_t& stage1, uint32_t& stage2, uint32_t& stage3, uint32_t& stage4,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        auto time1 = std::chrono::steady_clock::now();

        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(enhance_dev),
                                             PVA_EXECUTABLE_SIZE(enhance_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        auto time2 = std::chrono::steady_clock::now();

        prog["algorithmParams"].set((int *)&SprayParams, sizeof(SprayParam_t));

        RasterDataFlow &sourceDistDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler0 = prog["sourceDistDataFlowHandler0"];
        uint16_t *inputDistBufferVMEM0 = prog["inputDistBufferVMEM0"].ptr<uint16_t>();

        sourceDistDataFlow0.handler(sourceDistDataFlowHandler0)
            .src(DistIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceDistDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceDistDataFlowHandler1 = prog["sourceDistDataFlowHandler1"];
        uint16_t *inputDistBufferVMEM1 = prog["inputDistBufferVMEM1"].ptr<uint16_t>();

        sourceDistDataFlow1.handler(sourceDistDataFlowHandler1)
            .src(DistIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputDistBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRefDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler0 = prog["sourceRefDataFlowHandler0"];
        uint16_t *inputRefBufferVMEM0 = prog["inputRefBufferVMEM0"].ptr<uint16_t>();

        sourceRefDataFlow0.handler(sourceRefDataFlowHandler0)
            .src(RefIn0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &sourceRefDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRefDataFlowHandler1 = prog["sourceRefDataFlowHandler1"];
        uint16_t *inputRefBufferVMEM1 = prog["inputRefBufferVMEM1"].ptr<uint16_t>();

        sourceRefDataFlow1.handler(sourceRefDataFlowHandler1)
            .src(RefIn1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRefBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &sourceGlkDataFlow = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceGlkDataFlowHandler = prog["sourceGlkDataFlowHandler"];
        uint16_t *inputGlkBufferVMEM = prog["inputGlkBufferVMEM"].ptr<uint16_t>();

        sourceGlkDataFlow.handler(sourceGlkDataFlowHandler)
            .src(Glink_d, VIEW_WIDTH, VIEW_HEIGHT * 4, VIEW_WIDTH)
            .tileBuffer(inputGlkBufferVMEM)
            .tile(VIEW_WIDTH, TILE_HEIGHT * 4);

        RasterDataFlow &sourceRainDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRainDataFlowHandler0 = prog["sourceRainDataFlowHandler0"];
        uint16_t *inputRainBufferVMEM0 = prog["inputRainBufferVMEM0"].ptr<uint16_t>();

        sourceRainDataFlow0.handler(sourceRainDataFlowHandler0)
            .src(RainOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRainBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &sourceRainDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto sourceRainDataFlowHandler1 = prog["sourceRainDataFlowHandler1"];
        uint16_t *inputRainBufferVMEM1 = prog["inputRainBufferVMEM1"].ptr<uint16_t>();

        sourceRainDataFlow1.handler(sourceRainDataFlowHandler1)
            .src(RainOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(inputRainBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT)
            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &destinationDataFlow0 = prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler0 = prog["destinationDataFlowHandler0"];
        uint16_t *outputFinalBufferVMEM0 = prog["outputFinalBufferVMEM0"].ptr<uint16_t>();

        destinationDataFlow0.handler(destinationDataFlowHandler0)
            .dst(FinalOut0_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputFinalBufferVMEM0)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        RasterDataFlow &destinationDataFlow1 = prog.addDataFlowHead<RasterDataFlow>();
        auto destinationDataFlowHandler1 = prog["destinationDataFlowHandler1"];
        uint16_t *outputFinalBufferVMEM1 = prog["outputFinalBufferVMEM1"].ptr<uint16_t>();

        destinationDataFlow1.handler(destinationDataFlowHandler1)
            .dst(FinalOut1_d, VIEW_WIDTH, VIEW_HEIGHT, VIEW_WIDTH)
            .tileBuffer(outputFinalBufferVMEM1)
            .tile(VIEW_WIDTH, TILE_HEIGHT);

        auto time3 = std::chrono::steady_clock::now();
        
        prog.compileDataFlows();

        auto time4 = std::chrono::steady_clock::now();

        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];
        
        auto time5 = std::chrono::steady_clock::now();

        Spray_stream.submit({&prog, &rf}, status, IN_ORDER, 3500, 3000);
        
        auto time6 = std::chrono::steady_clock::now();

        fence.wait(); // rainenhance task timeout: 3.5ms
        
        auto time7 = std::chrono::steady_clock::now();

        stage1 = std::chrono::duration_cast<std::chrono::microseconds>(time2 - time1).count();
        stage2 = std::chrono::duration_cast<std::chrono::microseconds>(time3 - time2).count();
        stage3 = std::chrono::duration_cast<std::chrono::microseconds>(time4 - time3).count();
        stage4 = std::chrono::duration_cast<std::chrono::microseconds>(time5 - time4).count();
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