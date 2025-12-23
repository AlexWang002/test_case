/*******************************************************************************
 * \addtogroup denoise_programm
 * \{
 * \file denoise.cpp
 * \brief
 * \version 0.3
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
 * | 0.2 | 2025-09-29 | Use vpu's Wide-SIMD vector processor to accelerate denoise algo|
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.3 | 2025-11-28 | Synchronize model modifications and add exception log messages|
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "denoise.h"
#include <chrono>

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(denoise_dev)

uint16_t *denoise_dist_buffer_d = nullptr;
uint16_t *denoise_dist_buffer_h = nullptr;

uint16_t *denoise_mask_buffer_d = nullptr;
uint16_t *denoise_mask_buffer_h = nullptr;

Stream denoise_stream;
namespace {
    NoiseParam_t NoiseParams = DEFAULT_DENOISE_PARAM;
}

/**
 * \brief Alloc memory for denoise processing data structures
*/
int denoiseDataAlloc()
{
    try
    {
        denoise_dist_buffer_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        denoise_dist_buffer_h = (uint16_t *)mem::GetHostPointer(denoise_dist_buffer_d);

        denoise_mask_buffer_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        denoise_mask_buffer_h = (uint16_t *)mem::GetHostPointer(denoise_mask_buffer_d);

        denoise_stream = Stream::Create(PVA0, VPU1);
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

/**
 * \brief Free memory for denoise processing data structures
*/
int denoiseDataFree()
{
    try
    {
        mem::Free(denoise_dist_buffer_d);
        mem::Free(denoise_mask_buffer_d);
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

/**
 * \brief Create and submit pva task for denoise algo
 *
 * \param[in] exception_msg: Exception message
 *                 Range: NA. Accuracy: NA.
 *
 * \param[in] status_code: column index of the buffer
 *                 Range: 0-2. Accuracy: 1.
*/
int denoiseProcPva(std::string& exception_msg, int32_t& status_code, 
    uint32_t& stage1, uint32_t& stage2, uint32_t& stage3, uint32_t& stage4,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        auto time1 = std::chrono::steady_clock::now();

        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(denoise_dev),
                                             PVA_EXECUTABLE_SIZE(denoise_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        auto time2 = std::chrono::steady_clock::now();

        prog["algorithmParams"].set((int *)&NoiseParams, sizeof(NoiseParam_t));

        RasterDataFlow &DistDataflow = prog.addDataFlowHead<RasterDataFlow>();
        auto DistHandler             = prog["DistHandler"];
        uint16_t *DistBuffer         = prog["DistBuffer"].ptr<uint16_t>();

        DistDataflow.handler(DistHandler)
                            .src(denoise_dist_buffer_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(DistBuffer)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &MaskDataflow = prog.addDataFlowHead<RasterDataFlow>();
        auto MaskHandler             = prog["MaskHandler"];
        uint16_t *MaskBuffer         = prog["MaskBuffer"].ptr<uint16_t>();

        MaskDataflow.handler(MaskHandler)
                            .dst(denoise_mask_buffer_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(MaskBuffer)
                            .tile(TILE_WIDTH, TILE_HEIGHT);
        
        auto time3 = std::chrono::steady_clock::now();

        prog.compileDataFlows();

        auto time4 = std::chrono::steady_clock::now();

        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];
        
        auto time5 = std::chrono::steady_clock::now();
        
        denoise_stream.submit({&prog, &rf}, status);
        
        auto time6 = std::chrono::steady_clock::now();
        
        fence.wait(); // denoise task timeout: 2.5 ms
        
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

