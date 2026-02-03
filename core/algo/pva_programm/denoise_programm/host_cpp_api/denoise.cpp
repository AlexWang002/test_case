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
#include "../../pva_utils.h"
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

namespace {
    NoiseParam_t NoiseParams = DEFAULT_DENOISE_PARAM;
}

/**
 * \brief Get an Executable object instance
 *
 * \return An Executable object instance
 */
Executable& getDenoiseExec() {
    static Executable denoise_exec = Executable::Create(
        PVA_EXECUTABLE_DATA(denoise_dev),
        PVA_EXECUTABLE_SIZE(denoise_dev)
    );
    return denoise_exec;
}

/**
 * \brief Get an CmdProgram object instance
 *
 * \return An CmdProgram object instance
 */
CmdProgram& getDenoiseProg() {
    static CmdProgram denoise_prog = CmdProgram::Create(getDenoiseExec());
    return denoise_prog;
}

/**
 * \brief Compile the pva dataflow
 *
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
 */
int pvaDenoiseCompile()
{
    try
    {
        CmdProgram& denoise_prog = getDenoiseProg();
        denoise_prog["algorithmParams"].set((int *)&NoiseParams, sizeof(NoiseParam_t));

        RasterDataFlow &DistDataflow = denoise_prog.addDataFlowHead<RasterDataFlow>();
        auto DistHandler             = denoise_prog["DistHandler"];
        uint16_t *DistBuffer         = denoise_prog["DistBuffer"].ptr<uint16_t>();

        DistDataflow.handler(DistHandler)
                            .src(denoise_dist_buffer_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(DistBuffer)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        RasterDataFlow &MaskDataflow = denoise_prog.addDataFlowHead<RasterDataFlow>();
        auto MaskHandler             = denoise_prog["MaskHandler"];
        uint16_t *MaskBuffer         = denoise_prog["MaskBuffer"].ptr<uint16_t>();

        MaskDataflow.handler(MaskHandler)
                            .dst(denoise_mask_buffer_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(MaskBuffer)
                            .tile(TILE_WIDTH, TILE_HEIGHT);

        denoise_prog.compileDataFlows();
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Alloc memory for denoise processing data structures
 *
 * \return Error code
 * \retval 0: PVA task submitted successed
 * \retval 1: Caught a cuPVA exceptions
*/
int denoiseDataAlloc()
{
    try
    {
        denoise_dist_buffer_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        denoise_dist_buffer_h = (uint16_t *)mem::GetHostPointer(denoise_dist_buffer_d);

        denoise_mask_buffer_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        denoise_mask_buffer_h = (uint16_t *)mem::GetHostPointer(denoise_mask_buffer_d);

        if (pvaDenoiseCompile() != 0) {
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
 * \brief Free memory for denoise processing data structures
 *
 * \return Error code
 * \retval 0: PVA task submitted successed
 * \retval 1: Caught a cuPVA exceptions
*/
int denoiseDataFree()
{
    try
    {
        mem::Free(denoise_dist_buffer_d);
        mem::Free(denoise_mask_buffer_d);

        denoise_dist_buffer_h = nullptr;
        denoise_mask_buffer_h = nullptr;
    }
    catch (cupva::Exception const &e)
    {
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
 *
 * \return Error code
 * \retval 0: PVA task submitted successed
 * \retval 1: Caught a cuPVA exceptions
 * \retval 2: VPU Program returned an Error Code
*/
int denoiseProcPva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        CmdProgram& denoise_prog = getDenoiseProg();
        CmdStatus status[2];

        auto time5 = std::chrono::steady_clock::now();

        Stream& algo_stream = getAlgoStream();
        algo_stream.submit({&denoise_prog}, status);

        auto time6 = std::chrono::steady_clock::now();

        auto time7 = std::chrono::steady_clock::now();

        submit_time = std::chrono::duration_cast<std::chrono::microseconds>(time6 - time5).count();
        wait_time = std::chrono::duration_cast<std::chrono::microseconds>(time7 - time6).count();

        cupva::Error statusCode = CheckCommandStatus(status[0]);
        if ((statusCode != Error::None) && (statusCode != Error::OperationPending))
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
