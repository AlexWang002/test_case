/*******************************************************************************
 * \addtogroup denoise_programm
 * \{
 * \file denoise.cpp
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

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "denoise.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(denoise_dev)

uint16_t *denoise_dist_buffer_d = nullptr;
uint16_t *denoise_dist_buffer_h = nullptr;

int *denoise_mask_buffer_d = nullptr;
int *denoise_mask_buffer_h = nullptr;

namespace {
    NoiseParam_t NoiseParams = DEFAULT_DENOISE_PARAM;
}

int denoiseDataAlloc()
{
    try
    {
        denoise_dist_buffer_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        denoise_dist_buffer_h = (uint16_t *)mem::GetHostPointer(denoise_dist_buffer_d);

        denoise_mask_buffer_d = (int *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(int));
        denoise_mask_buffer_h = (int *)mem::GetHostPointer(denoise_mask_buffer_d);
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

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


int denoiseProcPva()
{
    try
    {
        /*host端mask缓冲区清0*/
        memset(denoise_mask_buffer_h, 0, TILE_WIDTH * VIEW_HEIGHT * sizeof(int));

        //构建可执行文件
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(denoise_dev),
                                             PVA_EXECUTABLE_SIZE(denoise_dev));


        Stream stream = Stream::Create(PVA0, VPU0);

        CmdProgram prog = CmdProgram::Create(exec);

        /*设置去噪算法相关参数*/
        prog["algorithmParams"].set((int *)&NoiseParams, sizeof(NoiseParam_t));

        /*host将半帧dist数据传输给pva（分为8个tile）*/
        RasterDataFlow &src_dist_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto src_dist_dataflow_handler      = prog["src_dist_dataflow_handler"];       //数据流的传输由句柄触发
        uint16_t *input_dist_vmem           = prog["input_dist_vmem"].ptr<uint16_t>();

        src_dist_dataflow.handler(src_dist_dataflow_handler)
                            .src(denoise_dist_buffer_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(input_dist_vmem)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT);

        /*pva将计算好的mask结果传回host端（分为8个tile）*/
        RasterDataFlow &dst_mask_dataflow   = prog.addDataFlowHead<RasterDataFlow>();   //向CmdProgram中添加一个RasterDataFlow
        auto dst_mask_dataflow_handler      = prog["dst_mask_dataflow_handler"];        //数据流的传输由句柄触发
        int *output_mask_vmem               = prog["output_mask_vmem"].ptr<int>();

        dst_mask_dataflow.handler(dst_mask_dataflow_handler)
                            .dst(denoise_mask_buffer_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(output_mask_vmem)
                            .tile(TILE_WIDTH, TILE_HEIGHT);
        
        /*传输最后4列mask数据*/
        RasterDataFlow &dst_last_mask_dataflow   = prog.addDataFlowHead<RasterDataFlow>();   //向CmdProgram中添加一个RasterDataFlow
        auto dst_last_mask_dataflow_handler      = prog["dst_last_mask_dataflow_handler"];        //数据流的传输由句柄触发
        int *output_last_mask_vmem               = prog["output_last_mask_vmem"].ptr<int>();

        dst_last_mask_dataflow.handler(dst_last_mask_dataflow_handler)
                            .dst(denoise_mask_buffer_d, TILE_WIDTH, 4, TILE_WIDTH)
                            .tileBuffer(output_last_mask_vmem)
                            .tile(TILE_WIDTH, 4);

        prog.compileDataFlows();

        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};

        CmdStatus status[2];
        stream.submit({&prog, &rf}, status);
        fence.wait();
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

