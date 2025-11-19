/*******************************************************************************
 * \addtogroup stray_programm
 * \{
 * \file stray.cpp
 * \brief
 * \version 0.1
 * \date 2025-10-13
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-11 | Init version |
 * 
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "stray.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(stray_dev)

StrayPvaBuffer stray_pva_buff;
Stream stray_stream;

int strayBufferAlloc()
{
    try
    {
        stray_pva_buff.dist_wave_d = (uint16_t *)mem::Alloc(2 * TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.dist_wave_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.dist_wave_d);

        stray_pva_buff.att0_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.att0_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.att0_d);

        stray_pva_buff.att1_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.att1_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.att1_d);

        stray_pva_buff.class_line_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.class_line_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.class_line_d);

        stray_pva_buff.ground_height_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.ground_height_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.ground_height_d);

        // refl_wave0, refl_wave1, grnd_wave0, grnd_wave1, high_wave0, high_wave1
        stray_pva_buff.raw_data_d = (uint16_t *)mem::Alloc(RAW_DATA_CNT * TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.raw_data_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.raw_data_d);

        stray_pva_buff.stray_var_d = (uint16_t *)mem::Alloc(STRAY_VAR_CNT * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.stray_var_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.stray_var_d);

        stray_pva_buff.stray_mask0_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.stray_mask0_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.stray_mask0_d);

        stray_pva_buff.stray_mask1_d = (uint16_t *)mem::Alloc(TILE_WIDTH * VIEW_HEIGHT * sizeof(uint16_t));
        stray_pva_buff.stray_mask1_h = (uint16_t *)mem::GetHostPointer(stray_pva_buff.stray_mask1_d);

        stray_stream = Stream::Create(PVA0, VPU1);
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

int strayBufferRelease()
{
    try
    {
        mem::Free(stray_pva_buff.dist_wave_d);
        mem::Free(stray_pva_buff.att0_d);
        mem::Free(stray_pva_buff.att1_d);
        mem::Free(stray_pva_buff.class_line_d);
        mem::Free(stray_pva_buff.ground_height_d);
        mem::Free(stray_pva_buff.raw_data_d);
        mem::Free(stray_pva_buff.stray_var_d);
        mem::Free(stray_pva_buff.stray_mask0_d);
        mem::Free(stray_pva_buff.stray_mask1_d);
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

/**
 * \brief Create and submit pva task for stray algo
*/
int strayProcPva(int rainwall_cnt, int rainwall_dist)
{
    try
    {
        //构建可执行文件
        Executable exec = Executable::Create(PVA_EXECUTABLE_DATA(stray_dev),
                                             PVA_EXECUTABLE_SIZE(stray_dev));

        CmdProgram prog = CmdProgram::Create(exec);

        // 设置杂散算法相关参数
        prog["rainwall_dist"].set((int *)&rainwall_dist, sizeof(int));
        prog["rainwall_cnt"].set((int *)&rainwall_cnt, sizeof(int));

        RasterDataFlow &stray_var_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto stray_var_handler               = prog["stray_var_handler"];       //数据流的传输由句柄触发
        uint16_t *stray_var                  = prog["stray_var"].ptr<uint16_t>();

        stray_var_dataflow.handler(stray_var_handler)
                            .src(stray_pva_buff.stray_var_d, STRAY_VAR_CNT, VIEW_HEIGHT, STRAY_VAR_CNT)
                            .tileBuffer(stray_var)
                            .tile(STRAY_VAR_CNT, VIEW_HEIGHT);

        RasterDataFlow &dist_wave_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto dist_wave_handler               = prog["dist_wave_handler"];       //数据流的传输由句柄触发
        uint16_t *dist_wave                  = prog["dist_wave"].ptr<uint16_t>();

        dist_wave_dataflow.handler(dist_wave_handler)
                            .src(stray_pva_buff.dist_wave_d, TILE_WIDTH, VIEW_HEIGHT * 2, TILE_WIDTH)
                            .tileBuffer(dist_wave)
                            .tile(TILE_WIDTH, TILE_HEIGHT * 2)
                            .halo(KR_W_DIST, KR_H_DIST);
        
        RasterDataFlow &att0_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto att0_handler               = prog["att0_handler"];       //数据流的传输由句柄触发
        uint16_t *att0                  = prog["att0"].ptr<uint16_t>();

        att0_dataflow.handler(att0_handler)
                            .src(stray_pva_buff.att0_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(att0)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KR_W_PEAK, KR_H_PEAK);
        
        RasterDataFlow &att1_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto att1_handler               = prog["att1_handler"];       //数据流的传输由句柄触发
        uint16_t *att1                  = prog["att1"].ptr<uint16_t>();

        att1_dataflow.handler(att1_handler)
                            .src(stray_pva_buff.att1_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(att1)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KR_W_PEAK, KR_H_PEAK);
        
        RasterDataFlow &class_line_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto class_line_handler               = prog["class_line_handler"];       //数据流的传输由句柄触发
        uint16_t *class_line                  = prog["class_line"].ptr<uint16_t>();

        class_line_dataflow.handler(class_line_handler)
                            .src(stray_pva_buff.class_line_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(class_line)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KR_W_CLASS, KR_H_CLASS);
        
        RasterDataFlow &ground_height_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto ground_height_handler               = prog["ground_height_handler"];       //数据流的传输由句柄触发
        uint16_t *ground_height                  = prog["ground_height"].ptr<uint16_t>();

        ground_height_dataflow.handler(ground_height_handler)
                            .src(stray_pva_buff.ground_height_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(ground_height)
                            .tile(TILE_WIDTH, TILE_HEIGHT);
        
        RasterDataFlow &raw_data_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto raw_data_handler               = prog["raw_data_handler"];       //数据流的传输由句柄触发
        uint16_t *raw_data                  = prog["raw_data"].ptr<uint16_t>();

        raw_data_dataflow.handler(raw_data_handler)
                            .src(stray_pva_buff.raw_data_d, TILE_WIDTH, VIEW_HEIGHT * RAW_DATA_CNT, TILE_WIDTH)
                            .tileBuffer(raw_data)
                            .tile(TILE_WIDTH, TILE_HEIGHT * RAW_DATA_CNT);
        
        RasterDataFlow &stray_mask0_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto stray_mask0_handler               = prog["stray_mask0_handler"];       //数据流的传输由句柄触发
        uint16_t *stray_mask0                  = prog["stray_mask0"].ptr<uint16_t>();

        stray_mask0_dataflow.handler(stray_mask0_handler)
                            .dst(stray_pva_buff.stray_mask0_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(stray_mask0)
                            .tile(TILE_WIDTH, TILE_HEIGHT);
        
        RasterDataFlow &stray_mask1_dataflow   = prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto stray_mask1_handler               = prog["stray_mask1_handler"];       //数据流的传输由句柄触发
        uint16_t *stray_mask1                  = prog["stray_mask1"].ptr<uint16_t>();

        stray_mask1_dataflow.handler(stray_mask1_handler)
                            .dst(stray_pva_buff.stray_mask1_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(stray_mask1)
                            .tile(TILE_WIDTH, TILE_HEIGHT);
        
        prog.compileDataFlows();

        SyncObj sync = SyncObj::Create();
        Fence fence{sync};
        CmdRequestFences rf{fence};

        CmdStatus status[2];
        stray_stream.submit({&prog, &rf}, status);
        fence.wait();
    }
    catch (cupva::Exception const &e)
    {
        std::cout << "Caught a cuPVA exception with message: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}