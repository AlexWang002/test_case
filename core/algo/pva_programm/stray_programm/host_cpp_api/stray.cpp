/*******************************************************************************
 * \addtogroup stray_programm
 * \{
 * \file stray.cpp
 * \brief
 * \version 0.2
 * \date 2025-12-02
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-11 | Init version |
 * | 0.2 | 2025-12-02 | Add comments |
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
#include "../../pva_utils.h"
#include <chrono>

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace cupva;

PVA_DECLARE_EXECUTABLE(stray_dev)

StrayPvaBuffer stray_pva_buff;

/**
 * \brief Get an Executable object instance
 *
 * \return An Executable object instance
 */
Executable& getStrayExec() {
    static Executable stray_exec = Executable::Create(
        PVA_EXECUTABLE_DATA(stray_dev),
        PVA_EXECUTABLE_SIZE(stray_dev)
    );
    return stray_exec;
}

/**
 * \brief Get an CmdProgram object instance
 *
 * \return An CmdProgram object instance
 */
CmdProgram& getStrayProg() {
    static CmdProgram stray_prog = CmdProgram::Create(getStrayExec());
    return stray_prog;
}

/**
 * \brief Compile the pva dataflow
 *
 * \return Error code
 * \retval 0: Dataflow compiled successed
 * \retval 1: Caught a cuPVA exceptions
 */
int pvaStrayCompile()
{
    try
    {
        CmdProgram& stray_prog = getStrayProg();
        RasterDataFlow &stray_var_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto stray_var_handler               = stray_prog["stray_var_handler"];               //数据流的传输由句柄触发
        uint16_t *stray_var                  = stray_prog["stray_var"].ptr<uint16_t>();

        stray_var_dataflow.handler(stray_var_handler)
                            .src(stray_pva_buff.stray_var_d, STRAY_VAR_CNT, VIEW_HEIGHT, STRAY_VAR_CNT)
                            .tileBuffer(stray_var)
                            .tile(STRAY_VAR_CNT, VIEW_HEIGHT);

        RasterDataFlow &dist_wave_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto dist_wave_handler               = stray_prog["dist_wave_handler"];               //数据流的传输由句柄触发
        uint16_t *dist_wave                  = stray_prog["dist_wave"].ptr<uint16_t>();

        dist_wave_dataflow.handler(dist_wave_handler)
                            .src(stray_pva_buff.dist_wave_d, TILE_WIDTH, VIEW_HEIGHT * 2, TILE_WIDTH)
                            .tileBuffer(dist_wave)
                            .tile(TILE_WIDTH, TILE_HEIGHT * 2)
                            .halo(KR_W_DIST, KR_H_DIST);

        RasterDataFlow &att0_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();   //向CmdProgram中添加一个RasterDataFlow
        auto att0_handler               = stray_prog["att0_handler"];                     //数据流的传输由句柄触发
        uint16_t *att0                  = stray_prog["att0"].ptr<uint16_t>();

        att0_dataflow.handler(att0_handler)
                            .src(stray_pva_buff.att0_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(att0)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KR_W_PEAK, KR_H_PEAK);

        RasterDataFlow &att1_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();   //向CmdProgram中添加一个RasterDataFlow
        auto att1_handler               = stray_prog["att1_handler"];                     //数据流的传输由句柄触发
        uint16_t *att1                  = stray_prog["att1"].ptr<uint16_t>();

        att1_dataflow.handler(att1_handler)
                            .src(stray_pva_buff.att1_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(att1)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KR_W_PEAK, KR_H_PEAK);

        RasterDataFlow &class_line_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto class_line_handler               = stray_prog["class_line_handler"];              //数据流的传输由句柄触发
        uint16_t *class_line                  = stray_prog["class_line"].ptr<uint16_t>();

        class_line_dataflow.handler(class_line_handler)
                            .src(stray_pva_buff.class_line_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(class_line)
                            .tile(TILE_WIDTH, TILE_HEIGHT)
                            .halo(KR_W_CLASS, KR_H_CLASS);

        RasterDataFlow &ground_height_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto ground_height_handler               = stray_prog["ground_height_handler"];           //数据流的传输由句柄触发
        uint16_t *ground_height                  = stray_prog["ground_height"].ptr<uint16_t>();

        ground_height_dataflow.handler(ground_height_handler)
                            .src(stray_pva_buff.ground_height_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(ground_height)
                            .tile(TILE_WIDTH, TILE_HEIGHT);

        RasterDataFlow &raw_data_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto raw_data_handler               = stray_prog["raw_data_handler"];                //数据流的传输由句柄触发
        uint16_t *raw_data                  = stray_prog["raw_data"].ptr<uint16_t>();

        raw_data_dataflow.handler(raw_data_handler)
                            .src(stray_pva_buff.raw_data_d, TILE_WIDTH, VIEW_HEIGHT * RAW_DATA_CNT, TILE_WIDTH)
                            .tileBuffer(raw_data)
                            .tile(TILE_WIDTH, TILE_HEIGHT * RAW_DATA_CNT);

        RasterDataFlow &stray_mask0_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto stray_mask0_handler               = stray_prog["stray_mask0_handler"];             //数据流的传输由句柄触发
        uint16_t *stray_mask0                  = stray_prog["stray_mask0"].ptr<uint16_t>();

        stray_mask0_dataflow.handler(stray_mask0_handler)
                            .dst(stray_pva_buff.stray_mask0_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(stray_mask0)
                            .tile(TILE_WIDTH, TILE_HEIGHT);

        RasterDataFlow &stray_mask1_dataflow   = stray_prog.addDataFlowHead<RasterDataFlow>();  //向CmdProgram中添加一个RasterDataFlow
        auto stray_mask1_handler               = stray_prog["stray_mask1_handler"];             //数据流的传输由句柄触发
        uint16_t *stray_mask1                  = stray_prog["stray_mask1"].ptr<uint16_t>();

        stray_mask1_dataflow.handler(stray_mask1_handler)
                            .dst(stray_pva_buff.stray_mask1_d, TILE_WIDTH, VIEW_HEIGHT, TILE_WIDTH)
                            .tileBuffer(stray_mask1)
                            .tile(TILE_WIDTH, TILE_HEIGHT);

        stray_prog.compileDataFlows();
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Alloc pva host memory
 *
 * \return Error code
 * \retval 0: PVA task submitted successed
 * \retval 1: Caught a cuPVA exceptions
*/
int strayDataAlloc()
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

        if (pvaStrayCompile() != 0) {
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
 * \brief Release the alloced pva host memory
 *
 * \return Error code
 * \retval 0: PVA task submitted successed
 * \retval 1: Caught a cuPVA exceptions
*/
int strayDataFree()
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

        stray_pva_buff.dist_wave_h = nullptr;
        stray_pva_buff.att0_h = nullptr;
        stray_pva_buff.att1_h = nullptr;
        stray_pva_buff.class_line_h = nullptr;
        stray_pva_buff.ground_height_h = nullptr;
        stray_pva_buff.raw_data_h = nullptr;
        stray_pva_buff.stray_var_h = nullptr;
        stray_pva_buff.stray_mask0_h = nullptr;
        stray_pva_buff.stray_mask1_h = nullptr;
    }
    catch (cupva::Exception const &e)
    {
        return 1;
    }
    return 0;
}

/**
 * \brief Set rainwall params in CmdProgram
 *
 * \param[in]  rainwall_cnt  : rainwall cnt
 * \param[in]  rainwall_dist : rainwall dist
*/
void setRainWallParams(int rainwall_cnt, int rainwall_dist)
{
    CmdProgram& stray_prog = getStrayProg();

    // 设置杂散算法相关参数
    stray_prog["rainwall_dist"].set((int *)&rainwall_dist, sizeof(int));
    stray_prog["rainwall_cnt"].set((int *)&rainwall_cnt, sizeof(int));
}

/**
 * \brief Create and submit pva task for stray algo
 *
 * \param[in]  rainwall_cnt  : rainwall cnt
 * \param[in]  rainwall_dist : rainwall dist
 * \param[out] exception_msg : pva task exception message
 * \param[out] status_code   : pva task error code
 * 
 * \return Error code
 * \retval 0: PVA task submitted successed
 * \retval 1: Caught a cuPVA exceptions
 * \retval 2: VPU Program returned an Error Code
*/
int strayProcPva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time)
{
    try
    {
        CmdProgram& stray_prog = getStrayProg();

        SyncObj sync = SyncObj::Create(true);
        Fence fence{sync};
        CmdRequestFences rf{fence};
        CmdStatus status[2];

        auto time5 = std::chrono::steady_clock::now();

        Stream& algo_stream = getAlgoStream();
        algo_stream.submit({&stray_prog, &rf}, status);

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
        //若捕获到异常，返回异常信息
        exception_msg = std::string(e.what());
        return 1;
    }
    return 0;
}