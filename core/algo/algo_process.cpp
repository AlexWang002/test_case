/*******************************************************************************
 * \addtogroup algo
 * \{
 * \file algo_process.cpp
 * \brief
 * \version 0.5
 * \date 2025-08-06
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-06-17 | Init version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-06-20 | Reduce threads to 2 |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.3 | 2025-06-27 | Use struct array replace map |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.4 | 2025-07-14 | Modify code style, change timeout to 200ms |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.5 | 2025-08-06 | Add comments;|
 * |     |            | Fix warnings |
 *
 ******************************************************************************/
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include "algo_process.h"
#include "thread_config.h"
#include "cpu_load.h"
#include "rs_new_logger.h"
#include "trail.h"
#include "denoise.h"
#include "upsample.h"
#include "highcalc.h"
#include "spray.h"
#include "stray.h"
#include <iostream>
#include <fstream>
/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include <cupva_host.hpp>           // Main host-side C++-API header file
#include <cupva_host_nonsafety.hpp> // Header file for VPU printf functionality.
#include <cupva_platform.h> // Header that includes macros for specifying PVA executables
/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense
{
namespace lidar
{
// SyncQueue<TimePoint> inputTimeQueue(20, [](TimePoint data) {});
// SyncQueue<TimePoint> inputTimeQueue1(20, [](TimePoint data) {});
/**
 * \brief  Set thread kName
 * \param[in] kName : new thread kName
 *                   range：0 - 2^32-1. Accuracy: 1.
 */
void CloudManager::setThreadName(const std::string& kName)
{
    int32_t ret = pthread_setname_np(pthread_self(), kName.substr(0, 15).c_str());
    if (ret != 0) {
        LogError("ERROR: setThreadName error: {}, thread name: {}", ret, kName);
        FaultManager8::getInstance().setFault(FaultBits8::LidarThreadNameFault);
    }
    else{
        if (FaultManager8::getInstance().hasFault(FaultBits8::LidarThreadNameFault))
            FaultManager8::getInstance().clearFault(FaultBits8::LidarThreadNameFault);
    }
}

/**
 * \brief  Msop send callback function.
 * \param[in] kCbSend : Msop send function
 *                Range: 0 - 2^32-1. Accuracy: 1.
 */
void CloudManager::regCallback(const std::function<void(const uint8_t* pkt, size_t size)>& kCbSend)
{
    cb_send_ = kCbSend;
}

/**
 * \brief  Start processing point cloud.
 */
void CloudManager::start(void)
{
    to_exit_handle_.store(false);

    algo_func_.algo_delay_switch_ = this->process_delay_switch_;

    cacl_done_.store(0);
    for (uint32_t i = 0; i < ALGO_THREAD_NUM; ++i) {
        thread::ThreadConfig config;
        if (thread::thread_params.size() > i+2)
        config = thread::thread_params[i+2];
        algo_handle_threads_.emplace_back(createConfiguredStdThread(config, [this, i] {
            setThreadName("RS-AlgoProc-" + std::to_string(i));
            this->algoProcess(i);
        }));
    }

    algo_func_.algoInit();

    for (int32_t i = 0; i < ALGO_FRM_BUF_SIZE; ++i) {
        frame_buffer_[i].frame_droped.store(false);
    }

    LogInfo("Algo process start! Version:{}.{}.{}",std::to_string(ALGO_VERSION_MAJOR),
        std::to_string(ALGO_VERSION_MINOR) ,std::to_string(ALGO_VERSION_PATCH));
#ifdef ALGO_REINJ
    if (!processed_file_.is_open()) {
        processed_file_.open("process_point_cloud.bin",
                                std::ios::binary | std::ios::out | std::ios::app);
        printf("process_point_cloud is opened for write \n");
    }
#endif
}

/**
 * \brief  Stop process point cloud.
 */
bool CloudManager::stop(void)
{
    for (int32_t i = 0; i < ALGO_FRM_BUF_SIZE; ++i) {
        frame_buffer_[i].frame_droped.store(true);
    }
    cv_recv_.notify_all();

    to_exit_handle_.store(true);
    for (auto& thread : algo_handle_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
#ifdef ALGO_REINJ
    if (processed_file_.is_open()) {
        processed_file_.close();
        printf("process_point_cloud is closed \n");
    }
#endif
    return true;
}

/**
 * \brief  Algorithm point cloud process.
 * \param[in] task_id: process thread index
 *               Range: 0-1. Accuracy: 1.
 */
void CloudManager::algoProcess(int32_t task_id)
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(800));
    try {
        pid_t tid = gettid();
        utils::addThread(tid, "algorithm");
        /*线程启动时在DRAM中为pva申请算法所需内存*/
        thread_local bool first{true};
        if (first) {
            first = false;
            upsampleDataAlloc();
        }

        while (false == to_exit_handle_.load()) {
            std::chrono::microseconds total_time = (std::chrono::microseconds)0;
            if (cacl_done_.load() == 0) {
                bool isLostPkt = false;
                AlgoFunction::tstFrameBuffer* frame_buffer = &frame_buffer_[proc_buffer_idx_.load()];

                auto resetAndSwitchFrame = [&]() {
                    algo_func_.algoFrameChange();
                    frame_buffer->recv_idx.store(0);
                    int32_t old_val = proc_buffer_idx_.load();
                    int32_t new_val;
                    do {
                        new_val = (old_val + 1) % ALGO_FRM_BUF_SIZE;
                    } while (!proc_buffer_idx_.compare_exchange_weak(old_val, new_val));
                };

                for (int32_t cc = 9; cc < algo_func_.VIEW_W; cc += 10) {
                    if (recvEnoughData(cc, frame_buffer)) {
                        auto start = std::chrono::steady_clock::now();

                        /*CPU流式处理一次性处理10列点云*/
                        for (int32_t col = cc - 9; col <= cc; col ++) {
                            algo_func_.pcAlgoMainFunc(col, frame_buffer);
                            /*PVA以整帧为单位执行点云算法*/
                            if(col == algo_func_.VIEW_W - 1){
                                algo_func_.denoiseExec(frame_buffer);

                                algo_func_.trailExec(frame_buffer);

                                algo_func_.highcalcExec(frame_buffer);

                                algo_func_.strayDeleteExec(frame_buffer);

                                algo_func_.sprayRemoveExec(frame_buffer);

                                auto time_start1 = std::chrono::steady_clock::now();
                                memcpy(dist_wave0, frame_buffer->dist0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(dist_wave1, frame_buffer->dist1, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);//加入双回波数据
                                memcpy(refl_wave0, frame_buffer->ref0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(refl_wave1, frame_buffer->ref1, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(attr_wave0, frame_buffer->att0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(attr_wave1, frame_buffer->att1, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);

                                for (int col_idx = 0; col_idx < algo_func_.VIEW_W; col_idx++)
                                {
                                    // 获取掩码指针
                                    uint16_t* trail_mask_out = algo_func_.trail_mask_out_frm[col_idx];
                                    uint16_t* denoise_mask_out = algo_func_.denoise_mask_out_frm[col_idx];
                                    uint16_t* stray_mask_out0 = algo_func_.stray_mask_out_frm0[col_idx];
                                    uint16_t* stray_mask_out1 = algo_func_.stray_mask_out_frm1[col_idx];
                                    uint16_t* spray_mask_out0 = algo_func_.spray_mark_out_frm0[col_idx];
                                    uint16_t* spray_mask_out1 = algo_func_.spray_mark_out_frm1[col_idx];
                                    uint16_t* Distwave0 = dist_wave0[col_idx];
                                    uint16_t* Distwave1 = dist_wave1[col_idx];
                                    uint16_t* Refwave0 = refl_wave0[col_idx];
                                    uint16_t* Refwave1 = refl_wave1[col_idx];
                                    uint16_t* Attrwave0 = attr_wave0[col_idx];
                                    uint16_t* Attrwave1 = attr_wave1[col_idx];
                                    for (int row_idx = 0; row_idx < algo_func_.VIEW_H; row_idx++){
                                        const bool trail = trail_mask_out[row_idx];
                                        const bool denoise = denoise_mask_out[row_idx];
                                        const bool stray0 = stray_mask_out0[row_idx];
                                        const bool stray1 = stray_mask_out1[row_idx];
                                        const bool spray0 = spray_mask_out0[row_idx];
                                        const bool spray1 = spray_mask_out1[row_idx];
                                        const int attr0 = Attrwave0[row_idx];
                                        const int attr1 = Attrwave1[row_idx];

                                        Attrwave0[row_idx] = (attr0 & 0x7E) + stray0;
                                        Attrwave1[row_idx] = (attr1 & 0x7E) + stray1;

                                        if(!denoise)
                                        {
                                            Distwave0[row_idx] = 0;
                                            Refwave0[row_idx] = 0;
                                        }

                                        if (algo_func_.algo_Param.DeleteOn) {
                                            const bool wave0_del_flag = stray0 || spray0 || trail;
                                            const bool wave1_sel_flag = !stray1 && !spray1 && !trail;
                                            // 情况1: 两回波均无效点
                                            if (wave0_del_flag && !wave1_sel_flag) {
                                                Distwave0[row_idx] = 0;
                                                Refwave0[row_idx] = 0;
                                                Attrwave0[row_idx] = 0;
                                            }
                                            // 情况2: 第一回波无效点，第二回波有效点
                                            else if (wave0_del_flag && wave1_sel_flag) {
                                                Distwave0[row_idx] = Distwave1[row_idx];
                                                Refwave0[row_idx] = Refwave1[row_idx];
                                                Attrwave0[row_idx] = Attrwave1[row_idx];
                                            }
                                        } else {
                                            const bool wave0_del_flag = stray0 || trail;
                                            const bool wave1_sel_flag = !stray1 && !spray1 && !trail;
                                            // 情况1: 两回波均无效点
                                            if (wave0_del_flag && !wave1_sel_flag) {
                                                Distwave0[row_idx] = 0;
                                                Refwave0[row_idx] = 0;
                                                Attrwave0[row_idx] = 0;
                                            }
                                            // 情况2: 第一回波无效点，第二回波有效点
                                            else if (wave0_del_flag && wave1_sel_flag) {
                                                Distwave0[row_idx] = Distwave1[row_idx];
                                                Refwave0[row_idx] = Refwave1[row_idx];
                                                Attrwave0[row_idx] = Attrwave1[row_idx];
                                            }
                                            if(spray0 && !wave1_sel_flag)
                                            {
                                                Attrwave0[row_idx] = ((spray0) << 7) + (Attrwave0[row_idx] & 0x7F);
                                            }
                                        }
                                        Attrwave0[row_idx] &= 0xF7;
                                    }
                                }

                                memcpy(DistDownIn_h, dist_wave0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(DistRawIn_h, frame_buffer->dist0_raw, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(RefDownIn_h, refl_wave0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(RefRawIn_h, frame_buffer->ref0_raw, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                memcpy(AttrIn_h, attr_wave0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                                auto time_end1 = std::chrono::steady_clock::now();
                                auto time_duration1 = std::chrono::duration_cast<std::chrono::microseconds>(time_end1 - time_start1);

                                algo_func_.upsampleExec(frame_buffer);

                                /** 最后一列置0 */
                                uint32_t start_idx = (algo_func_.VIEW_W - 1) * algo_func_.VIEW_H;
                                memset(&DistOutUp_h[start_idx], 0, algo_func_.VIEW_H * sizeof(uint16_t));
                                memset(&RefOutUp_h[start_idx], 0, algo_func_.VIEW_H * sizeof(uint16_t));
                                memset(&AttrOutUp_h[start_idx], 0, algo_func_.VIEW_H * sizeof(uint16_t));

                                auto assemblePkt = [this](auto* cloud_p,
                                                    const uint16_t* dist_p,
                                                    const uint16_t* refl_p,
                                                    const uint16_t* attr_p) {
                                    
                                #ifdef ALGO_REINJ
                                    for (size_t i = 0; i < 192; ++i) {
                                        cloud_p->pixels[i].raws[0].peak = (dist_p[i]);
                                        cloud_p->pixels[i].raws[0].width = refl_p[i];
                                        cloud_p->pixels[i].raws[1].peak = attr_p[i];
                                    }
                                    if (inj_frame_cnt_ < 10) {
                                        if (processed_file_.is_open()) {
                                            processed_file_.write(reinterpret_cast<const char*>(cloud_p), sizeof(RSEMXMsopPkt));
                                        }
                                    }
                                #else
                                    for (size_t i = 0; i < 192; ++i) {
                                        cloud_p->pixels[i].waves[0].radius = htons(dist_p[i]);
                                        cloud_p->pixels[i].waves[0].intensity = refl_p[i];
                                        cloud_p->pixels[i].waves[0].attribute = attr_p[i];
                                    }
                                #endif
                                };

                                for (int32_t col = 0; col < algo_func_.VIEW_W; ++col) {
                                    int32_t surface_id = frame_buffer->surface_id.load();
                                    /** 拷贝上采样后的数据到dist和ref中 */
                                    int offset = col * algo_func_.VIEW_H;
                                    if (0 == surface_id) {
                                        for(int32_t j = 0; j < 2; ++j) {
                                            int32_t index = frame_buffer->cloud_id[col * 2 + j];
                                            auto& cloud = proc_clouds_[index];
                                            if(j == 0){
                                                assemblePkt(&cloud, dist_wave0[col], refl_wave0[col], attr_wave0[col]);
                                            } else {
                                                assemblePkt(&cloud, &DistOutUp_h[offset], &RefOutUp_h[offset], &AttrOutUp_h[offset]);
                                            }
                                            cb_send_(reinterpret_cast<uint8_t*>(&cloud), sizeof(RSEMXMsopPkt));
                                        }
                                    } else {
                                        if(col == 0){
                                            int32_t index = frame_buffer->cloud_id[col];
                                            auto& cloud = proc_clouds_[index];
                                            static const uint16_t zero_dist[algo_func_.VIEW_H] = {0};
                                            static const uint16_t zero_refl[algo_func_.VIEW_H] = {0};
                                            static const uint16_t zero_attr[algo_func_.VIEW_H] = {0};
                                            assemblePkt(&cloud, zero_dist, zero_refl, zero_attr);
                                            cb_send_(reinterpret_cast<uint8_t*>(&cloud), sizeof(RSEMXMsopPkt));

                                            for(int32_t j = 0; j < 2; ++j) {
                                                int32_t index = frame_buffer->cloud_id[2 * col + j + 1];
                                                auto& cloud = proc_clouds_[index];
                                                if(j == 0){
                                                    assemblePkt(&cloud, dist_wave0[col], refl_wave0[col], attr_wave0[col]);
                                                } else {
                                                    assemblePkt(&cloud, &DistOutUp_h[offset], &RefOutUp_h[offset], &AttrOutUp_h[offset]);
                                                }
                                                cb_send_(reinterpret_cast<uint8_t*>(&cloud), sizeof(RSEMXMsopPkt));
                                            }
                                        }else if(col == algo_func_.VIEW_W - 1){
                                            int32_t index = frame_buffer->cloud_id[2 * col + 1];
                                            auto& cloud = proc_clouds_[index];
                                            assemblePkt(&cloud, dist_wave0[col], refl_wave0[col], attr_wave0[col]);
                                            cb_send_(reinterpret_cast<uint8_t*>(&cloud), sizeof(RSEMXMsopPkt));
                                        } else{
                                            for(int32_t j = 0; j < 2; ++j) {
                                                int32_t index = frame_buffer->cloud_id[2 * col + j + 1];
                                                auto& cloud = proc_clouds_[index];
                                                if(j == 0){
                                                    assemblePkt(&cloud, dist_wave0[col], refl_wave0[col], attr_wave0[col]);
                                                } else {
                                                    assemblePkt(&cloud, &DistOutUp_h[offset], &RefOutUp_h[offset], &AttrOutUp_h[offset]);
                                                }
                                                cb_send_(reinterpret_cast<uint8_t*>(&cloud), sizeof(RSEMXMsopPkt));
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        auto end = std::chrono::steady_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                        total_time += duration;
                    } else {
                        frame_buffer->frame_droped.store(false);
                        break;
                    }
                }
                cacl_done_.store(1);
                resetAndSwitchFrame();

                if (total_time.count() > kCalcTimeout_) {
                    LogError("ERROR: algoProcess calc time out :{} thread:{}", total_time.count(), task_id);
                }
            } else {
                std::unique_lock<std::mutex> lock(mtx_calc_);
                bool res = cv_calc_.wait_for(lock, std::chrono::milliseconds(200), [&] {
                    return (cacl_done_.load() == 0 || to_exit_handle_.load());
                });
            }
        }
        /** memory release */
        denoiseDataFree();
        TrailDataFree();
        upsampleDataFree();
        highcalcDataFree();
        strayBufferRelease();
        sprayDataFree();
    }
    catch (const std::exception& kE) {
        LogError("ERROR: algoProcess thread:{}, crashed:{}", task_id, kE.what());
        FaultManager8::getInstance().setFault(FaultBits8::LidarThreadCrash);
    }
}

/**
 * \brief Receive point cloud data packets and process.
 * \param[in] kMsopData: point cloud data packets
 *               Range: 0~2^32 - 1. Accuracy: 1.
 * \param[in] msop_data_size: size of point cloud data packets
 *               Range: 3116. Accuracy: 1.
 */
void CloudManager::receiveCloud(const uint8_t* kMsopData, int32_t msop_data_size)
{
    if (kMsopData == nullptr) {
        LogError("[CloudManager::receiveCloud] kMsopData is nullptr");
        return;
    }

    if (msop_data_size != sizeof(RSEMXMsopPkt)) {
        LogError("ERROR: msop data size error:{}", msop_data_size);
        return;
    }

    const auto& kRawData = *reinterpret_cast<const RSEMXMsopPkt*>(kMsopData);

    int32_t old_val;
    int32_t new_val;
    int32_t proc_cloud_idx = proc_cloud_idx_.load();

    std::lock_guard<std::mutex> lock(mtx_recv_);
    int32_t recv_buf_idx = recv_buffer_idx_.load();
    AlgoFunction::tstFrameBuffer* frame_buffer = &frame_buffer_[recv_buf_idx];
    int32_t pkt_cnt = ntohs(kRawData.header.pkt_seq) - 1;

    if ((pkt_cnt >= 0) && (pkt_cnt < algo_func_.UP_VIEW_W)) {
        //丢包之后的逻辑，先判断是否丢包
        if (frame_buffer->frame_droped.load()) {
            if (0 == pkt_cnt) {  //丢包之后，第一列数据，则重置丢包标志位
                frame_buffer->frame_droped.store(false);
                old_val = recv_buf_idx;
                do{
                    new_val = (old_val + 1) % ALGO_FRM_BUF_SIZE;
                } while (!recv_buffer_idx_.compare_exchange_weak(old_val, new_val));
                last_pkt_recv_ = 0;
                algo_func_.updateGroundFitParams();//更新地面拟合参数
            }
            else {
                return;
            }
        }

        //判断丢包逻辑
        if (((0 == last_pkt_recv_) && (0 == pkt_cnt))
            || (((algo_func_.UP_VIEW_W - 1) == last_pkt_recv_) && (0 == pkt_cnt))) {
            //帧起始或帧末尾
        } else if ((last_pkt_recv_ + 1) != pkt_cnt) {
            //丢包，三种形态的丢包都包含
            LogError("ERROR: receive loss lead to loss pkt:{} position:{}", last_pkt_recv_, pkt_cnt);
            frame_buffer->frame_droped.store(true);
            cv_recv_.notify_all();
            if (pkt_cnt != 0) {
                return;
            }
        }

        if ((last_pkt_recv_ > pkt_cnt) && (0 == pkt_cnt)) {
            old_val = recv_buf_idx;
            do {
                new_val = (old_val + 1) % ALGO_FRM_BUF_SIZE;
            } while (!recv_buffer_idx_.compare_exchange_weak(old_val, new_val));

            algo_func_.updateGroundFitParams();//更新地面拟合参数
        }
        last_pkt_recv_ = pkt_cnt;

        frame_buffer = &frame_buffer_[recv_buffer_idx_.load()];  //重新定向到当前帧的帧缓冲区

        uint8_t surface_id_pkt = kRawData.header.surface_id;
        if (0 == pkt_cnt) {
            frame_buffer->surface_id.store(surface_id_pkt);
        }
        int32_t recv_idx = frame_buffer->recv_idx.load();

        if ((0 == pkt_cnt) && (0 != recv_idx)) {
            LogError("ERROR: algorithms timeout lead to recv loss pkt:{}", recv_idx);
            FaultManager64::getInstance().overflow_position_ |=  0x4;
            FaultManager64::getInstance().setFault(FaultBits::LidarPointCloudBufferOverflowFault);
            frame_buffer->frame_droped.store(true);
            cv_recv_.notify_all();
            return;
        } else if(0 == pkt_cnt) {
            FaultManager64::getInstance().overflow_position_ &= 0x3;
            if(FaultManager64::getInstance().hasFault(FaultBits::LidarPointCloudBufferOverflowFault)
                && ((FaultManager64::getInstance().overflow_position_ & 0x7) == 0)) {
                FaultManager64::getInstance().clearFault(FaultBits::LidarPointCloudBufferOverflowFault);
            }
        }

        old_val = proc_cloud_idx;
        do {
            new_val = (old_val + 1) % kMaxCloudNum_;
        } while (!proc_cloud_idx_.compare_exchange_weak(old_val, new_val));
        proc_clouds_[proc_cloud_idx] = kRawData; // 将新的点云对象存储到正在处理的点云集合中
        frame_buffer->cloud_id[pkt_cnt] = proc_cloud_idx;

        int32_t upsample_col = pkt_cnt >> 1;
        auto processWaves = [&](bool condition, int32_t high_calc_col) {
            if (condition) {
                uint16_t* dist0 = &frame_buffer->dist0[upsample_col][0];
                uint16_t* dist1 = &frame_buffer->dist1[upsample_col][0];
                uint16_t* ref0 = &frame_buffer->ref0[upsample_col][0];
                uint16_t* ref1 = &frame_buffer->ref1[upsample_col][0];
                uint16_t* att0 = &frame_buffer->att0[upsample_col][0];
                uint16_t* att1 = &frame_buffer->att1[upsample_col][0];

                for (int32_t i = 0; i < algo_func_.VIEW_H; ++i) {
                    const RSEMXMsopWave* kWaves0 = &kRawData.pixels[i].waves[0];
                    const RSEMXMsopWave* kWaves1 = &kRawData.pixels[i].waves[1];
                    dist0[i] = ntohs(kWaves0->radius);
                    ref0[i] = kWaves0->intensity;
                    att0[i] = kWaves0->attribute;
                    dist1[i] = ntohs(kWaves1->radius);
                    ref1[i] = kWaves1->intensity;
                    att1[i] = kWaves1->attribute;
                }
            } else {
                uint16_t* dist0_raw = &frame_buffer->dist0_raw[upsample_col][0];
                uint16_t* ref0_raw = &frame_buffer->ref0_raw[upsample_col][0];
                for (int32_t i = 0; i < algo_func_.VIEW_H; ++i) {
                    const RSEMXMsopWave* kWaves0 = &kRawData.pixels[i].waves[0];
                    dist0_raw[i] = ntohs(kWaves0->radius);
                    ref0_raw[i] = kWaves0->intensity;
                }
            }
        };
        if (0 == surface_id_pkt) {  //当前帧数id,确定奇偶列是否保存
            processWaves((pkt_cnt & 0x1) == 0, pkt_cnt);
        } else {
            processWaves((pkt_cnt & 0x1) != 0, (algo_func_.UP_VIEW_W - pkt_cnt) - 1);
        }

        if ((pkt_cnt & 0x1) == 1) {
            if (0x1 == pkt_cnt) { // 点云完成第一列处理
                cacl_done_.store(0);
                std::lock_guard<std::mutex> lock(mtx_calc_);
                cv_calc_.notify_all();
            }

            if ((upsample_col + 1) % 10 == 0) {
                frame_buffer->recv_idx.store(upsample_col + 1);
                cv_recv_.notify_all();
            }
        }
    } else {
        LogError("ERROR: pkt_seq value is invalid:{}", pkt_cnt);
    }

    //cv_recv_.notify_all();
}

/**
 * \brief Obtain pending blocks (called by the analysis thread).
 * \param[in] col: column index
 *               Range: 0-759. Accuracy: 1.
 * \param[in] frame_buffer: frame point cloud data buffer
 *               Range: 0~2^32 -1. Accuracy: 1.
 */
bool CloudManager::recvEnoughData(int32_t col, AlgoFunction::tstFrameBuffer* frame_buffer)
{
    std::unique_lock<std::mutex> lock(mtx_recv_);
    // 等待前置列就绪
    int32_t required_col;
    if (col >= algo_func_.VIEW_W - 1) {
        required_col = algo_func_.VIEW_W - 1;
    } else {
        required_col = col;
    }

    if (frame_buffer != nullptr && frame_buffer->frame_droped.load()) {
        return false;
    }

    if (frame_buffer != nullptr && frame_buffer->recv_idx.load() > required_col) {
        return true;
    }
    bool success = cv_recv_.wait_for(lock, std::chrono::milliseconds(200),
                                        [&] { return ((frame_buffer->frame_droped.load()) || (frame_buffer->recv_idx.load() > required_col));});
    if ((!success) || frame_buffer->frame_droped.load()) {
        if (!success) {
            LogError("ERROR: receive timeout lead to recv loss pkt:{}", frame_buffer->recv_idx.load());
        }
        return false;
    }
    return true;
}


}
}
