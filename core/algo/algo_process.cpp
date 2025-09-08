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
#include "yaml_manager.h"
#include "thread_config.h"
#include "cpu_load.h"
#include "rs_new_logger.h"
#include "trail.h"
#include "denoise.h"
#include "groundfit.h"
#include "upsample.h"
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
    }
}

/**
 * \brief  Submit processed blocks.
 * \param[in] proc_col : processing column
 *                Range: -2^31 - 2^31-1. Accuracy: 1.
 * \param[in] frame_buffer : frame buffer
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] dist : current column distance data
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] ref : current column reflection data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] surface_id : channel index
 *                Range: 0-1. Accuracy: 1.
 */
void CloudManager::assemblePkt(int32_t proc_col,
                            AlgoFunction::tstFrameBuffer* frame_buffer,
                            uint16_t* dist, uint8_t* ref, int32_t surface_id) {
    if ((proc_col < 0) || proc_col >= algo_func_.UP_VIEW_W) {
        LogDebug("ALGO: proc_col is invalid {}", proc_col);
        return;
    }

    if (dist == nullptr || ref == nullptr || frame_buffer == nullptr) {
        LogError("[CloudManager::assemblePkt] dist | ref | frame_buffer is nullptr");
        return;
    }

    int32_t index = frame_buffer->cloud_id[proc_col];

    if ((index < 0) || (index >= kMaxCloudNum_)) {
        LogDebug("ALGO: send packet index out of range{}", index);
        return;
    };

    auto& cloud = proc_clouds_[index];
#ifdef ALGO_REINJ
    // 更新处理结果
    if (((0 == surface_id) && ((proc_col & 0x1) == 0))
        || ((1 == surface_id) && ((proc_col & 0x1) != 0))) {
        for (size_t i = 0; i < algo_func_.VIEW_H; ++i) {
            cloud.pixels[i].raws[0].peak = (dist[i]);
            cloud.pixels[i].raws[0].width = ref[i];
        }
    } else {
        for (size_t i = 0; i < algo_func_.VIEW_H; ++i) {
            cloud.pixels[i].raws[0].peak = (dist[i + algo_func_.VIEW_H]);
            cloud.pixels[i].raws[0].width = ref[i + algo_func_.VIEW_H];
        }
    }
    if (inj_frame_cnt_ < 10) {
        if (processed_file_.is_open()) {
            processed_file_.write(reinterpret_cast<const char*>(&cloud), sizeof(RSEMXMsopPkt));
        }
    }

    if (proc_col == (algo_func_.UP_VIEW_W - 1)) {
        inj_frame_cnt_++;
        printf("write frame = %d\n", inj_frame_cnt_);
    }
#else
    // 更新处理结果
    if (((0 == surface_id) && ((proc_col & 0x1) == 0))
        || ((1 == surface_id) && ((proc_col & 0x1) != 0))) {
        for (size_t i = 0; i < algo_func_.VIEW_H; ++i) {
            cloud.pixels[i].waves[0].radius = htons(dist[i]);
            cloud.pixels[i].waves[0].intensity = ref[i];
        }
    } else {
        for (size_t i = 0; i < algo_func_.VIEW_H; ++i) {
            cloud.pixels[i].waves[0].radius = htons(dist[i + algo_func_.VIEW_H]);
            cloud.pixels[i].waves[0].intensity = ref[i + algo_func_.VIEW_H];
        }
    }
#endif

    cb_send_(reinterpret_cast<uint8_t*>(&cloud), sizeof(RSEMXMsopPkt));
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

    for (auto& idx : algo_proc_idx_) {
        idx.store(0);
    }
    for (uint32_t i = 0; i < ALGO_THREAD_NUM; ++i) {
        cacl_done_[i].store(0);
        yaml::ThreadConfig config;
        if (yaml::thread_params.size() > i+3)
            config = yaml::thread_params[i+3];
        algo_handle_threads_.emplace_back(createConfiguredStdThread(config, [this, i] {
            setThreadName("RS-AlgoProc-" + std::to_string(i));
            this->algoProcess(i);
        }));
    }
    yaml::ThreadConfig config;
    if (yaml::thread_params.size() > 2)
        config = yaml::thread_params[2];
    handle_thread_ = createConfiguredStdThread(config, [this] {
        setThreadName("RS-FinalProc");
        this->algoFinalProcess();
    });

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
    handle_thread_.join();
#ifdef ALGO_REINJ
    if (processed_file_.is_open()) {
        processed_file_.close();
        printf("process_point_cloud is closed \n");
    }
#endif
    return true;
}

#ifdef ALGO_WRITE_FILE
/**
 * \brief  Write distance and reflection into file.
 * \param[in] dist : current row distance data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] ref : current row reflection data
 *                Range: 0-255. Accuracy: 1.
 * \param[in] x: row index
 *               Range: 0-759. Accuracy: 1.
 */
void CloudManager::writeFileFunc(uint16_t* dist, uint8_t* ref, int x)
{
    if (write_file_) {
        if (dist_file_.is_open() && ref_file_.is_open()) {
            for (int i = 0; i < algo_func_.VIEW_H; ++i) {
                dist_file_ << dist[i + (x * algo_func_.VIEW_H)] << " ";
                ref_file_ << static_cast<int>(ref[i + (x * algo_func_.VIEW_H)]) << " ";
            }
            dist_file_ << std::endl;
            ref_file_ << std::endl;
        }
    }
}
#endif

/**
 * \brief  Algorithm delete point and package msop.
 */
void CloudManager::algoFinalProcess(void)
{
    pid_t tid = gettid();
    utils::addThread(tid, "algoFinalProcess");
    /** 申请上采样变量内存空间 */
    upsampleDataAlloc();
    try {
        while (false == to_exit_handle_.load()) {
            uint16_t dist[algo_func_.VIEW_H * 2]{0};
            uint8_t ref[algo_func_.VIEW_H * 2]{0};
            AlgoFunction::tstFrameBuffer* frame_buffer = &frame_buffer_[proc_buffer_idx_.load()];
            std::chrono::microseconds total_time = (std::chrono::microseconds)0;
            auto resetAndSwitchFrame = [&]() {
                algo_func_.algoFrameChange();
                frame_buffer->recv_idx.store(0);
                for (auto& idx : algo_proc_idx_) {
                    idx.store(0);
                }
                int32_t old_val = proc_buffer_idx_.load();
                int32_t new_val;
                do {
                    new_val = (old_val + 1) % ALGO_FRM_BUF_SIZE;
                } while (!proc_buffer_idx_.compare_exchange_weak(old_val, new_val));

                std::lock_guard<std::mutex> lock(mtx_calc_);
                for (int32_t i = 0; i < ALGO_THREAD_NUM; ++i) {
                    cacl_done_[i].store(0);
                }
                cv_calc_.notify_all();
            };
            if (sendEnoughData(algo_func_.VIEW_W - 1)) {
                for (int col_idx = 0; col_idx < algo_func_.VIEW_W; col_idx++)
                {
                    for (int row_idx = 0; row_idx < algo_func_.VIEW_W; row_idx++){
                        // 按照优先级处理不同掩码情况
                        if (algo_func_.trail_mask_out_frm[col_idx][row_idx] == 1 ||
                            algo_func_.denoise_mask_out_frm[col_idx][row_idx] == 0){
                            // 只清空wave0数据
                            frame_buffer->dist0[col_idx][row_idx] = 0;
                            frame_buffer->ref0[col_idx][row_idx] = 0;
                        }
                    }
                }
                /** Data initialization */
                memcpy(DistDownIn_h, frame_buffer->dist0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                memcpy(DistRawIn_h, frame_buffer->dist0_raw, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                memcpy(RefDownIn_h, frame_buffer->ref0, sizeof(uint8_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                memcpy(RefRawIn_h, frame_buffer->ref0_raw, sizeof(uint8_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                auto start = std::chrono::steady_clock::now();
                upsample_main();
                auto end = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                total_time += duration;
                for (int32_t col = 0; col < algo_func_.VIEW_W + 1; ++col) {
                    int32_t surface_id = frame_buffer->surface_id.load();
                    /** 拷贝上采样后的数据到dist和ref中 */
                    int offset = col * algo_func_.VIEW_H;
                    memcpy(dist, &DistOutOri_h[offset], sizeof(uint16_t) * algo_func_.VIEW_H);
                    memcpy(&dist[algo_func_.VIEW_H], &DistOutUp_h[offset], sizeof(uint16_t) * algo_func_.VIEW_H);
                    memcpy(ref, &RefOutOri_h[offset], sizeof(uint8_t) * algo_func_.VIEW_H);
                    memcpy(&ref[algo_func_.VIEW_H], &RefOutUp_h[offset], sizeof(uint8_t) * algo_func_.VIEW_H);
                    if (1 == surface_id) {
                        if (col >= 1) {
                            if (1 == col) {
                                uint16_t dist_zero[algo_func_.VIEW_H * 2]{0};
                                uint8_t ref_zero[algo_func_.VIEW_H * 2]{0};
                                assemblePkt(0, frame_buffer, dist_zero, ref_zero, surface_id);
                                for(int32_t i = 0; i < 2; ++i) {
                                    assemblePkt((((col - 1) * 2) + i) + 1, frame_buffer, dist, ref, surface_id);
                                }
                            } else if (col < algo_func_.VIEW_W) {
                                for (int32_t i = 0; i < 2; ++i) {
                                    assemblePkt((((col - 1) * 2) + i) + 1, frame_buffer, dist, ref, surface_id);
                                }
                            } else {
                                for (int32_t i = 0; i < 1; ++i) {
                                    assemblePkt((((col - 1) * 2) + i) + 1, frame_buffer, dist, ref, surface_id);
                                }
                            }
                        }
                    } else {
                        if (col >= 1) {
                            for(int32_t i = 0; i < 2; ++i) {
                                assemblePkt(((col - 1) * 2) + i, frame_buffer, dist, ref, surface_id);
                            }
                        }
                    }
                    if (algo_func_.VIEW_W == col) {
                        resetAndSwitchFrame();
                    }
                }
            } else {
                //发生丢包之后，先清0当前帧数据，然后切换到下一帧
                // frame_buffer->frame_droped.store(false);
                // resetAndSwitchFrame();
                //break;
            }

        #ifdef ALGO_WRITE_FILE
            if (write_file_) {
                write_file_ = false;
            }
            if (dist_file_.is_open() && ref_file_.is_open()) {
                dist_file_.close();
                ref_file_.close();
            }
        #endif

            if (total_time.count() > kCalcTimeout_) {
                LogError("ERROR: algoProcess calc time out :{} thread:{}", total_time.count(), ALGO_THREAD_NUM);
            }
        }
    }
    catch (const std::exception& kE) {
        LogError("ERROR: algoProcess thread:{}, crashed:{}", ALGO_THREAD_NUM, kE.what());
    }
}


/**
 * \brief  Algorithm point cloud process.
 * \param[in] task_id: process thread index
 *               Range: 0-1. Accuracy: 1.
 */
void CloudManager::algoProcess(int32_t task_id)
{
    try {
        std::cout << "*******************" << std::endl;
        std::cout << "task_id: " << task_id << std::endl;
        std::cout << "*******************" << std::endl;

        if (task_id == 0) {
            pid_t tid = gettid();
            utils::addThread(tid, "denoise");

            /*线程启动时在DRAM中为pva申请denoise算法所需内存*/
            denoiseDataAlloc();
        }

        if (task_id == 1) {
            pid_t tid = gettid();
            utils::addThread(tid, "trail");

            /** only alloc once */
            TrailDataAlloc();
        }

        while (false == to_exit_handle_.load()) {
            // int32_t proc_col = 0;
            std::chrono::microseconds total_time = (std::chrono::microseconds)0;
            if (cacl_done_[task_id].load() == 0) {
                bool isLostPkt = false;
                AlgoFunction::tstFrameBuffer* frame_buffer = &frame_buffer_[proc_buffer_idx_.load()];
                    if (recvEnoughData(frame_buffer)) {
                        auto start = std::chrono::steady_clock::now();
                        /*PVA以整帧为单位执行denoise算法*/
                        if(task_id == 0){
                            /*拷贝整帧数据到denoise算法的PVA buffer中*/
                            memcpy((uint8_t *)denoise_dist_buffer_h, (uint8_t *)frame_buffer->dist0[0],  algo_func_.VIEW_H * algo_func_.VIEW_W * sizeof(uint16_t));

                            auto denoise_start = std::chrono::steady_clock::now();
                            denoiseProcPva();
                            auto denoise_end = std::chrono::steady_clock::now();
                            auto denoise_duration = std::chrono::duration_cast<std::chrono::microseconds>(denoise_end - denoise_start);
                            // std::cout << "denoise duration: " << denoise_duration.count() << "us" << std::endl;
                            memcpy((uint8_t *)&algo_func_.denoise_mask_out_frm[0], (uint8_t *)&denoise_mask_buffer_h[4 * algo_func_.VIEW_H], (algo_func_.VIEW_W - 4) * algo_func_.VIEW_H * sizeof(int));
                            //algo_func_.denoiseMaskMemcpy(0, (uint8_t *)&denoise_mask_buffer_h[4 * algo_func_.VIEW_H], (algo_func_.VIEW_W - 4) * algo_func_.VIEW_H * sizeof(int));
                        }

                        if(task_id == 1){
                            /** Data initialization */
                            memcpy(DistIn_h, frame_buffer->dist0, sizeof(uint16_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);
                            memcpy(RefIn_h, frame_buffer->ref0, sizeof(uint8_t) * algo_func_.VIEW_W * algo_func_.VIEW_H);

                            /** Process */
                            auto trail_start = std::chrono::steady_clock::now();
                            trail_main();
                            auto trail_end = std::chrono::steady_clock::now();
                            auto trail_duration = std::chrono::duration_cast<std::chrono::microseconds>(trail_end - trail_start);
                            // std::cout << "trail duration: " << trail_duration.count() << "us" << std::endl;
                            /** Mask copy */
                            memcpy(&algo_func_.trail_mask_out_frm[0], TrailMask, sizeof(int) *algo_func_.VIEW_W * algo_func_.VIEW_H);
                        }

                    // if (proc_col <= algo_func_.VIEW_W - 2) {
                        updateAlgoIdx(algo_func_.VIEW_W - 2, task_id);
                    // }
                    auto end = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                    total_time += duration;
                } else {
                    isLostPkt = true;
                    //break;
                }
                if (total_time.count() > kCalcTimeout_) {
                    LogError("ERROR: algoProcess calc time out :{} thread:{}", total_time.count(), task_id);
                }
                cacl_done_[task_id].store(1);
                if (isLostPkt) {
                    updateAlgoIdx(ALGO_LOSS_PKT_CODE - 1, task_id);
                } else {
                    updateAlgoIdx(algo_func_.VIEW_W - 2, task_id);
                }
            } else {
                std::unique_lock<std::mutex> lock(mtx_calc_);
                bool res = cv_calc_.wait_for(lock, std::chrono::milliseconds(200), [&] {
                    return (cacl_done_[task_id].load() == 0 || to_exit_handle_.load());
                });

                if (!res) {
                    LogError("ERROR: algoProcess thread:{}, cv_calc_ wait timeout", task_id);
                }
            }
        }
        if (task_id == 0) {
            denoiseDataFree();
        }

        if (task_id == 1) {
            /** memory release */
            TrailDataFree();
        }
    }
    catch (const std::exception& kE) {
        LogError("ERROR: algoProcess thread:{}, crashed:{}", task_id, kE.what());
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
            FaultManager::getInstance().setFault(FaultBits::LidarPointCloudBufferOverflowFault);
            frame_buffer->frame_droped.store(true);
            cv_recv_.notify_all();
            return;
        } else if (0 == pkt_cnt) {
            if (FaultManager::getInstance().hasFault(FaultBits::LidarPointCloudBufferOverflowFault))
                FaultManager::getInstance().clearFault(FaultBits::LidarPointCloudBufferOverflowFault);
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
                uint8_t* ref0 = &frame_buffer->ref0[upsample_col][0];
                uint8_t* ref1 = &frame_buffer->ref1[upsample_col][0];
                uint8_t* att0 = &frame_buffer->att0[upsample_col][0];
                uint8_t* att1 = &frame_buffer->att1[upsample_col][0];
                uint8_t* gnd0 = &frame_buffer->gnd_mark0[upsample_col][0];
                uint8_t* gnd1 = &frame_buffer->gnd_mark1[upsample_col][0];
                int32_t* high0 = &frame_buffer->high0[upsample_col][0];
                int32_t* high1 = &frame_buffer->high1[upsample_col][0];

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

                algo_func_.highCalcFunc(dist0, dist1, high_calc_col, high0, high1, gnd0, gnd1);
            } else {
                uint16_t* dist0_raw = &frame_buffer->dist0_raw[upsample_col][0];
                uint8_t* ref0_raw = &frame_buffer->ref0_raw[upsample_col][0];
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
            frame_buffer->recv_idx.store(upsample_col + 1);
        }
    } else {
        LogError("ERROR: pkt_seq value is invalid:{}", pkt_cnt);
    }

    cv_recv_.notify_all();
}

/**
 * \brief Judge if enough Data is able to send.
 * \param[in] col: column index
 *               Range: 0-759. Accuracy: 1.
 */
bool CloudManager::sendEnoughData(int32_t col)
{
    std::unique_lock<std::mutex> lock(mtx_send_);

    // 检查所有线程的处理状态
    bool all_loss = true;
    bool all_ready = true;
    for (const auto& kIdx : algo_proc_idx_) {
        int32_t val = kIdx.load();
        all_loss = all_loss && (val == ALGO_LOSS_PKT_CODE);
        all_ready = all_ready && (val >= col);
    }
    if (all_loss) {
        return false;
    }
    // if (algo_func_.VIEW_W == col) {
    //     return true;
    // }
    if (all_ready) {
        return true;
    }

    bool success = cv_send_.wait_for(lock, std::chrono::milliseconds(200), [&] {
        bool all_loss = true;
        bool all_ready = true;
        for(const auto& kIdx : algo_proc_idx_) {
            int32_t val = kIdx.load();
            all_loss = all_loss && (val == ALGO_LOSS_PKT_CODE);
            all_ready = all_ready && (val > col);
        }
        return ((all_loss || (algo_func_.VIEW_W == col)) || all_ready);
    });
    if ((!success) || (std::all_of(algo_proc_idx_.begin(), algo_proc_idx_.end(),
            [](const auto& kIdx) { return kIdx.load() == ALGO_LOSS_PKT_CODE; }))) {
        if (!success) {
            LogError("ERROR: send timeout lead to send loss pkt:{}", col);
        }
        return false;
    }
    return true;
}

/**
 * \brief Obtain pending blocks (called by the analysis thread).
 * \param[in] col: column index
 *               Range: 0-759. Accuracy: 1.
 * \param[in] frame_buffer: frame point cloud data buffer
 *               Range: 0~2^32 -1. Accuracy: 1.
 */
bool CloudManager::recvEnoughData(AlgoFunction::tstFrameBuffer* frame_buffer)
{
    std::unique_lock<std::mutex> lock(mtx_recv_);
    // 等待前置列就绪
    int32_t required_col = algo_func_.VIEW_W - 1;
    // if (col >= algo_func_.VIEW_W - 1) {
    //     required_col = algo_func_.VIEW_W - 1;
    // } else {
    //     required_col = col;
    // }

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

/**
 * \brief ‌Submit the processed blocks.
 * \param[in] proc_col: process point cloud column index
 *               Range: 0-759. Accuracy: 1.
 * \param[in] task_id: process thread index
 *               Range: 0-1. Accuracy: 1.
 */
void CloudManager::updateAlgoIdx(int32_t proc_col, uint32_t task_id)
{
    if (task_id >= algo_proc_idx_.size()) {
        LogError("ERROR: task_id is invalid:{}, max:{}", task_id, algo_proc_idx_.size() - 1);
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_send_);
    algo_proc_idx_[task_id].store(proc_col + 1);
    cv_send_.notify_one();
}
}
}