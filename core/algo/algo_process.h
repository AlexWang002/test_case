/*******************************************************************************
 * \addtogroup algo
 * \{
 * \headerfile algo_process.h "algo_process.h"
 * \brief
 * \version 0.2
 * \date 2025-08-06
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-06-07 | Init version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-08-06 | Add comments;|
 * |     |            | Fix warnings |
 *
 ******************************************************************************/
#ifndef I_ALGO_PROCESS_H
#define I_ALGO_PROCESS_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <pthread.h>
#include <sched.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "algo.h"
#include "common/fault_manager.h"
#include "decoder/include/decoder_emx.h"
#include "rs_new_logger.h"

#define ALGO_FRM_BUF_SIZE (3)
#define ALGO_THREAD_NUM (3)
#define ALGO_LOSS_PKT_CODE (-1)
#define ALGO_VERSION_MAJOR 01
#define ALGO_VERSION_MINOR 00
#define ALGO_VERSION_PATCH 16

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense
{
namespace lidar
{
/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
class CloudManager
{
  private:
    static constexpr int32_t kCalcTimeout_{100000};
    static constexpr int32_t kMaxCloudNum_{7600};
    static constexpr int32_t algo_frm_buf_size_{3};
    static constexpr int32_t algo_thread_num_{2};
    static constexpr int32_t algo_loss_pkt_code{-1};
    std::mutex mtx_recv_;
    std::mutex mtx_send_;
    std::mutex mtx_calc_;
    std::condition_variable cv_recv_;
    std::condition_variable cv_send_;
    std::condition_variable cv_calc_;
    int32_t last_pkt_recv_{0};

    // 存储原始的点云
    RSEMXMsopPkt proc_clouds_[kMaxCloudNum_];
    std::atomic<int32_t> proc_cloud_idx_{0};
    // 使用状态跟踪

    AlgoFunction::tstFrameBuffer frame_buffer_[ALGO_FRM_BUF_SIZE];
    std::atomic<int32_t> recv_buffer_idx_{0};
    std::atomic<int32_t> proc_buffer_idx_{0};
    std::atomic<int32_t> cacl_done_[ALGO_THREAD_NUM];
    std::array<std::atomic<int32_t>, ALGO_THREAD_NUM> algo_proc_idx_;

    std::vector<std::thread> algo_handle_threads_;
    std::thread handle_thread_;
    std::atomic<bool> to_exit_handle_{false};
    std::function<void(const uint8_t* pkt, size_t size)> cb_send_;

    AlgoFunction algo_func_;

#ifdef ALGO_WRITE_FILE
    std::ofstream dist_file_;
    std::ofstream ref_file_;
    bool write_file_{true};  // 是否写入文件
#endif

#ifdef ALGO_REINJ
    std::ofstream processed_file_;
    int inj_frame_cnt_{0};
#endif

  private:
    void setThreadName(const std::string& kName);
    void assemblePkt(int32_t proc_col, AlgoFunction::tstFrameBuffer* frame_buffer, uint16_t* dist, uint8_t* ref, int32_t surface_id);
#ifdef ALGO_WRITE_FILE
    void writeFileFunc(uint16_t* dist, uint8_t* ref, int x);
#endif
    void algoFinalProcess(void);
    void algoProcess(int32_t task_id);
    bool sendEnoughData(int32_t col);
    bool recvEnoughData(int32_t col, AlgoFunction::tstFrameBuffer* frame_buffer);
    void updateAlgoIdx(int32_t proc_col, uint32_t task_id);

  public:
    void start(void);
    bool stop(void);
    void receiveCloud(const uint8_t* kMsopData, int32_t msop_data_size);
    void regCallback(const std::function<void(const uint8_t* pkt, size_t size)>& kCbSend);

};

}  // namespace lidar
}  // namespace robosense

#endif /* I_ALGO_PROCESS_H */