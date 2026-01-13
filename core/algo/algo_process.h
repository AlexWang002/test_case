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
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <condition_variable>
#include <pthread.h>
#include <sched.h>
#include <fstream>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "algo.h"
#include "common/fault_manager.h"
#include "decoder/decoder_emx.h"
#include "rs_new_logger.h"
#include "sync_queue.h"

#define ALGO_FRM_BUF_SIZE (1)
#define ALGO_THREAD_NUM (1)
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
// using TimePoint = std::chrono::steady_clock::time_point;
// extern SyncQueue<TimePoint> inputTimeQueue;
// extern SyncQueue<TimePoint> inputTimeQueue1;

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
class CloudManager
{
  private:

    uint16_t dist_wave0[760][192];
    uint16_t dist_wave1[760][192];
    uint16_t refl_wave0[760][192];
    uint16_t refl_wave1[760][192];
    uint16_t attr_wave0[760][192];
    uint16_t attr_wave1[760][192];

    static constexpr int32_t kCalcTimeout_{100000};
    static constexpr int32_t kMaxCloudNum_{7600};
    std::mutex mtx_recv_;
    std::mutex mtx_calc_;
    std::condition_variable cv_recv_;
    std::condition_variable cv_calc_;
    int32_t last_pkt_recv_{0};

    // 存储原始的点云
    RSEMXMsopHeader proc_clouds_[kMaxCloudNum_];
    std::atomic<int32_t> proc_cloud_idx_{0};
    // 使用状态跟踪

    AlgoFunction::tstFrameBuffer frame_buffer_[ALGO_FRM_BUF_SIZE];
    std::atomic<int32_t> recv_buffer_idx_{0};
    std::atomic<int32_t> proc_buffer_idx_{0};
    std::atomic<int32_t> cacl_done_;

    std::vector<std::thread> algo_handle_threads_;
    std::atomic<bool> to_exit_handle_{false};
    std::function<void(const uint8_t* pkt, size_t size, 
                        const uint16_t* dist_p,
                        const uint16_t* refl_p,
                        const uint16_t* attr_p)> cb_send_;
    AlgoFunction algo_func_;
#ifdef ALGO_REINJ
    std::ofstream processed_file_;
    int inj_frame_cnt_{0};
#endif

  private:
    void setThreadName(const std::string& kName);
    void assemblePkt(int32_t proc_col, AlgoFunction::tstFrameBuffer* frame_buffer, uint16_t* dist, uint8_t* ref, int32_t surface_id);
    void algoFinalProcess(void);
    void algoProcess(int32_t task_id);
    bool sendEnoughData(int32_t col);
    bool recvEnoughData(int32_t col, AlgoFunction::tstFrameBuffer* frame_buffer);
    void updateAlgoIdx(int32_t proc_col, uint32_t task_id);

  public:
    bool process_delay_switch_ = false;
    void start(void);
    bool stop(void);
    void receiveCloud(const uint8_t* kMsopData, int32_t msop_data_size);
    void regCallback(const std::function<void(const uint8_t* pkt, size_t size,
                                const uint16_t* dist_p,
                                const uint16_t* refl_p,
                                const uint16_t* attr_p)>& kCbSend);

};

}  // namespace lidar
}  // namespace robosense

#endif /* I_ALGO_PROCESS_H */