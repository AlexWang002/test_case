/*******************************************************************************
 * \addtogroup impl
 * \{
 * \headerfile rs_lidar_sdk_impl.h "rs_lidar_sdk_impl.h"
 * \brief
 * \version 0.1
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-07 | Init version |
 * | 1.0 | 2025-10-14 | Add function injectAlarmInfo and powerOff and setCalibrationEnable |
 *
 ******************************************************************************/
#ifndef I_RS_LIDAR_SDK_IMPL_H
#define I_RS_LIDAR_SDK_IMPL_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "algo/algo_process.h"
#include "common/fps_counter.h"
#include "json.hpp"
#include "lidar_sdk_api.h"
#include "sync_queue.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense {
namespace lidar {

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/
#define MIPI_DATA_QUEUE_SIZE   (6)
#define POINT_CLOUD_QUEUE_SIZE (3)

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
struct MipiFrame {
    uint8_t* data;
    uint32_t len;
    uint32_t index;
};

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
/**
 * \class RSLidarSdkImpl rs_lidar_sdk_impl.h "rs_lidar_sdk_impl.h"
 * \brief This class implements the Lidar SDK interface.
 */
class RSLidarSdkImpl {
  public:
    RSLidarSdkImpl();
    ~RSLidarSdkImpl();

    // 接口实现
    const char* getLidarSdkVersion();
    LidarSdkErrorCode init(const LidarSdkCbks* fptrCbks, const char* configPath);
    LidarSdkErrorCode deInit();
    LidarSdkErrorCode start();
    LidarSdkErrorCode stop();
    LidarSdkErrorCode powerOff(void);
    LidarSdkErrorCode setCalibrationEnable(const bool enabled);
    LidarSdkErrorCode injectAdc(LidarSensorIndex sensor, const void* ptrAdc);
    LidarSdkErrorCode injectAlarmInfo(const LidarAlarmInfo& lidarAlarmInfo);
    LidarSdkErrorCode writeDid(uint16_t did, uint8_t* data, uint16_t dataLen, uint8_t* nrc);
    LidarSdkErrorCode readDid(uint16_t did, uint8_t* data, uint16_t* dataLen, uint8_t* nrc);
    LidarSdkErrorCode ridControl(uint8_t mode, uint16_t rid, uint8_t* dataIn, uint16_t dataInLen, uint8_t* dataOut,
                                 uint16_t* dataOutLen, uint8_t* nrc);
    bool getAlarmInfo();
    static RSLidarSdkImpl& getInstance() {
        static RSLidarSdkImpl instance;
        return instance;
    }

  private:
    // 使用自定义删除器的智能指针类型
    struct LidarCloudDeleter {
        void operator()(LidarPointCloudPackets* p) const {
            if (p) {
                if (p->lidar_parameter) {
                    free(p->lidar_parameter);
                    p->lidar_parameter = nullptr;
                }
                free(p);
            }
        }
    };
    bool lostpkg[18] = {false};
    uint8_t cur_rear_{0};
    bool frm_normal_detected_{true};
    bool frm_normal_reported_{false};
    std::chrono::steady_clock::time_point frm_normal_start_time_;
    using LidarPointCloudPtr = std::unique_ptr<LidarPointCloudPackets, LidarCloudDeleter>;

    // 内部状态
    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> is_started_{false};
    std::atomic<bool> is_running_{false};
    std::mutex mutex_;

    LidarSdkCbks callbacks_;
    nlohmann::json json_data_;
    bool alarm_status_{false};

    // 配置路径
    std::string config_path_;
    std::shared_ptr<Decoder> decoder_ptr_;
    std::shared_ptr<CloudManager> cloud_manager_ptr_;
    std::shared_ptr<FPSCounter> pointcloud_fps_counter_ptr_;
    std::shared_ptr<FPSCounter> device_info_fps_counter_ptr_;
    std::thread handle_lidar_mipi_data_thread_;
    std::thread handle_process_msop_data_thread_;
    SyncQueue<LidarPointCloudPtr> point_cloud_queue_;

    using MipiFramePtr = std::shared_ptr<MipiFrame>;
    SyncQueue<MipiFramePtr> mipi_data_queue_;
    LidarSensorIndex sensor_index_ {MIDDLE_LIDAR};
    uint16_t sdk_version_encoded_ {0};

    std::function<void(const uint8_t* mipi_data, const uint32_t mipi_data_len)> funcCheckMipiCrc_;

    // 内部方法
    bool setLidarSdkCallbacks(const LidarSdkCbks* src);
    void clearLidarSdkCallbacks();
    void putMsop(const uint8_t* data, uint32_t size);
    void handleMsopData();
    void splitFrame(uint32_t point_num);
    void runDeviceInfoCallback();
    void handleFault(const std::string& message, FaultBits fault);
    void releaseBuffer(LidarAdcBuffer* buffer);
    int32_t findCurrentPacketPart(uint16_t seq, const std::vector<PacketRangePerMipiFrame>& ranges);
    bool checkContinuity(int32_t pre, int32_t cur, int32_t diff);
    void handleLidarMipiData();
    bool loadConfiguration(const std::string& configPath);

    LidarPointCloudPtr deepCopyLidarCloud(const LidarPointCloudPackets* src);
};

} // namespace lidar
} // namespace robosense

/** \} impl */
#endif /* I_RS_LIDAR_SDK_IMPL_H */
