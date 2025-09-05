/*******************************************************************************
 * \addtogroup decoder
 * \{
 * \headerfile general_decoder.h "general_decoder.h"
 * \brief Declares general decoder base class for all RoboSense LiDAR data.
 * \version 0.2
 * \date 2025-08-06
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-06 | Init version |
 * | 0.2 | 2025-08-06 | Add comments |
 *
 ******************************************************************************/
#ifndef I_GENERAL_DECODER_H
#define I_GENERAL_DECODER_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <vector>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "lidar_sdk_api.h"
#include "driver_param.h"
#include "trigon.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/
constexpr uint32_t MAX_POINTCLOUD_NUM{5000000U};

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
/**
 * \brief Decoder const param.
 * \details This struct is used to store the const param of decoder.
 */
struct RSDecoderConstParam {
    // packet len
    uint16_t msop_len;          ///< data length of MSOP
    uint16_t difop_len;         ///< data length of DIFOP_2
    int16_t device_info_len;    ///< data length of device info (DIFOP_1)

    // packet identity
    uint8_t msop_id_len;
    uint8_t difop_id_len;
    uint8_t device_info_id_len;
    uint8_t msop_id[8];
    uint8_t difop_id[8];
    uint8_t device_info_id[8];

    uint16_t pixel_per_pkt;

    // distance
    float distance_min;
    float distance_max;
    float distance_res;
};

/**
 * \brief Packet range per MIPI frame.
 * \details This struct is used to store the packet range per MIPI frame.
 */
struct PacketRangePerMipiFrame {
    int32_t start;
    int32_t end;
    int32_t expectedCount;
};

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
/**
 * \brief Decoder base class.
 * \details This class is the base class of all decoders of RoboSense LiDAR.
 */
class Decoder {
  public:
    virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) = 0;
    virtual bool decodeDifopPkt(const uint8_t* pkt, size_t size) = 0;
    virtual bool decodeDeviceInfoPkt(const uint8_t* pkt, size_t size) = 0;
    virtual ~Decoder() = default;

    explicit Decoder(const RSDecoderConstParam& const_param,
                     const RSDecoderParam& param);

    bool init();
    bool processMsopPkt(const uint8_t* pkt, size_t size);
    bool processDifopPkt(const uint8_t* pkt, size_t size);
    bool processDeviceInfoPkt(const uint8_t* pkt, size_t size);

    void regCallback(const std::function<void(uint32_t)>& cb_split_frame);

    void resetPointCloud();
    void resetDeviceInfo();

    std::unique_ptr<LidarPointCloudPackets, void (*)(LidarPointCloudPackets*)> point_cloud_ {
        nullptr,
        [](LidarPointCloudPackets* p) {
            /// \note Using free(p) instead of the default delete indicates that
            ///    the LidarPointCloudPackets object is allocated using C-style malloc.
            if (p) {
                free(p);
            }
        }
    };

    std::mutex mtx_point_cloud_;

    std::unique_ptr<LidarDeviceInfo> device_info_;
    bool is_test_time_delay_ = false;
    uint32_t getTheoreticalPointsPerFrame();
    uint32_t getTheoreticalMipiDataLength();
    uint16_t getMsopLength();
    uint16_t getDifopLength();
    uint16_t getDeviceInfoLength();
    std::vector<PacketRangePerMipiFrame> getPacketRanges();

  protected:
    // TODO Further corrections are needed
    RSDecoderConstParam const_param_; ///< const param
    RSDecoderParam param_;            ///< user param
    std::function<void(uint32_t)> cb_split_frame_;
    // TODO refactor this trigon_ and macro using;
    Trigon trigon_;
#define SIN(angle) (trigon_.sin(angle))
#define COS(angle) (trigon_.cos(angle))

    bool angles_ready_{false}; ///< Is vert_angles/horiz_angles ready from csv file/difop packet?
    uint32_t last_frame_cnt_{0U};
    uint32_t point_num_{0U};
    uint32_t theoretical_point_num_{0U};
    uint32_t theoretical_mipi_data_len_{0U};
    float min_distance_{0.0f};
    float max_distance_{0.0f};

    std::vector<PacketRangePerMipiFrame> packet_ranges_;

    bool allocatePointCloud();
    bool allocateDeviceInfo();
    void initPointCloud();
    void initDeviceInfo();
    bool distance_section(float distance);
};

} // namespace robosense::lidar

/** \} decoder */
#endif /* I_GENERAL_DECODER_H */
