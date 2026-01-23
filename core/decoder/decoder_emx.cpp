/*******************************************************************************
 * \addtogroup decoder
 * \{
 * \file decoder_emx.cpp
 * \brief Defines the decoder class for the RoboSense EMX series LiDAR data.
 * \version 0.2
 * \date 2025-08-04
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-04 | Init version |
 * | 0.2 | 2025-08-04 | Add comments |
 * | 0.3 | 2025-08-22 | Update MSOP decoder |
 *
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cmath>
#include <cstdint>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "common/fault_manager.h"
#include "decoder_emx.h"
#include "rs_new_logger.h"
#include "time_utils.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense {
namespace lidar {

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Convert a 64-bit integer from network byte order to host byte order.
 * \param[in] x The 64-bit integer in network byte order.
 * \return The 64-bit integer in host byte order.
 */
inline uint64_t ntohll(uint64_t x) {
    // 使用移位操作手动交换字节
    uint32_t high = static_cast<uint32_t>(x >> 32);
    uint32_t low = static_cast<uint32_t>(x & 0xFFFFFFFF);

    // 分别交换高32位和低32位，然后合并
    return (static_cast<uint64_t>(ntohl(low)) << 32) | static_cast<uint64_t>(ntohl(high));
}

/**
 * \brief Swap the byte order of a 16-bit integer.
 * \param[in] value The 16-bit integer to be swapped.
 * \return The swapped 16-bit integer.
 */
inline int16_t RS_SWAP_INT16(int16_t value) {
    return (value << 8) | ((value >> 8) & 0xFF);
}

/******************************************************************************/
/*          Definition of public functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Constructor of the DecoderRSEMX class.
 * \param[in] param The decoder parameter
 */
DecoderRSEMX::DecoderRSEMX(const RSDecoderParam& param) : Decoder(getConstParam(), param) {
    packet_ranges_ = {{1, 281, 281},    {282, 562, 281},   {563, 843, 281},
                      {844, 1124, 281}, {1125, 1405, 281}, {1406, 1520, 115}};
    yaw_offset_.fill(0);

    std::array<int32_t, EMX_PIXELS_PER_COLUMN> defaultAngle;
    constexpr int32_t START_ANGLE{-2416};
    constexpr int32_t ANGLE_STEP{21};

    for (uint16_t i = 0; i < EMX_PIXELS_PER_COLUMN; ++i) {
        defaultAngle[i] = START_ANGLE + i * ANGLE_STEP;
    }
    pitch_angle_ = defaultAngle;
    surface_pitch_offset_.fill(0);
    theoretical_point_num_ = EMX_PIXELS_PER_COLUMN * EMX_COLUMNS_PER_FRAME;
    theoretical_mipi_data_len_ = EMX_MIPI_DATA_LEN;
    cached_pkt_timestamps_.resize(EMX_COLUMNS_PER_FRAME, 0);
}

/**
 * \brief Decode the DIFOP2 packet.
 * \param[in] packet The packet data.
 * \param[in] size The packet size.
 * \return The packet is decoded successfully.
 * \retval true: The packet decoding succeeded.
 * \retval false: The packet decoding failed.
 */
bool DecoderRSEMX::decodeDifopPkt(const uint8_t* packet, size_t size) {
    if ((true == difop2_received_) && (true == angles_ready_)) {
        return true;
    }

    if (packet == nullptr) {
        LogError("[DecoderRSEMX::decodeDifopPkt] packet is nullptr");
        return false;
    }
    const size_t difop2_len = sizeof(RSEMXDifop2Pkt);

    if (size != difop2_len) {
        LogError("wrong difop2 packet size: {}", size);
        return false;
    } else if (0 != memcmp(packet, const_param_.difop_id, const_param_.difop_id_len)) {
        LogError("wrong difop2 packet id");
        return false;
    }

    if (false == difop2_received_) {
        memcpy(difop2_, packet, difop2_len);
        difop2_received_ = true;
    }
    const RSEMXDifop2Pkt& pkt = *(const RSEMXDifop2Pkt*)(packet);

    for (uint16_t i = 0; i < EMX_VECSELS_PER_COLUMN; i++) {
        yaw_offset_[i] = static_cast<int32_t>(pkt.yaw_offset[i]);
    }

    for (uint16_t i = 0; i < EMX_PIXELS_PER_COLUMN; i++) {
        pitch_angle_[i] = static_cast<int32_t>(RS_SWAP_INT16(pkt.pitch_angle[i]) >> 1);
    }

    for (uint16_t i = 0; i < EMX_SURFACE_NUM; i++) {
        surface_pitch_offset_[i] = static_cast<int32_t>(RS_SWAP_INT16(pkt.surface_pitch_offset[i]) >> 1);
    }
    angles_ready_ = true;

    return true;
}

/**
 * \brief Decode the device info packet.
 * \param[in] packet The packet data.
 * \param[in] size The packet size.
 * \return The packet is decoded successfully.
 * \retval true: The packet decoding succeeded.
 * \retval false: The packet decoding failed.
 */
bool DecoderRSEMX::decodeDeviceInfoPkt(const uint8_t* packet, size_t size) {
    if (packet == nullptr) {
        LogError("[DecoderRSEMX::decodeDeviceInfoPkt] packet is nullptr");
        return false;
    }
    const size_t device_info_len = sizeof(RSEMXDeviceInfoPkt);

    if (size != device_info_len) {
        LogError("wrong difop1 packet size: {}", size);
        return false;
    } else if (0 != memcmp(packet, const_param_.device_info_id, const_param_.device_info_id_len)) {
        LogError("wrong device info id");
        return false;
    } else if (!device_info_) {
        LogError("device info is nullptr");
        return false;
    }

    const RSEMXDeviceInfoPkt& pkt = *(const RSEMXDeviceInfoPkt*)(packet);
    device_info_->firware_version = ntohs(pkt.device_version.sw_version);
    constexpr int obstruct_report_time_{500};
    static bool log_out_{false};

    if (pkt.work_info.win_block_status != 0) {
        auto now = std::chrono::steady_clock::now();
        if (!obstruct_detected_) {
            // 第一次检测到遮挡，开始计时
            obstruct_start_time_ = now;
            obstruct_detected_ = true;
            LogDebug("Obstruction detected, starting timer");
        } else {
            // 持续检测到遮挡，检查是否达到500ms
            auto duration = utils::timeInterval(obstruct_start_time_, now);

            if ((duration >= obstruct_report_time_) && (!log_out_)) {
                LogWarn("Lidar Obstruction Fault persisted for 500ms, win_block_status:{}",
                        pkt.work_info.win_block_status);
                FaultManager64::getInstance().setFault(FaultBits::LidarObstructionFault);
                log_out_ = true;
            }
        }
    } else {
        // 没有检测到遮挡
        if (obstruct_detected_) {
            LogDebug("Obstruction cleared");
            obstruct_detected_ = false;
            log_out_ = false;

            // 清除故障（如果之前已经报告过）
            if (FaultManager64::getInstance().hasFault(FaultBits::LidarObstructionFault)) {
                FaultManager64::getInstance().clearFault(FaultBits::LidarObstructionFault);
                LogInfo("Lidar Obstruction Fault cleared");
            }
        }
    }
    device_info_->lidar_operation_state = (pkt.fault_level != 0);
    device_info_->lidar_fault_state = (pkt.fault_level > 1) ? (2) : pkt.fault_level;
    device_info_->sdk_total_fault_number = FaultManager64::getInstance().countFaults();
    device_info_->sdk_fault_code_position = FaultManager64::getInstance().getFaults();
    device_info_->supplier_internal_fault_id = (pkt.fault_id1);

    (void)std::copy(std::begin(pkt.fault_value1), std::end(pkt.fault_value1),
                            device_info_->supplier_internal_fault_indicate);

    device_info_->time_sync_mode = pkt.time_info.time_sync_mode;
    device_info_->time_sync_status = (pkt.time_info.time_sync_status == 0x1) ? 1 : 0;
    device_info_->timestamp = parseTimeUTCWithUs(pkt.time_info.timestamp);
    device_info_->time_offset = ntohll(pkt.time_info.time_offset);

    (void)std::copy(std::begin(pkt.customer_sn), std::end(pkt.customer_sn), device_info_->lidar_product_sn);
    device_info_->model = 0x01;
    std::memcpy(&device_info_->reserved[0], &pkt.fault_id2, 2);
    std::memcpy(&device_info_->reserved[2], &pkt.fault_value2, 4);
    device_info_->reserved[6] = 0;

    for(size_t i = 0; i < 10; i++) {
        if(1 == ((pkt.dtc >> i) & 0x01)) {
            device_info_->reserved[6]++;
        }
    }
    std::memcpy(&device_info_->reserved[7], &pkt.dtc, 3);
    std::memcpy(&device_info_->reserved[11], &pkt.work_info.win_block_status, 19);

    std::unique_lock<std::mutex> lock(mtx_point_cloud_);
    // packets sync_status is take from TimesyncStatus in DIFOP1 and keep consistent with that in device_info.
    point_cloud_->sync_status = device_info_->time_sync_status;

    return true;
}

/**
 * \brief Decode the MSOP packet.
 * \param[in] packet_header The packet header data.
 * \param[in] size The packet header size.
 * \param[in] dist_p The algo processed distance.
 * \param[in] refl_p The algo processed reflection.
 * \param[in] attr_p The algo processed attribute label.
 * \return The packet is decoded successfully.
 * \retval true: The packet decoding succeeded.
 * \retval false: The packet decoding failed.
 */
bool DecoderRSEMX::decodeMsopPkt(const uint8_t* packet_header, size_t size,
                                const uint16_t* dist_p,
                                const uint16_t* refl_p,
                                const uint16_t* attr_p) {
    if (packet_header == nullptr) {
        LogError("[DecoderRSEMX::decodeMsopPkt] packet is nullptr");
        return false;
    }
    const size_t msop_pkt_header_size = sizeof(RSEMXMsopHeader);
    const uint16_t pixel_per_pkt = const_param_.pixel_per_pkt;

    if (size != msop_pkt_header_size) {
        LogError("Wrong MSOP packet header size: {}", size);
        return false;
    } else if (0 != memcmp(packet_header, const_param_.msop_id, const_param_.msop_id_len)) {
        LogError("Wrong MSOP packet id");
        return false;
    } else if (point_cloud_ == nullptr) {
        LogError("point_cloud_ is nullptr");
        return false;
    }
    const RSEMXMsopHeader& header = *reinterpret_cast<const RSEMXMsopHeader*>(packet_header);
    uint64_t pkt_ts = parseTimeUTCWithUs(header.timestamp);
    uint16_t raw_pkt_seq = ntohs(header.pkt_seq); // 1 ~ 1520

    if (raw_pkt_seq < EMX_PKT_SEQ_MIN || raw_pkt_seq > EMX_PKT_SEQ_MAX) {
        LogError("MSOP pkt seq: {}, out of range {} - {}", raw_pkt_seq, EMX_PKT_SEQ_MIN, EMX_COLUMNS_PER_FRAME);
        return false;
    }
    static uint16_t last_pkt_seq{0U};
    static uint64_t first_pkt_ts{0UL};
    uint32_t frame_cnt = ntohl(header.frame_cnt);

    // Determine whether packet loss has occurred
    if ((raw_pkt_seq % EMX_PKT_SEQ_MAX) != ((last_pkt_seq + 1) % EMX_PKT_SEQ_MAX)) {
        LogError("lost pkt, last seq is: {}, current seq is: {} , last frame_cnt:{}, frame_cnt:{}",
                 last_pkt_seq, raw_pkt_seq, last_frame_cnt_, frame_cnt);
    }
    last_pkt_seq = raw_pkt_seq;

    if (EMX_PKT_SEQ_MIN == raw_pkt_seq) {
        setPacketHeader(header);
        first_pkt_ts = pkt_ts;
    }
    uint16_t pkt_seq = raw_pkt_seq - 1; // 0 ~ 1519

    // Determine whether frame loss has occurred
    if (frame_cnt != last_frame_cnt_) {
        // If the current point number is not zero, it means that the frame has been lost
        if (0 != point_num_) {
            cb_split_frame_(point_num_); // call the split frame callback
            LogError("Frame loss, frame_cnt:{}, last_frame_cnt:{}, point_num:{}",
                frame_cnt, last_frame_cnt_, point_num_);
        }
        // And the current frame is a new frame.
        last_frame_cnt_ = frame_cnt;
        point_num_ = 0; // reset point num
    }
    uint8_t surface_index = header.surface_id;

    if (surface_index >= EMX_SURFACE_NUM) {
        LogError("surface_id out of range, surface_id is: {}, max is {}", surface_index, EMX_SURFACE_NUM);
        return false;
    }

    auto& current_block = point_cloud_->packets[pkt_seq];
    // Set the Block header
    current_block.time_offset = static_cast<uint16_t>(std::round(0.1 * (pkt_ts - first_pkt_ts))); // time offset, unit 10us
    current_block.motor_speed = header.pitch_offset;
    current_block.azimuth = header.yaw;

    for (uint16_t pixel_id = 0; pixel_id < pixel_per_pkt; ++pixel_id) {
        ChannelData& point = current_block.channel_data[pixel_id];
        point.radius = htons(dist_p[pixel_id]);
        point.intensity = refl_p[pixel_id];
        point.point_attribute = attr_p[pixel_id];
    }
    point_num_ += pixel_per_pkt;
    point_cloud_->point_num = point_num_;
    point_cloud_->data_length += sizeof(DataBlock);

    if (EMX_COLUMNS_PER_FRAME == raw_pkt_seq) {
        cb_split_frame_(point_num_);
        point_num_ = 0;
    }

    return true;
}

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Set the point cloud info.
 * \param[in] header The msop packet's header.
 */
void DecoderRSEMX::setPacketHeader(const RSEMXMsopHeader& header) {
    if (nullptr == point_cloud_) {
        LogError("point_cloud_ is nullptr");
        return;
    }
    std::unique_lock<std::mutex> lock(mtx_point_cloud_);
    point_cloud_->frame_timestamp = parseTimeUTCWithUs(header.timestamp);
    point_cloud_->point_num = 0U;
    point_cloud_->frame_seq = ntohl(header.frame_cnt);
    static bool lidar_parameter_malloc{false};

    if (difop2_received_) {
        if (!lidar_parameter_malloc) {
            point_cloud_->lidar_parameter = malloc(sizeof(difop2_));
            lidar_parameter_malloc = true;
        }
        memset(point_cloud_->lidar_parameter, 0, sizeof(difop2_));
        memcpy(point_cloud_->lidar_parameter, difop2_, sizeof(difop2_));
        point_cloud_->lidar_parameter_length = sizeof(RSEMXDifop2Pkt);
    } else {
        LogError("difop2 data not received");
        point_cloud_->lidar_parameter = nullptr;
        point_cloud_->lidar_parameter_length = 0;
    }
    point_cloud_->protocol_version = LIDAR_SDK_API_VER_MAJOR * 10000 + LIDAR_SDK_API_VER_MINOR * 100 + LIDAR_SDK_API_VER_PATCH;
    point_cloud_->return_mode = 0x01; // SDK默认输出为0x01最强回波
    point_cloud_->frame_sync = header.synchronized;
    point_cloud_->mirror_id = header.surface_id;
    point_cloud_->packet_num = EMX_PKT_SEQ_MAX;
    point_cloud_->data_length = sizeof(LidarPointCloudPackets) - sizeof(DataBlock);
    uint8_t* data_id = (uint8_t*)&(point_cloud_->data_id);
    memcpy(data_id, const_param_.msop_id, 4);
}

} // namespace lidar
} // namespace robosense

/* \}  decoder */