/*******************************************************************************
 * \addtogroup decoder
 * \{
 * \file general_decoder.cpp
 * \brief Defines general decoder base class for all RoboSense LiDAR data.
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

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "general_decoder.h"
#include "rs_new_logger.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

#define EMX_BLOCKS_PER_FRAME (1520U)
#define EMX_INNER_PARAM_SIZE (500U)

/******************************************************************************/
/*          Definition of public functions of classes or templates            */
/******************************************************************************/
/**
 * \brief The constructor of Decoder.
 * \param[in] const_param The const parameter for MSOP info, such as msop len,
 *                        difop len, device info len, packet range etc.
 * \param[in] param The parameter for Lidar Info, such as lidar type,
 *                  min distance, max distance, etc.
 */
Decoder::Decoder(const RSDecoderConstParam& const_param,
                 const RSDecoderParam& param) :
                    const_param_(const_param),
                    param_(param),
                    angles_ready_(false),
                    last_frame_cnt_(0U),
                    point_num_(0U),
                    theoretical_point_num_(0U),
                    theoretical_mipi_data_len_(0U),
                    min_distance_(const_param.distance_min),
                    max_distance_(const_param.distance_max) {
    float min_tmp = (param.min_distance < 0) ? 0.0f : param.min_distance;
    float max_tmp = (param.max_distance < 0) ? 0.0f : param.max_distance;
    min_distance_ = (min_tmp > 1e-6) ? min_tmp : min_distance_;
    max_distance_ = (max_tmp > 1e-6) ? max_tmp : max_distance_;
}

/**
 * \brief Register the callback function for split frame.
 * \param[out] cb_split_frame The pointer of callback function for split frame.
 */
void Decoder::regCallback(const std::function<void(uint32_t)>& cb_split_frame) {
    cb_split_frame_ = cb_split_frame;
}

/**
 * \brief Process the DIFOP_2 packet.
 * \param[in] pkt The pointer of DIFOP_2 packet.
 * \param[in] size The size of DIFOP_2 packet.
 * \retval true: DIFOP_2 packet decoding succeeded.
 * \retval false: DIFOP_2 packet decoding failed.
 */
bool Decoder::processDifopPkt(const uint8_t* pkt, size_t size) {
    return decodeDifopPkt(pkt, size);
}

/**
 * \brief Process the MSOP packet.
 * \param[in] pkt The pointer of MSOP packet.
 * \param[in] size The size of MSOP packet.
 * \retval true: MSOP packet decoding succeeded.
 * \retval false: MSOP packet decoding failed.
 */
bool Decoder::processMsopPkt(const uint8_t* pkt, size_t size) {
    return decodeMsopPkt(pkt, size);
}

/**
 * \brief Process the DIFOP_1 packet.
 * \param[in] pkt The pointer of DIFOP_1 packet.
 * \param[in] size The size of DIFOP_1 packet.
 * \retval true: DIFOP_1 packet decoding succeeded.
 * \retval false: DIFOP_1 packet decoding failed.
 */
bool Decoder::processDeviceInfoPkt(const uint8_t* pkt, size_t size) {
    return decodeDeviceInfoPkt(pkt, size);
}

/**
 * \brief Init the decoder.
 * \retval true: Init succeeded.
 * \retval false: Init failed.
 */
bool Decoder::init() {
    if (!allocatePointCloud()) {
        LogError("Failed to allocate point cloud!");
        FaultManager8::getInstance().setFault(FaultBits8::LidarPointCloudBufferNull);
        return false;
    }

    if (!allocateDeviceInfo()) {
        LogError("Failed to allocate device info!");
        FaultManager8::getInstance().setFault(FaultBits8::LidarPointCloudBufferNull);
        return false;
    }

    if (FaultManager8::getInstance().hasFault(FaultBits8::LidarPointCloudBufferNull)){
        FaultManager8::getInstance().clearFault(FaultBits8::LidarPointCloudBufferNull);
    }
    initPointCloud();
    initDeviceInfo();

    return true;
}

/**
 * \brief Reset the point cloud.
 */
void Decoder::resetPointCloud() {
    if (point_cloud_) {
        initPointCloud();
    }
}

/**
 * \brief Reset the device info.
 */
void Decoder::resetDeviceInfo() {
    if (device_info_) {
        initDeviceInfo();
    }
}

/**
 * \brief Get the theoretical points per frame.
 * \return The theoretical points per frame.
 */
uint32_t Decoder::getTheoreticalPointsPerFrame() {
    return theoretical_point_num_;
}

/**
 * \brief Get the theoretical MIPI data length.
 * \return The theoretical MIPI data length.
 */
uint32_t Decoder::getTheoreticalMipiDataLength() {
    return theoretical_mipi_data_len_;
}

/**
 * \brief Get the data length of MSOP packet.
 * \return The data length of MSOP packet.
 */
uint16_t Decoder::getMsopLength() {
    return const_param_.msop_len;
}

/**
 * \brief Get the data length of DIFOP packet.
 * \return The data length of DIFOP packet.
 */
uint16_t Decoder::getDifopLength() {
    return const_param_.difop_len;
}

/**
 * \brief Get the data length of device info packet.
 * \return The data length of device info packet.
 */
uint16_t Decoder::getDeviceInfoLength() {
    return const_param_.device_info_len;
}

/**
 * \brief Get the packet ranges.
 * \return The packet ranges.
 */
std::vector<PacketRangePerMipiFrame> Decoder::getPacketRanges() {
    return packet_ranges_;
}

/******************************************************************************/
/*        Definition of protected functions of classes or templates           */
/******************************************************************************/
/**
 * \brief Allocate memory for point cloud.
 * \retval true: Allocate succeeded.
 * \retval false: Allocate failed.
 */
bool Decoder::allocatePointCloud() {
    uint32_t pointCount = getTheoreticalPointsPerFrame();
    if (pointCount == 0 || pointCount > MAX_POINTCLOUD_NUM) {
        LogError("Invalid point count: {}", std::to_string(pointCount));
        return false;
    }
    size_t requiredSize = sizeof(LidarPointCloudPackets) +
                            (EMX_BLOCKS_PER_FRAME - 1) * sizeof(DataBlock);

    LidarPointCloudPackets* rawPtr =
            static_cast<LidarPointCloudPackets*>(malloc(requiredSize));
    if (!rawPtr) {
        LogError("Failed to allocate memory for point cloud!");
        return false;
    }
    memset((void*)rawPtr, 0, requiredSize);
    point_cloud_.reset(rawPtr);

    return true;
}

/**
 * \brief Allocate memory for device info.
 * \retval true: Allocate succeeded.
 * \retval false: Allocate failed.
 */
bool Decoder::allocateDeviceInfo() {
    try {
        device_info_ = std::make_unique<LidarDeviceInfo>();
        return true;
    } catch (const std::bad_alloc&) {
        LogError("Failed to allocate memory for device info!");
        return false;
    }
}

/**
 * \brief Init the point cloud.
 */
void Decoder::initPointCloud() {
    if (!point_cloud_) {
        return;
    }

    auto& cloud = *point_cloud_;
    cloud.protocol_version = 0x0007U; // LidarSDK Struct Version
    cloud.return_mode = 0x1U;
    cloud.sync_status = 0x0U;
    cloud.frame_sync = 0x0U;
    cloud.mirror_id = 0x0U;
    uint32_t pointCount = getTheoreticalPointsPerFrame();
    if (0U == pointCount || pointCount > MAX_POINTCLOUD_NUM) {
        LogError("Invalid point count in init: {}", pointCount);
        pointCount = 0U;
    }
    cloud.point_num = pointCount;
    cloud.frame_seq = 0U;
    cloud.frame_timestamp = 0U;
}

/**
 * \brief Init the device info.
 */
void Decoder::initDeviceInfo() {
    if (nullptr == device_info_) {
        LogError("device_info_ is null pointer!");
        return;
    }

    device_info_->firware_version = 0U;
    device_info_->motor_speed = 0x6FU;
    device_info_->return_mode = 0x01U;
    device_info_->timestamp = 0U;
    device_info_->lidar_operation_state = 0x00U;
    device_info_->lidar_fault_state = 0x00U;
    device_info_->sdk_total_fault_number = 0U;
    device_info_->sdk_fault_code_position = 0U;
    device_info_->supplier_internal_fault_id = 0U;
    std::fill(std::begin(device_info_->supplier_internal_fault_indicate),
              std::end(device_info_->supplier_internal_fault_indicate), 0U);
    device_info_->supplier_internal_fault_number = 0U;
    device_info_->supplier_internal_fault_position = 0U;

    device_info_->time_sync_mode = 0x00U;
    device_info_->time_sync_status = 0x00U;
    device_info_->time_offset = 0U;
    std::fill(std::begin(device_info_->lidar_product_sn),
              std::end(device_info_->lidar_product_sn), 0U);
    device_info_->manufacture = 0x03U;
    device_info_->model = 0U;
    std::fill(std::begin(device_info_->reserved),
              std::end(device_info_->reserved), 0U);
}

/**
 * \brief Check if the distance is in the section.
 * \param[in] distance The distance to be checked.
 * \retval true: The input distance is in the section.
 * \retval false: The input distance is out of the section.
 */
bool Decoder::distance_section(float distance) {
    return (min_distance_ <= distance) && (distance <= max_distance_);
}

} // namespace robosense::lidar

/* \}  decoder */