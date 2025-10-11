/*******************************************************************************
 * \addtogroup examples
 * \{
 * \file rs_msop_parse.cpp
 * \brief
 * \version 0.1
 * \date 2025-08-19
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-19 | Init version |
 *
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <arpa/inet.h>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "rs_msop_parse.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::msop {

/******************************************************************************/
/*                   Definition of local constant data                     */
/******************************************************************************/
static const uint16_t kChannelPerBlock {192U};
static const uint16_t kBlockPerFrame {1520U};
static constexpr uint32_t kPointsPerFrame {192U * 1520U};
static const float kDistanceRes {0.005f};

static constexpr float kYawFactor = 1.0f / 5.12f;
static constexpr double kDegToRadFactor = 3.14159265358979323846 / 180.0;

static const uint8_t kDifop2Id[4] = {0xA5U, 0xFFU, 0x00U, 0xAEU};
static const uint8_t kMsopId[4] = {0x55U, 0xAAU, 0x5AU, 0xA5U};

/******************************************************************************/
/*                 Declaration or Definition of local variables               */
/******************************************************************************/
bool difop2_init_ {false};
LidarDifop2Pkt difop2_;
std::array<int32_t, 24> yaw_offset_;
std::array<int32_t, 192> pitch_angle_;
std::array<int32_t, 2> surface_pitch_offset_;
float sin_table[45000] = {0.0f};
float cos_table[45000] = {0.0f};
float* sin_;
float* cos_;

std::unique_ptr<LidarPointCloud, void (*)(LidarPointCloud*)> point_cloud_ {
    nullptr,
    [](LidarPointCloud* p) {
        free(p);
    }
};

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Swap the int16_t value.
 * \param[in] value The int16_t value to be swapped.
 * \return The swapped int16_t value.
 */
inline int16_t swap_int16(int16_t value) {
    return (value << 8) | ((value >> 8) & 0xFF);
}

/**
 * \brief Get the sin value.
 * \param[in] index The index of the sin value.
 * \return The sin value.
 */
inline float sin_lookup(int32_t index) {
    return sin_[index];
}

/**
 * \brief Get the cos value.
 * \param[in] index The index of the cos value.
 * \return The cos value.
 */
inline float cos_lookup(int32_t index) {
    return cos_[index];
}

/**
 * \brief Check if the distance is valid.
 * \param[in] distance The distance to be checked.
 * \retval true: The input distance is valid.
 * \retval false: The input distance is not valid.
 */
inline bool distance_valid(float distance) {
    return ((distance >= 0.2f) && (distance <= 300.0f));
}

/**
 *
 */
bool readBinData(const std::string& file_path, char* data, uint32_t& len) {
    if (0 != access(file_path.c_str(), F_OK)) {
        std::cout << "File not exist" << file_path << std::endl;
        return false;
    }
    if (0 != access(file_path.c_str(), R_OK)) {
        std::cout << "File exist but not readable" << file_path << std::endl;
        return false;
    }
    std::ifstream file(file_path, std::ios::binary);

    if (!file.is_open()) {
        std::cout << "Failed to open file: " << file_path << std::endl;
        return false;
    }
    file.seekg(0, std::ios::end);
    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (!file.read(data, file_size)) {
        std::cout << "Failed to read file: " << file_path << std::endl;
        file.close();
        return false;
    }

    file.close();
    len = file_size;

    return true;
}

bool savePointCloudToPCD(const std::string& kFileName,
                         LidarPointCloud* cloud,
                         PCDFormat format) {
    if (((!cloud) || (!cloud->point)) || (cloud->point_num == 0)) {
        std::cerr << "Invalid point cloud data" << std::endl;
        return false;
    }
    std::ofstream os;

    // 根据格式打开文件，二进制模式需要指定ios::binary
    if (format == PCDFormat::PCD_BINARY) {
        os.open(kFileName, std::ios::out | std::ios::trunc | std::ios::binary);
    } else {
        os.open(kFileName, std::ios::out | std::ios::trunc);
    }

    if (!os.is_open()) {
        std::cerr << "Failed to open file: " << kFileName << std::endl;
        return false;
    }

    os << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
    os << "VERSION 0.7" << std::endl;
    // 注意：字段顺序需与写入顺序一致
    os << "FIELDS x y z  intensity channel_number timestamp" << std::endl;
    // SIZE：每个字段的字节数（严格对应结构体成员）
    os << "SIZE 4 4 4 1 1 2" << std::endl; // confidence是2字节数组
    // TYPE：字段数据类型（F=float, I=int16, U=uint8, U=uint8, U=uint8[2]）
    os << "TYPE F F F U U I" << std::endl;
    os << "COUNT 1 1 1 1 1 1" << std::endl; // confidence整体作为一个字段
    os << "WIDTH " << cloud->point_num << std::endl;
    os << "HEIGHT 1" << std::endl;
    os << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    os << "POINTS " << cloud->point_num << std::endl;

    if (format == PCDFormat::PCD_ASCII) {
        os << "DATA ascii" << std::endl;
        for (uint32_t i = 0; i < cloud->point_num; ++i) {
            const LidarPoint* kP = &cloud->point[i];
            os << kP->x << " " << kP->y << " " << kP->z << " "
               << static_cast<int>(kP->intensity) << " " // 反射率转int输出
               << static_cast<int>(kP->channel_number)
               << " " // uint8_t转int避免被解析为字符
               << kP->timestamp << std::endl;
        }
    } else {
        os << "DATA binary" << std::endl;
        for (uint32_t i = 0; i < cloud->point_num; ++i) {
            const LidarPoint* kP = &cloud->point[i];

            // 严格按结构体成员顺序写入，不做类型转换
            os.write(reinterpret_cast<const char*>(&kP->x), sizeof(kP->x)); // float (4字节)
            os.write(reinterpret_cast<const char*>(&kP->y), sizeof(kP->y)); // float (4字节)
            os.write(reinterpret_cast<const char*>(&kP->z), sizeof(kP->z)); // float (4字节)
            os.write(reinterpret_cast<const char*>(&kP->intensity), sizeof(kP->intensity)); // uint8_t (1字节，反射率)
            os.write(reinterpret_cast<const char*>(&kP->channel_number), sizeof(kP->channel_number)); // uint8_t (1字节)
            os.write(reinterpret_cast<const char*>(&kP->timestamp), sizeof(kP->timestamp)); // int16_t (2字节)
        }
    }
    os.close();
    std::cout << "Save point cloud as: " << kFileName << std::endl;

    return true;
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
/**
 * \brief Init the msop point cloud.
 * \return The msop point cloud is initialized status.
 * \retval true: The msop point cloud is initialized.
 * \retval false: The msop point cloud is not initialized.
 */
bool initMsop() {
    size_t requiredSize = sizeof(LidarPointCloud) + (kPointsPerFrame - 1) * sizeof(LidarPoint);
    LidarPointCloud* rawPtr = static_cast<LidarPointCloud*>(malloc(requiredSize));

    if (!rawPtr) {
        std::cout << "Failed to allocate memory for point cloud!" << std::endl;
        return false;
    }
    memset((void*)rawPtr, 0, requiredSize);
    point_cloud_.reset(rawPtr);
    yaw_offset_.fill(0);
    pitch_angle_.fill(0);
    surface_pitch_offset_.fill(0);
    difop2_init_ = false;

    for (int32_t i = -9000, j = 0; i < 36000; i++, j++) {
        double rad = (static_cast<double>(i) * 0.01) * kDegToRadFactor;
        sin_table[j] = (float)std::sin(rad);
        cos_table[j] = (float)std::cos(rad);
    }
    sin_ = sin_table + 9000;
    cos_ = cos_table + 9000;

    return true;
}

/**
 * \brief Parse the difop packet.
 * \param[in] packet The packet data.
 * \param[in] size The packet size.
 * \return The packet is parsed successfully.
 * \retval true: The packet parsing succeeded.
 * \retval false: The packet parsing failed.
 */
bool parseDifopPkt(const uint8_t* packet, size_t size) {
    if (packet == nullptr) {
        std::cout << "difop packet is nullptr" << std::endl;
        return false;
    }
    const size_t difop2_len = sizeof(LidarDifop2Pkt);

    if (size != difop2_len) {
        std::cout << "input difop packet size is not equal to difop2 len: "
                  << difop2_len << std::endl;
        return false;
    }
    if (0 != memcmp(packet, kDifop2Id, sizeof(kDifop2Id))) {
        std::cout << "wrong difop2 packet id" << std::endl;
        return false;
    }
    const LidarDifop2Pkt& pkt = *(const LidarDifop2Pkt*)(packet);

    for (uint16_t i = 0U; i < 24U; i++) {
        yaw_offset_[i] = static_cast<int32_t>(pkt.yaw_offset[i]);
    }
    for (uint16_t i = 0U; i < 192U; i++) {
        pitch_angle_[i] = static_cast<int32_t>(swap_int16(pkt.pitch_angle[i]) >> 1);
    }
    for (uint16_t i = 0; i < 2U; i++) {
        surface_pitch_offset_[i] = static_cast<int32_t>(swap_int16(pkt.surface_pitch_offset[i]) >> 1);
    }
    difop2_init_ = true;

    return true;
}

/**
 * \brief Get the point cloud.
 * \param[out] point_cloud The point cloud data.
 * \param[out] size The point cloud size.
 */
void getPointCloud(void* point_cloud, uint32_t size) {
    size = sizeof(LidarPointCloud) + (kPointsPerFrame - 1) * sizeof(LidarPoint);
    memcpy(point_cloud, point_cloud_.get(), size);
}

/**
 * \brief Decode the MSOP packet.
 * \param[in] packet The packet data.
 * \param[in] size The packet size.
 * \return The packet is decoded successfully.
 * \retval true: The packet decoding succeeded.
 * \retval false: The packet decoding failed.
 */
bool parseMsopPkt(const uint8_t* packet, size_t size) {
    if (nullptr == packet) {
        std::cout << "[DecoderRSEMX::decodeMsopPkt] packet is nullptr" << std::endl;
        return false;
    }
    const size_t msop_pkt_size = sizeof(LidarPointCloudPackets) + (kBlockPerFrame - 1) * sizeof(DataBlock);

    if (!difop2_init_) {
        std::cout << "Calibration angles not ready" << std::endl;
        return false;
    } else if (size != msop_pkt_size) {
        std::cout << "Wrong MSOP packet size: " << size << std::endl;
        return false;
    }

    const LidarPointCloudPackets& pkt = *reinterpret_cast<const LidarPointCloudPackets*>(packet);
    const uint8_t* data_id = reinterpret_cast<const uint8_t*>(&(pkt.data_id));

    std::cout << std::hex << std::setw(2) << std::setfill('0');
    for (uint8_t i = 0U; i < 4U; i++) {
        std::cout << "data_id[" << i << "]: " << (int)data_id[i] << std::endl;
    }
    std::cout << std::dec << std::endl;

    if (0 != memcmp(data_id, kMsopId, sizeof(kMsopId))) {
        std::cout << "Wrong MSOP packet id" << std::endl;
        return false;
    } else if (nullptr == point_cloud_) {
        std::cout << "point_cloud_ is nullptr" << std::endl;
        return false;
    }
    uint32_t msop_point_num = pkt.point_num;

    if (kPointsPerFrame != msop_point_num) {
        std::cout << "msop point num " << msop_point_num
                  << " is not equal to kPointsPerFrame: " << kPointsPerFrame
                  << std::endl;
        return false;
    }
    uint64_t pkt_ts = pkt.frame_timestamp;
    std::cout << "pkt_ts: " << pkt_ts << std::endl;
    uint32_t frame_cnt = ntohl(pkt.frame_seq);
    std::cout << "frame_cnt: " << frame_cnt << std::endl;

    const uint8_t surface_id = pkt.mirror_id;
    const int32_t surface_pitch_offset = surface_pitch_offset_[surface_id];
    point_cloud_->frame_seq = frame_cnt;
    point_cloud_->frame_timestamp = pkt_ts;
    point_cloud_->point_num = msop_point_num;

    LidarPoint* current_point = point_cloud_->point;

    for (uint16_t block_idx = 0U; block_idx < kBlockPerFrame; ++block_idx) {
        const DataBlock& block = pkt.packets[block_idx];
        const int32_t yaw_base = static_cast<int32_t>(swap_int16(block.azimuth));
        const ChannelData* channel = block.channel_data;
        const int16_t time_offset = static_cast<int16_t>(block.time_offset);

        for (uint16_t channel_idx = 0U; channel_idx < kChannelPerBlock; ++channel_idx) {
            int32_t yaw_value = yaw_base + yaw_offset_[channel_idx >> 3];
            int32_t yaw = static_cast<int32_t>(kYawFactor * yaw_value);
            const int32_t pitch = surface_pitch_offset + pitch_angle_[channel_idx];
            const float cos_pitch = cos_lookup(pitch);
            const float sin_pitch = sin_lookup(pitch);
            const float cos_yaw = cos_lookup(yaw);
            const float sin_yaw = sin_lookup(yaw);
            const float distance = ntohs(channel->radius) * kDistanceRes;

            if (distance_valid(distance)) {
                float cos_dis = distance * cos_pitch;
                current_point->x = cos_dis * cos_yaw;
                current_point->y = cos_dis * sin_yaw;
                current_point->z = distance * sin_pitch;
            } else {
                current_point->x = NAN;
                current_point->y = NAN;
                current_point->z = NAN;
            }
            current_point->channel_number = channel_idx;
            current_point->intensity = channel->intensity;
            current_point->confidence[0] = channel->point_attribute;
            current_point->timestamp = time_offset;

            current_point++;
            channel++;
        }
    }

    return true;
}

} // namespace robosense::msop

#if 0
int main(int32_t argc, char* argv[]) {
    using namespace robosense::msop;
    bool ret = initMsop();

    if (!ret) {
        std::cout << "initMsop failed" << std::endl;
        return -1;
    }
    char* difop2_data = static_cast<char*>(malloc(500));
    uint32_t difop2_len {0U};
    ret = readBinData("./difop2.bin", difop2_data, difop2_len);

    if (!ret || (500 != difop2_len)) {
        std::cout << "readBinData failed" << difop2_len << std::endl;

        return -1;
    }
    ret = parseDifopPkt((const uint8_t*)difop2_data, difop2_len);

    if (!ret) {
        std::cout << "parseDifop2 failed" << std::endl;
        return -1;
    }
    size_t msop_size = sizeof(LidarPointCloudPackets) + (kBlockPerFrame - 1) * sizeof(DataBlock);
    char* msop_data = static_cast<char*>(malloc(msop_size));
    uint32_t msop_len {0U};
    ret = readBinData("./msop.bin", msop_data, msop_len);

    if(!ret || msop_len != msop_size) {
        std::cout << "readBinData failed " << msop_len << ", " << msop_size << std::endl;
        return -1;
    }
    ret = parseMsopPkt((const uint8_t*)msop_data, msop_len);

    if (!ret) {
        std::cout << "parseMsopPkt failed" << std::endl;
        return -1;
    }
    free(difop2_data);
    free(msop_data);

    size_t requiredSize = sizeof(LidarPointCloud) + (kPointsPerFrame - 1) * sizeof(LidarPoint);
    uint8_t* point_cloud_data = static_cast<uint8_t*>(malloc(requiredSize));
    size_t point_cloud_size {0};

    getPointCloud(point_cloud_data, point_cloud_size);
    savePointCloudToPCD("./point_cloud.pcd", (LidarPointCloud*)point_cloud_data, PCD_BINARY);

    return 0;
}
#endif

/* \}  custom_demo */
