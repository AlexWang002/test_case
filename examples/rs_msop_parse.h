/*******************************************************************************
 * \addtogroup custom_demo
 * \{
 * \headerfile rs_msop_parse.h "rs_msop_parse.h"
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
#ifndef I_RS_MSOP_PARSE_H
#define I_RS_MSOP_PARSE_H

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "lidar_sdk_api.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::msop {

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
typedef struct {
    float x;                // 笛卡尔坐标系，X轴方向的距离值
    float y;                // 笛卡尔坐标系，Y轴方向的距离值
    float z;                // 笛卡尔坐标系，Z轴方向的距离值
    int16_t timestamp;      // 每个点的时间戳(精确到10微秒)
    uint8_t channel_number; // channel(激光束)的序号
    uint8_t intensity;      // 通道点反射强度值
    uint8_t confidence[2];  // 雨雾扬尘噪点置信度
} LidarPoint;

typedef struct {
    uint64_t frame_timestamp;  // 完整的时间戳(精确到微秒)
    uint32_t point_num;        // 一帧里点的个数
    uint32_t frame_seq;        // 数据帧序列号
    uint16_t protocol_version; // 通信协议版本号，供应商自己定义
    uint8_t  return_mode;      // 回波模式设置:0-双回波，1-最强回波，2-最后回波，3-第一回波
    uint8_t sync_status;       // 时间同步状态:0:已经同步 1:未同步
    uint8_t frame_sync;        // 帧同步，0表示未同步，1表示已同步
    uint8_t mirror_id;         // 奇偶帧标识（适用摆镜方案）
    uint8_t reserved[20];      // reserved
    LidarPoint point[1];       // Lidar点云数据
} LidarPointCloud;

typedef struct {
    uint8_t id[4];                   //   4 bytes
    uint8_t reserved_0[66];          //  66 bytes
    int8_t yaw_offset[24];           //  24 bytes
    int16_t pitch_angle[192];        // 384 bytes
    int16_t surface_pitch_offset[2]; //   4 bytes
    uint8_t reserved_1[6];           //   6 bytes
    uint16_t data_length;            //   2 bytes
    uint16_t counter;                //   2 bytes
    uint32_t data_id;                //   4 bytes
    uint32_t crc32;                  //   4 bytes
} LidarDifop2Pkt;                    // 500 bytes

typedef enum {
    PCD_ASCII,  // ASCII格式
    PCD_BINARY, // 二进制格式
} PCDFormat;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
bool initMsop();
bool parseDifopPkt(const uint8_t* packet, size_t size);
void getPointCloud(void* point_cloud, uint32_t size);
bool parseMsopPkt(const uint8_t* packet, size_t size);
bool savePointCloudToPCD(const std::string& kFileName,
                         LidarPointCloud* cloud,
                         PCDFormat format);

} // namespace robosense::msop

/** \} custom_demo */
#endif /* I_RS_MSOP_PARSE_H */
