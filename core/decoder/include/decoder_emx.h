/*******************************************************************************
 * \addtogroup decoder
 * \{
 * \headerfile decoder_emx.h
 * \brief
 * \version 0.1
 * \date 2025-06-20
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-06-20 | Init version |
 *
 ******************************************************************************/
#ifndef I_ROBOSENSE_DECODER_EMX_H
#define I_ROBOSENSE_DECODER_EMX_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <general_decoder.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/
#define EMX_SURFACE_NUM 2U
#define EMX_PIXELS_PER_COLUMN 192U
#define EMX_COLUMNS_PER_FRAME 1520U
#define MIDDLE_PACKET_SEQ 760U
#define EMX_VECSELS_PER_COLUMN 24U
#define EMX_PIXELS_PER_VCSEL 8U
#define EMX_GMSL_WIDTH 1920U
#define EMX_GMSL_HEIGHT 562U

#define EMX_MIPI_DATA_LEN EMX_GMSL_WIDTH* EMX_GMSL_HEIGHT
#define EMX_PKT_SEQ_MIN 1U
#define EMX_PKT_SEQ_MAX 1520U

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense::lidar
{
/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
inline uint64_t ntohll(uint64_t x)
{
  // 使用移位操作手动交换字节
  uint32_t high = static_cast<uint32_t>(x >> 32);
  uint32_t low = static_cast<uint32_t>(x & 0xFFFFFFFF);

  // 分别交换高32位和低32位，然后合并
  return (static_cast<uint64_t>(ntohl(low)) << 32) | static_cast<uint64_t>(ntohl(high));
}

#pragma pack(push, 1)

typedef struct
{
  uint8_t id[4];
  uint8_t reserved_0[66];
  int8_t yaw_offset[24];
  int16_t pitch_angle[192];
  int16_t surface_pitch_offset[2];
  uint8_t reserved_1[6];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEMXDifop2Pkt;

typedef struct
{
  uint8_t reserved[16];
  uint16_t sw_version;
  uint8_t reserved_1[3];
} RSEMXDeviceVersion;

typedef struct
{
  uint8_t time_sync_mode;
  uint8_t time_sync_status;
  RSTimeStamp timestamp;
  uint64_t time_offset;
} RSEMXTimeInfo;

typedef struct
{
  uint8_t work_mode;
  uint8_t wave_mode;
  uint8_t calib_mode;
  uint8_t win_block_status;
  uint8_t win_block_level[18];
  uint8_t frame_sync_time_out;
  uint8_t reserved_0[3];
} RSEMXWorkInfo;

typedef struct
{
  uint8_t id[4];
  RSEMXDeviceVersion device_version;
  uint8_t reserved[6];
  uint8_t customer_sn[25];
  RSEMXWorkInfo work_info;
  RSEMXTimeInfo time_info;
  uint8_t reserved_1[77];
  uint8_t fault_level;
  uint16_t fault_id;
  uint8_t fault_value[4];
  uint32_t dtc;
  uint8_t reserved_2[36];
} RSEMXDeviceInfoPkt;

typedef struct
{
  uint16_t radius;
  uint8_t intensity;
  uint8_t attribute;
} RSEMXMsopWave;  // 4-bytes

typedef struct
{
  uint16_t peak; /* 0 - 1023 */
  uint16_t width;
} RSEMXMsopRaw;  // 4-bytes

typedef struct
{
  uint8_t magic[4];
  uint16_t pkt_seq;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimeStamp timestamp;
  uint8_t synchronized;
  uint8_t frame_rate;
  uint16_t column;
  int16_t yaw;
  uint8_t surface_id;
  uint32_t reserved;
  uint32_t frame_cnt;
  uint8_t lidar_type;
} RSEMXMsopHeader;  // 32-bytes

typedef struct
{
  RSEMXMsopWave waves[2]; // 8-bytes
  RSEMXMsopRaw raws[2];   // 8-bytes
} RSEMXMsopPixel;  // 16-bytes

typedef struct
{
  RSEMXMsopHeader header;       // 32-bytes
  RSEMXMsopPixel pixels[192];   // 3072-bytes
  uint32_t reserved[3];         // 12-bytes
} RSEMXMsopPkt;  // 3116-bytes

#pragma pack(pop)

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
class DecoderRSEMX : public Decoder
{
public:
  explicit DecoderRSEMX(const RSDecoderParam& param);
  virtual ~DecoderRSEMX(){};

  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) override;

  virtual bool decodeDifopPkt(const uint8_t* pkt, size_t size) override;
  virtual bool decodeDeviceInfoPkt(const uint8_t* pkt, size_t size) override;

private:
  void setPointCloudInfo(const RSEMXMsopPkt& pkt);
  std::array<int32_t, EMX_PIXELS_PER_COLUMN> yaw_offset_;
  std::array<int32_t, EMX_PIXELS_PER_COLUMN> pitch_angle_;
  std::array<int32_t, EMX_SURFACE_NUM> surface_pitch_offset_;
  std::vector<uint64_t> cached_pkt_timestamps_;

  static RSDecoderConstParam& getConstParam()
  {
    static RSDecoderConstParam param = {
      .msop_len = 3116,                              // msop len
      .difop_len = 500,                              // difop len
      .device_info_len = 224,                        // device info len
      .msop_id_len = 4,                              // msop id len
      .difop_id_len = 4,                             // difop id len
      .device_info_id_len = 4,                       // device info id len
      .msop_id = { 0x55, 0xAA, 0x5A, 0xA5 },         // msop id
      .difop_id{ 0xA5, 0xFF, 0x00, 0xAE },           // difop id
      .device_info_id = { 0xA5, 0xFF, 0x00, 0x5A },  // device info id
      .pixel_per_pkt = 192,                          // pixels per packet
      .distance_min = 0.2f,                          // distance min
      .distance_max = 300.0f,                        // distance max
      .distance_res = 0.005f,                        // distance resolution
    };

    return param;
  }
};

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/

}  // namespace robosense::lidar

/** \} decoder */
#endif /* I_ROBOSENSE_DECODER_EMX_H */
