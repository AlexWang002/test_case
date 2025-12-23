/*******************************************************************************
 * \addtogroup decoder
 * \{
 * \headerfile decoder_emx "decoder_emx.h"
 * \brief Declares the decoder class for the RoboSense EMX series LiDAR data.
 * \version 0.2
 * \date 2025-08-04
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-06-20 | Init version |
 * | 0.2 | 2025-08-04 | Add comments |
 *
 ******************************************************************************/
#ifndef I_DECODER_EMX_H
#define I_DECODER_EMX_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "general_decoder.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/
#define EMX_SURFACE_NUM        (2U)
#define EMX_PIXELS_PER_COLUMN  (192U)
#define EMX_COLUMNS_PER_FRAME  (1520U)
#define MIDDLE_PACKET_SEQ      (760U)
#define EMX_VECSELS_PER_COLUMN (24U)
#define EMX_PIXELS_PER_VCSEL   (8U)
#define EMX_GMSL_WIDTH         (1920U)
#define EMX_GMSL_HEIGHT        (562U)
#define EMX_PKT_SEQ_MIN        (1U)
#define EMX_PKT_SEQ_MAX        (1520U)
#define EMX_MIPI_PART_LEN      (EMX_GMSL_WIDTH * EMX_GMSL_HEIGHT)

#ifdef MIPI_10HZ
#   define EMX_MIPI_PART_NUM    (6)
#elif defined(MIPI_30HZ)
#   define EMX_MIPI_PART_NUM    (2)
#else   // default 60Hz
#   define EMX_MIPI_PART_NUM    (1)
#endif

// #define EMX_MIPI_DATA_LEN       (EMX_MIPI_PART_LEN * EMX_MIPI_PART_NUM)
// #define EMX_EMBED_MIPI_DATA_LEN (EMX_MIPI_DATA_LEN + EMX_GMSL_WIDTH)
#define EMX_MIPI_PACKET_NUM     (3040)
#define EMX_MIPI_DATA_LEN       (EMX_GMSL_WIDTH * EMX_MIPI_PACKET_NUM)
#define EMX_EMBED_MIPI_DATA_LEN (EMX_MIPI_DATA_LEN + EMX_GMSL_WIDTH)

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
#pragma pack(push, 1)

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
} RSEMXDifop2Pkt;                    // 500 bytes

typedef struct {
    uint8_t reserved[16];  // 16 bytes
    uint16_t sw_version;   //  2 bytes
    uint8_t reserved_1[3]; //  3 bytes
} RSEMXDeviceVersion;      // 21 bytes

typedef struct {
    uint8_t time_sync_mode;   //  1 byte
    uint8_t time_sync_status; //  1 byte
    RSTimeStamp timestamp;    //  8 bytes
    uint64_t time_offset;     //  8 bytes
} RSEMXTimeInfo;              // 18 bytes

typedef struct {
    uint8_t work_mode;           //  1 byte
    uint8_t wave_mode;           //  1 byte
    uint8_t calib_mode;          //  1 byte
    uint8_t win_block_status;    //  1 byte
    uint8_t win_block_level[18]; // 18 bytes
    uint8_t frame_sync_time_out; //  1 byte
    uint8_t reserved_0[3];       //  3 bytes
} RSEMXWorkInfo;                 // 24 bytes

typedef struct {
    uint8_t id[4];                     //   4 bytes    byte 0   / size 4
    RSEMXDeviceVersion device_version; //  21 bytes    byte 4   / size 21
    uint8_t reserved[6];               //   6 bytes    byte 25  / size 6
    uint8_t customer_sn[25];           //  25 bytes    byte 31  / size 25
    RSEMXWorkInfo work_info;           //  26 bytes    byte 56  / size 26
    RSEMXTimeInfo time_info;           //  18 bytes    byte 82  / size 18
    uint8_t reserved_1[77];            //  77 bytes    byte 100 / size 77
    uint8_t fault_level;               //   1 byte     byte 177 / size 1
    uint16_t fault_id1;                //   2 bytes    byte 178 / size 2
    uint8_t fault_value1[4];           //   4 bytes    byte 180 / size 4
    uint16_t fault_id2;                //   2 bytes    byte 178 / size 2
    uint8_t fault_value2[4];           //   4 bytes    byte 180 / size 4
    uint32_t dtc;                      //   4 bytes    byte 190 / size 4
    uint8_t reserved_2[30];            //  36 bytes    byte 194 / size 30
} RSEMXDeviceInfoPkt;                  // 500 bytes    size 224

typedef struct {
    uint16_t radius;   // 2 bytes
    uint8_t intensity; // 1 byte
    uint8_t attribute; // 1 byte
} RSEMXMsopWave;       // 4 bytes

typedef struct {
    uint16_t peak;  // 2 bytes  range: 0 - 1023
    uint16_t width; // 2 bytes
} RSEMXMsopRaw;     // 4 bytes

typedef struct {
    uint8_t magic[4];      //  4 bytes
    uint16_t pkt_seq;      //  2 bytes
    uint8_t return_mode;   //  1 byte
    uint8_t time_mode;     //  1 byte
    RSTimeStamp timestamp; //  8 bytes
    uint8_t synchronized;  //  1 byte
    uint8_t frame_rate;    //  1 byte
    uint16_t column;       //  2 bytes
    int16_t yaw;           //  2 bytes
    uint8_t surface_id;    //  1 byte
    uint16_t pitch_offset; //  2 bytes
    uint16_t reserved;     //  2 bytes
    uint32_t frame_cnt;    //  4 bytes
    uint8_t lidar_type;    //  1 byte
} RSEMXMsopHeader;         // 32 bytes

typedef struct {
    RSEMXMsopWave waves[2]; //  8 bytes
    RSEMXMsopRaw raws[2];   //  8 bytes
} RSEMXMsopPixel;           // 16 bytes

typedef struct {
    RSEMXMsopHeader header;     //   32 bytes
    RSEMXMsopPixel pixels[192]; // 3072 bytes
    uint32_t reserved[3];       //   12 bytes
} RSEMXMsopPkt;                 // 3116 bytes

#pragma pack(pop)

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
class DecoderRSEMX : public Decoder {
  public:
    explicit DecoderRSEMX(const RSDecoderParam& param);
    virtual ~DecoderRSEMX() {};

    virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) override;
    virtual bool decodeDifopPkt(const uint8_t* pkt, size_t size) override;
    virtual bool decodeDeviceInfoPkt(const uint8_t* pkt, size_t size) override;
    // bool decodeSdkMsopPkt(const uint8_t* packet, size_t size);

  private:
    void setPacketHeader(const RSEMXMsopPkt& pkt);
    void setPointCloudInfo(const RSEMXMsopPkt& pkt);
    std::array<int32_t, EMX_PIXELS_PER_COLUMN> yaw_offset_;
    std::array<int32_t, EMX_PIXELS_PER_COLUMN> pitch_angle_;
    std::array<int32_t, EMX_SURFACE_NUM> surface_pitch_offset_;
    std::vector<uint64_t> cached_pkt_timestamps_;
    std::uint8_t difop2_[500];
    bool difop2_received_{false};
    std::chrono::steady_clock::time_point obstruct_start_time_;
    bool obstruct_detected_{false};

    static RSDecoderConstParam& getConstParam() {
        static RSDecoderConstParam param = {
            .msop_len = 3116,                             // msop len
            .difop_len = 500,                             // difop len
            .device_info_len = 224,                       // device info len
            .msop_id_len = 4,                             // msop id len
            .difop_id_len = 4,                            // difop id len
            .device_info_id_len = 4,                      // device info id len
            .msop_id = { 0x55, 0xAA, 0x5A, 0xA5 },        // msop id
            .difop_id { 0xA5, 0xFF, 0x00, 0xAE },         // difop id
            .device_info_id = { 0xA5, 0xFF, 0x00, 0x5A }, // device info id
            .pixel_per_pkt = 192,                         // pixels per packet
            .distance_min = 0.2f,                         // distance min
            .distance_max = 300.0f,                       // distance max
            .distance_res = 0.005f,                       // distance resolution
        };

        return param;
    }
};

} // namespace robosense::lidar

/** \} decoder */
#endif /* I_DECODER_EMX_H */