/*******************************************************************************
 * \addtogroup utils
 * \{
 * \headerfile difop2.h "difop2.h"
 * \brief Declares the difop2 data structure.
 * \version 0.2
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-24 | Init version |
 * | 0.2 | 2025-08-07 | Add comments |
 *
 ******************************************************************************/
#ifndef I_DIFOP2_H
#define I_DIFOP2_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <array>
#include <cstdint>

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar::utils {

/******************************************************************************/
/*                   Definition of exported constant data                     */
/******************************************************************************/
static constexpr uint16_t kMsopLen{3116U};
static constexpr uint16_t kDifopLen{500U};
extern const std::string kBinPath;
extern const std::string kJsonPath;

// packet identity
static const std::array<uint8_t, 4> kDifop2Id{0xA5U, 0xFFU, 0x00U, 0xAEU};

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
/**
 * \struct Difop2
 * \brief This struct defines the difop2 data.
 */
struct Difop2 {
    std::array<uint8_t, 4> info_header;             // 0~4       4 Bytes
    std::array<uint8_t, 63> reserve_data_0;         // 4~67     63 Bytes
    uint8_t surface_count;                          // 67~68     1 Byte
    uint8_t vcsel_pixel_count;                      // 68~69     1 Byte
    uint8_t vcsel_count;                            // 69~70     1 Byte
    std::array<uint8_t, 24> vcsel_yaw_offset;       // 70~94    24 Bytes
    std::array<uint8_t, 384> pixel_pitch;           // 94~478  384 Bytes
    std::array<uint16_t, 2> surface_pitch_offset;   // 478~482   4 Bytes
    uint16_t roll_offset;                           // 482~484   2 Bytes
    std::array<uint8_t, 4> reserve_data_1;          // 484~488   4 Bytes
    uint16_t data_length;                           // 488~490   2 Bytes
    uint16_t counter;                               // 490~492   2 Bytes
    std::array<uint8_t, 4> data_id;                 // 492~496   4 Bytes

    bool data_valid{false};
};

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern std::array<uint8_t, kDifopLen> difop2_mipi_data;
extern std::array<char, kDifopLen> difop2_bin_data;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern void compareDifop2();
extern bool readBinData(const std::string& file_path);

} // namespace robosense::lidar::utils

/** \} utils */
#endif /* I_DIFOP2_H */
