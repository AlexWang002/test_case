/*******************************************************************************
 * \addtogroup utils
 * \{
 * \headerfile DIFOP2 "DIFOP2"
 * \brief
 * \version 0.1
 * \date 2025-07-24
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-24 | Init version |
 *
 ******************************************************************************/
#ifndef I_DIFOP2_H
#define I_DIFOP2_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cstdint>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

namespace robosense::lidar::utils {

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/
const uint16_t MSOP_LEN {3116};
const uint16_t DIFOP_LEN {500};
const uint16_t DEVICE_INFO_LEN {224};

// packet identity
const uint8_t MSOP_ID_LEN {4};
const uint8_t DIFOP_ID_LEN {4};
const uint8_t DEVICE_INFO_ID_LEN {4};
const uint8_t MSOP_ID[4] {0x55, 0xAA, 0x5A, 0xA5};
const uint8_t DIFOP_ID[4] {0xA5, 0xFF, 0x00, 0xAE};
const uint8_t DEVICE_INFO_ID[4] {0xA5, 0xFF, 0x00, 0x5A};

const uint16_t PIXEL_PER_PKT {192};

// distance
const float DISTANCE_MIN {0.2f};
const float DISTANCE_MAX {300.0f};
const float DISTANCE_RES {0.005f};

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern std::array<uint8_t, DIFOP_LEN> difop2_mipi_data;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern void compareDifop2();

} // namespace robosense::lidar::utils

/** \} utils */
#endif /* I_DIFOP2_H */