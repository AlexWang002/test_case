/*******************************************************************************
 * \addtogroup api
 * \{
 * \headerfile RS_LIDAR_SDK_API "RS_LIDAR_SDK_API"
 * \brief
 * \version 1.1
 * \date 2025-08-04
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-07 | Init version |
 * | 1.1 | 2025-08-04 | Refactor file format |
| Init version |
 *
 ******************************************************************************/
#ifndef I_RS_LIDAR_SDK_API_H
#define I_RS_LIDAR_SDK_API_H

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "lidar_sdk_api.h"

#   ifndef EXPORT
#       define EXPORT __attribute__((visibility("default")))
#   endif

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar_sdk {

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
EXPORT LidarSdkInterface* getRSLidarSdkInterface(size_t sizeOfInterface);

} // namespace robosense::lidar_sdk

/******************************************************************************/
/*             Declaration of exported C type function prototypes             */
/******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

EXPORT LidarSdkInterface* getRSLidarSdkInterface(size_t sizeOfInterface);

#ifdef __cplusplus
}
#endif

/** \} api */
#endif /* I_RS_LIDAR_SDK_API_H */