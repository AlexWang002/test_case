/*******************************************************************************
 * \addtogroup interface
 * \{
 * \headerfile rs_lidar_sdk_interface.h "rs_lidar_sdk_interface.h"
 * \brief Declares the interface of the LiDAR SDK.
 * \version 0.2
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-07 | Init version |
 * | 0.2 | 2025-08-07 | Add Comments |
 * | 1.0 | 2025-10-14 | Add function injectAlarmInfo and powerOff and setCalibrationEnable |
 *
 ******************************************************************************/
#ifndef I_RS_LIDAR_SDK_INTERFACE_H
#define I_RS_LIDAR_SDK_INTERFACE_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <memory>
#include <mutex>
#include <string>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "core/impl/rs_lidar_sdk_impl.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
/**
 * \class rs_lidar_sdk_interface.h "rs_lidar_sdk_interface.h"
 * \brief A interface class for LiDAR SDK.
 */
class RSLidarSdkInterface {
  public:
    static RSLidarSdkInterface& getInstance();

    RSLidarSdkInterface(const RSLidarSdkInterface&) = delete;
    RSLidarSdkInterface& operator=(const RSLidarSdkInterface&) = delete;

    const char* getLidarSdkVersion();
    LidarSdkErrorCode init(const LidarSdkCbks* fptrCbks, const char* configPath);
    LidarSdkErrorCode deInit();
    LidarSdkErrorCode start();
    LidarSdkErrorCode stop();
    LidarSdkErrorCode powerOff(void);
    LidarSdkErrorCode setCalibrationEnable(const bool enabled);
    LidarSdkErrorCode injectAdc(LidarSensorIndex sensor, const void* ptrAdc);
    LidarSdkErrorCode injectAlarmInfo(const LidarAlarmInfo& lidarAlarmInfo);
    LidarSdkErrorCode writeDid(uint16_t did, uint8_t* data, uint16_t dataLen, uint8_t* nrc);
    LidarSdkErrorCode readDid(uint16_t did, uint8_t* data, uint16_t* dataLen, uint8_t* nrc);
    LidarSdkErrorCode ridControl(uint8_t mode, uint16_t rid, uint8_t* dataIn, uint16_t dataInLen, uint8_t* dataOut,
                                 uint16_t* dataOutLen, uint8_t* nrc);

  private:
    RSLidarSdkInterface();
    ~RSLidarSdkInterface();
    std::unique_ptr<RSLidarSdkImpl> impl_;
};

} // namespace robosense::lidar

/** \} interface */
#endif /* I_RS_LIDAR_SDK_INTERFACE_H */
