/*******************************************************************************
 * \addtogroup interface
 * \{
 * \file rs_lidar_sdk_interface.cpp
 * \brief Defines the interface of the LiDAR SDK.
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

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cstring>
#include <fstream>
#include <iostream>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "rs_lidar_sdk_interface.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*          Definition of public functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Returns the single instance of the LiDAR SDK interface.
 * \return The single instance of the LiDAR SDK interface.
 */
RSLidarSdkInterface& RSLidarSdkInterface::getInstance() {
    static RSLidarSdkInterface instance;
    return instance;
}

/**
 * \brief Returns the version of the LiDAR SDK.
 * \return The version of the LiDAR SDK.
 */
const char* RSLidarSdkInterface::getLidarSdkVersion() {
    if (impl_) {
        return impl_->getLidarSdkVersion(); // 与头文件中的版本一致
    } else {
        return "";
    }
}

/**
 * \brief Initializes the LiDAR SDK.
 * \param[in] fptrCbks The callback functions.
 * \param[in] configPath The path of the configuration file.
 * \retval LIDAR_SDK_SUCCESS: The initialization is successful.
 * \retval LIDAR_SDK_FAILD: The initialization fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::init(const LidarSdkCbks* fptrCbks, const char* configPath) {
    if (impl_) {
        return impl_->init(fptrCbks, configPath);
    }
    return LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/**
 * \brief Deinitializes the LiDAR SDK.
 * \retval LIDAR_SDK_SUCCESS: The deinitialization is successful.
 * \retval LIDAR_SDK_FAILD: The deinitialization fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::deInit() {
    if (impl_) {
        return impl_->deInit();
    }
    return LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/**
 * \brief Starts the LiDAR SDK.
 * \retval LIDAR_SDK_SUCCESS: The start is successful.
 * \retval LIDAR_SDK_FAILD: The start fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::start() {
    if (impl_) {
        return impl_->start();
    }
    return LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/**
 * \brief Stops the LiDAR SDK.
 * \retval LIDAR_SDK_SUCCESS: The stop is successful.
 * \retval LIDAR_SDK_FAILD: The stop fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::stop() {
    if (impl_) {
        return impl_->stop();
    }
    return LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/**
 * \brief Powers off the LiDAR SDK.
 * \retval LIDAR_SDK_SUCCESS: The power off is successful.
 * \retval LIDAR_SDK_FAILD: The power off fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::powerOff() {
    if (impl_) {
        return impl_->powerOff();
    }
    return LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/**
 * \brief Sets the calibration enable status of the LiDAR SDK.
 * \param[in] enabled The calibration enable status.
 * \retval LIDAR_SDK_SUCCESS: The set is successful.
 * \retval LIDAR_SDK_FAILD: The set fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::setCalibrationEnable(const bool enabled) {
    if (impl_) {
        return impl_->setCalibrationEnable(enabled);
    }
    return LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/**
 * \brief Injects ADC data into the LiDAR SDK.
 * \param[in] sensor The sensor index.
 * \param[in] ptrAdc The pointer to the ADC data.
 * \retval LIDAR_SDK_SUCCESS: The injection is successful.
 * \retval LIDAR_SDK_FAILD: The injection fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::injectAdc(LidarSensorIndex sensor, const void* ptrAdc) {
    if (!impl_) {
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    return impl_->injectAdc(sensor, ptrAdc);
}

/**
 * \brief Inject Alarm data to the Lidar SDK.
 * \param[in] lidarAlarmInfo The Alarm info.
 * \retval LIDAR_SDK_SUCCESS: The injection is successful.
 * \retval LIDAR_SDK_FAILD: The injection fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::injectAlarmInfo(const LidarAlarmInfo& lidarAlarmInfo) {
    if (!impl_) {
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    return impl_->injectAlarmInfo(lidarAlarmInfo);
}

/**
 * \brief Writes data to a DID.
 * \param[in] did The DID.
 * \param[in] data The pointer to the data.
 * \param[in] dataLen The length of the data.
 * \param[out] nrc The pointer to the NRC.
 * \retval LIDAR_SDK_SUCCESS: The write is successful.
 * \retval LIDAR_SDK_FAILD: The write fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::writeDid(uint16_t did, uint8_t* data, uint16_t dataLen, uint8_t* nrc) {
    if (!impl_) {
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    return impl_->writeDid(did, data, dataLen, nrc);
}

/**
 * \brief Reads data from a DID.
 * \param[in] did The DID.
 * \param[out] data The pointer to the data.
 * \param[in] dataLen The length of the data.
 * \param[out] nrc The pointer to the NRC.
 * \retval LIDAR_SDK_SUCCESS: The read is successful.
 * \retval LIDAR_SDK_FAILD: The read fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::readDid(uint16_t did, uint8_t* data, uint16_t* dataLen, uint8_t* nrc) {
    if (!impl_) {
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    return impl_->readDid(did, data, dataLen, nrc);
}

/**
 * \brief Controls the LiDAR SDK using RID.
 * \param[in] mode The mode.
 * \param[in] rid The RID.
 * \param[in] dataIn The pointer to the input data.
 * \param[in] dataInLen The length of the input data.
 * \param[out] dataOut The pointer to the output data.
 * \param[out] dataOutLen The length of the output data.
 * \param[out] nrc The pointer to the NRC.
 * \retval LIDAR_SDK_SUCCESS: The control is successful.
 * \retval LIDAR_SDK_FAILD: The control fails.
 */
LidarSdkErrorCode RSLidarSdkInterface::ridControl(uint8_t mode, uint16_t rid, uint8_t* dataIn, uint16_t dataInLen,
                                                  uint8_t* dataOut, uint16_t* dataOutLen, uint8_t* nrc) {
    if (!impl_) {
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    return impl_->ridControl(mode, rid, dataIn, dataInLen, dataOut, dataOutLen, nrc);
}

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Constructs a RSLidarSdkInterface object.
 */
RSLidarSdkInterface::RSLidarSdkInterface() {
    // 初始化代码
    impl_.reset(new RSLidarSdkImpl());
}

/**
 * \brief Destructs a RSLidarSdkInterface object.
 */
RSLidarSdkInterface::~RSLidarSdkInterface() {
    (void)deInit();
}

} // namespace robosense::lidar

/* \}  interface */
