/*******************************************************************************
 * \addtogroup api
 * \{
 * \file rs_lidar_sdk_api.cpp
 * \brief lidar SDK对外接口实现文件
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
 * | 1.0 | 2025-07-07 | Refactor file format, fix static warning |
 * | 1.1 | 2025-08-04 | Refactor file format |
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include "core/interface/rs_lidar_sdk_interface.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "rs_lidar_sdk_api.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace robosense::lidar;

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar_sdk {

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
EXPORT LidarSdkInterface* getRSLidarSdkInterface(size_t sizeOfInterface) {
    if (sizeOfInterface != sizeof(LidarSdkInterface)) {
        return nullptr;
    }

    static LidarSdkInterface interface = {
        .apiVersion = LIDAR_SDK_API_VER,
        .getLidarSdkVersion = [](void) -> const char* {
            return RSLidarSdkInterface::getInstance().getLidarSdkVersion();
        },
        .init = [](const LidarSdkCbks* fptrCbks,
                    const char* configPath) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().init(fptrCbks, configPath);
        },
        .deInit = [](void) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().deInit();
        },
        .start = [](void) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().start();
        },
        .stop = [](void) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().stop();
        },
        .injectAdc = [](LidarSensorIndex sensor,
                        const void* ptrAdc) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().injectAdc(sensor, ptrAdc);
        },
        .writeDid = [](uint16_t did, uint8_t* data, uint16_t dataLen,
                        uint8_t* nrc) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().writeDid(did, data, dataLen, nrc);
        },
        .readDid = [](uint16_t did, uint8_t* data,
                        uint16_t* dataLen, uint8_t* nrc) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().readDid(did, data, dataLen, nrc);
        },
        .ridControl = [](uint8_t mode, uint16_t rid, uint8_t* dataIn,
                        uint16_t dataInLen, uint8_t* dataOut,
                        uint16_t* dataOutLen, uint8_t* nrc) -> LidarSdkErrorCode {
            return RSLidarSdkInterface::getInstance().ridControl(
                        mode, rid, dataIn, dataInLen, dataOut, dataOutLen, nrc);
        }
    };

    return &interface;
}

} // namespace robosense::lidar_sdk

/******************************************************************************/
/*              Definition of exported C type function prototypes             */
/******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

EXPORT LidarSdkInterface* getRSLidarSdkInterface(size_t sizeOfInterface) {
    return robosense::lidar_sdk::getRSLidarSdkInterface(sizeOfInterface);
}

#ifdef __cplusplus
}
#endif
/* \}  api */