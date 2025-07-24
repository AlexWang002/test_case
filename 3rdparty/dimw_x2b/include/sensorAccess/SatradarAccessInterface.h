/**
 * @file SatradarAccessInterface.h
 * @brief
 * @details
 * @author zhang.peng103 (zhang.peng103@byd.com)
 * @version 1.0.0
 * @date 2022/11/21 16:49:29
 * @copyright Copyright (c) 2022 BYD Company Limited
 */

#ifndef CAMERAACCESSINTERFACE_H
#define CAMERAACCESSINTERFACE_H

#include <vector>
#include <tuple>
#include <map>
#include "SatradarAccessCommon.h"

namespace dimw
{
namespace satradarAccess
{
/**
 * @file SatradarAccessInterface.h
 * @brief CameraAccess提供给使用者的接口文件
 * @details CameraAccess提供给使用者的接口文件
 * @author zhang.peng103@byd.com
 * @version 1.0.0
 * @date 2022/04/24
 * @copyright Copyright (c) 2021 比亚迪股份有限公司
 */
class CameraAccessInterface 
{
public:
    /**
     * @brief 获取CameraAccessInterface对象
     * @details 获取CameraAccessInterface对象
     * @param[in]  无
     * @param[out] 无
     * @return getInstance的返回值
     *       @retval CameraAccessInterface & getInstance执行成功
     *       @retval NULL getInstance执行失败
     */
    static CameraAccessInterface &getInstance(void) ;

    /**
     * @brief 使能指定摄像头
     * @details 使能指定摄像头
     * @param[in]  cameraNo 指定摄像头ID
     * @param[out] 无
     * @return EnableCamera的返回值
     *       @retval 使能成功返回0，入参非法返回1，使能失败返回0xff
     */
    int32_t enableCamera(const uint8_t &cameraNo) __attribute__((deprecated(
        "enableCamera is deprecated. Use camera_sensor_enable instead.")));

    /**
     * @brief 去使能指定摄像头
     * @details 去使能指定摄像头
     * @param[in]  cameraNo 指定摄像头ID
     * @param[out] 无
     * @return disableCamera的返回值
     *       @retval 去使能成功返回0，入参非法返回1，去使能失败返回0xff
     */
    int32_t disableCamera(const uint8_t &cameraNo) __attribute__((deprecated(
        "disableCamera is deprecated. Use camera_sensor_disable instead.")));

    /**
     * @brief 获取某个摄像头模组信息
     * @details 获取某个摄像头模组信息
     * @param[in]  sensorId 指定摄像头ID
     * @param[out] moduleInfo 摄像头模组信息
     * @return getModuleInfo的返回值
     *       @retval 0执行成功  1执行失败
     */
    int32_t getModuleInfo(uint8_t sensorId, ModuleInfo &moduleInfo) __attribute__((deprecated(
        "getModuleInfo is deprecated. Use camera_get_sensor_module_info instead.")));

    /**
     * @brief 检测摄像头存在状态
     *
     * @param sensorId 指定摄像头ID
     * @return true 摄像头存在
     *      @retval 成功返回摄像头存在状态掩码, 比如0x70，表示摄像头编号4/5/6 存在。
     *      @retval 失败返回0xefff
     */
    uint16_t detectAllSensors(void) __attribute__((deprecated(
        "detectAllSensors is deprecated. Use camera_detect_all_sensors instead.")));

    /**
     * @brief 检测link lock 和video link
     *
     * @param 
     * @return true 执行成功
     */

    uint16_t detectLinkAndVideoMask(uint16_t &linkMask, uint16_t &videoMask) __attribute__((deprecated(
        "detectLinkAndVideoMask is deprecated. Use satradar_detect_link_video_mask instead.")));

    /**
     * @brief 获取当前soc上接入服务首次启动ptp时间
     * @details 获取当前soc上接入服务首次启动ptp时间
     * @param[in] 无
     * @return 获取当前soc上接入服务首次启动ptp时间
     */
    uint64_t get_fsync_first_time() __attribute__((deprecated(
        "get_Fsync_first_time is deprecated. Use camera_get_fsync_first_time instead."))); 

    /**
     * @brief 获取内参sn生成状态
     * @details 判断接入服务是否已经生成内参和SN文件
     * @param[in] 无
     * @return 0:已准备好，1：没准备好, 2:未知状态
     */
    uint32_t getSnState() __attribute__((deprecated(
        "getSnState is deprecated. Use get_sn_state instead."))); 

    bool setSyncRestart() __attribute__((deprecated(
        "setSyncRestart is deprecated. Use set_sync_restart instead."))); 

    /**
     * @brief 通过i2c接口读取雷达模组寄存器信息，读取1个字节
     * @param[in] sensorId 指定要获取雷达通道
                  address 要读取的寄存器地址
     * @param[out] data 读取到寄存器数据
     * @return  0：success ;<0:失败，
     */
    uint16_t readSensorRegUint8(uint32_t sensorId, uint16_t address, uint8_t &data);
    
    /**
     * @brief 通过i2c接口写雷达模组寄存器，写入1个字节
     * @param[in] sensorId 指定要获取雷达通道
                  address 要读取的寄存器地址
                  data 要写入的寄存器数据
     * @return  0：success ;<0:失败，
     */
    uint16_t writeSensorRegUint8(uint32_t sensorId, uint16_t address, uint8_t data);
    
    /**
     * @brief 通过i2c接口读取雷达模组寄存器信息，读取2个字节
     * @param[in] sensorId 指定要获取雷达通道
                  address 要读取的寄存器地址
     * @param[out] data 读取到寄存器数据
     * @return  0：success ;<0:失败，
     */
    uint16_t readSensorRegUint16(uint32_t sensorId, uint16_t address, uint16_t &data);
    
    /**
     * @brief 通过i2c接口写雷达模组寄存器，写入2个字节
     * @param[in] sensorId 指定要获取雷达通道
                  address 要读取的寄存器地址
                  data 要写入的寄存器数据
     * @return  0：success ;<0:失败，
     */
    uint16_t writeSensorRegUint16(uint32_t sensorId, uint16_t address, uint16_t data);

    /**
     * @brief  对外提供雷达传感器读取接口
     * @details
     * @param[in] sensorId 指定要获取雷达通道
     * @param[in] address 要读取的地址
     * @param[in] data 读取到数据存储位置，需要调用方提前设置vector大小为需要读取的长度
     * @return ClientErrorCode
     */
    uint16_t readSensorI2C(uint32_t sensorId, uint16_t address, std::vector<uint8_t> &data, uint16_t len);

    /**
     * @brief  对外提供雷达传感器写入接口
     * @details
     * @param[in] sensorId 指定要获取雷达通道
     * @param[in] address 要写入的地址
     * @param[in] data 需要写入的数据存储位置，长度为vector大小
     * @return ClientErrorCode
     */
    uint16_t writeSensorI2C(uint32_t sensorId, uint16_t address, const std::vector<uint8_t> &data, uint16_t len);

    /**
     * @brief  获取指定雷达基本信息
     * @details
     * @param[in] sensorId 指定雷达类型，返回对应雷达基本信息
     * @param[out] 雷达基本信息返回值
     * @return ClientErrorCode
     */
    uint16_t getSensorInfo(uint32_t sensorId, uint8_t &vendorType, uint8_t &sensorTypeID, uint32_t &sensorVer);

protected:
    CameraAccessInterface();
    ~CameraAccessInterface() = default;

    CameraAccessInterface(const CameraAccessInterface &interface) = delete;
    CameraAccessInterface operator=(const CameraAccessInterface &interface) = delete;
    CameraAccessInterface(const CameraAccessInterface &&interface) = delete;
    CameraAccessInterface operator=(const CameraAccessInterface &&interface) = delete;
};
} // namespace satradarAccess
} // namespace dimw
#endif
