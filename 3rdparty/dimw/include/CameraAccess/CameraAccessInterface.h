/**
 * @file CameraAccessInterface.h
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
#include <mutex>
#include <unordered_map>
#include "CameraAccessCommon.h"

namespace dimw
{
namespace cameraaccess
{

struct SensonInfo{
    uint8_t vendorType;
    uint8_t sensorTypeID; 
    uint64_t sensorVer;
};

/**
 * @file CameraAccessInterface.h
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
     * @brief 控制整组摄像头上电
     * @details 控制整组摄像头上电
     * @param[in]  groupNo
     * @param[out] 无
     * @return powerOnGroupCamera的返回值
     *       @retval 上电成功返回0，入参非法返回1，上电失败返回0xff
     */
    int32_t powerOnGroupCamera(const uint8_t &groupNo) __attribute__((deprecated(
        "powerOnGroupCamera is deprecated. Use camera_group_power_on instead.")));

    /**
     * @brief 控制整组摄像头下电
     * @details 控制整组摄像头下电
     * @param[in]  groupNo
     * @param[out] 无
     * @return powerOffGroupCamera的返回值
     *       @retval 下电成功返回0，入参非法返回1，下电失败返回0xff
     */
    int32_t powerOffGroupCamera(const uint8_t &groupNo) __attribute__((deprecated(
        "powerOffGroupCamera is deprecated. Use camera_group_power_off instead.")));

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
     * @brief 获取EEPROM密文数据
     * @details 获取EEPROM密文数据
     * @param[in]  sensorId 指定摄像头ID
     * @param[out] vEEPRomData 透传EEPROM字节数据，失败返回空vector
     * @return 成功返回true，失败返回false
     *       @retval true 执行成功
     *       @retval false 执行失败
     */
    bool getEEPROMData(uint8_t sensorId, std::vector<uint8_t> &vEEPRomData) __attribute__((deprecated(
        "getEEPROMData is deprecated. Use camera_get_ciphertext_data instead.")));
    
    /**
     * @brief 设置摄像头曝光时长，并获取曝光时间
     * @details 设置摄像头曝光时长，并获取曝光时间
     * @param[in]  expTimeIn曝光时间
     * @param[in]  cameraID 0表示同步设置四个环视相机 1表示设置前环视 2表示设置左环视 3表示设置后环视 4表示设置右环视
     * @param[out] expTimeOut 设置后的曝光时间
     * @return uint32_t  
     *       @retval 0表示最大曝光时长设置成功 1表示输入参数非法 2表示最大曝光时长设置失败
     */
    uint32_t setAvmMaxExpTime(uint32_t expTimeIn, uint32_t cameraID, std::vector<uint32_t> &expTimeOut) __attribute__((deprecated(
        "setAvmMaxExpTime is deprecated. Use camera_set_max_exp_time instead.")));

    /**
     * @brief 获取环视摄像头版本号和厂商信息
     * @details 获取环视摄像头版本号和厂商信息
     * @param[in]  无
     * @param[out] versionVec 环视摄像头版本信息
     * @param[out] cameraSupplierID 环视摄像头厂商信息,4个环视为一组,故此处只统计一个
     * @return 成功返回0，失败返回 (2: 通信出错, 3: 未知错误)
     */
    uint32_t getSurCameraVerion(std::map<uint32_t, std::string> &verinfomap, SupplierInfoID &cameraSupplierID, std::string & errInfo) __attribute__((deprecated(
        "getSurCameraVerion is deprecated. Use camera_get_sensor_verion instead.")));

    /**
     * @brief 升级环视摄像头版本
     * @details 升级环视摄像头版本
     * @param[in]  newFwPath 版本路径
     * @param[in]  isUpdata  true表示升级，false表示rollback
     * @param[out] 无
     * @return 成功返回0，失败返回1
     */
    uint32_t writeSurCameraFW(const std::string &newFwPath, std::string & errInfo, const bool &isUpdata = true) __attribute__((deprecated(
        "writeSurCameraFW is deprecated. Use camera_upgrade_sensor_firmware instead.")));

    /**
     * @brief 查询升级进度
     * @details 查询升级进度
     * @param[in]  无
     * @param[out] num 返回升级进度
     * @return 成功返回0，失败返回1
     */
    uint32_t getSurCameraFwUpdatePro(uint32_t &num, std::string & errInfo) __attribute__((deprecated(
        "getSurCameraFwUpdatePro is deprecated. Use camera_get_sensor_upgrade_progress instead.")));

    /**
     * @brief 检测摄像头存在状态
     *
     * @param sensorId 指定摄像头ID
     * @return true 摄像头存在
     *      @retval 成功返回摄像头存在状态掩码, 比如0x70，表示摄像头编号4/5/6 存在。
     *      @retval 失败返回0xefff
     */
    uint32_t detectAllSensors(void) __attribute__((deprecated(
        "detectAllSensors is deprecated. Use camera_detect_all_sensors instead.")));

    /**
     * @brief 获取前视带IMU摄像头链路初始化状态
     * @details 获取前视带IMU摄像头链路初始化状态	
     * @param[in] 无
     * @return 摄像头初始化状态
     */
    camera_channel_state_t get_camera_with_imu_channel_state() __attribute__((deprecated(
        "get_camera_with_imu_channel_state is deprecated. Use camera_get_imu_channel_state instead.")));
	/**
     * @brief 检测link lock 和video link
     *
     * @param 
     * @return true 执行成功
     */

    uint32_t detectLinkAndVideoMask(uint32_t &linkMask, uint32_t &videoMask) __attribute__((deprecated(
        "detectLinkAndVideoMask is deprecated. Use sensor_detect_link_video_mask instead.")));

    /**
     * @brief 获取当前soc上接入服务首次启动ptp时间
     * @details 获取当前soc上接入服务首次启动ptp时间
     * @param[in] 无
     * @return 获取当前soc上接入服务首次启动ptp时间
     */
    uint64_t get_fsync_first_time() __attribute__((deprecated(
        "get_Fsync_first_time is deprecated. Use camera_get_fsync_first_time instead."))); 

    /**
     * @brief 获取摄像头工作状态，nvstream是否建链成功
     * @details 获取摄像头工作状态，nvstream是否建链成功
     * @param[in] 无
     * @return true表示建链成功，false表示失败
     */
    bool get_camera_state() __attribute__((deprecated(
        "get_camera_work_state is deprecated. Use camera_get_work_state instead."))); 

    /**
     * @brief  
     * @details
     * @param[in] appName
     * @param[in] ipcNames
     * @return CAMERA_ERR_CODE
     */
    uint32_t registerCameraAttachChannel(const std::string &appName, const std::vector<std::string> &ipcNames) __attribute__((deprecated(
        "registerCameraAttachChannel is deprecated. Use camera_attach_channel_register instead.")));

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
    uint16_t readSensorI2C(uint32_t sensorId, uint16_t address, std::vector<uint8_t> &data, uint16_t &len);

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
    uint16_t getSensorInfo(uint32_t sensorId, uint8_t &vendorType, uint8_t &sensorTypeID, uint64_t &sensorVer);

    /**
     * @brief 设置某个模组的fsync delta时间
     * 
     * @param groupId 
     * @param delayTimeMs 
     * @return uint16_t 
     */
    uint16_t setFsyncDeltaMs(uint32_t groupId, uint64_t delayTimeMs);

    /**
     * @brief 获取所有传感器sensorType信息
     * 
     * @param allSensorTypeInfo 
     * @return uint16_t 
     */
    uint16_t getAllSensorTypeInfo(uint32_t sensorId, bool& dataValid, uint8_t& sensorType);

    uint16_t getRadarInnerParaState(uint32_t& state);
    uint16_t getLidarInnerParaState(uint32_t& state);
    
    /**
     * @brief 获取指定激光雷和毫米波雷达的信息
     * 
     * @param[in] sensorId 雷达ID
     * @param[out] verInfo 版本信息
     * @return 详见camera_access_common.h中的CAMERA_ERR_CODE枚举值
     */
    uint32_t getSensorVerInfo(SensorID sensorId, SensorVerInfo& verInfo);

    /**
     * @brief 雷达信息的状态
     * 
     * @param[in] 无
     * @param[out] status 雷达信息是否就位（true:雷达信息已准备好;false:雷达信息未准备好）
     * @return 详见camera_access_common.h中的CAMERA_ERR_CODE枚举值
     */
    uint32_t getSensorInfoState(bool& status);

    /**
     * @brief 根据告警的alarmId和alarmObj获取毫米波雷达的二级故障信息
     * 
     * @param[in] alarmId 用于标识告警类型
     * @param[in] alarmObj 用于标识告警Obj
     * @param[out] faultVec 对应的二级故障信息
     * @return 详见camera_access_common.h中的CAMERA_ERR_CODE枚举值
     */
    uint32_t getRadarLev2Fault(const uint16_t alarmId, const uint16_t alarmObj, std::vector<uint16_t> &faultVec);

protected:
    CameraAccessInterface();
    ~CameraAccessInterface() = default;

    CameraAccessInterface(const CameraAccessInterface &interface) = delete;
    CameraAccessInterface operator=(const CameraAccessInterface &interface) = delete;
    CameraAccessInterface(const CameraAccessInterface &&interface) = delete;
    CameraAccessInterface operator=(const CameraAccessInterface &&interface) = delete;

private:
    std::mutex m_mutex;
    std::unordered_map<uint32_t,SensonInfo> sensorInfoMap;
};
} // namespace cameraaccess
} // namespace dimw
#endif
