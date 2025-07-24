#ifndef SENSOR_IPC_CLIENT_H
#define SENSOR_IPC_CLIENT_H
#include "dilog/dilog.h"
#include "ImageCommon.hpp"
#include <vector>
#include <cstdint>
namespace dimw {
namespace sensor_access {

using namespace cameraipcclient;

class SensorIpcClient {
public:
    SensorIpcClient(AppType appType = AppType::SEN_IRC_RADAR_CONSUMER);
    ~SensorIpcClient() = default;

    /**
     * @brief 初始化
     * 
     * @param mask [in] mask ：每个bit标识SensorId，每个bit定义见枚举类型SensorName
     * @param irdLoger [in] irdLoger：log指针，默认为nullptr
     * @return 0：success ，-1：faild 
     */
    ClientErrorCode init(uint32_t &mask, dimw::dilog::Logger::ptr irdLoger = nullptr);

    /**
     * @brief：使能数据同步操作，成功或者超时后返回
     * @return :0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode start(void);

    /**
     * @brief：获取一帧camera数据，和 ReleaseFrame 配对使用;
     * @param[in] sensor ：指定要获取图像的摄像头通道
     * @param[out] frameobj ：获取到一帧数据信息，NvSciBufObj obj 中包camera 真实的camera数据，请勿修改frameobj
     * 中任何参数的值 通过NvSciBufObjGetConstCpuPtr可获取内存地址 \return :0：success ;<0:失败，根据 ClientErrorCode
     * 定义查看具体原因。
     */
    ClientErrorCode getFrame(const SensorType &sensor, FrameBufObj &frameobj);

    /**
     * @brief：释放一帧camera数据 和 GetFrame 配对使用
     * @param[in] obj ：释放 sensor 和 pip 指定camera 数据（使用GetFrame 中获取到 frameobj 对象）
     * @return :0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode releaseFrame(const FrameBufObj &frameobj);

    /**
     * @brief：stop
     */
    void stop(void);

    /**
     * @brief：去初始化
     */
    void deInit(void);

    /**
     * @brief 获取雷达的link lock状态和video lock状态
     * @param[out] linkMask 雷达的link lock状态mask
                   videoMask雷达的video lock状态mask
     * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode getSensorStatus(uint32_t& linkMask, uint32_t& videoMask);

    /**
     * @brief 获取雷达fsync信号开始发送时刻
     * @param[out] fsyncStartTime fsync开始发送时刻的时间戳
     * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode getSensorFsyncStartTime(uint64_t& fsyncStartTime);
    
    /**
     * @brief 通过i2c接口读取雷达模组寄存器信息，读取1个字节
     * @param[in] sensor 指定要获取雷达通道
     * @param[in] address 要读取的寄存器地址
     * @param[out] data 读取到寄存器数据
     * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode readSensorRegUint8(const SensorType &sensor, uint16_t address, uint8_t &data);
    
    /**
     * @brief 通过i2c接口写雷达模组寄存器，写入1个字节
     * @param[in] sensor 指定要获取雷达通道
     * @param[in] address 要读取的寄存器地址
     * @param[in] data 要写入的寄存器数据
     * @param[in] prio 优先级（暂时占位，无实际意义）
     * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode writeSensorRegUint8(const SensorType &sensor, uint16_t address, uint8_t data, uint8_t prio = 0);
    
    /**
     * @brief 通过i2c接口读取雷达模组寄存器信息，读取2个字节
     * @param[in] sensor 指定要获取雷达通道
     * @param[in] address 要读取的寄存器地址
     * @param[out] data 读取到寄存器数据
     * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode readSensorRegUint16(const SensorType &sensor, uint16_t address, uint16_t &data);
    
    /**
     * @brief 通过i2c接口写雷达模组寄存器，写入2个字节
     * @param[in] sensor 指定要获取雷达通道
     * @param[in] address 要读取的寄存器地址
     * @param[in] data 要写入的寄存器数据
     * @param[in] prio 优先级（暂时占位，无实际意义）
     * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    ClientErrorCode writeSensorRegUint16(const SensorType &sensor, uint16_t address, uint16_t data,  uint8_t prio = 0);

    /**
     * @brief  对外提供雷达传感器读取接口
     * @details
     * @param[in] sensor 指定要获取雷达通道
     * @param[in] address 要读取的地址
     * @param[in] data 读取到数据存储位置，需要调用方提前设置vector大小为需要读取的长度
     * @return ClientErrorCode
     */
    ClientErrorCode readSensorI2C(const SensorType &sensor, uint16_t address, uint8_t *data, uint16_t &len);

    /**
     * @brief  对外提供雷达传感器写入接口
     * @details
     * @param[in] sensor 指定要获取雷达通道
     * @param[in] address 要写入的地址
     * @param[in] data 需要写入的数据存储位置，长度为vector大小
     * @param[in] len 需要写入的数据长度
     * @param[in] prio 优先级（暂时占位，无实际意义）
     * @return ClientErrorCode
     */
    ClientErrorCode writeSensorI2C(const SensorType &sensor, uint16_t address, const uint8_t *data, uint16_t len, uint8_t prio = 0);

    /**
     * @brief  获取所有雷达基本信息
     * @details
     * @param[out] 所有雷达基本信息返回值
     * @return ClientErrorCode
     */
    ClientErrorCode getSensorInfo(std::vector<SensorInfo> &info);

    /**
     * @brief  获取指定雷达基本信息
     * @details
     * @param[in] sensor指定雷达类型，返回对应雷达基本信息
     * @param[out] 雷达基本信息返回值
     * @return ClientErrorCode
     */
    ClientErrorCode getSensorInfo(const SensorType &sensor, SensorInfo &info);

private:
    AppType m_appType;
};

}
}

#endif