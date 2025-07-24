#ifndef SENSOR_IPC_CLIENT_H
#define SENSOR_IPC_CLIENT_H
#include "dilog/dilog.h"
#include "nvscibuf.h"
#include <vector>
#include <cstdint>
// #include "satradar_common.hpp"
namespace dimw
{
namespace sensor_access
{
/**
 * @brief  SensorType
 * @details 雷达类型定义，请使用定义名称检索雷达类型，避免后续雷达调整ID值
 */
enum class SensorType : std::uint32_t
{
  FRONT_LEFT_RADAR = 0,   // 左前角毫米波雷达
  FRONT_RIGHT_RADAR = 1,  // 右前角毫米波雷达
  REAR_LEFT_RADAR = 2,    // 左后角毫米波雷达
  REAR_RIGHT_RADAR = 3,   // 右后角毫米波雷达
  FRONT_RADAR_B = 4,      // 前向毫米波雷达B
  FRONT_RADAR_C = 5,      // 前向毫米波雷达C
  MIDDLE_LIDAR = 6,       // 中激光雷达
  LEFT_LIDAR = 7,         // 左激光雷达
  RIGHT_LIDAR = 8,        // 右激光雷达
  SENSOR_TYPE_MAX,
};

/**
 * @brief   AppType
 * @details 创建nvstream时，根据传入这些枚举类型区分创建的不同类型的nvstream
 */
enum class AppType : uint32_t
{
  IPC_DEMO_CONSUMER = 0,  ///< 创建demo consumer
  IPC_SDK_CONSUMER,       ///<
  IPC_ABS_CONSUMER,       ///<
  IPC_NOTHING             ///<
};

/**
 * @brief 枚举定义：客户端错误码
 * 用于表示客户端操作的返回结果和错误状态。
 */
enum class ClientErrorCode : std::int32_t
{
  CLIENT_SERVER_REMOUTE_ERR = -101,    ///< 服务器远程错误
  CLIENT_FEATURE_NOT_SUPPORTED = -99,  ///< 功能不支持
  CLIENT_SENSOR_NOT_EXIST = -6,        ///< 摄像头链路不存在
  CLIENT_OPRATE_ERROR,                 ///< 操作错误
  CLIENT_PARAM_ERROR,                  ///< 参数错误
  CLIENT_SYNC_ERROR,
  CLIENT_BUF_ERROR,
  CLIENT_INIT_FAILD,
  CLIENT_SUCCESS = 0,  ///< 成功
};

/* 雷达基本信息结构体 */
struct SensorInfo
{
  std::uint8_t vendorType;  //雷达模组厂家类型
  SensorType sensorType;    //雷达类型/ID
  std::uint32_t sensorVer;  //雷达版本
};

/**
 * @brief 数据结构：缓冲区帧对象
 * 描述了图像帧的基本属性及其关联对象信息。
 */
struct FrameBufObj
{
  uint32_t sensor;                // 雷达id
  uint32_t totalObj;              // 存储数据的队列深度
  uint32_t objIndex;              // 当前帧在队列中的index
  uint32_t frameIndex;            // 当前帧的帧序号
  uint64_t tsof;                  // 当前帧的Tsof时间戳
  uint64_t teof;                  // 当前帧的Teof时间戳
  uint64_t frameCaptureTSC;       // 当前帧TSC时间戳
  uint32_t sensorStatus;          // 雷达传感器状态
  NvSciBufObj bufObj{ nullptr };  // 雷达数据（nv相关引擎使用数据类型）
  uint8_t* data{ nullptr };       //基于bufObj转换后的非nv相关引擎使用的数据存储位置
  uint32_t size{ 0 };             // data大小
};

/**
 * @brief 客户端类：SensorIpcClient
 * SDK接口类：提供所有对外接口
 */
class SensorIpcClient
{
public:
  SensorIpcClient() = default;
  ~SensorIpcClient() = default;

  /**
   * @brief 初始化客户端
   *
   * @param[in] mask 可用摄像头的掩码，每个位表示一个摄像头是否可用。
   * @return ClientErrorCode 初始化结果：
   *   - CLIENT_SUCCESS：初始化成功
   *   - 其他错误码：初始化失败，参见 ClientErrorCode 枚举
   */
  ClientErrorCode init(uint16_t& mask, AppType appType = AppType::IPC_SDK_CONSUMER, dimw::dilog::Logger::ptr irdLoger = nullptr);

  /**
   * @brief 启动客户端服务
   *
   * @return ClientErrorCode 启动结果：
   *    - CLIENT_SUCCESS：启动成功
   *    - 其他错误码：启动失败，参见 ClientErrorCode 枚举
   */
  ClientErrorCode start(void);

  /**
   * @brief 停止客户端服务
   * @return ClientErrorCode 操作结果：
   *   - CLIENT_SUCCESS：停止成功
   *   - 其他错误码：停止失败，参见 ClientErrorCode 枚举
   */
  ClientErrorCode stop(void);

  /**
   * @brief 去初始化客户端
   * @return ClientErrorCode 操作结果：
   *   - CLIENT_SUCCESS：去初始化成功
   *   - 其他错误码：去初始化失败，参见 ClientErrorCode 枚举
   */
  ClientErrorCode deInit(void);

  /**
   * @brief 获取指定sensor的帧数据
   *
   * @param[in]  sensor   传感器类型，参见 SensorType 枚举。
   * @param[out] frameobj 返回的帧数据对象指针，包含帧的基本信息。
   * @return ClientErrorCode 操作结果：
   *   - CLIENT_SUCCESS：获取成功
   *   - 其他错误码：获取失败，参见 ClientErrorCode 枚举
   */
  ClientErrorCode getFrame(const SensorType& sensor, FrameBufObj& frameobj);

  /**
   * @brief 释放帧数据
   *
   * @param[in] frameobj 待释放的帧数据对象指针。
   * @return ClientErrorCode 操作结果：
   *    - CLIENT_SUCCESS：释放成功
   *    - 其他错误码：释放失败，参见 ClientErrorCode 枚举
   */
  ClientErrorCode releaseFrame(const FrameBufObj& frameobj);
  /**
   * @brief 获取雷达的link lock状态和video lock状态
   * @param[out] linkMask 雷达的link lock状态mask
                 videoMask雷达的video lock状态mask
   * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
   */
  ClientErrorCode getSensorStatus(uint16_t& linkMask, uint16_t& videoMask);
  /**
   * @brief 获取雷达fsync信号开始发送时刻
   * @param[out] fsyncStartTime fsync开始发送时刻的时间戳
   * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
   */
  ClientErrorCode getSensorFsyncStartTime(uint64_t& fsyncStartTime);

  /**
   * @brief 通过i2c接口读取雷达模组寄存器信息，读取1个字节
   * @param[in] sensor 指定要获取雷达通道
                address 要读取的寄存器地址
   * @param[out] data 读取到寄存器数据
   * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
   */
  ClientErrorCode readSensorRegUint8(const SensorType& sensor, uint16_t address, uint8_t& data);

  /**
   * @brief 通过i2c接口写雷达模组寄存器，写入1个字节
   * @param[in] sensor 指定要获取雷达通道
                address 要读取的寄存器地址
                data 要写入的寄存器数据
   * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
   */
  ClientErrorCode writeSensorRegUint8(const SensorType& sensor, uint16_t address, uint8_t data);

  /**
   * @brief 通过i2c接口读取雷达模组寄存器信息，读取2个字节
   * @param[in] sensor 指定要获取雷达通道
                address 要读取的寄存器地址
   * @param[out] data 读取到寄存器数据
   * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
   */
  ClientErrorCode readSensorRegUint16(const SensorType& sensor, uint16_t address, uint16_t& data);

  /**
   * @brief 通过i2c接口写雷达模组寄存器，写入2个字节
   * @param[in] sensor 指定要获取雷达通道
                address 要读取的寄存器地址
                data 要写入的寄存器数据
   * @return  0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
   */
  ClientErrorCode writeSensorRegUint16(const SensorType& sensor, uint16_t address, uint16_t data);

  /**
   * @brief  对外提供雷达传感器读取接口
   * @details
   * @param[in] sensor 指定要获取雷达通道
   * @param[in] address 要读取的地址
   * @param[in] data 读取到数据存储位置，需要调用方提前设置vector大小为需要读取的长度
   * @return ClientErrorCode
   */
  ClientErrorCode readSensorI2C(const SensorType& sensor, uint16_t address, uint8_t* data, uint16_t& len);

  /**
   * @brief  对外提供雷达传感器写入接口
   * @details
   * @param[in] sensor 指定要获取雷达通道
   * @param[in] address 要写入的地址
   * @param[in] data 需要写入的数据存储位置，长度为vector大小
   * @return ClientErrorCode
   */
  ClientErrorCode writeSensorI2C(const SensorType& sensor, uint16_t address, const uint8_t* data, uint16_t len);

  /**
   * @brief  获取所有雷达基本信息
   * @details
   * @param[out] 所有雷达基本信息返回值
   * @return ClientErrorCode
   */
  ClientErrorCode getSensorInfo(std::vector<SensorInfo>& info);
  /**
   * @brief  获取指定雷达基本信息
   * @details
   * @param[in] sensor指定雷达类型，返回对应雷达基本信息
   * @param[out] 雷达基本信息返回值
   * @return ClientErrorCode
   */
  ClientErrorCode getSensorInfo(const SensorType& sensor, SensorInfo& info);
};
}  // namespace sensor_access
}  // namespace dimw
#endif