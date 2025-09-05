/**
 * @file lidar_sdk_api.h
 * @brief lidar SDK对外接口文件定义
 * @details 定义调用底软函数指针，sdk函数指针等
 * @author
 * @version 1.0.15
 * @date 2025/08/27
 * @copyright copyright(c)2025 比亚迪股份有限公司
 */

#ifndef LIDAR_SDK_API_H
#define LIDAR_SDK_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define LIDAR_SDK_API_VER_MAJOR 1
#define LIDAR_SDK_API_VER_MINOR 0
#define LIDAR_SDK_API_VER_PATCH 15

#define LIDAR_SDK_API_VER_MACRO(major, minor, patch) (major << 16 | minor << 8 | patch)

#define LIDAR_SDK_API_MAJOR_GET(version) ((version >> 16) & 0xFF)
#define LIDAR_SDK_API_MINOR_GET(version) ((version >> 8) & 0xFF)
#define LIDAR_SDK_API_PATCH_GET(version) (version & 0xFF)

#define LIDAR_SDK_API_VER LIDAR_SDK_API_VER_MACRO(LIDAR_SDK_API_VER_MAJOR, LIDAR_SDK_API_VER_MINOR, LIDAR_SDK_API_VER_PATCH)

typedef enum
{
  MIDDLE_LIDAR = 6,  // 中激光雷达
  LEFT_LIDAR = 7,    // 左激光雷达
  RIGHT_LIDAR = 8,   // 右激光雷达
} LidarSensorIndex;

typedef enum
{
  LIDAR_SDK_FAILD = -1,
  LIDAR_SDK_SUCCESS = 0,
} LidarSdkErrorCode;

typedef enum
{
  POSITIVE = 0X00,
  GENERAL_REJECT = 0X10U,
  INCORRECT_MESSAGE_LEN = 0X13U,
  CONDITIONS_NOT_CORRECT = 0X22U,
  REQUEST_SEQUENCE_ERROR = 0X24U,
  OUT_OF_RANGE = 0X31U,
  GENERAL_PROGRAMMING_FAILURE = 0X72U,
  RESOURCE_TEMPORARILY_NOT_AVAILABLE = 0X94U
} LidarSdkDtcNegativeResp;

typedef struct
{
  uint32_t sensor;          // 雷达id
  uint32_t frameIndex;      // 当前帧的帧序号(lidar mipi)
  uint64_t tsof;            // 当前帧的TSOf时间戳，底软打的时间戳
  uint64_t teof;            // 当前帧的reof时间戳，底软打的时间戳
  void* bufObj;             // 雷达数据(暂时跟图像写一个类型名，实际使用中需要根据雷达数据格式定义类型名)
  void* data;               // cpu侧数据指针
  uint32_t len;             // len为bufobj为CPU侧计算是有效，使用nv gpu时无效
  const void* reservedPtr;  // reserved only for middleware
} LidarAdcBuffer;

typedef struct
{
  uint16_t radius;           // 径向距离
  uint8_t intensity;         // 通道点反射强度值
  uint8_t point_attribute;   // 雨雾噪点属性
}__attribute__((packed)) ChannelData;

typedef struct
{
  uint16_t time_offset;          // 可以精确到10us的级别
  int16_t motor_speed;           // 电机转速
  int16_t azimuth;               // 水平角
  ChannelData channel_data[192]; // 192个通道数据
}__attribute__((packed)) DataBlock;

typedef struct
{
  uint64_t frame_timestamp;         // 完整的时间戳(精确到微秒)
  uint32_t point_num;               // 一帧里点的个数
  uint32_t frame_seq;               // 数据帧序列号
  void*    lidar_parameter;         // 内参信息，供应商提供
  uint16_t lidar_parameter_length;  // 内参信息长度，供应商提供
  uint16_t protocol_version;        // 通信协议版本号，供应商自己定义
  uint8_t  return_mode;             // 回波模式设置:0-双回波，1-最强回波，2-最后回波，3-第一回波
  uint8_t  sync_status;             // 时间同步状态:0:已经同步 1:未同步
  uint8_t  frame_sync;              // 帧同步，0表示未同步，1表示已同步
  uint8_t  mirror_id;               // 奇偶帧标识（适用摆镜方案）
  uint32_t data_length;             // 预留字段
  uint32_t data_id;                 // 预留字段
  uint32_t crc32;                   // 预留字段
  uint16_t counter;                 // 预留字段
  uint16_t packet_num;              // packets个数
  uint8_t  reserved[6];             // reserved
  DataBlock packets[1];             // packet数据，保证此内存和header内存连续
}__attribute__((packed)) LidarPointCloudPackets;

typedef struct
{
  uint16_t firware_version;                      // 固件版本号
  uint16_t sdk_version;                          // SDK版本号
  uint16_t motor_speed;                          // 电机转速，单位rpm
  uint8_t return_mode;                           // 回波模式设置：0-双回波，1-最强回波，2-最后回波，3-第一回波
  uint64_t timestamp;                            // 微秒级时间
  uint8_t lidar_operation_state;                 // 雷达运行状态(正常工作状态：0x00, 故障状态：0x01)
  uint8_t lidar_fault_state;                     // 雷达故障状态(无故障：0x00; 一级故障：0x01; 二级故障：0x02)
                                                 // 一级故障：有点云不影响性能, 二级故障：无点云或点云不可用
  uint8_t sdk_total_fault_number;                // SDK故障码总数
  uint64_t sdk_fault_code_position;              // SDK故障码在队列中的位置, 对于每bit位，0x00 - 不存
                                                 // 在此故障，0x01- 存在此故障。详见表《故障信息》
  uint16_t supplier_internal_fault_id;           // 供应商内部故障ID(按需填充，若无填充0x0)
  uint8_t supplier_internal_fault_indicate[12];  // 供应商故障详细指示信息(按需填充，若无填充0x0)
  uint8_t supplier_internal_fault_number;        // 供应商内部故障总数(按需填充，若无填充0x0)
  uint8_t supplier_internal_fault_position;      // 供应商内部故障在队列中的位置(按需填充，若无填充0x0)

  uint8_t time_sync_mode;        // 时间同步模式(0x00：当前使用雷达内部自己计时;
                                 // 0x01：CAN TSYN； 0x02： FSYNC；0x03：预留)
  uint8_t time_sync_status;      // 时间同步状态(0x00：时间同步失败; 0x01：时间同步成功)
  uint64_t time_offset;          // 时间偏差
  uint8_t lidar_product_sn[25];  // 产品序列号
  uint8_t manufacture;           // 制造商
  uint8_t model;                 // 型号
  uint8_t reserved[50];          // 预留字段
} LidarDeviceInfo;

typedef struct
{
  LidarSdkErrorCode (*releaseAdc)(LidarSensorIndex sensor, const void* ptrAdc);  // ptrAdc为结构体LidarAdcBuffer

  LidarSdkErrorCode (*getSensorFsyncStartTime)(uint64_t* time);

  uint64_t (*getTimeNowPhc)();  /// 获取物理面时间接口（纳秒精度）时间同步和传感器对齐的时间
  uint64_t (*getTimeNowSys)();  /// 获取系统面时间接口（纳秒精度）
  uint64_t (*getTimeNowMgr)();  /// 获取管理面时间接口（纳秒精度）

  LidarSdkErrorCode (*writeSensorRegUint8)(LidarSensorIndex sensor, uint16_t address, uint8_t data);
  LidarSdkErrorCode (*readSensorRegUint8)(LidarSensorIndex sensor, uint16_t address, uint8_t* data);
  LidarSdkErrorCode (*writeSensorRegUint16)(LidarSensorIndex sensor, uint16_t address, uint16_t data);
  LidarSdkErrorCode (*readSensorRegUint16)(LidarSensorIndex sensor, uint16_t address, uint16_t* data);
  LidarSdkErrorCode (*readSensorI2C)(LidarSensorIndex sensor, uint16_t address, uint8_t* data, uint16_t* length);
  LidarSdkErrorCode (*writeSensorI2C)(LidarSensorIndex sensor, uint16_t address, const uint8_t* data, uint16_t length);

  /**
   * pointcloud 回调中不会有拷贝操作
   */
  LidarSdkErrorCode (*pointCloud)(LidarSensorIndex sensor, const void* buffer, uint32_t length);  // buf对应的结构体为LidarPointCloud
  LidarSdkErrorCode (*deviceInfo)(LidarSensorIndex sensor, const void* buffer, uint32_t length);  // buf对应的结构体为LidarDeviceInfo
} LidarSdkCbks;

typedef struct
{
  uint32_t apiVersion;                  // set to LIDAR_SDK_API_VER for check, see demo
  const char* (*getLidarSdkVersion)();  // 版本号返回格式 x.xx.xx 字符串，增量0.00.01，和设备信息中的版本号代表的含义一致

  LidarSdkErrorCode (*init)(const LidarSdkCbks* fptrCbks, const char* configPath);
  LidarSdkErrorCode (*deInit)(void);
  LidarSdkErrorCode (*start)(void);
  LidarSdkErrorCode (*stop)(void);

  LidarSdkErrorCode (*injectAdc)(LidarSensorIndex sensor, const void* ptrAdc);  ////ptrAdc为结构体LidarAdcBuffer

  LidarSdkErrorCode (*writeDid)(uint16_t did, uint8_t* data, uint16_t dataLen, uint8_t* nrc);  ////nrc 对应的结构体LidarSdkDtcNegativeResp
  LidarSdkErrorCode (*readDid)(uint16_t did, uint8_t* data, uint16_t* dataLen, uint8_t* nrc);  ////nrc 对应的结构体LidarSdkDtcNegativeResp
  LidarSdkErrorCode (*ridControl)(uint8_t mode, uint16_t rid, uint8_t* dataIn, uint16_t dataInLen, uint8_t* dataOut, uint16_t* dataOutLen,
                                  uint8_t* nrc);  // nrc 对应的结构体LidarSdkDtcNegativeResp
} LidarSdkInterface;

// LidarSdkInterface * BYDLidarSdkGetInterface(size_t sizeOfInterface); // 供应商头文件中定义并实现

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_SDK_API_H