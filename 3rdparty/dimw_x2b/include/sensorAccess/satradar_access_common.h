/**
 * @file camera_access_common.h
 * @brief
 * @author
 * @version 1.0.0
 * @date 2023-09-14 09-22-19
 * @copyright Copyright (c) 2023 BYD Company Limited
 */

#ifndef CAMERA_ACCESS_COMMON__H
#define CAMERA_ACCESS_COMMON__H
#ifdef __cplusplus
#include <cmath>
#include <cinttypes>

extern "C" {
#else
#include <math.h>
#include <inttypes.h>
#endif

#define CAMERA_ID_DEF(offset) (1U << (offset - 1))

#define CAMERA_NULL (0U)                           // 无效ID,无意义
#define CAMERA_REAR CAMERA_ID_DEF(1)               // 后视摄像头
#define CAMERA_FRONT_WIDE CAMERA_ID_DEF(2)         // 前视广角摄像头,适用于一路广角的场景
#define CAMERA_FRONT_NARROW CAMERA_ID_DEF(3)       // 前视窄角摄像头
#define CAMERA_SIDE_LEFT_FRONT CAMERA_ID_DEF(4)    // 左前侧视摄像头
#define CAMERA_SIDE_RIGHT_FRONT CAMERA_ID_DEF(5)   // 右前侧视摄像头
#define CAMERA_SIDE_RIGHT_REAR CAMERA_ID_DEF(6)    // 右后侧视摄像头
#define CAMERA_SIDE_LEFT_REAR CAMERA_ID_DEF(7)     // 左后侧视摄像头
#define CAMERA_SURROUND_FRONT CAMERA_ID_DEF(8)     // 前环视摄像头
#define CAMERA_SURROUND_RIGHT CAMERA_ID_DEF(9)     // 右环视摄像头
#define CAMERA_SURROUND_REAR CAMERA_ID_DEF(10)     // 后环视摄像头
#define CAMERA_SURROUND_LEFT CAMERA_ID_DEF(11)     // 左环视摄像头
#define CAMERA_DMS CAMERA_ID_DEF(12)               // DMS摄像头
#define CAMERA_FRONT_WIDE_LEFT CAMERA_ID_DEF(13)   // 左前视广角摄像头,适用于两路广角场景
#define CAMERA_FRONT_WIDE_RIGHT CAMERA_ID_DEF(14)  // 右前视广角摄像头,适用于两路广角场景

#define SATRADAR_ID_DEF(offset) (1U << (offset - 1))
#define SATRADAR_FRONT_LEFT_RADAR SATRADAR_ID_DEF(1U)
#define SATRADAR_FRONT_RIGHT_RADAR SATRADAR_ID_DEF(2U)
#define SATRADAR_REAR_LEFT_RADAR SATRADAR_ID_DEF(3U)
#define SATRADAR_REAR_RIGHT_RADAR SATRADAR_ID_DEF(4U)
#define SATRADAR_FRONT_RADAR_B SATRADAR_ID_DEF(5U)
#define SATRADAR_FRONT_RADAR_C SATRADAR_ID_DEF(6U)
#define SATRADAR_MIDDLE_LIDAR SATRADAR_ID_DEF(7U)
#define SATRADAR_LEFT_LIDAR SATRADAR_ID_DEF(8U)
#define SATRADAR_RIGHT_LIDAR SATRADAR_ID_DEF(9U)

typedef uint32_t CAMERA_ID;

enum CAMERA_ERR_CODE : uint32_t
{
  CAMERA_CODE_OK,                   // 调用成功
  CAMERA_CODE_PARAMETER_ERROR,      // 传入的参数错误
  CAMERA_CODE_COMMUNICATE_FAILED,   // 和摄像头接入服务通信失败
  CAMERA_CODE_EXECUTE_FAILED,       // 函数执行失败
  CAMERA_CODE_INSUFFICIENT_MEMORY,  // 给定的缓存不足以存下结果
};

enum CAMERA_IMU_CHANNEL_STATE : uint32_t
{
  CAMERA_IMU_UNKNOWN,        // 未知状态
  CAMERA_IMU_UNINITIALIZED,  // 未初始化
  CAMERA_IMU_INITIALIZED,    // 已成功初始化
  CAMERA_IMU_UNSUPPORT,      // 摄像头不带IMU
};

struct sensor_ciphertext_data
{
  CAMERA_ID id;       // 数据对应的相机ID
  uint32_t size;      // data_ptr 给定的缓存大小
  uint8_t* data_ptr;  // 给定的存密文数据的缓存
  uint32_t length;    // 有效密文数据的长度
};

#pragma pack(push, 1)
struct sensor_imu_data
{
  // CAMERA_ID id;
  uint16_t length;
  struct
  {
    float a;
    float b;
    float c;
    double reserved;
  } acc_x_zero_drift;
  struct
  {
    float a;
    float b;
    float c;
    double reserved;
  } acc_y_zero_drift;
  struct
  {
    float a;
    float b;
    float c;
    double reserved;
  } acc_z_zero_drift;
  struct
  {
    float a;
    float b;
    float c;
    double reserved;
  } gyro_x_zero_drift;
  struct
  {
    float a;
    float b;
    float c;
    double reserved;
  } gyro_y_zero_drift;
  struct
  {
    float a;
    float b;
    float c;
    double reserved;
  } gyro_z_zero_drift;
  struct
  {
    float c11;
    float c12;
    float c13;
    float c21;
    float c22;
    float c23;
    float c31;
    float c32;
    float c33;
  } acc_om;  // 加速度计正交矩阵
  struct
  {
    float c11;
    float c12;
    float c13;
    float c21;
    float c22;
    float c23;
    float c31;
    float c32;
    float c33;
  } gom;  // 陀螺仪正交矩阵
  struct
  {
    float sx;
    float sy;
    float sz;
  } asfe;  // 加速度计刻度误差
  struct
  {
    float sx;
    float sy;
    float sz;
  } gsfe;              // 陀螺仪刻度误差
  float reserved_acc;  // 加速度计/陀螺仪预留位
  struct
  {
    float bn_acc_x;
    float bn_acc_y;
    float bn_acc_z;
    float bn_gyro_x;
    float bn_gyro_y;
    float bn_gyro_z;
  } zbi;  // 零偏不稳定性
  struct
  {
    float bw_acc_x;
    float bw_acc_y;
    float bw_acc_z;
    float bw_gyro_x;
    float bw_gyro_y;
    float bw_gyro_z;
  } arw;              // 角度随机游走
  uint16_t opt_flag;  // 零漂用了几阶算法
  uint8_t imuchecksum_h;
  uint8_t imuchecksum_l;
  struct
  {
    double a;
    double b;
    double c;
    double d;
  } ic_revolution_quaternion;  // IMU到摄像头的四元数
  struct
  {
    double sx;
    double sy;
    double sz;
  } ic_vector;  // IMU到摄像头向量
  struct
  {
    double reserved_1;
    double reserved_2;
    double reserved3;
    double reserved4;
    double reserved5;
    double reserved6;
    double reserved7;
  } reserve;
  uint8_t checksum_h;
  uint8_t checksum_l;
};
#pragma pack(pop)

struct sensor_firmware_version
{
  CAMERA_ID id;
  char version[64U];
};

struct sensor_exp_time
{
  CAMERA_ID id;
  uint32_t exp_time;
};

struct sensor_module_info
{
  uint8_t moduleType;  //@ 0x0009 of EEPROM
  uint8_t moduleVer;   // 0x003C~0x003E
  uint8_t sensorType;  // 0x0005
  uint8_t lensType;    // 0x0006
  float_t lensHFov;    // 0x0010~0x0013
  float_t lensVFov;    // 0x0014~0x0017
  uint8_t serType;     // 0x0007
  uint8_t derType;     // des型号，MAX96712固定值
  uint8_t framerate;
};

enum SN_STATE : uint32_t
{
  CAMERA_SN_VALID,    // 内参和sn准备好
  CAMERA_SN_INVALID,  // 内参和sn未准备好
  CAMERA_SN_UNKNOWN,  // 未知状态
};

#ifdef __cplusplus
}
#endif
#endif  //! CAMERA_ACCESS_COMMON__H