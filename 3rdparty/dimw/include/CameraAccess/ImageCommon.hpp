#ifndef IMAGE_COMMON_H
#define IMAGE_COMMON_H

#include <math.h>
#include <iostream>
#include "nvscibuf.h"
#include "nvscisync.h"

namespace dimw {
namespace cameraipcclient {

#define     RADAR_OR_LIDAR_FRAME_TIMEOUT    150U        //ms

#define     EXPOSURES_NUM           4U

typedef struct {
    uint32_t sensor;                            // 摄像头id
    uint32_t total_obj;                         // 存储数据的队列深度
    uint32_t obj_index;                         // 当前yuv图像帧在队列中的index
    uint32_t raw_obj_index;                     // 当前raw图像帧在队列中的index
    uint32_t frame_index;                       // 当前的帧序号
    uint64_t frame_uuid;                        // getframe接口获取当前帧的时间
    NvSciBufObj obj;                            // yuv图像数据
    NvSciBufObj raw_obj;                        // raw图像数据
    uint64_t frame_timestamp;                   // 域控收到每帧第一行数据的时间戳，基于nv的frameCaptureStartTSC，转换后的ptp时间，单位us
    uint64_t frame_timestamp_teof;              // 域控收到每帧完整图像时刻的时间戳，基于nv的frameCaptureTSC，转换后的ptp时间，单位us
    uint64_t frame_timestamp_texp;              // 图像曝光中心点时刻时间戳，单位us
    float_t exposure_timestamp[EXPOSURES_NUM];  // 曝光时长（对应L/M/S/SV四种模式），单位s
    float_t gain[EXPOSURES_NUM];                // 增益（对应L/M/S/SV四种模式下的AGain*DGain），单位dB
} BufFrameObj;

/* IRD亮度信息结构体 */
typedef struct {
    int32_t roi_stats_valid = 0;   // defalt 0; 1:valid
    float frame_brightness = 0.0;  // 帧亮度信息
    void *data_ptr = nullptr;      // float 类型数据连续存放
    uint32_t data_len = 0;         // 数据大小，字节数
    uint32_t region_num = 0;       // 感兴趣区域数量, 4
    uint32_t block_per_region = 0; // 每个感兴趣区域划分块的数量 32*32  4*4
    uint32_t value_per_block = 0;  // 每个分块值的数量 4 RGGB信息
    uint32_t frame_index = 0;
} CameraFrameRoiStatsData;

enum SensorName {
    CAMERA_FRONT_RESERVE = 0x00,
    CAMERA_REAR,
    CAMERA_FRONT_WIDE,
    CAMERA_FRONT_NARROW,
    CAMERA_SIDE_LEFT_FRONT,
    CAMERA_SIDE_RIGHT_FRONT,
    CAMERA_SIDE_RIGHT_REAR,
    CAMERA_SIDE_LEFT_REAR,
    CAMERA_SURROUND_FRONT,
    CAMERA_SURROUND_RIGHT,
    CAMERA_SURROUND_REAR,
    CAMERA_SURROUND_LEFT,
    CAMERA_DMS,
    CAMERA_RESERVE1,
    CAMERA_RESERVE2,
    CAMERA_RESERVE3
};


/**
 * @brief   AppType
 * @details 创建nvstream时，根据传入这些枚举类型区分创建的不同类型的nvstream
 */
enum class AppType : uint32_t {
    CAM_DEMO_CONSUMER = 0,      ///< 创建demo consumer
    CAM_IRC_CONSUMER,           ///< 
    CAM_MMTENC_CONSUMER,        ///< 
    CAM_MMTCUDA_CONSUMER,       ///< 
    CAM_DJIENC_CONSUMER,        ///< 
    CAM_DJICUDA_CONSUMER,       ///< 
    CAM_VIC_CONSUMER,           ///< 
    CAM_INENC_CONSUMER,         ///< 
    /* 雷达 */
    SEN_DEMO_CONSUMER,          ///< 创建demo consumer
    SEN_IRC_RADAR_CONSUMER,     ///< 
    SEN_IRC_LIDAR_CONSUMER,     ///< 
    SEN_ABS_CONSUMER,           ///< 
    IPC_NOTHING                 ///< 
};


/**
 * @brief 枚举定义：客户端错误码
 * 用于表示客户端操作的返回结果和错误状态。
 */
enum ClientErrorCode : std::int32_t {
    CLIENT_SERVER_REMOTE_ERR = -101,    ///< 服务器远程错误
    CLIENT_FEATURE_NOT_SUPPORTED = -99, ///< 功能不支持
    CLIENT_NOT_INIT = -9,               ///< 客户端未初始化
    CLIENT_TIME_OUT = -8,               ///< 超时
    CLIENT_SENSOR_NOT_EXIST = -7,       ///< 摄像头链路不存在
    CLIENT_OPRATE_ERROR = -6,                ///< 操作错误
    CLIENT_PARAM_ERROR = -5,                 ///< 参数错误
    CLIENT_SYNC_ERROR = -4,
    CLIENT_BUF_ERROR = -3,
    CLIENT_INIT_FAILD = -2,
    CLIENT_GETFRAME_FAILD = -1,
    CLIENT_SUCCESS = 0,               ///< 成功
};
/**
 * @brief  SensorType 
 * @details 雷达类型定义，请使用定义名称检索雷达类型，避免后续雷达调整ID值
 */
enum class SensorType : std::uint32_t {
    /* 摄像头传感器 */
    CAMERA_FRONT_RESERVE = 0x00,    ///< 前视广角摄像头左
    CAMERA_REAR,                    ///< 后视摄像头
    CAMERA_FRONT_WIDE,              ///< 前视广角摄像头右
    CAMERA_FRONT_NARROW,            ///< 前视窄角摄像头
    CAMERA_SIDE_LEFT_FRONT,         ///< 左前侧视摄像头
    CAMERA_SIDE_RIGHT_FRONT,        ///< 右前侧视摄像头
    CAMERA_SIDE_RIGHT_REAR,         ///< 右后侧视摄像头
    CAMERA_SIDE_LEFT_REAR,          ///< 左后侧视摄像头
    CAMERA_SURROUND_FRONT,          ///< 前环视摄像头
    CAMERA_SURROUND_RIGHT,          ///< 右环视摄像头
    CAMERA_SURROUND_REAR,           ///< 后环视摄像头
    CAMERA_SURROUND_LEFT,           ///< 左环视摄像头

    /* 激光雷达/毫米波雷达传感器 */
    FRONT_RADAR_SLAVE = 0x0C,       ///< 前向毫米波雷达B
    FRONT_RADAR_MASTER,             ///< 前向毫米波雷达C
    MIDDLE_LIDAR,                   ///< 中激光雷达
    FRONT_LEFT_RADAR = 0x10,        ///< 左前角毫米波雷达
    FRONT_RIGHT_RADAR,              ///< 右前角毫米波雷达
    REAR_LEFT_RADAR,                ///< 左后角毫米波雷达
    REAR_RIGHT_RADAR,               ///< 右后角毫米波雷达
    LEFT_LIDAR,                     ///< 左激光雷达
    RIGHT_LIDAR,                    ///< 右激光雷达

    SENSOR_TYPE_MAX,
};

enum InnerParaState : uint32_t {
    INNERPARA_NOREADY = 0U,     // 内参和sn未准备好
    INNERPARA_VALID,            // 内参和sn准备好
    INNERPARA_INVALID,          // 内参和sn无效
    INNERPARA_UNKNOWN,          // 未知状态
};

/* 设置fsync对应的groupid */
enum GroupId : uint32_t {
    FRONT_GROUP = 0U,
    SIDE_GROUP,
    SURROUND_GROUP,
    LIDAR_RADAR_GROUP,

    UNKNOWN_GROUP,
};

/**
 * @brief 数据结构：缓冲区帧对象
 * 描述了图像帧的基本属性及其关联对象信息。
 */
struct FrameBufObj {
    uint32_t sensor;            // 雷达id
    uint32_t totalObj;          // 存储数据的队列深度
    uint32_t objIndex;          // 当前帧在队列中的index
    uint32_t rawObjIndex;       // 当前帧在队列中的index
    uint32_t frameIndex;        // 当前帧的帧序号
    uint64_t frameUuid;   
    uint64_t tsof;              // 当前帧的Tsof时间戳
    uint64_t teof;              // 当前帧的Teof时间戳
    uint64_t frameCaptureTSC;   // 当前帧TSC时间戳
    uint32_t sensorStatus;      // 雷达传感器状态
    NvSciBufObj bufObj{nullptr};     // 雷达数据（nv相关引擎使用数据类型）
    NvSciBufObj rawBufObj{nullptr};  // 雷达数据（nv相关引擎使用数据类型）
    uint8_t *data{nullptr};    //基于bufObj转换后的非nv相关引擎使用的数据存储位置
    uint32_t size{0};          //data大小
};

/* 雷达基本信息结构体 */
struct SensorInfo {
    std::uint8_t vendorType;      //雷达模组厂家类型
    std::uint8_t sensorType;      //雷达类型/ID
    std::uint64_t sensorVer;      //雷达版本
};

/* 传感器类型信息结构体 */
struct SensorTypeInfo {
    bool dataValid = false;
    uint8_t sensorType = 0;
};

}
}

#endif