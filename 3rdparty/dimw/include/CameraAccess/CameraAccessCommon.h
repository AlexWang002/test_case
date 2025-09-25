#ifndef _CAMERAACCESSCOMMON_H_
#define _CAMERAACCESSCOMMON_H_
#include <cmath>
#include <cinttypes>
#include <atomic>
#include <vector>
#include <string>

namespace dimw
{
namespace cameraaccess
{

enum camera_channel_state_t 
{
    unknown,
    uninitialized,
    initialized,
    invalid,
};

struct ModuleInfo 
{
    uint8_t moduleType; //@ 0x0009 of EEPROM
    uint8_t moduleVer;  // 0x003C~0x003E
    uint8_t sensorType; // 0x0005
    uint8_t lensType;   // 0x0006
    float_t lensHFov;   // 0x0010~0x0013
    float_t lensVFov;   // 0x0014~0x0017
    uint8_t serType;    // 0x0007
    uint8_t derType;    // des型号，MAX96712固定值
    uint8_t framerate;
};

/* Camera reattach info */
/* 注册生命周期 */
static const std::string g_serviceName = "com.byd.systemd.cameraAccess.service";
static const std::uint32_t g_heartBeatPeriod = 3000U; /* ms */
static const std::uint32_t g_missLimit = 5U;
static const std::string g_serviceNameMmtSensor = "mmt.maf.merge_sensor.service";   /* mmt服务：同一时刻，只会存在一个 */
static const std::string g_serviceNameMmtSystem = "mmt.maf.merge_system.service";   /* mmt服务：同一时刻，只会存在一个 */
static const std::string g_serviceNameInenc     = "com.byd.systemd.imageCodec.service";
static const std::string g_serviceNameDji       = "dji.nvstream.ipc";  /* 卓驭 */
static const std::string g_serviceNameMulticast = "com.byd.systemd.multicast.service";
static const std::string g_serviceNameIrd       = "com.byd.adas.app.perception.service";  /* IRD服务 */
static const std::string g_serviceCameraAccess  = "camera.access";   /* 显示cameraaccess服务的重启次数使用 */
/* 接入服务监听各consumer关闭、启动告警ID */
static const std::uint32_t g_cameraListerAlarmId = 20053;
enum class ServiceAlarmObjID : uint32_t 
{
    IMAGE_CODEC_OBJID = 3,  /* 内部编解码: 3 */
    MMT_SENSOR_OBJID,       /* MMT: 4 */
    MMT_SYSTEM_OBJID,       /* MMT: 5 */
    CAMERA_ACCESS_OBJID,    /* camera: 6 */
    DJI_SERVICE_OBJID,      /* 卓驭算法: 7 */
    IRD_SERVICE_OBJID,      /* ird算法: 8 */
    MULTICAST_OBJID = 10    /* cpu测试: 10 */
};

enum SupplierInfoID : std::uint8_t 
{
    SUPPLIER_ERR = 0,
    SUPPLIER_BYD,
    SUPPLIER_HIK,
};

/**
 * @brief sensor枚举信息
 */
enum class SensorID : std::uint32_t 
{
    CAM_FRONT_LEFT              = 0U,      ///< 前视广角摄像头左
    CAM_REAR                    = 1U,      ///< 后视摄像头
    CAM_FRONT_WIDE              = 2U,      ///< 前视广角摄像头右
    CAM_FRONT_NARROW            = 3U,      ///< 前视窄角摄像头
    CAM_SIDE_LEFT_FRONT         = 4U,      ///< 左前侧视摄像头
    CAM_SIDE_RIGHT_FRONT        = 5U,      ///< 右前侧视摄像头
    CAM_SIDE_RIGHT_REAR         = 6U,      ///< 右后侧视摄像头
    CAM_SIDE_LEFT_REAR          = 7U,      ///< 左后侧视摄像头
    CAM_SURROUND_FRONT          = 8U,      ///< 前环视摄像头
    CAM_SURROUND_RIGHT          = 9U,      ///< 右环视摄像头
    CAM_SURROUND_REAR           = 10U,     ///< 后环视摄像头
    CAM_SURROUND_LEFT           = 11U,     ///< 左环视摄像头
    RADAR_FRONT_SLAVE           = 12U,     ///< 前向毫米波雷达B
    RADAR_FRONT_MASTER          = 13U,     ///< 前向毫米波雷达C
    LIDAR_MIDDLE                = 14U,     ///< 中激光雷达
    SENSOR_RESERVE              = 15U,     ///< 预留
    RADAR_FRONT_LEFT            = 16U,     ///< 左前角毫米波雷达
    RADAR_FRONT_RIGHT           = 17U,     ///< 右前角毫米波雷达
    RADAR_REAR_LEFT             = 18U,     ///< 左后角毫米波雷达
    RADAR_REAR_RIGHT            = 19U,     ///< 右后角毫米波雷达

    SENSOR_TYPE_MAX,
};


/**
 * @brief 描述雷达相关的版本的信息
 */
struct SensorVerInfo
{
     /** 软件版本信息 */
    std::vector<uint8_t> softVerInfo;
     /** 硬件版本信息 */
    std::vector<uint8_t> hardVerInfo;
};


} // namespace cameraaccess
} // namespace dimw

#endif
