#ifndef _CAMERAACCESSCOMMON_H_
#define _CAMERAACCESSCOMMON_H_
#include <cmath>
#include <cinttypes>
#include <atomic>
#include <vector>
#include <string>

namespace dimw
{
namespace satradarAccess
{

enum camera_channel_state_t {
    unknown,
    uninitialized,
    initialized,
    invalid,
};

struct ModuleInfo {
    std::uint8_t vendorType;      //雷达模组厂家类型
    std::uint8_t sensorTypeID;        //雷达类型/ID
    std::uint32_t sensorVer;      //雷达版本
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
static const std::string g_serviceName = "com.byd.systemd.sensorAccess.service";
static const std::uint32_t g_heartBeatPeriod = 3000U; /* ms */
static const std::uint32_t g_missLimit = 5U;
static const std::string g_serviceNameMmtSensor = "mmt.maf.merge_sensor.service";   /* mmt服务：同一时刻，只会存在一个 */
static const std::string g_serviceNameMmtSystem = "mmt.maf.merge_system.service";   /* mmt服务：同一时刻，只会存在一个 */
static const std::string g_serviceNameInenc     = "com.byd.systemd.imageCodec.service";
static const std::string g_serviceNameDji       = "dji.nvstream.ipc";  /* 卓驭 */
static const std::string g_serviceNameMulticast = "com.byd.systemd.multicast.service";
static const std::string g_serviceNameIrd       = "com.byd.adas.app.perception.service";  /* IRD服务 */
static const std::string g_serviceCameraAccess  = "satradar.access";   /* 显示satradaraccess服务的重启次数使用 */
/* 接入服务监听各consumer关闭、启动告警ID */
static const std::uint32_t g_cameraListerAlarmId = 20053;
enum class ServiceAlarmObjID : uint32_t {
    IMAGE_CODEC_OBJID = 3,  /* 内部编解码: 3 */
    MMT_SENSOR_OBJID,       /* MMT: 4 */
    MMT_SYSTEM_OBJID,       /* MMT: 5 */
    CAMERA_ACCESS_OBJID,    /* camera: 6 */
    DJI_SERVICE_OBJID,      /* 卓驭算法: 7 */
    IRD_SERVICE_OBJID,      /* ird算法: 8 */
    MULTICAST_OBJID = 10    /* cpu测试: 10 */
};
enum SupplierInfoID : std::uint8_t {
    SUPPLIER_ERR = 0,
    SUPPLIER_BYD,
    SUPPLIER_HIK,
};

} // namespace satradarAccess
} // namespace dimw

#endif
