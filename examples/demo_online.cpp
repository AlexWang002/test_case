/*******************************************************************************
 * \addtogroup DemoOnline
 * \{
 * \file demo_online.cpp
 * \brief
 * \version 0.2
 * \date 2025-06-26
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-05-22 | Init version |
 * | 0.2 | 2025-06-26 | Add algorithm switch yaml file parse |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include "sensor_ipcclient.h"
#include "sync_queue.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>
#include "cpu_load.h"
#include "crc32.h"
#include "difop2.h"
#include "rs_lidar_sdk_api.h"
#include "rs_msop_parse.h"
#include "yaml_manager.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace std::chrono_literals;
using namespace dimw::sensor_access;
using namespace robosense::lidar;

/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/
typedef enum
{
    PCD_ASCII,  // ASCII格式
    PCD_BINARY, // 二进制格式
} PCDFormat;
/******************************************************************************/
/*            Declaration of exported variables or constant data              */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/
bool is_exit{false};
bool save_pcd{false};
bool test_stop{false};
bool save_raw_data{false};
std::string save_path{"./"};
int32_t test_num{0};
uint8_t save_count{7};
uint64_t file_counter{0};
std::shared_ptr<dimw::sensor_access::SensorIpcClient> lidar_client{nullptr};

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/
#define POINTCLOUD_NUM_PER_FRAME 291840

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
SyncQueue<LidarPointCloudPackets*> stuffed_cloud_queue_(500, [](LidarPointCloudPackets* ptr) {
    if (ptr)
        delete ptr;
});

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/

/*******************************************************************************
 * \brief Release a frame of camera data.
 *
* \param[in] sensor : Sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] kVoidPtrAdc : frame buffer address
 *                Range: 0-255. Accuracy: 1.
 * \return release status
 ******************************************************************************/
LidarSdkErrorCode releaseAdc(LidarSensorIndex sensor, const void* kVoidPtrAdc) {
    LidarSdkErrorCode ret_sdk_code{LidarSdkErrorCode::LIDAR_SDK_FAILD};
    LidarAdcBuffer* ptrAdc =
        static_cast<LidarAdcBuffer*>(const_cast<void*>(kVoidPtrAdc));
    if (ptrAdc != nullptr) {
        FrameBufObj* frame_obj =
            static_cast<FrameBufObj*>(const_cast<void*>(ptrAdc->reservedPtr));
        if (frame_obj != nullptr) {
            if (lidar_client) {
                auto ret = lidar_client->releaseFrame(*frame_obj);
                if (ret == ClientErrorCode::CLIENT_SUCCESS) {
                    ret_sdk_code = LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
                } else {
                    std::cout << "lidar client release frame failed: "
                              << static_cast<std::int32_t>(ret) << std::endl;
                }
            } else {
                std::cout << "lidar client is null" << std::endl;
            }
            delete frame_obj;
        }
        delete ptrAdc;
    }
    return ret_sdk_code;
}

/*******************************************************************************
 * \brief Get the radar synchronization start time.
 * \param[in] time : current time
 *                Range: 0 - 2^64-1. Accuracy: 1.
 * \return Synchronization status of sensor
 ******************************************************************************/
LidarSdkErrorCode getSensorFsyncStartTime(uint64_t* time) {
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

/*******************************************************************************
 * \brief Obtain the time via the low-level software interface.
 * \return system microsecond time
 ******************************************************************************/
uint64_t getTimeNowPhc() {
    struct timeval tv;//实际根据底软接口获取时间，demo使用系统微妙时间
    if (gettimeofday(&tv, nullptr) == -1) {
        std::cout << "gettimeofday failed" << std::endl;
        return 0;
    }
    return ((tv.tv_sec * 1000) * 1000) + tv.tv_usec;
}

/*******************************************************************************
 * \brief Obtain current time via the low-level software interface.
 * \return system microsecond time
 ******************************************************************************/
uint64_t getTimeNowSys() {
    struct timeval tv;// 实际根据底软接口获取时间，demo使用系统微妙时间
    if (gettimeofday(&tv, nullptr) == -1) {
        std::cout << "gettimeofday failed" << std::endl;
        return 0;
    }
    return ((tv.tv_sec * 1000) * 1000) + tv.tv_usec;
}

/*******************************************************************************
 * \brief Obtain current time via the low-level software interface.
 * \return system microsecond time
 ******************************************************************************/
uint64_t getTimeNowMgr() {
    struct timeval tv;//实际根据底软接口获取时间，demo使用系统微妙时间
    if (gettimeofday(&tv, nullptr) == -1) {
        std::cout << "gettimeofday failed" << std::endl;
        return 0;
    }
    return ((tv.tv_sec * 1000) * 1000) + tv.tv_usec;
}

/*******************************************************************************
 * \brief Save the point cloud to a PCD file, supporting both ASCII and binary formats.
 *
 * \param[in] kFileName : The name of the saved file
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] cloud : Point Cloud data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] format : saved format（ASCII或BINARY）
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return Saved status.
 ******************************************************************************/
# if 0
bool savePointCloudToPCD(const std::string& kFileName, LidarPointCloudPackets* cloud, PCDFormat format) {
    if (((!cloud) || (!cloud->point)) || (cloud->point_num == 0)) {
        std::cerr << "Invalid point cloud data" << std::endl;
        return false;
    }
    std::ofstream os;

    // 根据格式打开文件，二进制模式需要指定ios::binary
    if (format == PCDFormat::PCD_BINARY) {
        os.open(kFileName, std::ios::out | std::ios::trunc | std::ios::binary);
    } else {
        os.open(kFileName, std::ios::out | std::ios::trunc);
    }

    if (!os.is_open()) {
        std::cerr << "Failed to open file: " << kFileName << std::endl;
        return false;
    }

    os << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
    os << "VERSION 0.7" << std::endl;
    // 注意：字段顺序需与写入顺序一致
    os << "FIELDS x y z  intensity channel_number timestamp" << std::endl;
    // SIZE：每个字段的字节数（严格对应结构体成员）
    os << "SIZE 4 4 4 1 1 2" << std::endl;  // confidence是2字节数组
    // TYPE：字段数据类型（F=float, I=int16, U=uint8, U=uint8, U=uint8[2]）
    os << "TYPE F F F U U I" << std::endl;
    os << "COUNT 1 1 1 1 1 1" << std::endl;  // confidence整体作为一个字段
    os << "WIDTH " << cloud->point_num << std::endl;
    os << "HEIGHT 1" << std::endl;
    os << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    os << "POINTS " << cloud->point_num << std::endl;

    if (format == PCDFormat::PCD_ASCII) {
        os << "DATA ascii" << std::endl;
        for (uint32_t i = 0; i < cloud->point_num; ++i) {
            const LidarPoint* kP = &cloud->point[i];
            os << kP->x << " "
            << kP->y << " "
            << kP->z << " "
            << static_cast<int>(kP->intensity) << " "       // 反射率转int输出
            << static_cast<int>(kP->channel_number) << " "  // uint8_t转int避免被解析为字符
            << kP->timestamp << std::endl;
        }
    } else {
        os << "DATA binary" << std::endl;
        for (uint32_t i = 0; i < cloud->point_num; ++i) {
            const LidarPoint* kP = &cloud->point[i];

            // 严格按结构体成员顺序写入，不做类型转换
            os.write(reinterpret_cast<const char*>(&kP->x), sizeof(kP->x));               // float (4字节)
            os.write(reinterpret_cast<const char*>(&kP->y), sizeof(kP->y));               // float (4字节)
            os.write(reinterpret_cast<const char*>(&kP->z), sizeof(kP->z));               // float (4字节)
            os.write(reinterpret_cast<const char*>(&kP->intensity), sizeof(kP->intensity)); // uint8_t (1字节，反射率)
            os.write(reinterpret_cast<const char*>(&kP->channel_number), sizeof(kP->channel_number)); // uint8_t (1字节)
            os.write(reinterpret_cast<const char*>(&kP->timestamp), sizeof(kP->timestamp)); // int16_t (2字节)
        }
    }
    os.close();
    std::cout << "Save point cloud as: " << kFileName << std::endl;

    return true;
}
#endif

/*******************************************************************************
 * \brief Get the timestamp of the current time
 * \return the timestamp of the current time
 ******************************************************************************/
std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

    // 格式化时间戳
    std::tm* local_time = std::localtime(&now_time);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y%m%d_%H%M%S");
    return oss.str() + "_" + std::to_string(now_ms.count());
}


/*******************************************************************************
 * \brief Save the raw data to a bin file.
 * \param[in] kMipiData : raw data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] length : data length
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] fileCounter : file counter
 *                Range: -2^31 - 2^31-1. Accuracy: 1.
 ******************************************************************************/
void saveRawData(const void* kMipiData, uint32_t length, int32_t fileCounter) {
    std::string filename = save_path + "pc_bin/lidar_data_" +
                            std::to_string(fileCounter) + "_" +
                            std::to_string(save_count) + ".bin";

    std::ofstream outFile(filename, std::ios::binary);
    if (outFile.is_open()) {
        outFile.write(reinterpret_cast<const char*>(kMipiData), length);
        outFile.close();
        std::cout << "Saved data to: " << filename << ", (size: " << length
                  << " bytes) " << std::endl;
    } else {
        std::cout << "Failed to open file: " << filename << std::endl;
    }
}

/*******************************************************************************
 * \brief Point cloud process
 ******************************************************************************/
void processPointCloud(void) {
    int32_t frame_seq{0};
    while (!is_exit) {
            using namespace robosense::msop;
            std::string fileName = "./pcd_files/" + std::to_string(lidar_cloud->frame_timestamp) + "_" +
                                    std::to_string(lidar_cloud->frame_seq) + ".pcd";

            int ret = parseDifopPkt((const uint8_t*)lidar_cloud->lidar_parameter, lidar_cloud->lidar_parameter_length);
            if (!ret) {
                std::cout << "parseDifop2 failed" << std::endl;
            }

            static const uint16_t kBlockPerFrame {1520U};
            size_t msop_size = sizeof(LidarPointCloudPackets) + (kBlockPerFrame - 1) * sizeof(DataBlock);
            ret = parseMsopPkt((const uint8_t*)lidar_cloud, msop_size);
            if (!ret) {
                std::cout << "parseMsopPkt failed" << std::endl;
            }
            
            static constexpr uint32_t kPointsPerFrame {192U * 1520U};
            size_t requiredSize = sizeof(LidarPointCloud) + (kPointsPerFrame - 1) * sizeof(LidarPoint);
            uint8_t* point_cloud_data = static_cast<uint8_t*>(malloc(requiredSize));
            size_t point_cloud_size {0};

            getPointCloud(point_cloud_data, point_cloud_size);

            savePointCloudToPCD(fileName, (LidarPointCloud*)point_cloud_data, robosense::msop::PCD_ASCII);
            
            frame_seq++;
            free(lidar_cloud->lidar_parameter);
            free(lidar_cloud);
            lidar_cloud = nullptr;
    }
}

/*******************************************************************************
 * \brief Callback of point cloud process and verify point cloud data.
 * \param[in] sensor : Sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] kBuffer : Point cloud data
 *                Range:1-2^32 -1. Accuracy: 1.
 * \param[in] length : length of point cloud data
 *                Range:1-2^32 -1. Accuracy: 1.
 * \return Data validity status.
 ******************************************************************************/
LidarSdkErrorCode pointCloudCallback(LidarSensorIndex sensor,
                                     const void* kBuffer,
                                     uint32_t length) {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    // 检查输入参数
    if ((!kBuffer) || (length == 0)) {
        std::cout << "Error: Invalid buffer or length in pointCloudCallback"
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }

    // 将 buffer 转换为 LidarPointCloudPackets 指针
    const LidarPointCloudPackets* kLidarCloud =
        static_cast<const LidarPointCloudPackets*>(kBuffer);
    // 计算实际的点云数据大小
    size_t expected_size = kLidarCloud->data_length;

    // 验证数据长度是否符合预期
    if (length < expected_size) {
        std::cout << "pointCloudCallback Error: Buffer length is smaller than "
                     "expected, length = "
                  << length << ", expected size = " << expected_size
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }
    // uint32_t cnt{0};
    // for (uint32_t i = 0; i < kLidarCloud->point_num; ++i) {
    //     const LidarPoint* kPoint = &kLidarCloud->point[i];
    //     if (std::isnan(kPoint->x)) {
    //         cnt++;
    //     }
    // }
    static uint64_t last_recv_frame_timestamp{0};
    static bool first_recv{true};

    uint64_t current_recv_frame_timestamp = getTimeNowPhc();
    uint64_t recv_time_interval =
        current_recv_frame_timestamp - last_recv_frame_timestamp;

    static uint64_t last_frame_timestamp{0};
    uint64_t frame_time_interval =
        kLidarCloud->frame_timestamp - last_frame_timestamp;

    if(first_recv)
    {
        first_recv = false;
        recv_time_interval = 100000;
        frame_time_interval = 100000;
    }
    float recv_fps{1000000.0 / recv_time_interval};
    float frame_fps{1000000.0 / frame_time_interval};
    static uint64_t max_interval{102000};
    static uint64_t min_interval{98000};
    static uint32_t last_fram_seq{0};
    if (((frame_time_interval > (max_interval)) ||
        (frame_time_interval < min_interval)) ||
        ((kLidarCloud->frame_seq % 1) == 0)) {
        std::cout << "[LidarPointCloudPackets Info :" << kLidarCloud->frame_seq << "]: "
                  << ", point_num:" << kLidarCloud->point_num
                  << ", frame_seq:" << kLidarCloud->frame_seq
                  << ", difop2_len:" << kLidarCloud->lidar_parameter_length
                  << ", protocol_version: " << kLidarCloud->protocol_version
                  << ", return_mode:" << static_cast<uint32_t>(kLidarCloud->return_mode)
                  << ", sync_status:" << static_cast<uint32_t>(kLidarCloud->sync_status)
                  << ", frame_sync:" << static_cast<uint32_t>(kLidarCloud->frame_sync)
                  << ", frame_timestamp:" << kLidarCloud->frame_timestamp << " us"
                  << ", last_fram_seq:" << last_fram_seq
                  << ", last_frame_timestamp:" << last_frame_timestamp << " us"
                  << ", last_recv_frame_timestamp:" << last_recv_frame_timestamp << " us"
                  << ", mirror_id:" << static_cast<uint32_t>(kLidarCloud->mirror_id)
                  << ", frame_time_interval：" << frame_time_interval << " us"
                  << ", frame_fps:" << frame_fps << " fps"
                  << ", recv_time_interval:" << recv_time_interval << " us"
                  << ", recv_fps:" << recv_fps << " fps"
                  << ", counter:" << static_cast<uint32_t>(kLidarCloud->counter)
                  << std::endl;

        // 输出预留字段（十六进制，每个字节占2位，不足补0）
        std::cout << "  Reserved: ";
        for (size_t i = 0; i < sizeof(kLidarCloud->reserved); ++i) {
            // 设置十六进制输出、 uppercase（可选，使A-F大写）、补零（宽度2位）
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int32_t>(kLidarCloud->reserved[i])
                      << std::dec << " "; // 恢复十进制输出，避免后续输出受影响
        }
        std::cout << std::endl << std::endl;
    }
    
    last_fram_seq = kLidarCloud->frame_seq;
    last_recv_frame_timestamp = current_recv_frame_timestamp;
    last_frame_timestamp = kLidarCloud->frame_timestamp;
    
    if (!save_pcd) {
        return LIDAR_SDK_SUCCESS;
    }
    
    size_t requiredSize = kLidarCloud->data_length;
    LidarPointCloudPackets* msg = static_cast<LidarPointCloudPackets*>(malloc(requiredSize));

    void* difop2_data = malloc(kLidarCloud->lidar_parameter_length);
    std::memcpy(difop2_data, kLidarCloud->lidar_parameter, kLidarCloud->lidar_parameter_length);

    std::memcpy(msg, kLidarCloud, requiredSize);
    msg->lidar_parameter = difop2_data;

    bool is_overwritten{false};
    size_t sz = stuffed_cloud_queue_.push(msg, is_overwritten);
    if (is_overwritten) {
        std::cout << "stuffed_cloud_queue_ is full, drop the oldest one sz:"
                  << sz << std::endl;
    }

    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

/*******************************************************************************
 * \brief Print radar equipment information.
 * \param[in] kInfo : device information
 *                Range: 0 - 2^32-1. Accuracy: 1.
 ******************************************************************************/
void printLidarDeviceInfo(const LidarDeviceInfo& kInfo) {
    std::cout << "========== Lidar Device Information ==========" << std::endl;

    // 打印固件版本号
    uint16_t firmware_version{kInfo.firware_version}; // 注：原拼写"firware"建议修正为"firmware"
    int32_t firmware_major{firmware_version / 10000};
    int32_t firmware_minor{(firmware_version % 10000) / 100};
    int32_t firmware_patch{firmware_version % 100};
    std::cout << "Firmware Version: " << firmware_major << "." << std::setw(2)
              << std::setfill('0') << firmware_minor << "." << std::setw(2)
              << std::setfill('0') << firmware_patch << ", raw: 0x" << std::hex
              << firmware_version << ", dec: " << std::dec << firmware_version
              << std::endl;

    // 打印SDK版本号
    uint16_t sdk_version{kInfo.sdk_version};
    int32_t major{sdk_version / 10000};
    int32_t minor{(sdk_version % 10000) / 100};
    int32_t patch{sdk_version % 100};
    std::cout << "SDK Version: " << major << "." << std::setw(2)
              << std::setfill('0') << minor << "." << std::setw(2)
              << std::setfill('0') << patch << ", raw: 0x" << std::hex
              << sdk_version << ", dec: " << std::dec << sdk_version
              << std::endl;

    // 打印电机转速
    std::cout << "Motor Speed: " << kInfo.motor_speed << " rpm" << std::endl;

    // 打印回波模式
    const std::string kReturnModes[4]{"Dual Return", "Strongest Return",
                                   "Last Return", "First Return"};
    std::cout << "Return Mode: " << kReturnModes[kInfo.return_mode] << " ("
              << static_cast<int32_t>(kInfo.return_mode) << ")" << std::endl;

    // 打印时间戳
    std::cout << "Timestamp: " << kInfo.timestamp << " μs" << std::endl;

    // 打印运行状态
    std::cout << "Operate State: "
              << (kInfo.lidar_operation_state == 0x00 ? "Normal" : "Fault")
              << " (0x" << std::hex
              << static_cast<int32_t>(kInfo.lidar_operation_state) << ")"
              << std::dec << std::endl;

    // 打印故障状态
    const std::string kFaultStates[3]{"No Fault", "Level 1 Fault",
                                   "Level 2 Fault"};
    std::cout << "Fault State: " << kFaultStates[kInfo.lidar_fault_state]
              << " (0x" << std::hex
              << static_cast<int32_t>(kInfo.lidar_fault_state) << ")" << std::dec
              << std::endl;

    // 打印故障码信息
    std::cout << "SDK Fault Code Count: "
              << static_cast<int32_t>(kInfo.sdk_total_fault_number) << std::endl;
    std::cout << "SDK Fault Code Position: 0x" << std::hex
              << kInfo.sdk_fault_code_position << std::dec << std::endl;
    std::cout << "Active SDK Faults: " << std::endl;
    for (size_t i = 0; i < 64; ++i) {
        if (kInfo.sdk_fault_code_position & (uint64_t(1) << i)) {
            std::cout << "  [" << static_cast<int32_t>(i) << "]" << std::endl;
        }
    }

    // 打印供应商内部故障信息
    std::cout << std::endl << "Supplier Internal Faults:" << std::endl;
    std::cout << "  Fault ID: 0x" << std::hex
              << static_cast<int32_t>(kInfo.supplier_internal_fault_id)
              << std::dec << std::endl;
    std::cout << "  Fault Indicate: ";
    for (size_t i = 0; i < sizeof(kInfo.supplier_internal_fault_indicate); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(
                         kInfo.supplier_internal_fault_indicate[i])
                  << std::dec << " ";
    }
    std::cout << std::endl;
    std::cout << "  Fault Number: "
              << static_cast<int32_t>(kInfo.supplier_internal_fault_number)
              << std::endl;
    std::cout << "  Fault Position: "
              << static_cast<int32_t>(kInfo.supplier_internal_fault_position)
              << std::endl;

    // 打印时间同步信息
    std::cout << std::endl << "Time Synchronization:" << std::endl;
    std::cout << "  Sync Mode: 0x" << std::hex
              << static_cast<int32_t>(kInfo.time_sync_mode) << std::dec
              << std::endl;
    std::cout << "  Sync Status: "
              << (kInfo.time_sync_status == 0x01 ? "Succeeded" : "Failed")
              << " (0x" << std::hex
              << static_cast<int32_t>(kInfo.time_sync_status) << ")" << std::dec
              << std::endl;
    std::cout << "  Time Offset: " << kInfo.time_offset << " μs" << std::endl;

    // 打印产品序列号（十六进制字符串）
    std::cout << "Product SN: ";
    for (size_t i = 0; i < sizeof(kInfo.lidar_product_sn); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(
                         static_cast<uint8_t>(kInfo.lidar_product_sn[i]));
    }
    std::cout << std::dec << std::endl; // 恢复十进制输出

    // 打印制造商和型号
    std::cout << "Manufacturer: 0x" << std::hex
              << static_cast<int32_t>(kInfo.manufacture) << std::dec
              << std::endl;
    std::cout << "Model: 0x" << std::hex << static_cast<int32_t>(kInfo.model)
              << std::dec << std::endl;

    // 打印预留字段（每10个字节换行）
    std::cout << "Reserved: " << std::endl;
    for (size_t i = 0; i < sizeof(kInfo.reserved); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(kInfo.reserved[i]) << std::dec << " ";
        if ((i + 1) % 10 == 0) { // 每10个字节换行
            std::cout << std::endl;
        }
    }
    std::cout << std::endl; // 结束预留字段输出

    std::cout << "=============================================" << std::endl;
}

/*******************************************************************************
 * \brief Callback of ridar device information.
 * \param[in] sensor : Sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] kBuffer : Point cloud data
 *                Range:1-2^32 -1. Accuracy: 1.
 * \param[in] length : length of point cloud data
 *                Range:1-2^32 -1. Accuracy: 1.
 * \return error code of lidar sdk
 ******************************************************************************/
LidarSdkErrorCode deviceInfoCallback(LidarSensorIndex sensor,
                                     const void* kBuffer,
                                     uint32_t length) {
    static int32_t device_frame_seq{0};

    if ((!kBuffer) || (length == 0)) {
        std::cout << "Error: Invalid buffer or length in deviceInfoCallback"
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }

    // 将 buffer 转换为 LidarDeviceInfo 指针
    const LidarDeviceInfo* kDeviceInfo{
        static_cast<const LidarDeviceInfo*>(kBuffer)};

    // 计算实际的数据大小
    size_t expected_size{sizeof(LidarDeviceInfo)};

    // 验证数据长度是否符合预期
    if (length < expected_size) {
        std::cout << "deviceInfoCallback Error: Buffer length is smaller than "
                     "expected, length = "
                  << length << ", expected size = " << expected_size
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }

    if ((kDeviceInfo->sdk_total_fault_number > 0) &&
        (kDeviceInfo->sdk_fault_code_position != 1)) {
        printLidarDeviceInfo(*kDeviceInfo);
    }
    if ((device_frame_seq % 600) == 0) {
        std::cout << "device_frame_seq = " << device_frame_seq << std::endl;
        printLidarDeviceInfo(*kDeviceInfo);
    }
    device_frame_seq++;
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

/*******************************************************************************
 * \brief Write value to the uint16_t register
 * \param[in] sensor : sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] address : register address
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] data :The value to be written into the register
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \return status of write sensor Register
 ******************************************************************************/
LidarSdkErrorCode writeSensorRegUint16(LidarSensorIndex sensor, uint16_t address, uint16_t data) {
    if (!lidar_client) {
        std::cout << "writeSensorRegUint16 error: lidar_client is nullptr"
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "writeSensorRegUint16: address: " << address
              << ", data: " << data << std::endl;
    ClientErrorCode clientRet = lidar_client->writeSensorRegUint16(
        SensorType::MIDDLE_LIDAR, address, data);
    std::cout << "writeSensorRegUint16: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/*******************************************************************************
 * \brief  Read the value of the uint16_t register.
* \param[in] sensor : sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] address : register address
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] data :The value of read register
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \return status of read sensor Register
 ******************************************************************************/
LidarSdkErrorCode readSensorRegUint16(LidarSensorIndex sensor, uint16_t address, uint16_t* data) {
    if ((!lidar_client) || (!data)) {
        std::cout << "readSensorRegUint16 error: lidar_client is nullptr"
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "readSensorRegUint16: address: " << address
              << ", data: " << *data << std::endl;
    ClientErrorCode clientRet = lidar_client->readSensorRegUint16(
        SensorType::MIDDLE_LIDAR, address, *data);
    std::cout << "readSensorRegUint16: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/*******************************************************************************
 * \brief Write value to the uint8_t register
 * \param[in] sensor : sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] address : register address
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] data :The value to be written into the register
 *                Range: 0 - 2^8-1. Accuracy: 1.
 * \return status of write sensor Register
 ******************************************************************************/
LidarSdkErrorCode writeSensorRegUint8(LidarSensorIndex sensor, uint16_t address, uint8_t data) {
    if (!lidar_client) {
        std::cout << "[writeSensorRegUint8]  lidar_client is nullptr "
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "writeSensorRegUint8: address: " << address
              << ", data: " << data << std::endl;
    ClientErrorCode clientRet = lidar_client->writeSensorRegUint8(
        SensorType::MIDDLE_LIDAR, address, data);
    std::cout << "writeSensorRegUint8: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/*******************************************************************************
 * \brief  Read the value of the uint8_t register.
* \param[in] sensor : sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] address : register address
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] data :The value of read register
 *                Range: 0 - 2^8-1. Accuracy: 1.
 * \return status of read sensor Register
 ******************************************************************************/
LidarSdkErrorCode readSensorRegUint8(LidarSensorIndex sensor, uint16_t address, uint8_t* data) {
    if ((!lidar_client) || (!data)) {
        std::cout << "readSensorRegUint8 error: lidar_client is nullptr "
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "readSensorRegUint8: address: " << address
              << ", data: " << *data << std::endl;
    ClientErrorCode clientRet = lidar_client->readSensorRegUint8(
        SensorType::MIDDLE_LIDAR, address, *data);
    std::cout << "readSensorRegUint8: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/*******************************************************************************
 * \brief  Reading radar data via I2C protocol.
* \param[in] sensor : sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] address : protocol address
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] data :The value of I2C protocol
 *                Range: 0 - 2^8-1. Accuracy: 1.
 * \return status of read I2C protocol
 ******************************************************************************/
LidarSdkErrorCode readSensorI2C(LidarSensorIndex sensor,
                                uint16_t address,
                                uint8_t* data,
                                uint16_t* length) {
    if (((!data) || (!length)) || (!lidar_client)) {
        std::cout << "readSensorI2C error: lidar_client is nullptr "
                  << std::endl;
        return LidarSdkErrorCode::
            LIDAR_SDK_FAILD; // 数据缓冲区和长度指针不可为空
    }

    std::cout << "readSensorI2C: address: " << address
              << ", expected length: " << *length << std::endl;
    // 模拟数据填充（仅用于示例）
    for (int32_t i = 0; i < *length; i++) {
        data[i] = i;
    }

    ClientErrorCode clientRet = lidar_client->readSensorI2C(
        SensorType::MIDDLE_LIDAR, address, data, *length);
    std::cout << "readSensorI2C: clientRet: " << static_cast<int32_t>(clientRet)
              << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/*******************************************************************************
 * \brief  Writing radar data via I2C protocol.
* \param[in] sensor : sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] address : I2C protocol address
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] kData :The value of I2C protocol
 *                Range: 0 - 2^8-1. Accuracy: 1.
 * \return status of read I2C protocol
 ******************************************************************************/
LidarSdkErrorCode writeSensorI2C(LidarSensorIndex sensor,
                                 uint16_t address,
                                 const uint8_t* kData,
                                 uint16_t length) {
    if ((!kData) || (!lidar_client)) {
        std::cout << "[writeSensorI2C] Invalid input parameter or "
                     "lidar_client is nullptr "
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }

    ClientErrorCode clientRet = lidar_client->writeSensorI2C(
        SensorType::MIDDLE_LIDAR, address, kData, length);
    std::cout << "writeSensorI2C: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

/*******************************************************************************
 * \brief Simulate fetching data via a data interface that has been obtained from the underlying software.
 * \return Initialized state.
 ******************************************************************************/
bool initSensorIpcClient() {
    lidar_client = std::make_shared<dimw::sensor_access::SensorIpcClient>(
        dimw::sensor_access::AppType::SEN_IRC_LIDAR_CONSUMER);
    uint32_t mask = 0b100000000000000;
    ClientErrorCode ret = lidar_client->init(mask);

    if (ret != ClientErrorCode::CLIENT_SUCCESS) {
        std::cout << "lidarClient init failed, error code is: "
                  << static_cast<std::int32_t>(ret);
        uint32_t linkMask, videoMask;
        auto ret = lidar_client->getSensorStatus(linkMask, videoMask);
        std::cout << "getSensorStatus: "
                  << std::to_string(static_cast<std::int32_t>(ret))
                  << ", linkMask: " << (int)linkMask
                  << ", videoMask: " << (int)videoMask;
        return false;
    }

    ret = lidar_client->start();
    if (ret != ClientErrorCode::CLIENT_SUCCESS) {
        std::cout << "lidarClient start failed, error code is: "
                  << std::to_string(static_cast<std::int32_t>(ret));
        return false;
    }
    return true;
}

/*******************************************************************************
 * \brief Retrieve data from the OS interface.
 * \param[in] lidar_interface : lidar interface
 *                Range: 0 - 2^32-1. Accuracy: 1.
 ******************************************************************************/
void getDataFromOsApi(LidarSdkInterface* lidar_interface) {
    int32_t raw_data_cnt{0};
    bool recv_first_packet{false};
    int32_t mipi_frame_cnt{0};
    while (!is_exit) {
        FrameBufObj* frame = new FrameBufObj();
        auto ret = lidar_client->getFrame(SensorType::MIDDLE_LIDAR, *frame);
        if (ret != ClientErrorCode::CLIENT_SUCCESS) {
            std::cout << "lidar_client getFrame failed, error code is: "
                      << static_cast<std::int32_t>(ret) << std::endl;
            uint32_t linkMask{0};
            uint32_t videoMask{0};
            ret = lidar_client->getSensorStatus(linkMask, videoMask);
            if (ret != ClientErrorCode::CLIENT_SUCCESS) {
                std::cout
                    << "lidarClient get sensor status failed, error code is: "
                    << static_cast<std::int32_t>(ret);
            } else {
                std::cout << "getSensorStatus: "
                          << ", linkMask: " << (int)linkMask
                          << ", videoMask: " << (int)videoMask;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        LidarAdcBuffer* buf = new LidarAdcBuffer();
        buf->sensor = frame->sensor;
        buf->frameIndex = frame->frameIndex;
        buf->teof = frame->teof;
        buf->tsof = frame->tsof;
        buf->bufObj = frame->bufObj;
        buf->data = frame->data;
        buf->len = frame->size;
        buf->reservedPtr = frame;

        // CRC test
        if (yaml::demo_test_param.data_valid && yaml::demo_test_param.enable_mipi_crc_check) {
            uint8_t* lidar_data = frame->data;
            size_t adc_data_len = frame->size;
            uint32_t calculate_crc = crc32::calculate_crc32(lidar_data, adc_data_len - 4);
            uint32_t expected_crc = lidar_data[adc_data_len - 1] << 24 |
                                    lidar_data[adc_data_len - 2] << 16 |
                                    lidar_data[adc_data_len - 3] << 8 |
                                    lidar_data[adc_data_len - 4];
            static uint32_t right_cnt{0};
            static uint32_t error_cnt{0};

            if (calculate_crc != expected_crc) {
                ++error_cnt;
            } else {
                ++right_cnt;
            }

            if (0 == ((right_cnt + error_cnt) % 600)) {
                std::cout << std::dec << "right count: " << right_cnt << ", error count: " << error_cnt <<
                                        ", error rate: " << static_cast<double>(error_cnt) /
                                        static_cast<double>(error_cnt + right_cnt) << std::endl;
                std::cout << std::endl;
            }
        }

        if (yaml::demo_test_param.data_valid && yaml::demo_test_param.enable_print_difop2) {
            uint8_t* difop_data = frame->data + utils::kMsopLen;
            std::memcpy(utils::difop2_mipi_data.data(), difop_data, utils::kDifopLen);
        }

        // Save point cloud frame, when the CPU usage of algorithm is overloaded.
        uint16_t msop_pkt_seq = (frame->data[4] << 8) | frame->data[5];

        if ((utils::g_save_bin && (msop_pkt_seq == 1)) && (save_count == 7)) {
            save_count = 1;
            file_counter = utils::fileCounter;
            utils::fileCounter++;
        }
        if (save_count < 7) {
            saveRawData(buf->data, buf->len, file_counter);
            save_count++;
        }

        mipi_frame_cnt++;

        if (save_raw_data && (mipi_frame_cnt > 60)) {
            uint16_t msop_pkt_seq = (frame->data[4] << 8) | frame->data[5];
            if (msop_pkt_seq == 1) {
                recv_first_packet = true;
            }
            if ((test_num > raw_data_cnt) && recv_first_packet) {
                raw_data_cnt++;
                uint64_t start_time = getTimeNowPhc();
                saveRawData(buf->data, buf->len, raw_data_cnt);
                uint64_t cost_time = getTimeNowPhc() - start_time;
                std::cout << "cost time: " << cost_time
                          << " ,mipi_frame_cnt:" << mipi_frame_cnt
                          << ", msop_pkt_seq:" << msop_pkt_seq << std::endl;
            }
        }

        LidarSdkErrorCode inject_ret =
            lidar_interface->injectAdc(LidarSensorIndex::MIDDLE_LIDAR, buf);
        if (inject_ret != LidarSdkErrorCode::LIDAR_SDK_SUCCESS) {
            releaseAdc(LidarSensorIndex::MIDDLE_LIDAR, buf);
        }
    }
}

/*******************************************************************************
 * \brief Print help information.
 ******************************************************************************/
void printHelp() {
    std::cout << "使用说明:\n";
    std::cout << "选项:\n";
    std::cout << "  -t, --test <测试模式> 进行stop接口测试。\n";
    std::cout << "  -p, --path <路径>   设置数据保存路径。\n";
    std::cout << "  -n, --num <数量>    设置数量。\n";
    std::cout << "  -s, --save <保存原始数据>    \n";
    std::cout << "  -d, --pcd <PCD文件名> 保存点云数据为PCD格式。\n";
    std::cout << "  -h, --help          显示此帮助信息。\n";
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/

/*******************************************************************************
 * \brief main function.
 * \param[in] argc : Number of input parameters
 *                Range: 0 - 2^31-1. Accuracy: 1.
 * \param[in] argc : data of input parameters
 *                Range: 0 - 2^8-1. Accuracy: 1.
 * \return status of running
 ******************************************************************************/
int32_t main(int32_t argc, char* argv[]) {
  robosense::lidar::utils::main_pid = getpid();
  std::cout << "==================== main tid:" << std::dec << robosense::lidar::utils::main_pid << std::endl;
    std::cout << "Compiled on " << __DATE__ << " at " << __TIME__ << std::endl;
    for (int32_t i = 1; i < argc; ++i) {
        std::string arg{argv[i]};

        if ((arg == "-h") || (arg == "--help")) {
            printHelp();
            return 0;
        } else if ((arg == "-t") || (arg == "--test")) {
            test_stop = true;
        } else if ((arg == "-p") || (arg == "--path")) {
            if ((i + 1) < argc) {
                save_path = argv[++i]; // 获取下一个参数作为路径
            } else {
                std::cout << "错误：-p/--path 选项需要指定路径参数"
                          << std::endl;
                printHelp();
                return 1;
            }
        } else if ((arg == "-n") || (arg == "--num")) {
            if (i + 1 < argc) {
                test_num = std::stoi(argv[++i]); // 获取下一个参数并转换为整数
                if (test_num <= 0) {
                    std::cout << "错误：-n/--num 选项需要指定大于0的次数参数"
                              << std::endl;
                    return 1;
                }
            } else {
                std::cout << "错误：-n/--num 选项需要指定次数参数" << std::endl;
                printHelp();
                return 1;
            }
        } else if ((arg == "-s") || (arg == "--save")) {
            save_raw_data = true;
        } else if ((arg == "-d") || (arg == "--pcd")) {
            save_pcd = true;
        }
    }
    if (!initSensorIpcClient()) {
        std::cout << "initSensorIpcClient failed" << std::endl;
        return -1;
    }
    // 获取SDK接口
    LidarSdkInterface* lidar_interface =
        robosense::lidar_sdk::getRSLidarSdkInterface(sizeof(LidarSdkInterface));
    if (nullptr == lidar_interface) {
        std::cout << "Failed to get SDK interface" << std::endl;
        return -1;
    }
    {
        LidarSdkCbks sdk_cbks;
        sdk_cbks.releaseAdc = releaseAdc;
        sdk_cbks.getSensorFsyncStartTime = getSensorFsyncStartTime;
        sdk_cbks.getTimeNowPhc = getTimeNowPhc;
        sdk_cbks.getTimeNowSys = getTimeNowSys;
        sdk_cbks.getTimeNowMgr = getTimeNowMgr;
        sdk_cbks.writeSensorRegUint16 = writeSensorRegUint16;
        sdk_cbks.readSensorRegUint16 = readSensorRegUint16;
        sdk_cbks.writeSensorRegUint8 = writeSensorRegUint8;
        sdk_cbks.readSensorRegUint8 = readSensorRegUint8;
        sdk_cbks.readSensorI2C = readSensorI2C;
        sdk_cbks.writeSensorI2C = writeSensorI2C;
        sdk_cbks.pointCloud = pointCloudCallback;
        sdk_cbks.deviceInfo = deviceInfoCallback;

        // 初始化SDK
        if (lidar_interface->init(&sdk_cbks, "ag_config.json") !=
            LIDAR_SDK_SUCCESS) {
            std::cout << "SDK initialization failed" << std::endl;
            return -1;
        }
    }
    uint32_t api_version{lidar_interface->apiVersion};
    std::cout << "API version: " << LIDAR_SDK_API_MAJOR_GET(api_version) << "."
              << LIDAR_SDK_API_MINOR_GET(api_version) << "."
              << LIDAR_SDK_API_PATCH_GET(api_version) << std::endl;
    const char* kVersion{lidar_interface->getLidarSdkVersion()};
    std::cout << "The sdk version is " << kVersion << std::endl;

    if (!save_pcd && yaml::demo_test_param.data_valid) {
        save_pcd = yaml::demo_test_param.enable_save_pcd;
        save_path = yaml::demo_test_param.save_path;
        std::cout << "save_pcd = " << save_pcd << std::endl;
        std::cout << "pcd_save_path_ = " << save_path << std::endl;
    }

    {
        using namespace robosense::msop;
        // 初始化msop_parser
        bool ret = initMsop();

        if (!ret) {
            std::cout << "initMsop failed" << std::endl;
            return -1;
        }
    }

    std::thread mock_thread([&]() {
        int32_t ret = pthread_setname_np(pthread_self(), "RS-DEMO-inject");
        if (ret != 0) {
            std::cout << "set inject thread name failed" << std::endl;
        }

        getDataFromOsApi(lidar_interface);
    });

    std::thread process_thread;
    if (save_pcd) {
        process_thread = std::thread([&]() {
            int32_t ret = pthread_setname_np(pthread_self(), "RS-DEMO-proc");
            if (ret != 0) {
                std::cout << "set process pointcloud thread name failed"
                          << std::endl;
            }
            processPointCloud();
        });
    }

    if (test_stop) {
        for (int32_t loop_cnt = 0; loop_cnt < test_num; loop_cnt++) {
            std::cout << "loop_cnt: " << loop_cnt << std::endl;
            // 启动SDK
            if (lidar_interface->start() != LIDAR_SDK_SUCCESS) {
                std::cout << "Failed to start SDK, loop_cnt: " << loop_cnt
                          << std::endl;
                lidar_interface->deInit();
                return -1;
            }
            std::cout << "Start sdk successfully, loop_cnt: " << loop_cnt
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));

            std::cout << "Stop sdk, loop_cnt: " << loop_cnt << std::endl;

            auto start = std::chrono::steady_clock::now();
            lidar_interface->stop();
            auto end = std::chrono::steady_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                      start);
            std::cout << "Stop cost time: " << duration.count()
                      << " us, loop_cnt: " << loop_cnt << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    } else {
        if (lidar_interface->start() != LIDAR_SDK_SUCCESS) {
            std::cout << "Failed to start SDK" << std::endl;
            lidar_interface->deInit();
            return -1;
        }

        uint8_t data[2]{0};
        uint16_t data_len{0};
        uint8_t nrc{0xFFU};

        lidar_interface->readDid(0x1112U, data, &data_len, &nrc);
        std::cout << "data: " << *(uint16_t*)data << ", data_len: " << data_len
                  << ", nrc: " << (uint16_t)nrc << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::string algo_yaml_path{"./algorithm_switch.yaml"};
        std::uint8_t interval{0};
        (void)yaml::readAlgoYaml(algo_yaml_path);

        std::thread cpu_usage_thread([&]() {
            pthread_setname_np(pthread_self(), "RS-CpuMonitor");
            utils::monitThreads();
        });

        std::thread difop2_thread([&]() {
            pthread_setname_np(pthread_self(), "RS-Difop2Monitor");
            utils::compareDifop2();
        });

        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if ((++interval) < 5) {
                continue;
            }
            interval = 0;
            (void)yaml::readAlgoYaml(algo_yaml_path);

            if (utils::algo_threshold != yaml::demo_test_param.cpu_threshold) {
                utils::algo_threshold = yaml::demo_test_param.cpu_threshold;
            }
        }
        lidar_interface->stop();
    }
    is_exit = true;
    lidar_client->stop();
    lidar_client->deInit();
    lidar_client = nullptr;
    std::cout << "lidar client deinit success" << std::endl;
    if (mock_thread.joinable()) {
        mock_thread.join();
    }
    std::cout << "mock_thread join success" << std::endl;
    lidar_interface->deInit();
    lidar_interface = nullptr;
    if (save_pcd) {
        stuffed_cloud_queue_.stopWait();

        stuffed_cloud_queue_.reset();
        stuffed_cloud_queue_.clear();

        if (process_thread.joinable()) {
            process_thread.join();
        }
    }
    std::cout << "SDK deinit success" << std::endl;

    return 0;
}
/* \}  DemoOnline */