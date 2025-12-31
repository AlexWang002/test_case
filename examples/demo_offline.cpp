/*******************************************************************************
 * \addtogroup DemoOffline
 * \{
 * \file demo_offline.cpp
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
#include "rs_lidar_sdk_api.h"
#include "sync_queue.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <deque>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdio.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace std::chrono_literals;
using namespace robosense::lidar;

/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/
struct FileInfo {
    std::string name;
    int index;
};
// 存储文件名和对应序号，比如 {"175256331648_2197_1.bin", 1}
typedef enum {
    PCD_ASCII,  // ASCII格式
    PCD_BINARY, // 二进制格式
} PCDFormat;

/******************************************************************************/
/*            Declaration of exported variables or constant data              */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/
std::string lidar_data_path {"../mipi_data/"};
bool is_exit {false};
bool test_stop {false};
bool save_pcd {false};
int32_t test_num {0};
std::vector<std::vector<char>> all_frames_data;

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/
#define POINTCLOUD_NUM_PER_FRAME 291840

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
SyncQueue<LidarPointCloudPackets*> stuffed_cloud_queue_(500, [](LidarPointCloudPackets* ptr) {
    if (ptr) {
        delete ptr;
    }
});

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/

/*******************************************************************************
 * \brief Extract the sequence number from the kFileName, assuming the format is xxx_xxx_number.bin.
* \param[in] kFileName : file name
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return number of file name
 ******************************************************************************/
int extractIndex(const std::string& kFileName) {
    std::regex re(R"(_(\d+)\.bin$)");
    std::smatch match;
    if (std::regex_search(kFileName, match, re) && (match.size() == 2)) {
        return std::stoi(match[1]);
    }
    return -1; // 异常情况返回 -1
}

/*******************************************************************************
 * \brief Use POSIX functions to obtain file information (filename + sequence number) of files in a directory that match a naming convention.
 * \param[in] kPath : file path
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return The collection of all files that match the naming convention
 ******************************************************************************/
std::vector<FileInfo> getSortedFileInfos(const std::string& kPath) {
    std::vector<FileInfo> infos;
    DIR* dir;
    struct dirent* entry;

    // 打开目录
    if ((dir = opendir(kPath.c_str())) == nullptr) {
        std::cerr << "无法打开目录: " << kPath << std::endl;
        return infos;
    }

    // 遍历目录中的文件
    while ((entry = readdir(dir)) != nullptr) {
        // 跳过目录本身和父目录
        if ((strcmp(entry->d_name, ".") == 0) || (strcmp(entry->d_name, "..") == 0)) {
            continue;
        }

        // 检查是否为普通文件
        if (entry->d_type == DT_REG) {
            std::string filename = entry->d_name;
            const int kIdx = extractIndex(filename);
            if (kIdx != -1) { // 符合命名规则才加入
                infos.push_back({filename, kIdx});
            }
        }
    }

    // 关闭目录
    closedir(dir);

    // 按序号升序排序
    std::sort(infos.begin(), infos.end(), [](const FileInfo& kA, const FileInfo& kB) { return kA.index < kB.index; });

    return infos;
}

/*******************************************************************************
 * \brief Release a frame of camera data.
* \param[in] sensor : Sensor index
 *                Range: 6/7/8. Accuracy: 1.
 * \param[in] kVoidPtrAdc : frame buffer address
 *                Range: 0-255. Accuracy: 1.
 * \return release status
 ******************************************************************************/
LidarSdkErrorCode releaseAdc(LidarSensorIndex sensor, const void* kVoidPtrAdc) {
    if (!kVoidPtrAdc) {
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    LidarAdcBuffer* buf = static_cast<LidarAdcBuffer*>(const_cast<void*>(kVoidPtrAdc));
    if (buf != nullptr) {
        delete buf;
    }
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
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
    struct timeval tv; //实际根据底软接口获取时间，demo使用系统微妙时间
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
    struct timeval tv; //实际根据底软接口获取时间，demo使用系统微妙时间
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
    struct timeval tv; //实际根据底软接口获取时间，demo使用系统微妙时间
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
#if 0
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
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    // 格式化时间戳
    std::tm* local_time = std::localtime(&now_time);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y%m%d_%H%M%S");
    return oss.str() + "_" + std::to_string(now_ms.count());
}

/*******************************************************************************
 * \brief Point cloud process
 ******************************************************************************/
void processPointCloud(void) {
    int32_t frame_seq {0};
    while (!is_exit) {
        LidarPointCloudPackets* lidar_cloud;
        const bool kState = stuffed_cloud_queue_.popWait(lidar_cloud, 1000000);
        if (!kState) {
            std::cout << "processPointCloud data timeout" << std::endl;
            continue;
        }
        if (lidar_cloud && save_pcd) {
            // uint32_t cnt{0};
            // for (uint32_t i = 0; i < lidar_cloud->point_num; ++i) {
            //     const LidarPoint* kPoint = &lidar_cloud->point[i];
            //     if (std::isnan(kPoint->x)) {
            //         cnt++;
            //     }
            // }
            // std::string fileName = lidar_data_path +"/" + std::to_string(lidar_cloud->frame_timestamp) + "_" +
            // std::to_string(lidar_cloud->frame_seq) + ".pcd";
            // savePointCloudToPCD(fileName, lidar_cloud, PCD_BINARY);
            frame_seq++;
            free(lidar_cloud);
            lidar_cloud = nullptr;
        }
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
LidarSdkErrorCode pointCloudCallback(LidarSensorIndex sensor, const void* kBuffer, uint32_t length) {
    // 检查输入参数
    if ((!kBuffer) || (length == 0)) {
        std::cout << "Error: Invalid buffer or length in pointCloudCallback" << std::endl;
        return LIDAR_SDK_FAILD;
    }

    // 将 buffer 转换为 LidarPointCloudPackets 指针
    const LidarPointCloudPackets* kLidarCloud = static_cast<const LidarPointCloudPackets*>(kBuffer);

    // 计算实际的点云数据大小
    // size_t expected_size = sizeof(LidarPointCloudPackets) +
    // ((kLidarCloud->point_num - 1) * sizeof(LidarPoint));
    size_t expected_size = sizeof(LidarPointCloudPackets) + (1520 - 1) * sizeof(DataBlock);

    // 验证数据长度是否符合预期
    if (length < expected_size) {
        std::cout << "pointCloudCallback Error: Buffer length is smaller than "
                     "expected, length = "
                  << length << ", expected size = " << expected_size << std::endl;
        return LIDAR_SDK_FAILD;
    }

    // uint32_t cnt{0};
    // for (uint32_t i = 0; i < kLidarCloud->point_num; ++i) {
    //     const LidarPoint* point{&kLidarCloud->point[i]};
    //     if (std::isnan(point->x)) {
    //         cnt++;
    //     }
    // }
    static uint64_t last_recv_frame_timestamp {0};
    uint64_t current_recv_frame_timestamp = getTimeNowPhc();
    uint64_t recv_time_interval = current_recv_frame_timestamp - last_recv_frame_timestamp;
    last_recv_frame_timestamp = current_recv_frame_timestamp;
    static uint64_t last_frame_timestamp {0};
    uint64_t frame_time_interval = kLidarCloud->frame_timestamp - last_frame_timestamp;
    last_frame_timestamp = kLidarCloud->frame_timestamp;
    {
        std::cout << "[LidarPointCloudPackets Info :" << kLidarCloud->frame_seq
                  << "]: protocol_version: " << kLidarCloud->protocol_version
                  << ", return_mode:" << static_cast<uint32_t>(kLidarCloud->return_mode)
                  << ", sync_status:" << static_cast<uint32_t>(kLidarCloud->sync_status)
                  << ", frame_sync:" << static_cast<uint32_t>(kLidarCloud->frame_sync)
                  << ", point_num:" << kLidarCloud->point_num << "，frame_seq:" << kLidarCloud->frame_seq
                  << ", frame_timestamp:" << kLidarCloud->frame_timestamp << " us"
                  << ",mirror_id:" << static_cast<uint32_t>(kLidarCloud->mirror_id) << ", frame_time_interval："
                  << frame_time_interval << " us"
                  << ", recv_time_interval:" << recv_time_interval
                  << " us"
                  //   << ", nan point num: " << cnt
                  //   << ", valid cnt:" << (kLidarCloud->point_num - cnt)
                  << std::endl;

        // 输出预留字段（十六进制，每个字节占2位，不足补0）
        std::cout << "  Reserved: ";
        for (size_t i = 0; i < sizeof(kLidarCloud->reserved); ++i) {
            // 设置十六进制输出、 uppercase（可选，使A-F大写）、补零（宽度2位）
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int32_t>(kLidarCloud->reserved[i])
                      << std::dec << " "; // 恢复十进制输出，避免后续输出受影响
        }
        std::cout << std::endl << std::endl;
    }
    if (!save_pcd) {
        return LIDAR_SDK_SUCCESS;
    }
    // uint32_t pointCount {POINTCLOUD_NUM_PER_FRAME};
    // size_t requiredSize = sizeof(LidarPointCloudPackets) + +((1520 - 1) * sizeof(DataBlock));

    // LidarPointCloudPackets* msg = static_cast<LidarPointCloudPackets*>(malloc(requiredSize));
    // // 复制LidarPointCloud结构体的基础成员
    // msg->protocol_version = kLidarCloud->protocol_version;
    // msg->return_mode = kLidarCloud->return_mode;
    // msg->sync_status = kLidarCloud->sync_status;
    // msg->frame_sync = kLidarCloud->frame_sync;
    // msg->point_num = kLidarCloud->point_num;
    // msg->frame_seq = kLidarCloud->frame_seq;
    // msg->frame_timestamp = kLidarCloud->frame_timestamp;

    // // // 复制reserved数组
    // std::memcpy(msg->reserved, kLidarCloud->reserved,
    //             sizeof(kLidarCloud->reserved));

    // // // 复制点云数据数组（关键部分）
    // if (msg->point_num > 0) {
    //     size_t pointDataSize = msg->point_num * sizeof(LidarPoint);
    //     std::memcpy(msg->point, kLidarCloud->point, pointDataSize);
    // }
    // bool is_overwritten{false};
    // size_t sz = stuffed_cloud_queue_.push(msg, is_overwritten);
    // if (is_overwritten) {
    //     std::cout << "stuffed_cloud_queue_ is full, drop the oldest one sz:"
    //               << sz << std::endl;
    // }
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
    uint16_t firmware_version = kInfo.firware_version; // 注：原拼写"firware"建议修正为"firmware"
    int32_t firmware_major = firmware_version / 10000;
    int32_t firmware_minor = (firmware_version % 10000) / 100;
    int32_t firmware_patch = firmware_version % 100;
    std::cout << "Firmware Version: " << firmware_major << "." << std::setw(2) << std::setfill('0') << firmware_minor
              << "." << std::setw(2) << std::setfill('0') << firmware_patch << ", raw: 0x" << std::hex
              << firmware_version << ", dec: " << std::dec << firmware_version << std::endl;

    // 打印SDK版本号
    uint16_t sdk_version = kInfo.sdk_version;
    int32_t major = sdk_version / 10000;
    int32_t minor = (sdk_version % 10000) / 100;
    int32_t patch = sdk_version % 100;
    std::cout << "SDK Version: " << major << "." << std::setw(2) << std::setfill('0') << minor << "." << std::setw(2)
              << std::setfill('0') << patch << ", raw: 0x" << std::hex << sdk_version << ", dec: " << std::dec
              << sdk_version << std::endl;

    // 打印电机转速
    std::cout << "Motor Speed: " << kInfo.motor_speed << " rpm" << std::endl;

    // 打印回波模式
    const std::string kReturnModes[4] {"Dual Return", "Strongest Return", "Last Return", "First Return"};
    std::cout << "Return Mode: " << kReturnModes[kInfo.return_mode] << " (" << static_cast<int32_t>(kInfo.return_mode)
              << ")" << std::endl;

    // 打印时间戳
    std::cout << "Timestamp: " << kInfo.timestamp << " μs" << std::endl;

    // 打印运行状态
    std::cout << "Operate State: " << (kInfo.lidar_operation_state == 0x00 ? "Normal" : "Fault") << " (0x" << std::hex
              << static_cast<int32_t>(kInfo.lidar_operation_state) << ")" << std::dec << std::endl;

    // 打印故障状态
    const std::string kFaultStates[3] {"No Fault", "Level 1 Fault", "Level 2 Fault"};
    std::cout << "Fault State: " << kFaultStates[kInfo.lidar_fault_state] << " (0x" << std::hex
              << static_cast<int32_t>(kInfo.lidar_fault_state) << ")" << std::dec << std::endl;

    // 打印故障码信息
    std::cout << "SDK Fault Code Count: " << static_cast<int32_t>(kInfo.sdk_total_fault_number) << std::endl;
    std::cout << "SDK Fault Code Position: 0x" << std::hex << kInfo.sdk_fault_code_position << std::dec << std::endl;
    std::cout << "Active SDK Faults: " << std::endl;
    for (size_t i = 0; i < 64; ++i) {
        if (kInfo.sdk_fault_code_position & (uint64_t(1) << i)) {
            std::cout << "  [" << static_cast<int32_t>(i) << "]" << std::endl;
        }
    }

    // 打印供应商内部故障信息
    std::cout << std::endl << "Supplier Internal Faults:" << std::endl;
    std::cout << "  Fault ID1: 0x" << std::hex << static_cast<int32_t>(kInfo.supplier_internal_fault_id) << std::dec
              << std::endl;
    std::cout << "  Fault Indicate1: ";
    for (size_t i = 0; i < 4; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(kInfo.supplier_internal_fault_indicate[i]) << std::dec << " ";
    }
    std::cout << std::endl;

    std::cout << "  Fault ID2: 0x";
    for (size_t i = 0; i < 2; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int32_t>(kInfo.reserved[i])
                  << std::dec << " ";
    }
    std::cout << std::endl;
    std::cout << "  Fault Indicate2: ";
    for (size_t i = 2; i < 6; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int32_t>(kInfo.reserved[i])
                  << std::dec << " ";
    }
    std::cout << std::endl;
    std::cout << "  Fault Number: " << static_cast<int32_t>(kInfo.reserved[6]) << std::endl;
    std::cout << "  Fault Position: ";
    for (size_t i = 7; i < 10; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int32_t>(kInfo.reserved[i])
                  << std::dec << " ";
    }
    std::cout << std::endl;

    // 打印时间同步信息
    std::cout << std::endl << "Time Synchronization:" << std::endl;
    std::cout << "  Sync Mode: 0x" << std::hex << static_cast<int32_t>(kInfo.time_sync_mode) << std::dec << std::endl;
    std::cout << "  Sync Status: " << (kInfo.time_sync_status == 0x01 ? "Succeeded" : "Failed") << " (0x" << std::hex
              << static_cast<int32_t>(kInfo.time_sync_status) << ")" << std::dec << std::endl;
    std::cout << "  Time Offset: " << kInfo.time_offset << " μs" << std::endl;

    // 打印产品序列号（十六进制字符串）
    std::cout << "Product SN: ";
    for (size_t i = 0; i < sizeof(kInfo.lidar_product_sn); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(static_cast<uint8_t>(kInfo.lidar_product_sn[i]));
    }
    std::cout << std::dec << std::endl; // 恢复十进制输出

    // 打印制造商和型号
    std::cout << "Manufacturer: 0x" << std::hex << static_cast<int32_t>(kInfo.manufacture) << std::dec << std::endl;
    std::cout << "Model: 0x" << std::hex << static_cast<int32_t>(kInfo.model) << std::dec << std::endl;

    // 打印预留字段（每10个字节换行）
    std::cout << "Reserved: " << std::endl;
    for (size_t i = 10; i < sizeof(kInfo.reserved); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int32_t>(kInfo.reserved[i])
                  << std::dec << " ";
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
LidarSdkErrorCode deviceInfoCallback(LidarSensorIndex sensor, const void* kBuffer, uint32_t length) {
    static int32_t device_frame_seq {0};

    if ((!kBuffer) || (length == 0)) {
        std::cout << "Error: Invalid buffer or length in deviceInfoCallback" << std::endl;
        return LIDAR_SDK_FAILD;
    }

    // 将 buffer 转换为 LidarDeviceInfo 指针
    const LidarDeviceInfo* kDeviceInfo = static_cast<const LidarDeviceInfo*>(kBuffer);

    // 计算实际的数据大小
    size_t expected_size = sizeof(LidarDeviceInfo);

    // 验证数据长度是否符合预期
    if (length < expected_size) {
        std::cout << "deviceInfoCallback Error: Buffer length is smaller than "
                     "expected, length = "
                  << length << ", expected size = " << expected_size << std::endl;
        return LIDAR_SDK_FAILD;
    }

    if ((device_frame_seq % 600) == 0) {
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
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
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
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
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
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
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
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
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
LidarSdkErrorCode readSensorI2C(LidarSensorIndex sensor, uint16_t address, uint8_t* data, uint16_t* length) {
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
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
LidarSdkErrorCode writeSensorI2C(LidarSensorIndex sensor, uint16_t address, const uint8_t* kData, uint16_t length) {
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

/*******************************************************************************
 * \brief Reading hex data from a file.
 * \param[in] kFilePath : file path
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[out] buffer : data buffer
 *                Range: 0 - 2^8-1. Accuracy: 1.
 * \return status of read data from file
 ******************************************************************************/
bool readHexDataFromFile(const std::string& kFilePath, std::vector<char>& buffer) {
    std::ifstream file(kFilePath, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
        std::cout << "无法打开文件!, kFilePath:" << kFilePath << std::endl;
        return false;
    }

    // 获取文件大小
    std::streamsize size = file.tellg();
    (void)file.seekg(0, std::ios::beg);
    buffer.resize(size);

    // 读取文件内容到缓冲区
    if (file.read(buffer.data(), size)) {
        std::cout << "read " << size << " bytes" << std::endl;
    } else {
        std::cout << "无法读取文件!" << std::endl;
        file.close();
        return false;
    }

    file.close();
    return true;
}

/*******************************************************************************
 * \brief Converting data to hex.
 * \param[in] kBuffer : data buffer
 *                Range: 0 - 2^8-1. Accuracy: 1.
 * \param[in] frame_idx : index of frame
 *                Range: 0 - 2^63-1. Accuracy: 1.
 * \return The collection of all files that match the naming convention
 ******************************************************************************/
void convertToHex(const std::vector<char>& kBuffer, size_t frame_idx) {
    const size_t kMsopSize {3116};
    const size_t kDifop2Size {500};
    const size_t kDifop1Size {224};

    std::ofstream out_file(lidar_data_path + "/" + std::to_string(frame_idx + 1) + ".txt");
    if (out_file.is_open()) {
        for (size_t i = 0; i < kBuffer.size(); i++) {
            size_t j = i % ((kMsopSize + kDifop2Size) + kDifop1Size);
            if (i > 0 && j == 0) {
                out_file << std::endl;
            }
            if (j == kMsopSize) {
                out_file << "\n";
            } else if (j == (kMsopSize + kDifop2Size)) {
                out_file << "\n";
            }
            out_file << std::hex << std::setw(2) << std::setfill('0')
                     << static_cast<int32_t>(static_cast<std::uint8_t>(kBuffer[i])) << " ";
        }
        out_file << std::endl;
        out_file.close();
        std::cout << "save data to " << lidar_data_path << "/" << std::to_string(frame_idx + 1) << ".txt" << std::endl;
    } else {
        std::cout << "open file failed" << std::endl;
        return;
    }
}

/*******************************************************************************
 * \brief Simulating the retrieval of data via the data interface already made available by the underlying software.
 * \param[in] lidar_interface : lidar interface
 *                Range: 0 - 2^32-1. Accuracy: 1.
 ******************************************************************************/
void getDataFromOsApi(LidarSdkInterface* lidar_interface) {

    bool is_loop {false};
    // 1. 获取排序后的文件信息
    std::vector<FileInfo> fileInfos = getSortedFileInfos(lidar_data_path);

    // 2. 遍历排好序的文件
    for (size_t i = 0; i < fileInfos.size(); ++i) {
        const auto& kInfo = fileInfos[i];
        std::string file_name {lidar_data_path + "/" + kInfo.name};
        std::vector<char> mipi_data;

        if (!readHexDataFromFile(file_name, mipi_data)) {
            continue;
        }
        if (mipi_data.size() < 1) {
            std::cout << "mipi_data size is 0" << std::endl;
            continue;
        }
        std::cout << "load data from " << file_name << ", data size:" << mipi_data.size() << std::endl;
        all_frames_data.push_back(mipi_data);
    }
    uint32_t mipi_frame_counter {0};
    size_t current_frame_index {0};
    const size_t kDataCnt {all_frames_data.size()};
    const double kIntervalMs {100.0 / 6};
    while (!is_exit) {
        auto start_time = std::chrono::steady_clock::now();
        auto& mipi_data = all_frames_data[current_frame_index];
        LidarAdcBuffer* buf = new LidarAdcBuffer();
        buf->sensor = LidarSensorIndex::MIDDLE_LIDAR;
        buf->frameIndex = mipi_frame_counter++;
        buf->teof = getTimeNowMgr();
        buf->tsof = getTimeNowMgr();
        buf->bufObj = nullptr;
        buf->data = mipi_data.data();
        buf->len = mipi_data.size();
        buf->reservedPtr = nullptr;

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        auto sleepTime = std::chrono::microseconds((int64_t)(kIntervalMs * 1000)) - duration;
        if (sleepTime > std::chrono::microseconds(0)) {
            std::this_thread::sleep_for(sleepTime);
        }
        if (lidar_interface != nullptr) {
            LidarSdkErrorCode ret = lidar_interface->injectAdc(LidarSensorIndex::MIDDLE_LIDAR, buf);
            if (ret == LidarSdkErrorCode::LIDAR_SDK_FAILD) {
                (void)releaseAdc(LidarSensorIndex::MIDDLE_LIDAR, buf);
            }

            current_frame_index = (current_frame_index + 1) % kDataCnt;
            if (!is_loop) {
                if (current_frame_index == 0) {
                    std::cout << "file end" << std::endl;
                    break;
                }
            }
        }
    }
}

/*******************************************************************************
 * \brief Print help information.
 ******************************************************************************/
void printHelp() {
    std::cout << "使用说明:" << std::endl;
    std::cout << "程序通过命令行参数获取文件夹路径。" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  -path <folder_path>  指定文件夹路径。" << std::endl;
    std::cout << "  -n, --num <次数>    设置测试次数。" << std::endl;
    std::cout << "  -d, --pcd <PCD文件名> 保存点云数据为PCD格式。\n";
    std::cout << "  -h, --help          显示此帮助信息。" << std::endl;
}

/*******************************************************************************
 * \brief Exit path.
 *
 * \param[in] kPath : file path
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return status of path existing
 ******************************************************************************/
bool pathExists(const std::string& kPath) {
    return access(kPath.c_str(), F_OK) == 0;
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
int main(int argc, char* argv[]) {
    std::cout << "Compiled on " << __DATE__ << " at " << __TIME__ << std::endl;
    for (int32_t i = 1; i < argc; ++i) {
        std::string arg {argv[i]};

        if ((arg == "-h") || (arg == "--help")) {
            printHelp();
            return 0;
        } else if (arg == "-path") {
            if ((i + 1) < argc) {
                lidar_data_path = argv[++i]; // 获取下一个参数作为路径
            } else {
                std::cout << "错误：-p/--path 选项需要指定路径参数" << std::endl;
                printHelp();
                return 1;
            }
        } else if ((arg == "-n") || (arg == "--num")) {
            if ((i + 1) < argc) {
                test_num = std::stoi(argv[++i]); // 获取下一个参数并转换为整数
                if (test_num <= 0) {
                    std::cout << "错误：测试次数必须为正整数" << std::endl;
                    printHelp();
                    return 1;
                }
                test_stop = true;
            } else {
                std::cout << "错误：-n/--num 选项需要指定次数参数" << std::endl;
                printHelp();
                return 1;
            }
        } else if ((arg == "-d") || (arg == "--pcd")) {
            save_pcd = true;
        }
    }
    if (!pathExists(lidar_data_path)) {
        std::cout << "错误: 指定的路径不存在: " << lidar_data_path << std::endl;
        return 1;
    }
    std::cout << "模拟数据文件夹路径: " << lidar_data_path << std::endl;
    // 获取SDK接口
    LidarSdkInterface* lidar_interface = robosense::lidar_sdk::getRSLidarSdkInterface(sizeof(LidarSdkInterface));
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
        if (lidar_interface->init(&sdk_cbks, "./ag_config.json") != LIDAR_SDK_SUCCESS) {
            std::cout << "SDK initialization failed" << std::endl;
            return -1;
        }
    }
    uint32_t api_version {lidar_interface->apiVersion};
    std::cout << "API version: " << LIDAR_SDK_API_MAJOR_GET(api_version) << "." << LIDAR_SDK_API_MINOR_GET(api_version)
              << "." << LIDAR_SDK_API_PATCH_GET(api_version) << std::endl;
    const char* kVersion = lidar_interface->getLidarSdkVersion();
    std::cout << "The sdk version is " << kVersion << std::endl;
    std::thread mockThread([&]() {
        int32_t ret {pthread_setname_np(pthread_self(), "RS-DEMO-inject")};
        if (ret != 0) {
            std::cout << "set inject thread name failed" << std::endl;
        }
        getDataFromOsApi(lidar_interface);
    });
    std::thread process_thread;
    if (save_pcd) {
        process_thread = std::thread([&]() {
            int32_t ret {pthread_setname_np(pthread_self(), "RS-DEMO-proc")};
            if (ret != 0) {
                std::cout << "set process pointcloud thread name failed" << std::endl;
            }
            processPointCloud();
        });
    }

    if (test_stop) {
        for (int32_t loop_cnt = 0; loop_cnt < test_num; loop_cnt++) {
            std::cout << "Begin start sdk, loop_cnt: " << loop_cnt << std::endl;
            // 启动SDK
            if (lidar_interface->start() != LIDAR_SDK_SUCCESS) {
                std::cout << "Failed to start SDK, loop_cnt: " << loop_cnt << std::endl;
                lidar_interface->deInit();
                return -1;
            }
            std::cout << "Start sdk successfully, loop_cnt: " << loop_cnt << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::cout << "Begin stop sdk, loop_cnt: " << loop_cnt << std::endl;

            auto start = std::chrono::steady_clock::now();
            lidar_interface->stop();
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "Stop cost time: " << duration.count() << " us, loop_cnt: " << loop_cnt << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    } else {
        if (lidar_interface->start() != LIDAR_SDK_SUCCESS) {
            std::cout << "Failed to start SDK" << std::endl;
            lidar_interface->deInit();
            return -1;
        }
        uint8_t data[2] {0};
        uint16_t data_len {0};
        uint8_t nrc {0xFFU};

        lidar_interface->readDid(0x1112U, data, &data_len, &nrc);
        std::cout << std::hex << std::setw(4) << "data: 0x" << *(uint16_t*)data
                  << std::dec << ", dec: " << *(uint16_t*)data
                  << ", data_len: " << data_len << ", nrc: " << (uint16_t)nrc
                  << std::endl;

        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        lidar_interface->stop();
    }
    is_exit = true;
    if (mockThread.joinable()) {
        mockThread.join();
    }
    std::cout << "mockThread join success" << std::endl;
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
/* \}  DemoOffline */