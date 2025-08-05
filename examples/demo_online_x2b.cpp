
#include "rs_lidar_sdk_api.h"
#include "sensor_ipcclient.h"
#include "sync_queue.h"
#include "yaml_manager.h"
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

using namespace std::chrono_literals;
using namespace dimw::sensor_access;
using namespace robosense::lidar;

#define POINTCLOUD_NUM_PER_FRAME 291840
bool is_exit_ = false;
bool save_pcd_ = false;
bool test_stop_ = false;
bool save_raw_data_ = false;
std::string save_path_ = "./";
int32_t test_num_ = 0;
int32_t raw_data_file_num_max_ = 0;

std::shared_ptr<dimw::sensor_access::SensorIpcClient> lidar_client_ = nullptr;
SyncQueue<LidarPointCloud*> stuffed_cloud_queue_(500, [](LidarPointCloud* ptr) {
    if (ptr)
        delete ptr;
});

LidarSdkErrorCode releaseAdc(LidarSensorIndex sensor, const void* void_ptrAdc) {
    LidarSdkErrorCode ret_sdk_code = LidarSdkErrorCode::LIDAR_SDK_FAILD;
    LidarAdcBuffer* ptrAdc =
        static_cast<LidarAdcBuffer*>(const_cast<void*>(void_ptrAdc));
    if (ptrAdc != nullptr) {
        FrameBufObj* frame_obj =
            static_cast<FrameBufObj*>(const_cast<void*>(ptrAdc->reservedPtr));
        if (frame_obj != nullptr) {
            if (lidar_client_) {
#if 0
        std::cout << "injectAdc:" << ptrAdc << " , frame:" << frame_obj << ", buf->data :" << ptrAdc->data << ", buf->bufObj: " << ptrAdc->bufObj
               << ", buf->reservedPtr: " << ptrAdc->reservedPtr << ", buf->frameIndex: " << ptrAdc->frameIndex << ", frame->frameIndex: " << frame_obj->frameIndex << ", "
               << "release\n ";
#endif
                auto ret = lidar_client_->releaseFrame(*frame_obj);
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

LidarSdkErrorCode getSensorFsyncStartTime(uint64_t* time) {
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

uint64_t getTimeNowPhc() {
    /**
   * 实际根据底软接口获取时间，demo使用系统微妙时间
   */
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == -1) {
        std::cout << "gettimeofday failed" << std::endl;
        return 0;
    }
    return tv.tv_sec * 1000 * 1000 + tv.tv_usec;
}

uint64_t getTimeNowSys() {
    /**
   * 实际根据底软接口获取时间，demo使用系统微妙时间
   */
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == -1) {
        std::cout << "gettimeofday failed" << std::endl;
        return 0;
    }
    return tv.tv_sec * 1000 * 1000 + tv.tv_usec;
}

uint64_t getTimeNowMgr() {
    /**
   * 实际根据底软接口获取时间，demo使用系统微妙时间
   */
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == -1) {
        std::cout << "gettimeofday failed" << std::endl;
        return 0;
    }
    return tv.tv_sec * 1000 * 1000 + tv.tv_usec;
}

void savePcd(const std::string& pcd_path, LidarPointCloud* cloud) {
    std::cout << "Save point cloud as: " << pcd_path << std::endl;
    std::ofstream os(pcd_path, std::ios::out | std::ios::trunc);
    os << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
    os << "VERSION 0.7" << std::endl;
    os << "FIELDS x y z intensity ring timestamp" << std::endl;
    os << "SIZE 4 4 4 4 4 4" << std::endl;
    os << "TYPE F F F F F F" << std::endl;
    os << "COUNT 1 1 1 1 1 1" << std::endl;
    os << "WIDTH " << cloud->point_num << std::endl;
    os << "HEIGHT 1" << std::endl;
    os << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    os << "POINTS " << cloud->point_num << std::endl;
    os << "DATA ascii" << std::endl;
    for (uint32_t i = 0; i < cloud->point_num; ++i) {
        os << (cloud->point + i)->x << " ";
        os << (cloud->point + i)->y << " ";
        os << (cloud->point + i)->z << " ";
        os << static_cast<int32_t>((cloud->point + i)->intensity) << " ";
        os << static_cast<int32_t>((cloud->point + i)->channel_number) << " ";
        os << (cloud->point + i)->timestamp << std::endl;
    }
    os.close();
}

std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

    // 格式化时间戳
    std::tm* local_time = std::localtime(&now_time);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", local_time);
    return std::string(timestamp) + "_" + std::to_string(now_ms.count());
}

void processPointCloud(void) {
    int32_t frame_seq = 0;
    while (!is_exit_) {
        LidarPointCloud* lidar_cloud;
        bool state = stuffed_cloud_queue_.popWait(lidar_cloud, 1000000);
        if (!state) {
            std::cout << "processPointCloud data timeout" << std::endl;
            continue;
        }
        if (lidar_cloud && save_pcd_) {
            uint32_t cnt = 0;
            for (uint32_t i = 0; i < lidar_cloud->point_num; ++i) {
                const LidarPoint* point = &lidar_cloud->point[i];
                if (std::isnan(point->x)) {
                    cnt++;
                }
            }

            if (frame_seq % 60 == 0) {
                std::string fileName = save_path_ + getTimestamp() + "_" +
                                       std::to_string(lidar_cloud->frame_seq) +
                                       ".pcd";
                savePcd(fileName, lidar_cloud);
            }

            frame_seq++;
            free(lidar_cloud);
            lidar_cloud = nullptr;
        }
    }
}
LidarSdkErrorCode pointCloudCallback(LidarSensorIndex sensor,
                                     const void* buffer,
                                     uint32_t length) {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    // 检查输入参数
    if (!buffer || length == 0) {
        std::cout << "Error: Invalid buffer or length in pointCloudCallback"
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }

    // 将 buffer 转换为 LidarPointCloud 指针
    const LidarPointCloud* lidar_cloud =
        static_cast<const LidarPointCloud*>(buffer);

    // 计算实际的点云数据大小
    size_t expected_size = sizeof(LidarPointCloud) +
                           (lidar_cloud->point_num - 1) * sizeof(LidarPoint);

    // 验证数据长度是否符合预期
    if (length < expected_size) {
        std::cout << "pointCloudCallback Error: Buffer length is smaller than "
                     "expected, length = "
                  << length << ", expected size = " << expected_size
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }
    uint32_t cnt = 0;
    for (uint32_t i = 0; i < lidar_cloud->point_num; ++i) {
        const LidarPoint* point = &lidar_cloud->point[i];
        if (std::isnan(point->x)) {
            cnt++;
        }
    }
    static uint64_t last_recv_frame_timestamp = 0;
    uint64_t current_recv_frame_timestamp = getTimeNowPhc();
    uint64_t recv_time_interval =
        current_recv_frame_timestamp - last_recv_frame_timestamp;
    last_recv_frame_timestamp = current_recv_frame_timestamp;
    static uint64_t last_frame_timestamp = 0;
    uint64_t frame_time_interval =
        lidar_cloud->frame_timestamp - last_frame_timestamp;
    last_frame_timestamp = lidar_cloud->frame_timestamp;
    {
        std::cout << "[LidarPointCloud Info :" << lidar_cloud->frame_seq
                  << "]: protocol_version: " << lidar_cloud->protocol_version
                  << ", return_mode:"
                  << static_cast<uint32_t>(lidar_cloud->return_mode)
                  << ", sync_status:"
                  << static_cast<uint32_t>(lidar_cloud->sync_status)
                  << ", frame_sync:"
                  << static_cast<uint32_t>(lidar_cloud->frame_sync)
                  << ", point_num:" << lidar_cloud->point_num
                  << "，frame_seq:" << lidar_cloud->frame_seq
                  << ", frame_timestamp:" << lidar_cloud->frame_timestamp
                  << " us"
                  << ",mirror_id:"
                  << static_cast<uint32_t>(lidar_cloud->mirror_id)
                  << ", frame_time_interval：" << frame_time_interval << " us"
                  << ", recv_time_interval:" << recv_time_interval << " us"
                  << ", nan point num: " << cnt
                  << ", valid cnt:" << (lidar_cloud->point_num - cnt)
                  << std::endl;

        // 输出预留字段（十六进制，每个字节占2位，不足补0）
        std::cout << "  Reserved: ";
        for (size_t i = 0; i < sizeof(lidar_cloud->reserved); ++i) {
            // 设置十六进制输出、 uppercase（可选，使A-F大写）、补零（宽度2位）
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int32_t>(lidar_cloud->reserved[i])
                      << std::dec << " "; // 恢复十进制输出，避免后续输出受影响
        }
        std::cout << std::endl << std::endl;
    }

    if (!save_pcd_) {
        return LIDAR_SDK_SUCCESS;
    }
    uint32_t pointCount = POINTCLOUD_NUM_PER_FRAME;
    size_t requiredSize =
        sizeof(LidarPointCloud) + (pointCount - 1) * sizeof(LidarPoint);

    LidarPointCloud* msg = static_cast<LidarPointCloud*>(malloc(requiredSize));
    // // 复制LidarPointCloud结构体的基础成员
    msg->protocol_version = lidar_cloud->protocol_version;
    msg->return_mode = lidar_cloud->return_mode;
    msg->sync_status = lidar_cloud->sync_status;
    msg->frame_sync = lidar_cloud->frame_sync;
    msg->point_num = lidar_cloud->point_num;
    msg->frame_seq = lidar_cloud->frame_seq;
    msg->frame_timestamp = lidar_cloud->frame_timestamp;

    // // 复制reserved数组
    std::memcpy(msg->reserved, lidar_cloud->reserved,
                sizeof(lidar_cloud->reserved));

    // // 复制点云数据数组（关键部分）
    if (msg->point_num > 0) {
        size_t pointDataSize = msg->point_num * sizeof(LidarPoint);
        std::memcpy(msg->point, lidar_cloud->point, pointDataSize);
    }
    bool is_overwritten = false;
    size_t sz = stuffed_cloud_queue_.push(msg, is_overwritten);
    if (is_overwritten) {
        std::cout << "stuffed_cloud_queue_ is full, drop the oldest one sz:"
                  << sz << std::endl;
    }
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

void printLidarDeviceInfo(const LidarDeviceInfo& info) {
    std::cout << "========== Lidar Device Information ==========" << std::endl;

    // 打印固件版本号
    uint16_t firmware_version =
        info.firware_version; // 注：原拼写"firware"建议修正为"firmware"
    int32_t firmware_major = firmware_version / 10000;
    int32_t firmware_minor = (firmware_version % 10000) / 100;
    int32_t firmware_patch = firmware_version % 100;
    std::cout << "Firmware Version: " << firmware_major << "." << std::setw(2)
              << std::setfill('0') << firmware_minor << "." << std::setw(2)
              << std::setfill('0') << firmware_patch << ", raw: 0x" << std::hex
              << firmware_version << ", dec: " << std::dec << firmware_version
              << std::endl;

    // 打印SDK版本号
    uint16_t sdk_version = info.sdk_version;
    int32_t major = sdk_version / 10000;
    int32_t minor = (sdk_version % 10000) / 100;
    int32_t patch = sdk_version % 100;
    std::cout << "SDK Version: " << major << "." << std::setw(2)
              << std::setfill('0') << minor << "." << std::setw(2)
              << std::setfill('0') << patch << ", raw: 0x" << std::hex
              << sdk_version << ", dec: " << std::dec << sdk_version
              << std::endl;

    // 打印电机转速
    std::cout << "Motor Speed: " << info.motor_speed << " rpm" << std::endl;

    // 打印回波模式
    const char* return_modes[] = { "Dual Return", "Strongest Return",
                                   "Last Return", "First Return" };
    std::cout << "Return Mode: " << return_modes[info.return_mode] << " ("
              << static_cast<int32_t>(info.return_mode) << ")" << std::endl;

    // 打印时间戳
    std::cout << "Timestamp: " << info.timestamp << " μs" << std::endl;

    // 打印运行状态
    std::cout << "Operate State: "
              << (info.lidar_operation_state == 0x00 ? "Normal" : "Fault")
              << " (0x" << std::hex
              << static_cast<int32_t>(info.lidar_operation_state) << ")"
              << std::dec << std::endl;

    // 打印故障状态
    const char* fault_states[] = { "No Fault", "Level 1 Fault",
                                   "Level 2 Fault" };
    std::cout << "Fault State: " << fault_states[info.lidar_fault_state]
              << " (0x" << std::hex
              << static_cast<int32_t>(info.lidar_fault_state) << ")" << std::dec
              << std::endl;

    // 打印故障码信息
    std::cout << "SDK Fault Code Count: "
              << static_cast<int32_t>(info.sdk_total_fault_number) << std::endl;
    std::cout << "SDK Fault Code Position: 0x" << std::hex
              << info.sdk_fault_code_position << std::dec << std::endl;
    std::cout << "Active SDK Faults: " << std::endl;
    for (size_t i = 0; i < 64; ++i) {
        if (info.sdk_fault_code_position & (uint64_t(1) << i)) {
            std::cout << "  [" << static_cast<int32_t>(i) << "]" << std::endl;
        }
    }

    // 打印供应商内部故障信息
    std::cout << std::endl << "Supplier Internal Faults:" << std::endl;
    std::cout << "  Fault ID: 0x" << std::hex
              << static_cast<int32_t>(info.supplier_internal_fault_id)
              << std::dec << std::endl;
    std::cout << "  Fault Indicate: ";
    for (size_t i = 0; i < sizeof(info.supplier_internal_fault_indicate); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(
                         info.supplier_internal_fault_indicate[i])
                  << std::dec << " ";
    }
    std::cout << std::endl;
    std::cout << "  Fault Number: "
              << static_cast<int32_t>(info.supplier_internal_fault_number)
              << std::endl;
    std::cout << "  Fault Position: "
              << static_cast<int32_t>(info.supplier_internal_fault_position)
              << std::endl;

    // 打印时间同步信息
    std::cout << std::endl << "Time Synchronization:" << std::endl;
    std::cout << "  Sync Mode: 0x" << std::hex
              << static_cast<int32_t>(info.time_sync_mode) << std::dec
              << std::endl;
    std::cout << "  Sync Status: "
              << (info.time_sync_status == 0x01 ? "Success" : "Failed")
              << " (0x" << std::hex
              << static_cast<int32_t>(info.time_sync_status) << ")" << std::dec
              << std::endl;
    std::cout << "  Time Offset: " << info.time_offset << " μs" << std::endl;

    // 打印产品序列号（十六进制字符串）
    std::cout << "Product SN: ";
    for (size_t i = 0; i < sizeof(info.lidar_product_sn); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(
                         static_cast<uint8_t>(info.lidar_product_sn[i]));
    }
    std::cout << std::dec << std::endl; // 恢复十进制输出

    // 打印制造商和型号
    std::cout << "Manufacturer: 0x" << std::hex
              << static_cast<int32_t>(info.manufacture) << std::dec
              << std::endl;
    std::cout << "Model: 0x" << std::hex << static_cast<int32_t>(info.model)
              << std::dec << std::endl;

    // 打印预留字段（每10个字节换行）
    std::cout << "Reserved: " << std::endl;
    for (size_t i = 0; i < sizeof(info.reserved); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(info.reserved[i]) << std::dec << " ";
        if ((i + 1) % 10 == 0) { // 每10个字节换行
            std::cout << std::endl;
        }
    }
    std::cout << std::endl; // 结束预留字段输出

    std::cout << "=============================================" << std::endl;
}

LidarSdkErrorCode deviceInfoCallback(LidarSensorIndex sensor,
                                     const void* buffer,
                                     uint32_t length) {
    static int32_t device_frame_seq = 0;

    if (!buffer || length == 0) {
        std::cout << "Error: Invalid buffer or length in deviceInfoCallback"
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }

    // 将 buffer 转换为 LidarDeviceInfo 指针
    const LidarDeviceInfo* device_info =
        static_cast<const LidarDeviceInfo*>(buffer);

    // 计算实际的数据大小
    size_t expected_size = sizeof(LidarDeviceInfo);

    // 验证数据长度是否符合预期
    if (length < expected_size) {
        std::cout << "deviceInfoCallback Error: Buffer length is smaller than "
                     "expected, length = "
                  << length << ", expected size = " << expected_size
                  << std::endl;
        return LIDAR_SDK_FAILD;
    }

    if (device_info->sdk_total_fault_number > 0 &&
        device_info->sdk_fault_code_position != 1) {
        printLidarDeviceInfo(*device_info);
    }
    if (device_frame_seq % 600 == 0) {
        printLidarDeviceInfo(*device_info);
    }
    device_frame_seq++;

    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

LidarSdkErrorCode
writeSensorRegUint16(LidarSensorIndex sensor, uint16_t address, uint16_t data) {
    if (!lidar_client_) {
        std::cout << "writeSensorRegUint16 error: lidar_client_ is nullptr"
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "writeSensorRegUint16: address: " << address
              << ", data: " << data << std::endl;
    ClientErrorCode clientRet = lidar_client_->writeSensorRegUint16(
        SensorType::MIDDLE_LIDAR, address, data);
    std::cout << "writeSensorRegUint16: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

LidarSdkErrorCode
readSensorRegUint16(LidarSensorIndex sensor, uint16_t address, uint16_t* data) {
    if (!lidar_client_ || !data) {
        std::cout << "readSensorRegUint16 error: lidar_client_ is nullptr"
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "readSensorRegUint16: address: " << address
              << ", data: " << *data << std::endl;
    ClientErrorCode clientRet = lidar_client_->readSensorRegUint16(
        SensorType::MIDDLE_LIDAR, address, *data);
    std::cout << "readSensorRegUint16: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

LidarSdkErrorCode
writeSensorRegUint8(LidarSensorIndex sensor, uint16_t address, uint8_t data) {
    if (!lidar_client_) {
        std::cout << "[writeSensorRegUint8]  lidar_client_ is nullptr "
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "writeSensorRegUint8: address: " << address
              << ", data: " << data << std::endl;
    ClientErrorCode clientRet = lidar_client_->writeSensorRegUint8(
        SensorType::MIDDLE_LIDAR, address, data);
    std::cout << "writeSensorRegUint8: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

LidarSdkErrorCode
readSensorRegUint8(LidarSensorIndex sensor, uint16_t address, uint8_t* data) {
    if (!lidar_client_ || !data) {
        std::cout << "readSensorRegUint8 error: lidar_client_ is nullptr "
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    std::cout << "readSensorRegUint8: address: " << address
              << ", data: " << *data << std::endl;
    ClientErrorCode clientRet = lidar_client_->readSensorRegUint8(
        SensorType::MIDDLE_LIDAR, address, *data);
    std::cout << "readSensorRegUint8: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}
LidarSdkErrorCode readSensorI2C(LidarSensorIndex sensor,
                                uint16_t address,
                                uint8_t* data,
                                uint16_t* length) {
    if (!data || !length || !lidar_client_) {
        std::cout << "readSensorI2C error: lidar_client_ is nullptr "
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

    ClientErrorCode clientRet = lidar_client_->readSensorI2C(
        SensorType::MIDDLE_LIDAR, address, data, *length);
    std::cout << "readSensorI2C: clientRet: " << static_cast<int32_t>(clientRet)
              << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}
LidarSdkErrorCode writeSensorI2C(LidarSensorIndex sensor,
                                 uint16_t address,
                                 const uint8_t* data,
                                 uint16_t length) {
    if (!data || !lidar_client_) {
        std::cout << "[writeSensorI2C] Invalid input parameter or "
                     "lidar_client_ is nullptr "
                  << std::endl;
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }

    ClientErrorCode clientRet = lidar_client_->writeSensorI2C(
        SensorType::MIDDLE_LIDAR, address, data, length);
    std::cout << "writeSensorI2C: clientRet: "
              << static_cast<int32_t>(clientRet) << std::endl;
    return clientRet == ClientErrorCode::CLIENT_SUCCESS
               ? LidarSdkErrorCode::LIDAR_SDK_SUCCESS
               : LidarSdkErrorCode::LIDAR_SDK_FAILD;
}

void saveRawData(const void* mipi_data, uint32_t length, int32_t fileCounter) {
    std::string filename =
        save_path_ + "/lidar_data_" + std::to_string(fileCounter) + ".bin";

    std::ofstream outFile(filename, std::ios::binary);
    if (outFile.is_open()) {
        outFile.write(reinterpret_cast<const char*>(mipi_data), length);
        outFile.close();
        std::cout << "Saved data to: " << filename << ", (size: " << length
                  << " bytes) " << std::endl;
    } else {
        std::cout << "Failed to open file: " << filename << std::endl;
    }
}
/**
 * @brief 模拟从底软已经拿到数据接口获取数据
 * @return no
 *
 */

void getDataFromOsApi(LidarSdkInterface* lidar_interface) {
    uint16_t mask = 0b111000000;
    lidar_client_ = std::make_shared<dimw::sensor_access::SensorIpcClient>();
    ClientErrorCode ret = lidar_client_->init(mask);
    if (ret != ClientErrorCode::CLIENT_SUCCESS) {
        std::cout << "Failed to init lidarClient, error code is: "
                  << static_cast<std::int32_t>(ret) << std::endl;
        return;
    }

    ret = lidar_client_->start();
    if (ret != ClientErrorCode::CLIENT_SUCCESS) {
        std::cout << "Failed to start lidarClient, error code is: "
                  << static_cast<std::int32_t>(ret) << std::endl;
        return;
    }
    int32_t raw_data_cnt = 0;
    bool recv_first_packet = false;
    int32_t mipi_frame_cnt = 0;
    while (!is_exit_) {
        FrameBufObj* frame = new FrameBufObj();
        auto ret = lidar_client_->getFrame(SensorType::MIDDLE_LIDAR, *frame);
        if (ret != ClientErrorCode::CLIENT_SUCCESS) {
            std::cout << "lidar_client_ getFrame failed, error code is: "
                      << static_cast<std::int32_t>(ret) << std::endl;
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
        std::cout << "injectAdc:" << buf << " , frame:" << frame
                  << " buf->data :" << buf->data
                  << " buf->bufObj: " << buf->bufObj
                  << " buf->reservedPtr: " << buf->reservedPtr
                  << " buf->frameIndex: " << buf->frameIndex
                  << " frame->frameIndex: " << frame->frameIndex << std::endl;
        mipi_frame_cnt++;
        if (save_raw_data_ && mipi_frame_cnt > 60) {
            uint16_t msop_pkt_seq = frame->data[4] << 8 | frame->data[5];
            if (msop_pkt_seq == 1) {
                recv_first_packet = true;
            }
            if (test_num_ > raw_data_cnt && recv_first_packet) {
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
        if (inject_ret == LidarSdkErrorCode::LIDAR_SDK_FAILD) {
            releaseAdc(LidarSensorIndex::MIDDLE_LIDAR, buf);
        }
    }
}

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

int main(int argc, char* argv[]) {
    std::cout << "Compiled on " << __DATE__ << " at " << __TIME__ << std::endl;
    for (int32_t i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printHelp();
            return 0;
        } else if (arg == "-t" || arg == "--test") {
            test_stop_ = true;
        } else if (arg == "-p" || arg == "--path") {
            if (i + 1 < argc) {
                save_path_ = argv[++i]; // 获取下一个参数作为路径
            } else {
                std::cout << "错误：-p/--path 选项需要指定路径参数"
                          << std::endl;
                printHelp();
                return 1;
            }
        } else if (arg == "-n" || arg == "--num") {
            if (i + 1 < argc) {
                test_num_ = std::stoi(argv[++i]); // 获取下一个参数并转换为整数
                if (test_num_ <= 0) {
                    std::cout << "错误：-n/--num 选项需要指定大于0的次数参数"
                              << std::endl;
                    return 1;
                }
            } else {
                std::cout << "错误：-n/--num 选项需要指定次数参数" << std::endl;
                printHelp();
                return 1;
            }
        } else if (arg == "-s" || arg == "--save") {
            save_raw_data_ = true;
        } else if (arg == "-d" || arg == "--pcd") {
            save_pcd_ = true;
        }
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
        if (lidar_interface->init(&sdk_cbks, "thread_params.yaml") !=
            LIDAR_SDK_SUCCESS) {
            std::cout << "SDK initialization failed" << std::endl;
            return -1;
        }
    }
    uint32_t api_version = lidar_interface->apiVersion;
    std::cout << "API version: " << LIDAR_SDK_API_MAJOR_GET(api_version) << "."
              << LIDAR_SDK_API_MINOR_GET(api_version) << "."
              << LIDAR_SDK_API_PATCH_GET(api_version) << std::endl;
    const char* version = lidar_interface->getLidarSdkVersion();
    std::cout << "The sdk version is " << version << std::endl;

    if (!save_pcd_ && (true == yaml::demo_test_param.data_valid)) {
        save_pcd_ = yaml::demo_test_param.enable_save_pcd;
        save_path_ = yaml::demo_test_param.save_path;
        std::cout << "save_pcd_ = " << save_pcd_ << std::endl;
        std::cout << "pcd_save_path_ = " << save_path_ << std::endl;
    }

    std::thread mock_thread([&]() {
        int32_t ret = pthread_setname_np(pthread_self(), "RS-DEMO-inject");
        if (ret != 0) {
            std::cout << "set inject thread name failed" << std::endl;
        }

        getDataFromOsApi(lidar_interface);
    });

    std::thread process_thread;
    if (save_pcd_) {
        process_thread = std::thread([&]() {
            int32_t ret = pthread_setname_np(pthread_self(), "RS-DEMO-proc");
            if (ret != 0) {
                std::cout << "set process pointcloud thread name failed"
                          << std::endl;
            }
            processPointCloud();
        });
    }

    if (test_stop_) {
        for (int32_t loop_cnt = 0; loop_cnt < test_num_; loop_cnt++) {
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

        uint8_t data[2] = { 0 };
        uint16_t data_len { 0 };
        uint8_t nrc { 0xFFU };

        lidar_interface->readDid(0x1112U, data, &data_len, &nrc);
        std::cout << "data: " << *(uint16_t*)data << ", data_len: " << data_len
                  << ", nrc: " << (uint16_t)nrc << std::endl;

        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        lidar_interface->stop();
    }
    is_exit_ = true;
    lidar_client_->stop();
    lidar_client_->deInit();
    lidar_client_ = nullptr;
    std::cout << "lidar client deinit success" << std::endl;
    if (mock_thread.joinable()) {
        mock_thread.join();
    }
    std::cout << "mock_thread join success" << std::endl;
    lidar_interface->deInit();
    lidar_interface = nullptr;
    if (save_pcd_) {
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
