#include "rs_lidar_sdk_api.h"
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
#include <sstream>
#include <stdio.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace std::chrono_literals;
std::string lidar_data_path_ = "../mipi_data/";
bool is_exit_ = false;
bool test_stop_ = false;
int32_t test_num_ = 0;
LidarSdkErrorCode releaseAdc(LidarSensorIndex sensor, const void* void_ptrAdc) {
    if (!void_ptrAdc) {
        return LidarSdkErrorCode::LIDAR_SDK_FAILD;
    }
    LidarAdcBuffer* buf =
        static_cast<LidarAdcBuffer*>(const_cast<void*>(void_ptrAdc));
    if (buf != nullptr) {
        delete buf;
    }
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
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

LidarSdkErrorCode pointCloudCallback(LidarSensorIndex sensor,
                                     const void* buffer,
                                     uint32_t length) {
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

    if (device_frame_seq % 600 == 0) {
        printLidarDeviceInfo(*device_info);
    }
    device_frame_seq++;

    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}
LidarSdkErrorCode
writeSensorRegUint16(LidarSensorIndex sensor, uint16_t address, uint16_t data) {
    /**
   */
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

LidarSdkErrorCode
readSensorRegUint16(LidarSensorIndex sensor, uint16_t address, uint16_t* data) {
    /**
   */
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

LidarSdkErrorCode
writeSensorRegUint8(LidarSensorIndex sensor, uint16_t address, uint8_t data) {
    /**
   */
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

LidarSdkErrorCode
readSensorRegUint8(LidarSensorIndex sensor, uint16_t address, uint8_t* data) {
    /**
   */
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}
LidarSdkErrorCode readSensorI2C(LidarSensorIndex sensor,
                                uint16_t address,
                                uint8_t* data,
                                uint16_t* length) {
    /**
   */
    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}
LidarSdkErrorCode writeSensorI2C(LidarSensorIndex sensor,
                                 uint16_t address,
                                 const uint8_t* data,
                                 uint16_t length) {
    /**
   */

    return LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
}

bool readHexDataFromFile(const std::string& filePath,
                         std::vector<char>& buffer) {
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);

    if (!file.is_open()) {
        std::cout << "无法打开文件!, filePath:" << filePath << std::endl;
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

void convertToHex(const std::vector<char>& buffer, size_t frame_idx) {
    const size_t msop_size = 3116;
    const size_t difop2_size = 500;
    const size_t difop1_size = 224;

    std::ofstream out_file(lidar_data_path_ + "/" +
                           std::to_string(frame_idx + 1) + ".txt");
    if (out_file.is_open()) {
        for (size_t i = 0; i < buffer.size(); i++) {
            size_t j = i % (msop_size + difop2_size + difop1_size);
            if (i > 0 && j == 0) {
                out_file << std::endl;
            }
            if (j == msop_size) {
                out_file << "\n";
            } else if (j == msop_size + difop2_size) {
                out_file << "\n";
            }
            out_file << std::hex << std::setw(2) << std::setfill('0')
                     << static_cast<int32_t>(
                            static_cast<std::uint8_t>(buffer[i]))
                     << " ";
        }
        out_file << std::endl;
        out_file.close();
        std::cout << "save data to " << lidar_data_path_ << "/"
                  << std::to_string(frame_idx + 1) << ".txt" << std::endl;
    } else {
        std::cout << "open file failed" << std::endl;
        return;
    }
}
/**
 * @brief 模拟从底软已经拿到数据接口获取数据
 *
 * @return no
 */

void getDataFromOsApi(LidarSdkInterface* lidar_interface) {
    bool is_loop = true;
    const size_t max_frame_count = 60;

    std::vector<std::vector<char>> all_frames_data;

    for (size_t frame_idx = 0; frame_idx < max_frame_count; ++frame_idx) {
        std::vector<char> mipi_data;
        std::string file_name = lidar_data_path_ + "/lidar_data_" +
                                std::to_string(frame_idx + 1) + ".bin";
        if (!readHexDataFromFile(file_name, mipi_data)) {
            continue;
        }

        if (mipi_data.size() < 1) {
            std::cout << "mipi_data size is 0" << std::endl;
            return;
        }
        convertToHex(mipi_data, frame_idx);
        std::cout << "load data from " << file_name
                  << ", data size:" << mipi_data.size() << std::endl;
        all_frames_data.push_back(mipi_data);
    }

    uint32_t mipi_frame_counter = 0;
    size_t current_frame_index = 0;
    const size_t data_cnt = all_frames_data.size();
    const double interval_ms = 100.0 / 6;
    while (!is_exit_) {
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
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time);
        auto sleepTime =
            std::chrono::microseconds((int64_t)(interval_ms * 1000)) - duration;
        if (sleepTime > std::chrono::microseconds(0)) {
            std::this_thread::sleep_for(sleepTime);
        }
        if (lidar_interface != nullptr) {
            LidarSdkErrorCode ret =
                lidar_interface->injectAdc(LidarSensorIndex::MIDDLE_LIDAR, buf);
            if (ret == LidarSdkErrorCode::LIDAR_SDK_FAILD) {
                (void)releaseAdc(LidarSensorIndex::MIDDLE_LIDAR, buf);
            }

            current_frame_index = (current_frame_index + 1) % data_cnt;
            if (!is_loop) {
                if (current_frame_index == 0) {
                    std::cout << "file end" << std::endl;
                    break;
                }
            }
        }
    }
}

void printHelp() {
    std::cout << "使用说明:" << std::endl;
    std::cout << "程序通过命令行参数获取文件夹路径。" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  -path <folder_path>  指定文件夹路径。" << std::endl;
    std::cout << "  -n, --num <次数>    设置测试次数。" << std::endl;
    std::cout << "  -h, --help          显示此帮助信息。" << std::endl;
}

bool pathExists(const std::string& path) {
    return access(path.c_str(), F_OK) == 0;
}
int main(int argc, char* argv[]) {
    std::cout << "Compiled on " << __DATE__ << " at " << __TIME__ << std::endl;
    for (int32_t i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printHelp();
            return 0;
        } else if (arg == "-path") {
            if (i + 1 < argc) {
                lidar_data_path_ = argv[++i]; // 获取下一个参数作为路径
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
                    std::cout << "错误：测试次数必须为正整数" << std::endl;
                    printHelp();
                    return 1;
                }
                test_stop_ = true;
            } else {
                std::cout << "错误：-n/--num 选项需要指定次数参数" << std::endl;
                printHelp();
                return 1;
            }
        }
    }
    if (!pathExists(lidar_data_path_)) {
        std::cout << "错误: 指定的路径不存在: " << lidar_data_path_
                  << std::endl;
        return 1;
    }
    std::cout << "模拟数据文件夹路径: " << lidar_data_path_ << std::endl;
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
    std::thread mockThread([&]() {
        int32_t ret = pthread_setname_np(pthread_self(), "RS-DEMO-inject");
        if (ret != 0) {
            std::cout << "set inject thread name failed" << std::endl;
        }
        getDataFromOsApi(lidar_interface);
    });

    if (test_stop_) {
        for (int32_t loop_cnt = 0; loop_cnt < test_num_; loop_cnt++) {
            std::cout << "Begin start sdk, loop_cnt: " << loop_cnt << std::endl;
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
            std::cout << "Begin stop sdk, loop_cnt: " << loop_cnt << std::endl;

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
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        lidar_interface->stop();
    }
    is_exit_ = true;
    if (mockThread.joinable()) {
        mockThread.join();
    }
    std::cout << "mockThread join success" << std::endl;
    lidar_interface->deInit();
    lidar_interface = nullptr;
    std::cout << "SDK deinit success" << std::endl;
    return 0;
}