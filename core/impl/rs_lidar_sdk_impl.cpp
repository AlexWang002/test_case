/*******************************************************************************
 * \addtogroup impl
 * \{
 * \file rs_lidar_sdk_impl.cpp
 * \brief Defines the implementation of the RSLidarSdkImpl class.
 * \version 0.2
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-07 | Init version |
 * | 0.2 | 2025-08-07 | Add comments |
 *
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <sys/syscall.h>
#include <cstring>
#include <fstream>
#include <iostream>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "rs_lidar_sdk_impl.h"
#include "crc32.h"
#include "cpu_load.h"
#include "rs_new_logger.h"
#include "thread_config.h"
#include "yaml_manager.h"
#include "version/version.hpp"
#include "spdlog/fmt/bin_to_hex.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Print the thread configurations.
 * \param[in] thread_params The vector of thread configurations.
 */
void printThreadConfigs(const std::vector<yaml::ThreadConfig>& thread_params) {
    std::cout << "==== Loaded ThreadConfig ====" << std::endl;
    for (const auto& cfg : thread_params) {
        std::cout << "Thread ID        : " << cfg.id << std::endl;
        std::cout << "Policy           : " << cfg.policy << std::endl;
        std::cout << "Priority         : " << cfg.priority << std::endl;
        std::cout << "CPU Affinity     : [";
        for (size_t i = 0; i < cfg.cpu_affinity.size(); ++i) {
            std::cout << cfg.cpu_affinity[i];
            if (i != cfg.cpu_affinity.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        std::cout << "-----------------------------" << std::endl;
    }
}

/**
 * \brief Read the configuration JSON file.
 * \param[in] file_path The path of the configuration JSON file.
 * \param[out] json_data The JSON data read from the file.
 * \return True if the file is read successfully, false otherwise.
 */
bool readConfigJson(const std::string& file_path, nlohmann::json& json_data) {
    if (0 != access(file_path.c_str(), F_OK)) {
        std::cout << "Config json file " << file_path << " not exist." << std::endl;
        return false;
    }
    if (0 != access(file_path.c_str(), R_OK)) {
        std::cout << "Config json file " << file_path << " exist but not readable." << std::endl;
        return false;
    }
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cout << "Failed to open json file: " << file_path << std::endl;
        return false;
    }

    try {
        nlohmann::json j;
        file >> j;
        file.close();

        if (!j.contains("RobosenseLidar")) {
            std::cout << "Json file " << file_path << " not contain RobosenseLidar." << std::endl;
            return false;
        }
        json_data = j["RobosenseLidar"];

        return true;
    } catch (const nlohmann::json::parse_error& e) {
        file.close();
        std::cout << "Failed to parse JSON file: " << file_path << ", " << e.what() << std::endl;
    }
}

// #define __ARM_NEON__ 1

// ARM架构下的NEON硬件加速拷贝函数
#ifdef __ARM_NEON__
#include <arm_neon.h>
static void neon_memcpy(void* dest, const void* src, size_t size) {
    // 按64字节块进行NEON加速拷贝
    size_t blocks = size / 64;
    size_t remainder = size % 64;
    uint8_t* dest8 = (uint8_t*)dest;
    const uint8_t* src8 = (const uint8_t*)src;

    for (size_t i = 0; i < blocks; i++) {
        uint8x16x4_t v = vld1q_u8_x4(src8);
        vst1q_u8_x4(dest8, v);
        src8 += 64;
        dest8 += 64;
    }
    // 处理剩余字节
    if (remainder) {
        memcpy(dest8, src8, remainder);
    }
}
#   define HW_MEMCPY neon_memcpy
#else
#   define HW_MEMCPY memcpy // 非ARM架构回退到标准memcpy
#endif

/******************************************************************************/
/*          Definition of public functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Constructor of RSLidarSdkImpl class.
 */
RSLidarSdkImpl::RSLidarSdkImpl() :
    mipi_data_queue_(MIPI_DATA_QUEUE_SIZE,
        [this](LidarAdcBuffer* data) {
            this->releaseBuffer(std::move(data));
        }),
    msop_data_queue_(MSOP_DATA_QUEUE_SIZE,
        [](MsopData data) {
            if (data.data) {
                delete[] data.data;
            }
        }),
    free_msop_data_queue_(MSOP_DATA_QUEUE_SIZE,
        [](MsopData data) {
            if (data.data) {
                delete[] data.data;
            }
        }),
    point_cloud_queue_(POINT_CLOUD_QUEUE_SIZE,
        [](LidarPointCloudPtr ptr) {
        }) {
}

/**
 * \brief Destructor of RSLidarSdkImpl class.
 */
RSLidarSdkImpl::~RSLidarSdkImpl() {
  (void)deInit();
}

/**
 * \brief Get the version of the Lidar SDK.
 * \return The version of the Lidar SDK.
 */
const char* RSLidarSdkImpl::getLidarSdkVersion() {
  static char version[16];

  std::snprintf(version, sizeof(version), "%d.%02d.%02d",
                RSLIDARSDK_VERSION_MAJOR,
                RSLIDARSDK_VERSION_MINOR,
                RSLIDARSDK_VERSION_PATCH);

  return version;
}

/**
 * \brief Initialize the Lidar SDK.
 * \param[in] fptrCbks The pointer of callbacks structure of the Lidar SDK.
 * \param[in] configPath The path of the configuration file.
 * \return The error code of the initialization.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Initialization succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Initialization failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::init(const LidarSdkCbks* fptrCbks,
                                    const char* configPath) {
  std::lock_guard<std::mutex> lock(mutex_);
  RSLogConfig log_config;
  log_config.type_ = LoggerType::ASYNC_FILE;
  log_config.level_ = LogLevel::INFO;
#if defined(__arm__) || defined(__aarch64__)
  log_config.log_file_path_ = "/applog/lidar/rs_lidar_sdk.log";
#else
  log_config.log_file_path_ = "./rs_lidar_sdk.log";
#endif
  log_config.max_file_size_mb_ = 6;
  log_config.max_files_ = 3;

  bool config_valid = readConfigJson(configPath, json_data_);
  std::string thread_yaml_path = "./thread_params.yaml";
  if (!config_valid) {
    std::cout << "ReadConfigJson failed! file path: " << configPath << std::endl;
    return LIDAR_SDK_FAILD;
  }
  else {
    if (json_data_.contains("LOG_CONFIG_FILE")) {
      log_config.log_file_path_ = json_data_["LOG_CONFIG_FILE"].get<std::string>();

      InitLog(log_config);
      LogInfo("LOG_CONFIG_FILE: {}", log_config.log_file_path_);
      LogInfo("Begin to initialize LidarSdk");
    }
    else {
      std::cout << "LOG_CONFIG_FILE not found in json file" << std::endl;
      return LIDAR_SDK_FAILD;
    }

    if (json_data_.contains("ENABLE_MIPI_CRC")) {
      enable_mipi_crc_ = json_data_["ENABLE_MIPI_CRC"].get<bool>();
      LogInfo("ENABLE_MIPI_CRC: {}", enable_mipi_crc_);
    } else {
        LogWarn("ENABLE_MIPI_CRC not found in json file, using default value: {}", enable_mipi_crc_);
    }
    if (json_data_.contains("INNER_PARAM_PATH")) {
      config_path_ = json_data_["INNER_PARAM_PATH"].get<std::string>();
      LogInfo("INNER_PARAM_PATH: {}", config_path_);
    } else {
        LogWarn("INNER_PARAM_PATH not found in json file, using default config: {}", config_path_);
    }
    if (json_data_.contains("THREAD_CONFIG_FILE")) {
      thread_yaml_path = json_data_["THREAD_CONFIG_FILE"].get<std::string>();
      LogInfo("THREAD_CONFIG_FILE: {}", thread_yaml_path);
    } else {
        LogWarn("THREAD_CONFIG_FILE not found in json file, using default config: {}", thread_yaml_path);
    }
  }

  if (is_initialized_.load(std::memory_order_seq_cst)) {
    LogWarn("LidarSdk has been initialized!");
    return LIDAR_SDK_SUCCESS;
  }

  if (!fptrCbks) {
    LogError("callbacks is null!");
    return LIDAR_SDK_FAILD;
  }

#if defined(__arm__) || defined(__aarch64__)
  config_path_ = "/osdata/satelite/lidar/";
#else
  config_path_ = "./";
#endif


  yaml::ThreadConfigYaml thread_config_yaml(thread_yaml_path);
  yaml::ErrorCode erro_code = thread_config_yaml.parseYamlFile();
  std::cout << "============== thread_config_yaml: " << yaml::thread_params.size() << " " << (int32_t)erro_code << std::endl;
  printThreadConfigs(yaml::thread_params);

  if (!loadConfiguration(config_path_)) {
    LogError("load configuration failed!");
    return LIDAR_SDK_FAILD;
  }
  LogInfo("load configuration succeeded!");
  setLidarSdkCallbacks(fptrCbks);
  if (!validateCallbacks()) {
    LogError("validate callbacks failed!");
    return LIDAR_SDK_FAILD;
  }

  RSDecoderParam param;
  param.lidar_type = LidarType::RSEMX;
  decoder_ptr_ = std::make_shared<DecoderRSEMX>(param);

  if (nullptr == decoder_ptr_) {
    LogError("create decoder failed! LidarType: {}", static_cast<int32_t>(param.lidar_type));
    return LIDAR_SDK_FAILD;
  }
  if (!decoder_ptr_->init()) {
    LogError("decoder init failed!");
    return LIDAR_SDK_FAILD;
  }
  sdk_version_encoded_ = ((RSLIDARSDK_VERSION_MAJOR % 10) * 10000) +
                          (RSLIDARSDK_VERSION_MINOR % 100 * 100) +
                          RSLIDARSDK_VERSION_PATCH;

  decoder_ptr_->regCallback(std::bind(&RSLidarSdkImpl::splitFrame, this, std::placeholders::_1));
  cloud_manager_ptr_ = std::make_shared<CloudManager>();
  cloud_manager_ptr_->regCallback(std::bind(&RSLidarSdkImpl::putMsop, this, std::placeholders::_1, std::placeholders::_2));
  pointcloud_fps_counter_ptr_.reset(new FPSCounter("PointCloud", 10));
  device_info_fps_counter_ptr_.reset(new FPSCounter("DeviceInfo", 60));
  is_initialized_.store(true, std::memory_order_seq_cst);

  LogInfo("LidarSdk initialized successfully! Version: {}", sdk_version_encoded_);
  return LIDAR_SDK_SUCCESS;
}

/**
 * \brief Deinitialize the Lidar SDK.
 * \return The error code of the deinitialization.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Deinitialization succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Deinitialization failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::deInit() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_initialized_.load(std::memory_order_seq_cst))
  {
    return LIDAR_SDK_SUCCESS;
  }
  LogInfo("Begin to deInit lidar sdk");
  LidarSdkErrorCode ret_code = LIDAR_SDK_SUCCESS;
  if (is_started_.load(std::memory_order_seq_cst))
  {
    if (stop() != LIDAR_SDK_SUCCESS)
    {
      LogError("Failed to stop LidarSdk during deinitialization");
      ret_code = LIDAR_SDK_FAILD;
    }
  }

  DestroyLog();
  clearLidarSdkCallbacks();
  is_initialized_.store(false, std::memory_order_seq_cst);
  LogInfo("Completed deinitialized LidarSdk: Result = {}", ret_code == LIDAR_SDK_SUCCESS ? "SUCCESS" : "FAILURE");
  return ret_code;
}

/**
 * \brief Start the Lidar SDK.
 * \return The error code of the start.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Starting succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Starting failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  LogInfo("Begin to start LidarSdk");
  if (!is_initialized_.load(std::memory_order_seq_cst))
  {
    LogError("LidarSdk start failed, please call init() first");
    return LIDAR_SDK_FAILD;
  }

  if (is_started_.load(std::memory_order_seq_cst))
  {
    LogWarn("LidarSdk is already started");
    return LIDAR_SDK_SUCCESS;
  }
  LidarSdkErrorCode ret = LIDAR_SDK_FAILD;
  try
  {
    is_running_.store(true, std::memory_order_seq_cst);
    LogInfo("Creating data handling thread...");
    yaml::ThreadConfig handle_mipi_config;
    if (yaml::thread_params.size() > 0)
        handle_mipi_config = yaml::thread_params[0];
    handle_lidar_mipi_data_thread_ = createConfiguredStdThread(handle_mipi_config, [this] {
      int32_t ret = pthread_setname_np(pthread_self(), "RS-HandleMIPI");
      if (ret != 0) {
        LogError("Failed to set thread name: %d", ret);
      }
      this->handleLidarMipiData();
    });
    yaml::ThreadConfig handle_msop_config;
    if (yaml::thread_params.size() > 0)
        handle_msop_config = yaml::thread_params[1];
    handle_process_msop_data_thread_ = createConfiguredStdThread(handle_msop_config, [this] {
      int32_t ret = pthread_setname_np(pthread_self(), "RS-HandleMsop");
      if (ret != 0) {
        LogError("Failed to set thread name: %d", ret);
      }
      this->handleMsopData();
    });
    if (cloud_manager_ptr_)
    {
      cloud_manager_ptr_->start();
    }
    is_started_.store(true, std::memory_order_seq_cst);
    ret = LIDAR_SDK_SUCCESS;
  }
  catch (const std::exception& e)
  {
    LogError("Failed to start lidar thread: {}", e.what());
    is_running_.store(false, std::memory_order_seq_cst);
    if (handle_lidar_mipi_data_thread_.joinable())
    {
      try
      {
        handle_lidar_mipi_data_thread_.detach();
        LogInfo("Detached orphaned mipi data handling thread");
      }
      catch (...)
      {
        LogError("Failed to detach orphaned mipi data thread during start failure cleanup");
      }
    }
    if (handle_process_msop_data_thread_.joinable())
    {
      handle_process_msop_data_thread_.join();
    }
    else
    {
      LogError("handle_process_msop_data_thread_ is not joinable");
    }
    ret = LIDAR_SDK_FAILD;
  }
  LogInfo("Completed starting LidarSdk: Result = {}", ret == LIDAR_SDK_SUCCESS ? "SUCCESS" : "FAILURE");
  return ret;
}

/**
 * \brief Stop the Lidar SDK.
 * \return The error code of the stop.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Stopping succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Stopping failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::stop() {
  if (!is_started_.load(std::memory_order_seq_cst))
  {
    LogWarn("LidarSdk is not started");
    return LIDAR_SDK_SUCCESS;
  }
  LogInfo("Begin to stop LidarSdk");
  static constexpr uint16_t REQUEST_ADDR{ 0x0071U };
  static constexpr uint16_t ACK_ADDR{ 0x0072U };
  static constexpr uint8_t POWEROFF_SIGNAL{ 0x01U };
  static constexpr uint8_t READY_POWEROFF{ 0x01U };
  static constexpr uint8_t NOT_READY_POWEROFF{ 0x00U };
  uint16_t data_length{ 0x01U };
  uint8_t poweroff_request_ack{ NOT_READY_POWEROFF };

  std::lock_guard<std::mutex> lock(mutex_);

  auto start_time = std::chrono::steady_clock::now();
  if (callbacks_.writeSensorI2C)
  {
    callbacks_.writeSensorI2C(sensor_index_, REQUEST_ADDR, &POWEROFF_SIGNAL, data_length);
  }
  else
  {
    LogError("callbacks_.writeSensorI2C is INVALID.");
  }

  LidarSdkErrorCode ret = LIDAR_SDK_SUCCESS;
  is_running_.store(false, std::memory_order_seq_cst);

  mipi_data_queue_.stopWait();
  mipi_data_queue_.reset();

  if (handle_lidar_mipi_data_thread_.joinable())
  {
    handle_lidar_mipi_data_thread_.join();
  }
  else
  {
    LogError("handle_lidar_mipi_data_thread_ is not joinable");
  }
  if (handle_process_msop_data_thread_.joinable())
  {
    handle_process_msop_data_thread_.join();
  }
  else
  {
    LogError("handle_process_msop_data_thread_ is not joinable");
  }

  mipi_data_queue_.clear();
  if (cloud_manager_ptr_)
  {
    if (cloud_manager_ptr_->stop())
    {
      ret = LIDAR_SDK_SUCCESS;
      LogInfo("Cloud manager stopped successfully");
    }
    else
    {
      LogError("SDK Stop cloud manager thread failed");
      ret = LIDAR_SDK_FAILD;
    }
  }
  else
  {
    LogWarn("Cloud manager pointer is null");
  }

  if (callbacks_.readSensorI2C)
  {
    callbacks_.readSensorI2C(sensor_index_, ACK_ADDR, &poweroff_request_ack, &data_length);

    while (poweroff_request_ack != READY_POWEROFF)
    {
      (void)usleep(50e3);  // sleep 50ms
      auto end = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);

      if (duration.count() > 1800)
      {  // wait 1.8s
        LogError("Time out, register: {} is {}, It is NOT ready for safely power off.", ACK_ADDR, poweroff_request_ack);
        break;
      }
      callbacks_.readSensorI2C(sensor_index_, ACK_ADDR, &poweroff_request_ack, &data_length);
    }

    LogInfo("register: {} is {}.", ACK_ADDR, poweroff_request_ack);
  }
  else
  {
    LogError("callbacks_.readSensorI2C is INVALID.");
  }
  auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time);
  is_started_.store(false, std::memory_order_seq_cst);
  LogInfo("Completed stopping LidarSdk (total time: {} ms, result: {})", total_duration.count(), ret == LIDAR_SDK_SUCCESS ? "SUCCESS" : "FAILURE");
  return ret;
}

/**
 * \brief Inject ADC data to the Lidar SDK.
 * \param[in] sensor The sensor index.
 * \param[in] ptrAdc The pointer to the ADC data.
 * \return The error code of the inject.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Injection succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Injection failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::injectAdc(LidarSensorIndex sensor,
                                        const void* ptrAdc) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_initialized_.load(std::memory_order_seq_cst)) {
    LogError("LidarSdk is not initialized");
    return LIDAR_SDK_FAILD;
  }
  if (!is_started_.load(std::memory_order_seq_cst)) {
    LogError("LidarSdk is not started");
    return LIDAR_SDK_FAILD;
  }
  if (!ptrAdc) {
    LogError("LidarSdk inject adc data failed, ptrAdc is null");
    FaultManager::getInstance().setFault(FaultBits::LidarMIPIPacketLengthFault);
    return LIDAR_SDK_FAILD;
  }

  LidarAdcBuffer* adc_data = (LidarAdcBuffer*)ptrAdc;

  const static uint32_t expected_len = decoder_ptr_->getTheoreticalMipiDataLength();
  uint32_t adc_data_len = adc_data->len;
  if (expected_len != adc_data_len) {
    LogError("Invalid ADC data length, expected {}, got {}", expected_len, adc_data_len);
    FaultManager::getInstance().setFault(FaultBits::LidarMIPIPacketLengthFault);
    runDeviceInfoCallback();
    return LIDAR_SDK_FAILD;
  }
  static uint32_t last_mipi_frame_idx = 0;  ///< Mipi frame index, Range: [0, 4294967295]
  uint32_t mipi_frame_idx = adc_data->frameIndex;
  if ((last_mipi_frame_idx != 0) && (mipi_frame_idx != last_mipi_frame_idx + 1)) {
    LogError("Mipi frame index not continuous, last index: {}, current index: {}", last_mipi_frame_idx, mipi_frame_idx);
  }

  // NOTE Check the duration of MIPI data
  static uint64_t last_ts_of {0};
  static uint64_t last_te_of {0};
  auto now_ts_of = adc_data->tsof;
  auto now_te_of = adc_data->teof;
  auto dur_tsof = now_ts_of - last_ts_of;

  if (dur_tsof > 20e3) {  // Mipi data duration timeout, 20ms
    LogError("duration timeout");
    LogWarn("Mipi frame index: {}, last tsof: {}, now tsof:{}, last teof: {}, now teof: {}", mipi_frame_idx, last_ts_of, now_ts_of, last_te_of, now_te_of);
  }
  last_mipi_frame_idx = mipi_frame_idx;
  last_te_of = now_te_of;
  last_ts_of = now_ts_of;

  // NOTE Detect the continuity of frame counter in E2E
  uint8_t* lidar_data = (uint8_t*)adc_data->data;
  static uint16_t last_frame_counter {0};   ///< E2E frame counter, Range: [0, 65535]
  uint16_t current_frame_counter = (static_cast<uint16_t>(lidar_data[adc_data_len - 9]) << 8) |
                                   static_cast<uint16_t>(lidar_data[adc_data_len - 10]);

  if ((last_frame_counter != 0) && (current_frame_counter != ++last_frame_counter)) {
    LogError("E2E frame counter not continuous, last counter: {}, current counter: {}",
                          --last_frame_counter, current_frame_counter);
    LogError("Mipi frame index: {}, MIPI tsof: {} us, MIPI teof: {} us",
                          mipi_frame_idx, now_ts_of, now_te_of);
    uint32_t msop_sec = (static_cast<uint32_t>(lidar_data[8]) << 24) |
                        (static_cast<uint32_t>(lidar_data[9]) << 16) |
                        (static_cast<uint32_t>(lidar_data[10]) << 8) |
                        static_cast<uint32_t>(lidar_data[11]);
    uint32_t msop_usec = (static_cast<uint32_t>(lidar_data[12]) << 24) |
                        (static_cast<uint32_t>(lidar_data[13]) << 16) |
                        (static_cast<uint32_t>(lidar_data[14]) << 8) |
                        static_cast<uint32_t>(lidar_data[15]);
    uint64_t cur_msop_usec = (static_cast<uint64_t>(msop_sec)) * (1e6) +
                              static_cast<uint64_t>(msop_usec);
    auto host_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch());
    auto cur_unix_usec = host_time.count();
    LogError("E2E frame counter: {}, E2E timestamp: {} us, host timestamp: {} us",
                          current_frame_counter, cur_msop_usec, cur_unix_usec);
    LogError("Delay of Lidar to Middleware: {:.3f} ms",
                          (now_te_of - cur_msop_usec) * (1e-3));
    LogError("Delay of Middleware to Host: {:.3f} ms, total delay: {:.3f} ms",
                          (cur_unix_usec - now_te_of) * (1e-3),
                          (cur_unix_usec - cur_msop_usec) * (1e-3));
  }
  last_frame_counter = current_frame_counter;

  // NOTE Test the delay of the mipi data transmission
  if (0 == mipi_frame_idx % 300) { // 5s
    LogDebug("Mipi frame index: {}, MIPI tsof: {} us, MIPI teof: {} us",
                          mipi_frame_idx, now_ts_of, now_te_of);
    uint32_t msop_sec = (static_cast<uint32_t>(lidar_data[8]) << 24) |
                        (static_cast<uint32_t>(lidar_data[9]) << 16) |
                        (static_cast<uint32_t>(lidar_data[10]) << 8) |
                        static_cast<uint32_t>(lidar_data[11]);
    uint32_t msop_usec = (static_cast<uint32_t>(lidar_data[12]) << 24) |
                        (static_cast<uint32_t>(lidar_data[13]) << 16) |
                        (static_cast<uint32_t>(lidar_data[14]) << 8) |
                        static_cast<uint32_t>(lidar_data[15]);
    uint64_t cur_msop_usec = (static_cast<uint64_t>(msop_sec)) * (1e6) +
                              static_cast<uint64_t>(msop_usec);
    auto host_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch());
    auto cur_unix_usec = host_time.count();
    LogDebug("E2E frame counter: {}, E2E timestamp: {} us, host timestamp: {} us",
                          current_frame_counter, cur_msop_usec, cur_unix_usec);
    LogDebug("Delay of Lidar to Middleware: {:.3f} ms",
                          (now_te_of - cur_msop_usec) * (1e-3));
    LogDebug("Delay of Middleware to Host: {:.3f} ms, total delay: {:.3f} ms",
                          (cur_unix_usec - now_te_of) * (1e-3),
                          (cur_unix_usec - cur_msop_usec) * (1e-3));
  }

  // NOTE Check the CRC32 of MIPI data
  if (enable_mipi_crc_) {
    uint32_t calculate_crc = crc32::calculate_crc32(lidar_data, adc_data_len - 4);
    uint32_t expected_crc =
        lidar_data[adc_data_len - 1] << 24 | lidar_data[adc_data_len - 2] << 16 |
        lidar_data[adc_data_len - 3] << 8 | lidar_data[adc_data_len - 4];
    static uint32_t right_cnt {0};
    static uint32_t error_cnt {0};

    if (calculate_crc != expected_crc) {
      ++error_cnt;
      LogError("Expected CRC: 0x{:8x}, Calculate CRC: 0x{:8x}, cnt: {}",
                expected_crc, calculate_crc,
                (static_cast<uint16_t>(lidar_data[adc_data_len - 9]) << 8) |
                static_cast<uint16_t>(lidar_data[adc_data_len - 10]));
      LogError("right count: {}, error count: {}, error rate: {:6f}",
                right_cnt, error_cnt,
                static_cast<double>(error_cnt) / static_cast<double>(error_cnt + right_cnt));
    } else {
      ++right_cnt;
    }

    if (0 == (right_cnt + error_cnt) % 600 ) {
      LogInfo("right count: {}, error count: {}, error rate: {:6f}",
              right_cnt, error_cnt,
              static_cast<double>(error_cnt) / static_cast<double>(error_cnt + right_cnt));
    }
  }

  sensor_index_ = sensor;
  bool overflow = false;
  size_t sz = mipi_data_queue_.push(adc_data, overflow);
  if (overflow) {
    LogError("mipi_data_queue_ is full, drop one packet, sz:{}", sz);
  }

  return LIDAR_SDK_SUCCESS;
}

/**
 * \brief Write DID data to the Lidar SDK.
 * \param[in] did The DID index.
 * \param[in] data The pointer to the DID data.
 * \param[in] dataLen The length of the DID data.
 * \param[out] nrc The pointer to the NRC data.
 * \return The error code of the write.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Did writing succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Did writing failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::writeDid(uint16_t did, uint8_t* data,
                                        uint16_t dataLen, uint8_t* nrc) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_started_.load(std::memory_order_seq_cst))
  {
    LogError("Write Did failed: LidarSdk is not started");
    return LIDAR_SDK_FAILD;
  }

  if (!data)
  {
    LogError("write Did failed: data is null");
    return LIDAR_SDK_FAILD;
  }
  if (0 == dataLen)
  {
    LogError("write Did failed: dataLen is 0");
    return LIDAR_SDK_FAILD;
  }

  // 实现DID写入逻辑
  return LIDAR_SDK_SUCCESS;
}

/**
 * \brief Read DID data from the Lidar SDK.
 * \param[in] did The DID index.
 * \param[out] data The pointer to the DID data.
 * \param[out] dataLen The length of the DID data.
 * \param[out] nrc The pointer to the NRC data.
 * \return The error code of the read.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Did reading succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Did reading failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::readDid(uint16_t did, uint8_t* data,
                                        uint16_t* dataLen, uint8_t* nrc) {
  static const uint16_t READ_SDK_VERSION_DID = 0x1112U;
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_started_.load(std::memory_order_seq_cst))
  {
    LogError("Read Did failed: LidarSdk is not started");
    return LIDAR_SDK_FAILD;
  }

  if (!data || !dataLen)
  {
    LogError("Read Did failed: data or dataLen is null");
    return LIDAR_SDK_FAILD;
  }

  // 实现DID读取逻辑
  if (READ_SDK_VERSION_DID == did)
  {
    (void)std::memcpy(data, &sdk_version_encoded_, sizeof(sdk_version_encoded_));
    *dataLen = sizeof(sdk_version_encoded_);
    *nrc = static_cast<uint8_t>(POSITIVE);
  }
  else
  {
    *dataLen = 0;
    *nrc = static_cast<uint8_t>(REQUEST_SEQUENCE_ERROR);
  }

  return LIDAR_SDK_SUCCESS;
}

/**
 * \brief Control the Lidar SDK by RID.
 * \param[in] mode The mode of the control.
 * \param[in] rid The RID index.
 * \param[in] dataIn The pointer to the input data.
 * \param[in] dataInLen The length of the input data.
 * \param[out] dataOut The pointer to the output data.
 * \param[out] dataOutLen The length of the output data.
 * \param[out] nrc The pointer to the NRC data.
 * \return The error code of the control.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Controling succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Controling failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::ridControl(uint8_t mode, uint16_t rid,
        uint8_t* dataIn, uint16_t dataInLen, uint8_t* dataOut,
        uint16_t* dataOutLen, uint8_t* nrc) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_started_.load(std::memory_order_seq_cst))
  {
    LogError("RSLidarSdkImpl::ridControl: Lidar is not started");
    return LIDAR_SDK_FAILD;
  }

  // 实现RID控制逻辑
  return LIDAR_SDK_SUCCESS;
}

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Set the Lidar SDK callbacks.
 * \param[in] src The pointer of callbacks structure to the Lidar SDK.
 */
void RSLidarSdkImpl::setLidarSdkCallbacks(const LidarSdkCbks* src) {
  if (src)
  {
    callbacks_.releaseAdc = src->releaseAdc;
    callbacks_.getSensorFsyncStartTime = src->getSensorFsyncStartTime;
    callbacks_.getTimeNowPhc = src->getTimeNowPhc;
    callbacks_.getTimeNowSys = src->getTimeNowSys;
    callbacks_.getTimeNowMgr = src->getTimeNowMgr;
    callbacks_.writeSensorRegUint8 = src->writeSensorRegUint8;
    callbacks_.readSensorRegUint8 = src->readSensorRegUint8;
    callbacks_.writeSensorRegUint16 = src->writeSensorRegUint16;
    callbacks_.readSensorRegUint16 = src->readSensorRegUint16;
    callbacks_.readSensorI2C = src->readSensorI2C;
    callbacks_.writeSensorI2C = src->writeSensorI2C;
    callbacks_.deviceInfo = src->deviceInfo;
    callbacks_.pointCloud = src->pointCloud;
  }
  else
  {
    clearLidarSdkCallbacks();
  }
}

/**
 * \brief Clear the Lidar SDK callbacks.
 */
void RSLidarSdkImpl::clearLidarSdkCallbacks() {
  (void)memset(&callbacks_, 0, sizeof(callbacks_));
}

/**
 * \brief Put the MSOP data into the queue.
 * \param[in] data The pointer to the MSOP data.
 * \param[in] size The size of the MSOP data.
 */
void RSLidarSdkImpl::putMsop(const uint8_t* data, uint32_t size) {
  decoder_ptr_->processMsopPkt(data, size);
}

/**
 * \brief Handle the MSOP data.
 */
void RSLidarSdkImpl::handleMsopData() {
  pid_t tid = gettid();
  utils::addThread(tid, "handleMsopData");

  while (is_running_.load(std::memory_order_seq_cst)) {
    LidarPointCloudPtr cloud_ptr;
    if (point_cloud_queue_.popWait(cloud_ptr, 300e3)) { // 300ms超时
      if (callbacks_.pointCloud) {
        auto current_time = callbacks_.getTimeNowPhc();
        static auto last_time = callbacks_.getTimeNowPhc(); // unit: ns, 1e-9 sec
        pointcloud_fps_counter_ptr_->updateFrame(current_time);
        if (current_time - last_time > 1.1e9) { // 1.1秒，用于打印FPS
          LogDebug("(handleMsopData) current_time: {} ns, last_time: {}", current_time, last_time);
          last_time = current_time;
          LogDebug(pointcloud_fps_counter_ptr_->getStatus().c_str());
        }
        auto callback_start = std::chrono::steady_clock::now();
        size_t bufferSize = cloud_ptr->data_length;
        callbacks_.pointCloud(sensor_index_, cloud_ptr.get(), static_cast<uint32_t>(bufferSize));
        auto callback_end = std::chrono::steady_clock::now();
        LogTrace("[pointCloud callback time: {} us", std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start).count());
      }
    }
  }
}

/**
 * \brief Split the frame and send it to the callback.
 * \param[in] point_num The number of points in the frame.
 */
void RSLidarSdkImpl::splitFrame(uint32_t point_num) {
  if (!decoder_ptr_) {
    LogError("decoder_ptr_ is null!");
    decoder_ptr_->resetPointCloud();
    return;
  }
  const uint32_t expected_points = decoder_ptr_->getTheoreticalPointsPerFrame();
  auto* cloud = decoder_ptr_->point_cloud_.get();

  if (point_num == expected_points && cloud && callbacks_.pointCloud) {
    const size_t bufferSize = sizeof(LidarPointCloudPackets) +
                              (1520 - 1) * sizeof(DataBlock); // TODO magic number
    auto current_time = callbacks_.getTimeNowPhc(); ///< unit: ns, 1e-9 sec
    static auto last_time = callbacks_.getTimeNowPhc();

    pointcloud_fps_counter_ptr_->updateFrame(current_time);

    if (current_time - last_time > 110e6) { // 110 ms
      LogDebug("(splitFrame) current_time: {} ns, last_time: {}", current_time, last_time);
      last_time = current_time;
      LogDebug(pointcloud_fps_counter_ptr_->getStatus().c_str());
    }

    // if (std::abs(pointcloud_fps_counter_ptr_->getCurrentFPS() - 10) > 0.2) {
    //   LogDebug("(splitFrame) fps: {:.3f}", pointcloud_fps_counter_ptr_->getCurrentFPS());
    //   LogError(pointcloud_fps_counter_ptr_->getStatus().c_str());
    // }
    LidarPointCloudPackets* cloud_copy = (LidarPointCloudPackets*)malloc(bufferSize);

    if(cloud_copy) {
      bool override = false;
      std::unique_lock<std::mutex> lock(decoder_ptr_->mtx_point_cloud_);
      memcpy(cloud_copy, cloud, bufferSize);
      point_cloud_queue_.push(LidarPointCloudPtr(cloud_copy), override);
    } else {
      LogError("Failed to allocate memory for point cloud copy");
    }
  } else if (point_num != expected_points) {
    LogError("[RSLidarSdkImpl::splitFrame] point_num {}!= expected_points {}", point_num, expected_points);
  } else if (!callbacks_.pointCloud) {
    LogError("[RSLidarSdkImpl::splitFrame] Point cloud callback is null!");
  } else {
    LogError("Cloud data is null!");
  }

  decoder_ptr_->resetPointCloud();
}

/**
 * \brief callback function for device info
 */
void RSLidarSdkImpl::runDeviceInfoCallback() {
  if (!decoder_ptr_) {
    LogError("Decoder pointer is null!");
    return;
  }
  auto* device_info = decoder_ptr_->device_info_.get();

  static bool first_call = true;

  if (device_info && callbacks_.deviceInfo) {
    device_info->sdk_total_fault_number = 0;    // Set total fault number to 0, do NOT report DTC error.
    device_info->sdk_fault_code_position = 0;   // Set fault code position to 0, 0 means no error.
    device_info->sdk_version = sdk_version_encoded_;

    if (first_call) {
      first_call = false;
      LogInfo("FIRWARE VERSION: {}", device_info->firware_version);
      LogInfo("SDK VERSION: {}", device_info->sdk_version);
      
    }

    if (callbacks_.getTimeNowPhc()) {
      device_info_fps_counter_ptr_->updateFrame(callbacks_.getTimeNowPhc());
    }
    callbacks_.deviceInfo(sensor_index_, device_info, static_cast<uint32_t>(sizeof(LidarDeviceInfo)));
    FaultManager::getInstance().clearFaults(~0ULL);  // clear all fault code
  } else {
    LogError(device_info == nullptr ? "Device info callback is null!" : "callbacks_.deviceInfo is null!");
  }
  decoder_ptr_->resetDeviceInfo();
}

/**
 * \brief Handle the fault.
 * \param[in] message The message of the fault.
 * \param[in] fault The fault bits.
 */
void RSLidarSdkImpl::handleFault(const std::string& message, FaultBits fault) {
  LogError("call the handleFault function: fault:{}", fault);
  FaultManager::getInstance().setFault(fault);
  runDeviceInfoCallback();
  LogError(message.c_str());
}

/**
 * \brief Release the buffer.
 * \param[in] buffer The pointer to the buffer.
 */
void RSLidarSdkImpl::releaseBuffer(LidarAdcBuffer* buffer) {
  if (!buffer)
  {
    LogError("releaseBuffer failed, buffer is INVALID.");
    return;
  }
  if (callbacks_.releaseAdc)
  {
    callbacks_.releaseAdc(sensor_index_, buffer);
  }
  else
  {
    LogError("callbacks_.releaseAdc is INVALID.");
  }
}

/**
 * \brief Find the current packet part.
 * \param[in] seq The sequence number of the packet.
 * \param[in] ranges The ranges of the packets.
 * \return The index of the packet part.
 */
int32_t RSLidarSdkImpl::findCurrentPacketPart(uint16_t seq,
                    const std::vector<PacketRangePerMipiFrame>& ranges) {
  for (size_t i = 0; i < ranges.size(); ++i)
    if (seq == ranges[i].start)
      return static_cast<int32_t>(i);
  return -1;
}

/**
 * \brief Check the continuity of the packet.
 * \param[in] pre The previous packet part.
 * \param[in] cur The current packet part.
 * \param[in] diff The difference between the previous and current packet part.
 * \return True if the packet is continuous, false otherwise.
 * \retval true: The packets is continuous.
 * \retval false: The packets is not continuous.
 */
bool RSLidarSdkImpl::checkContinuity(int32_t pre, int32_t cur, int32_t diff) {
  return (pre == -1 || cur == pre + 1 || (cur + diff) == pre);
}

/**
 * \brief Handle the MIPI data.
 */
void RSLidarSdkImpl::handleLidarMipiData() {
  const int32_t msop_size = decoder_ptr_->getMsopLength();
  const int32_t difop_size = decoder_ptr_->getDifopLength();
  const int32_t device_info_size = decoder_ptr_->getDeviceInfoLength();
  const uint16_t max_msop_pkt_diff = 1519;  // 1520 - 1
  const size_t packet_pair_size = msop_size + difop_size + device_info_size;
  const auto& packets_ranges = decoder_ptr_->getPacketRanges();
  const uint32_t max_wait_time = 300000;
  const uint32_t max_process_time = 20;  // unit: ms
  const uint32_t max_data_check_time = 8; // unit: ms
  int32_t pre_packet_part = -1;
  uint16_t last_pkt_seq = 0;
  auto time_now = std::chrono::steady_clock::now();

  pid_t tid = gettid();
  utils::addThread(tid, "handleLidarMipiData");

  while (is_running_.load(std::memory_order_seq_cst)) {
    LidarAdcBuffer* mipi_buffer;

    if (!mipi_data_queue_.popWait(mipi_buffer, max_wait_time)) {
      LogError("LidarSDK recv mipi data timeout");
      continue;
    }
    auto start_time = std::chrono::steady_clock::now();

    if (!mipi_buffer || !mipi_buffer->data) {
      handleFault(mipi_buffer != nullptr ? "mipi_buffer->data is null" : "mipi_buffer is null", FaultBits::DataAbnormalFault);
      releaseBuffer(mipi_buffer);
      continue;
    }
    uint8_t* mipi_data = static_cast<uint8_t*>(mipi_buffer->data);
    size_t buffer_size = mipi_buffer->len;
    uint16_t first_seq = (static_cast<uint16_t>(mipi_data[4]) << 8) | static_cast<uint16_t>(mipi_data[5]);
    int32_t current_part = findCurrentPacketPart(first_seq, packets_ranges);

    // NOTE Print the timestamp in MSOP data
    static uint64_t last_msop_usec{0U};
    static uint64_t last_unix_usec{0U};
    uint32_t msop_sec = (static_cast<uint32_t>(mipi_data[8]) << 24) |
                        (static_cast<uint32_t>(mipi_data[9]) << 16) |
                        (static_cast<uint32_t>(mipi_data[10]) << 8) |
                        static_cast<uint32_t>(mipi_data[11]);
    uint32_t msop_usec = (static_cast<uint32_t>(mipi_data[12]) << 24) |
                        (static_cast<uint32_t>(mipi_data[13]) << 16) |
                        (static_cast<uint32_t>(mipi_data[14]) << 8) |
                        static_cast<uint32_t>(mipi_data[15]);
    uint64_t cur_msop_usec = (static_cast<uint64_t>(msop_sec)) * (1e6) + static_cast<uint64_t>(msop_usec);
    uint64_t msop_diff_usec = cur_msop_usec - last_msop_usec;
    auto host_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
    auto cur_unix_usec = host_time.count();
    auto unix_diff_usec = cur_unix_usec - last_unix_usec;

    last_msop_usec = cur_msop_usec;
    last_unix_usec = cur_unix_usec;

    if (msop_diff_usec > 20e3) { // 20 ms
      LogError("E2E timeout, 20ms");
      LogError("MSOP part: {}, host time: {} us, msop time: {} us, host duration time: {} us, msop duration time: {} us",
        current_part, cur_unix_usec, cur_msop_usec, unix_diff_usec, msop_diff_usec);
    }

    if (current_part == -1) {
      handleFault("Invalid pkt_seq: " + std::to_string(first_seq), FaultBits::DataAbnormalFault);
      pre_packet_part = -1;
      last_pkt_seq = 0;
      releaseBuffer(mipi_buffer);
      continue;
    }
    const uint16_t pkt_num = packets_ranges[current_part].expectedCount;

    if (!checkContinuity(pre_packet_part, current_part, (packets_ranges.size() - 1))) {
      LogError("MSOP part not continuous, last part: {}, current part: {} ", pre_packet_part, current_part);
    }
    pre_packet_part = current_part;
    const size_t expected_size = packet_pair_size * pkt_num;

    if (expected_size > buffer_size) {
      LogError("Buffer size insufficient! Expected at least {}, but got {} bytes", expected_size, buffer_size);
      pre_packet_part = -1;
      last_pkt_seq = 0;
      releaseBuffer(mipi_buffer);
      continue;
    }

    bool ret{false};
    uint8_t* difop_data = mipi_data + msop_size;
    ret = decoder_ptr_->processDifopPkt(difop_data, difop_size);

    if (!ret) {
      LogError("processDifopPkt failed, first_seq: {}", first_seq);
    }
    uint8_t* device_info_data = mipi_data + msop_size + difop_size;
    ret = decoder_ptr_->processDeviceInfoPkt(device_info_data, device_info_size);

    if (!ret) {
      LogError("processDeviceInfoPkt failed, first_seq: {}", first_seq);
    }
    time_now = std::chrono::steady_clock::now();
    auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - start_time);

    if (time_cost.count() > max_data_check_time) {
      LogDebug("cost time: {}", time_cost.count());
    }

    // Send the cloud data in MIPI data to cloud manager
    for (int32_t i = 0; i < pkt_num; i++) {
      const uint8_t* pair_start = mipi_data + i * packet_pair_size;

      if (cloud_manager_ptr_) {
        cloud_manager_ptr_->receiveCloud(pair_start, msop_size);
      }
      uint16_t pkt_seq = (static_cast<uint16_t>(pair_start[4]) << 8) | static_cast<uint16_t>(pair_start[5]);
      if (!checkContinuity(last_pkt_seq, pkt_seq, max_msop_pkt_diff)) {
        LogError("MSOP pkt seq not continuous, last seq is: {}, current seq is: {}, i: {}", last_pkt_seq, pkt_seq, i);
      }
      last_pkt_seq = pkt_seq;
    }
    auto data_check_end = std::chrono::steady_clock::now();
    auto data_check_cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(data_check_end - start_time);

    if (data_check_cost_time.count() > (max_process_time - max_data_check_time)) {
      LogWarn("handleLidarMipiData frame index {} , Data check time: {} ms, exceeds the limit: {} ms", mipi_buffer->frameIndex, data_check_cost_time.count(), (max_process_time - max_data_check_time));
    }
    runDeviceInfoCallback();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - data_check_end);

    if (duration.count() > max_data_check_time) {
      LogError("runDeviceInfoCallback time out, frame index {}, cost time: {} ms", mipi_buffer->frameIndex, duration.count());
    }
    releaseBuffer(mipi_buffer);
  }
}

/**
 * \brief Validate the Lidar SDK callbacks.
 * \return True if the callbacks are valid, false otherwise.
 * \retval true: Callbacks are valid.
 * \retval false: Callbacks are invalid.
 */
bool RSLidarSdkImpl::validateCallbacks() {
  if (!callbacks_.releaseAdc)
  {
    LogError("RSLidarSdkImpl::init: releaseAdc callback is null");
    return false;
  }

  if (!callbacks_.pointCloud)
  {
    LogError("RSLidarSdkImpl::init: pointCloud callback is null");
    return false;
  }

  if (!callbacks_.deviceInfo)
  {
    LogError("RSLidarSdkImpl::init: deviceInfo callback is null");
    return false;
  }

  return true;
}

/**
 * \brief Load the configuration.
 * \param[in] configPath The path of the configuration file.
 * \return True if the configuration is loaded successfully, false otherwise.
 * \retval true: The configuration is loaded successfully.
 * \retval false: The configuration is not loaded successfully.
 */
bool RSLidarSdkImpl::loadConfiguration(const std::string& configPath) {
  LogTrace("Begin of load the inner parameter configuration example ...");

#if defined(__arm__) || defined(__aarch64__)
  bool result{ false };
#else
  bool result{ true };
#endif

  // NOTE Set true cause the inner parameter save function NOT yet fully supported.
  //bool result{ false };
  std::string bin_file_path = configPath + "middle_lidar_inner_para.bin";
  std::string json_file_path = configPath + "lidar_sn_inner_para_crc.json";

  if (true == utils::readBinData(bin_file_path)) {
    LogInfo("read inner parameters from file: {} succeeded", bin_file_path);
  } else {
    LogError("read inner parameters from file: {} failed", bin_file_path);
  }

  if (true != crc32::verify_crc32(bin_file_path, json_file_path)) {
    LogError("verify crc32 failure");
    FaultManager::getInstance().setFault(FaultBits::LidarInternalParamReadFault);
  } else {
    LogInfo("verify crc32 success");
    result = true;
  }
  LogTrace("End of load the inner parameter configuration example ...");

  return result;
}

} // namespace robosense::lidar

/* \}  impl */
