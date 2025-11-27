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
 * | 1.0 | 2025-10-14 | Add function injectAlarmInfo and powerOff and setCalibrationEnable |
 *
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cstring>
#include <fstream>
#include <iostream>
#include <sys/syscall.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "cpu_load.h"
#include "crc32.h"
#include "difop2.h"
#include "json_reader.h"
#include "nvBufObjToCpuBuf.h"
#include "rs_lidar_sdk_impl.h"
#include "rs_new_logger.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "thread_config.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/
static constexpr uint16_t POWEROFF_REQUEST_ADDR {0x0071U};
static constexpr uint16_t POWEROFF_ACK_ADDR     {0x0072U};
static constexpr uint16_t CALIB_REQUEST_ADDR    {0x0073U};
static constexpr uint16_t CALIB_ACK_ADDR        {0x0074U};
static constexpr uint8_t POWEROFF_SIGNAL        {0x01U};

enum class PowerOffStatus : uint8_t {
    NOT_READY = 0x00U,
    READY = 0x01U,
};

enum class CalibSignal : uint8_t {
    DEFAULT = 0x00U,
    ENTER = 0x01U,
    EXIT = 0x02U,
};

enum class CalibSwitchStatus : uint8_t {
    DEFAULT = 0x00U,
    SUCCESS = 0x01U,
    FAILED = 0x02U,
};

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Get the msop usec from mipi data.
 *
 * \param mipi_data Mipi data.
 * \return uint64_t Msop usec.
 */
inline uint64_t getMsopUsec(const uint8_t* mipi_data) noexcept {
    uint32_t msopSec = (static_cast<uint32_t>(mipi_data[8]) << 24) |
                       (static_cast<uint32_t>(mipi_data[9]) << 16) |
                       (static_cast<uint32_t>(mipi_data[10]) << 8) |
                        static_cast<uint32_t>(mipi_data[11]);
    uint32_t msopUsec = (static_cast<uint32_t>(mipi_data[12]) << 24) |
                        (static_cast<uint32_t>(mipi_data[13]) << 16) |
                        (static_cast<uint32_t>(mipi_data[14]) << 8) |
                         static_cast<uint32_t>(mipi_data[15]);
    return (static_cast<uint64_t>(msopSec)) * (1e6) + static_cast<uint64_t>(msopUsec);
}

/**
 * \brief Check the continuity of mipi frame index.
 *
 * \param mipi_index Mipi frame index.
 */
inline void checkMipiIndexContinuity(const uint32_t mipi_index) noexcept {
    static uint32_t last_mipi_index = 0U; ///< Mipi frame index, Range: [0, 4294967295]

    if ((0U != last_mipi_index) && (mipi_index != last_mipi_index + 1)) {
        LogError("Adc frame index not continuous, last index: {}, current index: {}", last_mipi_index, mipi_index);
    }
    last_mipi_index = mipi_index;
}

/**
 * \brief Check the time consumption of mipi data.
 *
 * \param mipi_index Mipi frame index.
 * \param start_time Start time of mipi data.
 * \param end_time End time of mipi data.
 */
inline void
checkMipiTimeConsumption(const uint32_t mipi_index, const uint64_t start_time, const uint64_t end_time) noexcept {
    static uint64_t last_start_time{0UL};
    static uint64_t last_end_time{0UL};
    auto time_consumption{start_time - last_end_time};

    if (time_consumption > 25e3) { // Mipi data duration timeout, 25ms
        LogError("MIPI duration timeout");
        LogWarn("Adc MIPI frame index: {}, last tsof: {}, now tsof:{}, last teof: {}, now teof: {}", mipi_index,
                last_start_time, start_time, last_end_time, end_time);
    }
    last_end_time = end_time;
    last_start_time = start_time;
}

/**
 * \brief Check the continuity of mipi frame counter in E2E.
 *
 * \param mipi_index Mipi frame index.
 * \param mipi_data Mipi data.
 * \param mipi_data_len Mipi data length.
 */
inline void
checkMipiCounterContinuity(const uint32_t mipi_index, const uint8_t* mipi_data, uint32_t mipi_data_len) noexcept {
    static uint16_t last_msop_counter{0U}; ///< MSOP frame counter, Range: [0, 65535]
    uint16_t current_msop_counter = (static_cast<uint16_t>(mipi_data[mipi_data_len - 9]) << 8) |
                                     static_cast<uint16_t>(mipi_data[mipi_data_len - 10]);

    if ((0U != last_msop_counter) && (current_msop_counter != ++last_msop_counter)) {
        LogError("MIPI frame counter not continuous, MIPI frame index: {}, last counter: {}, current counter: {}",
                    mipi_index, --last_msop_counter, current_msop_counter);
        uint64_t current_msop_usec = getMsopUsec(mipi_data);
        LogError("MIPI frame counter: {}, MSOP timestamp: {} us", current_msop_counter, current_msop_usec);
    }
    last_msop_counter = current_msop_counter;
}

/**
 * \brief Check the CRC32 of mipi data.
 *
 * \param mipi_data Mipi data.
 * \param mipi_data_len Mipi data length.
 */
inline void checkMipiCrc(const uint8_t* mipi_data, const uint32_t mipi_data_len) {
    static uint32_t right_cnt{0U};
    static uint32_t error_cnt{0U};
    uint32_t calculate_crc = crc32::calculate_crc32(mipi_data, mipi_data_len - 4);
    uint32_t expected_crc = mipi_data[mipi_data_len - 1] << 24 | mipi_data[mipi_data_len - 2] << 16 |
                            mipi_data[mipi_data_len - 3] << 8 | mipi_data[mipi_data_len - 4];

    if (calculate_crc != expected_crc) {
        ++error_cnt;
        LogError("Expected CRC: 0x{:8x}, Calculate CRC: 0x{:8x}, cnt: {}", expected_crc, calculate_crc,
                    (static_cast<uint16_t>(mipi_data[mipi_data_len - 9]) << 8) |
                        static_cast<uint16_t>(mipi_data[mipi_data_len - 10]));
        LogError("right count: {}, error count: {}, error rate: {:6f}", right_cnt, error_cnt,
                    static_cast<double>(error_cnt) / static_cast<double>(error_cnt + right_cnt));
    } else {
        ++right_cnt;
    }

    if (0 == (right_cnt + error_cnt) % 600) {
        LogInfo("right count: {}, error count: {}, error rate: {:6f}", right_cnt, error_cnt,
                static_cast<double>(error_cnt) / static_cast<double>(error_cnt + right_cnt));
    }
}

inline void noop(const uint8_t* mipi_data, const uint32_t mipi_data_len) noexcept {
    // Do nothing
}


/******************************************************************************/
/*          Definition of public functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Constructor of RSLidarSdkImpl class.
 */
RSLidarSdkImpl::RSLidarSdkImpl() :
        point_cloud_queue_(POINT_CLOUD_QUEUE_SIZE, [](LidarPointCloudPtr&& ptr) {}),
        mipi_data_queue_(MIPI_DATA_QUEUE_SIZE, [this](MipiFramePtr data) {}) {
    // Do other initialization if needed
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

    std::snprintf(version, sizeof(version), "%d.%02d.%02d", RSLIDARSDK_VERSION_MAJOR, RSLIDARSDK_VERSION_MINOR,
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
LidarSdkErrorCode RSLidarSdkImpl::init(const LidarSdkCbks* fptrCbks, const char* configPath) {
    std::lock_guard<std::mutex> lock(mutex_);
    RSLogConfig log_config;
    log_config.type_ = LoggerType::ASYNC_FILE;
    log_config.level_ = LogLevel::INFO;
#if defined(__arm__) || defined(__aarch64__)
    log_config.log_file_path_ = "/applog/lidar/rs_lidar_sdk.log";
    config_path_ = "/osdata/satelite/lidar/";
#else
    log_config.log_file_path_ = "./rs_lidar_sdk.log";
    config_path_ = "./";
#endif
    log_config.max_file_size_mb_ = 6;
    log_config.max_files_ = 3;

    std::vector<json_reader::DataVariant> config_data = json_reader::parseJsonFile(configPath);

    log_config.log_file_path_ = mpark::get<std::string>(config_data[2]);
    InitLog(log_config);
    json_reader::DataVariantVisitor visitor;

    for (const auto& var : config_data) {
        std::string str = mpark::visit(visitor, var);
        LogInfo(str.c_str());
    }
    bool enable_mipi_crc = mpark::get<bool>(config_data[0]);
    config_path_ = mpark::get<std::string>(config_data[1]);
    log_config.log_file_path_ = mpark::get<std::string>(config_data[2]);
    thread::thread_params = mpark::get<std::vector<thread::ThreadConfig>>(config_data[3]);

    funcCheckMipiCrc_ = enable_mipi_crc ? checkMipiCrc : noop;

    if (is_initialized_.load(std::memory_order_seq_cst)) {
        LogWarn("LidarSdk has been initialized!");
        return LIDAR_SDK_SUCCESS;
    }
    if (!fptrCbks) {
        LogError("callbacks is null!");
        return LIDAR_SDK_FAILD;
    }
    if (!loadConfiguration(config_path_)) {
        LogError("load configuration failed!");
        return LIDAR_SDK_FAILD;
    }
    LogInfo("load configuration succeeded!");

    if (!setLidarSdkCallbacks(fptrCbks)) {
        LogError("Set lidar SDK callbacks failed!");
        clearLidarSdkCallbacks();
        return LIDAR_SDK_FAILD;
    }
    LogInfo("set lidar SDK callbacks succeeded!");

    RSDecoderParam param;
    decoder_ptr_ = std::make_shared<DecoderRSEMX>(param);

    if (nullptr == decoder_ptr_) {
        LogError("create decoder failed! LidarType: {}", static_cast<int32_t>(param.lidar_type));
        return LIDAR_SDK_FAILD;
    }
    if (!decoder_ptr_->init()) {
        LogError("decoder init failed!");
        return LIDAR_SDK_FAILD;
    }
    decoder_ptr_->regCallback(std::bind(&RSLidarSdkImpl::splitFrame, this, std::placeholders::_1));
    cloud_manager_ptr_ = std::make_shared<CloudManager>();
    cloud_manager_ptr_->regCallback(
        std::bind(&RSLidarSdkImpl::putMsop, this, std::placeholders::_1, std::placeholders::_2));
    pointcloud_fps_counter_ptr_.reset(new FPSCounter("PointCloud", 10));
    device_info_fps_counter_ptr_.reset(new FPSCounter("DeviceInfo", 60));
    is_initialized_.store(true, std::memory_order_seq_cst);
    LogInfo("LidarSdk initialized successfully! Version: {}", kSdkVersionEncoded);

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
    if (!is_initialized_.load(std::memory_order_seq_cst)) {
        return LIDAR_SDK_SUCCESS;
    }
    LogInfo("Begin to deInit lidar sdk");
    LidarSdkErrorCode ret_code = LIDAR_SDK_SUCCESS;

    if (is_started_.load(std::memory_order_seq_cst)) {
        if (stop() != LIDAR_SDK_SUCCESS) {
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
    if (!is_initialized_.load(std::memory_order_seq_cst)) {
        LogError("LidarSdk start failed, please call init() first");
        return LIDAR_SDK_FAILD;
    }

    if (is_started_.load(std::memory_order_seq_cst)) {
        LogWarn("LidarSdk is already started");
        return LIDAR_SDK_SUCCESS;
    }
    LidarSdkErrorCode ret = LIDAR_SDK_FAILD;

    try {
        is_running_.store(true, std::memory_order_seq_cst);
        LogInfo("Creating data handling thread...");
        thread::ThreadConfig handle_mipi_config;

        if (thread::thread_params.size() > 0) {
            handle_mipi_config = thread::thread_params[0];
        }
        handle_lidar_mipi_data_thread_ = createConfiguredStdThread(handle_mipi_config, [this] {
            int32_t ret = pthread_setname_np(pthread_self(), "RS-HandleMIPI");

            if (ret != 0) {
                LogError("Failed to set thread name: %d", ret);
            }
            this->handleLidarMipiData();
        });
        thread::ThreadConfig handle_msop_config;

        if (thread::thread_params.size() > 0) {
            handle_msop_config = thread::thread_params[1];
        }
        handle_process_msop_data_thread_ = createConfiguredStdThread(handle_msop_config, [this] {
            int32_t ret = pthread_setname_np(pthread_self(), "RS-HandleMsop");
            if (ret != 0) {
                LogError("Failed to set thread name: %d", ret);
            }
            this->handleMsopData();
        });

        if (cloud_manager_ptr_) {
            cloud_manager_ptr_->start();
        }
        is_started_.store(true, std::memory_order_seq_cst);
        ret = LIDAR_SDK_SUCCESS;
    } catch (const std::exception& e) {
        LogError("Failed to start lidar thread: {}", e.what());
        is_running_.store(false, std::memory_order_seq_cst);
        if (handle_lidar_mipi_data_thread_.joinable()) {
            try {
                handle_lidar_mipi_data_thread_.detach();
                LogInfo("Detached orphaned mipi data handling thread");
            } catch (...) {
                LogError("Failed to detach orphaned mipi data thread during start failure cleanup");
            }
        }

        if (handle_process_msop_data_thread_.joinable()) {
            handle_process_msop_data_thread_.join();
        } else {
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
    if (!is_started_.load(std::memory_order_seq_cst)) {
        LogWarn("LidarSdk is not started");
        return LIDAR_SDK_SUCCESS;
    }
    LogInfo("Begin to stop LidarSdk");
    std::lock_guard<std::mutex> lock(mutex_);
    LidarSdkErrorCode ret{LIDAR_SDK_FAILD};
    auto start_time = std::chrono::steady_clock::now();

    is_running_.store(false, std::memory_order_seq_cst);
    mipi_data_queue_.stopWait();
    mipi_data_queue_.reset();

    if (handle_lidar_mipi_data_thread_.joinable()) {
        handle_lidar_mipi_data_thread_.join();
    } else {
        LogError("handle_lidar_mipi_data_thread_ is not joinable");
    }

    if (handle_process_msop_data_thread_.joinable()) {
        handle_process_msop_data_thread_.join();
    } else {
        LogError("handle_process_msop_data_thread_ is not joinable");
    }

    mipi_data_queue_.clear();
    if (cloud_manager_ptr_) {
        if (cloud_manager_ptr_->stop()) {
            ret = LIDAR_SDK_SUCCESS;
            LogInfo("Cloud manager stopped successfully");
        } else {
            LogError("SDK Stop cloud manager thread failed");
            ret = LIDAR_SDK_FAILD;
        }
    } else {
        LogWarn("Cloud manager pointer is null");
    }

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    is_started_.store(false, std::memory_order_seq_cst);
    LogInfo("Completed stopping LidarSdk (total time: {} ms, result: {})", duration.count(),
                ((ret == LIDAR_SDK_SUCCESS) ? "SUCCESS" : "FAILURE"));

    return ret;
}

/**
 * \brief Power off the Lidar SDK.
 * \return The error code of the power off.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Power off succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Power off failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::powerOff() {
    uint16_t data_length{0x01U};
    PowerOffStatus status{PowerOffStatus::NOT_READY};
    LidarSdkErrorCode result{LIDAR_SDK_FAILD};
    auto start_time{std::chrono::steady_clock::now()};
    int64_t duration{0};

    if (callbacks_.writeSensorI2C) {
        callbacks_.writeSensorI2C(sensor_index_, POWEROFF_REQUEST_ADDR, &POWEROFF_SIGNAL, data_length);
    } else {
        LogError("callbacks_.writeSensorI2C is INVALID.");
        return result;
    }

    if (callbacks_.readSensorI2C) {
        callbacks_.readSensorI2C(sensor_index_, POWEROFF_ACK_ADDR, reinterpret_cast<uint8_t*>(&status), &data_length);

        while (PowerOffStatus::READY != status) {
            (void)usleep(50e3); // sleep 50ms
            auto end_time{std::chrono::steady_clock::now()};
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            if (duration > 1800) { // wait 1.8s
                LogError("Time out, register: 0x{:x} is 0x{:x}, It's NOT ready for safely power off.", POWEROFF_ACK_ADDR, static_cast<uint32_t>(status));
                return result;
            }
            callbacks_.readSensorI2C(sensor_index_, POWEROFF_ACK_ADDR, reinterpret_cast<uint8_t *>(&status), &data_length);
        }
        result = LIDAR_SDK_SUCCESS;
        LogInfo("PowerOff register: 0x{:x} is 0x{:x}, took {} ms", POWEROFF_ACK_ADDR, static_cast<uint32_t>(status), duration);
    } else {
        LogError("callbacks_.readSensorI2C is INVALID.");
    }

    return result;
}

/**
 * \brief Set the calibration enable or disable.
 * \param[in] enabled
 *  true: enable calibration,   reg0x73 write 0x1, after 600ms read reg 0x74
 *  false: disable calibration, reg0x73 write 0x2, after 600ms read reg 0x74
 * \return The error code of the set calibration enable.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Switch calibration mode succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Switch calibration mode failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::setCalibrationEnable(const bool enabled) {
    uint16_t data_length{0x01U};
    CalibSwitchStatus status{CalibSwitchStatus::DEFAULT};
    CalibSignal signal{CalibSignal::DEFAULT};
    LidarSdkErrorCode result{LIDAR_SDK_FAILD};
    auto start_time{std::chrono::steady_clock::now()};
    int64_t duration{0};

    if (!callbacks_.writeSensorI2C) {
        LogError("callbacks_.writeSensorI2C is INVALID, set calibration failed.");
        return result;
    }
    signal = enabled ? CalibSignal::ENTER : CalibSignal::EXIT;
    callbacks_.writeSensorI2C(sensor_index_, CALIB_REQUEST_ADDR, reinterpret_cast<const uint8_t*>(&signal), data_length);

    if (!callbacks_.readSensorI2C) {
        LogError("callbacks_.readSensorI2C is INVALID. set calibration failed.");
        return result;
    }
    (void)usleep(600e3); // sleep 600ms

    callbacks_.readSensorI2C(sensor_index_, CALIB_ACK_ADDR, reinterpret_cast<uint8_t*>(&status), &data_length);
    auto end_time{std::chrono::steady_clock::now()};
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    LogInfo("Calibration register: {:x} is {:x}. Took {} ms", CALIB_ACK_ADDR, static_cast<uint32_t>(status), duration);
    result = (CalibSwitchStatus::SUCCESS == status) ? LIDAR_SDK_SUCCESS : LIDAR_SDK_FAILD;

    return result;
}

/**
 * \brief Inject ADC data to the Lidar SDK.
 * \param[in] sensor The sensor index.
 * \param[in] ptrAdc The pointer to the ADC data.
 * \return The error code of the inject.
 * \retval LidarSdkErrorCode::LIDAR_SDK_SUCCESS: Injection succeeded.
 * \retval LidarSdkErrorCode::LIDAR_SDK_FAILD: Injection failed.
 */
LidarSdkErrorCode RSLidarSdkImpl::injectAdc(LidarSensorIndex sensor, const void* ptrAdc) {
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
        return LIDAR_SDK_FAILD;
    }
    static const uint32_t expected_len = decoder_ptr_->getTheoreticalMipiDataLength();
    LidarAdcBuffer* adc_buffer = (LidarAdcBuffer*)ptrAdc;
    MipiFramePtr mipi_frame = std::make_shared<MipiFrame>();

    mipi_frame->data = nullptr;
    mipi_frame->len = 0U;
    mipi_frame->index = adc_buffer->frameIndex;

#if defined(__arm__) || defined(__aarch64__)
    using ClientErrorCode = dimw::cameraipcclient::ClientErrorCode;
    using dimw::gpuBuf::nvBufObjToCpuBuffer;
    NvSciBufObj gpu_buf_obj = static_cast<NvSciBufObj>(adc_buffer->bufObj);
    ClientErrorCode error_code = nvBufObjToCpuBuffer(gpu_buf_obj, mipi_frame->data, mipi_frame->len);

    if (ClientErrorCode::CLIENT_SUCCESS != error_code) {
        LogError("nvBufObjToCpuBuffer failed, error_code: {}", static_cast<int32_t>(error_code));
        return LIDAR_SDK_FAILD;
    }
#else
    mipi_frame->data = (uint8_t*)adc_buffer->data;
    mipi_frame->len = adc_buffer->len;
#endif

    if (nullptr == mipi_frame->data) {
        LogError("[injectAdc] Mipi data is null, release adc buffer.");
        releaseBuffer(adc_buffer);
        runDeviceInfoCallback();
        return LIDAR_SDK_FAILD;
    }

    if (expected_len != mipi_frame->len) {
        LogError("Invalid ADC data length, expected {}, got {}, release adc buffer.", expected_len, mipi_frame->len);
        releaseBuffer(adc_buffer);
        runDeviceInfoCallback();
        return LIDAR_SDK_FAILD;
    }

    checkMipiIndexContinuity(mipi_frame->index);
    checkMipiTimeConsumption(mipi_frame->index, adc_buffer->tsof, adc_buffer->teof);
    checkMipiCounterContinuity(mipi_frame->index, mipi_frame->data, mipi_frame->len);
    funcCheckMipiCrc_(mipi_frame->data, mipi_frame->len);
    sensor_index_ = sensor;

    bool overflow = false;
    size_t sz = mipi_data_queue_.push(mipi_frame, overflow);

    if (overflow) {
        FaultManager64::getInstance().overflow_position_ |= 0x1;
        LogError("mipi_data_queue_ is full, drop one packet, sz:{}", sz);
        FaultManager64::getInstance().setFault(FaultBits::LidarPointCloudBufferOverflowFault);
    } else {
        FaultManager64::getInstance().overflow_position_ &= 0x6;
    }
    releaseBuffer(adc_buffer);

    return LIDAR_SDK_SUCCESS;
}

/**
 * \brief Inject Alarm data to the Lidar SDK.
 * \param[in] lidarAlarmInfo The Alarm info.
 */
LidarSdkErrorCode RSLidarSdkImpl::injectAlarmInfo(const LidarAlarmInfo& lidarAlarmInfo) {
    LidarSdkErrorCode result = LidarSdkErrorCode::LIDAR_SDK_FAILD;

    if (((lidarAlarmInfo.alarmId == 20105) || (lidarAlarmInfo.alarmId == 20106) ||
         (lidarAlarmInfo.alarmId == 20107) || (lidarAlarmInfo.alarmId == 20119)) &&
        (lidarAlarmInfo.alarmObj == 0)) {

        if (lidarAlarmInfo.status == 1) {
            alarm_status_ = true;
        } else {
            alarm_status_ = false;
        }
        result = LidarSdkErrorCode::LIDAR_SDK_SUCCESS;
    }

    return result;
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
LidarSdkErrorCode RSLidarSdkImpl::writeDid(uint16_t did, uint8_t* data, uint16_t dataLen, uint8_t* nrc) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_started_.load(std::memory_order_seq_cst)) {
        LogError("Write Did failed: LidarSdk is not started");
        return LIDAR_SDK_FAILD;
    }

    if (!data) {
        LogError("write Did failed: data is null");
        return LIDAR_SDK_FAILD;
    }

    if (0 == dataLen) {
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
LidarSdkErrorCode RSLidarSdkImpl::readDid(uint16_t did, uint8_t* data, uint16_t* dataLen, uint8_t* nrc) {
    static const uint16_t READ_SDK_VERSION_DID = 0x1112U;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_started_.load(std::memory_order_seq_cst)) {
        LogError("Read Did failed: LidarSdk is not started");
        return LIDAR_SDK_FAILD;
    }

    if (!data || !dataLen) {
        LogError("Read Did failed: data or dataLen is null");
        return LIDAR_SDK_FAILD;
    }

    // 实现DID读取逻辑
    if (READ_SDK_VERSION_DID == did) {
        (void)std::memcpy(data, &kSdkVersionEncoded, sizeof(kSdkVersionEncoded));
        *dataLen = sizeof(kSdkVersionEncoded);
        *nrc = static_cast<uint8_t>(POSITIVE);
    } else {
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
LidarSdkErrorCode RSLidarSdkImpl::ridControl(uint8_t mode, uint16_t rid, uint8_t* dataIn, uint16_t dataInLen,
                                             uint8_t* dataOut, uint16_t* dataOutLen, uint8_t* nrc) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_started_.load(std::memory_order_seq_cst)) {
        LogError("RSLidarSdkImpl::ridControl: Lidar is not started");
        return LIDAR_SDK_FAILD;
    }

    // 实现RID控制逻辑
    return LIDAR_SDK_SUCCESS;
}

/**
 * \brief Get the alarm status of the Lidar SDK.
 * \return The alarm status of the Lidar SDK.
 */
bool RSLidarSdkImpl::getAlarmInfo() {
    return alarm_status_;
}

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/
/**
 * \brief Set the Lidar SDK callbacks.
 * \param[in] src The pointer of callbacks structure to the Lidar SDK.
 */
bool RSLidarSdkImpl::setLidarSdkCallbacks(const LidarSdkCbks* src) {
    if (!src) {
        LogError("callbacks is null!");
        return false;
    }

    if (!src->releaseAdc) {
        LogError("RSLidarSdkImpl::init: releaseAdc callback is null");
        return false;
    }
    callbacks_.releaseAdc = src->releaseAdc;

    if (!src->getSensorFsyncStartTime) {
        LogError("RSLidarSdkImpl::init: getSensorFsyncStartTime callback is null");
        return false;
    }
    callbacks_.getSensorFsyncStartTime = src->getSensorFsyncStartTime;

    if (!src->getTimeNowPhc) {
        LogError("RSLidarSdkImpl::init: getTimeNowPhc callback is null");
        return false;
    }
    callbacks_.getTimeNowPhc = src->getTimeNowPhc;

    if (!src->getTimeNowSys) {
        LogError("RSLidarSdkImpl::init: getTimeNowSys callback is null");
        return false;
    }
    callbacks_.getTimeNowSys = src->getTimeNowSys;

    if (!src->getTimeNowMgr) {
        LogError("RSLidarSdkImpl::init: getTimeNowMgr callback is null");
        return false;
    }
    callbacks_.getTimeNowMgr = src->getTimeNowMgr;

    if (!src->writeSensorRegUint8) {
        LogError("RSLidarSdkImpl::init: writeSensorRegUint8 callback is null");
        return false;
    }
    callbacks_.writeSensorRegUint8 = src->writeSensorRegUint8;

    if (!src->readSensorRegUint8) {
        LogError("RSLidarSdkImpl::init: readSensorRegUint8 callback is null");
        return false;
    }
    callbacks_.readSensorRegUint8 = src->readSensorRegUint8;

    if (!src->writeSensorRegUint16) {
        LogError("RSLidarSdkImpl::init: writeSensorRegUint16 callback is null");
        return false;
    }
    callbacks_.writeSensorRegUint16 = src->writeSensorRegUint16;

    if (!src->readSensorRegUint16) {
        LogError("RSLidarSdkImpl::init: readSensorRegUint16 callback is null");
        return false;
    }
    callbacks_.readSensorRegUint16 = src->readSensorRegUint16;

    if (!src->readSensorI2C) {
        LogError("RSLidarSdkImpl::init: readSensorI2C callback is null");
        return false;
    }
    callbacks_.readSensorI2C = src->readSensorI2C;

    if (!src->writeSensorI2C) {
        LogError("RSLidarSdkImpl::init: writeSensorI2C callback is null");
        return false;
    }
    callbacks_.writeSensorI2C = src->writeSensorI2C;

    if (!src->deviceInfo) {
        LogError("RSLidarSdkImpl::init: deviceInfo callback is null");
        return false;
    }
    callbacks_.deviceInfo = src->deviceInfo;

    if (!src->pointCloud) {
        LogError("RSLidarSdkImpl::init: pointCloud callback is null");
        return false;
    }
    callbacks_.pointCloud = src->pointCloud;

    return true;
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
        LidarPointCloudPtr cloud_ptr {nullptr, {}};

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
                LogTrace("[pointCloud callback time: {} us",
                         std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start).count());
            }
        }
    }
}

/**
 * \brief Deepcopy function of the struct LidarPointCloudPackets.
 */
RSLidarSdkImpl::LidarPointCloudPtr RSLidarSdkImpl::deepCopyLidarCloud(const LidarPointCloudPackets* src) {
    if (!src) {
        return {nullptr, {}};
    }

    // Compute the total size of the required memory
    size_t bufferSize = sizeof(LidarPointCloudPackets) + (src->packet_num - 1) * sizeof(DataBlock);

    // Malloc memory
    LidarPointCloudPackets* copy = static_cast<LidarPointCloudPackets*>(malloc(bufferSize));
    if (!copy) {
        LogError("Failed to allocate memory for point cloud copy");
        return {nullptr, {}};
    }

    // Copy the value of all the members of the struct LidarPointCloudPackets
    (void)memcpy(copy, src, bufferSize);

    // Deepcopy the value pointed by the lidar_parameter pointer
    if (src->lidar_parameter && src->lidar_parameter_length > 0) {
        copy->lidar_parameter = malloc(src->lidar_parameter_length);
        if (!copy->lidar_parameter) {
            free(copy);
            LogError("Failed to allocate memory for lidar parameter copy");
            return {nullptr, {}};
        }
        (void)memcpy(copy->lidar_parameter, src->lidar_parameter, src->lidar_parameter_length);
    } else {
        copy->lidar_parameter = nullptr;
        copy->lidar_parameter_length = 0;
    }

    // return the unique_ptr with a proper deleter
    return RSLidarSdkImpl::LidarPointCloudPtr(copy);
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
        auto current_time = callbacks_.getTimeNowPhc(); ///< unit: ns, 1e-9 sec
        static auto last_time = callbacks_.getTimeNowPhc();

        pointcloud_fps_counter_ptr_->updateFrame(current_time);

        if (current_time - last_time > 110e6) { // 110 ms
            LogDebug("(splitFrame) current_time: {} ns, last_time: {}", current_time, last_time);
            last_time = current_time;
            LogDebug(pointcloud_fps_counter_ptr_->getStatus().c_str());
        }

        LidarPointCloudPtr cloud_copy = deepCopyLidarCloud(cloud);

        if (cloud_copy) {
            bool override = false;

            (void)point_cloud_queue_.push(std::move(cloud_copy), override);
            if (override) {
                FaultManager64::getInstance().overflow_position_ |= 0x2;
                FaultManager64::getInstance().setFault(FaultBits::LidarPointCloudBufferOverflowFault);
            } else {
                FaultManager64::getInstance().overflow_position_ &= 0x5;
                if (FaultManager64::getInstance().hasFault(FaultBits::LidarPointCloudBufferOverflowFault) &&
                    ((FaultManager64::getInstance().overflow_position_ & 0x7) == 0)) {
                    FaultManager64::getInstance().clearFault(FaultBits::LidarPointCloudBufferOverflowFault);
                }
            }
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

    int32_t sdk_fault_state = FaultManager64::getInstance().getFaultLevel();
    int32_t fault_state = static_cast<int32_t>(device_info->lidar_fault_state);
    fault_state = (getAlarmInfo() == true) ? 2 : fault_state;
    device_info->lidar_fault_state = std::max(fault_state, sdk_fault_state); // TODO determine return u32 or u8 ?
    device_info->sdk_total_fault_number = FaultManager64::getInstance().countFaults();
    device_info->sdk_fault_code_position = FaultManager64::getInstance().getFaults();
    device_info->reserved[10] = FaultManager8::getInstance().getFaults();

    static bool first_call = true;

    if (device_info && callbacks_.deviceInfo) {
        device_info->sdk_version = kSdkVersionEncoded;

        if (first_call) {
            first_call = false;
            LogInfo("FIRWARE VERSION: {}", device_info->firware_version);
            LogInfo("SDK VERSION: {}", device_info->sdk_version);
        }

        if (callbacks_.getTimeNowPhc()) {
            device_info_fps_counter_ptr_->updateFrame(callbacks_.getTimeNowPhc());
        }
        callbacks_.deviceInfo(sensor_index_, device_info, static_cast<uint32_t>(sizeof(LidarDeviceInfo)));
        FaultManager64::getInstance().clearFaults(~0ULL); // clear all fault code
        FaultManager8::getInstance().clearFaults(0xFF);   // clear all fault code
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
    FaultManager64::getInstance().setFault(fault);
    LogError(message.c_str());
}

/**
 * \brief Release the buffer.
 * \param[in] buffer The pointer to the buffer.
 */
void RSLidarSdkImpl::releaseBuffer(LidarAdcBuffer* buffer) {
    if (!buffer) {
        LogError("releaseBuffer failed, buffer is INVALID.");
        return;
    }
    if (callbacks_.releaseAdc) {
        callbacks_.releaseAdc(sensor_index_, buffer);
    } else {
        LogError("callbacks_.releaseAdc is INVALID.");
    }
}

/**
 * \brief Find the current packet part.
 * \param[in] seq The sequence number of the packet.
 * \param[in] ranges The ranges of the packets.
 * \return The index of the packet part.
 */
int32_t RSLidarSdkImpl::findCurrentPacketPart(uint16_t seq, const std::vector<PacketRangePerMipiFrame>& ranges) {
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
    static const int32_t kMsopSize = decoder_ptr_->getMsopLength();
    static const int32_t kDifopSize = decoder_ptr_->getDifopLength();
    static const int32_t kDeviceInfoSize = decoder_ptr_->getDeviceInfoLength();
    static const uint16_t kMaxMsopPktDiff{1519U}; // 1520 - 1
    static const size_t kPacketPairSize = kMsopSize + kDifopSize + kDeviceInfoSize;
    static const auto& kPacketsRanges = decoder_ptr_->getPacketRanges();
    static const uint32_t kMaxWaitTimeUs{300000U};
#ifdef MIPI_10HZ
    static const uint32_t kMaxProcessTimeMs{30U};
#else
    static const uint32_t kMaxProcessTimeMs{20U};
#endif
    static const uint32_t kDeviceInfoProcessTimeMs{8U};
    static const uint32_t kMsopProcessTimeMs{kMaxProcessTimeMs - kDeviceInfoProcessTimeMs};
    int32_t pre_packet_part{-1};
    uint16_t last_pkt_seq{0U};
    auto time_now = std::chrono::steady_clock::now();
    pid_t tid = gettid();
    utils::addThread(tid, "handleLidarMipiData");

    while (is_running_.load(std::memory_order_seq_cst)) {
        MipiFramePtr mipi_frame;

        if (!mipi_data_queue_.popWait(mipi_frame, kMaxWaitTimeUs)) {
            LogError("LidarSDK recv mipi data timeout");
            FaultManager64::getInstance().setFault(FaultBits::LidarMIPIPacketLossFault);
            runDeviceInfoCallback();
            frm_normal_detected_ = false;
            frm_normal_reported_ = true;
            continue;
        }
        auto start_time = std::chrono::steady_clock::now();

        if (!mipi_frame || !mipi_frame->data) {
            LogError("LidarSDK recv mipi data is null");
            continue;
        }
        constexpr int32_t lostfrm_report_time_{1000};

        if (!frm_normal_detected_) {
            frm_normal_start_time_ = start_time;
            frm_normal_detected_ = true;
        } else if (frm_normal_reported_) {
            auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(start_time - frm_normal_start_time_);

            if (time_cost.count() > lostfrm_report_time_) {
                if (FaultManager64::getInstance().hasFault(FaultBits::LidarMIPIPacketLossFault)) {
                    FaultManager64::getInstance().clearFault(FaultBits::LidarMIPIPacketLossFault);
                    frm_normal_reported_ = false;
                }
            }
        }
        uint8_t* mipi_data = mipi_frame->data;
        size_t frame_size = mipi_frame->len;
#ifdef MIPI_10HZ
        for (uint8_t part_index{0U}; part_index < kPacketsRanges.size(); ++part_index) {
            uint8_t* part_data = mipi_data + part_index * EMX_MIPI_PART_LEN;
#else
        uint8_t* part_data = mipi_data;
#endif
        uint16_t first_seq = (static_cast<uint16_t>(part_data[4]) << 8) | static_cast<uint16_t>(part_data[5]);
        int32_t current_part = findCurrentPacketPart(first_seq, kPacketsRanges);

        if (current_part == -1) {
            pre_packet_part = -1;
            last_pkt_seq = 0;
            continue;
        }
        const uint16_t pkt_num = kPacketsRanges[current_part].expectedCount;

        if (!checkContinuity(pre_packet_part, current_part, (kPacketsRanges.size() - 1))) {
            lostpkg[cur_rear_] = true;
            LogError("MSOP part not continuous, last part: {}, current part: {} ", pre_packet_part, current_part);
            bool condition1{false};

            for (uint8_t i = 6; i < 12; i++) {
                condition1 = (condition1) || (lostpkg[(cur_rear_ - i + 18) % 18]);
            }
            if (condition1) {
                FaultManager64::getInstance().setFault(FaultBits::DataAbnormalFault);
            }
            cur_rear_ = (cur_rear_ + 1) % 18;
        } else {
            lostpkg[cur_rear_] = false;
            bool condition2{false};

            for (uint8_t i = 0; i < 12; i++) {
                condition2 = (condition2) || (lostpkg[(cur_rear_ - i + 18) % 18]);
            }
            if (!condition2) {
                if (FaultManager64::getInstance().hasFault(FaultBits::DataAbnormalFault)) {
                    FaultManager64::getInstance().clearFault(FaultBits::DataAbnormalFault);
                }
            }
            cur_rear_ = (cur_rear_ + 1) % 18;
        }
        pre_packet_part = current_part;
        const size_t expected_size = kPacketPairSize * pkt_num;

        if (expected_size > frame_size) {
            LogError("Buffer size insufficient! Expected at least {}, but got {} bytes", expected_size, frame_size);
            pre_packet_part = -1;
            last_pkt_seq = 0;
            continue;
        }
        bool ret{false};
        uint8_t* difop_data = part_data + kMsopSize;
        ret = decoder_ptr_->processDifopPkt(difop_data, kDifopSize);

        if (!ret) {
            LogError("processDifopPkt failed, first_seq: {}", first_seq);
        }
        uint8_t* device_info_data = part_data + kMsopSize + kDifopSize;
        ret = decoder_ptr_->processDeviceInfoPkt(device_info_data, kDeviceInfoSize);

        if (!ret) {
            LogError("processDeviceInfoPkt failed, first_seq: {}", first_seq);
        }
        time_now = std::chrono::steady_clock::now();
        auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - start_time);

        if (time_cost.count() > kDeviceInfoProcessTimeMs) {
            LogDebug("Process difop and device info data cost time: {}", time_cost.count());
        }

        // Send the cloud data in MIPI data to cloud manager
        for (int32_t i = 0; i < pkt_num; i++) {
            const uint8_t* pair_start = part_data + i * kPacketPairSize;

            if (cloud_manager_ptr_) {
                cloud_manager_ptr_->receiveCloud(pair_start, kMsopSize);
            }
            uint16_t pkt_seq = (static_cast<uint16_t>(pair_start[4]) << 8) | static_cast<uint16_t>(pair_start[5]);
            if (!checkContinuity(last_pkt_seq, pkt_seq, kMaxMsopPktDiff)) {
                LogError("MSOP pkt seq not continuous, last seq is: {}, current seq is: {}, i: {}", last_pkt_seq,
                         pkt_seq, i);
            }
            last_pkt_seq = pkt_seq;
        }
        auto data_check_end = std::chrono::steady_clock::now();
        auto data_check_cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(data_check_end - start_time);

        if (data_check_cost_time.count() > kMsopProcessTimeMs) {
            LogWarn("handleLidarMipiData frame index {} , Data check time: {} ms, exceeds the limit: {} ms",
                    mipi_frame->index, data_check_cost_time.count(), kMsopProcessTimeMs);
        }
        runDeviceInfoCallback();

        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - data_check_end);

        if (duration.count() > kDeviceInfoProcessTimeMs) {
            LogError("runDeviceInfoCallback time out, frame index {}, cost time: {} ms", mipi_frame->index,
                     duration.count());
        }
    }
#ifdef MIPI_10HZ
    }
#endif
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
    // NOTE Set true cause the inner parameter save function NOT yet fully supported.
#if defined(__arm__) || defined(__aarch64__)
    bool result{false};
#else
    bool result{true};
#endif
    std::string bin_file_path = configPath + "middle_lidar_inner_para.bin";
    std::string json_file_path = configPath + "lidar_sn_inner_para_crc.json";

    if (true == utils::readBinData(bin_file_path)) {
        LogInfo("read inner parameters from file: {} succeeded", bin_file_path);
    } else {
        LogError("read inner parameters from file: {} failed", bin_file_path);
    }

    if (true != crc32::verify_crc32(bin_file_path, json_file_path)) {
        LogError("verify crc32 failure");
        FaultManager64::getInstance().setFault(FaultBits::LidarInternalParamReadFault);
    } else {
        LogInfo("verify crc32 success");
        result = true;
    }
    LogTrace("End of load the inner parameter configuration example ...");

    return result;
}

} // namespace robosense::lidar

/* \}  impl */
