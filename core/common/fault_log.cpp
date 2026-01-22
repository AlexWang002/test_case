/***********************************************************************************************************************
 * \addtogroup common
 * \{
 * \file fault_log.cpp
 * \brief
 * \version 0.1
 * \date 2026-01-20
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2026-01-20 | Init version |
 *
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/*                 Include dependant library headers                                                                  */
/**********************************************************************************************************************/
#include <algorithm>
#include <iomanip>
#include <regex>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <sys/stat.h>
#include <error.h>
#include <dirent.h>

/**********************************************************************************************************************/
/*                 Include headers of the component                                                                   */
/**********************************************************************************************************************/
#include "fault_log.h"
#include "rs_new_logger.h"
#include "time_utils.h"

/**********************************************************************************************************************/
/*                 Definition of namespace                                                                            */
/**********************************************************************************************************************/
namespace robosense {
namespace lidar {

/**********************************************************************************************************************/
/*                 Definition of local constant data                                                                  */
/**********************************************************************************************************************/
const std::string kFaultLogDir{"/applog/lidar/fault_log"};
const std::string kFileNamePrefix{"rs_lidar_fault_"};
const std::string kFileNameSuffix{".log"};
const std::string kTimestampFormat{"%Y-%m-%d %H:%M:%S"};
const std::string kFileNameTimeFormat{"%Y_%m_%d"};

/**********************************************************************************************************************/
/*                 Definition of public functions of classes or templates                                             */
/**********************************************************************************************************************/
/**
 * @brief Constructor of FaultLog class.
 * @param folder_path Path of the folder to store fault log files.
 * @param max_size Maximum number of fault log files.
 */
FaultLog::FaultLog() : log_folder_path_(kFaultLogDir) {
    folder_exist_ = createDirectoryIfNotExist(log_folder_path_);
    getAllFilesInfo();

    LogInfo("Fault log folder {} already has {} log files, max file counter is {}.",
        log_folder_path_.c_str(), log_files_.size(), max_file_counter_);
    showFileVector();
}

/**
 * @brief Constructor of FaultLog class.
 * @param folder_path Path of the folder to store fault log files.
 * @param max_size Maximum number of fault log files.
 */
FaultLog::FaultLog(const std::string& folder_path) : log_folder_path_(folder_path) {
    folder_exist_ = createDirectoryIfNotExist(log_folder_path_);
    getAllFilesInfo();

    LogInfo("Fault log folder {} already has {} log files, max file counter is {}.",
        log_folder_path_.c_str(), log_files_.size(), max_file_counter_);
    showFileVector();
}

/**
 * @brief Write fault log data to a file.
 * @param data Pointer to the fault log data.
 * @param size Size (in bytes) of the fault log data.
 */
void FaultLog::writeLog(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        LogError("Write fault log error: data is nullptr or size is 0.");
        return;
    }

    if (!folder_exist_) {
        LogError("Fault log folder {} is NOT exist.", kFaultLogDir);
        return;
    }
    ensureSpaceAvailable();

    std::string time_prefix = utils::getCurrentTimeStr(kFileNameTimeFormat);
    std::string timestamp = utils::getCurrentTimeStr(kTimestampFormat);
    std::string file_name = kFileNamePrefix + time_prefix + "_" + getNextSequenceNumber(time_prefix) + kFileNameSuffix;
    std::string file_path = log_folder_path_ + "/" + file_name;
    uint8_t* data_ptr = (uint8_t*)data;
    std::ofstream log_file(file_path, std::ios::trunc);

    if (!log_file.is_open()) {
        LogError("Open fault log file {} error: {}", file_path.c_str(), strerror(errno));
        return;
    }
    log_file.write(timestamp.c_str(), timestamp.size());

    for (size_t i = 0; i < size; i++) {
        if (0 == i % 40) {
            log_file.write("\n", 1);
        }
        log_file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data_ptr[i]);
    }
    log_file.write("\n", 1);
    log_file.close();
    LogInfo("Device info saved in {}, file size {} Bytes.", file_name, getFileSize(file_path));
    updateFileVector(file_name);
    showFileVector();
}

/**********************************************************************************************************************/
/*                 Definition of private functions of classes or templates                                            */
/**********************************************************************************************************************/
/**
 * @brief Create a directory if it does not exist.
 * @param path Path of the directory to create.
 * @return True if the directory is created successfully, false otherwise.
 */
bool FaultLog::createDirectoryIfNotExist(const std::string& path) {
    if (path.empty()) {
        LogError("Create fault log directory error: the log folder path is empty.");
        return false;
    }

    if (0 == access(path.c_str(), F_OK)) {
        LogInfo("Fault log directory {} already exist.", path.c_str());
        return true;
    }
    size_t pos = path.find_last_of("/");

    if (pos != std::string::npos) {
        std::string parent_path = path.substr(0, pos);
        LogInfo("parent_path: {}", parent_path);

        if (!createDirectoryIfNotExist(parent_path)) {
            LogError("Create fault log directory {} error: {}", parent_path.c_str(), strerror(errno));
            return false;
        }
    }

    if (-1 == mkdir(path.c_str(), 0755)) {
        LogError("Create directory {} error: {}", path.c_str(), strerror(errno));
        return false;
    } else {
        LogInfo("Create fault log directory {} success.", path.c_str());
    }

    return true;
}

/**
 * @brief Parse the log file name to extract time information.
 * @param filename Name of the log file.
 * @param info LogFileTimeInfo structure to store parsed time information.
 * @return True if the log file name is parsed successfully, false otherwise.
 */
bool FaultLog::parseLogFileName(const std::string& filename, LogFileTimeInfo& info) {
    std::regex pattern(R"(^rs_lidar_fault_(\d{4})_(\d{2})_(\d{2})_(\d{3})\.log$)");
    std::smatch matches;

    if (std::regex_match(filename, matches, pattern)) {
        info.name = matches[0].str();
        info.year = std::stoi(matches[1].str());
        info.month = std::stoi(matches[2].str());
        info.day = std::stoi(matches[3].str());
        info.sequence = std::stoi(matches[4].str());

        return true;
    } else {
        LogWarn("Found non-standard log file {} in folder {}.", filename, log_folder_path_);
    }

    return false;
}

/**
 * @brief Get the next sequence number for a given time prefix.
 * @param timePrefix Time prefix string in the format "YYYY_MM_DD".
 * @return Next sequence number string with leading zeros (3 digits).
 */
std::string FaultLog::getNextSequenceNumber(const std::string& timePrefix) {
    int max_sequence = 1;

    if (log_files_.empty()) {
        std::ostringstream oss;
        oss << std::setw(3) << std::setfill('0') << max_sequence;

        return oss.str();
    }
    LogFileTimeInfo last_file = log_files_.back();

    if (last_file.year == std::stoi(timePrefix.substr(0, 4)) &&
        last_file.month == std::stoi(timePrefix.substr(5, 2)) &&
        last_file.day == std::stoi(timePrefix.substr(8, 2))) {
        max_sequence = last_file.sequence + 1;
    }
    std::ostringstream oss;
    oss << std::setw(3) << std::setfill('0') << max_sequence;

    return oss.str();
}

/**
 * @brief Get the size of a file in bytes.
 * @param file_path Path of the file to get size.
 * @return Size of the file in bytes, or 0 if an error occurs.
 */
size_t FaultLog::getFileSize(const std::string& file_path) {
    struct stat file_stat;

    if (0 != stat(file_path.c_str(), &file_stat)) {
        LogError("Get file {} size error: {}", file_path.c_str(), strerror(errno));
        return 0;
    }

    return file_stat.st_size;
}

/**
 * @brief Get the size of the log folder in bytes.
 * @return Total size of all files in the log folder in bytes, or 0 if an error occurs.
 */
size_t FaultLog::getFolderSize() {
    size_t total_size = 0; // unit: Byte
    DIR* dir = opendir(log_folder_path_.c_str()); // open the log folder

    if (nullptr == dir) {
        LogError("Open directory {} error: {}", log_folder_path_.c_str(), strerror(errno));
        return total_size;
    }
    struct dirent* entry; // the pointer to the directory entry

    while (nullptr != (entry = readdir(dir))) {    // enumerate all files in the log folder
        if (DT_REG == entry->d_type) {             // only consider regular files
            std::string file_path = log_folder_path_ + "/" + entry->d_name;
            total_size += getFileSize(file_path);
        }
    }
    closedir(dir);

    return total_size;
}

/**
 * @brief Get all fault log files information in the log folder.
 */
void FaultLog::getAllFilesInfo() {
    if (log_folder_path_.empty()) {
        LogError("Get all fault log files info error: the log folder path is empty.");
        return;
    }
    DIR* dir = opendir(log_folder_path_.c_str());   // open the log folder

    if (nullptr == dir) {
        LogError("Open directory {} error: {}", log_folder_path_.c_str(), strerror(errno));
        return;
    }
    struct dirent* entry;   // the pointer to the directory entry

    while (nullptr != (entry = readdir(dir))) {    // enumerate all files in the log folder
        if (DT_REG == entry->d_type) {             // only consider regular files
            std::string file_name = entry->d_name; // get the file name
            LogFileTimeInfo info;

            if (parseLogFileName(file_name, info)) {
                log_files_.emplace_back(info); // only add valid log files
            }
        }
    }
    std::sort(log_files_.begin(), log_files_.end());

    closedir(dir);
}

/**
 * @brief Delete the oldest fault log file in the log folder, ordered by the time in the file name.
 */
void FaultLog::deleteOldestFile() {
    if (log_files_.empty()) {
        return;
    }
    LogFileTimeInfo oldest_file = log_files_.front();
    std::string oldest_file_path = log_folder_path_ + "/" + oldest_file.name;

    if (0 != remove(oldest_file_path.c_str())) {
        LogError("Delete file {} error: {}", oldest_file_path.c_str(), strerror(errno));
        return;
    }

    log_files_.erase(log_files_.begin());
    LogInfo("Delete oldest log file {} success.", oldest_file_path.c_str());
}

/**
 * @brief Ensure that there is enough space available in the log folder to store new log files.
 * @param required_size Required size in bytes to store new log files.
 */
void FaultLog::ensureSpaceAvailable() {

    while (log_files_.size() >= max_file_counter_) {
        LogWarn("Fault log folder has %ld log files, max file counter is %ld, start to delete old files.",
                log_files_.size(), max_file_counter_);
        deleteOldestFile(); // delete the oldest file to free space
    }
}

/**
 * @brief Show all fault log files information in the log folder.
 */
void FaultLog::showFileVector() {
    if (log_files_.empty()) {
        LogInfo("The fault log file vector is empty.");
        return;
    }
    LogInfo("==========Show File List============");

    if (log_files_.size() < 5) {
        for (const auto& info : log_files_) {
            LogInfo("Log file name: %s, year: %d, month: %d, day: %d, sequence: %d",
                    info.name.c_str(), info.year, info.month, info.day, info.sequence);
        }
    } else {
        auto& info = log_files_.front();

        for (int i = 0; i < 3; i++) {
            info = log_files_[i];
            LogInfo("Log file name: %s, year: %d, month: %d, day: %d, sequence: %d",
                    info.name.c_str(), info.year, info.month, info.day, info.sequence);
        }
        LogInfo("...");
        info = log_files_.back();
        LogInfo("Log file name: %s, year: %d, month: %d, day: %d, sequence: %d",
                info.name.c_str(), info.year, info.month, info.day, info.sequence);
    }
    LogInfo("====================================");
}

/**
 * @brief Update the fault log files information vector with a new log file.
 * @param file_name Name of the new log file to be added.
 */
void FaultLog::updateFileVector(const std::string& file_name) {
    if (file_name.empty()) {
        LogError("Update file vector error: the file name is empty.");
        return;
    }
    LogFileTimeInfo info;

    if (parseLogFileName(file_name, info)) {
        log_files_.emplace_back(info);
    } else {
        LogError("Update file vector error: parse log file name {} error.", file_name.c_str());
    }
}

} // namespace lidar
} // namespace robosense

/* \}  common */