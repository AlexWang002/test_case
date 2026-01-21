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
const size_t kFolderMaxSize{1024 * 1024}; // 1MB
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
 * @param max_size Maximum size (in bytes) of the fault log folder.
 */
FaultLog::FaultLog() :
        log_folder_path_(kFaultLogDir), folder_max_size_(kFolderMaxSize) {
    createDirectoryIfNotExist(log_folder_path_);
    getAllFilesInfo();

    LogInfo("Fault log folder {} already has {} log files, max size is {} current used size is {}.",
        log_folder_path_.c_str(), log_files_.size(), folder_max_size_, getFolderSize());
}

/**
 * @brief Constructor of FaultLog class.
 * @param folder_path Path of the folder to store fault log files.
 * @param max_size Maximum size (in bytes) of the fault log folder.
 */
FaultLog::FaultLog(const std::string& folder_path, size_t max_size) :
        log_folder_path_(folder_path), folder_max_size_(max_size) {
    createDirectoryIfNotExist(log_folder_path_);
    getAllFilesInfo();

    LogInfo("Fault log folder {} already has {} log files, max size is {} current used size is {}.",
        log_folder_path_.c_str(), log_files_.size(), folder_max_size_, getFolderSize());
}

/**
 * @brief Write fault log data to a file.
 * @param data Pointer to the fault log data.
 * @param size Size (in bytes) of the fault log data.
 * @return True if the fault log is written successfully, false otherwise.
 */
bool FaultLog::writeLog(const void* data, size_t size) {
    if (data == nullptr || size == 0) {
        LogError("Write fault log error: data is nullptr or size is 0.");
        return false;
    }
    showFileVector();
    ensureSpaceAvailable(size);

    std::string time_prefix = utils::getCurrentTimeStr(kFileNameTimeFormat);
    std::string timestamp = utils::getCurrentTimeStr(kTimestampFormat);
    // std::string sequence_prefix = getNextSequenceNumber(time_prefix);
    std::string file_name = kFileNamePrefix + time_prefix + "_" + getNextSequenceNumber(time_prefix) + kFileNameSuffix;
    std::string file_path = log_folder_path_ + "/" + file_name;
    uint8_t* data_ptr = (uint8_t*)data;
    std::ofstream log_file(file_path, std::ios::trunc);

    if (!log_file.is_open()) {
        LogError("Open fault log file %s error: %s", file_path.c_str(), strerror(errno));
        return false;
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
    updateFileVector(file_name);
    showFileVector();

    return true;
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
        LogInfo("Fault log directory %s already exist.", path.c_str());
        return true;
    }
    size_t pos = path.find_last_of("/");

    if (pos != std::string::npos) {
        std::string parent_path = path.substr(0, pos);

        return createDirectoryIfNotExist(parent_path);
    }

    if (-1 == mkdir(path.c_str(), 0755)) {
        LogError("Create directory %s error: %s", path.c_str(), strerror(errno));
        return false;
    } else {
        LogInfo("Create fault log directory %s success.", path.c_str());
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
    // std::regex pattern(R"(^file_(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{3})\.log$)");
    std::regex pattern(R"(^rs_lidar_fault_(\d{4})_(\d{2})_(\d{2})_(\d{3})\.log$)");
    std::smatch matches;

    if (std::regex_match(filename, matches, pattern)) {
        info.name = matches[0].str();
        info.year = std::stoi(matches[1].str());
        info.month = std::stoi(matches[2].str());
        info.day = std::stoi(matches[3].str());
        // info.hour = std::stoi(matches[4].str());
        // info.sequence = std::stoi(matches[5].str());
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
        LogError("Get file %s size error: %s", file_path.c_str(), strerror(errno));
        return 0;
    }

    return file_stat.st_size;
}

/**
 * @brief Get the size of the log folder in bytes.
 * @return Total size of all files in the log folder in bytes, or 0 if an error occurs.
 */
size_t FaultLog::getFolderSize() {
    size_t total_size = 0; // unit: byte
    DIR* dir = opendir(log_folder_path_.c_str()); // open the log folder

    if (nullptr == dir) {
        LogError("Open directory %s error: %s", log_folder_path_.c_str(), strerror(errno));
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
        LogError("Open directory %s error: %s", log_folder_path_.c_str(), strerror(errno));
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
        LogError("Delete file %s error: %s", oldest_file_path.c_str(), strerror(errno));
        return;
    }

    log_files_.erase(log_files_.begin());
    LogInfo("Delete oldest log file %s success.", oldest_file_path.c_str());
}

/**
 * @brief Ensure that there is enough space available in the log folder to store new log files.
 * @param required_size Required size in bytes to store new log files.
 */
void FaultLog::ensureSpaceAvailable(size_t required_size) {
    size_t folder_size = getFolderSize();

    while (folder_size + required_size >= folder_max_size_) {
        LogWarn("Fault log folder size %ld exceeds max size %ld, start to delete old files.",
                folder_size, folder_max_size_);
        deleteOldestFile(); // delete the oldest file to free space
        folder_size = getFolderSize();
    }
}

/**
 * @brief Show all fault log files information in the log folder.
 */
void FaultLog::showFileVector() {
    if (log_files_.empty()) {
        std::cout << "Log file vector is empty." << std::endl;
        return;
    }
    std::cout << "==========Show File List============" << std::endl;

    for (const auto& info : log_files_) {
        std::cout << "Log file name: " << info.name << ", year: " << info.year
                << ", month: " << info.month << ", day: " << info.day
                << ", sequence: " << info.sequence << std::endl;
    }
    std::cout << "====================================" << std::endl;
}

/**
 * @brief Update the fault log files information vector with a new log file.
 * @param file_name Name of the new log file to be added.
 */
void FaultLog::updateFileVector(const std::string& file_name) {
    if (file_name.empty()) {
        LogError("Update file vector error: the file name is empty.");
        // std::cerr << "Update file vector error: the file name is empty." << std::endl;
        return;
    }
    LogFileTimeInfo info;

    if (parseLogFileName(file_name, info)) {
        log_files_.emplace_back(info);
    } else {
        LogError("Update file vector error: parse log file name %s error.", file_name.c_str());
        // std::cerr << "Update file vector error: parse log file name " << file_name << " error." << std::endl;
    }
}

} // namespace lidar
} // namespace robosense

/* \}  common */