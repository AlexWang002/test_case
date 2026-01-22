/***********************************************************************************************************************
 * \addtogroup common
 * \{
 * \headerfile fault_log "fault_log"
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
#ifndef I_FAULT_LOG_H
#define I_FAULT_LOG_H

/**********************************************************************************************************************/
/*                 Include dependant library headers                                                                  */
/**********************************************************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <cstdint>
#include <deque>

/**********************************************************************************************************************/
/*                 Definition of namespace                                                                            */
/**********************************************************************************************************************/
namespace robosense {
namespace lidar {

/**********************************************************************************************************************/
/*                 Definition of classes or templates                                                                 */
/**********************************************************************************************************************/
class FaultLog {
  public:
    FaultLog();
    FaultLog(const std::string& folder_path, size_t max_size);
    ~FaultLog() = default;

    void writeLog(const void* data, size_t size);
    std::string getFileTotalNumber();

  private:
    struct LogFileTimeInfo {
        int year;
        int month;
        int day;
        int sequence;
        std::string name;

        bool operator<(const LogFileTimeInfo& other) const {
            if (year != other.year) return year < other.year;
            if (month != other.month) return month < other.month;
            if (day != other.day) return day < other.day;
            return sequence < other.sequence;
        }
    };

    std::string log_folder_path_{""};
    size_t folder_max_size_{1024};
    bool folder_exist_{false};

    std::vector<LogFileTimeInfo> log_files_;

    std::string getNextSequenceNumber(const std::string& timePrefix);
    size_t getFileSize(const std::string& filePath);
    size_t getFolderSize();
    bool parseLogFileName(const std::string& filename, LogFileTimeInfo& info);
    bool createDirectoryIfNotExist(const std::string& path);
    void showFileVector();
    void getAllFilesInfo();
    void deleteOldestFile();
    void updateFileVector(const std::string& file_name);
    void ensureSpaceAvailable(size_t required_size);
};

} // namespace lidar
} // namespace robosense


/** \} common */
#endif /* I_FAULT_LOG_H */
