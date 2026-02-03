/***********************************************************************************************************************
 * \addtogroup utils
 * \{
 * \headerfile time_utils "time_utils"
 * \brief
 * \version 0.1
 * \date 2025-11-25
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-11-25 | Init version |
 *
 **********************************************************************************************************************/
#ifndef I_TIME_UTILS_H
#define I_TIME_UTILS_H

/**********************************************************************************************************************/
/*                 Include dependant library headers                                                                  */
/**********************************************************************************************************************/
#include <chrono>

/**********************************************************************************************************************/
/*                 Definition of namespace                                                                            */
/**********************************************************************************************************************/
namespace robosense {
namespace lidar {
namespace utils {

/**********************************************************************************************************************/
/*                 Using namespace, type or template alias                                                            */
/**********************************************************************************************************************/
using TimePoint = std::chrono::steady_clock::time_point;
using TimePointSystem = std::chrono::system_clock::time_point;

/**********************************************************************************************************************/
/*                 Definition of classes or templates                                                                 */
/**********************************************************************************************************************/
/**
 * @brief Calculate the time interval between two time points.
 *
 * @tparam Period The time period type, default is std::chrono::milliseconds.
 * @param start_time The start time point.
 * @param end_time The end time point, default is the current time point.
 * @return long The time interval in the specified period.
 */
template <typename Period = std::chrono::milliseconds>
inline long timeInterval(const TimePoint& start_time,
                         const TimePoint& end_time = std::chrono::steady_clock::now()) {
    return std::chrono::duration_cast<Period>(end_time - start_time).count();
}

/**
 * @brief Get the Unix timestamp of a time point.
 *
 * @tparam Period The time period type, default is std::chrono::milliseconds.
 * @param time The time point, default is the current time point.
 * @return long The Unix timestamp in the specified period.
 */
template <typename Period = std::chrono::milliseconds>
inline long unixTimestamp(const TimePoint& time = std::chrono::steady_clock::now()) {
    return std::chrono::duration_cast<Period>(time.time_since_epoch()).count();
}

/**
 * @brief Get the current time string in the specified format.
 *
 * @param format The time format string, default is "%Y_%m_%d".
 * @return std::string The current time string in the specified format.
 */
inline std::string getCurrentTimeStr(const std::string& format = "%Y_%m_%d") {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time_t);
    std::ostringstream oss;
    // format time string
    oss << std::put_time(&now_tm, format.c_str());

    return oss.str();
}

} // namespace utils
} // namespace lidar
} // namespace robosense

/** \} utils */
#endif /* I_TIME_UTILS_H */
