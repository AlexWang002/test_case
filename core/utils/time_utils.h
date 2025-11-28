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
template <typename Period = std::chrono::milliseconds>
inline long timeInterval(const TimePoint& start_time,
                         const TimePoint& end_time = std::chrono::steady_clock::now()) {
    return std::chrono::duration_cast<Period>(end_time - start_time).count();
}

template <typename Period = std::chrono::milliseconds>
inline long unixTimestamp(const TimePoint& time = std::chrono::steady_clock::now()) {
    return std::chrono::duration_cast<Period>(time.time_since_epoch()).count();
}

} // namespace utils
} // namespace lidar
} // namespace robosense

/** \} utils */
#endif /* I_TIME_UTILS_H */
