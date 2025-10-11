/*******************************************************************************
 * \addtogroup decoder
 * \{
 * \headerfile driver_param.h "driver_param.h"
 * \brief Defines some structures and functions of the driver parameter.
 * \version 0.2
 * \date 2025-08-06
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-06 | Init version |
 * | 0.2 | 2025-08-06 | Add comments |
 *
 ******************************************************************************/
#ifndef I_DRIVER_PARAM_H
#define I_DRIVER_PARAM_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cstring>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
/**
 * \brief LiDAR type
 */
enum class LidarType : int32_t {
    RSEMX = 0x71,

    WRONG_TYPE = -1,
};

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
inline std::string lidarTypeToStr(const LidarType& type);
inline LidarType strToLidarType(const std::string& type);

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
/**
 * \brief LiDAR decoder parameter
 */
struct RSDecoderParam {
    LidarType lidar_type = LidarType::RSEMX; ///< Lidar type
    bool wait_for_difop =
        true; ///< true: start sending point cloud until receive difop packet
    float min_distance = 0.0f; ///< min distances of point cloud range.
    float max_distance = 0.0f; ///< max distances of point cloud range.

    /**
     * \brief Convert the struct member variable to string
     * \return string
     */
    std::string toString() const {
        std::stringstream ss;
        ss << "lidar_type: " << lidarTypeToStr(lidar_type) << std::endl;
        ss << "wait_for_difop: " << wait_for_difop << std::endl;
        ss << "min_distance: " << min_distance << std::endl;
        ss << "max_distance: " << max_distance << std::endl;
        return ss.str();
    }
};

/******************************************************************************/
/*                Definition of exported function prototypes                  */
/******************************************************************************/
/**
 * \brief Convert lidar type to string
 * \param[in] type lidar type
 * \return string
 */
inline std::string lidarTypeToStr(const LidarType& type) {
    static const std::unordered_map<LidarType, std::string> lidarTypeMap = {
        { LidarType::RSEMX, "RSEMX" },
    };

    auto it = lidarTypeMap.find(type);
    if (it != lidarTypeMap.end()) {
        return it->second;
    } else {
        static std::string str = "WRONG_TYPE";
        return str;
    }
}

/**
 * \brief Convert string to lidar type
 * \param[in] type lidar type string
 * \return lidar type
 */
inline LidarType strToLidarType(const std::string& type) {
    static const std::unordered_map<std::string, LidarType> strLidarTypeMap = {
        { "RSEMX", LidarType::RSEMX },
    };

    auto it = strLidarTypeMap.find(type);
    if (it != strLidarTypeMap.end()) {
        return it->second;
    } else {
        return LidarType::WRONG_TYPE;
    }
}

} // namespace robosense::lidar

/** \} decoder */
#endif /* I_DRIVER_PARAM_H */