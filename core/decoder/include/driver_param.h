#ifndef ROBOSENSE_LIDAR_DRIVER_PARAM_H
#define ROBOSENSE_LIDAR_DRIVER_PARAM_H

#include <string>
#include <map>
#include <cstring>
#include <sstream>
#include <iostream>
#include <unordered_map>
namespace robosense
{
namespace lidar
{
enum LidarType  ///< LiDAR type
{
  RSEMX = 0x71,

  WRONG_TYPE = -1,

};

inline std::string lidarTypeToStr(const LidarType& type)
{
  static const std::unordered_map<LidarType, std::string> lidarTypeMap = {
    { LidarType::RSEMX, "RSEMX" },
  };

  auto it = lidarTypeMap.find(type);
  if (it != lidarTypeMap.end())
  {
    return it->second;
  }
  else
  {
    static std::string str = "WRONG_TYPE";
    return str;
  }
}

inline LidarType strToLidarType(const std::string& type)
{
  static const std::unordered_map<std::string, LidarType> strLidarTypeMap = {
    { "RSEMX", LidarType::RSEMX },
  };

  auto it = strLidarTypeMap.find(type);
  if (it != strLidarTypeMap.end())
  {
    return it->second;
  }
  else
  {
    return WRONG_TYPE;
  }
}

struct RSDecoderParam  ///< LiDAR decoder parameter
{
  LidarType lidar_type = LidarType::RSEMX;  ///< Lidar type
  bool wait_for_difop = true;               ///< true: start sending point cloud until receive difop packet
  float min_distance = 0.0f;                ///< min/max distances of point cloud range. valid if min distance or max distance > 0
  float max_distance = 0.0f;

  std::string toString() const
  {
    std::stringstream ss;
    ss << "lidar_type: " << lidarTypeToStr(lidar_type) << std::endl;
    ss << "wait_for_difop: " << wait_for_difop << std::endl;
    ss << "min_distance: " << min_distance << std::endl;
    ss << "max_distance: " << max_distance << std::endl;
    return ss.str();
  }
};

}  // namespace lidar
}  // namespace robosense
#endif  // ROBOSENSE_LIDAR_DRIVER_PARAM_H