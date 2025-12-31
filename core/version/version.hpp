#ifndef I_SDK_VERSION_H
#define I_SDK_VERSION_H

#include <string>
#include <cstdint>

namespace robosense{
namespace lidar{

const uint32_t kRSLidarSdkVersionMajor{3};
const uint32_t kRSLidarSdkVersionMinor{02};
const uint32_t kRSLidarSdkVersionPatch{00};
const std::string kRSLidarSdkVersionStr{"3.02.00"};
constexpr uint32_t kSdkVersionEncoded{((kRSLidarSdkVersionMajor % 10) * 10000) +
                                      ((kRSLidarSdkVersionMinor % 100) * 100) +
                                        kRSLidarSdkVersionPatch};
} // namespace lidar
} // namespace robosense

#endif // I_SDK_VERSION_H
