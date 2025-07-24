/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#ifndef ROBOSENSE_GENERAL_DECODER_H_
#define ROBOSENSE_GENERAL_DECODER_H_

#include "lidar_sdk_api.h"
#include <driver_param.h>
#include <trigon.h>
#include <section.h>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES  // for VC++, required to use const M_IP in <math.h>
#endif

#include <cmath>
#include <functional>
#include <memory>
#include <iomanip>
#include <vector>

namespace robosense
{
namespace lidar
{
// decoder const param
struct RSDecoderConstParam
{
  // packet len
  uint16_t msop_len;
  uint16_t difop_len;
  uint16_t device_info_len;

  // packet identity
  uint8_t msop_id_len;
  uint8_t difop_id_len;
  uint8_t device_info_id_len;
  uint8_t msop_id[8];
  uint8_t difop_id[8];
  uint8_t device_info_id[8];

  uint16_t pixel_per_pkt;

  // distance
  float distance_min;
  float distance_max;
  float distance_res;
};

struct PacketRangePerMipiFrame
{
  int32_t start;
  int32_t end;
  int32_t expectedCount;
};
constexpr uint32_t MAX_POINTCLOUD_NUM = 5000000;

class Decoder
{
public:
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual bool decodeDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual bool decodeDeviceInfoPkt(const uint8_t* pkt, size_t size) = 0;
  virtual ~Decoder() = default;

  bool init();
  bool processMsopPkt(const uint8_t* pkt, size_t size);
  bool processDifopPkt(const uint8_t* pkt, size_t size);
  bool processDeviceInfoPkt(const uint8_t* pkt, size_t size);

  explicit Decoder(const RSDecoderConstParam& const_param, const RSDecoderParam& param);
  //

  void regCallback(const std::function<void(uint32_t)>& cb_split_frame);

  void resetPointCloud();
  void resetDeviceInfo();

  std::unique_ptr<LidarPointCloud, void (*)(LidarPointCloud*)> point_cloud_{ nullptr, [](LidarPointCloud* p) { free(p); } };
  std::unique_ptr<LidarDeviceInfo> device_info_;
  bool is_test_time_delay_ = false;
  uint32_t getTheoreticalPointsPerFrame()
  {
    return theoretical_point_num_;
  }

  uint32_t getTheoreticalMipiDataLength()
  {
    return theoretical_mipi_data_len_;
  }

  uint16_t getMsopLength()
  {
    return const_param_.msop_len;
  }
  uint16_t getDifopLength()
  {
    return const_param_.difop_len;
  }
  uint16_t getDeviceInfoLength()
  {
    return const_param_.device_info_len;
  }
  std::vector<PacketRangePerMipiFrame> getPacketRanges()
  {
    return packet_ranges_;
  }

protected:
  // Further corrections are needed
  //
  RSDecoderConstParam const_param_;  // const param
  RSDecoderParam param_;             // user param
  std::function<void(uint32_t)> cb_split_frame_;
  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  DistanceSection distance_section_;  // invalid section of distance

  bool angles_ready_;  // is vert_angles/horiz_angles ready from csv file/difop packet?
  uint32_t last_frame_cnt_;
  uint32_t point_num_;
  uint32_t theoretical_point_num_;
  uint32_t theoretical_mipi_data_len_;

  std::vector<PacketRangePerMipiFrame> packet_ranges_;

  bool allocatePointCloud();
  bool allocateDeviceInfo();
  void initPointCloud();
  void initDeviceInfo();
};

}  // namespace lidar
}  // namespace robosense

#endif  // ROBOSENSE_GENERAL_DECODER_H_