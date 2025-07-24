#include <cstdint>
#include "decoder_emx.h"
#include "common/fault_manager.h"
#include "rs_new_logger.h"
namespace robosense
{
namespace lidar
{
DecoderRSEMX::DecoderRSEMX(const RSDecoderParam& param) : Decoder(getConstParam(), param)
{
  this->packet_ranges_ = { { 1, 281, 281 }, { 282, 562, 281 }, { 563, 843, 281 }, { 844, 1124, 281 }, { 1125, 1405, 281 }, { 1406, 1520, 115 } };
  this->yaw_offset_.fill(0);

  std::array<int32_t, EMX_PIXELS_PER_COLUMN> defaultAngle;
  constexpr int32_t START_ANGLE = -2416;
  constexpr int32_t ANGLE_STEP = 21;
  for (uint16_t i = 0; i < EMX_PIXELS_PER_COLUMN; ++i)
  {
    defaultAngle[i] = START_ANGLE + i * ANGLE_STEP;
  }
  this->pitch_angle_ = defaultAngle;

  this->surface_pitch_offset_.fill(0);
  this->theoretical_point_num_ = EMX_PIXELS_PER_COLUMN * EMX_COLUMNS_PER_FRAME;
  this->theoretical_mipi_data_len_ = EMX_MIPI_DATA_LEN;
  cached_pkt_timestamps_.resize(EMX_COLUMNS_PER_FRAME, 0);
}

bool DecoderRSEMX::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  if(packet == nullptr)
  {
    LogError("[DecoderRSEMX::decodeDifopPkt] packet is nullptr");
    return false;
  }
  const size_t difop2_len = sizeof(RSEMXDifop2Pkt);

  if (size == difop2_len && memcmp(packet, this->const_param_.difop_id, const_param_.difop_id_len) == 0)
  {
    const RSEMXDifop2Pkt& pkt = *(const RSEMXDifop2Pkt*)(packet);
    for (uint16_t i = 0; i < EMX_VECSELS_PER_COLUMN; i++)
    {
      this->yaw_offset_[i] = static_cast<int32_t>(pkt.yaw_offset[i]);
    }
    for (uint16_t i = 0; i < EMX_PIXELS_PER_COLUMN; i++)
    {
      this->pitch_angle_[i] = static_cast<int32_t>(RS_SWAP_INT16(pkt.pitch_angle[i]) / 2);
    }

    for (uint16_t i = 0; i < EMX_SURFACE_NUM; i++)
    {
      this->surface_pitch_offset_[i] = static_cast<int32_t>(RS_SWAP_INT16(pkt.surface_pitch_offset[i]) / 2);
    }
    this->angles_ready_ = true;
    return true;
  }
  if (size != difop2_len)
  {
    LogError("wrong difop2 packet size: {}", size);
  }
  else if (memcmp(packet, this->const_param_.difop_id, const_param_.difop_id_len) != 0)
  {
    LogError("wrong difop2 packet id");
  }
  return false;
}
bool DecoderRSEMX::decodeDeviceInfoPkt(const uint8_t* packet, size_t size)
{
  if(packet == nullptr)
  {
    LogError("[DecoderRSEMX::decodeDeviceInfoPkt] packet is nullptr");
    return false;
  }
  const size_t device_info_len = sizeof(RSEMXDeviceInfoPkt);

  if (size == device_info_len && (memcmp(packet, this->const_param_.device_info_id, const_param_.device_info_id_len) == 0) && this->device_info_)
  {
    const RSEMXDeviceInfoPkt& pkt = *(const RSEMXDeviceInfoPkt*)(packet);

    this->device_info_->firware_version = ntohs(pkt.device_version.sw_version);
    if (pkt.work_info.win_block_status != 0)
    {
      FaultManager::getInstance().setFault(FaultBits::LidarObstructionFault);
    }

    this->device_info_->lidar_operation_state = (pkt.fault_level != 0);
    this->device_info_->lidar_fault_state = pkt.fault_level > 1 ? 2 : pkt.fault_level;
    this->device_info_->supplier_internal_fault_id = ntohs(pkt.fault_id);
    (void)std::reverse_copy(std::begin(pkt.fault_value), std::end(pkt.fault_value), this->device_info_->supplier_internal_fault_indicate);
    this->device_info_->time_sync_mode = pkt.time_info.time_sync_mode;
    this->device_info_->time_sync_status = pkt.time_info.time_sync_status;

    this->device_info_->timestamp = parseTimeUTCWithUs(pkt.time_info.timestamp);
    this->device_info_->time_offset = ntohll(pkt.time_info.time_offset);

    (void)std::reverse_copy(std::begin(pkt.customer_sn), std::end(pkt.customer_sn), this->device_info_->lidar_product_sn);
    this->device_info_->model = 0x01;
    return true;
  }
  if (size != device_info_len)
  {
    LogError("wrong difop1 packet size: {}", size);
  }
  else if (memcmp(packet, this->const_param_.device_info_id, const_param_.device_info_id_len) != 0)
  {
    LogError("wrong device info id");
  }else if(!this->device_info_)
  {
    LogError("device info is nullptr");
  }
  return false;
}

bool DecoderRSEMX::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  if(packet == nullptr)
  {
    LogError("[DecoderRSEMX::decodeMsopPkt] packet is nullptr");
    return false;
  }
  const size_t msop_pkt_size = sizeof(RSEMXMsopPkt);
  const uint16_t pixel_per_pkt = this->const_param_.pixel_per_pkt;
  const float distance_res = this->const_param_.distance_res;
  const float yaw_factor = 1.0f / 5.12f;
  if (size == msop_pkt_size && this->angles_ready_ && memcmp(packet, this->const_param_.msop_id, this->const_param_.msop_id_len) == 0 && this->point_cloud_)
  {
    const RSEMXMsopPkt& pkt = *reinterpret_cast<const RSEMXMsopPkt*>(packet);

    uint64_t pkt_ts = parseTimeUTCWithUs(pkt.header.timestamp);

    uint16_t raw_pkt_seq = ntohs(pkt.header.pkt_seq);

    if (raw_pkt_seq >= EMX_PKT_SEQ_MIN && raw_pkt_seq <= EMX_PKT_SEQ_MAX)
    {
      static uint16_t last_pkt_seq = 0;
      uint32_t frame_cnt = ntohl(pkt.header.frame_cnt);
      if (last_pkt_seq != 0 && raw_pkt_seq != (last_pkt_seq + 1) && (raw_pkt_seq + 1519) != last_pkt_seq)
      {
        LogError("lost pkt, last seq is: {}, current seq is: {} ,frame_cnt:{}", last_pkt_seq, raw_pkt_seq, frame_cnt);
      }
      last_pkt_seq = raw_pkt_seq;

      if (MIDDLE_PACKET_SEQ == raw_pkt_seq)
      {
        this->point_cloud_->frame_timestamp = pkt_ts;
      }
      uint16_t pkt_seq = raw_pkt_seq - 1;

      if (pkt_seq < this->cached_pkt_timestamps_.size())
      {
        this->cached_pkt_timestamps_[pkt_seq] = pkt_ts;
      }

      if (frame_cnt != this->last_frame_cnt_)
      {
        this->last_frame_cnt_ = frame_cnt;
      }
      uint8_t surface_index = pkt.header.surface_id;
      if (surface_index < EMX_SURFACE_NUM)
      {
        const int32_t yaw_base = RS_SWAP_INT16(pkt.header.yaw);
        const int32_t surface_pitch_offset_val = this->surface_pitch_offset_[surface_index];
        const uint32_t base_point_cnt = pkt_seq * pixel_per_pkt;
        for (uint16_t pixel_id = 0; pixel_id < pixel_per_pkt; ++pixel_id)
        {
          const RSEMXMsopPixel& pixle = pkt.pixels[pixel_id];
          int32_t yaw = yaw_base + this->yaw_offset_[pixel_id / EMX_PIXELS_PER_VCSEL];
          yaw = static_cast<int32_t>(yaw * yaw_factor);
          const int32_t pitch = this->pitch_angle_[pixel_id] + surface_pitch_offset_val;
          const float cos_pitch = COS(pitch);
          const float sin_pitch = SIN(pitch);
          const float cos_yaw = COS(yaw);
          const float sin_yaw = SIN(yaw);
          uint32_t point_count = base_point_cnt + pixel_id;
          if (point_count < this->theoretical_point_num_)
          {
            const RSEMXMsopWave& wave = pixle.waves[0];
            const float distance = ntohs(wave.radius) * distance_res;
            auto& current_point = this->point_cloud_->point[point_count];
            if (this->distance_section_.in(distance))
            {
              float x = distance * cos_pitch * cos_yaw;
              float y = distance * cos_pitch * sin_yaw;
              float z = distance * sin_pitch;

              current_point.x = x;
              current_point.y = y;
              current_point.z = z;
              current_point.channel_number = pixel_id;
              current_point.intensity = wave.intensity;
              current_point.confidence[0] = wave.attribute;
            }
            else
            {
              current_point.x = NAN;
              current_point.y = NAN;
              current_point.z = NAN;
              current_point.channel_number = pixel_id;
              current_point.intensity = wave.intensity;
              current_point.confidence[0] = wave.attribute;
            }
          }
          else
          {
            LogError("point_count :{} >= this->theoretical_point_num_ : {}; pkt_seq:{}", point_count, this->theoretical_point_num_, pkt_seq);
            return false;
          }
        }

        this->point_num_ += pixel_per_pkt;

        if (EMX_COLUMNS_PER_FRAME == raw_pkt_seq)
        {
          setPointCloudInfo(pkt);
        }
        return true;
      }
      else
      {
        LogError("surface_id out of range, surface_id is: {}, max is {}", surface_index, EMX_SURFACE_NUM);
        return false;
      }
    }
    else
    {
      LogError("msop pkt seq: {}, out of range {} - {}", raw_pkt_seq, EMX_PKT_SEQ_MIN, EMX_COLUMNS_PER_FRAME);
      return false;
    }
  }
  if (!this->angles_ready_)
  {
    LogError("calibration angles not ready");
  }
  else if (size != msop_pkt_size)
  {
    LogError("wrong msop packet size: {}", size);
  }
  else if (memcmp(packet, this->const_param_.msop_id, this->const_param_.msop_id_len) != 0)
  {
    LogError("wrong msop packet id");
  }else if(this->point_cloud_ == nullptr)
  {
    LogError("point_cloud_ is nullptr");
  }
  return false;
}

void DecoderRSEMX::setPointCloudInfo(const RSEMXMsopPkt& pkt)
{
  if(this->point_cloud_ == nullptr)
  {
    LogError("point_cloud_ is nullptr");
    return;
  }
  const float timestamp_factor = 0.1f;

  const uint16_t pixel_per_pkt = this->const_param_.pixel_per_pkt;
  size_t cached_pkt_num = this->cached_pkt_timestamps_.size();
  if (cached_pkt_num * pixel_per_pkt <= this->theoretical_point_num_)
  {
    uint64_t frame_timestamp = this->point_cloud_->frame_timestamp;
    std::vector<int16_t> offsets;
    offsets.reserve(cached_pkt_num);
    for (size_t i = 0; i < cached_pkt_num; i++)
    {
      uint64_t cached_pkt_ts = this->cached_pkt_timestamps_[i];
      int64_t offset = (cached_pkt_ts >= frame_timestamp) ? (cached_pkt_ts - frame_timestamp) : -(frame_timestamp - cached_pkt_ts);
      offset = static_cast<int64_t>(offset * timestamp_factor);

      int16_t clamped_offset = (offset < INT16_MIN) ? INT16_MIN : (offset > INT16_MAX) ? INT16_MAX : static_cast<int16_t>(offset);

      offsets.push_back(clamped_offset);
    }

    for (size_t i = 0; i < offsets.size(); i++)
    {
      int16_t offset = offsets[i];
      for (int32_t j = 0; j < pixel_per_pkt; j++)
      {
        this->point_cloud_->point[i * pixel_per_pkt + j].timestamp = offset;
      }
    }
  }
  else
  {
    LogError("point cloud size is not equal to theoretical point num, pixel_per_pkt:{}, point_cloud size:{}, theoretical point num:{}", pixel_per_pkt,
             cached_pkt_num, this->theoretical_point_num_);
  }

  this->point_cloud_->frame_seq = ntohl(pkt.header.frame_cnt);
  this->point_cloud_->sync_status = pkt.header.time_mode > 0 ? 1 : 0;
  this->point_cloud_->frame_sync = pkt.header.synchronized;
  this->point_cloud_->mirror_id = pkt.header.surface_id;

  this->cb_split_frame_(this->point_num_);
  this->point_num_ = 0;
}
}  // namespace lidar
}  // namespace robosense
