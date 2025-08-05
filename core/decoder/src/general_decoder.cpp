#include "general_decoder.h"
#include "rs_new_logger.h"
namespace robosense
{
namespace lidar
{
Decoder::Decoder(const RSDecoderConstParam& const_param, const RSDecoderParam& param)
  : const_param_(const_param)
  , param_(param)
  , distance_section_(const_param.distance_min, const_param.distance_max, param.min_distance, param.max_distance)
  , angles_ready_(false)
  , last_frame_cnt_(0)
  , point_num_(0)
  , theoretical_point_num_(0)
  , theoretical_mipi_data_len_(0)
{
}
void Decoder::regCallback(const std::function<void(uint32_t)>& cb_split_frame)
{
  cb_split_frame_ = cb_split_frame;
}

bool Decoder::processDifopPkt(const uint8_t* pkt, size_t size)
{
  return decodeDifopPkt(pkt, size);
}

bool Decoder::processMsopPkt(const uint8_t* pkt, size_t size)
{
  return decodeMsopPkt(pkt, size);
}

bool Decoder::processDeviceInfoPkt(const uint8_t* pkt, size_t size)
{
  return decodeDeviceInfoPkt(pkt, size);
}
bool Decoder::init()
{
  if (!allocatePointCloud())
  {
    LogError("Failed to allocate point cloud!");
    return false;
  }

  if (!allocateDeviceInfo())
  {
    LogError("Failed to allocate device info!");
    return false;
  }

  initPointCloud();
  initDeviceInfo();
  return true;
}

bool Decoder::allocatePointCloud()
{
  uint32_t pointCount = getTheoreticalPointsPerFrame();
  if (pointCount == 0 || pointCount > MAX_POINTCLOUD_NUM)
  {
    LogError("Invalid point count: {}", std::to_string(pointCount));
    return false;
  }
  size_t requiredSize = sizeof(LidarPointCloud) + (pointCount - 1) * sizeof(LidarPoint);

  LidarPointCloud* rawPtr = static_cast<LidarPointCloud*>(malloc(requiredSize));
  if (!rawPtr)
  {
    LogError("Failed to allocate memory for point cloud!");
    return false;
  }

  point_cloud_.reset(rawPtr);
  return true;
}

bool Decoder::allocateDeviceInfo()
{
  try
  {
    device_info_ = std::make_unique<LidarDeviceInfo>();
    return true;
  }
  catch (const std::bad_alloc&)
  {
    LogError("Failed to allocate memory for device info!");
    return false;
  }
}

void Decoder::initPointCloud()
{
  if (!point_cloud_)
    return;

  auto& cloud = *point_cloud_;
  cloud.protocol_version = 0x0007;  // LidarSDK Struct Version
  cloud.return_mode = 0x1;
  cloud.sync_status = 0x0;
  cloud.frame_sync = 0x0;
  cloud.mirror_id = 0x0;
  uint32_t pointCount = getTheoreticalPointsPerFrame();
  if (pointCount == 0 || pointCount > MAX_POINTCLOUD_NUM)
  {
    LogError("Invalid point count in init: {}", pointCount);
    pointCount = 0;
  }
  cloud.point_num = pointCount;
  cloud.frame_seq = 0;
  cloud.frame_timestamp = 0;
}

void Decoder::initDeviceInfo()
{
  if (!device_info_)
    return;

  device_info_->firware_version = 0;
  device_info_->motor_speed = 0x6F;
  device_info_->return_mode = 0x01;
  device_info_->timestamp = 0;
  device_info_->lidar_operation_state = 0x00;
  device_info_->lidar_fault_state = 0x00;
  device_info_->sdk_total_fault_number = 0;
  device_info_->sdk_fault_code_position = 0;
  device_info_->supplier_internal_fault_id = 0;
  std::fill(std::begin(device_info_->supplier_internal_fault_indicate), std::end(device_info_->supplier_internal_fault_indicate), 0);
  device_info_->supplier_internal_fault_number = 0;
  device_info_->supplier_internal_fault_position = 0;

  device_info_->time_sync_mode = 0x00;
  device_info_->time_sync_status = 0x00;
  device_info_->time_offset = 0;
  std::fill(std::begin(device_info_->lidar_product_sn), std::end(device_info_->lidar_product_sn), 0);
  device_info_->manufacture = 0x03;
  device_info_->model = 0;
  std::fill(std::begin(device_info_->reserved), std::end(device_info_->reserved), 0);
}
void Decoder::resetPointCloud()
{
  if (point_cloud_)
  {
    initPointCloud();
  }
}

void Decoder::resetDeviceInfo()
{
  if (device_info_)
  {
    initDeviceInfo();
  }
}

}  // namespace lidar
}  // namespace robosense
