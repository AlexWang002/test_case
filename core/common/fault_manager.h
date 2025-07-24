
#ifndef ROBOSENSE_FAULT_MANAGER_H
#define ROBOSENSE_FAULT_MANAGER_H

#include <atomic>
#include <string>
#include <sstream>
#include <cstring>
#include <bitset>
#include <mutex>
#include <thread>

namespace robosense
{
namespace lidar
{
enum class FaultBits : uint64_t
{
  LidarObstructionFault = 0,
  DataAbnormalFault = 1,
  LidarMIPIPacketLengthFault = 2,
  LidarPointCloudBufferOverflowFault = 3,
  LidarSDKLogicFault = 4,
  LidarLaserAbnormal = 5,
  LidarInternalParamReadFault = 6,
};

class FaultManager
{
private:
  std::atomic<uint64_t> fault_status_;
  mutable std::mutex fallback_mutex_;           // 降级时使用的互斥锁
  static constexpr int32_t MAX_ATTEMPTS = 100;  // 最大重试次数

  FaultManager() : fault_status_(0)
  {
  }

  FaultManager(const FaultManager&) = delete;
  FaultManager& operator=(const FaultManager&) = delete;

public:
  static FaultManager& getInstance()
  {
    static FaultManager instance;
    return instance;
  }

  // 安全设置故障位，带重试限制和降级机制
  void setFault(FaultBits fault)
  {
    uint64_t mask = 1ULL << static_cast<uint64_t>(fault);
    setFaultWithMask(mask);
  }

  // 安全清除故障位，带重试限制和降级机制
  void clearFault(FaultBits fault)
  {
    uint64_t mask = 1ULL << static_cast<uint64_t>(fault);
    clearFaultWithMask(mask);
  }

  bool hasFault(FaultBits fault) const
  {
    uint64_t mask = 1ULL << static_cast<uint64_t>(fault);
    return (fault_status_.load(std::memory_order_acquire) & mask) != 0;
  }

  uint64_t getFaults() const
  {
    return fault_status_.load(std::memory_order_acquire);
  }

  int32_t countFaults() const
  {
    std::bitset<64> bits(fault_status_.load(std::memory_order_acquire));
    return static_cast<int32_t>(bits.count());
  }

  void setFaults(uint64_t faultMask)
  {
    setFaultWithMask(faultMask);
  }

  void clearFaults(uint64_t faultMask)
  {
    clearFaultWithMask(faultMask);
  }

  std::string getFaultDescription() const
  {
    std::stringstream ss;
    uint64_t currentFaults = fault_status_.load(std::memory_order_acquire);

    if (currentFaults == 0)
    {
      return "No faults";
    }

    ss << "Active faults: ";
    for (size_t i = 0; i < 64; ++i)
    {
      if (currentFaults & (uint64_t(1) << i))
      {
        ss << "[" << static_cast<int32_t>(i) << "] ";
      }
    }

    return ss.str();
  }

private:
  // 设置故障位的通用实现，带重试和降级
  void setFaultWithMask(uint64_t mask)
  {
    uint64_t oldValue, newValue;
    int32_t attempts = 0;

    // 尝试使用原子操作
    while (attempts < MAX_ATTEMPTS)
    {
      oldValue = fault_status_.load(std::memory_order_relaxed);
      newValue = oldValue | mask;

      if (fault_status_.compare_exchange_weak(oldValue, newValue, std::memory_order_release, std::memory_order_relaxed))
      {
        return;  // 操作成功
      }

      attempts++;
      if (attempts % 10 == 0)
      {
        std::this_thread::yield();  // 让出CPU，减少竞争
      }
    }

    // 原子操作失败，使用互斥锁降级
    std::lock_guard<std::mutex> lock(fallback_mutex_);
    (void)fault_status_.fetch_or(mask, std::memory_order_release);
  }

  // 清除故障位的通用实现，带重试和降级
  void clearFaultWithMask(uint64_t mask)
  {
    uint64_t oldValue, newValue;
    int32_t attempts = 0;

    // 尝试使用原子操作
    while (attempts < MAX_ATTEMPTS)
    {
      oldValue = fault_status_.load(std::memory_order_relaxed);
      newValue = oldValue & ~mask;

      if (fault_status_.compare_exchange_weak(oldValue, newValue, std::memory_order_release, std::memory_order_relaxed))
      {
        return;  // 操作成功
      }

      attempts++;
      if (attempts % 10 == 0)
      {
        std::this_thread::yield();  // 让出CPU，减少竞争
      }
    }

    // 原子操作失败，使用互斥锁降级
    std::lock_guard<std::mutex> lock(fallback_mutex_);
    (void)fault_status_.fetch_and(~mask, std::memory_order_release);
  }
};

}  // namespace lidar
}  // namespace robosense
#endif  // ROBOSENSE_FAULT_MANAGER_H