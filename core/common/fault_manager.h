
/*******************************************************************************
 * \addtogroup common
 * \{
 * \headerfile falult_manager.h
 * \brief
 * \version 0.2
 * \date 2025-08-06
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-06-04 | Init version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-08-06 | Add comments |
 *
 ******************************************************************************/
#ifndef ROBOSENSE_FAULT_MANAGER_H
#define ROBOSENSE_FAULT_MANAGER_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <atomic>
#include <bitset>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense {
namespace lidar {
/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/
enum class FaultBits : uint64_t {
    LidarObstructionFault = 0,
    DataAbnormalFault = 1,
    LidarMIPIPacketLossFault = 2,
    LidarPointCloudBufferOverflowFault = 3,
    LidarSDKLogicFault = 4,
    LidarLaserAbnormal = 5,
    LidarInternalParamReadFault = 6,
};

enum class FaultBits8 : uint8_t {
    LidarThreadNameFault = 0,
    LidarThreadCrash = 1,
    LidarThreadNumError = 2,
    LidarThreadSetError = 3,
    LidarPointCloudBufferNull = 4,
};

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
template <typename T = uint64_t, typename FaultEnum = FaultBits>
class FaultManager {
    static_assert(std::is_unsigned<T>::value, "FaultManager only supports unsigned types");
    static_assert(sizeof(T) >= sizeof(FaultEnum), "T must be large enough to hold FaultEnum values");

  private:
    std::atomic<T> fault_status_;
    mutable std::mutex fallback_mutex_;          // 降级时使用的互斥锁
    static constexpr int32_t MAX_ATTEMPTS = 100; // 最大重试次数

    /**
     * \brief  Constructor function.
     */
    FaultManager() : fault_status_(0) {
    }

    FaultManager(const FaultManager&) = delete;
    FaultManager& operator=(const FaultManager&) = delete;

  public:
    uint8_t overflow_position_ {0};
    /**
     * \brief  Get an instance of FaultManager.
     *
     * \return an instance of FaultManager
     */
    static FaultManager& getInstance() {
        static FaultManager instance;
        return instance;
    }

    /**
     * \brief  Set fault bits safely, with retry limits and degradation mechanism.
     *
     * \param[in] fault : fault bits
     */
    void setFault(FaultEnum fault) {
        T mask = static_cast<T>(1) << static_cast<T>(fault);
        setFaultWithMask(mask);
    }

    /**
     * \brief  Clear fault bits safely, with retry limits and degradation mechanism.
     *
     * \param[in] fault : fault bits
     */
    void clearFault(FaultEnum fault) {
        T mask = static_cast<T>(1) << static_cast<T>(fault);
        clearFaultWithMask(mask);
    }

    /**
     * \brief  Check whether given fault bits are set.
     *
     * \param[in] fault : fault bits
     *
     * \return true : given fault bits have been set; false : given fault bits
     *         haven't been set
     */
    bool hasFault(FaultEnum fault) const {
        T mask = static_cast<T>(1) << static_cast<T>(fault);
        return (fault_status_.load(std::memory_order_acquire) & mask) != 0;
    }

    /**
     * \brief  Get the value of current fault bits.
     *
     * \return : the value of current fault bits
     */
    T getFaults() const {
        return fault_status_.load(std::memory_order_acquire);
    }

    /**
     * \brief  Get the total count of the fault bits that are set.
     *
     * \return the total count of the fault bits that are set
     */
    int32_t countFaults() const {
        std::bitset<sizeof(T) * 8> bits(fault_status_.load(std::memory_order_acquire));
        return static_cast<int32_t>(bits.count());
    }

    /**
     * \brief  Set fault bits using given fault mask.
     *
     * \param[in] faultMask : given fault mask
     */
    void setFaults(T faultMask) {
        setFaultWithMask(faultMask);
    }

    /**
     * \brief  Clear fault bits using given fault mask.
     *
     * \param[in] faultMask : given fault mask
     */
    void clearFaults(T faultMask) {
        clearFaultWithMask(faultMask);
    }

    /**
     * \brief  Get the description of the current fault bits.
     *
     * \return the description of the current fault bits
     */
    std::string getFaultDescription() const {
        std::stringstream ss;
        T currentFaults = fault_status_.load(std::memory_order_acquire);

        if (currentFaults == 0) {
            return "No faults";
        }

        ss << "Active faults: ";
        for (size_t i = 0; i < sizeof(T) * 8; ++i) {
            if (currentFaults & (static_cast<T>(1) << i)) {
                ss << "[" << static_cast<int32_t>(i) << "] ";
            }
        }

        return ss.str();
    }

    /**
     * @brief Get the fault Level object
     * @return int32_t fault level
     */
    template <typename U = T, typename E = FaultEnum>
    std::enable_if_t<std::is_same<U, uint64_t>::value && std::is_same<E, FaultBits>::value, int32_t> getFaultLevel() {
        constexpr uint64_t kAllDefinedFaultsMask {0xFF}; // Covers bits 0 to 7
        constexpr uint64_t kPrimaryFaultMask {1ULL << static_cast<uint64_t>(FaultBits::LidarObstructionFault)};
        constexpr uint64_t kSecondaryFaultMask {kAllDefinedFaultsMask & ~kPrimaryFaultMask};

        T current_status = fault_status_.load(std::memory_order_acquire);

        // Check for any secondary faults (including only defined bits)
        if (current_status & kSecondaryFaultMask) {
            return 2;
        }
        // Check for primary fault
        if (current_status & kPrimaryFaultMask) {
            return 1;
        }
        return 0;
    }

  private:
    /**
     * \brief  Set fault bits using given fault mask.
     *
     * \param[in] faultMask : given fault mask
     */
    void setFaultWithMask(T mask) {
        T oldValue, newValue;
        int32_t attempts = 0;

        // 尝试使用原子操作
        while (attempts < MAX_ATTEMPTS) {
            oldValue = fault_status_.load(std::memory_order_relaxed);
            newValue = oldValue | mask;

        if (fault_status_.compare_exchange_weak(oldValue, newValue,
                                                    std::memory_order_release,
                                                    std::memory_order_relaxed)) {
                return; // 操作成功
            }

            attempts++;
            if (attempts % 10 == 0) {
                std::this_thread::yield(); // 让出CPU，减少竞争
            }
        }

        // 原子操作失败，使用互斥锁降级
        std::lock_guard<std::mutex> lock(fallback_mutex_);
        (void)fault_status_.fetch_or(mask, std::memory_order_release);
    }

    /**
     * \brief  Clear fault bits using given fault mask.
     *
     * \param[in] faultMask : given fault mask
     */
    void clearFaultWithMask(T mask) {
        T oldValue, newValue;
        int32_t attempts = 0;

        // 尝试使用原子操作
        while (attempts < MAX_ATTEMPTS) {
            oldValue = fault_status_.load(std::memory_order_relaxed);
            newValue = oldValue & ~mask;

        if (fault_status_.compare_exchange_weak(oldValue, newValue,
                                                std::memory_order_release,
                                                std::memory_order_relaxed)) {
                return; // 操作成功
            }

            attempts++;
            if (attempts % 10 == 0) {
                std::this_thread::yield(); // 让出CPU，减少竞争
            }
        }

        // 原子操作失败，使用互斥锁降级
        std::lock_guard<std::mutex> lock(fallback_mutex_);
        (void)fault_status_.fetch_and(~mask, std::memory_order_release);
    }
};

using FaultManager64 = FaultManager<uint64_t, FaultBits>;
using FaultManager8 = FaultManager<uint8_t, FaultBits8>;

} // namespace lidar
} // namespace robosense
#endif // ROBOSENSE_FAULT_MANAGER_H