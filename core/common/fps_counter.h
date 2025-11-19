/*******************************************************************************
 * \addtogroup common
 * \{
 * \headerfile fps_counter.h
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
 * | 0.1 | 2025-06-07 | Init version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-08-06 | Add comments;|
 *
 ******************************************************************************/
#ifndef ROBOSENSE_LIDAR_FPS_COUNTER_H
#define ROBOSENSE_LIDAR_FPS_COUNTER_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <string>
#include <deque>
#include <mutex>
#include <cstdint>

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense {
namespace lidar {

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
class FPSCounter {
public:
    /**
     * @brief 构造函数，初始化帧率统计参数
     * @param name 统计类型名称（如"PointCloud"）
     * @param base_fps 基准帧率（用于计算阈值）
     * @param tolerance_percent 允许的波动百分比（如10表示±10%）
     * @param updateInterval 平均帧率更新间隔（单位：秒）
     */
    explicit FPSCounter(const std::string& name, double base_fps = 30.0,
                       double tolerance_percent = 10.0, double updateInterval = 1.0);

    /**
     * @brief 更新一帧数据，传入自定义时间戳（微秒单位）
     * @param timestamp_us 时间戳（单位：微秒）
     */
    void updateFrame(uint64_t timestamp_us);

    /**
     * @brief 获取当前帧率（瞬时值）
     * @return 瞬时帧率（FPS）
     */
    double getCurrentFPS() const;

    /**
     * @brief 获取平均帧率
     * @return 平均帧率（FPS）
     */
    double getAverageFPS() const;

    /**
     * @brief 设置新的基准帧率和波动百分比
     * @param base_fps 新的基准帧率
     * @param tolerance_percent 新的波动百分比
     */
    void setBaseFPS(double base_fps, double tolerance_percent = -1.0);

    /**
     * @brief 判断当前帧率是否超差
     * @param use_average 是否使用平均帧率进行判断（默认false，使用瞬时帧率）
     * @return true: 超差，false: 正常
     */
    bool isFPSAbnormal(bool use_average = false) const;

    /**
     * @brief 获取帧率状态字符串
     * @return 包含当前帧率、平均帧率等信息的字符串
     */
    std::string getStatus() const;

private:
    // 更新阈值（基于基准帧率和波动百分比）
    void updateThresholds();

    std::string name_;                          // 统计类型名称
    double base_fps_;                           // 基准帧率
    double tolerance_percent_;                  // 允许的波动百分比（如10%）
    double update_interval_;                    // 平均帧率更新间隔（秒）
    mutable std::mutex mutex_;                  // 线程安全锁
    std::deque<uint64_t> frame_timestamps_us_;  // 存储时间戳（微秒）
    double current_fps_ = 0.0;                  // 瞬时帧率
    double average_fps_ = 0.0;                  // 平均帧率
    double fps_lower_threshold_ = 0.0;          // 帧率下限
    double fps_upper_threshold_ = 0.0;          // 帧率上限
};

}  // namespace lidar
}  // namespace robosense

#endif  // ROBOSENSE_LIDAR_FPS_COUNTER_H