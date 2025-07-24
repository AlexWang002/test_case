#include "fps_counter.h"
#include <string> // 用于std::to_string

namespace robosense {
namespace lidar {

// 构造函数实现
FPSCounter::FPSCounter(const std::string& name,
                       double base_fps,
                       double tolerance_percent,
                       double updateInterval) :
  name_(name),
  base_fps_(base_fps),
  tolerance_percent_(tolerance_percent),
  update_interval_(updateInterval) {
    // 初始化阈值
    updateThresholds();
}

// 更新一帧数据
void FPSCounter::updateFrame(uint64_t timestamp_us) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 添加当前时间戳到队列
    frame_timestamps_us_.push_back(timestamp_us);

    // 移除超出时间窗口的旧时间戳
    uint64_t earliest_time_us =
        timestamp_us - static_cast<uint64_t>(update_interval_ * 1e6);
    while (!frame_timestamps_us_.empty() &&
           frame_timestamps_us_.front() < earliest_time_us) {
        frame_timestamps_us_.pop_front();
    }

    // 计算瞬时帧率（基于最后两帧的时间差）
    if (frame_timestamps_us_.size() >= 2) {
        uint64_t delta_us =
            frame_timestamps_us_.back() -
            frame_timestamps_us_[frame_timestamps_us_.size() - 2];
        current_fps_ = (delta_us > 0) ? 1e6 / delta_us : 0.0;
    } else {
        current_fps_ = 0.0;
    }

    // 计算平均帧率（时间窗口内的帧数 / 总时长）
    if (!frame_timestamps_us_.empty()) {
        uint64_t total_duration_us =
            frame_timestamps_us_.back() - frame_timestamps_us_.front();
        double total_duration_sec = total_duration_us / 1e6;
        average_fps_ = (total_duration_sec > 0)
                           ? static_cast<double>(frame_timestamps_us_.size()) /
                                 total_duration_sec
                           : 0.0;
    } else {
        average_fps_ = 0.0;
    }
}

// 获取当前帧率
double FPSCounter::getCurrentFPS() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_fps_;
}

// 获取平均帧率
double FPSCounter::getAverageFPS() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return average_fps_;
}

// 设置新的基准帧率和波动百分比
void FPSCounter::setBaseFPS(double base_fps, double tolerance_percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    base_fps_ = base_fps;

    // 更新波动百分比（如果提供了新值）
    if (tolerance_percent > 0) {
        tolerance_percent_ = tolerance_percent;
    }

    // 重新计算阈值
    updateThresholds();
}

// 判断帧率是否超差
bool FPSCounter::isFPSAbnormal(bool use_average) const {
    std::lock_guard<std::mutex> lock(mutex_);

    // 基准帧率为0，视为未设置阈值
    if (base_fps_ <= 0) {
        return false;
    }

    double fps = use_average ? average_fps_ : current_fps_;

    // 判断超差条件（低于下限或高于上限）
    return (fps < fps_lower_threshold_ || fps > fps_upper_threshold_);
}

// 获取状态字符串
std::string FPSCounter::getStatus() const {
    // 注意：调用getCurrentFPS()和getAverageFPS()已加锁，无需重复加锁
    return "[" + name_ + "] FPS: Current=" + std::to_string(getCurrentFPS()) +
           ", Average=" + std::to_string(getAverageFPS()) +
           ", Frames=" + std::to_string(frame_timestamps_us_.size());
}

// 私有函数：更新阈值
void FPSCounter::updateThresholds() {
    double tolerance = base_fps_ * tolerance_percent_ / 100.0;
    fps_lower_threshold_ = std::max(0.0, base_fps_ - tolerance); // 下限不低于0
    fps_upper_threshold_ = base_fps_ + tolerance;
}

} // namespace lidar
} // namespace robosense