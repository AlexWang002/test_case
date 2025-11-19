#ifndef I_THREAD_CONFIG_H
#define I_THREAD_CONFIG_H

#include <functional>
#include <thread>
#include <vector>

namespace robosense::lidar::thread {
/**
     * \struct ThreadConfig
     * \brief This struct defines the thread config.
     */
struct ThreadConfig {
    uint16_t id {0};
    std::string policy {"SCHED_OTHER"};
    int priority {0};
    std::vector<int> cpu_affinity {1};
};

extern std::vector<ThreadConfig> thread_params;

std::thread createConfiguredStdThread(const ThreadConfig& config, std::function<void()> entry);

} // namespace robosense::lidar::thread

#endif /* I_THREAD_CONFIG_H */