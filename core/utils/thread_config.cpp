
#include <sched.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <errno.h>

#include "rs_new_logger.h"
#include "thread_config.h"

namespace robosense
{
namespace lidar
{
    std::thread createConfiguredStdThread(const yaml::ThreadConfig& config, std::function<void()> entry) {
    return std::thread([config, entry]() {
        LogWarn("Thread id: {} policy: {} priority: {}", config.id, config.policy, config.priority);
        for (int i : config.cpu_affinity) {
            LogWarn("Thread id: {} cpu affinity: {}", config.id, i);
        }

        // 设置调度策略和优先级
        int policy = SCHED_OTHER;
        if (config.policy == "SCHED_FIFO") policy = SCHED_FIFO;
        else if (config.policy == "SCHED_RR") policy = SCHED_RR;

        sched_param param;
        param.sched_priority = config.priority;
        if (pthread_setschedparam(pthread_self(), policy, &param) != 0) {
            std::cerr << "Failed to set schedparam: " << std::strerror(errno) << std::endl;
            LogError("Failed to set schedparam: {}", std::strerror(errno));
        }

        // 设置 CPU 亲和性
        if (!config.cpu_affinity.empty()) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            for (int core : config.cpu_affinity) {
                CPU_SET(core, &cpuset);
            }
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
                std::cerr << "Failed to set CPU affinity: " << std::strerror(errno) << std::endl;
                LogError("Failed to set CPU affinity: {}", std::strerror(errno));
            }
        }

        // 执行线程主体
        entry();
    });
}
}
}
