
#include <cstring>
#include <errno.h>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <sys/resource.h>
#include <sys/syscall.h>

#include "common/fault_manager.h"
#include "rs_new_logger.h"
#include "thread_config.h"

namespace robosense::lidar::thread {

std::vector<ThreadConfig> thread_params;

std::thread createConfiguredStdThread(const ThreadConfig& config, std::function<void()> entry) {
    return std::thread([config, entry]() {
        LogWarn("Thread id: {} policy: {} priority: {}", config.id, config.policy, config.priority);
        for (int i : config.cpu_affinity) {
            LogWarn("Thread id: {} cpu affinity: {}", config.id, i);
        }

        // 设置调度策略和优先级
        int policy = SCHED_OTHER;
        if (config.policy == "SCHED_FIFO")
            policy = SCHED_FIFO;
        else if (config.policy == "SCHED_RR")
            policy = SCHED_RR;

        sched_param param;

        // 保存原始优先级用于设置nice值（仅SCHED_OTHER策略）
        int nice_value = 0;
        if (policy == SCHED_OTHER) {
            // 对于SCHED_OTHER，保存原始priority作为nice值
            nice_value = config.priority;
            // SCHED_OTHER必须使用优先级0
            param.sched_priority = 0;
            if (nice_value < -20 || nice_value > 19) {
                LogError("Invalid nice value: {}, nice value must be in range [-20, 19]", nice_value);
                LogWarn("Invalid nice value: {}, using -20 instead", nice_value);
                nice_value = -20;
            }
        } else {
            // 实时调度策略使用原始优先级
            param.sched_priority = config.priority;
            if ((param.sched_priority < 1) || (param.sched_priority > 99)) {
                LogError("Invalid priority: {}, priority must be in range [1, 99]", param.sched_priority);
                FaultManager8::getInstance().setFault(FaultBits8::LidarThreadSetError);
            }
        }

        // 设置调度参数
        if (pthread_setschedparam(pthread_self(), policy, &param) != 0) {
            std::cerr << "Failed to set schedparam: " << std::strerror(errno) << std::endl;
            LogError("Failed to set schedparam: {}", std::strerror(errno));
            FaultManager8::getInstance().setFault(FaultBits8::LidarThreadSetError);
        }

        // 对于SCHED_OTHER策略，设置nice值
        if (policy == SCHED_OTHER) {
            pid_t tid = syscall(SYS_gettid);
            if (setpriority(PRIO_PROCESS, tid, nice_value) != 0) {
                std::cerr << "Failed to set nice value: " << std::strerror(errno) << std::endl;
                LogError("Failed to set nice value: {}", std::strerror(errno));
                FaultManager8::getInstance().setFault(FaultBits8::LidarThreadSetError);
            } else {
                LogWarn("Thread id: {}, tid: {} set nice value: {}", config.id, tid, nice_value);
            }
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
                FaultManager8::getInstance().setFault(FaultBits8::LidarThreadSetError);
            }
        }

        // 执行线程主体
        entry();
    });
}

} // namespace robosense::lidar::thread
