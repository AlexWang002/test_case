#ifndef I_THREAD_CONFIG_H
#define I_THREAD_CONFIG_H
#include "yaml_manager.h"
#include <thread>
#include <functional>
namespace robosense
{
namespace lidar
{
    std::thread createConfiguredStdThread(const yaml::ThreadConfig& config, std::function<void()> entry);
}
}

#endif /* I_THREAD_CONFIG_H */