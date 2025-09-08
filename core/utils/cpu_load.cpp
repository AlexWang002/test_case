/*******************************************************************************
 * \addtogroup utils
 * \{
 * \file cpu_load.cpp
 * \brief
 * \version 0.1
 * \date 2025-07-09
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-09 | Init version |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <sys/syscall.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include "rs_new_logger.h"
#include "yaml_manager.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "cpu_load.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

namespace robosense::lidar::utils {

/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/
pid_t main_pid;
std::atomic<double> algo_threshold{60.0};
std::atomic<uint64_t> fileCounter{1UL};
std::map<pid_t, ThreadInfo> g_threads;
std::atomic<bool> g_stop{false};
std::atomic<bool> g_save_bin{false};

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
// 分割字符串函数
inline std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream token_stream(str);

    while (std::getline(token_stream, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

// 获取系统总的 CPU 时间
uint64_t getTotalCPUTime() {
    std::ifstream stat_file("/proc/stat");
    std::string line;
    (void)std::getline(stat_file, line);
    std::vector<std::string> parts = split(line, ' ');
    uint64_t total_time = 0;

    for (size_t i = 2; i < parts.size(); ++i) {
        total_time += std::stoull(parts[i]);
    }

    return total_time;
}

// 获取指定进程的 CPU 时间
uint64_t getProcessCPUTime(std::string const& file_name) {
    std::ifstream stat_file(file_name);
    std::string line;
    std::getline(stat_file, line);
    std::vector<std::string> parts = split(line, ' ');
    // utime 在第 14 个字段，stime 在第 15 个字段
    uint64_t utime = std::stoull(parts[13]);
    uint64_t stime = std::stoull(parts[14]);

    return utime + stime;
}

double calculateCPUUsage(ThreadInfo& thread) {
    uint64_t cur_process_time = getProcessCPUTime(thread.file);
    uint64_t cur_total_time = getTotalCPUTime();
    double process_diff = static_cast<double>(cur_process_time) - static_cast<double>(thread.last_thread_time);
    double total_diff = static_cast<double>(cur_total_time) - static_cast<double>(thread.last_total_cpu_time);
    double result{0.0};

    process_diff = process_diff > 0.0 ? process_diff : 0.0;
    total_diff = total_diff > 0.0 ? total_diff : 0.0;

    if (total_diff > 1e-6) {
        result = (process_diff / total_diff) * 100.0 * sysconf(_SC_NPROCESSORS_ONLN);
    }

    // LogTrace("{}--{}, last {}, cur {}, lt {}, ct {}", thread.name, result, thread.last_thread_time, cur_process_time, thread.last_total_cpu_time, cur_total_time);

    thread.last_thread_time = cur_process_time;
    thread.last_total_cpu_time = cur_total_time;
    thread.cpu_usage = result;

    return result;
}

// 计算进程的 CPU 使用率
double calculateCPUUsage(std::string const& file_name,
                        uint64_t& last_process_time, uint64_t& last_total_time,
                        const std::string & thread_name) {
    uint64_t cur_process_time = getProcessCPUTime(file_name);
    uint64_t cur_total_time = getTotalCPUTime();
    uint64_t process_diff = cur_process_time - last_process_time;
    uint64_t total_diff = cur_total_time - last_total_time;

    process_diff = process_diff > 0 ? process_diff : 0;
    total_diff = total_diff > 0 ? total_diff : 0;

    if (0 == total_diff) {
        return 0.0;
    }

    double result = (static_cast<double>(process_diff) / static_cast<double>(total_diff)) * 100.0 * sysconf(_SC_NPROCESSORS_ONLN);
    last_process_time = cur_process_time;
    last_total_time = cur_total_time;

    return result;
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
void addThread(pid_t tid, const std::string& name) {
    std::string file = "/proc/" + std::to_string(main_pid) + "/task/" + std::to_string(tid) + "/stat";
    ThreadInfo thread_info;
    thread_info.tid = tid;
    thread_info.name = name;
    thread_info.file = file;
    LogWarn("addThread tid: {}, name: {} \n", tid, name);

    utils::g_threads[tid] = thread_info;
}

void monitThreads() {
    std::cout << "[Start cpu monitor thread]" << std::endl;

    //LogDebug("monitThreads interval_time: {}", yaml::demo_test_param.cpu_monitor_cycle);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // Sleep 2 seconds to wait for all threads to start.
    auto now = std::chrono::system_clock::now();
    uint64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    double algo_time{0.0};
    double main_cpu_usage{0.0};
    ThreadInfo main_thread;

    main_thread.tid = main_pid;
    main_thread.name = "main_process";
    main_thread.file = "/proc/" + std::to_string(main_pid) + "/stat";

    main_cpu_usage = calculateCPUUsage(main_thread);
    for (auto& kv : g_threads) {
        (void)calculateCPUUsage(kv.second);
    }

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        g_save_bin = false;
        now = std::chrono::system_clock::now();
        timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        algo_time = 0.0;
        main_cpu_usage = calculateCPUUsage(main_thread);

        std::cout << "cpu payload--------------" << std::endl;
        for (auto& kv : g_threads) {
            auto thread_name = kv.second.name;
            double thread_cpu_usage = calculateCPUUsage(kv.second);

            if ("handleMsopData" != thread_name) {
                algo_time += thread_cpu_usage;
            }
            std::cout << "[thread name] " << kv.second.name 
                << ", [cpu usage]: " << thread_cpu_usage << std::endl;
        }
        std::cout << "--------------" << std::endl << std::endl;;

        if (algo_time > algo_threshold) {
            LogInfo("algo_time:{} > algo_threshold: {}", algo_time, algo_threshold);
            g_save_bin = true;
            LogInfo("SDK main process name: {}, cpu_usage: {}", main_thread.name, main_thread.cpu_usage);
            LogInfo("size: {}, names: ", g_threads.size());
            for (auto& kv : g_threads) {
                LogInfo("name: {}, cpu_usage: {}", kv.second.name, kv.second.cpu_usage);
            }
        } else {
        }
    }
}

/******************************************************************************/
/*          Definition of public functions of classes or templates             */
/******************************************************************************/

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/

} // namespace robosense::lidar::utils

/* \}  utils */