/*******************************************************************************
 * \addtogroup utils
 * \{
 * \headerfile cpu_load.h "cpu_load.h"
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
#ifndef I_CPU_LOAD_H
#define I_CPU_LOAD_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <vector>
#include <atomic>
#include <map>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

namespace robosense::lidar::utils {

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/



/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
class ThreadInfo {
public:
  ThreadInfo() = default;

  pid_t tid;
  std::string name;
  std::string file;
  uint64_t last_thread_time{0UL};
  uint64_t last_total_cpu_time{0UL};
  double cpu_usage{0.0};
};

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern pid_t main_pid;
extern std::map<pid_t, ThreadInfo> g_threads;
extern std::atomic<bool> g_stop;
extern std::atomic<bool> g_save_bin;
extern std::atomic<uint64_t> fileCounter;
extern std::atomic<double> algo_threshold;

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
void monitThreads();
void addThread(pid_t pid, const std::string& name);
void saveBinFile(const void* mipi_data, uint32_t length);

double calculateCPUUsage(std::string const& file_name,
                        uint64_t prev_process_time, uint64_t prev_total_time,
                        const std::string & thread_name);
uint64_t getProcessCPUTime(std::string const& file_name);
uint64_t getTotalCPUTime();
} // namespace robosense::lidar::utils

/** \} utils */
#endif /* I_CPU_LOAD_H */