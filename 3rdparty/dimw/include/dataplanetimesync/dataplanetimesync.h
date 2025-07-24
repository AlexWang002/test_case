/*
 * datatimesync - Implementation of gPTP(IEEE 802.1AS)
 * Copyright (C) 2019 Corporation
 *
 * This file is part of datatimesync.
 *
 * datatimesync is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * datatimesync is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with datatimesync.  If not, see
 * <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>.
 */
/**
 * @addtogroup gptp
 * @{
 * @file gptpmasterclock.h
 * @author Shiro Ninomiya
 * @copyright Copyright (C) 2017-2018 Corporation
 * @brief file contains gptp master clock related functions.
 *
 */

#ifndef __GPTPMASTERCLOCK_H_
#define __GPTPMASTERCLOCK_H_

#ifdef __cplusplus
extern "C"{
#else
#include <stdbool.h>
#endif
#include <stdint.h>

typedef enum {
    SYNC_UNLOCKED,
    SYNC_LOCKED
} sync_state;

typedef enum {
    INIT_UNFINISHED,
    INIT_FINISHED
} init_state;

typedef enum {
    MAC,
    GNSS,
    RTC,
    DILINK,
    LAST_SHUTDOWN,
    DEFAULT_TIME,
    BUTT,
} dataplant_clock_source_t;

/**
 * @brief 初始化接口，使用其它接口前必须保证已调用并且成功，多线程安全
 * @return 小于0 on error, 0 on Successful initialization.
 */
int tdsync_gptpmasterclock_init();

/**
 * @brief 释放相关的库资源，建议程序退出时调用，多线程安全
 * @return -1: on error, 0:on successfull
 */
void tdsync_gptpmasterclock_deinit();

/**
 * @brief 获取ptp时间，该接口性能最佳,建议主要使用该接口获取ptp时间
 * @return 返回大于等于0有效，小于0无效，单位ns
 */
int64_t tdsync_gptpmasterclock_getts64(void);

/**
*   @brief 根据网卡名获取对应的ptp设备，不同网卡有不同的ptp设备绑定
*   @param[inout] ptpdev  用于获取设备名的缓冲区，获取到的数据如“/dev/ptp1”
*   @param[in] len  用于获取设备名的缓冲区大小（注意：len >= MAX_PTPDEV_NAME(32)）
*   @return 小于0: on error, 0:on successfull
*/
int tdsync_gptpmasterclock_getptpdevname(char ptpdev[], int len);

/**
*   @brief 直接读取指定ptp设备的时间（注意：内部会重复打开ptp设备，效率低，推荐'gptpmasterclock_getts64'）
*   @param[in] ptpdev  指定ptp设备名，如"/dev/ptp1"
*   @return 返回大于等于0有效，小于0无效，单位ns
*/
int64_t tdsync_gptpmasterclock_getts64_ptpdev(const char *ptpdev);

/**
*   @brief 通过字符串设置ptp时间
*   @param[in] tstr 时间字符串（字符串格式如"1980-01-01 00:00:00"）
*   @return 小于0: on error, 0:on successfull
*/
int tdsync_gptpmasterclock_settime(char *tstr); // 设置数据面时间

/**
*   @brief 通过数字设置ptp时间，数字表示从1970-01-01 00:00:00的纳秒数
*   @param[in] time_stamp 时间数字（如'1568271454000000000'表示'Thu Sep 12 06:57:34 UTC 2019'）
*   @return 小于0: on error, 0:on successfull
*/
int tdsync_gptpmasterclock_settime_by_num(uint64_t time_stamp); // 设置数据面时间

/**
*   @brief 获取当前ptp时钟初始化状态和初始化源
*   @param[inout] init_state 初始化状态（true-已初始化 false-未初始化）
*   @param[inout] clocksource 初始化时间源（详见'dataplant_clock_source_t'）
*   @return 小于0: on error, 0:on successfull
*/
int tdsync_get_clocksource_init_status(bool *init_state, dataplant_clock_source_t *clocksource);

/**
*   @brief 获取当前ptp时钟同步状态
*   @param[inout] state 当前同步状态（true-成功 false-失败）
*   @return 小于0: on error, 0:on successfull
*/
int tdsync_get_gptp_sync_status(bool *state);

/**
*   @brief 获取ptp时间offset
*   @param[inout] ts64 offset值，单位ns
*   @return 小于0: on error, 0:on successfull
*/
int tdsync_get_gptp_sync_offset(int64_t *ts64);

/**
*   @brief 获取ptp时间offset
*   @param[inout] ts64 offset值，单位us
*   @return 小于0: on error, 0:on successfull
*/
int tdsync_get_gptp_sync_offset_us(int64_t *ts64);

#ifdef __cplusplus
}
#endif

#endif
/** @}*/
