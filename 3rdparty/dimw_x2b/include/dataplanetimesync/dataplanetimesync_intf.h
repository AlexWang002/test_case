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

#ifndef __GPTPMASTERCLOCKINTF_H_
#define __GPTPMASTERCLOCKINTF_H_

#ifdef __cplusplus
extern "C"{
#else
#include <stdbool.h>
#endif
#include <stdint.h>

/**
*   @brief 打开网卡对应的ptp设备，不同网卡有不同的ptp设备绑定
*   @return 返回0成功，小于0失败
*/
int gptpmasterclock_getptpdevname(void);

/**
*   @brief 直接读取指定ptp设备的时间
*   @return 返回大于0有效，-1无效
*/
int64_t gptpmasterclock_getts64_ptpdev(void);

#ifdef __cplusplus
}
#endif

#endif
/** @}*/
