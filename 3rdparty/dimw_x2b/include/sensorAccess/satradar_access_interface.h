/**
 * @file SatradarAccessInterface.h
 * @brief 
 * @author 
 * @version 1.0.0
 * @date 2023-09-14 09-21-02
 * @copyright Copyright (c) 2023 BYD Company Limited
 */

#ifndef CAMERA_ACCESS_INTERFACE__H
#define CAMERA_ACCESS_INTERFACE__H

#include "satradar_access_common.h"
#ifdef __cplusplus
#include <cstdint>
#include <vector>
extern "C" {
#else
#include <stdint.h>
#endif

/**
 * @brief   检测摄像头存在状态
 * @details 检测摄像头存在状态
 * @param[in]  size 给定存储摄像头ID缓存的大小
 * @param[out] ID[] 存储摄像头ID的缓存,需要足够 size 个元素的空间数组
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 *      CAMERA_CODE_INSUFFICIENT_MEMORY     内存不足
 */
CAMERA_ERR_CODE camera_detect_all_sensors(const uint32_t size, CAMERA_ID ID[]);
/**
 * @brief   检测摄像头存在状态
 * @details 检测摄像头存在状态
 * @param[in]  size 给定存储摄像头ID缓存的大小
 * @param[out] ID[] 存储摄像头ID的缓存,需要足够 size 个元素的空间数组
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 *      CAMERA_CODE_INSUFFICIENT_MEMORY     内存不足
 */
CAMERA_ERR_CODE satradar_detect_link_video_mask(uint16_t *linkMask, uint16_t *videoMask);

/**
 * @brief   使能指定摄像头
 * @details 使能指定摄像头
 * @param[in]  sensor_id 指定摄像头ID
 * @param[out] 无
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_sensor_enable(const CAMERA_ID sensor_id);

/**
 * @brief   去使能指定摄像头
 * @details 去使能指定摄像头
 * @param[in]  sensor_id 指定摄像头ID
 * @param[out] 无
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_sensor_disable(const CAMERA_ID sensor_id);

/**
 * @brief   获取指定摄像头模组信息
 * @details 获取指定摄像头模组信息
 * @param[in]  sensor_id 指定摄像头ID
 * @param[in]  number 指定获取几路摄像头模组信息
 * @param[out] module_info[] 摄像头模组信息，需要足够 number 个元素的数组
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_get_sensor_module_info(const CAMERA_ID sensor_id, const uint32_t number, sensor_module_info module_info[]);

/**
 * @brief   获取soc上电接入服务首次启动ptp时间
 * @details 获取soc上电接入服务首次启动ptp时间
 * @param[out] 返回ptpTime 
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_get_fsync_first_time(uint64_t * ptpTime);

/**
 * @brief 获取内参sn生成状态
 * @details 判断接入服务是否已经生成内参和SN文件
 * @param[out] state CAMERA_SN_VALID:已准备好，CAMERA_SN_INVALID：没准备好
 * @return CAMERA_ERR_CODE 
 * CAMERA_CODE_OK                           执行成功
 * CAMERA_CODE_PARAMETER_ERROR              参数错误
 * CAMERA_CODE_COMMUNICATE_FAILED           通信失败
 * CAMERA_CODE_EXECUTE_FAILED               执行失败
 */
CAMERA_ERR_CODE camera_get_sn_state(SN_STATE *state);

/**
 * @brief 重启sync接口
 * @return CAMERA_ERR_CODE 
 */
CAMERA_ERR_CODE camera_sync_restart();

#ifdef __cplusplus
}
#endif

#endif //! CAMERA_ACCESS_INTERFACE__H