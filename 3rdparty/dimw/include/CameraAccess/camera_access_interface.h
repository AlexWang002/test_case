/**
 * @file camera_access_interface.h
 * @brief 
 * @author 
 * @version 1.0.0
 * @date 2023-09-14 09-21-02
 * @copyright Copyright (c) 2023 BYD Company Limited
 */

#ifndef CAMERA_ACCESS_INTERFACE__H
#define CAMERA_ACCESS_INTERFACE__H

#include "camera_access_common.h"
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
 * @brief   控制整组摄像头上电
 * @details 控制整组摄像头上电
 * @param[in]  group_no  相机组编号 [0~3]
 * @param[out] 无
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_group_power_on(const uint32_t group_no);

/**
 * @brief   控制整组摄像头下电
 * @details 控制整组摄像头下电
 * @param[in]  group_no  相机组编号 [0~3]
 * @param[out] 无
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_group_power_off(const uint32_t group_no);
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
CAMERA_ERR_CODE sensor_detect_link_video_mask(uint16_t *linkMask, uint16_t *videoMask);

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
 * @brief 获取密文数据
 * @details 获取密文数据; 注意：该接口暂只支持IMX728类型前视摄像头
 * @param[in]  sensor_id 指定摄像头ID
 * @param[in]  number 指定获取几路摄像头密文数据
 * @param[out] data[] 透传密文数据的缓存，需要足够 number 个元素的数组
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 *      CAMERA_CODE_INSUFFICIENT_MEMORY     内存不足
 */
CAMERA_ERR_CODE camera_get_ciphertext_data(const CAMERA_ID sensor_id, const uint32_t number, sensor_ciphertext_data data[]);

/**
 * @brief 获取imu明文数据
 * @details 支持六部前视摄像头imu内参明文数据
 * @param[in]  sensor_id 指定摄像头ID
 * @param[in]  number 指定获取几路摄像头密文数据
 * @param[out] data[] 透传密文数据的缓存，需要足够 number 个元素的数组
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 *      CAMERA_CODE_INSUFFICIENT_MEMORY     内存不足
 */
CAMERA_ERR_CODE camera_get_imu_data(const CAMERA_ID sensor_id, const uint32_t number, sensor_imu_data data[]);

/**
 * @brief   设置摄像头曝光时长，并获取曝光时间
 * @details 设置摄像头曝光时长，并获取曝光时间; 注意：该接口暂只支持环视摄像头
 * @param[in]  number 设置的摄像头个数
 * @param[in]  in[] 曝光时间，需要有 number 个元素
 * @param[out] out[] 设置后的曝光时间，需要留有 number 个元素空间
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 *      CAMERA_CODE_INSUFFICIENT_MEMORY     内存不足
 */
CAMERA_ERR_CODE camera_set_max_exp_time(const uint32_t number, const sensor_exp_time in[], sensor_exp_time out[]);

/**
 * @brief   获取摄像头软件版本号
 * @details 获取摄像头软件版本号; 注意：该接口暂只支持环视摄像头
 * @param[in]  sensor_id 获取摄像头的ID
 * @param[in]  number 指定获取几路摄像头版本信息
 * @param[out] versions[] 摄像头软件版本信息，需要留有 number 个元素空间
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 *      CAMERA_CODE_INSUFFICIENT_MEMORY     内存不足
 */
CAMERA_ERR_CODE camera_get_sensor_verion(const CAMERA_ID sensor_id, const uint32_t number, sensor_firmware_version version[]);

/**
 * @brief   升级摄像头版本
 * @details 升级摄像头版本; 注意：该接口暂只支持环视摄像头
 * @param[in]  fw_path 固件路径
 * @param[in]  is_updata true表示当前是升级,false表示当前是rollback; sensor不支持rollback, 当前预留
 * @param[out] 无
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_upgrade_sensor_firmware(const char *fw_path, const bool is_updata = true);

/**
 * @brief   查询摄像头升级进度
 * @details 查询摄像头升级进度
 * @param[in]  无
 * @param[out] num 返回升级进度
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_get_sensor_upgrade_progress(uint32_t *num);

/**
 * @brief   获取IMU链路初始化状态
 * @details 获取IMU链路初始化状态
 * @param[in] state IMU链路初始化状态
 * @return 返回值 CAMERA_ERR_CODE
 *      CAMERA_CODE_OK                      执行成功
 *      CAMERA_CODE_PARAMETER_ERROR         参数错误
 *      CAMERA_CODE_COMMUNICATE_FAILED      通信失败
 *      CAMERA_CODE_EXECUTE_FAILED          执行失败
 */
CAMERA_ERR_CODE camera_get_imu_channel_state(CAMERA_IMU_CHANNEL_STATE *state);

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
 * @brief 获取摄像头工作状态
 * @details 判断接入服务是否启动ok，是否建链成功
 * @param[out] state true:已准备好，false：没准备好
 * @return CAMERA_ERR_CODE 
 * CAMERA_CODE_OK                           执行成功
 * CAMERA_CODE_PARAMETER_ERROR              参数错误
 * CAMERA_CODE_COMMUNICATE_FAILED           通信失败
 * CAMERA_CODE_EXECUTE_FAILED               执行失败
 */
CAMERA_ERR_CODE camera_get_state(bool *state);

/**
 * @brief  nvstream连接注册接口
 * @details consumer注册服务service名称以及ipc名称列表
 * @param[in] app_name  service名称
 * @param[in] ipc_names 需要链接的ipc channel列表
 * @return CAMERA_ERR_CODE
 */
CAMERA_ERR_CODE camera_reg_attach_channel(const char *app_name, const char ipc_names[][100], uint32_t ipc_number);

/**
 * @brief 重启sync接口
 * @return CAMERA_ERR_CODE 
 */
CAMERA_ERR_CODE camera_sync_restart();

#ifdef __cplusplus
}
#endif

#endif //! CAMERA_ACCESS_INTERFACE__H