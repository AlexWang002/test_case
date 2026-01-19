/*******************************************************************************
 * \addtogroup driver
 * \{
 * \file algo.cpp
 * \brief
 * \version 0.3
 * \date 2025-06-27
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-06-04 | Init version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-06-20 | Trail&Denoise thread merge |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.3 | 2025-06-27 | Fix Denoise last row data lead to
 *                             accidentally delete issue;
 *                             Add get algorithm switch from yaml|
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.4 | 2025-07-14 | Move ground fit to thread 0 |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.5 | 2025-08-11 | Optimize trail&denoise&filter algorithms load |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include "cpu_load.h"
#include "common/fault_manager.h"
#include "denoise.h"
#include "trail.h"
#include "stray.h"
#include "spray.h"
#include "upsample.h"
#include "highcalc.h"
#include "json_reader.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "algo.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense
{
namespace lidar
{
/**
 * \brief  Take the remainder of a number
 *
 * \param[in] a : original input data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] b : remainder input data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return int : output remainder result
 */
inline int AlgoFunction::matlabMod(int a, int b){
    return (a % b + b) % b;  // 确保结果非负
}

/**
 * \brief  Matrix calculate inverse function
 *
 * \param[in] mat: input matrix
 *              Range: N/A. Accuracy: N/A.
 * \param[out] inv: output inverse matrix
 *              Range: N/A. Accuracy: N/A.
 */
void AlgoFunction::matrixInverse(const Matrix3x3& mat, Matrix3x3& inv) {
    // 计算行列式
    float det = mat(0,0)*(mat(1,1)*mat(2,2) - mat(1,2)*mat(2,1))
              - mat(0,1)*(mat(1,0)*mat(2,2) - mat(1,2)*mat(2,0))
              + mat(0,2)*(mat(1,0)*mat(2,1) - mat(1,1)*mat(2,0));

    if (std::fabs(det) < 1e-6f) {
        // 不可逆时返回单位矩阵
        for (int i = 0; i < 9; i++) inv.data[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        return;
    }

    float inv_det = 1.0f / det;

    inv(0,0) = (mat(1,1)*mat(2,2) - mat(1,2)*mat(2,1)) * inv_det;
    inv(0,1) = (mat(0,2)*mat(2,1) - mat(0,1)*mat(2,2)) * inv_det;
    inv(0,2) = (mat(0,1)*mat(1,2) - mat(0,2)*mat(1,1)) * inv_det;

    inv(1,0) = (mat(1,2)*mat(2,0) - mat(1,0)*mat(2,2)) * inv_det;
    inv(1,1) = (mat(0,0)*mat(2,2) - mat(0,2)*mat(2,0)) * inv_det;
    inv(1,2) = (mat(0,2)*mat(1,0) - mat(0,0)*mat(1,2)) * inv_det;

    inv(2,0) = (mat(1,0)*mat(2,1) - mat(1,1)*mat(2,0)) * inv_det;
    inv(2,1) = (mat(0,1)*mat(2,0) - mat(0,0)*mat(2,1)) * inv_det;
    inv(2,2) = (mat(0,0)*mat(1,1) - mat(0,1)*mat(1,0)) * inv_det;
}

/**
 * \brief  Matrix calculate vector mutilply function
 *
 * \param[in] A: input matrix
 *              Range: N/A. Accuracy: N/A.
 * \param[in] v: input vector
 *              Range: N/A. Accuracy: N/A.
 * \param[out] result: output mutilply matrix
 *              Range: N/A. Accuracy: N/A.
 */
void AlgoFunction::matVecMulti(const Matrix3x3& A, const Vector3& v, Vector3& result) {
    // 矩阵-向量乘法: A(3x3) * v(3x1) -> result(3x1)
    result[0] = A(0,0)*v[0] + A(0,1)*v[1] + A(0,2)*v[2];
    result[1] = A(1,0)*v[0] + A(1,1)*v[1] + A(1,2)*v[2];
    result[2] = A(2,0)*v[0] + A(2,1)*v[1] + A(2,2)*v[2];
}

/**
 * \brief  Select ground fit main function
 *
 * \param[out] best_model: output best model result
 *              Range: N/A. Accuracy: N/A.
 */
void AlgoFunction::selectGroundFitFunc(Vector3& best_model)
{
    const int rows = VIEW_H; // 192
    const int cols = GND_VIEW_W;  // 152 下采样后列数: 760 / 5 = 152
    const int dist_th = 300;
    const float roi_z_center = -300.0f;
    int idx;
    int next_idx;
    int diff_idx;

    // 差分矩阵
    int dist_diff[GND_LEN];
    int X_diff[GND_LEN];
    int Z_diff[GND_LEN];

    // 掩码矩阵
    bool near_mask[GND_LEN];
    bool near_down[GND_ORI_LEN] = {false};
    bool near_up[GND_ORI_LEN] = {false};

    // 几何特征计算
    int norm_cur2neib_square[GND_LEN];
    float norm_cur2neib[GND_LEN];
    int norm_neib_square[GND_ORI_LEN];

    // 计算差分
    for (int r = 0; r < rows - 1; r++) {
        for (int c = 0; c < cols; c++) {
            int idx = r * cols + c;
            int pre_idx = (r - 1) * cols + c;
            int next_idx = (r + 1) * cols + c;

            dist_diff[idx] = dist_wave0_buffer6[next_idx] - dist_wave0_buffer6[idx];
            X_diff[idx] = X_buffer6[next_idx] - X_buffer6[idx];
            Z_diff[idx] = Z_buffer6[next_idx] - Z_buffer6[idx];

            // 近邻掩码
            near_mask[idx] = std::abs(dist_diff[idx]) < dist_th;
            near_down[idx] = near_mask[idx];       // 下邻点
            near_up[next_idx] = near_mask[idx];    // 上邻点

            int x_sq = X_diff[idx] * X_diff[idx];
            int z_sq = Z_diff[idx] * Z_diff[idx];
            norm_cur2neib_square[idx] = x_sq + z_sq;
            norm_cur2neib[idx] = std::sqrt((float)norm_cur2neib_square[idx]);
        }
    }

    // 间隔点差分
    int X_diff2[GND_ORI_LEN] = {0};
    int Z_diff2[GND_ORI_LEN] = {0};
    // 角度计算
    int a_square[GND_ORI_LEN] = {0};
    int b_square[GND_ORI_LEN] = {0};
    float a[GND_ORI_LEN] = {0};
    float b[GND_ORI_LEN] = {0};
    float cos_theta[GND_ORI_LEN] = {0};
    // 地面平行掩码
    bool cos_ground_mask[GND_LEN];
    bool parallel_down[GND_ORI_LEN] = {false};
    bool parallel_up[GND_ORI_LEN] = {false};
    // ROI区域掩码
    bool mask_roi[GND_ORI_LEN] = {false};
    int cnt_roi = 0;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int idx = r * cols + c;
            int pre_idx = (r - 1) * cols + c;
            int next_idx = (r + 1) * cols + c;
            if(0 == r){
                X_diff2[idx] = 0 - X_buffer6[next_idx];
                Z_diff2[idx] = 0 - Z_buffer6[next_idx];
            }
            else if(r >= 1 && r < rows - 1){
                X_diff2[idx] = X_buffer6[pre_idx] - X_buffer6[next_idx];
                Z_diff2[idx] = Z_buffer6[pre_idx] - Z_buffer6[next_idx];
            }
            else{
                X_diff2[idx] = X_buffer6[pre_idx] - 0;
                Z_diff2[idx] = Z_buffer6[pre_idx] - 0;
            }
            int x2_sq = X_diff2[idx] * X_diff2[idx];
            int z2_sq = Z_diff2[idx] * Z_diff2[idx];
            norm_neib_square[idx] = x2_sq + z2_sq;

            // 设置a和b
            if (r > 0) {
                int prev_idx = (r-1) * cols + c;
                a_square[idx] = norm_cur2neib_square[prev_idx];
                a[idx] = norm_cur2neib[prev_idx];
            }
            if (r < rows-1) {
                b_square[idx] = norm_cur2neib_square[idx];
                b[idx] = norm_cur2neib[idx];
            }

            // 计算cos_theta (避免除以零)
            if (a[idx] > 1e-6 && b[idx] > 1e-6) {
                cos_theta[idx] = (a_square[idx] + b_square[idx] - norm_neib_square[idx])
                                 / (2.0f * a[idx] * b[idx]);
            } else {
                cos_theta[idx] = 1.0f; // 无效值
            }

            if(r < rows - 1)
            {
                if (norm_cur2neib[idx] > 1e-6) {
                    cos_ground_mask[idx] = std::fabs(X_diff[idx] / norm_cur2neib[idx]) > 0.86f;
                    parallel_down[idx] = cos_ground_mask[idx];      // 下邻点
                    parallel_up[(r+1)*cols+c] = cos_ground_mask[idx]; // 上邻点
                }
            }

            mask_roi[idx] = (X_buffer6[idx] > 0) &&
                           (X_buffer6[idx] < 10000) &&  // 50*200 = 10000
                           (std::fabs(Y_buffer6[idx]) < 2000) &&   // 10*200 = 2000
                           (std::fabs(Z_buffer6[idx] + 300) < 600); // 3*200=600

            if (mask_roi[idx]) cnt_roi++;
        }
    }

    // 地面点预筛选
    bool mask_ground_pre[GND_ORI_LEN] = {false};
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int idx = r * cols + c;
            mask_ground_pre[idx] = mask_roi[idx] &&
                                  near_down[idx] &&
                                  near_up[idx] &&
                                  parallel_down[idx] &&
                                  parallel_up[idx] &&
                                  cos_theta[idx] < -0.86f;
        }
    }

    // 形态学腐蚀 (5x5内核)
    bool mask_ground[GND_ORI_LEN] = {false};
    const int kernel_radius = 2; // 5x5内核

    for (int r = kernel_radius; r < rows - kernel_radius; r++) {
        for (int c = kernel_radius; c < cols - kernel_radius; c++) {
            int idx = r * cols + c;
            if (!mask_ground_pre[idx]) continue;

            bool all_ones = true;
            for (int i = -kernel_radius; i <= kernel_radius && all_ones; i++) {
                for (int j = -kernel_radius; j <= kernel_radius; j++) {
                    int neighbor_idx = (r+i)*cols + (c+j);
                    if (!mask_ground_pre[neighbor_idx]) {
                        all_ones = false;
                        break;
                    }
                }
            }
            mask_ground[idx] = all_ones;
        }
    }

    // 收集地面点样本
    int x_sample[GND_ORI_LEN / 2] = {0};
    int y_sample[GND_ORI_LEN / 2] = {0};
    int z_sample[GND_ORI_LEN / 2] = {0};
    int cnt_sample = 0;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int idx = r * cols + c;
            if (mask_ground[idx]) {
                x_sample[cnt_sample] = X_buffer6[idx];
                y_sample[cnt_sample] = Y_buffer6[idx];
                z_sample[cnt_sample] = Z_buffer6[idx];
                cnt_sample++;
            }
        }
    }

    // 默认模型 (没有足够点时的返回值)
    best_model[0] = 0.0f;
    best_model[1] = 0.0f;
    best_model[2] = roi_z_center;

    // 样本不足直接返回
    if (cnt_sample < 100) {
        return;
    }

    // 构建线性系统: X_mat * model = z_vec
    // 其中 X_mat = [x, y, 1], model = [a, b, c], z_vec = z
    Matrix3x3 A = {0};
    Vector3 b_vec = {0};

    // 计算 X_mat'*X_mat 和 X_mat'*z_vec
    for (size_t i = 0; i < cnt_sample; i++) {
        float x = x_sample[i];
        float y = y_sample[i];
        float z = z_sample[i];

        // 更新 A = X_mat' * X_mat
        A(0,0) += x * x;
        A(0,1) += x * y;
        A(0,2) += x;
        A(1,1) += y * y;
        A(1,2) += y;
        A(2,2) += 1;

        // 更新 b_vec = X_mat' * z_vec
        b_vec[0] += x * z;
        b_vec[1] += y * z;
        b_vec[2] += z;
    }

    // 对称部分
    A(1,0) = A(0,1);
    A(2,0) = A(0,2);
    A(2,1) = A(1,2);

    // 求解线性系统: A * model = b_vec
    Matrix3x3 A_inv;
    matrixInverse(A, A_inv);
    matVecMulti(A_inv, b_vec, best_model);

    // 残差分析
    int best_cnt = 0;
    const float residual_threshold = 10.0f; // 0.05*200 = 10

    for (size_t i = 0; i < cnt_sample; i++) {
        float z_pred = best_model[0] * x_sample[i] +
                       best_model[1] * y_sample[i] +
                       best_model[2];
        float residual = std::fabs(z_sample[i] - z_pred);
        if (residual < residual_threshold) {
            best_cnt++;
        }
    }

    // 检查地面点比例
    if (best_cnt < cnt_roi * 0.2f) {
        best_model[0] = 0.0f;
        best_model[1] = 0.0f;
        best_model[2] = roi_z_center;
    }
}

/**
 * \brief  Calculate circular index of the buffer
 *
 * \param[in] rear: rear index of the buffer
 *              Range: N/A. Accuracy: N/A.
 * \param[in] size: size of the buffer
 *              Range: N/A. Accuracy: N/A.
 */
inline void AlgoFunction::circularCalcIdx(int& rear, int size)
{
    rear = (rear + 1 + size) % size;
}

/**
 * \brief  Update ground fit params
 */
void AlgoFunction::updateGroundFitParams(void)
{
    //更新地面拟合结果
    fit_Params.a = best_model_fit[0];
    fit_Params.b = best_model_fit[1];
    fit_Params.c = best_model_fit[2];
}

/**
 * \brief  Algorithm params initialize
 */
void AlgoFunction::algoInit()
{
    pc_Param.initialize();
    best_model_fit[0] = 0;
    best_model_fit[1] = 0;
    best_model_fit[2] = -300.0f;

    if (json_reader::algo_switch.data_valid) {
        LogInfo("Using algorithm switch in json file.");
        algo_Param.TrailRemoveOn = json_reader::algo_switch.enable_trail_remove;
        algo_Param.DenoiseOn = json_reader::algo_switch.enable_denoise;
        algo_Param.StrayRemoveOn = json_reader::algo_switch.enable_stray;
        algo_Param.SprayRemoveOn = json_reader::algo_switch.enable_spray;
        algo_Param.SprayDeleteOn = json_reader::algo_switch.enable_delete;
    } else {
        LogWarn(__FILE__, __LINE__, __func__, "Algorithm switch read from json file FAILED, Using default switch.");
    }

    LogDebug(__FILE__, __LINE__, __func__,
            "TrailRemoveOn: {}", algo_Param.TrailRemoveOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "DenoiseOn: {}", algo_Param.DenoiseOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "StrayRemoveOn: {}", algo_Param.StrayRemoveOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "SprayRemoveOn: {}", algo_Param.SprayRemoveOn);
}

/**
 * \brief  Submit pva task with retry
 *
 * \param[in] func: pva task submission function
 * \param[in] algo_name: algo name
 */
void AlgoFunction::submitPvaTaskWithRetry(PvaFunc func, std::string algo_name)
{
    static int cnt{0};
    std::string exception_msg;
    int32_t status_code;
    int ret = 0, retry_cnt = 0;
    uint32_t submit_time, wait_time;

    if (algo_name == "algo1") {
        cnt = (cnt + 1) % 10;
    }

    for (retry_cnt = 0; retry_cnt < RETRY_CNT; retry_cnt ++) {
        auto time_start = std::chrono::steady_clock::now();
        ret = func(exception_msg, status_code, submit_time, wait_time);
        auto time_end = std::chrono::steady_clock::now();
        auto time_duration = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start);

        if (ret == 0) {
            if (this->algo_delay_switch_ && cnt == 0) {
                LogInfo("[{}] {}us.", algo_name, time_duration.count());
                LogInfo("[{}] submit {}us, wait {}us.",
                                algo_name, submit_time, wait_time);
            }

            break;
        }
        if (ret == 1) {
            LogWarn("[{}] Caught a cuPVA exception with message {}, process time {}us.", algo_name, exception_msg, time_duration.count());
        }
        else if (ret == 2) {
            LogWarn("[{}] VPU Program returned an Error Code {}, process time {}us.", algo_name, status_code, time_duration.count());
        }
    }
    if (ret != 0) {
        LogError("[{}] Fail to submit pva task {} times!", algo_name, RETRY_CNT);
    }
    else if (retry_cnt > 0) {
        LogWarn("[{}] PVA task submitted {} times.", algo_name, retry_cnt + 1);
    }
}

/**
 * \brief  Denoise algorithm main process in pva
 *
 * \param[in] pstFrameBuffer: frame buffer
 *              Range: N/A. Accuracy: N/A.
 */
void AlgoFunction::denoiseExec(tstFrameBuffer* pstFrameBuffer)
{
    if (algo_Param.DenoiseOn) {
        /*拷贝整帧数据到denoise算法的PVA buffer中*/
        memcpy(denoise_dist_buffer_h, pstFrameBuffer->dist0,  VIEW_H * VIEW_W * sizeof(uint16_t));

        submitPvaTaskWithRetry(std::bind(denoiseProcPva, 
                                        std::placeholders::_1, std::placeholders::_2, 
                                        std::placeholders::_3, std::placeholders::_4), 
                                        "algo1");
    }
    else {
        for (int cc = 0; cc < VIEW_W; cc ++) {
            for (int rr = 0; rr < VIEW_H; rr ++) {
                denoise_mask_out_frm[cc][rr] = 1;
            }
        }
    }
}

/**
 * \brief  Trail algorithm main process in pva
 *
 * \param[in] pstFrameBuffer: frame buffer
 *              Range: N/A. Accuracy: N/A.
 */
void AlgoFunction::trailExec(tstFrameBuffer* pstFrameBuffer)
{
    if (algo_Param.TrailRemoveOn) {
        memcpy(DistIn_h, pstFrameBuffer->dist0, sizeof(uint16_t) * VIEW_W * VIEW_H);

        submitPvaTaskWithRetry(std::bind(trailProcPva, 
                                        std::placeholders::_1, std::placeholders::_2, 
                                        std::placeholders::_3, std::placeholders::_4), 
                                        "algo2");
    }
    else {
        for (int cc = 0; cc < VIEW_W; cc ++) {
            for (int rr = 0; rr < VIEW_H; rr ++) {
                trail_mask_out_frm[cc][rr] = 0;
            }
        }
    }
}

/**
 * \brief  Stray-delete algo function
 *
 * \param[in] pstFrameBuffer: frame buffer
 */
void AlgoFunction::strayDeleteExec(tstFrameBuffer* pstFrameBuffer)
{
    thread_local int cnt{0};
    cnt = (cnt + 1) % 10;

    if (algo_Param.StrayRemoveOn) {
        for (int col_idx = 0; col_idx < VIEW_W; col_idx ++) {
            gndHeightCalc(col_idx, pstFrameBuffer);
        }

        //拷贝计算好的标签及部分原始数据到pva host buffer
        for (int i = 0; i < 76; i ++) {
            memcpy((uint16_t *)&stray_pva_buff.dist_wave_h[i * 10 * 192 * 2], pstFrameBuffer->dist0[i * 10], 10 * 192 * sizeof(uint16_t));
            memcpy((uint16_t *)&stray_pva_buff.dist_wave_h[i * 10 * 192 * 2 + 10 * 192], pstFrameBuffer->dist1[i * 10], 10 * 192 * sizeof(uint16_t));

            memcpy((uint16_t *)&stray_pva_buff.raw_data_h[i * 10 * 192 * 6], pstFrameBuffer->ref0[i * 10], 10 * 192 * sizeof(uint16_t));
            memcpy((uint16_t *)&stray_pva_buff.raw_data_h[i * 10 * 192 * 6 + 10 * 192], pstFrameBuffer->ref1[i * 10], 10 * 192 * sizeof(uint16_t));

            memcpy((uint16_t *)&stray_pva_buff.raw_data_h[i * 10 * 192 * 6 + 20 * 192], pstFrameBuffer->gnd_mark0[i * 10], 10 * 192 * sizeof(uint16_t));
            memcpy((uint16_t *)&stray_pva_buff.raw_data_h[i * 10 * 192 * 6 + 30 * 192], pstFrameBuffer->gnd_mark1[i * 10], 10 * 192 * sizeof(uint16_t));

            memcpy((uint16_t *)&stray_pva_buff.raw_data_h[i * 10 * 192 * 6 + 40 * 192], pstFrameBuffer->high0[i * 10], 10 * 192 * sizeof(uint16_t));
            memcpy((uint16_t *)&stray_pva_buff.raw_data_h[i * 10 * 192 * 6 + 50 * 192], pstFrameBuffer->high1[i * 10], 10 * 192 * sizeof(uint16_t));
        }

        memcpy(stray_pva_buff.att0_h, pstFrameBuffer->att0[0], 760 * 192 * sizeof(uint16_t));
        memcpy(stray_pva_buff.att1_h, pstFrameBuffer->att1[0], 760 * 192 * sizeof(uint16_t));

        setRainWallParams(RainWall_in.cnt, RainWall_in.dist);
        submitPvaTaskWithRetry(std::bind(strayProcPva,
                                        std::placeholders::_1, std::placeholders::_2, 
                                        std::placeholders::_3, std::placeholders::_4), 
                                        "algo4");

    }
    else {
        for (int cc = 0; cc < VIEW_W; cc ++) {
            for (int rr = 0; rr < VIEW_H; rr ++) {
                stray_mask_out_frm0[cc][rr] = 0;
                stray_mask_out_frm1[cc][rr] = 0;
            }
        }
    }
}


/**
 * @brief calculate ground hieght for stray delete algorithm
 *
 * @param[in] col_idx column index of the buffer
 *                  Range: 0 - 759. Accuracy: 1.
 * @param[in] stray_col_buffer stray buffer column index
 *                  Range: 0 - 2. Accuracy: 1.
 */
void AlgoFunction::gndHeightCalc(int col_idx, tstFrameBuffer* pstFrameBuffer) {
    if(col_idx < 0 || col_idx >= VIEW_W){
        return;
    }

    static int stray_var_cnt{0};
    if (col_idx == 0) {
        stray_var_cnt = 0;
    }

    if (algo_Param.StrayRemoveOn) {
        uint16_t *ground_height_pva = (uint16_t *)&stray_pva_buff.ground_height_h[col_idx * VIEW_H];

        // 地面标记表
        int gnd_cnt_tab[28] = {0};
        int gnd_row_ed_tab[28] = {0};
        int gnd_height_st_tab[28];
        std::fill_n(gnd_height_st_tab, 28, 32767);
        int gnd_dist_max = 0;
        int gnd_row_farest = 0;

        uint16_t* dist0 = pstFrameBuffer->dist0[col_idx];
        uint16_t* dist1 = pstFrameBuffer->dist1[col_idx];
        uint16_t* grnd0 = pstFrameBuffer->gnd_mark0[col_idx];
        uint16_t* grnd1 = pstFrameBuffer->gnd_mark1[col_idx];
        int16_t* high0 = pstFrameBuffer->high0[col_idx];
        int16_t* high1 = pstFrameBuffer->high1[col_idx];

        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            // 获取当前行数据
            int dist_cur = dist0[row_idx];
            int dist_cur2 = dist1[row_idx];

            int gnd_cur = grnd0[row_idx];
            int gnd_cur2 = grnd1[row_idx];

            int height_cur = high0[row_idx];
            int height_cur2 = high1[row_idx];

            uint16_t gnd_seg = dist_cur >> 9;
            if (gnd_seg >= 28) gnd_seg = 27;

            ground_height_pva[row_idx] = gnd_seg;

            uint16_t gnd_seg2 = dist_cur2 >> 9;
            if (gnd_seg2 >= 28) gnd_seg2 = 27;

            // ------ 地面标记统计 ------
            if (dist_cur > 0 && dist_cur < 14000 && gnd_cur) {

                if (gnd_cnt_tab[gnd_seg] == 0) {
                    gnd_cnt_tab[gnd_seg] = 1;
                    gnd_row_ed_tab[gnd_seg] = row_idx;
                    gnd_height_st_tab[gnd_seg] = height_cur;
                } else if (row_idx - gnd_row_ed_tab[gnd_seg] <= 3 &&
                            std::abs(height_cur - gnd_height_st_tab[gnd_seg]) < 50) {
                    gnd_cnt_tab[gnd_seg] = std::min(32, gnd_cnt_tab[gnd_seg] + 1);
                    gnd_row_ed_tab[gnd_seg] = row_idx;
                }

                if ((dist_cur > gnd_dist_max && dist_cur - gnd_dist_max < 800 && row_idx - gnd_row_farest < 15) ||
                        gnd_dist_max == 0) {
                    gnd_dist_max = dist_cur;
                    gnd_row_farest = row_idx;
                }
            }
            else if (dist_cur2 > 0 && dist_cur2 < 14000 && gnd_cur2) {

                if (gnd_cnt_tab[gnd_seg2] == 0) {
                    gnd_cnt_tab[gnd_seg2] = 1;
                    gnd_row_ed_tab[gnd_seg2] = row_idx;
                    gnd_height_st_tab[gnd_seg2] = height_cur2;
                } else if (row_idx - gnd_row_ed_tab[gnd_seg2] <= 3 &&
                            std::abs(height_cur2 - gnd_height_st_tab[gnd_seg2]) < 50) {
                    gnd_cnt_tab[gnd_seg2] = std::min(32, gnd_cnt_tab[gnd_seg2] + 1);
                    gnd_row_ed_tab[gnd_seg2] = row_idx;
                }

                if ((dist_cur2 > gnd_dist_max && dist_cur2 - gnd_dist_max < 800 && row_idx - gnd_row_farest < 15) ||
                        gnd_dist_max == 0) {
                    gnd_dist_max = dist_cur2;
                    gnd_row_farest = row_idx;
                }
            }
        }

        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            uint16_t ground_seg = ground_height_pva[row_idx];
            uint16_t ground_height = gnd_height_st_tab[ground_seg];
            if (ground_seg >= 9) {
                if (gnd_cnt_tab[ground_seg] == 0) {
                    if (gnd_cnt_tab[ground_seg - 1] > 0) {
                        ground_height = gnd_height_st_tab[ground_seg - 1];
                    } else if (gnd_cnt_tab[ground_seg - 2] > 0) {
                        ground_height = gnd_height_st_tab[ground_seg - 2];
                    }
                }
            }

            ground_height_pva[row_idx] = ground_height;
        }

        stray_pva_buff.stray_var_h[stray_var_cnt] = gnd_dist_max;
        stray_var_cnt += STRAY_VAR_CNT;
    }
}

/**
 * @brief stray delete algorithm of lidar
 *
 * @param[in] col_idx column index of the buffer
 *                  Range: 0 - 759. Accuracy: 1.
 * @param[in] stray_col_buffer stray buffer column index
 *                  Range: 0 - 2. Accuracy: 1.
 */
void AlgoFunction::strayDelete(int col_idx, tstFrameBuffer* pstFrameBuffer) {
    if(col_idx < 0 || col_idx >= VIEW_W){
        return;
    }

    static int stray_var_cnt{0};
    if (col_idx == 0) {
        stray_var_cnt = 0;
    }

    if (algo_Param.StrayRemoveOn) {
        static int ceil_stray_dist[3] = {0};
        static int stray_chain_cnt[3] = {0};
        static int stray_chain_dist[3] = {0};
        static int stray_chain_row_st[3] = {0};
        static int stray_chain_row_ed[3] = {0};

        static int stray_conn_dist = 0;
        static int stray_conn_cols = 0;
        static int stray_conn_cnt = 0;
        static int stray_conn_col_st = 0;
        static int stray_conn_col_ed = 0;

        static int rain_wall_cnt = 0;
        static int rain_wall_dist = 0;
        static int stray_chain_height[3] = {0};

        if(0 == col_idx){
            memset(ceil_stray_dist, 0, sizeof(ceil_stray_dist));
            memset(stray_chain_cnt, 0, sizeof(stray_chain_cnt));
            memset(stray_chain_dist, 0, sizeof(stray_chain_dist));
            memset(stray_chain_row_st, 0, sizeof(stray_chain_row_st));
            memset(stray_chain_row_ed, 0, sizeof(stray_chain_row_ed));
            stray_conn_dist = 0;
            stray_conn_cols = 0;
            stray_conn_cnt = 0;
            stray_conn_col_st = 0;
            stray_conn_col_ed = 0;
            rain_wall_cnt = 0;
            rain_wall_dist = 0;
        }

        // 滑动窗口更新
        ceil_stray_dist[0] = ceil_stray_dist[1];
        ceil_stray_dist[1] = ceil_stray_dist[2];
        ceil_stray_dist[2] = 0;

        // 聚类变量
        uint16_t class_line[VIEW_H] = {0};
        uint16_t blue_line[VIEW_H] = {0};
        uint16_t pva_line[VIEW_H] = {0};

        int class_row_st = 0;
        int class_row_ed = 0;
        int class_cnt = 0;
        int class_dist = 0;
        int class_stray_cnt = 0;
        int class_stray_row = 0;
        int class_stray_type = 0; //当前最新的杂散聚类块是否是与普通块相连
        int class_cnt_last = 0; //上一个杂散聚类块点数
        int class_dist_last = 0; //上一个杂散聚类块距离
        int class_stray_cnt_last = 0; //上一个杂散聚类块杂散点个数

        int normal_dist = 0;
        int normal_row_st = -1;
        int normal_row_ed = 0;
        int normal_height_st = 0;
        int normal_height_ed = 0;
        int normal_bright = 0;

        int stray_chain_points_c = 0;
        int stray_chain_cnt_c = 0;
        int stray_chain_dist_c = 0;
        int stray_chain_row_st_c = 0;
        int stray_chain_row_ed_c = 0;
        int stray_chain_ht_st_c = 0; //当列最长的杂散链起始高度
        int stray_chain_ht_ed_c = 0; //当列最长的杂散链终止高度

        int class_height_st = 0;
        int class_height_ed = 0;

        // 更新全局杂散链数组
        stray_chain_cnt[0] = stray_chain_cnt[1];
        stray_chain_cnt[1] = stray_chain_cnt[2];
        stray_chain_cnt[2] = stray_chain_cnt_c;

        stray_chain_dist[0] = stray_chain_dist[1];
        stray_chain_dist[1] = stray_chain_dist[2];
        stray_chain_dist[2] = stray_chain_dist_c;

        stray_chain_row_st[0] = stray_chain_row_st[1];
        stray_chain_row_st[1] = stray_chain_row_st[2];
        stray_chain_row_st[2] = stray_chain_row_st_c;

        stray_chain_row_ed[0] = stray_chain_row_ed[1];
        stray_chain_row_ed[1] = stray_chain_row_ed[2];
        stray_chain_row_ed[2] = stray_chain_row_ed_c;

        stray_chain_height[0] = stray_chain_height[1];
        stray_chain_height[1] = stray_chain_height[2];
        stray_chain_height[2] = 0;

        // =============== 前级信息计算 ===============
        uint16_t* dist0 = pstFrameBuffer->dist0[col_idx];
        uint16_t* dist1 = pstFrameBuffer->dist1[col_idx];
        uint16_t* ref0 = pstFrameBuffer->ref0[col_idx];
        uint16_t* ref1 = pstFrameBuffer->ref1[col_idx];
        uint16_t* att0 = pstFrameBuffer->att0[col_idx];
        uint16_t* att1 = pstFrameBuffer->att1[col_idx];
        int16_t* high0 = pstFrameBuffer->high0[col_idx];
        int16_t* high1 = pstFrameBuffer->high1[col_idx];

        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            // 获取当前行数据
            int dist_cur = dist0[row_idx];
            int dist_cur2 = dist1[row_idx];

            int ref_cur = ref0[row_idx];
            int ref_cur2 = ref1[row_idx];

            int row_idx_up = std::max(0, row_idx - 1);
            int dist_up = dist0[row_idx_up];
            int dist_up2 = dist1[row_idx_up];

            int crosstalk_cur = att0[row_idx] & 0x1;
            int crosstalk_cur2 = att1[row_idx] & 0x1;
            int crosstalk_up = att0[row_idx_up] & 0x1;
            int crosstalk_up2 = att1[row_idx_up] & 0x1;

            int height_cur = high0[row_idx];
            int height_cur2 = high1[row_idx];

            bool save_flag_col = false;

            // ------ 天花板杂散标记 ------
            if (row_idx >= 139 && row_idx <= 143) { // 140-144行对应索引139-143
                if (crosstalk_cur &&
                    ((std::abs(dist_cur - dist_up) < 100 && crosstalk_up) ||
                        (std::abs(dist_cur - dist_up2) < 100 && crosstalk_up2))) {
                    ceil_stray_dist[2] = dist_cur;
                }
                else if (crosstalk_cur2 &&
                        ((std::abs(dist_cur2 - dist_up) < 100 && crosstalk_up) ||
                            (std::abs(dist_cur2 - dist_up2) < 100 && crosstalk_up2))) {
                    ceil_stray_dist[2] = dist_cur2;
                }
            }

            // ------ 列向距离处理 ------
            int normal_save_flag = 0;
            if(dist_cur > 0){
                if(normal_dist == 0){
                    normal_dist = dist_cur;
                    normal_row_st = row_idx;
                    normal_row_ed = row_idx;
                    normal_height_st = height_cur;
                    normal_height_ed = height_cur;
                    normal_bright = ref_cur > 50;
                } else if((abs(dist_cur - normal_dist) < 160) && (row_idx - normal_row_ed < 5)){
                    normal_dist = dist_cur;
                    normal_row_ed = row_idx;
                    normal_height_ed = height_cur;
                    if (ref_cur > 50) {
                        normal_bright = 1;
                    }
                }else {
                    normal_save_flag = 1;
                }
            }

            if(normal_save_flag == 1 || row_idx == VIEW_H - 1){
                if(normal_bright == 0 && normal_row_st != -1){
                    for (int i = normal_row_st; i <= normal_row_ed; ++i) {
                        blue_line[i] = 1;
                    }
                }
                normal_dist = dist_cur;
                normal_row_st = row_idx;
                normal_row_ed = row_idx;
                normal_height_st = height_cur;
                normal_height_ed = height_cur;
                normal_bright = 0;
            }
            // ------ 聚类处理 ------
            int dist_th_classify_cur = (dist_cur < stray_Param.dist_node1) ?
                                        stray_Param.dist_th_classify[0] : stray_Param.dist_th_classify[1];

            if (dist_cur > 1600) {
                if (0 == class_cnt) {
                    class_row_st = row_idx;
                    class_row_ed = row_idx;
                    class_cnt = 1;
                    class_dist = dist_cur;
                    if (crosstalk_cur > 0) {
                        class_stray_cnt++;
                        class_stray_row = row_idx;
                    }
                    class_height_st = height_cur;
                    class_height_ed = height_cur;
                }
                else if (std::abs(class_dist - dist_cur) < dist_th_classify_cur &&
                            row_idx - class_row_ed < 3) {
                    // 普通块处理
                    if (0 == class_stray_cnt) {
                        if (crosstalk_cur > 0 && row_idx - class_row_st > 3) {
                            save_flag_col = true;
                            class_stray_type = 1;
                        } else {
                            class_row_ed = row_idx;
                            class_cnt++;
                            class_dist = dist_cur;
                            class_height_ed = height_cur;
                            if (crosstalk_cur > 0) {
                                class_stray_cnt++;
                                class_stray_row = row_idx;
                            }
                        }
                    }
                    // 杂散块处理
                    else if (class_stray_cnt > 0) {
                        class_row_ed = row_idx;
                        class_cnt++;
                        class_dist = dist_cur;
                        class_height_ed = height_cur;
                        if (crosstalk_cur > 0) {
                            class_stray_cnt++;
                            class_stray_row = row_idx;
                        }
                        if (row_idx - class_stray_row > 5) {
                            save_flag_col = true;
                        }
                    }
                }
                else {
                    save_flag_col = true;
                }
            }

            // 保存聚类结果
            if (save_flag_col || row_idx == (VIEW_H - 1)) {
                bool class_type = (class_stray_cnt > 0);
                bool wr_stray_class_flag = false;
                int chain_last_row_st = 0;
                int chain_last_row_ed = 0;
                // 杂散链统计
                if(class_type && (class_stray_cnt * 4 >= class_row_ed - class_row_st)
                    && (!((abs(((class_row_st + class_row_ed + 2)/2) - 120) < 15) && (class_stray_cnt < 30)
                    && (class_cnt_last > 20) && (class_stray_cnt_last < 5) && (abs(class_dist - class_dist_last) < 400)))){
                    if (0 == stray_chain_cnt_c && class_stray_cnt > 3) {
                        stray_chain_dist_c = class_dist;
                        stray_chain_points_c = class_cnt;
                        stray_chain_cnt_c = class_stray_cnt;
                        stray_chain_row_st_c = class_row_st;
                        stray_chain_row_ed_c = class_row_ed;
                        stray_chain_ht_st_c = class_height_st;
                        stray_chain_ht_ed_c = class_height_ed;
                    }
                    // 合并条件
                    else if(std::abs(class_dist - stray_chain_dist_c) < 100 &&
                            (class_row_st - stray_chain_row_ed_c < 20 || stray_chain_cnt_c > 5 && class_stray_cnt > 5)) {
                        stray_chain_dist_c = class_dist;
                        stray_chain_points_c += class_cnt;
                        stray_chain_cnt_c += class_stray_cnt;
                        stray_chain_row_ed_c = class_row_ed;
                        stray_chain_ht_ed_c = class_height_ed;
                    }
                    // 替换条件
                    else if(stray_chain_cnt_c > 0 && class_stray_cnt > stray_chain_cnt_c) {
                        wr_stray_class_flag = (stray_chain_cnt_c >= 5) &&
                            (stray_chain_cnt_c * 2 >= stray_chain_points_c) &&
                            (abs(stray_chain_dist_c - RainWall_in.dist) < 960) &&
                            (RainWall_in.cnt > 150);
                        chain_last_row_st = stray_chain_row_st_c;
                        chain_last_row_ed = stray_chain_row_ed_c;
                        stray_chain_dist_c = class_dist;
                        stray_chain_points_c = class_cnt;
                        stray_chain_cnt_c = class_stray_cnt;
                        stray_chain_row_st_c = class_row_st;
                        stray_chain_row_ed_c = class_row_ed;
                        stray_chain_ht_st_c = class_height_st;
                        stray_chain_ht_ed_c = class_height_ed;
                    }
                }

                // 距离聚类标记
                if (class_cnt >= 1 && ((!class_type && class_cnt <= 10) ||
                    (class_type && class_cnt <= 5 &&
                        (class_cnt - class_stray_cnt) <= 3 &&
                        class_stray_cnt * 2 >= class_cnt))) {
                    for (int i = class_row_st; i <= class_row_ed; ++i) {
                        class_line[i] = 1;
                    }
                }
                else if(class_type && class_stray_cnt > 3
                        && (class_stray_cnt*4 >= class_cnt*3 || class_stray_cnt * 2 >= class_cnt && class_stray_cnt > 5)
                        && !class_stray_type)

                {
                    // class_line(class_row_st:class_row_ed) = 1;
                    //密集的杂散块
                    for (int i = class_row_st; i <= class_row_ed; ++i) {
                        class_line[i] = 1;
                    }
                }
                else if (class_cnt <= 10 && class_type)
                {
                    for (int i = class_row_st; i <= class_row_ed; ++i) {
                        class_line[i] = 2;
                    }
                }

                if (row_idx == (VIEW_H - 1) && stray_chain_cnt_c >= 5 &&
                    stray_chain_cnt_c * 2 >= stray_chain_points_c &&
                    std::abs(stray_chain_dist_c - RainWall_in.dist) < 960 &&
                    RainWall_in.cnt > 150) {
                    chain_last_row_st = stray_chain_row_st_c;
                    chain_last_row_ed = stray_chain_row_ed_c;
                    wr_stray_class_flag = true;

                }
                if (wr_stray_class_flag)
                {
                    for (int i = chain_last_row_st; i <= chain_last_row_ed; ++i) {
                        class_line[i] = 1;
                    }
                }

                if (save_flag_col && row_idx == (VIEW_H - 1)) {
                    class_line[VIEW_H - 1] = 1;
                }

                // 重置聚类变量
                if (dist_cur > 1600) {
                    class_cnt_last = class_cnt;
                    class_dist_last = class_dist;
                    class_stray_cnt_last = class_stray_cnt;
                    class_row_st = row_idx;
                    class_row_ed = row_idx;
                    class_cnt = 1;
                    class_dist = dist_cur;
                    class_stray_cnt = (crosstalk_cur > 0) ? 1 : 0;
                    class_stray_row = (crosstalk_cur > 0) ? row_idx : 0;
                    class_stray_type = 0;
                    class_height_st = height_cur;
                    class_height_ed = height_cur;
                }
            }

            // 行结束处理
            if (row_idx == VIEW_H - 1) {
                bool save_flag_row = false;

                if (stray_chain_cnt_c > 0) {
                    if (stray_chain_ht_ed_c - stray_chain_ht_st_c > 160 && stray_chain_row_ed_c > 71) {

                        if (stray_conn_cols == 0 &&
                            stray_chain_cnt_c * 2 > (stray_chain_row_ed_c - stray_chain_row_st_c)) {
                            stray_conn_dist = stray_chain_dist_c;
                            stray_conn_col_st = col_idx;
                            stray_conn_col_ed = col_idx;
                            stray_conn_cols = 1;
                            stray_conn_cnt = stray_chain_cnt_c;
                        }
                        else if (std::abs(stray_conn_dist - stray_chain_dist_c) < 200 &&
                                    col_idx - stray_conn_col_ed < 20) {
                            stray_conn_cols++;
                            stray_conn_dist = stray_chain_dist_c;
                            stray_conn_col_ed = col_idx;
                            stray_conn_cnt += stray_chain_cnt_c;
                        }
                        else {
                            save_flag_row = true;
                        }
                    }
                }

                if (save_flag_row || col_idx == VIEW_W - 1) {
                    if (stray_conn_cnt > 20 && stray_conn_cnt > rain_wall_cnt) {
                        rain_wall_cnt = stray_conn_cnt;
                        rain_wall_dist = stray_conn_dist;
                    }

                    if (col_idx == VIEW_W - 1) {
                        if (rain_wall_cnt > 0) {
                            RainWall_out.dist = rain_wall_dist;
                            RainWall_out.cnt = rain_wall_cnt;
                            RainWall_out.hold_cnt = 0;
                        } else {
                            memcpy(&RainWall_out, &RainWall_in, sizeof(RainWall_out));
                            RainWall_out.hold_cnt++;
                            if (RainWall_out.hold_cnt >= stray_Param.rwall_hold_th) {
                                RainWall_out = {0, 0, 0};
                            }
                        }
                    }

                    if (stray_chain_cnt_c > 0) {
                        stray_conn_dist = stray_chain_dist_c;
                        stray_conn_col_st = col_idx;
                        stray_conn_col_ed = col_idx;
                        stray_conn_cnt = stray_chain_cnt_c;
                        stray_conn_cols = 1;
                    }
                }
            }
        }

        // 更新杂散链信息
        stray_chain_cnt[2] = stray_chain_cnt_c;
        stray_chain_dist[2] = stray_chain_dist_c;
        stray_chain_row_st[2] = stray_chain_row_st_c;
        stray_chain_row_ed[2] = stray_chain_row_ed_c;
        stray_chain_height[2] = stray_chain_ht_ed_c - stray_chain_ht_st_c; //当列最长的杂散链高度

        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            pva_line[row_idx] = (blue_line[row_idx] << 8) + class_line[row_idx];
        }
        memcpy(&stray_pva_buff.class_line_h[col_idx * VIEW_H], pva_line, VIEW_H * sizeof(uint16_t));

        stray_var_cnt += 1;
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = ceil_stray_dist[0];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = ceil_stray_dist[1];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = ceil_stray_dist[2];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_dist[0];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_dist[1];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_dist_c;
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_row_st[0];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_row_ed[0];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_row_st[1];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_row_ed[1];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_row_st_c;
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_row_ed_c;
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_cnt[0];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_cnt[1];
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_cnt_c;
        stray_pva_buff.stray_var_h[stray_var_cnt ++] = stray_chain_height[2];
    }
}

/**
 * @brief Calculate gnd label for spray remove algo on cpu.
 *
 * @param[in] pstFrameBuffer frame buffer
 * @param[in] col_idx column index
 *                  Range: 0 - 759. Accuracy: 1.
 */

void AlgoFunction::sprayRemoveCpu(int32_t col_idx, tstFrameBuffer* pstFrameBuffer)
{
    uint16_t* dist_cur0 = pstFrameBuffer->dist0[col_idx];
    uint16_t* dist_cur1 = pstFrameBuffer->dist1[col_idx];
    for (int row_idx = 0; row_idx < VIEW_H; row_idx++){
        // 第一回波数据
        int cur_dist_1 = dist_cur0[row_idx];
        int cur_dist_2 = dist_cur1[row_idx];

        /******************接地保护*******************/
        int zone_idx = row_idx / 12;
        //12个通道一个采样
        if (row_idx % 12 == 0) {
            spray_Param.dist_cap_zone[zone_idx][0] = cur_dist_1;
            spray_Param.dist_cap_zone[zone_idx][1] = cur_dist_2;

            //接地判别(第一回波)
            if (zone_idx > 0 && spray_Param.ground_cap_zone[zone_idx - 1][0] &&
                ((std::abs(spray_Param.dist_cap_zone[zone_idx][0] - spray_Param.dist_cap_zone[zone_idx - 1][0]) <= spray_Param.dist_diff_thr_fix &&
                spray_Param.dist_cap_zone[zone_idx - 1][0]) ||
                (std::abs(spray_Param.dist_cap_zone[zone_idx][0] - spray_Param.dist_cap_zone[zone_idx - 1][1]) <= spray_Param.dist_diff_thr_fix &&
                spray_Param.dist_cap_zone[zone_idx - 1][1]))) {
                spray_Param.ground_cap_zone[zone_idx][0] = 1;
            }
            else {
                spray_Param.ground_cap_zone[zone_idx][0] = 0;
            }

            //接地判别(第二回波)
            if (zone_idx > 0 && spray_Param.ground_cap_zone[zone_idx - 1][1] &&
                ((std::abs(spray_Param.dist_cap_zone[zone_idx][1] - spray_Param.dist_cap_zone[zone_idx - 1][0]) <= spray_Param.dist_diff_thr_fix && spray_Param.dist_cap_zone[zone_idx - 1][0]) ||
                (std::abs(spray_Param.dist_cap_zone[zone_idx][1] - spray_Param.dist_cap_zone[zone_idx - 1][1]) <= spray_Param.dist_diff_thr_fix && spray_Param.dist_cap_zone[zone_idx - 1][1]))){
                spray_Param.ground_cap_zone[zone_idx][1] = 1;
            }
            else {
                spray_Param.ground_cap_zone[zone_idx][1] = 0;
            }
        }

        //悬空判断的link2和link3预处理
        int col_id1 = col_idx / 20;
        int col_id2 = col_idx % 20;
        //第一回波
        int glk0 = 0;
        if ((zone_idx > 0) && ((std::abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx-1][0]) > spray_Param.dist_diff_thr_fix
        && std::abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx-1][1]) > spray_Param.dist_diff_thr_fix) || !spray_Param.ground_cap_zone[zone_idx][0])){
            glk0 = 1;
        }
        if ((zone_idx > 0) && ((std::abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx-1][0]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[zone_idx][0]) ||
                (std::abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx-1][1]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[zone_idx][1]))){
            glk0 = 0;
        }

        gnd_link_cpu0[col_idx][row_idx] = glk0;

        //第二回波
        int glk1 = 0;
        if ((zone_idx > 0) &&((std::abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx-1][0]) > spray_Param.dist_diff_thr_fix
            && std::abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx-1][1]) > spray_Param.dist_diff_thr_fix))){
            glk1 = 1;
        }
        if ((zone_idx > 0) && ((std::abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx-1][0]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[zone_idx][0]) ||
            (std::abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx-1][1]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[zone_idx][1]))){
            glk1 = 0;
        }
        gnd_link_cpu1[col_idx][row_idx] = glk1;
    }
}

/**
 * @brief Execuate spray-remove and rain-enhance algo using vpu.
 *
 * @param[in] pstFrameBuffer frame buffer
 */

void AlgoFunction::sprayRemoveExec(tstFrameBuffer* pstFrameBuffer)
{
    thread_local int cnt{0};
    cnt = (cnt + 1) % 10;

    if (algo_Param.SprayRemoveOn) {
        memcpy(DistIn0_h, pstFrameBuffer->dist0, VIEW_H * VIEW_W * sizeof(uint16_t));
        memcpy(DistIn1_h, pstFrameBuffer->dist1, VIEW_H * VIEW_W * sizeof(uint16_t));
        memcpy(RefIn0_h, pstFrameBuffer->ref0, VIEW_H * VIEW_W * sizeof(uint16_t));
        memcpy(RefIn1_h, pstFrameBuffer->ref1, VIEW_H * VIEW_W * sizeof(uint16_t));
        memcpy(AttIn0_h, pstFrameBuffer->att0, VIEW_H * VIEW_W * sizeof(uint16_t));
        memcpy(AttIn1_h, pstFrameBuffer->att1, VIEW_H * VIEW_W * sizeof(uint16_t));

        for (int i = 0; i < 38; i ++) {
            int offset2 = (i * 80) * 192;
            int offset3 = (i * 80 + 20) * 192;

            int offset0 = (i * 80 + 40) * 192;
            int offset1 = (i * 80 + 60) * 192;
            /*地面标签，给spray-remove algo使用*/
            memcpy((uint8_t *)&Glink_h[offset2], (uint8_t *)gnd_link_cpu0[i * 20], VIEW_H * 20 * sizeof(uint16_t));
            memcpy((uint8_t *)&Glink_h[offset3], (uint8_t *)gnd_link_cpu1[i * 20], VIEW_H * 20 * sizeof(uint16_t));

            memcpy(&Glink_h[offset0], pstFrameBuffer->gnd_mark0[i * 20], VIEW_H * 20 * sizeof(uint16_t));
            memcpy(&Glink_h[offset1], pstFrameBuffer->gnd_mark1[i * 20], VIEW_H * 20 * sizeof(uint16_t));
        }

        //提交spray remove pva任务
        submitPvaTaskWithRetry(std::bind(sprayRemovePva, 
                                std::placeholders::_1, std::placeholders::_2, 
                                std::placeholders::_3, std::placeholders::_4), 
                                "algo5.1");
        
        //提交rain enhance pva任务
        submitPvaTaskWithRetry(std::bind(rainEnhancePva, 
                        std::placeholders::_1, std::placeholders::_2, 
                        std::placeholders::_3, std::placeholders::_4), 
                        "algo5.2");

        memcpy(stray_mask_out_frm0[0], &stray_pva_buff.stray_mask0_h[0], VIEW_W * VIEW_H * sizeof(uint16_t));
        memcpy(stray_mask_out_frm1[0], &stray_pva_buff.stray_mask1_h[0], VIEW_W * VIEW_H * sizeof(uint16_t));
        memcpy(spray_mark_out_frm0[0], &FinalOut0_h[0], VIEW_W * VIEW_H * sizeof(uint16_t));
        memcpy(spray_mark_out_frm1[0], &FinalOut1_h[0], VIEW_W * VIEW_H * sizeof(uint16_t));
    }
    else {
        for (int cc = 0; cc < VIEW_W; cc ++) {
            for (int rr = 0; rr < VIEW_W; rr ++) {
                uint16_t rain_mark0 = pstFrameBuffer->att0[cc][rr];
                rain_mark0 = rain_mark0 >> 7;
                spray_mark_out_frm0[cc][rr] = rain_mark0;

                uint16_t rain_mark1 = pstFrameBuffer->att1[cc][rr];
                rain_mark1 = rain_mark1 >> 7;
                spray_mark_out_frm1[cc][rr] = rain_mark1;
            }
        }
    }
}

/**
 * \brief  High calculation algorithm main process in pva
 *
 * \param[in] pstFrameBuffer: frame buffer
 *              Range: N/A. Accuracy: N/A.
 */
void AlgoFunction::highcalcExec(tstFrameBuffer* pstFrameBuffer)
{
    memcpy((uint8_t *)h_dist_in0_h, (uint8_t *)pstFrameBuffer->dist0[0], VIEW_H * VIEW_W * sizeof(uint16_t));
    memcpy((uint8_t *)h_dist_in1_h, (uint8_t *)pstFrameBuffer->dist1[0], VIEW_H * VIEW_W * sizeof(uint16_t));
    memcpy((uint8_t *)h_high_in0_h, (uint8_t *)pstFrameBuffer->high0[0], VIEW_H * VIEW_W * sizeof(uint16_t));
    memcpy((uint8_t *)h_high_in1_h, (uint8_t *)pstFrameBuffer->high1[0], VIEW_H * VIEW_W * sizeof(uint16_t));
    memcpy((uint8_t *)h_proj_dist0_h, (uint8_t *)pstFrameBuffer->proj_dist0[0], VIEW_H * VIEW_W * sizeof(uint16_t));
    memcpy((uint8_t *)h_proj_dist1_h, (uint8_t *)pstFrameBuffer->proj_dist1[0], VIEW_H * VIEW_W * sizeof(uint16_t));

    submitPvaTaskWithRetry(std::bind(highcalcPva, 
                                    std::placeholders::_1, std::placeholders::_2, 
                                    std::placeholders::_3, std::placeholders::_4), 
                                    "algo3");

    memcpy(denoise_mask_out_frm[0], &denoise_mask_buffer_h[0], VIEW_W * VIEW_H * sizeof(uint16_t));
    memcpy(trail_mask_out_frm[0], &ValidOut_h[0], sizeof(uint16_t) * VIEW_W * VIEW_H);
    memcpy((uint8_t *)pstFrameBuffer->gnd_mark0[0], (uint8_t *)&h_gnd_out0_h[0], VIEW_W * VIEW_H * sizeof(uint16_t));
    memcpy((uint8_t *)pstFrameBuffer->gnd_mark1[0], (uint8_t *)&h_gnd_out1_h[0], VIEW_W * VIEW_H * sizeof(uint16_t));
}

/**
 * \brief  Upsample algorithm main process in pva
 *
 * \param[in] pstFrameBuffer: frame buffer
 *              Range: N/A. Accuracy: N/A.
 */
void AlgoFunction::upsampleExec(tstFrameBuffer* pstFrameBuffer)
{
    submitPvaTaskWithRetry(std::bind(upsampleProcPva, 
                                std::placeholders::_1, std::placeholders::_2, 
                                std::placeholders::_3, std::placeholders::_4), 
                                "algo6");
}


/**
 * @brief point cloud algorithm main process
 *
 * @param col_idx column index of the buffer
 *                  Range: 0 - 759. Accuracy: 1.
 * @param pstFrameBuffer frame buffer of whole point cloud
 *                  Range: N/A. Accuracy: N/A.
 * @param task_id thread tash id of algorithm
 *                  Range: 0 - 2. Accuracy: 1.
 * @return int process column of point cloud
 *                  Range: 0 - 759. Accuracy: 1.
 */
void AlgoFunction::pcAlgoMainFunc(int col_idx, tstFrameBuffer* pstFrameBuffer)
{
    if(col_idx < 0 || col_idx >= VIEW_W) {
        return;
    }

    int surface_id = pstFrameBuffer->surface_id.load();
    int real_col;
    int org_high0;
    int org_high1;
    int fit_ground_high0;
    int fit_ground_high1;
    const float a = fit_Params.a;
    const float b = fit_Params.b;
    const float c = fit_Params.c;
    uint16_t* dist0 = &pstFrameBuffer->dist0[col_idx][0];
    uint16_t* dist1 = &pstFrameBuffer->dist1[col_idx][0];
    int16_t* high0 = &pstFrameBuffer->high0[col_idx][0];
    int16_t* high1 = &pstFrameBuffer->high1[col_idx][0];
    uint16_t* proj_dist0 = &pstFrameBuffer->proj_dist0[col_idx][0];
    uint16_t* proj_dist1 = &pstFrameBuffer->proj_dist1[col_idx][0];

    if(0 == surface_id) {
        real_col = (col_idx << 1);
    } else {
        real_col = UP_VIEW_W - (col_idx << 1) - 1;
    }

    for (int i = 0; i < VIEW_H; ++i) {
        int dist_val0 = dist0[i];
        int dist_val1 = dist1[i];
        int Ix_val = Ix_in[i][real_col];
        int Iy_val = Iy_in[i][real_col];
        int Iz_val = Iz_in[i][real_col];

        org_high0 = (dist_val0 * Iz_val) >> 15;
        org_high1 = (dist_val1 * Iz_val) >> 15;
        fit_ground_high0 = std::floor((float)dist_val0 *
            (a * Ix_val / 32768.0 + b * Iy_val / 32768.0) + c);
        fit_ground_high1 = std::floor((float)dist_val1 *
            (a * Ix_val / 32768.0 + b * Iy_val / 32768.0) + c);
        high0[i] = org_high0 - fit_ground_high0;
        high1[i] = org_high1 - fit_ground_high1;
        proj_dist0[i] = (dist_val0 * cosd_pitch_lut[i]) >> 15;
        proj_dist1[i] = (dist_val1 * cosd_pitch_lut[i]) >> 15;
    }

    //杂散删除杂散标签预处理
    if (algo_Param.StrayRemoveOn) {
        //杂散删除 初始化输出
        if(0 == col_idx){
            memcpy(&RainWall_in, &RainWall_out, sizeof(RainWall_out));
            RainWall_out = {0, 0, 0};
        }

        //在CPU端计算杂散相关的标签
        strayDelete(col_idx, pstFrameBuffer);
    }

    if (algo_Param.SprayRemoveOn) {
        sprayRemoveCpu(col_idx, pstFrameBuffer);
    }

    // 地面拟合
    if((col_idx % gnd_step) == 0){
        uint16_t dist_line[VIEW_H];
        uint16_t col_pos = col_idx / gnd_step;
        memcpy(dist_line, pstFrameBuffer->dist0[col_idx], sizeof(uint16_t) * VIEW_H);

        for(int row = 0; row < VIEW_H; ++row){
            int idx = row * GND_VIEW_W + col_pos;
            dist_wave0_buffer6[idx] = dist_line[row];
            int dist = dist_line[row];
            X_buffer6[idx] = dist * Ix_in[row][real_col] / 32768.0f;
            Y_buffer6[idx] = dist * Iy_in[row][real_col] / 32768.0f;
            Z_buffer6[idx] = dist * Iz_in[row][real_col] / 32768.0f;
        }
        if(col_idx == (VIEW_W - gnd_step)){
            selectGroundFitFunc(best_model_fit);
        }
    }
}

}   // namespace lidar
}   // namespace robosense