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

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "algo.h"
#include "common/fault_manager.h"
/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense
{
namespace lidar
{
/**
 * \brief  Trail remove main process
 * \param[in] trail_col : Current column index
 *                  Range: 0 - 760. Accuracy: 1.
 * \param[in] trail_col_buffer : Current column buffer index
 *                  Range: 0 - 4. Accuracy: 1.
 * \param[in] trail_col_neib_buf : Neighbor column buffer index
 *                  Range:0-2^32-1. Accuracy:1.
 * \param[out] trail_mask_out : Trail mask output
 *                  Range:0-2^32-1. Accuracy:1.
 */
void AlgoFunction::trailRemove(int trail_col,
                               int trail_col_buffer,
                               int* trail_col_neib_buf,
                               int* trail_mask_out)
{
    if (trail_col < 0 || trail_col >= algo_Param.ViewField_Wide
        || !algo_Param.TrailRemoveOn) {
        return;
    }
    const int HL = trail_Param.HL;
    //constexpr int NEIGHBOR_COUNT = 5;
    constexpr int LONGITUDINAL_RANGE = 4;
    const int bypass_distance = trail_Param.BypassDis;
    const int near_threshold = trail_Param.near_cnt_th_h;

    for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
        int near_dist_th = 0;
        int near_cnt_h = 0;
        const int dist_trail = dist_wave0_buffer2[trail_col_buffer][row_idx];
        if (dist_trail <= 0 || dist_trail >= bypass_distance) continue;
        int trail_refer_mask = 0;

        near_dist_th = clamp(((dist_trail * trail_Param.dist_th_ratio) >> 3), 2, 20);
        int dist_tmp[HL];
        for (int i = 0; i < HL; ++i) {
            dist_tmp[i] = dist_wave0_buffer2[trail_col_neib_buf[i]][row_idx];
        }
        for (int i = 0; i < HL- 1; ++i) {
            near_cnt_h += (std::abs(dist_tmp[i + 1] - dist_tmp[i]) < near_dist_th);
        }
        trail_refer_mask = (near_cnt_h >= near_threshold);
        trail_Param.near_cnt_h = near_cnt_h;
        trail_Param.near_dist_th = near_dist_th;

        if (trail_refer_mask != 0) continue;
        int Hortrail_judge = 0;
        int wall_judge = 0;
        HorTrailRemove(dist_tmp, dist_trail, &Hortrail_judge, &wall_judge);
        trail_Param.wall_judge = wall_judge;
        if(Hortrail_judge == 1)
        {
            int dist_longit[9] = {0};
            const int row_range_up = std::min(row_idx, LONGITUDINAL_RANGE);
            const int row_range_down = std::min(VIEW_H - 1 - row_idx,
                                                LONGITUDINAL_RANGE);

            const int start_idx = 4 - row_range_up;
            for (int i = 0; i <= row_range_up + row_range_down; ++i) {
                const int buf_idx = start_idx + i;
                dist_longit[buf_idx] = dist_wave0_buffer2
                                        [trail_col_buffer][row_idx - row_range_up + i];
            }
            int Vertrail_judge = VerTrailRemove(dist_trail, dist_longit);
            if(Vertrail_judge == 1)
            {
                trail_mask_out[row_idx] = 1;
            }
        }
    }
}

/**
 * \brief  Limit value to minimum and maximum
 * \param[in] val : Value to be limited
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] min_val : Minimum value
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] max_val : Maximum value
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return Limited value
*/
int AlgoFunction::clamp(int val, int min_val, int max_val) {
    return std::max(min_val, std::min(val, max_val));
}

/**
 * \brief  Horizontal trail remove judge function
 *
 * \param[in] dist_tmp : Distance input array
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] dist_trail : Reflectance input array
 * \param[out] Hortrail_judge : Horizontal trail success flag
 *                Range: 0-1. Accuracy: 1.
 * \param[out] wall_judge : Horizontal trail success flag
 *                Range: 0-1. Accuracy: 1.
 */
void AlgoFunction::HorTrailRemove(int* dist_tmp,
                             int dist_trail,
                             int* Hortrail_judge,
                             int* wall_judge)
{
    int weight[5] = {1,2,0,2,1};
    *Hortrail_judge = 0;
    *wall_judge = 0;

    const int HL = trail_Param.HL;
    const int half_HL = trail_Param.half_HL;

    // 差异数组初始化
    int dif_distC_abs[HL] = {0};
    int dif_dist[HL-1] = {0};
    int dif_dist_abs[HL-1] = {0};
    int dif2_dist_abs[HL-2] = {0};
    // 计算自适应阈值
    const int AdjDisThreD = std::max(1, (dist_trail * trail_Param.DisThreRatio) >> 3);
    int zero_cnt = 0;
    int dif_dist_abs_cnt = 0;

    // 参数提取
    const int& D_H = trail_Param.D_H;

    //初始化参数
    int dist_tmpC;
    int dif_distC_tmp;
    int dif_dist_tmp;

    // 水平方向处理 - 第一遍循环
    for (int i = 0; i < HL; i++)
    {
        // 处理无效点
        dist_tmpC = dist_tmp[i];
        if (dist_tmpC == 0) {
        weight[i] = 0;
        dif_distC_tmp = 65535;
        zero_cnt++;
        }
        else {
            dif_distC_tmp = dist_tmpC - dist_trail;
        }

        // 计算绝对值
        dif_distC_abs[i] = std::abs(dif_distC_tmp);

        // 相邻点差异计算
        if (i < HL - 1) {
            dif_dist_tmp = dist_tmp[i + 1] - dist_tmp[i];
            dif_dist[i] = dif_dist_tmp;
            dif_dist_abs[i] = std::abs(dif_dist_tmp);
            // 大差异计数
            if (dif_dist_abs[i] > AdjDisThreD) {
                dif_dist_abs_cnt++;
            }

            // 二阶差分计算
            if (i > 0) {
                dif2_dist_abs[i - 1] = std::abs(dif_dist[i] - dif_dist[i - 1]);
            }
        }
    }

    // 主判断条件
    bool con0 = zero_cnt <= half_HL + 1;

    // 条件1计算
    bool con1_1 = (dif_distC_abs[half_HL - 1] > AdjDisThreD) &&
    (dif_distC_abs[half_HL - 1] < D_H);
    bool con1_2 = (dif_distC_abs[half_HL + 1] > AdjDisThreD) &&
    (dif_distC_abs[half_HL + 1] < D_H);
    bool con1_3 = dif_dist_abs_cnt > 3;
    bool con1 = (con1_1 && con1_2) || con1_3;

    // 条件2计算
    int weighted_sum = 0;
    for (int i = 0; i < HL; i++) {
        weighted_sum += weight[i] * dif_distC_abs[i];
    }
    int distdiff_mean = std::round(weighted_sum / 4.0);
    bool con2 = (distdiff_mean > AdjDisThreD) && (distdiff_mean < D_H);

    if (con0 && (con1 || con2)) {
        *Hortrail_judge = 1;
    }
    int sum_dif2_dist_abs = 0;
    for (int i = 0; i < HL; i++)
    {
        sum_dif2_dist_abs += dif2_dist_abs[i];
    }
    int wall_mask1 = sum_dif2_dist_abs > trail_Param.SlopDifThre;
    int wall_mask2 = dif2_dist_abs[half_HL - 2] > trail_Param.SlopDifThre;
    if(wall_mask1 && wall_mask2)
    {
      *wall_judge = 1;
    }
}

/**
 * \brief  verital trail remove judge function
 *
 * \param[in] dist_trail : Reflectance input array
 * \param[out] Hortrail_judge : Horizontal trail success flag
 *                Range: 0-1. Accuracy: 1.
 * \param[out] wall_judge : Horizontal trail success flag
 *                Range: 0-1. Accuracy: 1.
 */
int AlgoFunction::VerTrailRemove(
                             int dist_trail,
                             int* dist_longit)
{
    int Vertrail_judge = 0;
    const int HL = trail_Param.HL;
    const int half_HL = trail_Param.half_HL;
    const int near_cnt_h = trail_Param.near_cnt_h;

    // 差异数组初始化
    int dif_distC_ver_abs[9] = {0};
    int near_cnt_v = 0;
    int ver_cnt = 0;
    for(int i = 0;i < 9; i++)
    {
        int dist_longit_tmp = dist_longit[i];
        if(dist_longit_tmp == 0) {
            dif_distC_ver_abs[i] = 65535;
        }
        else {
            dif_distC_ver_abs[i] = abs(dist_longit_tmp - dist_trail);
        }
        if(dif_distC_ver_abs[i] < trail_Param.near_dist_th)
        {
            near_cnt_v++;
        }
        if(dif_distC_ver_abs[i] < trail_Param.SlopDifThre * 2)
        {
          ver_cnt++;
        }

    }
    //距离判断条件3 保护细杆
    bool con3_1 = near_cnt_h < trail_Param.near_cnt_th_h;
    bool con3_2 = near_cnt_v < trail_Param.near_cnt_th_v;
    bool con3 = con3_1 && con3_2;
    // 距离判断条件4 保护斜墙
    bool con4_3 = ver_cnt < 3;
    bool con4 = trail_Param.wall_judge || con4_3;
    if(con3 && con4)
    {
        Vertrail_judge = 1;
    }
    return Vertrail_judge;
}

/**
 * \brief  Rebuild distance and reflectance function
 *
 * \param[in] dist_tmp1: distance neighborhood of wave 0
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] dist_tmp2: distance neighborhood of wave 1
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] mask1: mask neighborhood of wave 0
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] mask2: mask neighborhood of wave 1
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] ad_mask1: additional mask neighborhood of wave 0
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] ad_mask2: additional mask neighborhood of wave 1
 *                Range: 0 - 2^32-1. Accuracy: 1.
 */
void AlgoFunction::rebuildFunc(int* dist_tmp1,
                               int* dist_tmp2,
                               int* mask1,
                               int* mask2,
                               int* ad_mask1,
                               int* ad_mask2,
                               int* PLRawFillMask)
{
    memset(mask1, 0, 3 * sizeof(int));
    memset(mask2, 0, 3 * sizeof(int));
    memset(ad_mask1, 0, 3 * sizeof(int));
    memset(ad_mask2, 0, 3 * sizeof(int));
    *PLRawFillMask = 0;
    // 参数解包到局部变量（减少结构体访问开销）
    const int diff_th = upsample_Param.diff_th;
    const int diff_ratio_max = upsample_Param.diff_ratio_max;
    const int diff2_th = upsample_Param.diff2_th;
    const float ratio_scale = diff_ratio_max / 4096.0;

    // 替代 Eigen::Vector3i dist_c1 = dist_tmp1.segment(1, 3)
    int dist_c1[3] = { dist_tmp1[1], dist_tmp1[2], dist_tmp1[3] };  // 注意原始数据索引偏移
    int dist_c2[3] = { dist_tmp2[1], dist_tmp2[2], dist_tmp2[3] };

    // 计算距离差值（展开所有计算）
    if (std::abs(dist_c1[1] - dist_c2[1]) < ((dist_c1[1]*diff_ratio_max) >> 12) && dist_c1[1] > 0)
    {
        *PLRawFillMask = 1;
        return;
    }

    const int dist_diff11 = dist_tmp1[2] - dist_tmp1[1];
    const int dist_diff12 = dist_tmp1[3] - dist_tmp1[2];
    const int dist_diff13 = dist_tmp1[3] - dist_tmp1[1];
    const int dist_diff21 = dist_tmp2[2] - dist_tmp2[1];
    const int dist_diff22 = dist_tmp2[3] - dist_tmp2[2];
    const int dist_diff23 = dist_tmp2[3] - dist_tmp2[1];

    // 绝对值计算（展开所有计算）
    const int abs_dist_diff[6] = {
        std::abs(dist_diff11), std::abs(dist_diff12), std::abs(dist_diff13),
        std::abs(dist_diff21), std::abs(dist_diff22), std::abs(dist_diff23)
    };

    // 交叉差异计算
    const int abs_cross_diff[3] = {
        std::abs(dist_tmp1[1] - dist_tmp2[1]),
        std::abs(dist_tmp1[2] - dist_tmp2[2]),
        std::abs(dist_tmp1[3] - dist_tmp2[3])
    };

    // 阈值计算（展开为数组操作）
    int diff_th_max1[3], diff_th_max2[3];
    for (int i = 0; i < 3; ++i) {
        diff_th_max1[i] = static_cast<int>(dist_c1[i] * ratio_scale);
        diff_th_max2[i] = static_cast<int>(dist_c2[i] * ratio_scale);
    }

    // 初始化掩码数组（使用基础类型替代Eigen向量）
    int mask1_hor[3] = { 0 }, mask2_hor[3] = { 0 };
    int mask1_tri[3] = { 0 }, mask2_tri[3] = { 0 };
    int ad_tri_mask1[3] = { 0 }, ad_tri_mask2[3] = { 0 };

    /* 水平一致性检测 (替代lambda函数) */
    auto checkConsistency = [&](const int* dist, const int* diffs,
        int* mask) {
            if (diffs[0] < diff_th && dist[1] > 0 && dist[2] > 0) {
                mask[0] = mask[1] = 1;
            }
            if (diffs[1] < diff_th && dist[2] > 0 && dist[3] > 0) {
                mask[1] = mask[2] = 1;
            }
            if (diffs[2] < diff_th && dist[1] > 0 && dist[3] > 0) {
                mask[0] = mask[2] = 1;
            }
    };

    // 调用检测函数（参数需要重组）
    const int diffs1[3] = { abs_dist_diff[0], abs_dist_diff[1], abs_dist_diff[2] };
    const int diffs2[3] = { abs_dist_diff[3], abs_dist_diff[4], abs_dist_diff[5] };
    checkConsistency(dist_tmp1, diffs1, mask1_hor);
    checkConsistency(dist_tmp2, diffs2, mask2_hor);

    // 处理左侧数据（优化为数组操作）
    int res1 = 0, consis_cnt1 = 0;
    for (int i = 0; i < 3; ++i) {
        res1 += mask1_hor[i] * dist_c1[i];
        consis_cnt1 += mask1_hor[i];
    }
    const int coef1 = (consis_cnt1 > 0) ? upsample_Param.coef_inv[consis_cnt1 - 1] : 0;
    const int consis_dist1 = (res1 * coef1) >> 12;

    // 处理右侧数据（相同模式优化）

    int res2 = 0, consis_cnt2 = 0;
    for (int i = 0; i < 3; ++i) {
        res2 += mask2_hor[i] * dist_c2[i];
        consis_cnt2 += mask2_hor[i];
    }
    const int coef2 = (consis_cnt2 > 0) ? upsample_Param.coef_inv[consis_cnt2 - 1] : 0;
    const int consis_dist2 = (res2 * coef2) >> 12;

    // 主条件判断优化
    const int diff_threshold = (consis_dist1 * diff_ratio_max) >> 12;
    const bool main_condition = (consis_cnt1 >= 2 && consis_cnt2 >= 2)
                                && std::abs(consis_dist1 - consis_dist2) < ((consis_dist1 * diff_ratio_max) >> 12)
                                && (mask1_hor[1] + mask2_hor[1] > 0);

    if (main_condition) {
        memcpy(mask1, mask1_hor, sizeof(mask1_hor));
        memcpy(mask2, mask2_hor, sizeof(mask2_hor));

        // 优化分支判断
        ad_mask1[1] = (mask1_hor[1] == 0) ? 2 : ad_mask1[1];
        ad_mask2[1] = (mask2_hor[1] == 0) ? 2 : ad_mask2[1];

        return;
    }

    // 初始化交叉掩码（展开为数组操作）
    int mask_cross[3] = {
        (abs_cross_diff[0] < diff_th),
        (abs_cross_diff[1] < diff_th),
        (abs_cross_diff[2] < diff_th)
    };

    // 应用距离有效性过滤（展开循环）
    for (int i = 0; i < 3; ++i) {
        mask_cross[i] &= (dist_c1[i] > 0);
    }
    // 构建掩码数组（展开所有计算）
    int mask_c1[3], mask_c2[3];
    mask_c1[0] = (abs_dist_diff[0] < diff_th);  // abs_dist_diff11
    mask_c1[1] = 0 < diff_th;
    mask_c1[2] = (abs_dist_diff[1] < diff_th);  // abs_dist_diff12

    mask_c2[0] = (abs_dist_diff[3] < diff_th);  // abs_dist_diff21
    mask_c2[1] = 0 < diff_th;
    mask_c2[2] = (abs_dist_diff[4] < diff_th);  // abs_dist_diff22

    // 边缘掩码计算（预计算差值）
    const int edge_diff1_0 = std::abs(dist_tmp1[0] - dist_tmp1[1]);
    const int edge_diff1_3 = std::abs(dist_tmp1[3] - dist_tmp1[4]);
    const int edge_diff2_0 = std::abs(dist_tmp2[0] - dist_tmp2[1]);
    const int edge_diff2_3 = std::abs(dist_tmp2[3] - dist_tmp2[4]);

    int mask_e1[3] = { edge_diff1_0 < diff_th, 65535 < diff_th, edge_diff1_3 < diff_th };
    int mask_e2[3] = { edge_diff2_0 < diff_th, 65535 < diff_th, edge_diff2_3 < diff_th };

    // 主条件判断逻辑（优化条件结构）
    const int mask_cross_sum = mask_cross[0] + mask_cross[1] + mask_cross[2];
    const int mask_c1_sum = mask_c1[0] + mask_c1[1] + mask_c1[2];
    const int mask_c2_sum = mask_c2[0] + mask_c2[1] + mask_c2[2];


    if ((3 == consis_cnt1 || (std::abs(dist_diff12 - dist_diff13) < diff2_th &&
        abs_dist_diff[1] < diff_th_max1[0])) && (mask_cross_sum == 1)) {

        int product_sum = 0;
        product_sum += mask_cross[0] * mask_e2[0];
        product_sum += mask_cross[1] * mask_e2[1];
        product_sum += mask_cross[2] * mask_e2[2];

        if (product_sum > 0) {
            mask1_tri[0] = 1; mask1_tri[1] = 1; mask1_tri[2] = 1;
            mask2_tri[0] = mask_cross[0]; mask2_tri[1] = mask_cross[1]; mask2_tri[2] = mask_cross[2];
        }
        else if (mask_cross[1] == 0) {
            ad_tri_mask2[0] = mask_cross[0];
            ad_tri_mask2[1] = mask_cross[1];
            ad_tri_mask2[2] = mask_cross[2];
        }
        else if (std::abs(dist_tmp2[0] - dist_tmp2[2]) < upsample_Param.diff_th) {
            mask2_tri[0] = 0; mask2_tri[1] = 1; mask2_tri[2] = 0;
            mask1_tri[0] = 1; mask1_tri[1] = 1; mask1_tri[2] = 1;
            ad_tri_mask2[0] = 2; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 0;
        }
        else if (std::abs(dist_tmp2[4] - dist_tmp2[2]) < upsample_Param.diff_th) {
            mask2_tri[0] = 0; mask2_tri[1] = 1; mask2_tri[2] = 0;
            mask1_tri[0] = 1; mask1_tri[1] = 1; mask1_tri[2] = 1;
            ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 2;
        }
        else {
            ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
        }
    }
    else if ((consis_cnt2 == 3 || (std::abs(dist_diff22 - dist_diff23) < diff2_th &&
        abs_dist_diff[4] < diff_th_max2[0])) && mask_cross_sum == 1) {

        int product_sum = 0;
        product_sum += mask_cross[0] * mask_e1[0];
        product_sum += mask_cross[1] * mask_e1[1];
        product_sum += mask_cross[2] * mask_e1[2];


        if (product_sum > 0) {
            mask1_tri[0] = mask_cross[0]; mask1_tri[1] = mask_cross[1]; mask1_tri[2] = mask_cross[2];
            mask2_tri[0] = 1; mask2_tri[1] = 1; mask2_tri[2] = 1;
        }
        else if (mask_cross[1] == 0) {
            ad_tri_mask1[0] = mask_cross[0];
            ad_tri_mask1[1] = mask_cross[1];
            ad_tri_mask1[2] = mask_cross[2];
        }
        else if (std::abs(dist_tmp1[0] - dist_tmp1[2]) < upsample_Param.diff_th) {
            mask1_tri[0] = 0; mask1_tri[1] = 1; mask1_tri[2] = 0;
            mask2_tri[0] = 1; mask2_tri[1] = 1; mask2_tri[2] = 1;
            ad_tri_mask1[0] = 2; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 0;
        }
        else if (std::abs(dist_tmp1[4] - dist_tmp1[2]) < upsample_Param.diff_th) {
            mask1_tri[0] = 0; mask1_tri[1] = 1; mask1_tri[2] = 0;
            mask2_tri[0] = 1; mask2_tri[1] = 1; mask2_tri[2] = 1;
            ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 2;
        }
        else {
            ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 1; ad_tri_mask1[2] = 0;
        }
    }
    else if (mask_cross[1] && (mask_c1_sum == 2)) {
        if (mask_c1[0] > 0) {
            if (std::abs(dist_tmp1[0] - dist_tmp1[1]) >= diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (std::abs(dist_tmp2[2] - dist_tmp2[0]) < diff_th) {
                    ad_tri_mask2[0] = 2; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 0;
                }
            }
            else if (std::abs(dist_tmp2[2] - dist_tmp2[0]) >= diff_th) {
                ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
            }
        }
        else if (mask_c1[2]) {
            if (std::abs(dist_tmp1[3] - dist_tmp1[4]) >= diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (std::abs(dist_tmp2[2] - dist_tmp2[4]) < diff_th) {
                    ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 2;
                }
            }
            else if (std::abs(dist_tmp2[2] - dist_tmp2[4]) >= diff_th) {
                ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
            }
        }
    }
    else if (mask_cross[1] && (mask_c2_sum == 2)) {
        if (mask_c2[0] > 0) {
            if (std::abs(dist_tmp2[0] - dist_tmp2[1]) >= upsample_Param.diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (std::abs(dist_tmp1[2] - dist_tmp1[0]) < upsample_Param.diff_th) {
                    ad_tri_mask1[0] = 2; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 0;
                }
            }
            else if (std::abs(dist_tmp1[2] - dist_tmp1[0]) >= upsample_Param.diff_th) {
                ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 1; ad_tri_mask1[2] = 0;
            }
        }
        else if (mask_c2[2]) {
            if (std::abs(dist_tmp2[3] - dist_tmp2[4]) >= upsample_Param.diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (std::abs(dist_tmp1[2] - dist_tmp1[4]) < upsample_Param.diff_th) {
                    ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 2;
                }
            }
            else if (std::abs(dist_tmp1[2] - dist_tmp1[4]) >= upsample_Param.diff_th) {
                ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 1; ad_tri_mask1[2] = 0;
            }
        }
    }
    else if (mask_cross[1]) {
        mask1_tri[1] = 1;
        mask2_tri[1] = 1;
    }
    else if (std::abs(dist_tmp1[1] - dist_tmp2[3]) < upsample_Param.diff_th &&
        dist_tmp1[1] > 0 && dist_tmp2[3] > 0) {
        mask1_tri[0] = 1;
        mask2_tri[2] = 1;
    }
    else if (std::abs(dist_tmp2[1] - dist_tmp1[3]) < upsample_Param.diff_th &&
        dist_tmp2[1] > 0 && dist_tmp1[3] > 0) {
        mask1_tri[2] = 1;
        mask2_tri[0] = 1;
    }

    // 最终计数（使用快速求和）
    int tri_cnt1 = mask1_tri[0] + mask1_tri[1] + mask1_tri[2];
    int tri_cnt2 = mask2_tri[0] + mask2_tri[1] + mask2_tri[2];

    if (tri_cnt1 >= 1 && tri_cnt2 >= 1) {
        memcpy(mask1, mask1_tri, 3 * sizeof(int));
        memcpy(mask2, mask2_tri, 3 * sizeof(int));
        memcpy(ad_mask1, ad_tri_mask1, 3 * sizeof(int));
        memcpy(ad_mask2, ad_tri_mask2, 3 * sizeof(int));
    }
}

/**
 * \brief  Up sample distance and reflectance main function
 *
 * \param[in] up_smpl_col: column index of sample point
 *                Range: 0 - 759. Accuracy: 1.
 * \param[in] up_smpl_col_buffer: column index of sample point in buffer
 *                Range: 0 - 1. Accuracy: 1.
 * \param[in] up_smpl_col2_buffer: column index of sample point in buffer
 *                Range: 0 - 1. Accuracy: 1.
 * \param[in] pu16Dist: distance neighborhood of wave 0
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] pu8Ref: reflectance neighborhood of wave 0
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] pstFrameBuffer: frame buffer
 */
void AlgoFunction::processUpSample(int up_smpl_col,
                                    int up_smpl_col_buffer,
                                    int up_smpl_col2_buffer,
                                    uint16_t *pu16Dist,
                                    uint8_t *pu8Ref,
                                    tstFrameBuffer* frame_buffer) {
    //列有效性检查（转换为0-based索引）
    if (up_smpl_col < 0 || up_smpl_col >= VIEW_W) return;
    int surface_id = frame_buffer->surface_id.load();
    int up_smpl_col_in_raw;
    uint16_t dist_wave0_buffer_raw[VIEW_H] = {0};
    uint8_t refl_wave0_buffer_raw[VIEW_H] = {0};
    if(0 == surface_id) {
        up_smpl_col_in_raw = up_smpl_col;
    }
    else {
        up_smpl_col_in_raw = up_smpl_col + 1;
    }

    if((up_smpl_col_in_raw >= 0) && (up_smpl_col_in_raw < VIEW_W)) {
        memcpy(dist_wave0_buffer_raw, &frame_buffer->dist0_raw[up_smpl_col_in_raw][0], VIEW_H * sizeof(uint16_t));
        memcpy(refl_wave0_buffer_raw, &frame_buffer->ref0_raw[up_smpl_col_in_raw][0], VIEW_H * sizeof(uint8_t));
    }

    //插入模式处理
    uint16_t dist_ins_c[VIEW_H] = { 0 };
    uint8_t ref_ins_c[VIEW_H] = { 0 };
    if(0 == up_smpl_col){
        data_abnormal_ = false;
    }

    if (up_smpl_col < VIEW_W - 1) {
        const int window_size = (upsample_Param.WH << 1) + 1;

        int dist_tmp1[5];
        int dist_tmp2[5];
        int ref_tmp1[5];
        int ref_tmp2[5];

        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            // 快速初始化（比循环更快）
            memset(dist_tmp1, 0, window_size * sizeof(int));
            memset(dist_tmp2, 0, window_size * sizeof(int));
            memset(ref_tmp1, 0, window_size * sizeof(int));
            memset(ref_tmp2, 0, window_size * sizeof(int));
            int dist_raw = dist_wave0_buffer_raw[row_idx];
            int ref_raw = refl_wave0_buffer_raw[row_idx];

            // 合并数据提取逻辑
            if (row_idx > upsample_Param.WH - 1 && row_idx < VIEW_H - upsample_Param.WH) {
                const int start_row = row_idx - upsample_Param.WH;
                for (int i = 0; i < window_size; ++i) {
                    const int src_row = start_row + i;
                    dist_tmp1[i] = dist_wave0_buffer5[up_smpl_col_buffer][src_row];
                    dist_tmp2[i] = dist_wave0_buffer5[up_smpl_col2_buffer][src_row];
                    ref_tmp1[i] = refl_wave0_buffer5[up_smpl_col_buffer][src_row];
                    ref_tmp2[i] = refl_wave0_buffer5[up_smpl_col2_buffer][src_row];
                }
            }
            else if (row_idx <= upsample_Param.WH - 1) {
                const int valid_rows = row_idx + upsample_Param.WH + 1;
                const int dest_start = upsample_Param.WH - row_idx;
                for (int i = 0; i < valid_rows; ++i) {
                    dist_tmp1[dest_start + i] = dist_wave0_buffer5[up_smpl_col_buffer][i];
                    dist_tmp2[dest_start + i] = dist_wave0_buffer5[up_smpl_col2_buffer][i];
                    ref_tmp1[dest_start + i] = refl_wave0_buffer5[up_smpl_col_buffer][i];
                    ref_tmp2[dest_start + i] = refl_wave0_buffer5[up_smpl_col2_buffer][i];
                }
            }
            else {
                const int valid_rows = upsample_Param.WH + (VIEW_H - row_idx);
                const int src_start = row_idx - upsample_Param.WH;
                for (int i = 0; i < valid_rows; ++i) {
                    const int src_row = src_start + i;
                    dist_tmp1[i] = dist_wave0_buffer5[up_smpl_col_buffer][src_row];
                    dist_tmp2[i] = dist_wave0_buffer5[up_smpl_col2_buffer][src_row];
                    ref_tmp1[i] = refl_wave0_buffer5[up_smpl_col_buffer][src_row];
                    ref_tmp2[i] = refl_wave0_buffer5[up_smpl_col2_buffer][src_row];
                }
            }

            // 使用栈数组存储mask和ad_mask
            int mask1[3], mask2[3], ad_mask1[3], ad_mask2[3];
            int PLRawFillMask = 0;
            rebuildFunc(dist_tmp1, dist_tmp2, mask1, mask2, ad_mask1, ad_mask2, &PLRawFillMask);

            // 提取中心3个元素（对应原始代码的segment(1,3)）
            int dist_c1[3], dist_c2[3], ref_c1[3], ref_c2[3];
            for (int i = 0; i < 3; ++i) {
                dist_c1[i] = dist_tmp1[i + 1];
                dist_c2[i] = dist_tmp2[i + 1];
                ref_c1[i] = ref_tmp1[i + 1];
                ref_c2[i] = ref_tmp2[i + 1];
            }

            // 权重计算优化
            int w1_sum = 0, w2_sum = 0;
            int w1[3], w2[3];
            for (int i = 0; i < 3; ++i) {
                w1[i] = upsample_Param.weight1[i] * mask1[i];
                w2[i] = upsample_Param.weight2[i] * mask2[i];
                w1_sum += w1[i];
                w2_sum += w2[i];
            }
            const int total_weight = w1_sum + w2_sum;

            int dist_ins = 0, ref_ins = 0;
            if(PLRawFillMask)
            {
                if(abs(dist_c1[1] - dist_raw) < 200)
                {
                    dist_ins = dist_raw;
                    ref_ins = ref_raw;
                }
            }
            else if (total_weight > 0) {
                // 合并dist和ref的项计算
                int term_dist1 = 0, term_dist2 = 0;
                int term_ref1 = 0, term_ref2 = 0;
                for (int i = 0; i < 3; ++i) {
                    term_dist1 += w1[i] * dist_c1[i];
                    term_dist2 += w2[i] * dist_c2[i];
                    term_ref1 += w1[i] * ref_c1[i];
                    term_ref2 += w2[i] * ref_c2[i];
                }

                const int coef = upsample_Param.coef_inv[total_weight - 1];
                dist_ins = ((term_dist1 + term_dist2) * coef) >> 12;
                ref_ins = ((term_ref1 + term_ref2) * coef) >> 12;

                // 特殊边界优化
                if (w1[1] > 0 && w2[1] > 0) {
                    const int avg = (dist_c1[1] + dist_c2[1]) >> 1;
                    const int diff = std::abs(dist_ins - avg);
                    if (diff > 30 || ((dist_ins > dist_c1[1]) == (dist_ins > dist_c2[1]))) {
                        dist_ins = avg;
                    }
                }

                // Ref边界优化
                const int ref_c1_1 = ref_c1[1], ref_c2_1 = ref_c2[1];
                if (ref_c1_1 > ref_ins && ref_c2_1 > ref_ins) {
                    ref_ins = std::min(ref_c1_1, ref_c2_1);
                }
                else if (ref_c1_1 < ref_ins && ref_c2_1 < ref_ins) {
                    ref_ins = std::max(ref_c1_1, ref_c2_1);
                }

                if(!((dist_raw > 0) && (dist_ins > 0) && (std::abs(dist_raw - dist_ins) < 100))){
                    dist_ins = 0;
                    ref_ins = 0;
                }
            }

            // 处理ad_mask优化
            for (int i = 0; i < 3; ++i) {
                if (ad_mask1[i] == 1) {
                    dist_c1[i] = ref_c1[i] = 0;
                }
                else if (ad_mask1[i] == 2) {
                    dist_c1[i] = (dist_tmp1[i] + dist_tmp1[i + 2]) >> 1;
                    ref_c1[i] = std::min(ref_tmp1[i], ref_tmp1[i + 2]);
                }

                if (ad_mask2[i] == 1) {
                    dist_c2[i] = ref_c2[i] = 0;
                }
                else if (ad_mask2[i] == 2) {
                    dist_c2[i] = (dist_tmp2[i] + dist_tmp2[i + 2]) >> 1;
                    ref_c2[i] = std::min(ref_tmp2[i], ref_tmp2[i + 2]);
                }
            }

            // 存储结果
            dist_ins_c[row_idx] = dist_ins;
            ref_ins_c[row_idx] = ref_ins;

            // 优化缓冲区更新逻辑
            if (row_idx > 0 && row_idx < VIEW_H - 1) {
                const int update_row = row_idx - 1;
                for (int i = 0; i < 3; ++i) {
                    dist_wave0_buffer5[up_smpl_col_buffer][update_row + i] = dist_c1[i];
                    dist_wave0_buffer5[up_smpl_col2_buffer][update_row + i] = dist_c2[i];
                    refl_wave0_buffer5[up_smpl_col_buffer][update_row + i] = ref_c1[i];
                    refl_wave0_buffer5[up_smpl_col2_buffer][update_row + i] = ref_c2[i];
                }
            }
            else if (row_idx == 0) {
                dist_wave0_buffer5[up_smpl_col_buffer][0] = dist_c1[1];
                dist_wave0_buffer5[up_smpl_col_buffer][1] = dist_c1[2];
                dist_wave0_buffer5[up_smpl_col2_buffer][0] = dist_c2[1];
                dist_wave0_buffer5[up_smpl_col2_buffer][1] = dist_c2[2];

                refl_wave0_buffer5[up_smpl_col_buffer][0] = ref_c1[1];
                refl_wave0_buffer5[up_smpl_col_buffer][1] = ref_c1[2];
                refl_wave0_buffer5[up_smpl_col2_buffer][0] = ref_c2[1];
                refl_wave0_buffer5[up_smpl_col2_buffer][1] = ref_c2[2];
            }
            else {
                const int row_end = VIEW_H - 1;
                dist_wave0_buffer5[up_smpl_col_buffer][row_end - 1] = dist_c1[0];
                dist_wave0_buffer5[up_smpl_col_buffer][row_end] = dist_c1[1];
                dist_wave0_buffer5[up_smpl_col2_buffer][row_end - 1] = dist_c2[0];
                dist_wave0_buffer5[up_smpl_col2_buffer][row_end] = dist_c2[1];

                refl_wave0_buffer5[up_smpl_col_buffer][row_end - 1] = ref_c1[0];
                refl_wave0_buffer5[up_smpl_col_buffer][row_end] = ref_c1[1];
                refl_wave0_buffer5[up_smpl_col2_buffer][row_end - 1] = ref_c2[0];
                refl_wave0_buffer5[up_smpl_col2_buffer][row_end] = ref_c2[1];
            }
            if(dist_wave0_buffer5[up_smpl_col_buffer][row_idx] < 0 || dist_wave0_buffer5[up_smpl_col_buffer][row_idx] > 60000) {
                LogError("Data abnormal: dist_wave0_buffer5[up_smpl_col_buffer][row_idx] = %d", dist_wave0_buffer5[up_smpl_col_buffer][row_idx]);
                FaultManager::getInstance().setFault(FaultBits::DataAbnormalFault);
                data_abnormal_ = true;
            }
        }
    }
    if((up_smpl_col == (VIEW_W - 1)) && !data_abnormal_){
        if(FaultManager::getInstance().hasFault(FaultBits::DataAbnormalFault))
            FaultManager::getInstance().clearFault(FaultBits::DataAbnormalFault);
    }

    memcpy(pu16Dist, dist_wave0_buffer5[up_smpl_col_buffer], sizeof(uint16_t) * VIEW_H);
    memcpy(&pu16Dist[VIEW_H], dist_ins_c, sizeof(uint16_t) * VIEW_H);
    memcpy(pu8Ref, refl_wave0_buffer5[up_smpl_col_buffer], sizeof(uint8_t) * VIEW_H);
    memcpy(&pu8Ref[VIEW_H], ref_ins_c, sizeof(uint8_t) * VIEW_H);
}

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
 * \brief  Denoise main process function
 *
 * \param[in] denoise_col_buffer: buffer index of sample point
 *                 Range: 0 - 10. Accuracy: 1.
 * \param[in] denoise_col_valid: valid index of sample point
 *                 Range: 0 - 10. Accuracy: 1.
 * \param[in] denoise_col_neib_buf: neighbor buffer index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] denoise_col_neib_vld: neighbor buffer index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] col_idx: neighbor valid index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 */
void AlgoFunction::denoiseProcessing(int denoise_col_buffer,
    int denoise_col_valid, int* denoise_col_neib_buf,
    int* denoise_col_neib_vld, int col_idx)
{
    const int current_col = col_idx;
    const int win_len_h = noise_params.WinLenH;
    const int win_len_v = noise_params.WinLenV;
    const int HL = 2* noise_params.WinLenH + 1;
    const int HV = 2* noise_params.WinLenV + 1;

    // 边界检查
    if (current_col < 0 || current_col >= algo_Param.ViewField_Wide) {
        return;
    }

    // 非去噪模式处理
    if (!algo_Param.DenoiseOn) {
        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            if (denoise_valid_buffer[denoise_col_valid][row_idx] == 0) {
                denoise_valid_buffer[denoise_col_valid][row_idx] = 1;
            }
        }
        return;
    }

    // 预加载常用参数
    const auto& params = noise_params;
    const int block_size = BlockSize;
    const int dist_seg_div = 12000; // 合并除法运算

    // 主处理循环
     for (int row_idx = 0; row_idx < VIEW_H; ++row_idx)
    {
        const int dist_cur = dist_wave0_buffer1[denoise_col_buffer][row_idx];
        const int valid_cur = denoise_valid_buffer[denoise_col_buffer][row_idx];
        const int block_id = (row_idx + BlockSize) / BlockSize - 1;
        const int dist_seg = (dist_cur / dist_seg_div);
        const int region = params.region_list[block_id] - 1;
        const int threshold = noise_params.denoise_point[dist_seg][region][0];
        const int threshold1 = noise_params.denoise_point[dist_seg][region][1];
        const int threshold2 = noise_params.denoise_point[dist_seg][region][2];
        // 跳过无效距离值
        if(dist_cur > 8000 || (valid_cur != 1 && dist_cur > 0))
        {
            int denoise_refer_mask = 0;
            // 区块和距离段计算优化
            const int block_id = (row_idx + block_size) / block_size - 1;
            const int diff_th = params.denoise_offset[dist_seg]; // 数组访问优化

            // 邻域距离值预取
            const int HL = 2 * params.WinLenH + 1;
            const int HV = 2 * params.WinLenV + 1;

            int dist_neib[HL][HV] ={0};
            int ValidMask[HL][HV] ={0};
            for(int i = -params.WinLenH; i <= params.WinLenH; i++)
            {
                dist_neib[i + params.WinLenH][params.WinLenV] = dist_wave0_buffer1[denoise_col_neib_buf[2 + i]][row_idx];
            }

            // 邻域有效性统计优化
            int valid_cnt = 0;
            for (int i = 0; i < HL; ++i)
            {
                bool is_valid = (std::abs(dist_neib[i][params.WinLenV] - dist_cur) <= diff_th);
                ValidMask[i][params.WinLenV] = is_valid;
                valid_cnt += is_valid;
            }

            // 两侧有效性统计
            int valid_pre_2 = ValidMask[params.WinLenH - 1][params.WinLenV]
                                + ValidMask[params.WinLenH + 1][params.WinLenV];

            denoise_refer_mask = (valid_pre_2 >= threshold1)
                                        ? ((valid_cnt > 2) ? 2 : 1)
                                        : 3;
            if (denoise_refer_mask == 1) {
                // 邻域窗口构建优化
                int valid_pre_3 = ValidMask[0][win_len_v] + ValidMask[HL - 1][win_len_v];
                int valid_neib[HL][HV]= {0};
                int NeibValidNum[3] = {0};
                for(int i = 0; i < HL; i++)
                {
                    valid_neib[i][win_len_v] = denoise_valid_buffer[denoise_col_neib_buf[i]][row_idx];
                }
                for (int dv = -win_len_v; dv <= win_len_v; dv++) {
                    if ((dv == 0) && (row_idx != (VIEW_H - 1))) continue;
                    const int row_neib = row_idx + dv;
                    if (row_neib < 0 || row_neib >= VIEW_H || dv == row_idx) continue;

                    const int v_idx = dv + win_len_v;
                    for (int dh = -win_len_h; dh <= win_len_h; dh++) {
                        const int h_idx = dh + win_len_h;
                        const int col_buf_idx = denoise_col_neib_buf[h_idx];

                        dist_neib[h_idx][v_idx] = dist_wave0_buffer1[col_buf_idx][row_neib];
                        valid_neib[h_idx][v_idx] = denoise_valid_buffer[col_buf_idx][row_neib];

                        // 有效性判断
                        const int nei_val = dist_neib[h_idx][v_idx];
                        if (std::abs(nei_val - dist_cur) <= diff_th && (valid_neib[h_idx][v_idx] != 6)) {
                            const int zone = noise_params.zone_matrix[v_idx][h_idx];
                            if (zone > 0) {
                                NeibValidNum[zone - 1]++;
                                ValidMask[h_idx][v_idx] = 1;
                            }
                        }
                    }
                }

                // 去噪条件判断优化
                if (((NeibValidNum[0] + valid_pre_2) >= threshold)
                    || (row_idx == (VIEW_H - 1) && (valid_pre_2 >= threshold1))
                    || (NeibValidNum[0] + valid_pre_2 >= threshold)
                    || (valid_pre_3 + valid_pre_2 >= threshold2)) {
                    denoise_valid_buffer[denoise_col_valid][row_idx] = 1;

                    // 传播有效标记优化
                    for (int dv = -win_len_v; dv <= win_len_v; dv++) {
                        if ((dv == 0) && (row_idx != (VIEW_H - 1))) continue;
                        const int v_idx = dv + win_len_v;
                        const int target_row = row_idx + dv;
                        if (target_row < 0 || target_row >= VIEW_H) continue;

                        for (int dh = -win_len_h; dh <= win_len_h; dh++) {
                            const int h_idx = dh + win_len_h;
                            if (ValidMask[h_idx][v_idx] == 1) {
                                int idx = denoise_col_neib_vld[h_idx];
                                denoise_valid_buffer[idx][target_row] = 1;
                            }
                        }
                    }
                }
            }
            else if (denoise_refer_mask == 2) {
                // 直接对左右补点优化
                denoise_valid_buffer[denoise_col_valid][row_idx] = 1;

                for (int j = -noise_params.WinLenH; j <= noise_params.WinLenH; j++) {
                    if (ValidMask[noise_params.WinLenH + j][noise_params.WinLenV] == 1) {
                        int idx = denoise_col_neib_vld[3 + j - 1];
                        denoise_valid_buffer[idx][row_idx] = 1;
                    }
                }
            }
        }
    }
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
 * \brief  Calculate height of the buffer
 *
 * \param[in] pu16Dist: distance buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] col: column index of the buffer
 *                 Range: 0 - 759. Accuracy: 1.
 * \param[in] fit_high: height of the buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 */
void AlgoFunction::highCalcFunc(uint16_t *dist0,uint16_t *dist1, int col,
                                int *fit_high0, int *fit_high1, uint8_t *gnd_mark0, uint8_t *gnd_mark1)
{
    int org_high0;
    int org_high1;
    int fit_ground_high0;
    int fit_ground_high1;
    const float a = fit_Params.a;
    const float b = fit_Params.b;
    const float c = fit_Params.c;
    int proj_dist0[VIEW_H];
    int proj_dist1[VIEW_H];
    const int dist_diff_th = 600;
    const int delta_z_th = 20;
    const int z_th = 300;
    const int dist_max_th = 16000;

    memset(gnd_mark0, 0, VIEW_H);
    memset(gnd_mark1, 0, VIEW_H);
    for (int i = 0; i < VIEW_H; ++i) {
        int dist_val0 = dist0[i];
        int dist_val1 = dist1[i];
        int Ix_val = Ix_in[i][col];
        int Iy_val = Iy_in[i][col];
        int Iz_val = Iz_in[i][col];

        org_high0 = (dist_val0 * Iz_val) >> 15;
        org_high1 = (dist_val1 * Iz_val) >> 15;
        fit_ground_high0 = std::floor((float)dist_val0 *
            (a * Ix_val / 32768.0 + b * Iy_val / 32768.0) + c);
        fit_ground_high1 = std::floor((float)dist_val1 *
            (a * Ix_val / 32768.0 + b * Iy_val / 32768.0) + c);
        fit_high0[i] = org_high0 - fit_ground_high0;
        fit_high1[i] = org_high1 - fit_ground_high1;
        proj_dist0[i] = (dist_val0 * cosd_pitch_lut[i]) >> 15;
        proj_dist1[i] = (dist_val1 * cosd_pitch_lut[i]) >> 15;
    }

    // 3. 地面检测主循环
    for (int row_idx = 1; row_idx < VIEW_H - 1; row_idx++) {
        // 当前点数据
        int dist_cur = dist0[row_idx];
        int x_cur = proj_dist0[row_idx];
        int z_cur = fit_high0[row_idx];
        int dist_cur2 = dist1[row_idx];
        int x_cur2 = proj_dist1[row_idx];
        int z_cur2 = fit_high1[row_idx];

        // 下邻点数据 (row_idx-1)
        int dist_down = dist0[row_idx-1];
        int x_down = proj_dist0[row_idx-1];
        int z_down = fit_high0[row_idx-1];
        int dist_down2 = dist1[row_idx-1];
        int x_down2 = proj_dist1[row_idx-1];
        int z_down2 = fit_high1[row_idx-1];

        // 上邻点数据 (row_idx+1)
        int dist_up = dist0[row_idx+1];
        int x_up = proj_dist0[row_idx+1];
        int z_up = fit_high0[row_idx+1];
        int dist_up2 = dist1[row_idx+1];
        int x_up2 = proj_dist1[row_idx+1];
        int z_up2 = fit_high1[row_idx+1];

        // 计算邻近标志
        bool near_up_1 = (dist_up > 0) && (dist_up - dist_cur > 0) && (dist_up - dist_cur <= dist_diff_th);
        bool near_up_2 = (dist_up2 > 0) && (dist_up2 - dist_cur > 0) && (dist_up2 - dist_cur <= dist_diff_th);
        bool near_down_1 = (dist_down > 0) && (dist_cur - dist_down > 0) && (dist_cur - dist_down <= dist_diff_th);
        bool near_down_2 = (dist_down2 > 0) && (dist_cur - dist_down2 > 0) && (dist_cur - dist_down2 <= dist_diff_th);

        // 计算邻近标志 (第二回波)
        bool near2_up_1 = (dist_up > 0) && (dist_up - dist_cur2 > 0) && (dist_up - dist_cur2 <= dist_diff_th);
        bool near2_up_2 = (dist_up2 > 0) && (dist_up2 - dist_cur2 > 0) && (dist_up2 - dist_cur2 <= dist_diff_th);
        bool near2_down_1 = (dist_down > 0) && (dist_cur2 - dist_down > 0) && (dist_cur2 - dist_down <= dist_diff_th);
        bool near2_down_2 = (dist_down2 > 0) && (dist_cur2 - dist_down2 > 0) && (dist_cur2 - dist_down2 <= dist_diff_th);

        // 计算平坦标志 (第一回波)
        int dz_up = std::abs(z_up - z_cur);
        int dz_down = std::abs(z_down - z_cur);
        int dx_up = std::abs(x_up - x_cur);
        int dx_down = std::abs(x_down - x_cur);
        int dz_down2 = std::abs(z_down2 - z_cur);
        int dx_down2 = std::abs(x_down2 - x_cur);
        int dz_up2 = std::abs(z_up2 - z_cur);
        int dx_up2 = std::abs(x_up2 - x_cur);

        bool flat_flag11 = (dz_up * 2 <= dx_up) && (dz_down * 2 <= dx_down);
        bool flat_flag12 = (dz_up * 2 <= dx_up) && (dz_down2 * 2 <= dx_down2);
        bool flat_flag21 = (dz_up2 * 2 <= dx_up2) && (dz_down * 2 <= dx_down);
        bool flat_flag22 = (dz_up2 * 2 <= dx_up2) && (dz_down2 * 2 <= dx_down2);

        // 计算平坦标志 (第二回波)
        int dz2_up = std::abs(z_up - z_cur2);
        int dz2_down = std::abs(z_down - z_cur2);
        int dx2_up = std::abs(x_up - x_cur2);
        int dx2_down = std::abs(x_down - x_cur2);
        int dz2_down2 = std::abs(z_down2 - z_cur2);
        int dx2_down2 = std::abs(x_down2 - x_cur2);
        int dz2_up2 = std::abs(z_up2 - z_cur2);
        int dx2_up2 = std::abs(x_up2 - x_cur2);

        bool flat2_flag11 = (dz2_up * 2 <= dx2_up) && (dz2_down * 2 <= dx2_down);
        bool flat2_flag12 = (dz2_up * 2 <= dx2_up) && (dz2_down2 * 2 <= dx2_down2);
        bool flat2_flag21 = (dz2_up2 * 2 <= dx2_up2) && (dz2_down * 2 <= dx2_down);
        bool flat2_flag22 = (dz2_up2 * 2 <= dx2_up2) && (dz2_down2 * 2 <= dx2_down2);

        // 计算角度标志 (第一回波)
        bool angle_flag11 = std::abs(z_up + z_down - 2*z_cur) <= delta_z_th;
        bool angle_flag12 = std::abs(z_up + z_down2 - 2*z_cur) <= delta_z_th;
        bool angle_flag21 = std::abs(z_up2 + z_down - 2*z_cur) <= delta_z_th;
        bool angle_flag22 = std::abs(z_up2 + z_down2 - 2*z_cur) <= delta_z_th;

        // 计算角度标志 (第二回波)
        bool angle2_flag11 = std::abs(z_up + z_down - 2*z_cur2) <= delta_z_th;
        bool angle2_flag12 = std::abs(z_up + z_down2 - 2*z_cur2) <= delta_z_th;
        bool angle2_flag21 = std::abs(z_up2 + z_down - 2*z_cur2) <= delta_z_th;
        bool angle2_flag22 = std::abs(z_up2 + z_down2 - 2*z_cur2) <= delta_z_th;

        // ROI标志
        bool roi_flag = (dist_cur > 0) && (dist_cur <= dist_max_th) && (std::abs(z_cur) <= z_th);
        bool roi_flag2 = (dist_cur2 > 0) && (dist_cur2 <= dist_max_th) && (std::abs(z_cur2) <= z_th);

        // 组合标志 (第一回波)
        bool flag_11 = near_up_1 && near_down_1 && flat_flag11 && angle_flag11;
        bool flag_21 = near_up_1 && near_down_2 && flat_flag12 && angle_flag12;
        bool flag_12 = near_up_2 && near_down_1 && flat_flag21 && angle_flag21;
        bool flag_22 = near_up_2 && near_down_2 && flat_flag22 && angle_flag22;

        // 组合标志 (第二回波)
        bool flag2_11 = near2_up_1 && near2_down_1 && flat2_flag11 && angle2_flag11;
        bool flag2_21 = near2_up_1 && near2_down_2 && flat2_flag12 && angle2_flag12;
        bool flag2_12 = near2_up_2 && near2_down_1 && flat2_flag21 && angle2_flag21;
        bool flag2_22 = near2_up_2 && near2_down_2 && flat2_flag22 && angle2_flag22;

        // 地面标志计算
        bool ground_flag = roi_flag && (flag_11 || flag_12 || flag_21 || flag_22) ||
                          (std::abs(z_cur) < 20 && dist_cur < 4000);

        bool ground_flag2 = !ground_flag && roi_flag2 &&
                           (flag2_11 || flag2_12 || flag2_21 || flag2_22) ||
                           (std::abs(z_cur2) < 20 && dist_cur2 < 4000);

        // 更新地面标记 (当前行)
        if (ground_flag) gnd_mark0[row_idx] |= 1;
        if (ground_flag2) {
            gnd_mark1[row_idx] |= 1;
            gnd_mark0[row_idx] = 0; // 第一回波标记置0
        }

        // 更新 row_idx-1 的第一回波标记 (gnd_mark_line(row_idx-1))
        if ((ground_flag && (flag_11 || flag_12)) ||
            (ground_flag2 && (flag2_11 || flag2_12))) {
            gnd_mark0[row_idx-1] |= 1;
        }

        // 更新 row_idx+1 的第一回波标记 (gnd_mark_line(row_idx+1))
        if ((ground_flag && (flag_11 || flag_21)) ||
            (ground_flag2 && (flag2_11 || flag2_21))) {
            gnd_mark0[row_idx+1] |= 1;
        }

        // 更新 row_idx-1 的第二回波标记 (gnd_mark_line2(row_idx-1))
        if ((ground_flag && (flag_21 || flag_22)) ||
            (ground_flag2 && (flag2_21 || flag2_22))) {
            gnd_mark1[row_idx-1] |= 1;
        }

        // 更新 row_idx+1 的第二回波标记 (gnd_mark_line2(row_idx+1))
        if ((ground_flag && (flag_12 || flag_22)) ||
            (ground_flag2 && (flag2_12 || flag2_22))) {
            gnd_mark1[row_idx+1] |= 1;
        }

        // 低高度强制标记
        if (std::abs(z_down) < 20 && dist_down < 4000) {
            gnd_mark0[row_idx-1] = 2;
        }
        if (std::abs(z_down2) < 20 && dist_down2 < 4000) {
            gnd_mark1[row_idx-1] = 2;
        }
    }
}

/**
 * \brief  Final decision of the algorithm, delete points with mask
 *
 * \param[in] col_idx: column index of the buffer
 *                 Range: 0 - 759. Accuracy: 1.
 * \param[in] pu16Dist: distance buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] pu8Ref: reflectance buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] pstFrameBuffer: frame buffer
 */
void AlgoFunction::algoFianlDecision(int col_idx, uint16_t *pu16Dist, uint8_t *pu8Ref, tstFrameBuffer* frame_buffer)
{
    memcpy(pu16Dist, frame_buffer->dist0[col_idx], sizeof(uint16_t) * VIEW_H);
    memcpy(&pu16Dist[VIEW_H], frame_buffer->dist0[col_idx], sizeof(uint16_t) * VIEW_H);
    memcpy(pu8Ref, frame_buffer->ref0[col_idx], sizeof(uint8_t) * VIEW_H);
    memcpy(&pu8Ref[VIEW_H], frame_buffer->ref0[col_idx], sizeof(uint8_t) * VIEW_H);
    //提前计算环形缓冲区索引
    circularCalcIdx(rear5, buffer_size_upsampling);
    const int decision_col = col_idx;
    const int decision_col_buffer = matlabMod(rear5 - 1 + 1, buffer_size_upsampling);

    // 边界检查提前
    if (decision_col >= 0 && decision_col < VIEW_W) {
        // 获取当前列的指针，减少数组访问
        uint16_t* current_dist_wave0 = stFilterFrmBuffer.dist_wave0[decision_col];
        uint16_t* current_dist_wave1 = stFilterFrmBuffer.dist_wave1[decision_col];
        uint8_t* current_refl_wave0 = stFilterFrmBuffer.refl_wave0[decision_col];
        uint8_t* current_refl_wave1 = stFilterFrmBuffer.refl_wave1[decision_col];

        uint16_t* buffer_dist_wave0 = dist_wave0_buffer5[decision_col_buffer];
        uint16_t* buffer_dist_wave1 = dist_wave1_buffer5[decision_col_buffer];
        uint8_t* buffer_refl_wave0 = refl_wave0_buffer5[decision_col_buffer];
        uint8_t* buffer_refl_wave1 = refl_wave1_buffer5[decision_col_buffer];

        // 一次性拷贝整列数据
        memcpy(buffer_dist_wave0, current_dist_wave0, sizeof(uint16_t) * VIEW_H);
        memcpy(buffer_dist_wave1, current_dist_wave1, sizeof(uint16_t) * VIEW_H);
        memcpy(buffer_refl_wave0, current_refl_wave0, sizeof(uint8_t) * VIEW_H);
        memcpy(buffer_refl_wave1, current_refl_wave1, sizeof(uint8_t) * VIEW_H);

        // 获取掩码指针
        uint8_t* trail_mask_out = trail_mask_out_frm[decision_col];
        int* stray_mask_out0 = stray_mask_out_frm0[decision_col];
        int* stray_mask_out1 = stray_mask_out_frm1[decision_col];
        int* spray_mask_out0 = spray_mark_out_frm0[decision_col];
        int* spray_mask_out1 = spray_mark_out_frm1[decision_col];

        // 优化处理循环
        for (int row_idx = 0; row_idx < VIEW_H; row_idx++)
        {
            const bool stray0 = stray_mask_out0[row_idx];
            const bool stray1 = stray_mask_out1[row_idx];
            const bool spray0 = spray_mask_out0[row_idx];
            const bool spray1 = spray_mask_out1[row_idx];
            const bool trail = trail_mask_out[row_idx];
            const bool wave0_del_flag = stray0 || spray0 || trail;
            const bool wave1_del_flag = !stray1 && !spray1 && !trail;

            // 情况1: 两回波均无效点
            if (wave0_del_flag && !wave1_del_flag) {
                buffer_dist_wave0[row_idx] = 0;
                buffer_refl_wave0[row_idx] = 0;
            }
            // 情况2: 第一回波无效点，第二回波有效点
            else if (wave0_del_flag && wave1_del_flag) {
                buffer_dist_wave0[row_idx] = buffer_dist_wave1[row_idx];
                buffer_refl_wave0[row_idx] = buffer_refl_wave1[row_idx];
            }
            // 情况3: 第一回波有效，第二回波无效
            else if (!wave0_del_flag && !wave1_del_flag) {
                buffer_dist_wave1[row_idx] = 0;
                buffer_refl_wave1[row_idx] = 0;
            }
        }
    }

    // 上采样处理
    const int up_smpl_col = decision_col - UpSamplingDelayCol;
    const int up_smpl_col_buffer = matlabMod(decision_col_buffer + 1 - UpSamplingDelayCol - 1, buffer_size_upsampling);
    const int up_smpl_col2_buffer = matlabMod(up_smpl_col_buffer + 2 - 1, buffer_size_upsampling);

    processUpSample(up_smpl_col, up_smpl_col_buffer, up_smpl_col2_buffer, pu16Dist, pu8Ref, frame_buffer);
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

    using namespace robosense::lidar::yaml;

    if (algo_switch_param.data_valid) {
        LogDebug("Using algorithm switch in yaml file.");
        algo_Param.TrailRemoveOn = algo_switch_param.enable_trail_remove;
        algo_Param.DenoiseOn = algo_switch_param.enable_denoise;
        algo_Param.StrayRemoveOn = algo_switch_param.enable_stray;
        algo_Param.SprayRemoveOn = algo_switch_param.enable_spray;
    } else {
        LogWarn(__FILE__, __LINE__, __func__,
            "Algorithm switch yaml file read FAILED, Using default switch.");
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
 * \brief  Algorithm frame change
 */
void AlgoFunction::algoFrameChange(void)
{
    memset(denoise_valid_buffer, 0, sizeof(denoise_valid_buffer));

    memset(dist_wave0_buffer1, 0, sizeof(dist_wave0_buffer1));
    memset(dist_wave1_buffer1, 0, sizeof(dist_wave1_buffer1));
    memset(refl_wave0_buffer1, 0, sizeof(refl_wave0_buffer1));
    memset(refl_wave1_buffer1, 0, sizeof(refl_wave1_buffer1));
    memset(attr_wave0_buffer1, 0, sizeof(attr_wave0_buffer1));
    memset(attr_wave1_buffer1, 0, sizeof(attr_wave1_buffer1));
    memset(high_wave0_buffer1, 0, sizeof(high_wave0_buffer1));
    memset(high_wave1_buffer1, 0, sizeof(high_wave1_buffer1));

    memset(dist_wave0_buffer2, 0, sizeof(dist_wave0_buffer2));
    memset(refl_wave0_buffer2, 0, sizeof(refl_wave0_buffer2));
    memset(dist_wave0_buffer4, 0, sizeof(dist_wave0_buffer4));
    memset(refl_wave0_buffer4, 0, sizeof(refl_wave0_buffer4));

    memset(dist_wave0_buffer5, 0, sizeof(dist_wave0_buffer5));
    memset(dist_wave1_buffer5, 0, sizeof(dist_wave1_buffer5));
    memset(refl_wave0_buffer5, 0, sizeof(refl_wave0_buffer5));
    memset(refl_wave1_buffer5, 0, sizeof(refl_wave1_buffer5));

    rear1 = -1; //去噪
    rear2 = -1; //拖点
    rear3 = -1; //杂散
    rear4 = -1; //雨雾
    rear5 = -1; //上采样

    basic_rear = -1;
    filter_rear = -1;
    empty_rear = -1;
}

/**
 * @brief stray delete algorithm of lidar
 *
 * @param col_idx column index of the buffer
 *                  Range: 0 - 759. Accuracy: 1.
 * @param stray_col_buffer stray buffer column index
 *                  Range: 0 - 2. Accuracy: 1.
 * @param stray_col_neib_buf stray neighbor buffer column index
 *                  Range: 0 - 2. Accuracy: 1.
 * @param stray_mark_out0 stray wave0 buffer mark value
 *                  Range: 0 - 1. Accuracy: 1.
 * @param stray_mark_out1 stray wave1 buffer mark value
 *                  Range: 0 - 1. Accuracy: 1.
 */
void AlgoFunction::strayDelete(int col_idx,
    int stray_col_buffer,
    int *stray_col_neib_buf,
    int *stray_mark_out0,
    int *stray_mark_out1) {
    if(col_idx < 0 || col_idx >= VIEW_W){
        return;
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
        static int peak_buffer[VIEW_H][3];

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
        // 地面标记表
        int gnd_cnt_tab[28] = {0};
        int gnd_row_ed_tab[28] = {0};
        int gnd_height_st_tab[28];
        std::fill_n(gnd_height_st_tab, 28, 65535);
        int gnd_dist_max = 0;

        // 滑动窗口更新
        ceil_stray_dist[0] = ceil_stray_dist[1];
        ceil_stray_dist[1] = ceil_stray_dist[2];
        ceil_stray_dist[2] = 0;

        // 聚类变量
        int class_line[VIEW_H] = {0};

        int class_row_st = 0;
        int class_row_ed = 0;
        int class_cnt = 0;
        int class_dist = 0;
        int class_stray_cnt = 0;
        int class_stray_row = 0;

        int stray_chain_points_c = 0;
        int stray_chain_cnt_c = 0;
        int stray_chain_dist_c = 0;
        int stray_chain_row_st_c = 0;
        int stray_chain_row_ed_c = 0;

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

        // =============== 前级信息计算 ===============
        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            // 获取当前行数据
            int dist_cur = dist_wave0_buffer3[stray_col_buffer][row_idx];
            int dist_cur2 = dist_wave1_buffer3[stray_col_buffer][row_idx];

            int row_idx_up = std::max(0, row_idx - 1);
            int dist_up = dist_wave0_buffer3[stray_col_buffer][row_idx_up];
            int dist_up2 = dist_wave1_buffer3[stray_col_buffer][row_idx_up];

            int crosstalk_cur = crtk_wave0_buffer3[stray_col_buffer][row_idx];
            int crosstalk_cur2 = crtk_wave1_buffer3[stray_col_buffer][row_idx];
            int crosstalk_up = crtk_wave0_buffer3[stray_col_buffer][row_idx_up];
            int crosstalk_up2 = crtk_wave1_buffer3[stray_col_buffer][row_idx_up];

            int gnd_cur = grnd_wave0_buffer3[stray_col_buffer][row_idx];
            int gnd_cur2 = grnd_wave1_buffer3[stray_col_buffer][row_idx];

            int height_cur = high_wave0_buffer3[stray_col_buffer][row_idx];
            int height_cur2 = high_wave1_buffer3[stray_col_buffer][row_idx];

            bool save_flag_col = false;

            // ------ 地面标记统计 ------
            if (dist_cur > 0 && dist_cur < 14000 && gnd_cur) {
                int gnd_seg = dist_cur >> 9; // 等价于除以512
                if (gnd_seg >= 28) gnd_seg = 27;

                if (gnd_cnt_tab[gnd_seg] == 0) {
                    gnd_cnt_tab[gnd_seg] = 1;
                    gnd_row_ed_tab[gnd_seg] = row_idx;
                    gnd_height_st_tab[gnd_seg] = height_cur;
                } else if (row_idx - gnd_row_ed_tab[gnd_seg] <= 3 &&
                            std::abs(height_cur - gnd_height_st_tab[gnd_seg]) < 50) {
                    gnd_cnt_tab[gnd_seg] = std::min(32, gnd_cnt_tab[gnd_seg] + 1);
                    gnd_row_ed_tab[gnd_seg] = row_idx;
                }

                if ((dist_cur > gnd_dist_max && dist_cur - gnd_dist_max < 800) ||
                        gnd_dist_max == 0) {
                    gnd_dist_max = dist_cur;
                }
            }
            else if (dist_cur2 > 0 && dist_cur2 < 14000 && gnd_cur2) {
                int gnd_seg = dist_cur2 >> 9;
                if (gnd_seg >= 28) gnd_seg = 27;

                if (gnd_cnt_tab[gnd_seg] == 0) {
                    gnd_cnt_tab[gnd_seg] = 1;
                    gnd_row_ed_tab[gnd_seg] = row_idx;
                    gnd_height_st_tab[gnd_seg] = height_cur2;
                } else if (row_idx - gnd_row_ed_tab[gnd_seg] <= 3 &&
                            std::abs(height_cur2 - gnd_height_st_tab[gnd_seg]) < 50) {
                    gnd_cnt_tab[gnd_seg] = std::min(32, gnd_cnt_tab[gnd_seg] + 1);
                    gnd_row_ed_tab[gnd_seg] = row_idx;
                }

                if ((dist_cur2 > gnd_dist_max && dist_cur2 - gnd_dist_max < 800) ||
                        gnd_dist_max == 0) {
                    gnd_dist_max = dist_cur2;
                }
            }

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
                }
                else if (std::abs(class_dist - dist_cur) < dist_th_classify_cur &&
                            row_idx - class_row_ed < 3) {
                    // 普通块处理
                    if (0 == class_stray_cnt) {
                        if (crosstalk_cur > 0 && row_idx - class_row_st > 3) {
                            save_flag_col = true;
                        } else {
                            class_row_ed = row_idx;
                            class_cnt++;
                            class_dist = dist_cur;
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
                if(class_type && (class_stray_cnt * 4 >= class_row_ed - class_row_st)) {
                    if (0 == stray_chain_cnt_c) {
                        stray_chain_dist_c = class_dist;
                        stray_chain_points_c = class_cnt;
                        stray_chain_cnt_c = class_stray_cnt;
                        stray_chain_row_st_c = class_row_st;
                        stray_chain_row_ed_c = class_row_ed;
                    }
                    // 合并条件
                    else if(std::abs(class_dist - stray_chain_dist_c) < 100 &&
                            (class_row_st - stray_chain_row_ed_c < 20 || class_stray_cnt > 5)) {
                        stray_chain_dist_c = class_dist;
                        stray_chain_points_c += class_cnt;
                        stray_chain_cnt_c += class_stray_cnt;
                        stray_chain_row_ed_c = class_row_ed;
                    }
                    // 替换条件
                    else if(class_stray_cnt > stray_chain_cnt_c) {
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
                    class_row_st = row_idx;
                    class_row_ed = row_idx;
                    class_cnt = 1;
                    class_dist = dist_cur;
                    class_stray_cnt = (crosstalk_cur > 0) ? 1 : 0;
                    class_stray_row = (crosstalk_cur > 0) ? row_idx : 0;
                }
            }

            // 行结束处理
            if (row_idx == VIEW_H - 1) {
                bool save_flag_row = false;

                if (stray_chain_cnt_c > 0) {
                    int span_condition = (stray_chain_dist_c * (stray_chain_row_ed_c - stray_chain_row_st_c) * 7) >> 12;
                    if (span_condition > 120 && stray_chain_row_ed_c > 71) {
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

        // =============== 杂散判断 ===============
        for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
            // 获取当前行数据
            int dist_cur = dist_wave0_buffer3[stray_col_buffer][row_idx];
            int dist_cur2 = dist_wave1_buffer3[stray_col_buffer][row_idx];

            int row_idx_down = std::max(0, row_idx - 1);
            int row_idx_up = std::min(VIEW_H - 1, row_idx + 1);

            int dist_down = dist_wave0_buffer3[stray_col_buffer][row_idx_down];
            int dist_up = dist_wave0_buffer3[stray_col_buffer][row_idx_up];

            int ref_cur = refl_wave0_buffer3[stray_col_buffer][row_idx];
            int ref_cur2 = refl_wave1_buffer3[stray_col_buffer][row_idx];

            int peak_mark_cur = peak_wave0_buffer3[stray_col_buffer][row_idx];
            int peak_mark_l = peak_wave0_buffer3[stray_col_neib_buf[0]][row_idx];
            int peak_mark_r = peak_wave0_buffer3[stray_col_neib_buf[2]][row_idx];

            int gnd_cur = grnd_wave0_buffer3[stray_col_buffer][row_idx];
            int gnd_cur2 = grnd_wave1_buffer3[stray_col_buffer][row_idx];

            int height_cur = high_wave0_buffer3[stray_col_buffer][row_idx];
            int height_cur2 = high_wave1_buffer3[stray_col_buffer][row_idx];

            int crosstalk_cur = crtk_wave0_buffer3[stray_col_buffer][row_idx];
            int crosstalk_cur2 = crtk_wave1_buffer3[stray_col_buffer][row_idx];

            int smlr_cur2 = smlr_wave1_buffer3[stray_col_buffer][row_idx];

            int class_raw = class_line[row_idx];
            int class_down = (row_idx > 0) ? class_line[row_idx_down] : 0;
            int class_up = (row_idx < VIEW_H - 1) ? class_line[row_idx_up] : 0;

            // 地面高度处理
            int ground_seg = (dist_cur >> 9);
            if (ground_seg >= 28) ground_seg = 27;

            int ground_height = gnd_height_st_tab[ground_seg];
            int ground_dist_diff = 700;

            if (ground_seg >= 9) {
                if (gnd_cnt_tab[ground_seg] == 0) {
                    if (gnd_cnt_tab[ground_seg - 1] > 0) {
                        ground_height = gnd_height_st_tab[ground_seg - 1];
                    } else if (gnd_cnt_tab[ground_seg - 2] > 0) {
                        ground_height = gnd_height_st_tab[ground_seg - 2];
                    }
                }
            }

            // 更新地面标记
            if ((dist_cur2 > dist_cur && (gnd_cur2 && gnd_cur)) ||
                (ground_height != 65535 && (height_cur - ground_height > 50))) {
                gnd_cur = 0;
            }

            if (ground_height != 65535 && (height_cur2 - ground_height > 50)) {
                gnd_cur2 = 0;
            }

            // 连接标志
            bool connect_ground_flag =
                !gnd_cur &&
                ((height_cur - ground_height < 400 &&
                    std::abs(gnd_dist_max - dist_cur) <= ground_dist_diff) ||
                    (dist_cur > gnd_dist_max &&
                    dist_cur - gnd_dist_max <= 1600 &&
                    height_cur < 400));

            bool connect_ceil_flag =
                std::abs(dist_cur - ceil_stray_dist[2]) <= 400 ||
                std::abs(dist_cur - ceil_stray_dist[1]) <= 400 ||
                std::abs(dist_cur - ceil_stray_dist[0]) <= 400 ||
                (std::abs(dist_cur - RainWall_in.dist) < 960 &&
                    ((RainWall_in.cnt > 300) ||
                    (RainWall_in.dist > 3000 && RainWall_in.cnt > 80)));

            // 更新分类标记
            int class_cur = (class_raw == 1)  || (class_down == 1) || (class_up == 1) ||
                (std::abs(dist_up - dist_cur) > 400 &&
                    std::abs(dist_down - dist_cur) > 400) ||
                ((class_raw == 2) && (ref_cur < stray_Param.ref_del_th) && (abs(dist_cur - RainWall_in.dist) < 960));

            // 杂散和镜像标志
            bool stray_flag = (peak_mark_cur || peak_mark_l || peak_mark_r);
            bool mirror_flag = (row_idx < RegionSize * 2) &&
                                (dist_cur > 5000) &&
                                (height_cur < -600);

            // 删除条件
            bool straight_del_en = crosstalk_cur && class_cur;
            bool column_del_en =
                (stray_chain_cnt_c >= stray_Param.stray_cnt_th &&
                    std::abs(dist_cur - stray_chain_dist_c) < 240 && class_cur) ||
                (stray_chain_cnt[0] >= stray_Param.stray_cnt_th &&
                    std::abs(dist_cur - stray_chain_dist[0]) < 240 &&
                    ((row_idx >= stray_chain_row_st[0] && row_idx <= stray_chain_row_ed[0]) || class_cur)) ||
                (stray_chain_cnt[1] >= stray_Param.stray_cnt_th &&
                    std::abs(dist_cur - stray_chain_dist[1]) < 240 &&
                    ((row_idx >= stray_chain_row_st[1] && row_idx <= stray_chain_row_ed[1]) || class_cur));

            bool frame_del_en =
                (RainWall_in.cnt > 80) &&
                (std::abs(dist_cur - RainWall_in.dist) < 960) &&
                (height_cur > 0) &&
                class_cur;

            // 最终删除标记
            bool stray_tag_en = (algo_Param.StrayRemoveOn &&
                                (stray_flag || mirror_flag) &&
                                (straight_del_en || column_del_en || frame_del_en) && !gnd_cur);

            bool stray_del_en = stray_tag_en && !(connect_ground_flag && !connect_ceil_flag);

            // 第二回波选择逻辑
            bool wave1_sel_base = (smlr_cur2 >= 1) && (crosstalk_cur2 == 0);
            bool wave1_sel_column =
                (stray_chain_cnt_c < stray_Param.stray_cnt_th ||
                        std::abs(dist_cur2 - stray_chain_dist_c) > 240) &&
                (stray_chain_cnt[0] < stray_Param.stray_cnt_th ||
                        std::abs(dist_cur2 - stray_chain_dist[0]) > 240) &&
                (stray_chain_cnt[1] < stray_Param.stray_cnt_th ||
                        std::abs(dist_cur2 - stray_chain_dist[1]) > 240);

            bool wave1_sel_frame =
                (RainWall_in.dist == 0) ||
                (dist_cur2 > RainWall_in.dist + 200) ||
                (dist_cur2 < RainWall_in.dist - 960);

            bool wave1_sel_en = (wave1_sel_base && wave1_sel_column && wave1_sel_frame) || gnd_cur2;

            // 设置输出掩码
            // if (stray_del_en) {
            //     stray_mark_out0[row_idx] = wave1_sel_en ? 2 : 3;
            // } else if (!wave1_sel_frame) {
            //     stray_mark_out0[row_idx] = 1;
            // }
            if(stray_del_en){
                if(wave1_sel_en){
                    stray_mark_out0[row_idx] = 1;
                }
                else{
                    stray_mark_out0[row_idx] = 1;
                    stray_mark_out1[row_idx] = 1;
                }
            }
            else{
                if(!wave1_sel_frame){
                    stray_mark_out1[row_idx] = 1;
                }
            }
        }
    }
}

/**
 * @brief rain enhance algorithm of lidar
 *
 * @param spray_col column index of the buffer
 *                  Range: 0 - 759. Accuracy: 1.
 * @param spray_col_buffer spray buffer column index
 *                  Range: 0 - 6. Accuracy: 1.
 * @param spray_col_neib_buf spray neighbor buffer column index
 *                  Range: 0 - 6. Accuracy: 1.
 * @param spray_mark_out0 spray wave0 buffer mark value
 *                  Range: 0 - 1. Accuracy: 1.
 * @param spray_mark_out1 spray wave0 buffer mark value
 *                  Range: 0 - 1. Accuracy: 1.
 */
void AlgoFunction::rainEnhance(int spray_col,
    int spray_col_buffer,
    int* spray_col_neib_buf,
    int* spray_mark_out0,
    int* spray_mark_out1) {
    if ((spray_col >= 0) && (spray_col < VIEW_W)) {
        bool MidAreaMask = std::floor((spray_col + 1) / 50) == 7;
        int rowStep = (MidAreaMask) ? 1 : 2;
        int rowEnd = MidAreaMask - 1;
        if (algo_Param.SprayRemoveOn) {
            // 预计算行范围
            for (int row_idx = 0; row_idx < VIEW_H + rowEnd; row_idx = row_idx + rowStep) {
                int row_range_up = spray_Param.Area_size_y;//bias
                int row_range_down = spray_Param.Area_size_y;//bias
                if (row_idx <= spray_Param.Area_size_y - 1)
                {
                    row_range_up = row_idx;
                }
                if (row_idx >= VIEW_H - spray_Param.Area_size_y)
                {
                    row_range_down = VIEW_H - 1 - row_idx;
                }
                int local_dist_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_ref_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_mark_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_head_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_tail_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_dist_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_ref_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_mark_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_head_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_tail_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };

                // 第一回波数据
                int cur_dist_1 = dist_wave0_buffer4[spray_col_buffer][row_idx];
                int cur_dist_2 = dist_wave1_buffer4[spray_col_buffer][row_idx];
                /******************接地保护*******************/
                int zone_idx = (row_idx + 1) / 12;
                if ((row_idx + 1) % 12 == 0) //12个通道一个采样
                {
                    spray_Param.dist_cap_zone[zone_idx][0] = cur_dist_1;
                    spray_Param.dist_cap_zone[zone_idx][1] = cur_dist_2;

                    //接地判别
                    if (zone_idx > 0 && spray_Param.ground_cap_zone[0] &&
                        ((abs(spray_Param.dist_cap_zone[zone_idx][0] - spray_Param.dist_cap_zone[zone_idx - 1][0]) <= spray_Param.dist_diff_thr_fix &&
                            spray_Param.dist_cap_zone[zone_idx - 1][0]) ||
                            (abs(spray_Param.dist_cap_zone[zone_idx][0] - spray_Param.dist_cap_zone[zone_idx - 1][1]) <= spray_Param.dist_diff_thr_fix &&
                                spray_Param.dist_cap_zone[zone_idx - 1][1])))
                    {
                        spray_Param.ground_cap_zone[0] = 1;
                    }
                    else
                    {
                        spray_Param.ground_cap_zone[0] = 0;
                    }
                    if (zone_idx > 0 && spray_Param.ground_cap_zone[1] &&
                        ((abs(spray_Param.dist_cap_zone[zone_idx][1] - spray_Param.dist_cap_zone[zone_idx - 1][0]) <= spray_Param.dist_diff_thr_fix && spray_Param.dist_cap_zone[zone_idx - 1][0]) ||
                            (abs(spray_Param.dist_cap_zone[zone_idx][1] - spray_Param.dist_cap_zone[zone_idx - 1][1]) <= spray_Param.dist_diff_thr_fix && spray_Param.dist_cap_zone[zone_idx - 1][1])))
                    {
                        spray_Param.ground_cap_zone[1] = 1;
                    }
                    else
                    {
                        spray_Param.ground_cap_zone[1] = 0;
                    }
                }
                //data same
                int local_dist_A[2][RAIN_SIZE_Y];
                int local_dist_B[2][RAIN_SIZE_Y];
                int local_dist_C[2][RAIN_SIZE_Y];
                int local_dist_D[2][RAIN_SIZE_Y];
                int local_dist_E[2][RAIN_SIZE_Y];
                int* local_dist_a;
                int* local_dist_b;
                int* local_dist_c;
				int MmOffsetLen[3] = { 0 };
                int dist_diff_col[RAIN_SIZE_Y];
                int dist_diff_row[RAIN_SIZE_Y];
                int dist_diff_thr;
                int dist_diff_diff_thr;
                bool cond3;
                bool cond4;
                bool cond_glink_0;
                bool cond_glink_1;
                bool cond_glink_2;
                bool cond_glink_3;
                bool cond6;
                bool local_ref_mask_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                bool local_ref_mask_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
                int local_data[5][RAIN_SIZE_X][RAIN_SIZE_Y * 2] = { 0 };
                int* local_ref_A; // 第0行 -> A
                int* local_ref_B; // 第1行 -> B
                int* local_ref_C; // 第2行 -> C
                int* local_ref_D; // 第3行 -> D
                int* local_ref_E; // 第4行 -> E
                int* local_ref_mask_A; // 第0行 -> A
                int* local_ref_mask_B; // 第1行 -> B
                int* local_ref_mask_C; // 第2行 -> C
                int* local_ref_mask_D; // 第3行 -> D
                int* local_ref_mask_E; // 第4行 -> E

                int* local_mark_A; // 第0行 -> A
                int* local_mark_B; // 第1行 -> B
                int* local_mark_C; // 第2行 -> C
                int* local_mark_D; // 第3行 -> D
                int* local_mark_E; // 第4行 -> E

                int* local_head_A; // 第0行 -> A
                int* local_head_B; // 第1行 -> B
                int* local_head_C; // 第2行 -> C
                int* local_head_D; // 第3行 -> D
                int* local_head_E; // 第4行 -> E

                int* local_tail_A; // 第0行 -> A
                int* local_tail_B; // 第1行 -> B
                int* local_tail_C; // 第2行 -> C
                int* local_tail_D; // 第3行 -> D
                int* local_tail_E; // 第4行 -> E
                bool cond0_0;
                bool cond0_1;
                bool cond1;
                bool cond0_2;
                bool cond0_3;

                bool cond2;
                int* local_ref_mask_a;
                int* local_ref_mask_b;
                int* local_ref_mask_c;
                auto sign = [&](int x) {
                    return (x > 0) - (x < 0);
                };
                //雨雾标记读取 第一回波
                uint8_t cur_mark_1 = rain_wave0_buffer4[spray_col_buffer][row_idx];// 第一回波
                bool cond_self_01 = !cur_mark_1;
                if (cond_self_01)
                {
                    int cur_ref_1 = refl_wave0_buffer4[spray_col_buffer][row_idx];
                    bool cond_self_1 = cur_dist_1 && (cur_dist_1 > spray_Param.Dist_threshold_min) &&
                        (cur_dist_1 <= spray_Param.Dist_threshold2) && (cur_ref_1 <= spray_Param.Ref_threshold);
                    if (cond_self_1)
                    {
                        bool cur_ground_1 = grnd_wave0_buffer4[spray_col_buffer][row_idx];
                        /**********neibor statistics****************/
                        for (int i = 0; i < RAIN_SIZE_X; i++)
                        {
                            for (int j = -row_range_up; j <= row_range_down; j++)
                            {
                                local_dist_1[i][spray_Param.Area_size_y + j] = dist_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                local_dist_2[i][spray_Param.Area_size_y + j] = dist_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                            }
                        }

                        for (int i = 0; i < RAIN_SIZE_Y; i++)
                        {
                            local_dist_A[0][i] = local_dist_1[0][i];
                            local_dist_B[0][i] = local_dist_1[1][i];
                            local_dist_C[0][i] = local_dist_1[2][i];
                            local_dist_D[0][i] = local_dist_1[3][i];
                            local_dist_E[0][i] = local_dist_1[4][i];
                            local_dist_A[1][i] = local_dist_2[0][i];
                            local_dist_B[1][i] = local_dist_2[1][i];
                            local_dist_C[1][i] = local_dist_2[2][i];
                            local_dist_D[1][i] = local_dist_2[3][i];
                            local_dist_E[1][i] = local_dist_2[4][i];
                        }

                        /********************组合计算及判断*********************/
                        //gradient
                        if (cur_dist_1 < spray_Param.Dist_threshold)// Secondary segment, reduce the resolution by half, 8m
                        {
                            local_dist_a = &local_dist_A[0][0];
                            local_dist_b = &local_dist_C[0][0];
                            local_dist_c = &local_dist_E[0][0];
                            MmOffsetLen[0] = 0;
                            MmOffsetLen[1] = 2;
                            MmOffsetLen[2] = 4;
                        }
                        else
                        {
                            local_dist_a = &local_dist_B[0][0];
                            local_dist_b = &local_dist_C[0][0];
                            local_dist_c = &local_dist_D[0][0];
                            MmOffsetLen[0] = 1;
                            MmOffsetLen[1] = 2;
                            MmOffsetLen[2] = 3;
                        }

                        dist_diff_col[0] = local_dist_b[3] - local_dist_a[3];
                        dist_diff_col[1] = local_dist_c[3] - local_dist_b[3];
                        dist_diff_col[2] = local_dist_c[3] - local_dist_a[3];

                        dist_diff_row[0] = local_dist_b[3] - local_dist_b[0];
                        dist_diff_row[1] = local_dist_b[1] - local_dist_b[3];
                        dist_diff_row[2] = local_dist_b[1] - local_dist_b[0];
                        dist_diff_thr = spray_Param.dist_diff_thr_seq[(cur_dist_1) >> 9];
                        dist_diff_diff_thr = spray_Param.dist_diff_diff_thr_seq[cur_dist_1 >> 9];

                        //梯度一致性判断
                        int consistency_col_1 = 1;
                        if ((sign(dist_diff_col[0]) == sign(dist_diff_col[1]) &&
                            abs(dist_diff_col[0]) <= dist_diff_thr && abs(dist_diff_col[1]) <= dist_diff_thr &&
                            abs(abs(dist_diff_col[0]) - abs(dist_diff_col[1])) <= dist_diff_diff_thr) ||
                            (abs(dist_diff_col[0]) <= 30 && abs(dist_diff_col[1]) <= 30))
                        {
                            consistency_col_1 = 1;
                        }
                        else
                        {
                            consistency_col_1 = 0;
                        }

                        int consistency_row_1 = 1;
                        if ((sign(dist_diff_row[0]) == sign(dist_diff_row[1]) &&
                            abs(dist_diff_row[0]) <= dist_diff_thr && abs(dist_diff_row[1]) <= dist_diff_thr &&
                            abs(abs(dist_diff_row[0]) - abs(dist_diff_row[1])) <= dist_diff_diff_thr) ||
                            (abs(dist_diff_row[0]) <= 30 && abs(dist_diff_row[1]) <= 30))
                        {
                            consistency_row_1 = 1;
                        }
                        else
                        {
                            consistency_row_1 = 0;
                        }
                        // 梯度计算
                        cond3 = spray_Param.consistency_en && (consistency_col_1 || consistency_row_1);

                        // 地面标记
                        cond4 = spray_Param.ground_judge_en && cur_ground_1;
                        if ((!cond3) && !cond4)
                        {
                            if (MidAreaMask)
                            {
                                rain_wave0_buffer4[spray_col_buffer][row_idx] = 1;
                                rain_final0_buffer[spray_col_buffer][row_idx] = 1;
                            }
                            else
                            {
                                for (int i = 0; i < 3; i++)
                                {
                                    rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                    rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                    rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                    rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                }
                            }
                        }
                        else
                        {
                            /*****悬空计算**/
                            bool ground_link_valid_1 = 0;
                            cond_glink_0 = !cur_ground_1 && cur_ref_1 <= spray_Param.Ref_threshold2;
                            cond_glink_1 = ((cur_dist_1 <= spray_Param.Dist_threshold && zone_idx > spray_Param.zone_idx_min_a && zone_idx < spray_Param.zone_idx_max_a) ||
                                (cur_dist_1 > spray_Param.Dist_threshold && cur_dist_1 < spray_Param.Dist_threshold2&& zone_idx > spray_Param.zone_idx_min_b - 1 && zone_idx < spray_Param.zone_idx_max_b));
                            cond_glink_2 = zone_idx > 0 && ((abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx - 1][0]) > spray_Param.dist_diff_thr_fix && abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx - 1][1]) > spray_Param.dist_diff_thr_fix) || !spray_Param.ground_cap_zone[0]);
                            cond_glink_3 = zone_idx > 0 && ((abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx - 1][0]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[0]) ||
                                (abs(cur_dist_1 - spray_Param.dist_cap_zone[zone_idx - 1][1]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[1]));
                            if (cond_glink_0 && cond_glink_1 && cond_glink_2 && !cond_glink_3)
                            {
                                ground_link_valid_1 = 1;
                            }

                            //接地判别
                            cond6 = spray_Param.ground_link_judge_en && ground_link_valid_1;

                            if (cond6 && !cond3)
                            {
                                if (MidAreaMask)
                                {
                                    rain_wave0_buffer4[spray_col_buffer][row_idx] = 1;
                                    rain_final0_buffer[spray_col_buffer][row_idx] = 1;
                                }
                                else
                                {
                                    for (int i = 0; i < 3; i++)
                                    {
                                        rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                        rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                        rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                        rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                    }
                                }
                            }
                            else
                            {
                                if (!cond4)
                                {
                                    for (int i = 0; i < RAIN_SIZE_X; i++)
                                    {
                                        for (int j = -row_range_up; j <= row_range_down; j++)
                                        {
                                            local_ref_1[i][j + spray_Param.Area_size_y] = refl_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_mark_1[i][j + spray_Param.Area_size_y] = rain_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_head_1[i][j + spray_Param.Area_size_y] = head_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_tail_1[i][j + spray_Param.Area_size_y] = tail_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];

                                            local_ref_2[i][j + spray_Param.Area_size_y] = refl_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_mark_2[i][j + spray_Param.Area_size_y] = rain_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_head_2[i][j + spray_Param.Area_size_y] = head_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_tail_2[i][j + spray_Param.Area_size_y] = tail_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                        }
                                        for (int k = 0; k < RAIN_SIZE_Y; k++)
                                        {
                                            local_ref_mask_1[i][k] = local_ref_1[i][k] <= spray_Param.Ref_threshold;
                                            local_ref_mask_2[i][k] = local_ref_2[i][k] <= spray_Param.Ref_threshold;
                                        }
                                    }
                                    memset(local_data, 0, sizeof(local_data));
                                    // 填充局部数据

                                    for (int i = 0; i < RAIN_SIZE_X; i++)
                                    {
                                        for (int j = 0; j < RAIN_SIZE_Y; j++) {
                                            int wave2_j = j + RAIN_SIZE_Y;
                                            local_data[0][i][j] = local_ref_1[i][j];
                                            local_data[1][i][j] = local_ref_mask_1[i][j];
                                            local_data[2][i][j] = local_mark_1[i][j];
                                            local_data[3][i][j] = local_head_1[i][j];
                                            local_data[4][i][j] = local_tail_1[i][j];
                                            local_data[0][i][wave2_j] = local_ref_2[i][j];
                                            local_data[1][i][wave2_j] = local_ref_mask_2[i][j];
                                            local_data[2][i][wave2_j] = local_mark_2[i][j];
                                            local_data[3][i][wave2_j] = local_head_2[i][j];
                                            local_data[4][i][wave2_j] = local_tail_2[i][j];
                                        }
                                    }

                                    local_ref_A = local_data[0][0]; // 第0行 -> A
                                    local_ref_B = local_data[0][1]; // 第1行 -> B
                                    local_ref_C = local_data[0][2]; // 第2行 -> C
                                    local_ref_D = local_data[0][3]; // 第3行 -> D
                                    local_ref_E = local_data[0][4]; // 第4行 -> E

                                    local_ref_mask_A = local_data[1][0]; // 第0行 -> A
                                    local_ref_mask_B = local_data[1][1]; // 第1行 -> B
                                    local_ref_mask_C = local_data[1][2]; // 第2行 -> C
                                    local_ref_mask_D = local_data[1][3]; // 第3行 -> D
                                    local_ref_mask_E = local_data[1][4]; // 第4行 -> E

                                    local_mark_A = local_data[2][0]; // 第0行 -> A
                                    local_mark_B = local_data[2][1]; // 第1行 -> B
                                    local_mark_C = local_data[2][2]; // 第2行 -> C
                                    local_mark_D = local_data[2][3]; // 第3行 -> D
                                    local_mark_E = local_data[2][4]; // 第4行 -> E

                                    local_head_A = local_data[3][0]; // 第0行 -> A
                                    local_head_B = local_data[3][1]; // 第1行 -> B
                                    local_head_C = local_data[3][2]; // 第2行 -> C
                                    local_head_D = local_data[3][3]; // 第3行 -> D
                                    local_head_E = local_data[3][4]; // 第4行 -> E

                                    local_tail_A = local_data[4][0]; // 第0行 -> A
                                    local_tail_B = local_data[4][1]; // 第1行 -> B
                                    local_tail_C = local_data[4][2]; // 第2行 -> C
                                    local_tail_D = local_data[4][3]; // 第3行 -> D
                                    local_tail_E = local_data[4][4]; // 第4行 -> E
                                    //A列6回波
                                        // 2. 统计各类标记数量
                                    int mark_num[5] = { 0 }, head_num[5] = { 0 }, tail_num[5] = { 0 };
                                    // const int dist_thr = spray_Param.dist_diff_thr_2;

                                        // 辅助lambda函数减少重复代码
                                    auto countStats = [&](const int* dist, const int* ref_mask, const int* mark,
                                        const int* head, const int* tail, int count, int idx) {
                                            for (int i = 0; i < count; i++) {
                                                if (idx == 2 && i == 3) continue;
                                                if ((dist[i] != 0) && (cur_dist_1 != 0) && (ref_mask[i] != 0) && (std::abs(dist[i] - cur_dist_1) <= spray_Param.dist_diff_thr_2)) {
                                                    if (mark[i]) mark_num[idx]++;
                                                    if (head[i]) head_num[idx]++;
                                                    if (tail[i]) tail_num[idx]++;
                                                }
                                            }
                                    };

                                    // 统计A行（6个元素）
                                    countStats(&local_dist_A[0][0], local_ref_mask_A, local_mark_A, local_head_A, local_tail_A, 6, 0);

                                    // 统计B行（3个元素）
                                    countStats(&local_dist_B[0][0], local_ref_mask_B, local_mark_B, local_head_B, local_tail_B, 6, 1);

                                    // 统计C行（特殊处理：排除i=1位置）
                                    countStats(&local_dist_C[0][0], local_ref_mask_C, local_mark_C, local_head_C, local_tail_C, 6, 2);

                                    // 统计D行（3个元素）
                                    countStats(&local_dist_D[0][0], local_ref_mask_D, local_mark_D, local_head_D, local_tail_D, 6, 3);

                                    // 统计E行（3个元素）
                                    countStats(&local_dist_E[0][0], local_ref_mask_E, local_mark_E, local_head_E, local_tail_E, 6, 4);

                                    // 总和计算
                                    int sum_mark_num_1 = mark_num[0] + mark_num[1] + mark_num[2] + mark_num[3] + mark_num[4];
                                    int sum_head_num_1 = head_num[0] + head_num[1] + head_num[2] + head_num[3] + head_num[4];
                                    int sum_tail_num_1 = tail_num[0] + tail_num[1] + tail_num[2] + tail_num[3] + tail_num[4];

                                    bool cur_head_1 = head_wave0_buffer4[spray_col_buffer][row_idx];
                                    bool cur_tail_1 = tail_wave0_buffer4[spray_col_buffer][row_idx];
                                    // 459连波标记
                                    bool head_tail_cur_1 = cond_self_1 & cur_head_1 & cur_tail_1;
                                    //拆波标记
                                    cond0_0 = spray_Param.head_tail_judge_en && head_tail_cur_1;
                                    cond0_1 = spray_Param.head_tail_judge_en && (sum_tail_num_1 >= spray_Param.min_tail_threshold ||
                                        sum_head_num_1 + sum_tail_num_1 >= spray_Param.min_split_threshold);
                                    //雨雾标记
                                    cond1 = spray_Param.rain_mark_en && sum_mark_num_1 >= spray_Param.min_mark_threshold;
                                    cond0_3 = spray_Param.head_tail_judge_en && cur_head_1 && !cur_tail_1 && cur_dist_1 > 1000 && (cond0_1 || cond1);

                                    if (cond0_3 || cond0_0)
                                    {
                                        if (MidAreaMask)
                                        {
                                            rain_wave0_buffer4[spray_col_buffer][row_idx] = 1;
                                            rain_final0_buffer[spray_col_buffer][row_idx] = 1;
                                        }
                                        else {
                                            for (int i = 0; i < 3; i++)
                                            {
                                                rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                                rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (!cond3)
                                        {
                                            //方差计算，3x3

                                            if (cur_dist_1 < spray_Param.Dist_threshold)// 次发段，分辨率放一半
                                            {
                                                local_dist_a = &local_dist_A[0][0];
                                                local_ref_mask_a = local_ref_mask_A;
                                                local_dist_b = &local_dist_C[0][0];
                                                local_ref_mask_b = local_ref_mask_C;
                                                local_dist_c = &local_dist_E[0][0];
                                                local_ref_mask_c = local_ref_mask_E;
                                                MmOffsetLen[0] = 0;
                                                MmOffsetLen[1] = 2;
                                                MmOffsetLen[2] = 4;
                                            }
                                            else
                                            {
                                                local_dist_a = &local_dist_B[0][0];
                                                local_ref_mask_a = local_ref_mask_B;
                                                local_dist_b = &local_dist_C[0][0];
                                                local_ref_mask_b = local_ref_mask_C;
                                                local_dist_c = &local_dist_D[0][0];
                                                local_ref_mask_c = local_ref_mask_D;
                                                MmOffsetLen[0] = 1;
                                                MmOffsetLen[1] = 2;
                                                MmOffsetLen[2] = 3;
                                            }
                                            //第一回波
                                            int sum_count1[3] = { 0 }, sum_dist1[3] = { 0 }, sum_sq1[3] = { 0 };
                                            auto variCalc = [&](const int* dist, const int* ref_mask, const int count, const int idx) {
                                                for (int i = 0; i < count; i++)
                                                {
                                                    if (idx == 1 && (i == 3 || i == 2))continue;
                                                    if (dist[i] != 0 && ref_mask[i] != 0 && abs(dist[i] - cur_dist_1) < 3 * 200)
                                                    {
                                                        sum_count1[idx]++;
                                                        sum_dist1[idx]++;
                                                        sum_sq1[idx]++;
                                                    }
                                                }
                                            };

                                            variCalc(local_dist_a, local_ref_mask_a, 6, 0);
                                            variCalc(local_dist_b, local_ref_mask_b, 6, 1);
                                            variCalc(local_dist_c, local_ref_mask_c, 6, 2);

                                            int count_var_1 = sum_count1[0] + sum_count1[1] + sum_count1[2];
                                            int sum_dist_1 = sum_dist1[0] + sum_dist1[1] + sum_dist1[2];
                                            int sum_sq_1 = sum_sq1[0] + sum_sq1[1] + sum_sq1[2];
                                            int inv_n_1 = 0;
                                            if (count_var_1 != 0)
                                            {
                                                inv_n_1 = spray_Param.inv_table[count_var_1 - 1];
                                            }
                                            else
                                            {
                                                inv_n_1 = 0;
                                            }
                                            int mu_1 = (sum_dist_1 * inv_n_1) >> 12;
                                            int local_var_1 = ((sum_sq_1 - sum_dist_1 * mu_1) * inv_n_1) >> 12;

                                            //方差计算
                                            cond0_2 = spray_Param.head_tail_judge_en && !cur_head_1 && cur_tail_1 && cur_dist_1 > 1000 && (cond0_1 || cond1);
                                            cond2 = spray_Param.local_var_en && ((local_var_1 >= spray_Param.threshold_var && row_idx > 95) ||
                                                (local_var_1 >= spray_Param.threshold_var2 && row_idx <= 95));
                                            if (cond0_1 || cond1 || cond2 || cond0_2) {
                                                if (MidAreaMask)
                                                {
                                                    rain_wave0_buffer4[spray_col_buffer][row_idx] = 1;
                                                    rain_final0_buffer[spray_col_buffer][row_idx] = 1;
                                                }
                                                else
                                                {
                                                    for (int i = 0; i < 3; i++)
                                                    {
                                                        rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                        rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                        rain_wave0_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                                        rain_final0_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                                    }
                                                }
                                            }

                                        }
                                    }

                                }
                            }
                        }
                    }
                }
                //雨雾标记读取 第二回波
                uint8_t cur_mark_2 = rain_wave1_buffer4[spray_col_buffer][row_idx];// secondary wave
                bool cond_self_02 = !cur_mark_2;
                if (cond_self_02)
                {
                    int cur_ref_2 = refl_wave1_buffer4[spray_col_buffer][row_idx];
                    bool cond_self_2 = cur_dist_2 && (cur_dist_2 > spray_Param.Dist_threshold_min) &&
                        (cur_dist_2 <= spray_Param.Dist_threshold2) && (cur_ref_2 <= spray_Param.Ref_threshold);
                    if (cond_self_2)
                    {
                        bool cur_ground_2 = grnd_wave1_buffer4[spray_col_buffer][row_idx];
                        /**********neibor statistics****************/
                        for (int i = 0; i < RAIN_SIZE_X; i++)
                        {
                            for (int j = -row_range_up; j <= row_range_down; j++)
                            {
                                local_dist_1[i][spray_Param.Area_size_y + j] = dist_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                local_dist_2[i][spray_Param.Area_size_y + j] = dist_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                            }
                        }

                        for (int i = 0; i < RAIN_SIZE_Y; i++)
                        {
                            local_dist_A[0][i] = local_dist_1[0][i];
                            local_dist_B[0][i] = local_dist_1[1][i];
                            local_dist_C[0][i] = local_dist_1[2][i];
                            local_dist_D[0][i] = local_dist_1[3][i];
                            local_dist_E[0][i] = local_dist_1[4][i];
                            local_dist_A[1][i] = local_dist_2[0][i];
                            local_dist_B[1][i] = local_dist_2[1][i];
                            local_dist_C[1][i] = local_dist_2[2][i];
                            local_dist_D[1][i] = local_dist_2[3][i];
                            local_dist_E[1][i] = local_dist_2[4][i];
                        }

                        /********************组合计算及判断*********************/
                        //gradient
                        if (cur_dist_2 < spray_Param.Dist_threshold)// Secondary segment, reduce the resolution by half, 8m
                        {
                            local_dist_a = &local_dist_A[0][0];
                            local_dist_b = &local_dist_C[0][0];
                            local_dist_c = &local_dist_E[0][0];
                            MmOffsetLen[0] = 0;
                            MmOffsetLen[1] = 2;
                            MmOffsetLen[2] = 4;
                        }
                        else
                        {
                            local_dist_a = &local_dist_B[0][0];
                            local_dist_b = &local_dist_C[0][0];
                            local_dist_c = &local_dist_D[0][0];
                            MmOffsetLen[0] = 1;
                            MmOffsetLen[1] = 2;
                            MmOffsetLen[2] = 3;
                        }

                        dist_diff_col[0] = local_dist_b[2] - local_dist_a[2];
                        dist_diff_col[1] = local_dist_c[2] - local_dist_b[2];
                        dist_diff_col[2] = local_dist_c[2] - local_dist_a[2];

                        dist_diff_row[0] = local_dist_b[2] - local_dist_b[4];
                        dist_diff_row[1] = local_dist_b[5] - local_dist_b[2];
                        dist_diff_row[2] = local_dist_b[5] - local_dist_b[4];
                        dist_diff_thr = spray_Param.dist_diff_thr_seq[(cur_dist_2) >> 9];
                        dist_diff_diff_thr = spray_Param.dist_diff_diff_thr_seq[cur_dist_2 >> 9];

                        //梯度一致性判断
                        int consistency_col_2 = 1;
                        if ((sign(dist_diff_col[0]) == sign(dist_diff_col[1]) &&
                            abs(dist_diff_col[0]) <= dist_diff_thr && abs(dist_diff_col[1]) <= dist_diff_thr &&
                            abs(abs(dist_diff_col[0]) - abs(dist_diff_col[1])) <= dist_diff_diff_thr) ||
                            (abs(dist_diff_col[0]) <= 30 && abs(dist_diff_col[1]) <= 30))
                        {
                            consistency_col_2 = 1;
                        }
                        else
                        {
                            consistency_col_2 = 0;
                        }

                        int consistency_row_2 = 1;
                        if ((sign(dist_diff_row[0]) == sign(dist_diff_row[1]) &&
                            abs(dist_diff_row[0]) <= dist_diff_thr && abs(dist_diff_row[1]) <= dist_diff_thr &&
                            abs(abs(dist_diff_row[0]) - abs(dist_diff_row[1])) <= dist_diff_diff_thr) ||
                            (abs(dist_diff_row[0]) <= 30 && abs(dist_diff_row[1]) <= 30))
                        {
                            consistency_row_2 = 1;
                        }
                        else
                        {
                            consistency_row_2 = 0;
                        }
                        // 梯度计算
                        cond3 = spray_Param.consistency_en && (consistency_col_2 || consistency_row_2);

                        // 地面标记
                        cond4 = spray_Param.ground_judge_en && cur_ground_2;
                        if ((!cond3) && !cond4)
                        {
                            if (MidAreaMask)
                            {
                                rain_wave1_buffer4[spray_col_buffer][row_idx] = 1;
                                rain_final1_buffer[spray_col_buffer][row_idx] = 1;
                            }
                            else
                            {
                                for (int i = 0; i < 3; i++)
                                {
                                    rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                    rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                    rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                    rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                }
                            }
                        }
                        else
                        {
                            /*****悬空计算**/
                            bool ground_link_valid_2 = 0;
                            cond_glink_0 = !cur_ground_2 && cur_ref_2 <= spray_Param.Ref_threshold2;
                            cond_glink_1 = ((cur_dist_2 <= spray_Param.Dist_threshold && zone_idx > spray_Param.zone_idx_min_a && zone_idx < spray_Param.zone_idx_max_a) ||
                                (cur_dist_2 > spray_Param.Dist_threshold && cur_dist_2 < spray_Param.Dist_threshold2&& zone_idx > spray_Param.zone_idx_min_b && zone_idx < spray_Param.zone_idx_max_b));
                            cond_glink_2 = zone_idx > 0 && ((abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx - 1][0]) > spray_Param.dist_diff_thr_fix && abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx - 1][1]) > spray_Param.dist_diff_thr_fix) || !spray_Param.ground_cap_zone[0]);
                            cond_glink_3 = zone_idx > 0 && ((abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx - 1][0]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[0]) ||
                                (abs(cur_dist_2 - spray_Param.dist_cap_zone[zone_idx - 1][1]) <= spray_Param.dist_diff_thr_fix && spray_Param.ground_cap_zone[1]));
                            if (cond_glink_0 && cond_glink_1 && cond_glink_2 && !cond_glink_3)
                            {
                                ground_link_valid_2 = 1;
                            }

                            //接地判别
                            cond6 = spray_Param.ground_link_judge_en && ground_link_valid_2;
                            if (cond6 && !cond3)
                            {
                                if (MidAreaMask)
                                {
                                    rain_wave1_buffer4[spray_col_buffer][row_idx] = 1;
                                    rain_final1_buffer[spray_col_buffer][row_idx] = 1;
                                }
                                else
                                {
                                    for (int i = 0; i < 3; i++)
                                    {
                                        rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                        rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                        rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                        rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                    }
                                }
                            }
                            else
                            {
                                if (!cond4)
                                {
                                    for (int i = 0; i < RAIN_SIZE_X; i++)
                                    {
                                        for (int j = -row_range_up; j <= row_range_down; j++)
                                        {
                                            local_ref_1[i][j + spray_Param.Area_size_y] = refl_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_mark_1[i][j + spray_Param.Area_size_y] = rain_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_head_1[i][j + spray_Param.Area_size_y] = head_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_tail_1[i][j + spray_Param.Area_size_y] = tail_wave0_buffer4[spray_col_neib_buf[i]][row_idx + j];

                                            local_ref_2[i][j + spray_Param.Area_size_y] = refl_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_mark_2[i][j + spray_Param.Area_size_y] = rain_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_head_2[i][j + spray_Param.Area_size_y] = head_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                            local_tail_2[i][j + spray_Param.Area_size_y] = tail_wave1_buffer4[spray_col_neib_buf[i]][row_idx + j];
                                        }
                                        for (int k = 0; k < RAIN_SIZE_Y; k++)
                                        {
                                            local_ref_mask_1[i][k] = local_ref_1[i][k] <= spray_Param.Ref_threshold;
                                            local_ref_mask_2[i][k] = local_ref_2[i][k] <= spray_Param.Ref_threshold;
                                        }
                                    }

                                    memset(local_data, 0, sizeof(local_data));
                                    // 填充局部数据

                                    for (int i = 0; i < RAIN_SIZE_X; i++)
                                    {
                                        for (int j = 0; j < (row_range_up + row_range_down + 1); j++) {
                                            int wave2_j = j + RAIN_SIZE_Y;
                                            local_data[0][i][j] = local_ref_1[i][j];
                                            local_data[1][i][j] = local_ref_mask_1[i][j];
                                            local_data[2][i][j] = local_mark_1[i][j];
                                            local_data[3][i][j] = local_head_1[i][j];
                                            local_data[4][i][j] = local_tail_1[i][j];
                                            local_data[0][i][wave2_j] = local_ref_2[i][j];
                                            local_data[1][i][wave2_j] = local_ref_mask_2[i][j];
                                            local_data[2][i][wave2_j] = local_mark_2[i][j];
                                            local_data[3][i][wave2_j] = local_head_2[i][j];
                                            local_data[4][i][wave2_j] = local_tail_2[i][j];
                                        }
                                    }

                                    local_ref_A = local_data[0][0]; // 第0行 -> A
                                    local_ref_B = local_data[0][1]; // 第1行 -> B
                                    local_ref_C = local_data[0][2]; // 第2行 -> C
                                    local_ref_D = local_data[0][3]; // 第3行 -> D
                                    local_ref_E = local_data[0][4]; // 第4行 -> E

                                    local_ref_mask_A = local_data[1][0]; // 第0行 -> A
                                    local_ref_mask_B = local_data[1][1]; // 第1行 -> B
                                    local_ref_mask_C = local_data[1][2]; // 第2行 -> C
                                    local_ref_mask_D = local_data[1][3]; // 第3行 -> D
                                    local_ref_mask_E = local_data[1][4]; // 第4行 -> E

                                    local_mark_A = local_data[2][0]; // 第0行 -> A
                                    local_mark_B = local_data[2][1]; // 第1行 -> B
                                    local_mark_C = local_data[2][2]; // 第2行 -> C
                                    local_mark_D = local_data[2][3]; // 第3行 -> D
                                    local_mark_E = local_data[2][4]; // 第4行 -> E

                                    local_head_A = local_data[3][0]; // 第0行 -> A
                                    local_head_B = local_data[3][1]; // 第1行 -> B
                                    local_head_C = local_data[3][2]; // 第2行 -> C
                                    local_head_D = local_data[3][3]; // 第3行 -> D
                                    local_head_E = local_data[3][4]; // 第4行 -> E

                                    local_tail_A = local_data[4][0]; // 第0行 -> A
                                    local_tail_B = local_data[4][1]; // 第1行 -> B
                                    local_tail_C = local_data[4][2]; // 第2行 -> C
                                    local_tail_D = local_data[4][3]; // 第3行 -> D
                                    local_tail_E = local_data[4][4]; // 第4行 -> E
                                    //A列6回波
                                        // 2. 统计各类标记数量
                                    int mark_num_2[5] = { 0 }, head_num_2[5] = { 0 }, tail_num_2[5] = { 0 };

                                    // 辅助lambda函数减少重复代码
                                    auto countStats2 = [&](const int* dist, const int* ref_mask, const int* mark,
                                        const int* head, const int* tail, int count, int idx) {
                                            for (int i = 0; i < count; i++) {
                                                if (idx == 2 && i == 3) continue;
                                                if ((dist[i] != 0) && (cur_dist_2 != 0) && (ref_mask[i] != 0) && (std::abs(dist[i] - cur_dist_2) <= spray_Param.dist_diff_thr_2)) {
                                                    if (mark[i]) mark_num_2[idx]++;
                                                    if (head[i]) head_num_2[idx]++;
                                                    if (tail[i]) tail_num_2[idx]++;
                                                }
                                            }
                                    };

                                    // 统计A行（6个元素）
                                    countStats2(&local_dist_A[0][0], local_ref_mask_A, local_mark_A, local_head_A, local_tail_A, 6, 0);

                                    // 统计B行（6个元素）
                                    countStats2(&local_dist_B[0][0], local_ref_mask_B, local_mark_B, local_head_B, local_tail_B, 6, 1);

                                    // 统计C行（特殊处理：排除i=1位置）
                                    countStats2(&local_dist_C[0][0], local_ref_mask_C, local_mark_C, local_head_C, local_tail_C, 6, 2);

                                    // 统计D行（6个元素）
                                    countStats2(&local_dist_D[0][0], local_ref_mask_D, local_mark_D, local_head_D, local_tail_D, 6, 3);

                                    // 统计E行（6个元素）
                                    countStats2(&local_dist_E[0][0], local_ref_mask_E, local_mark_E, local_head_E, local_tail_E, 6, 4);

                                    // 总和计算
                                    int sum_mark_num_2 = mark_num_2[0] + mark_num_2[1] + mark_num_2[2] + mark_num_2[3] + mark_num_2[4];
                                    int sum_head_num_2 = head_num_2[0] + head_num_2[1] + head_num_2[2] + head_num_2[3] + head_num_2[4];
                                    int sum_tail_num_2 = tail_num_2[0] + tail_num_2[1] + tail_num_2[2] + tail_num_2[3] + tail_num_2[4];

                                    bool cur_head_2 = head_wave1_buffer4[spray_col_buffer][row_idx];
                                    bool cur_tail_2 = tail_wave1_buffer4[spray_col_buffer][row_idx];
                                    // 459连波标记
                                    bool head_tail_cur_2 = cond_self_2 & cur_head_2 & cur_tail_2;
                                    //拆波标记
                                    cond0_0 = spray_Param.head_tail_judge_en && head_tail_cur_2;
                                    cond0_1 = spray_Param.head_tail_judge_en && (sum_tail_num_2 >= spray_Param.min_tail_threshold ||
                                        sum_head_num_2 + sum_tail_num_2 >= spray_Param.min_split_threshold);
                                    //雨雾标记
                                    cond1 = spray_Param.rain_mark_en && sum_mark_num_2 >= spray_Param.min_mark_threshold;
                                    cond0_2 = spray_Param.head_tail_judge_en && !cur_head_2 && cur_tail_2 && cur_dist_2 > 1000 && (cond0_1 || cond1);
                                    cond0_3 = spray_Param.head_tail_judge_en && cur_head_2 && !cur_tail_2 && cur_dist_2 > 1000 && (cond0_1 || cond1);

                                    if (cond0_3 || cond0_0)
                                    {
                                        if (MidAreaMask)
                                        {
                                            rain_wave1_buffer4[spray_col_buffer][row_idx] = 1;
                                            rain_final1_buffer[spray_col_buffer][row_idx] = 1;
                                        }
                                        else
                                        {
                                            for (int i = 0; i < 3; i++)
                                            {
                                                rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                                rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (!cond3)
                                        {
                                            //方差计算，3x3
                                            if (cur_dist_1 < spray_Param.Dist_threshold)// 次发段，分辨率放一半
                                            {
                                                local_dist_a = &local_dist_A[0][0];
                                                local_ref_mask_a = local_ref_mask_A;
                                                local_dist_b = &local_dist_C[0][0];
                                                local_ref_mask_b = local_ref_mask_C;
                                                local_dist_c = &local_dist_E[0][0];
                                                local_ref_mask_c = local_ref_mask_E;
                                                MmOffsetLen[0] = 0;
                                                MmOffsetLen[1] = 2;
                                                MmOffsetLen[2] = 4;
                                            }
                                            else
                                            {
                                                local_dist_a = &local_dist_B[0][0];
                                                local_ref_mask_a = local_ref_mask_B;
                                                local_dist_b = &local_dist_C[0][0];
                                                local_ref_mask_b = local_ref_mask_C;
                                                local_dist_c = &local_dist_D[0][0];
                                                local_ref_mask_c = local_ref_mask_D;
                                                MmOffsetLen[0] = 1;
                                                MmOffsetLen[1] = 2;
                                                MmOffsetLen[2] = 3;
                                            }
                                            //第一回波
                                            int sum_count2[3] = { 0 }, sum_dist2[3] = { 0 }, sum_sq2[3] = { 0 };
                                            auto variCalc2 = [&](const int* dist, const int* ref_mask, const int count, const int idx) {
                                                for (int i = 0; i < count; i++)
                                                {
                                                    if (idx == 1 && (i == 3 || i == 2))continue;
                                                    if (dist[i] != 0 && ref_mask[i] != 0 && abs(dist[i] - cur_dist_2) < 3 * 200)
                                                    {
                                                        sum_count2[idx]++;
                                                        sum_dist2[idx]++;
                                                        sum_sq2[idx]++;
                                                    }
                                                }
                                            };

                                            variCalc2(local_dist_a, local_ref_mask_a, 6, 0);
                                            variCalc2(local_dist_b, local_ref_mask_b, 6, 1);
                                            variCalc2(local_dist_c, local_ref_mask_c, 6, 2);

                                            int count_var_2 = sum_count2[0] + sum_count2[1] + sum_count2[2];
                                            int sum_dist_2 = sum_dist2[0] + sum_dist2[1] + sum_dist2[2];
                                            int sum_sq_2 = sum_sq2[0] + sum_sq2[1] + sum_sq2[2];
                                            int inv_n_2 = 0;
                                            if (count_var_2 != 0)
                                            {
                                                inv_n_2 = spray_Param.inv_table[count_var_2 - 1];
                                            }
                                            else
                                            {
                                                inv_n_2 = 0;
                                            }
                                            int mu_2 = (sum_dist_2 * inv_n_2) >> 12;
                                            int local_var_2 = ((sum_sq_2 - sum_dist_2 * mu_2) * inv_n_2) >> 12;

                                            //方差计算

                                            cond2 = spray_Param.local_var_en && ((local_var_2 >= spray_Param.threshold_var && row_idx > 95) ||
                                                (local_var_2 >= spray_Param.threshold_var2 && row_idx <= 95));
                                            if (cond0_1 || cond1 || cond2 || cond0_2) {
                                                if (MidAreaMask)
                                                {
                                                    rain_wave1_buffer4[spray_col_buffer][row_idx] = 1;
                                                    rain_final1_buffer[spray_col_buffer][row_idx] = 1;
                                                }
                                                else
                                                {
                                                    for (int i = 0; i < 3; i++)
                                                    {
                                                        rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                        rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx] = 1;
                                                        rain_wave1_buffer4[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                                        rain_final1_buffer[spray_col_neib_buf[MmOffsetLen[i]]][row_idx + 1] = 1;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                //雨雾标记读取 第二回波
            }
        }
    }

    // 第二部分：雨滴过滤
    int spray_filter_col = spray_col - SprayFilterDelayCol;
    if (spray_filter_col < 0 || spray_filter_col >= VIEW_W) return;
    int spray_filter_col_buffer = matlabMod(spray_col_buffer + 1 - SprayFilterDelayCol - 1, buffer_size_spray);

    // 计算过滤邻居列索引
    int spray_filter_col_neib_buf[RAIN_SIZE_X];
    for (int i = 0; i < RAIN_SIZE_X; i++) {
        spray_filter_col_neib_buf[i] = matlabMod((spray_filter_col_buffer - spray_Param.Area_size_x + i), buffer_size_spray);
    }

    if (algo_Param.SprayRemoveOn) {
        for (int row_idx = 0; row_idx < VIEW_H; row_idx++) {
            int cur_dist_1 = dist_wave0_buffer4[spray_filter_col_buffer][row_idx];
            bool cur_mark_1 = rain_wave0_buffer4[spray_filter_col_buffer][row_idx];
            int cur_dist_2 = dist_wave1_buffer4[spray_filter_col_buffer][row_idx];
            bool cur_mark_2 = rain_wave1_buffer4[spray_filter_col_buffer][row_idx];
            int cur_ref_1 = refl_wave0_buffer4[spray_filter_col_buffer][row_idx];
            int cur_ref_2 = refl_wave1_buffer4[spray_filter_col_buffer][row_idx];
            bool cur_gnd_1 = grnd_wave0_buffer4[spray_filter_col_buffer][row_idx];
            bool cur_gnd_2 = grnd_wave1_buffer4[spray_filter_col_buffer][row_idx];

            // 计算行范围
            int row_range_up = (row_idx <= 0) ? 0 : spray_Param.Area_size_y;
            int row_range_down = (row_idx >= VIEW_H - 1) ? 0 : spray_Param.Area_size_y;
            int total_rows = row_range_up + row_range_down + 1;

            // 局部数据缓冲区
            int local_dist_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
            bool local_mark_1[RAIN_SIZE_X][RAIN_SIZE_Y] = { false };
            int local_dist_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { 0 };
            bool local_mark_2[RAIN_SIZE_X][RAIN_SIZE_Y] = { false };

            // 填充局部数据
            for (int i = 0; i < RAIN_SIZE_X; i++) {
                int col_idx = spray_filter_col_neib_buf[i];
                for (int j = -row_range_up; j <= row_range_down; j++) {
                    int r = row_idx + j;
                    if (r < 0 || r >= VIEW_H) continue;

                    local_dist_1[i][j + 1] = dist_wave0_buffer4[col_idx][r];
                    local_mark_1[i][j + 1] = rain_wave0_buffer4[col_idx][r];
                    local_dist_2[i][j + 1] = dist_wave1_buffer4[col_idx][r];
                    local_mark_2[i][j + 1] = rain_wave1_buffer4[col_idx][r];
                }
            }

            // 第一回波过滤
            int count_1 = 15; // 初始化为最大值
            if (cur_dist_1 > 0 && !cur_mark_1 &&
                cur_dist_1 <= spray_Param.filter_dist_max &&
                cur_ref_1 <= spray_Param.filter_ref_max) {

                count_1 = 0;
                for (int i = 0; i < RAIN_SIZE_X; i++) {
                    for (int j = 0; j < RAIN_SIZE_Y; j++) {
                        // 跳过中心点
                        if (i == spray_Param.Area_size_x && j == spray_Param.Area_size_y) continue;

                        if (local_dist_1[i][j] > 0 && !local_mark_1[i][j] &&
                            std::abs(local_dist_1[i][j] - cur_dist_1) <= spray_Param.dist_diff_thr) {
                            count_1++;
                        }
                        else if (local_dist_2[i][j] > 0 && !local_mark_2[i][j] &&
                            std::abs(local_dist_2[i][j] - cur_dist_1) <= spray_Param.dist_diff_thr) {
                            count_1++;
                        }
                    }
                }
            }

            // 第二回波过滤（类似）
            int count_2 = 15;
            if (cur_dist_2 > 0 && !cur_mark_2 &&
                cur_dist_2 <= spray_Param.filter_dist_max &&
                cur_ref_2 <= spray_Param.filter_ref_max) {

                count_2 = 0;
                for (int i = 0; i < RAIN_SIZE_X; i++) {
                    for (int j = 0; j < RAIN_SIZE_Y; j++) {
                        // 跳过中心点
                        if (i == spray_Param.Area_size_x && j == spray_Param.Area_size_y) continue;

                        if (local_dist_1[i][j] > 0 && !local_mark_1[i][j] &&
                            std::abs(local_dist_1[i][j] - cur_dist_2) <= spray_Param.dist_diff_thr) {
                            count_2++;
                        }
                        else if (local_dist_2[i][j] > 0 && !local_mark_2[i][j] &&
                            std::abs(local_dist_2[i][j] - cur_dist_2) <= spray_Param.dist_diff_thr) {
                            count_2++;
                        }
                    }
                }
            }

            // 更新最终标记
            bool cur_filter_mask_1 = (cur_mark_1 || count_1 < spray_Param.filter_count_thr) && !cur_gnd_1;
            bool cur_filter_mask_2 = (cur_mark_2 || count_2 < spray_Param.filter_count_thr) && !cur_gnd_2;

            if (cur_filter_mask_1 && cur_filter_mask_2) {
                rain_final0_buffer[spray_filter_col_buffer][row_idx] = 1;
                rain_final1_buffer[spray_filter_col_buffer][row_idx] = 1;
            }
            else if (cur_filter_mask_1) {
                rain_final0_buffer[spray_filter_col_buffer][row_idx] = 1;
                rain_final1_buffer[spray_filter_col_buffer][row_idx] = 0;
            }
            else if (cur_filter_mask_2) {
                rain_final0_buffer[spray_filter_col_buffer][row_idx] = 0;
                rain_final1_buffer[spray_filter_col_buffer][row_idx] = 1;
            }
        }
    }

    // 第三部分：输出处理
    int spray_out_col = spray_filter_col - SprayOutDelayCol;
    if (spray_out_col < 0 || spray_out_col >= VIEW_W) return;
    int spray_out_col_buffer = matlabMod(spray_filter_col_buffer + 1 - SprayOutDelayCol - 1, buffer_size_spray);

    // 复制输出数据
    if (algo_Param.SprayRemoveOn)
    {
        for (int i = 0; i < VIEW_H; i++) {
            spray_mark_out0[i] = rain_final0_buffer[spray_out_col_buffer][i];
            spray_mark_out1[i] = rain_final1_buffer[spray_out_col_buffer][i];
        }
    }
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
int AlgoFunction::pcAlgoMainFunc(tstFrameBuffer* pstFrameBuffer, int task_id)
{
    int proc_col = -1;
    switch (task_id)
    {
        case STRAY_ALGO:
        {
            static bool first{true};

            if (first) {
                pid_t tid = gettid();
                utils::addThread(tid, "Stray_Algo");
                first = false;
            }
            for (int32_t col_idx = 0; col_idx < VIEW_W + max_data_size; ++col_idx){
                // 地面拟合
                if(col_idx >= 0 && col_idx < VIEW_W){
                    if((col_idx % gnd_step) == 0){
                        uint16_t dist_line[VIEW_H];
                        uint16_t col_pos = col_idx / gnd_step;
                        memcpy(dist_line, pstFrameBuffer->dist0[col_idx], sizeof(uint16_t) * VIEW_H);

                        int surface_id = pstFrameBuffer->surface_id.load();
                        int real_col;
                        if(0 == surface_id)
                        {
                            real_col = (col_idx << 1);
                        }
                        else
                        {
                            real_col = UP_VIEW_W - (col_idx << 1) - 1;
                        }

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
                circularCalcIdx(rear3, buffer_size_stray);

                int stray_col_in = col_idx - StrayInDelayCol;
                if (stray_col_in >= 0 && stray_col_in < VIEW_W){
                    memcpy(dist_wave0_buffer3[rear3], pstFrameBuffer->dist0[stray_col_in], sizeof(uint16_t) * VIEW_H);
                    memcpy(dist_wave1_buffer3[rear3], pstFrameBuffer->dist1[stray_col_in], sizeof(uint16_t) * VIEW_H);
                    memcpy(refl_wave0_buffer3[rear3], pstFrameBuffer->ref0[stray_col_in], sizeof(uint8_t) * VIEW_H);
                    memcpy(refl_wave1_buffer3[rear3], pstFrameBuffer->ref1[stray_col_in], sizeof(uint8_t) * VIEW_H);
                    memcpy(grnd_wave0_buffer3[rear3], pstFrameBuffer->gnd_mark0[stray_col_in], sizeof(uint8_t) * VIEW_H);
                    memcpy(grnd_wave1_buffer3[rear3], pstFrameBuffer->gnd_mark1[stray_col_in], sizeof(uint8_t) * VIEW_H);
                    memcpy(high_wave0_buffer3[rear3], pstFrameBuffer->high0[stray_col_in], sizeof(int) * VIEW_H);
                    memcpy(high_wave1_buffer3[rear3], pstFrameBuffer->high1[stray_col_in], sizeof(int) * VIEW_H);
                    for(int i = 0; i < VIEW_H; ++i){
                        uint8_t att0 = pstFrameBuffer->att0[stray_col_in][i];
                        uint8_t att1 = pstFrameBuffer->att1[stray_col_in][i];
                        smlr_wave0_buffer3[rear3][i] = (att0 >> 5) & 0x03;
                        smlr_wave1_buffer3[rear3][i] = (att1 >> 5) & 0x03;
                        peak_wave0_buffer3[rear3][i] = (att0 >> 3) & 0x01;
                        peak_wave1_buffer3[rear3][i] = (att1 >> 3) & 0x01;
                        crtk_wave0_buffer3[rear3][i] = att0 & 0x01;
                        crtk_wave1_buffer3[rear3][i] = att1 & 0x01;
                        rain_wave0_buffer3[rear3][i] = att0 >> 7;
                        rain_wave1_buffer3[rear3][i] = att1 >> 7;
                        head_wave0_buffer3[rear3][i] = (att0 >> 1) & 0x01;
                        head_wave1_buffer3[rear3][i] = (att1 >> 1) & 0x01;
                        tail_wave0_buffer3[rear3][i] = (att0 >> 2) & 0x01;
                        tail_wave0_buffer3[rear3][i] = (att1 >> 2) & 0x01;
                    }
                }
                else {
                    //超过最大列，帧间buffer补0
                    memset(dist_wave0_buffer3[rear3], 0, sizeof(uint16_t) * VIEW_H);
                    memset(dist_wave1_buffer3[rear3], 0, sizeof(uint16_t) * VIEW_H);
                    memset(refl_wave0_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(refl_wave1_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(grnd_wave0_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(grnd_wave1_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(high_wave0_buffer3[rear3], 0, sizeof(int) * VIEW_H);
                    memset(high_wave1_buffer3[rear3], 0, sizeof(int) * VIEW_H);
                    memset(smlr_wave0_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(smlr_wave1_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(peak_wave0_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(peak_wave1_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(crtk_wave0_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(crtk_wave1_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(rain_wave0_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(rain_wave1_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(head_wave0_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                    memset(head_wave1_buffer3[rear3], 0, sizeof(uint8_t) * VIEW_H);
                }
                int stray_col = stray_col_in - StrayDelayCol;
                int stray_col_buffer = matlabMod(rear3 + 1 - StrayDelayCol - 1, buffer_size_stray);
                int stray_col_neib_buf[3];
                for (int i = 0; i < 3; i++) {
                    stray_col_neib_buf[i] = matlabMod((stray_col_buffer + 1 - 1 + i) - 1, buffer_size_stray);
                }
                int stray_mark_out0[VIEW_H] = { 0 };
                int stray_mark_out1[VIEW_H] = { 0 };

                //杂散删除 初始化输出
                if(0 == col_idx){
                    memcpy(&RainWall_in, &RainWall_out, sizeof(RainWall_out));
                    RainWall_out = {0, 0, 0};
                }
                strayDelete(stray_col, stray_col_buffer, stray_col_neib_buf, stray_mark_out0, stray_mark_out1);

                if(stray_col >= 0 && stray_col < VIEW_W){
                    memcpy(stray_mask_out_frm0[stray_col], stray_mark_out0, sizeof(stray_mark_out0));
                    memcpy(stray_mask_out_frm1[stray_col], stray_mark_out1, sizeof(stray_mark_out1));
                }
            }
        }break;
        case RAIN_ALGO:
        {
            static bool first{true};

            if (first) {
                pid_t tid = gettid();
                utils::addThread(tid, "Rain_Algo");
                first = false;
            }
            for (int32_t col_idx = 0; col_idx < VIEW_W + max_data_size; ++col_idx){
                circularCalcIdx(rear4, buffer_size_spray);
                int spray_col_in = col_idx - SprayInDelayCol;
                if (spray_col_in >= 0 && spray_col_in < VIEW_W) {
                    memcpy(dist_wave0_buffer4[rear4], pstFrameBuffer->dist0[spray_col_in], sizeof(uint16_t) * VIEW_H);
                    memcpy(dist_wave1_buffer4[rear4], pstFrameBuffer->dist1[spray_col_in], sizeof(uint16_t) * VIEW_H);
                    memcpy(refl_wave0_buffer4[rear4], pstFrameBuffer->ref0[spray_col_in], sizeof(uint8_t) * VIEW_H);
                    memcpy(refl_wave1_buffer4[rear4], pstFrameBuffer->ref1[spray_col_in], sizeof(uint8_t) * VIEW_H);
                    memcpy(grnd_wave0_buffer4[rear4], pstFrameBuffer->gnd_mark0[spray_col_in], sizeof(uint8_t) * VIEW_H);
                    memcpy(grnd_wave1_buffer4[rear4], pstFrameBuffer->gnd_mark1[spray_col_in], sizeof(uint8_t) * VIEW_H);
                    for (int i = 0; i < VIEW_H; ++i) {
                        uint8_t att0 = pstFrameBuffer->att0[spray_col_in][i];
                        uint8_t att1 = pstFrameBuffer->att1[spray_col_in][i];
                        rain_wave0_buffer4[rear4][i] = att0 >> 7;
                        rain_wave1_buffer4[rear4][i] = att1 >> 7;
                        head_wave0_buffer4[rear4][i] = (att0 >> 1) & 0x01;
                        head_wave1_buffer4[rear4][i] = (att1 >> 1) & 0x01;
                        tail_wave0_buffer4[rear4][i] = (att0 >> 2) & 0x01;
                        tail_wave1_buffer4[rear4][i] = (att1 >> 2) & 0x01;
                        rain_final0_buffer[rear4][i] = att0 >> 7;
                        rain_final1_buffer[rear4][i] = att1 >> 7;
                    }
                }
                else {
                    //超过最大列，帧间buffer补0
                    memset(dist_wave0_buffer4[rear4], 0, sizeof(uint16_t) * VIEW_H);
                    memset(dist_wave1_buffer4[rear4], 0, sizeof(uint16_t) * VIEW_H);
                    memset(refl_wave0_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(refl_wave1_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(grnd_wave0_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(grnd_wave1_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(rain_wave0_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(rain_wave1_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(head_wave0_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(head_wave1_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(tail_wave0_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(tail_wave1_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(rain_final0_buffer[rear4], 0, sizeof(uint8_t) * VIEW_H);
                    memset(rain_final1_buffer[rear4], 0, sizeof(uint8_t) * VIEW_H);
                }
                int spray_col = spray_col_in - SprayDelayCol;
                int spray_col_buffer = matlabMod(rear4 + 1 - SprayDelayCol - 1, buffer_size_spray);
                int spray_col_neib_buf[RAIN_SIZE_X];
                for (int i = 0; i < RAIN_SIZE_X; i++) {
                    spray_col_neib_buf[i] = matlabMod((spray_col_buffer + 1 - spray_Param.Area_size_x + i) - 1, buffer_size_spray);
                }
                int spray_mark_out0[VIEW_H]{ 0 };
                int spray_mark_out1[VIEW_H]{ 0 };
                rainEnhance(spray_col, spray_col_buffer, spray_col_neib_buf, spray_mark_out0, spray_mark_out1);

                int spray_out_col = spray_col - SprayFilterDelayCol - SprayOutDelayCol;
                if (spray_out_col >= 0 && spray_out_col < VIEW_W) {
                    memcpy(spray_mark_out_frm0[spray_out_col], spray_mark_out0, sizeof(spray_mark_out0));
                    memcpy(spray_mark_out_frm1[spray_out_col], spray_mark_out1, sizeof(spray_mark_out1));
                }
            }
        }break;
        default:
        break;
    };

    return proc_col;
}

}   // namespace lidar
}   // namespace robosense