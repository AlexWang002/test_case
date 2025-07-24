/*******************************************************************************
 * \addtogroup driver
 * \{
 * \headerfile algo.cpp
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
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/

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

/*******************************************************************************
 * \brief  Trail remove pre process
 *
 * \param[in] trail_col : Current column index
 *                  Range: 0 - 760. Accuracy: 1.
 * \param[in] trail_col_buffer : Current column buffer index
 *                  Range: 0 - 4. Accuracy: 1.
 * \param[in] trail_col_neib_buf : Neighbor column buffer index
 *                  Range:0-2^32-1. Accuracy:1.
 * \param[in] trail_refer_mask : Trail refer mask
 *                  Range:0-2^32-1. Accuracy:1.
 ******************************************************************************/
void AlgoFunction::trailPreProc(int trail_col, int trail_col_buffer,
    int* trail_col_neib_buf,int* trail_refer_mask)
{

    if (trail_col < 0 || trail_col >= algo_Param.ViewField_Wide
        || !algo_Param.TrailRemoveOn) {
        return;
    }

    constexpr int HL = 5;
    const int near_threshold = trail_Param.near_cnt_th_h;

    for (int row_idx = 0; row_idx < VIEW_H; ++row_idx)
    {
        const int dist_pre = dist_wave0_buffer2[trail_col_buffer][row_idx];
        if (dist_pre <= 0) continue;

        const int diff_th = clamp(std::round(dist_pre *  trail_Param.dist_th_ratio),2,20);
        int dist_pre_buf[HL];
        for (int i = 0; i < HL; ++i) {
            dist_pre_buf[i] = dist_wave0_buffer2[trail_col_neib_buf[i]][row_idx];
        }

        int near_cnt_h = 0;
        for (int i = 0; i < HL - 1; ++i) {
            near_cnt_h += (std::abs(dist_pre_buf[i + 1] - dist_pre_buf[i])
            < diff_th);
        }

        trail_refer_mask[row_idx] = (near_cnt_h >= near_threshold);
    }
}

/*******************************************************************************
 * \brief  Trail remove main process
 *
 * \param[in] trail_col : Current column index
 *                  Range: 0 - 760. Accuracy: 1.
 * \param[in] trail_col_buffer : Current column buffer index
 *                  Range: 0 - 4. Accuracy: 1.
 * \param[in] trail_col_neib_buf : Neighbor column buffer index
 *                  Range:0-2^32-1. Accuracy:1.
 * \param[in] trail_refer_mask : Trail refer mask
 *                  Range:0-2^32-1. Accuracy:1.
 * \param[out] trail_mask_out : Trail mask output
 *                  Range:0-2^32-1. Accuracy:1.
 ******************************************************************************/
void AlgoFunction::trailRemove(int trail_col,
                               int trail_col_buffer,
                               int* trail_col_neib_buf,
                               int* trail_refer_mask,
                               int* trail_mask_out)
{
    if (trail_col < 0 || trail_col >= algo_Param.ViewField_Wide
        || !algo_Param.TrailRemoveOn) {
        return;
    }

    constexpr int NEIGHBOR_COUNT = 5;
    constexpr int LONGITUDINAL_RANGE = 4;
    const int bypass_distance = trail_Param.BypassDis;

    for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
        if (trail_refer_mask[row_idx] != 0) continue;

        const int dist_trail = dist_wave0_buffer2[trail_col_buffer][row_idx];
        if (dist_trail <= 0 || dist_trail >= bypass_distance) continue;

        const int dist_seg = (dist_trail / 12000);
        const int diff_th = noise_params.denoise_offset[dist_seg];


        int dist_tmp[NEIGHBOR_COUNT];
        int ref_tmp[NEIGHBOR_COUNT];
        for (int i = 0; i < NEIGHBOR_COUNT; ++i) {
        const int idx = trail_col_neib_buf[i];
        dist_tmp[i] = dist_wave0_buffer2[idx][row_idx];
        ref_tmp[i] = refl_wave0_buffer2[idx][row_idx];
        }

        int dist_longit[9] = {0};
        int ref_longit[9] = {0};

        const int row_range_up = std::min(row_idx, LONGITUDINAL_RANGE);
        const int row_range_down = std::min(VIEW_H - 1 - row_idx,
                                            LONGITUDINAL_RANGE);

        const int start_idx = 4 - row_range_up;
        for (int i = 0; i <= row_range_up + row_range_down; ++i) {
        const int buf_idx = start_idx + i;
        dist_longit[buf_idx] = dist_wave0_buffer2
                                [trail_col_buffer][row_idx - row_range_up + i];
        ref_longit[buf_idx] = refl_wave0_buffer2
                                [trail_col_buffer][row_idx - row_range_up + i];
        }

        if (trailJudge(dist_tmp, ref_tmp,
            dist_longit, ref_longit, diff_th)) {
            trail_mask_out[row_idx] = 1;
        }
    }
}

/*******************************************************************************
 * \brief  Limit value to minimum and maximum
 * \param[in] val : Value to be limited
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] min_val : Minimum value
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] max_val : Maximum value
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return Limited value
*******************************************************************************/
int AlgoFunction::clamp(int val, int min_val, int max_val) {
    return std::max(min_val, std::min(val, max_val));
}

/*******************************************************************************
 * \brief  Trail remove judge function
 *
 * \param[in] dist_tmp : Distance input array
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] ref_tmp : Reflectance input array
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] dist_longit : Longitudinal distance array
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] ref_longit : Longitudinal reflectance array
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] diff_th : Difference threshold
 *                Range: 128-300. Accuracy: 1.
 * \return 0: Not trail, 1: Trail
 ******************************************************************************/
int AlgoFunction::trailJudge(int* dist_tmp,
                             int* ref_tmp,
                             int* dist_longit,
                             int* ref_longit,
                             const int diff_th)
{
    int trail_judge = 0;
    const int HL = 5;
    const int half_HL = HL / 2;
    const int distC = dist_tmp[half_HL];

    // 差异数组初始化
    int dif_distC[HL] = {0};
    int dif_distC_abs[HL] = {0};
    int dif_dist[HL-1] = {0};
    int dif_dist_abs[HL-1] = {0};
    int dif2_dist_abs[HL-2] = {0};
    int dif_distC_ver_abs[9] = {0};

    // 参数提取
    const int& D_H = trail_Param.D_H;
    const double& dist_th_ratio = trail_Param.dist_th_ratio;
    const int& SlopDifThre = trail_Param.SlopDifThre;
    const double& DisThreRatio = trail_Param.DisThreRatio;
    const int& near_cnt_th_h = trail_Param.near_cnt_th_h;
    const int& near_cnt_th_v = trail_Param.near_cnt_th_v;

    // 计算自适应阈值
    const int AdjDisThreD = std::max(1, static_cast<int>(std::floor(distC * DisThreRatio)));
    const int near_dist_th = clamp(static_cast<int>(std::round(distC * dist_th_ratio)), 2, 20);

    // 初始化权重数组
    int weight[HL] = {1, 1, 1, 1, 1};
    weight[half_HL - 1] = 2;
    weight[half_HL] = 0;
    weight[half_HL + 1] = 2;

    // 统计变量初始化
    int zero_cnt = 0;
    int dif_dist_abs_cnt = 0;
    int near_cnt_h = 0;
    int near_cnt_v = 0;
    int ver_cnt = 0;

    // 水平方向处理 - 第一遍循环
    for (int i = 0; i < HL; i++)
    {
        // 处理无效点
        if (dist_tmp[i] == 0) {
        weight[i] = 0;
        dif_distC[i] = 65535;
        zero_cnt++;
        }
        else {
            dif_distC[i] = dist_tmp[i] - distC;
        }

        // 计算绝对值
        dif_distC_abs[i] = abs(dif_distC[i]);

        // 相邻点差异计算
        if (i < HL - 1) {
            dif_dist[i] = dist_tmp[i + 1] - dist_tmp[i];
            dif_dist_abs[i] = abs(dif_dist[i]);

            // 大差异计数
            if (dif_dist_abs[i] > AdjDisThreD) {
                dif_dist_abs_cnt++;
            }

            // 近邻计数
            if (dif_dist_abs[i] < near_dist_th) {
            near_cnt_h++;
            }

            // 二阶差分计算
            if (i > 0) {
            dif2_dist_abs[i - 1] = abs(dif_dist[i] - dif_dist[i - 1]);
            }
        }
    }

    // 主判断条件
    bool con0 = zero_cnt <= half_HL + 1;
    if (!con0) {
    return trail_judge;
    }

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

    if (!con1 && !con2) {
        return trail_judge;
    }

    // 垂直方向处理
    for (int j = 0; j < 9; j++) {
    if (dist_longit[j] == 0) {
    dif_distC_ver_abs[j] = 65535;
    } else {
        dif_distC_ver_abs[j] = abs(dist_longit[j] - distC);
    }

    // 近邻计数
    if (dif_distC_ver_abs[j] < near_dist_th) {
    near_cnt_v++;
    }

    // 垂直计数
    if (dif_distC_ver_abs[j] < SlopDifThre * 2) {
    ver_cnt++;
    }
    }

    // 条件3计算
    bool con3_1 = near_cnt_h < near_cnt_th_h;
    bool con3_2 = near_cnt_v < near_cnt_th_v;
    bool con3 = con3_1 && con3_2;

    // 条件4计算
    int sumTmp = 0;
    for (int i = 0; i < 3; i++) {
        sumTmp += dif2_dist_abs[i];
    }
    bool con4_1 = sumTmp > SlopDifThre;
    bool con4_2 = (half_HL >= 2) ? (dif2_dist_abs[half_HL - 2] > SlopDifThre) : false;
    bool con4_3 = ver_cnt < 3;
    bool con4 = (con4_1 && con4_2) || con4_3;

    // 最终判断
    if (con3 && con4) {
        trail_judge = 1;
    }

    return trail_judge;
}

/*******************************************************************************
 * \brief  Choose distance filter according to the length of the data
 *
 * \param[in] data: input distance of filter neighborhood
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] length: the length of the data
 *                Range: 1 - 11. Accuracy: 1.
 * \param[in] order: fitting coefficient
*                Range: 1,2. Accuracy: 1.
 * \return int: the result of the distance filter
 ******************************************************************************/
int AlgoFunction::applyFilter(int* data, int length, int order)
{
    int num = 0;

    switch (length) {
        case 1:
        {
            for (int j = 0; j < 3; j++) {
                num += (order == 1 ? Smooth_params.B31[0][j] : Smooth_params.B31[1][j]) * data[j];
            }
        }break;
        case 2:
        case 3:
        {
            int filter_size = (length == 2) ? 3 : 3; // B33或B32
            const int* filter = (length == 2) ?
                (order == 1 ? Smooth_params.B33[0] : Smooth_params.B33[1]):
                (order == 1 ? Smooth_params.B32[0] : Smooth_params.B32[1]);
            for (int j = 0; j < filter_size; j++) {
                num += filter[j] * data[j];
            }
        }break;
        case 5:
        case 7:
        case 9:
        case 11:
        {
            const int* filter = (length == 5) ? (order == 1 ? Smooth_params.B5[0] : Smooth_params.B5[1]) :
                            (length == 7) ? (order == 1 ? Smooth_params.B7[0] : Smooth_params.B7[1]) :
                            (length == 9) ? (order == 1 ? Smooth_params.B9[0] : Smooth_params.B9[1]) :
                                            (order == 1 ? Smooth_params.B11[0] : Smooth_params.B11[1]);
            for (int j = 0; j < length; j++) {
                num += filter[j] * data[j];
            }
        }break;
        default:
            // 默认处理
            for (int j = 0; j < length; j++) {
                num += (order == 1 ? Smooth_params.B11[0][j] : Smooth_params.B11[1][j]) * data[j];
            }
        break;
    }

    return num >> 12;
}

/*******************************************************************************
 * \brief  Horizontal smooth processing
 * 
 * \param[in] hor_smooth_col: the column of the horizontal smooth
 *                Range: 0 - 759. Accuracy: 1.
 * \param[in] hor_smooth_col_buffer: the buffer index of the horizontal smooth
 *                Range: 0 -10. Accuracy: 1.
 * \param[in] hor_smooth_col_neib_buf: the buffer index of the horizontal smooth neighbor
 *                Range: 0 - 2^32-1. Accuracy: 1.
*******************************************************************************/
void AlgoFunction::hSmooth(int hor_smooth_col,
    int hor_smooth_col_buffer, int* hor_smooth_col_neib_buf)
{
    // 提前条件判断，减少不必要的计算
    if (hor_smooth_col < 0 || hor_smooth_col >= VIEW_W ||
        (!algo_Param.HorDistSmoothOn && !algo_Param.HorRefSmoothOn)) {
        return;
    }

    const int SmoothH = Smooth_params.SmoothH;

    // 合并距离和反射的平滑处理
    for (int row_idx = 0; row_idx < VIEW_H; ++row_idx)
    {
        int dist_cur = dist_wave0_buffer1[hor_smooth_col_buffer][row_idx];
        int ref_cur = refl_wave0_buffer1[hor_smooth_col_buffer][row_idx];

        // 提前判断阈值条件
        if (dist_cur <= Smooth_params.dist_filter_start_th || dist_cur > Smooth_params.dist_filter_end_th) {
            continue;
        }

        // 合并数据读取
        int dist_src[5] = {0};
        int ref_src[5] = {0};
        for (int i = 0; i < 5; ++i) {
            dist_src[i] = dist_wave0_buffer1[hor_smooth_col_neib_buf[i]][row_idx];
            ref_src[i] = refl_wave0_buffer1[hor_smooth_col_neib_buf[i]][row_idx];
        }

        // 计算公共阈值
        int dist_th = clamp((dist_cur * Smooth_params.dist_th_ratio) >> 12, 30, 150);
        int filter_diff_th = 0, mean_diff_th = 0;

        if (algo_Param.HorDistSmoothOn) {
            filter_diff_th = clamp((dist_cur * Smooth_params.filter_diff_th_ratio) >> 12, 50, 100);
            mean_diff_th = clamp((dist_cur * Smooth_params.mean_diff_th_ratio) >> 12, 50, 100);
        }

        // 合并左右数据处理
        int dist_dst_L[3] = {0}, dist_dst_R[3] = {0};
        int ref_dst_L[3] = {0}, ref_dst_R[3] = {0};
        int realL_L = 0, realL_R = 0;
        int realL_ref_L = 0, realL_ref_R = 0;
        int end_flag_L = 0, end_flag_R = 0;
        int sum_diff = 0, sum_diff_abs = 0, mean_dist = 0;

        int cosis_cnt_L = 0;
        int cosis_cnt_R = 0;
        int diff_pre_L = 0;
        int diff_pre_R = 0;
        for (int i = 0; i < SmoothH; ++i)
        {
            // 右数据处理
            int dist_tmp_R = dist_src[SmoothH + 1 + i];
            int ref_tmp_R = ref_src[SmoothH + 1 + i];
            int diff_R = dist_tmp_R - dist_cur;
            int abs_diff_R = abs(diff_R);
            int abs_ref_diff_R = abs(ref_tmp_R - ref_cur);

            if (abs_diff_R < dist_th && end_flag_R < 2) {
                if (abs(diff_R - diff_pre_R) < Smooth_params.dist_th_cosisH)
                {
                    cosis_cnt_R++;
                }
                dist_dst_R[realL_R] = dist_tmp_R;
                realL_R++;
                if (abs_ref_diff_R < Smooth_params.ref_th) {
                    ref_dst_R[realL_ref_R++] = ref_tmp_R;
                }
                sum_diff += diff_R;
                sum_diff_abs += abs_diff_R;
                mean_dist += dist_tmp_R;
                diff_pre_R = diff_R;
            } else if (dist_tmp_R <= 0 || abs_diff_R > 400) {
                end_flag_R++;
            }

            // 左数据处理
            int dist_tmp_L = dist_src[SmoothH - 1 - i];
            int ref_tmp_L = ref_src[SmoothH - 1 - i];
            int diff_L = dist_tmp_L - dist_cur;
            int abs_diff_L = abs(diff_L);
            int abs_ref_diff_L = abs(ref_tmp_L - ref_cur);

            if (abs_diff_L < dist_th && end_flag_L < 2) {
                if (abs(diff_L - diff_pre_L) < Smooth_params.dist_th_cosisH)
                {
                    cosis_cnt_L = cosis_cnt_L + 1;
                }
                dist_dst_L[realL_L] = dist_tmp_L;
                realL_L++;
                if (abs_ref_diff_L < Smooth_params.ref_th) {
                    ref_dst_L[realL_ref_L++] = ref_tmp_L;
                }
                sum_diff += diff_L;
                sum_diff_abs += abs_diff_L;
                mean_dist += dist_tmp_L;
                diff_pre_L = diff_L;
            } else if (dist_tmp_L <= 0 || abs_diff_L > 400) {
                end_flag_L++;
            }
        }

        // 距离平滑处理
        if (algo_Param.HorDistSmoothOn)
        {
            sum_diff = abs(sum_diff);
            int dist_dst[11] = {0}; // 使用最大可能大小
            int real_L = 0;

            if (realL_R != 0 && realL_L != 0) {
                int realHL = std::min(realL_R, realL_L);
                real_L = 2 * realHL + 1;

                for (int i = 0; i < realHL; i++) {
                    dist_dst[i] = dist_dst_L[realHL - 1 - i];
                }
                dist_dst[realHL] = dist_cur;
                for (int i = 0; i < realHL; i++) {
                    dist_dst[realHL + 1 + i] = dist_dst_R[i];
                }

                int idx = realL_R + realL_L - 1;
                mean_dist = (int)round((mean_dist * Smooth_params.data_len_inv[idx]) / 4096.0);
            }
            else if (realL_R > 1) {
                real_L = 1;
                dist_dst[0] = dist_cur;
                if (realL_R >= 2) {
                    dist_dst[1] = dist_dst_R[0];
                    dist_dst[2] = dist_dst_R[1];
                }
                mean_dist = dist_cur;
            }
            else if (realL_L > 1) {
                real_L = 2;
                dist_dst[0] = dist_dst_L[1];
                dist_dst[1] = dist_dst_L[0];
                dist_dst[2] = dist_cur;
                mean_dist = dist_cur;
            }

            if (real_L != 0)
            {
                bool need_filter = (sum_diff_abs < Smooth_params.abs_diff_sum_th) ||
                                    (sum_diff < Smooth_params.diff_sum_th) ||
                                    (abs(mean_dist - dist_cur) < mean_diff_th);
                    bool edge_flag = (cosis_cnt_R >= realL_R - 1 && abs(diff_pre_R) < Smooth_params.dist_th_cosisH) ||
                        (cosis_cnt_L >= realL_L - 1 && abs(diff_pre_L) < Smooth_params.dist_th_cosisH);

                    if (need_filter && (!edge_flag))
                {
                    int order = (sum_diff_abs < ((dist_cur * Smooth_params.var_ratio) >> 12) ||
                                (abs(mean_dist - dist_cur) < filter_diff_th)) ? 1 : 2;

                    int dist_filter = applyFilter(dist_dst, real_L, order);

                        if (abs(dist_filter - dist_cur) < filter_diff_th && dist_filter > 0) {
                        dist_wave0_buffer1[hor_smooth_col_buffer][row_idx] = dist_filter;
                    }
                }
                else if (real_L >= 3) {
                    int num = 0;
                    int mid = real_L / 2;
                    for (int j = mid; j < mid + 3; j++) {
                        num += Smooth_params.B32[1][j - mid] * dist_dst[j - 1];
                    }
                    int dist_filter = num >> 12;
                        if (dist_filter > 0 && abs(dist_filter - dist_cur) < filter_diff_th) {
                        dist_wave0_buffer1[hor_smooth_col_buffer][row_idx] = dist_filter;
                    }
                }
            }
        }

        // 反射平滑处理
        if (algo_Param.HorRefSmoothOn)
        {
            int ref_dst[11] = {0}; // 使用最大可能大小
            int real_ref_L = 0;

            if (realL_ref_R != 0 && realL_ref_L != 0) {
                int realHrefL = std::min(realL_ref_L, realL_ref_R);
                real_ref_L = 2 * realHrefL + 1;
                for (int i = 0; i < realHrefL; i++) {
                    ref_dst[i] = ref_dst_L[realHrefL - 1 - i];
                }
                ref_dst[realHrefL] = ref_cur;
                for (int i = 0; i < realHrefL; i++) {
                    ref_dst[realHrefL + 1 + i] = ref_dst_R[i];
                }
            }
            else if (realL_ref_R > 1) {
                real_ref_L = 1;
                ref_dst[0] = ref_cur;
                ref_dst[1] = ref_dst_R[0];
                ref_dst[2] = ref_dst_R[1];
            }
            else if (realL_ref_L > 1) {
                real_ref_L = 2;
                ref_dst[0] = ref_dst_L[1];
                ref_dst[1] = ref_dst_L[0];
                ref_dst[2] = ref_cur;
            }

            if (real_ref_L != 0) {
                int order = 1;
                int ref_filter = applyFilter(ref_dst, real_ref_L, order);
                if (ref_filter > 0 && ref_filter <= 255)
                {
                    refl_wave0_buffer1[hor_smooth_col_buffer][row_idx] = ref_filter;
                }
            }
        }
    }
}

/*******************************************************************************
 * \brief  Vertical smooth processing
 * 
 * \param[in] ver_smooth_col: the column of the vertical smooth
 *                Range: 0 - 759. Accuracy: 1.
 * \param[in] ver_smooth_col_buffer: the buffer index of the vertical smooth
 *                Range: 0 -10. Accuracy: 1.
*******************************************************************************/
void AlgoFunction::vSmooth(int ver_smooth_col, int ver_smooth_col_buffer)
{
    const int SmoothH = 2;

    if (ver_smooth_col < 0 || ver_smooth_col >= VIEW_W) return;

    // 提前加载常用参数
    const bool verDistSmoothOn = algo_Param.VerDistSmoothOn == 1;
    const bool verRefSmoothOn = algo_Param.VerRefSmoothOn == 1;
    if (!verDistSmoothOn && !verRefSmoothOn) return;

    const int interface_rows = Smooth_params.interface_rows;
    const int ex_row = Smooth_params.ex_row;
    const int gap_len = Smooth_params.gap_len;
    const int dist_filter_start_th = Smooth_params.dist_filter_start_th;
    const int dist_filter_end_th = Smooth_params.dist_filter_end_th;
    const int ref_th = Smooth_params.ref_th;

    // 预计算常用值
    const int interface_end = interface_rows - ex_row + 1;
    const int gap_interface_end = interface_rows - gap_len + 1;

    uint16_t dist_line[VIEW_H];
    uint8_t ref_line[VIEW_H];

    memcpy(dist_line,dist_wave0_buffer1[ver_smooth_col_buffer],sizeof(uint16_t) * VIEW_H);
    memcpy(ref_line,refl_wave0_buffer1[ver_smooth_col_buffer],sizeof(uint8_t) * VIEW_H);

    for (int row_idx = 0; row_idx < VIEW_H; row_idx++)
    {
        const int dist_cur = dist_line[row_idx];
        const int ref_cur = ref_line[row_idx];

        // 快速跳过不在距离范围内的点
        if (dist_cur < dist_filter_start_th || dist_cur > dist_filter_end_th) continue;

        // 计算行区域和SmoothV
        const int row_in_region = (row_idx + 1) % interface_rows;
        const int SmoothV = (row_in_region <= ex_row || row_in_region >= interface_end)
                                ? Smooth_params.SmoothV_list[1]
                                : Smooth_params.SmoothV_list[0];

        // 计算gap位置
        const bool gap_pos = ((row_idx >= gap_len && row_in_region >= gap_interface_end) ||
                            (row_idx < VIEW_H - gap_len && row_in_region <= gap_len));
        if (!gap_pos) continue;

        // 准备数据缓冲区 - 优化内存访问
        int dist_src[7] = {0};
        int ref_src[7] = {0};
        const int window_size = 2 * SmoothV + 1;

        // 边界处理优化
        if (row_idx <= SmoothV)
        {
            const int valid_size = std::min(row_idx + SmoothV + 1, VIEW_H);
            for (int i = 0; i < valid_size; i++)
            {
                const int src_pos = 2 * SmoothV - valid_size + 1 + i;
                dist_src[src_pos] = dist_line[i];
                ref_src[src_pos] = ref_line[i];
            }
        }
        else if (row_idx >= VIEW_H - SmoothV)
        {
            const int valid_rows = std::min(SmoothV + 1, VIEW_H - row_idx);
            for (int i = 0; i < valid_rows; i++)
            {
                dist_src[i] = dist_line[VIEW_H - valid_rows + i];
                ref_src[i] = ref_line[VIEW_H - valid_rows + i];
            }
        }
        else
        {
            const int start_row = row_idx - SmoothV;
            for (int i = 0; i < window_size; i++)
            {
                dist_src[i] = dist_line[start_row + i];
                ref_src[i] = ref_line[start_row + i];
            }
        }

        // 计算距离阈值
        int dist_th = (dist_cur * Smooth_params.dist_th_ratio) >> 12;
        dist_th = (dist_th < 30) ? 30 : (dist_th > 500) ? 500 : dist_th;

        // 邻域分析
        int realL_L = 0, realL_R = 0;
        int realL_ref_L = 0, realL_ref_R = 0;
        int end_flag_L = 0, end_flag_R = 0;
        int sum_diff = 0, sum_diff_abs = 0, mean_dist = 0;
        int dist_dst_L[5] = {0}, dist_dst_R[5] = {0};
        int ref_dst_L[5] = {0}, ref_dst_R[5] = {0};
        int cosis_cnt_L = 0;
        int cosis_cnt_R = 0;
        int diff_pre_L = 0;
        int diff_pre_R = 0;

        for (int i = 1; i <= SmoothV; i++)
        {
            const int idx_R = SmoothV + i;
            const int idx_L = SmoothV - i;
            const int dist_tmp_R = dist_src[idx_R];
            const int dist_tmp_L = dist_src[idx_L];
            const int ref_tmp_R = ref_src[idx_R];
            const int ref_tmp_L = ref_src[idx_L];
            //const int diff_tmp_L = dist_src[SmoothV] - dist_src[idx_L];
            //const int diff_tmp_R = dist_src[idx_R] - dist_src[SmoothV];

            const int diff_R = dist_tmp_R - dist_cur;
            const int diff_L = dist_tmp_L - dist_cur;
            const int abs_diff_R = abs(diff_R);
            const int abs_diff_L = abs(diff_L);
            const int abs_ref_diff_R = abs(ref_tmp_R - ref_cur);
            const int abs_ref_diff_L = abs(ref_tmp_L - ref_cur);

            // 右侧邻域处理
            if (abs_diff_R < dist_th && end_flag_R < 2)
            {
                if (abs(diff_R - diff_pre_R) < Smooth_params.dist_th_cosisV)
                {
                    cosis_cnt_R++;
                }
                dist_dst_R[realL_R++] = dist_tmp_R;
                if (abs_ref_diff_R < ref_th || gap_pos)
                {
                    ref_dst_R[realL_ref_R++] = ref_tmp_R;
                }
                sum_diff += diff_R;
                sum_diff_abs += abs_diff_R;
                mean_dist += dist_tmp_R;
                diff_pre_R = diff_R;
            }
            else if (dist_tmp_R <= 0 || abs_diff_R > 400)
            {
                end_flag_R++;
            }

            // 左侧邻域处理
            if (abs_diff_L < dist_th && end_flag_L < 2)
            {
                if (abs(diff_L - diff_pre_L) < Smooth_params.dist_th_cosisV) {
                    cosis_cnt_L++;
                }
                dist_dst_L[realL_L++] = dist_tmp_L;
                if (abs_ref_diff_L < ref_th || gap_pos)
                {
                    ref_dst_L[realL_ref_L++] = ref_tmp_L;
                }
                sum_diff += diff_L;
                sum_diff_abs += abs_diff_L;
                mean_dist += dist_tmp_L;
                diff_pre_L = diff_L;
            }
            else if (dist_tmp_L <= 0 || abs_diff_L > 400)
            {
                end_flag_L++;
            }
        }

        // 距离平滑处理
        if (verDistSmoothOn && (realL_L > 0 || realL_R > 0))
        {
            int dist_dst_vec[11] = {0};
            int real_L = 0;
            const int sum_diff_abs_val = abs(sum_diff);

            if (realL_R != 0 && realL_L != 0)
            {
                const int realHL = std::min(realL_R, realL_L);
                real_L = 2 * realHL + 1;
                for (int i = 0; i < realHL; i++)
                {
                    dist_dst_vec[i] = dist_dst_L[realHL - 1 - i];
                }
                dist_dst_vec[realHL] = dist_cur;
                for (int i = 0; i < realHL; i++)
                {
                    dist_dst_vec[realHL + 1 + i] = dist_dst_R[i];
                }
                mean_dist = (mean_dist * Smooth_params.data_len_inv[realL_R + realL_L - 1] + 2048) >> 12;
            }
            else if (realL_R > 1)
            {
                real_L = 1;
                dist_dst_vec[0] = dist_cur;
                dist_dst_vec[1] = dist_dst_R[0];
                dist_dst_vec[2] = dist_dst_R[1];
                mean_dist = dist_cur;
            }
            else if (realL_L > 1)
            {
                real_L = 2;
                dist_dst_vec[0] = dist_dst_L[1];
                dist_dst_vec[1] = dist_dst_L[0];
                dist_dst_vec[2] = dist_cur;
                mean_dist = dist_cur;
            }

            if (real_L > 0)
            {
                const int filter_diff_th = std::max(50, std::min((dist_cur * Smooth_params.filter_diff_th_ratio) >> 12, 100));
                const int mean_diff_th = std::max(50, std::min((dist_cur * Smooth_params.mean_diff_th_ratio) >> 12, 100));
                const int need_filter = sum_diff_abs < Smooth_params.abs_diff_sum_th ||
                    sum_diff_abs_val < Smooth_params.diff_sum_th ||
                    abs(mean_dist - dist_cur) < mean_diff_th;
                const bool  edge_flag = (cosis_cnt_R >= realL_R - 1 && abs(diff_pre_R) < Smooth_params.dist_th_cosisV) ||
                    (cosis_cnt_L >= realL_L - 1 && abs(diff_pre_L) < Smooth_params.dist_th_cosisV);

                if (!edge_flag && need_filter)
                {
                    const int order = (sum_diff_abs < ((dist_cur * Smooth_params.var_ratio) >> 12) ||
                                        abs(mean_dist - dist_cur) < filter_diff_th) ? 1 : 2;

                    int dist_filter = dist_cur;
                    const int* filter_coeff = nullptr;
                    int filter_size = 0;

                    // 选择滤波器系数
                    if (real_L == 1)
                    {
                        filter_coeff = Smooth_params.B31[order - 1];
                        filter_size = 3;
                    }
                    else if (real_L == 2)
                    {
                        filter_coeff = Smooth_params.B33[order - 1];
                        filter_size = 3;
                    }
                    else if (real_L == 3)
                    {
                        filter_coeff = Smooth_params.B32[order - 1];
                        filter_size = 3;
                    }
                    else if (real_L == 5)
                    {
                        filter_coeff = Smooth_params.B5[order - 1];
                        filter_size = 5;
                    }
                    else if (real_L == 7)
                    {
                        filter_coeff = Smooth_params.B7[order - 1];
                        filter_size = 7;
                    }
                    else if (real_L == 9)
                    {
                        filter_coeff = Smooth_params.B9[order - 1];
                        filter_size = 9;
                    }
                    else
                    {
                        filter_coeff = Smooth_params.B11[order - 1];
                        filter_size = 11;
                    }

                    // 应用滤波器
                    int num = 0;
                    for (int j = 0; j < filter_size; j++)
                    {
                        num += filter_coeff[j] * dist_dst_vec[j];
                    }
                    dist_filter = num >> 12;

                    if (abs(dist_filter - dist_cur) < filter_diff_th && dist_filter> 0)
                    {
                        dist_wave0_buffer1[ver_smooth_col_buffer][row_idx] = dist_filter;
                    }
                }
                else if (real_L >= 3)
                {
                    const int mid = real_L / 2;
                    const int* filter_coeff = Smooth_params.B32[1];
                    int num = 0;
                    for (int j = mid; j < mid + 3; j++)
                    {
                        num += filter_coeff[j - mid] * dist_dst_vec[j - 1];
                    }
                    const int dist_filter = num >> 12;
                    const int filter_diff_th = std::max(50, std::min((dist_cur * Smooth_params.filter_diff_th_ratio) >> 12, 100));

                        if (abs(dist_filter - dist_cur) < filter_diff_th && dist_filter > 0)
                    {
                        dist_wave0_buffer1[ver_smooth_col_buffer][row_idx] = dist_filter;
                    }
                }
            }
        }

        // 反射率平滑处理
        if (verRefSmoothOn && (realL_ref_L > 0 || realL_ref_R > 0))
        {
            int ref_dst_vec[11] = {0};
            int real_ref_L = 0;

            if (realL_ref_R != 0 && realL_ref_L != 0)
            {
                const int realHrefL = std::min(realL_ref_L, realL_ref_R);
                real_ref_L = 2 * realHrefL + 1;
                for (int i = 0; i < realHrefL; i++)
                {
                    ref_dst_vec[i] = ref_dst_L[realHrefL - 1 - i];
                }
                ref_dst_vec[realHrefL] = ref_cur;
                for (int i = 0; i < realHrefL; i++)
                {
                    ref_dst_vec[realHrefL + 1 + i] = ref_dst_R[i];
                }
            }
            else if (realL_ref_R > 1)
            {
                real_ref_L = 1;
                ref_dst_vec[0] = ref_cur;
                ref_dst_vec[1] = ref_dst_R[0];
                ref_dst_vec[2] = ref_dst_R[1];
            }
            else if (realL_ref_L > 1)
            {
                real_ref_L = 2;
                ref_dst_vec[0] = ref_dst_L[1];
                ref_dst_vec[1] = ref_dst_L[0];
                ref_dst_vec[2] = ref_cur;
            }

            if (real_ref_L > 0)
            {
                const int order = 1; // 固定1阶
                const int* filter_coeff = nullptr;
                int filter_size = 0;

                // 选择滤波器系数
                if (real_ref_L == 1)
                {
                    filter_coeff = Smooth_params.B31[order - 1];
                    filter_size = 3;
                }
                else if (real_ref_L == 2)
                {
                    filter_coeff = Smooth_params.B33[order - 1];
                    filter_size = 3;
                }
                else if (real_ref_L == 3)
                {
                    filter_coeff = Smooth_params.B32[order - 1];
                    filter_size = 3;
                }
                else if (real_ref_L == 5)
                {
                    filter_coeff = Smooth_params.B5[order - 1];
                    filter_size = 5;
                }
                else if (real_ref_L == 7)
                {
                    filter_coeff = Smooth_params.B7[order - 1];
                    filter_size = 7;
                }
                else if (real_ref_L == 9)
                {
                    filter_coeff = Smooth_params.B9[order - 1];
                    filter_size = 9;
                }
                else
                {
                    filter_coeff = Smooth_params.B11[order - 1];
                    filter_size = 11;
                }

                // 应用滤波器
                int num = 0;
                for (int j = 0; j < filter_size; j++)
                {
                    num += filter_coeff[j] * ref_dst_vec[j];
                }
                const int ref_filter = num >> 12;

                if (gap_pos)
                {
                    if (ref_filter > ref_cur && ref_filter > 0 && ref_filter <= 255)
                    {
                        refl_wave0_buffer1[ver_smooth_col_buffer][row_idx] = ref_filter;
                    }
                }
                else
                {
                    if (ref_filter > 0 && ref_filter <= 255)
                    {
                        refl_wave0_buffer1[ver_smooth_col_buffer][row_idx] = ref_filter;
                    }
                }
            }
        }
    }
}

/*******************************************************************************
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
*******************************************************************************/
void AlgoFunction::rebuildFunc(int* dist_tmp1,
                               int* dist_tmp2,
                               int* mask1,
                               int* mask2,
                               int* ad_mask1,
                               int* ad_mask2)
{
    InsertParam_t params;
    memset(mask1, 0, 3 * sizeof(int));
    memset(mask2, 0, 3 * sizeof(int));
    memset(ad_mask1, 0, 3 * sizeof(int));
    memset(ad_mask2, 0, 3 * sizeof(int));

    // 参数解包到局部变量（减少结构体访问开销）
    const int diff_th = params.diff_th;
    const int diff_ratio_max = params.diff_ratio_max;
    const int diff2_th = params.diff2_th;
    const double ratio_scale = diff_ratio_max / 4096.0;

    // 替代 Eigen::Vector3i dist_c1 = dist_tmp1.segment(1, 3)
    int dist_c1[3] = { dist_tmp1[1], dist_tmp1[2], dist_tmp1[3] };  // 注意原始数据索引偏移
    int dist_c2[3] = { dist_tmp2[1], dist_tmp2[2], dist_tmp2[3] };

    // 计算距离差值（展开所有计算）
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
    const int coef1 = (consis_cnt1 > 0) ? params.coef_inv[consis_cnt1 - 1] : 0;
    const int consis_dist1 = (res1 * coef1) >> 12;

    // 处理右侧数据（相同模式优化）

    int res2 = 0, consis_cnt2 = 0;
    for (int i = 0; i < 3; ++i) {
        res2 += mask2_hor[i] * dist_c2[i];
        consis_cnt2 += mask2_hor[i];
    }
    const int coef2 = (consis_cnt2 > 0) ? params.coef_inv[consis_cnt2 - 1] : 0;
    const int consis_dist2 = (res2 * coef2) >> 12;


    // 主条件判断优化
    const int diff_threshold = (consis_dist1 * diff_ratio_max) >> 12;
    const bool main_condition =
        (std::abs(consis_dist1 - consis_dist2) < diff_threshold &&
            consis_cnt1 >= 2 && consis_cnt2 >= 2 &&
            (mask1_hor[1] + mask2_hor[1] > 0));

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

#if 0
    int mask1_ver[3] = { 0 }, mask2_ver[3] = { 0 };
    /* 垂直一致性检测（展开计算）*/
    for (int i = 0; i < 3; ++i) {
        if (abs_cross_diff[i] < diff_th) {
            mask1_ver[i] = (dist_c1[i] > 0) ? 1 : 0;
            mask2_ver[i] = (dist_c2[i] > 0) ? 1 : 0;
        }
    }

    /* 三角一致性检测（展开计算）*/
    auto checkTriangle = [&](const int* dist, const int* diffs,
        int* tri_mask, int* ad_tri_mask) {
            for (int i = 0; i < 3; ++i) {
                const bool cond1 = (diffs[i] < diff2_th);
                const bool cond2 = (diffs[(i + 1) % 3] < diff2_th);

                if (cond1 && cond2) {
                    tri_mask[i] = 1;
                    ad_tri_mask[i] = (dist[i + 1] == 0) ? 2 : 0;
                }
                else if (cond1 || cond2) {
                    ad_tri_mask[i] = 1;
                }
            }
    };

    // 执行三角检测
    checkTriangle(dist_tmp1, abs_dist_diff, mask1_tri, ad_tri_mask1);
    checkTriangle(dist_tmp2, abs_dist_diff + 3, mask2_tri, ad_tri_mask2);

    for (int i = 0; i < 3; ++i) {
        // 水平掩码优先
        mask1[i] = (mask1_hor[i] > 0) ? 1 : ((mask1_ver[i] > 0) ? 1 : 0);
        mask2[i] = (mask2_hor[i] > 0) ? 1 : ((mask2_ver[i] > 0) ? 1 : 0);

        // 三角掩码覆盖
        if (mask1_tri[i] > 0) {
            mask1[i] = 1;
            ad_mask1[i] = ad_tri_mask1[i];
        }
        if (mask2_tri[i] > 0) {
            mask2[i] = 1;
            ad_mask2[i] = ad_tri_mask2[i];
        }
    }

    //const int diffs3[3] = { abs_dist_diff[0], abs_dist_diff[1], abs_dist_diff[2] };
    //const int diffs4[3] = { abs_dist_diff[3], abs_dist_diff[4], abs_dist_diff[5] };

    //checkConsistency(dist_tmp1, diffs3, mask1_hor);
    //checkConsistency(dist_tmp2, diffs4, mask2_hor);



    /* 条件判断集合优化（展开为独立判断）*/
    // 条件1
    if ((mask_cross[0] || mask_cross[1]) &&
        dist_tmp1[1] > 0 && dist_tmp2[1] > 0 &&
        dist_tmp1[2] > 0 && dist_tmp2[2] > 0 &&
        std::abs(dist_diff11 - dist_diff21) < diff2_th &&
        abs_dist_diff[0] < diff_th_max1[0])
    {
        mask1_ver[0] = mask1_ver[1] = 1;
        mask2_ver[0] = mask2_ver[1] = 1;
    }

    // 条件2（优化重复项计算）
    const bool cond2_base =
        mask_cross[1] &&
        dist_tmp1[2] > 0 && dist_tmp2[2] > 0 &&
        dist_tmp1[1] > 0 && dist_tmp2[3] > 0;
    if (cond2_base &&
        std::abs(dist_diff11 - dist_diff22) < diff2_th &&
        abs_dist_diff[0] < diff_th_max1[0])
    {
        mask1_ver[0] = mask1_ver[1] = 1;
        mask2_ver[1] = mask2_ver[2] = 1;
    }

    // 条件3-8采用相同模式优化...
    // [为节省篇幅，这里展示典型条件优化模式，实际需完整展开]

    // 条件3
    if (mask_cross[1] &&
        dist_tmp1[2] > 0 && dist_tmp2[2] > 0 &&
        dist_tmp1[3] > 0 && dist_tmp2[1] > 0 &&
        std::abs(dist_diff12 - dist_diff21) < diff2_th &&
        abs_dist_diff[1] < diff_th_max1[1])
    {
        mask1_ver[1] = mask1_ver[2] = 1;
        mask2_ver[0] = mask2_ver[1] = 1;
    }

    // 计算单列计数（展开循环优化）
    int mono_cnt1 = mask1_ver[0] + mask1_ver[1] + mask1_ver[2];
    int mono_cnt2 = mask2_ver[0] + mask2_ver[1] + mask2_ver[2];

    // 左侧与右侧单调判断（优化分支预测）
    if (mono_cnt1 >= 2 && mono_cnt2 >= 2) {
        memcpy(mask1, mask1_ver, 3 * sizeof(int));
        memcpy(mask2, mask2_ver, 3 * sizeof(int));

        // 使用位运算代替条件判断
        ad_mask1[1] = (mask1_ver[1] == 0) ? 2 : ad_mask1[1];
        ad_mask2[1] = (mask2_ver[1] == 0) ? 2 : ad_mask2[1];
        return;
    }
#endif
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


    if ((3 == consis_cnt1 || (fabs(dist_diff12 - dist_diff13) < diff2_th &&
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
        else if (fabs(dist_tmp2[0] - dist_tmp2[2]) < params.diff_th) {
            mask2_tri[0] = 0; mask2_tri[1] = 1; mask2_tri[2] = 0;
            mask1_tri[0] = 1; mask1_tri[1] = 1; mask1_tri[2] = 1;
            ad_tri_mask2[0] = 2; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 0;
        }
        else if (fabs(dist_tmp2[4] - dist_tmp2[2]) < params.diff_th) {
            mask2_tri[0] = 0; mask2_tri[1] = 1; mask2_tri[2] = 0;
            mask1_tri[0] = 1; mask1_tri[1] = 1; mask1_tri[2] = 1;
            ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 2;
        }
        else {
            ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
        }
    }
    else if ((consis_cnt2 == 3 || (fabs(dist_diff22 - dist_diff23) < diff2_th &&
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
        else if (fabs(dist_tmp1[0] - dist_tmp1[2]) < params.diff_th) {
            mask1_tri[0] = 0; mask1_tri[1] = 1; mask1_tri[2] = 0;
            mask2_tri[0] = 1; mask2_tri[1] = 1; mask2_tri[2] = 1;
            ad_tri_mask1[0] = 2; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 0;
        }
        else if (fabs(dist_tmp1[4] - dist_tmp1[2]) < params.diff_th) {
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
            if (fabs(dist_tmp1[0] - dist_tmp1[1]) >= diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (fabs(dist_tmp2[2] - dist_tmp2[0]) < diff_th) {
                    ad_tri_mask2[0] = 2; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 0;
                }
            }
            else if (fabs(dist_tmp2[2] - dist_tmp2[0]) >= diff_th) {
                ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
            }
        }
        else if (mask_c1[2]) {
            if (fabs(dist_tmp1[3] - dist_tmp1[4]) >= diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (fabs(dist_tmp2[2] - dist_tmp2[4]) < diff_th) {
                    ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 2;
                }
            }
            else if (fabs(dist_tmp2[2] - dist_tmp2[4]) >= diff_th) {
                ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
            }
        }
    }
    else if (mask_cross[1] && (mask_c2_sum == 2)) {
        if (mask_c2[0] > 0) {
            if (fabs(dist_tmp2[0] - dist_tmp2[1]) >= params.diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (fabs(dist_tmp1[2] - dist_tmp1[0]) < params.diff_th) {
                    ad_tri_mask1[0] = 2; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 0;
                }
            }
            else if (fabs(dist_tmp1[2] - dist_tmp1[0]) >= params.diff_th) {
                ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 1; ad_tri_mask1[2] = 0;
            }
        }
        else if (mask_c2[2]) {
            if (fabs(dist_tmp2[3] - dist_tmp2[4]) >= params.diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (fabs(dist_tmp1[2] - dist_tmp1[4]) < params.diff_th) {
                    ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 2;
                }
            }
            else if (fabs(dist_tmp1[2] - dist_tmp1[4]) >= params.diff_th) {
                ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 1; ad_tri_mask1[2] = 0;
            }
        }
    }
    else if (mask_cross[1]) {
        mask1_tri[1] = 1;
        mask2_tri[1] = 1;
    }
    else if (fabs(dist_tmp1[1] - dist_tmp2[3]) < params.diff_th &&
        dist_tmp1[1] > 0 && dist_tmp2[3] > 0) {
        mask1_tri[0] = 1;
        mask2_tri[2] = 1;
    }
    else if (fabs(dist_tmp2[1] - dist_tmp1[3]) < params.diff_th &&
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

/*******************************************************************************
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
*******************************************************************************/
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
        const int window_size = (UpSamplingParam.WH << 1) + 1;

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
            if (row_idx > UpSamplingParam.WH - 1 && row_idx < VIEW_H - UpSamplingParam.WH) {
                const int start_row = row_idx - UpSamplingParam.WH;
                for (int i = 0; i < window_size; ++i) {
                    const int src_row = start_row + i;
                    dist_tmp1[i] = dist_wave0_buffer5[up_smpl_col_buffer][src_row];
                    dist_tmp2[i] = dist_wave0_buffer5[up_smpl_col2_buffer][src_row];
                    ref_tmp1[i] = refl_wave0_buffer5[up_smpl_col_buffer][src_row];
                    ref_tmp2[i] = refl_wave0_buffer5[up_smpl_col2_buffer][src_row];
                }
            }
            else if (row_idx <= UpSamplingParam.WH - 1) {
                const int valid_rows = row_idx + UpSamplingParam.WH + 1;
                const int dest_start = UpSamplingParam.WH - row_idx;
                for (int i = 0; i < valid_rows; ++i) {
                    dist_tmp1[dest_start + i] = dist_wave0_buffer5[up_smpl_col_buffer][i];
                    dist_tmp2[dest_start + i] = dist_wave0_buffer5[up_smpl_col2_buffer][i];
                    ref_tmp1[dest_start + i] = refl_wave0_buffer5[up_smpl_col_buffer][i];
                    ref_tmp2[dest_start + i] = refl_wave0_buffer5[up_smpl_col2_buffer][i];
                }
            }
            else {
                const int valid_rows = UpSamplingParam.WH + (VIEW_H - row_idx);
                const int src_start = row_idx - UpSamplingParam.WH;
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
            rebuildFunc(dist_tmp1, dist_tmp2, mask1, mask2, ad_mask1, ad_mask2);

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
                w1[i] = UpSamplingParam.weight1[i] * mask1[i];
                w2[i] = UpSamplingParam.weight2[i] * mask2[i];
                w1_sum += w1[i];
                w2_sum += w2[i];
            }
            const int total_weight = w1_sum + w2_sum;

            int dist_ins = 0, ref_ins = 0;
            if (total_weight > 0) {
                // 合并dist和ref的项计算
                int term_dist1 = 0, term_dist2 = 0;
                int term_ref1 = 0, term_ref2 = 0;
                for (int i = 0; i < 3; ++i) {
                    term_dist1 += w1[i] * dist_c1[i];
                    term_dist2 += w2[i] * dist_c2[i];
                    term_ref1 += w1[i] * ref_c1[i];
                    term_ref2 += w2[i] * ref_c2[i];
                }

                const int coef = UpSamplingParam.coef_inv[total_weight - 1];
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
            if(dist_wave0_buffer5[up_smpl_col_buffer][row_idx] < 0 || dist_wave0_buffer5[up_smpl_col_buffer][row_idx] > 60000)
            {
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

/*******************************************************************************
 * \brief  Take the remainder of a number
 * 
 * \param[in] a : original input data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] b : remainder input data
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return int : output remainder result
*******************************************************************************/
inline int AlgoFunction::matlabMod(int a, int b){
    return (a % b + b) % b;  // 确保结果非负
}

/*******************************************************************************
 * \brief  Frame filter main function
 * 
 * \param[in] filter_col: column index of sample point
 *                 Range: 0 - 759. Accuracy: 1.
 * \param[in] filter_col_buffer: buffer index of sample point
 *                 Range: 0 - 10. Accuracy: 1.
 * \param[in] filter_col_valid: valid index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] filter_col_neib_buf: neighbor buffer index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] filter_col_neib_vld: neighbor valid index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] empty_col: empty column index
 *                 Range: 0 - 2. Accuracy: 1.
 * \param[in] empty_out_col: empty output column index
 *                 Range: 0 - 2. Accuracy: 1.
*******************************************************************************/
void AlgoFunction::frameFilter(int filter_col,
                               int filter_col_buffer,
                               int filter_col_valid,
                               int* filter_col_neib_buf,
                               int* filter_col_neib_vld,
                               int empty_col,
                               int empty_out_col) {
    // 环形缓冲区索引计算函数

    // 初始化滑动窗口缓冲区
    if (filter_col >= 0 && filter_col < algo_Param.ViewField_Wide)
    {
        if (algo_Param.FilterOn)
        {
            for (int point_row = 0; point_row < VIEW_H; ++point_row) {
                const int dist_cur = dist_wave0_buffer1[filter_col_buffer][point_row];
                const int filter_valid_cur = filter_valid_buffer[filter_col_valid][point_row];

                // 空掩码生成
                int empty_mask_cur = 0;
                if (dist_cur < Filter_params.multi_frame_dist_th) {
                    empty_mask_cur = -(((dist_cur * 20) >> 13) * 2 + 1); // 优化为位操作
                }
                else {
                    empty_mask_cur = (((dist_cur - Filter_params.multi_frame_dist_th) * 14) >> 13) * 2 + 1;
                }

                // 距离段索引检测（优化位操作）
                int dist_segment_idx = 1;
                const uint16_t s = (dist_cur >> 12); // 等价于除以4096
                for (int i = 4; i >= 0; --i) {
                    if ((s & (1 << i)) && (dist_segment_idx == 1)) {
                        dist_segment_idx = i + 2;
                        break; // 找到最高位立即退出
                    }
                }

                // 高度投影计算（使用预计算参数）
                const int height_cur_real = high_wave0_buffer1[filter_col_buffer][point_row];

                // 高度段索引（带饱和处理）
                const int height_segment_idx = std::max(0, std::min(static_cast<int>(std::floor(std::abs(height_cur_real) / 512.0)), 4));

                // 水平位置分区（分支预测优化）
                int hor_pos_segment_idx;
                if (pc_Param.reverse_mask) {
                    hor_pos_segment_idx = ((algo_Param.ViewField_Wide - filter_col - 1) >> 8) + 1;
                }
                else {
                    hor_pos_segment_idx = ((filter_col + 1) >> 8) + 1;
                }
                hor_pos_segment_idx = std::max(1, std::min(hor_pos_segment_idx, 6));

                if (dist_cur > 0)
                {
                    if (1 != filter_valid_cur)
                    {
                        // 参数获取
                        const int Range_Threshold = Filter_params.DiffRhresholdList[dist_segment_idx - 1];
                        int ValidNum = Filter_params.ValidNumList[height_segment_idx][dist_segment_idx - 1];
                        int FrameValidNum = Filter_params.FrameValidNumList[height_segment_idx][dist_segment_idx - 1];
                        int max_diff_unit = Filter_params.DiffUnitList[dist_segment_idx - 1];
                        int Range_High = dist_cur + Range_Threshold;
                        int Range_Low = std::max(1, dist_cur - Range_Threshold);

                        // 滑动窗口初始化
                        const int window_size = 2 * Filter_params.filter_L + 1;
                        int dist_neighbour[filter_valid_size][filter_valid_size] = { 0 };
                        int empty_mask_neib[filter_valid_size][filter_valid_size] = { 0 };
                        int valid_mask[filter_valid_size][filter_valid_size] = { 0 };

                        // 边界范围计算
                        int row_range_up = Filter_params.filter_L;
                        int row_range_down = Filter_params.filter_L;
                        int col_range_left = Filter_params.filter_L;
                        int col_range_right = Filter_params.filter_L;

                        if (point_row < Filter_params.filter_L)
                            row_range_up = point_row;
                        if (point_row >= VIEW_H - Filter_params.filter_L)
                            row_range_down = VIEW_H - point_row - 1;
                        if (filter_col < Filter_params.filter_L)
                            col_range_left = filter_col;
                        if (filter_col >= algo_Param.ViewField_Wide - Filter_params.filter_L)
                            col_range_right = algo_Param.ViewField_Wide - filter_col - 1;

                        // 滑动窗口数据填充
                        const int row_start = point_row - row_range_up;
                        const int col_start = filter_col - col_range_left;

                        // Copy block from dist_wave0_buffer1 to dist_neighbour
                        for (int i = 0; i < row_range_up + row_range_down + 1; i++) {
                            for (int j = 0; j < window_size; j++) {
                                dist_neighbour[0 + j][Filter_params.filter_L - row_range_up + i] =
                                    dist_wave0_buffer1[filter_col_neib_buf[j]][row_start + i];
                            }
                        }

                        // Copy block from EmptyMask to empty_mask_neib
                        for (int i = 0; i < row_range_up + row_range_down + 1; i++) {
                            for (int j = 0; j < col_range_left + col_range_right + 1; j++) {
                                empty_mask_neib[Filter_params.filter_L - col_range_left + j][Filter_params.filter_L - row_range_up + i] =
                                    EmptyMask[col_start + j][row_start + i];
                            }
                        }


                        // 严格条件判断
                        int strict_con = 0;
                        if (height_cur_real > 300 &&
                            zone_valid_cnt_in[height_segment_idx][hor_pos_segment_idx - 1][dist_segment_idx - 1] < Filter_params.zone_cnt_th) {
                            strict_con = 1;
                            max_diff_unit = 1;
                            ValidNum = ValidNum + 1;
                            FrameValidNum = FrameValidNum + 1;
                        }

                        // 邻域统计分析
                        int neib_cnt = 0;
                        int frame_valid_cnt = 0;

    // #pragma omp parallel for collapse(2) reduction(+:neib_cnt, frame_valid_cnt)
                        for (int i = -Filter_params.filter_L; i <= Filter_params.filter_L; ++i) {
                            for (int j = -Filter_params.filter_L; j <= Filter_params.filter_L; ++j) {
                                const int idx_i = Filter_params.filter_L + i;
                                const int idx_j = Filter_params.filter_L + j;

                                // 距离范围检查
                                if (dist_neighbour[idx_j][idx_i] >= Range_Low && dist_neighbour[idx_j][idx_i] <= Range_High) {
                                    valid_mask[idx_j][idx_i] = 1;
                                    neib_cnt++;
                                }

                                // 空掩码验证
                                const int mask_diff = std::abs(empty_mask_neib[idx_j][idx_i] - empty_mask_cur);
                                if (empty_mask_neib[idx_j][idx_i] != 0 &&
                                    (std::ceil((mask_diff) / 2.0) <= max_diff_unit) &&
                                    (!strict_con || matlabMod(empty_mask_neib[idx_j][idx_i], 2) == 1)) {
                                    frame_valid_cnt++;
                                }
                            }
                        }

                        // 有效性判定及更新
                        if (!(neib_cnt <= ValidNum && frame_valid_cnt <= FrameValidNum)) {
    // #pragma omp parallel for collapse(2)
                            for (int i = -Filter_params.filter_L; i <= Filter_params.filter_L; ++i) {
                                for (int j = -Filter_params.filter_L; j <= Filter_params.filter_L; ++j) {
                                    if (valid_mask[Filter_params.filter_L + j][Filter_params.filter_L + i] == 1) {
                                        const int target_col = filter_col_neib_vld[Filter_params.filter_L + j];
                                        filter_valid_buffer[target_col][point_row + i] = 1;
                                    }
                                }
                            }
                        }
                    }
                    empty_buffer[empty_col][point_row] = empty_mask_cur;
                }
                else
                {
                    // 空缓冲区更新
                    if (matlabMod(EmptyMask[filter_col][point_row] , 2) == 1) {
                        empty_buffer[empty_col][point_row] = floor(EmptyMask[filter_col][point_row] / 2.0) * 2;
                    }
                }
                if (filter_valid_buffer[filter_col_valid][point_row] == 1) {
                    //condition_met = true;
                    zone_valid_cnt[height_segment_idx][hor_pos_segment_idx - 1][dist_segment_idx - 1]++;
                }
            }

            if (pc_Param.reverse_mask) {
                if (filter_col == algo_Param.ViewField_Wide - 1) {
                    for (int c = -Filter_params.filter_L; c <= 0; ++c) {
                        const int empty_c = (empty_col  + 1 + c - 1 + empty_size) % empty_size;
                        const int target_col = -c; // reverse_mask为true且filter_col在末尾
                        if (target_col >= 0 && target_col < algo_Param.ViewField_Wide) {
                            for (int i = 0; i < VIEW_H; i++) {  // num_rows should be the row count of both matrices
                                EmptyMask_out[target_col][i] = empty_buffer[empty_c][i];
                            }
                        }
                    }
                }
                else if (filter_col >= Filter_params.filter_L) {
                    const int target_col = algo_Param.ViewField_Wide - filter_col + Filter_params.filter_L - 1;
                    if (target_col >= 0 && target_col < algo_Param.ViewField_Wide) {
                        for (int i = 0; i < VIEW_H; i++) {  // num_rows should be the row count of both matrices
                            EmptyMask_out[target_col][i] = empty_buffer[empty_out_col][i];
                        }
                    }
                }
            }
            else {
                if (filter_col == algo_Param.ViewField_Wide - 1) {
                    for (int c = -Filter_params.filter_L; c <= 0; ++c) {
                        const int empty_c = (empty_col + 1 + c - 1 + empty_size) % empty_size;
                        const int target_col = filter_col + c;
                        if (target_col >= 0 && target_col < algo_Param.ViewField_Wide) {
                            for (int i = 0; i < VIEW_H; i++) {  // num_rows should be the row count of both matrices
                                EmptyMask_out[target_col][i] = empty_buffer[empty_c][i];
                            }
                        }
                    }
                }
                else if (filter_col >= Filter_params.filter_L) {
                    const int target_col = filter_col - Filter_params.filter_L;
                    if (target_col >= 0 && target_col < algo_Param.ViewField_Wide) {
                        for (int i = 0; i < VIEW_H; i++) {  // num_rows should be the row count of both matrices
                            EmptyMask_out[target_col][i] = empty_buffer[empty_out_col][i];
                        }
                    }
                }
            }
        }
        else {
            // 第三部分：如果条件未满足，设置整个列
            for (int row_idx = 0; row_idx < VIEW_H; row_idx++)
            {
                filter_valid_buffer[filter_col_valid][row_idx] = 1;
            }
        }
    }
}


/*******************************************************************************
 * \brief  Denoise pre-processing
 * 
 * \param[in] denoise_col: column index of sample point
 *                 Range: 0 - 759. Accuracy: 1.
 * \param[in] denoise_col_buffer: buffer index of sample point
 *                 Range: 0 - 10. Accuracy: 1.
 * \param[in] denoise_col_neib_buf: neighbor buffer index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] valid_pre_2: neighbor valid index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] denoise_refer_mask: refer mask of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] valid_pre_mask: neighbor valid index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
*******************************************************************************/
void AlgoFunction::denoisePreProc(int denoise_col,
                                  int denoise_col_buffer,
                                  int* denoise_col_neib_buf,
                                  int* valid_pre_2,
                                  int* denoise_refer_mask,
                                  int (*valid_pre_mask)[5])
{
    const int HL = 5;
    const int half_HL = HL / 2;
    const int view_width = algo_Param.ViewField_Wide;
    const int last_row = VIEW_H - 1;

    // 提前检查条件，避免无效计算
    if (!(denoise_col >= 0 && denoise_col < view_width && algo_Param.DenoiseOn)) {
        return;
    }

    // 预加载常用参数
    const auto& params = noise_params;
    const int block_size = BlockSize;
    const int dist_seg_div = 3000 * 4; // 合并除法运算

    for (int row_idx = 0; row_idx < last_row; ++row_idx)
    {
        const int dist_pre = dist_wave0_buffer4[denoise_col_buffer][row_idx];

        // 跳过无效距离值
        if (dist_pre <= 0) continue;

        // 区块和距离段计算优化
        const int block_id = (row_idx + block_size) / block_size - 1;
        const int dist_seg = (dist_pre / dist_seg_div);
        const int diff_th = params.denoise_offset[dist_seg]; // 数组访问优化

        // 邻域距离值预取
        int dist_pre_buf[5];
        for (int i = 0; i < HL; ++i) {
            dist_pre_buf[i] = dist_wave0_buffer4[denoise_col_neib_buf[i]][row_idx];
        }

        // 邻域有效性统计优化
        int valid_cnt = 0;
        for (int i = 0; i < HL; ++i)
        {
            bool is_valid = (std::abs(dist_pre_buf[i] - dist_pre) <= diff_th);
            valid_pre_mask[row_idx][i] = is_valid;
            valid_cnt += is_valid;
        }

        // 两侧有效性统计
        valid_pre_2[row_idx] = valid_pre_mask[row_idx][half_HL - 1]
                            + valid_pre_mask[row_idx][half_HL + 1];

        // 参考掩码设置
        const int region = params.region_list[block_id] - 1;
        const int threshold = params.denoise_point[dist_seg][region][1];

        denoise_refer_mask[row_idx] = (valid_pre_2[row_idx] >= threshold)
                                    ? ((valid_cnt > 2) ? 2 : 1)
                                    : 3;
    }
    denoise_refer_mask[VIEW_H - 1] = 1; // 最后一个点强制为有效
}

/*******************************************************************************
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
 * \param[in] denoise_refer_mask: refer mask of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] valid_pre_2: neighbor valid index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] valid_pre_mask: neighbor valid index of sample point
 *                 Range: 0 - 2^32-1. Accuracy: 1.
*******************************************************************************/
void AlgoFunction::denoiseProcessing(int denoise_col_buffer,
    int denoise_col_valid,int* denoise_col_neib_buf,
    int* denoise_col_neib_vld,int col_idx,
    int* denoise_refer_mask,int* valid_pre_2, int (*valid_pre_mask)[5])
{
    const int current_col = col_idx;
    const int win_len_v = noise_params.WinLenV;
    const int win_len_h = noise_params.WinLenH;

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

    // 主处理循环
    for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
        const int dist_cur = dist_wave0_buffer4[denoise_col_buffer][row_idx];
        const int refer_mask = denoise_refer_mask[row_idx];

        // 跳过无效情况
        if (dist_cur <= 0) continue;

        if (refer_mask == 1) {
            // 邻域窗口构建优化
            int dist_neib[5][3] = {0};
            int valid_neib[5][3] = {0};

            // 预计算常用值
            const int dist_seg = (dist_cur / 3000) / 4;
            const int diff_th = noise_params.denoise_offset[dist_seg];
            const int block_id = row_idx / BlockSize;
            const int region = noise_params.region_list[block_id] - 1;
            const int threshold = noise_params.denoise_point[dist_seg][region][0];
            const int threshold1 = noise_params.denoise_point[dist_seg][region][1];
            const int threshold2 = noise_params.denoise_point[dist_seg][region][2];

            // 邻域分析优化
            int NeibValidNum[3] = {0};
            int ValidMask[5][3] = {0};

            for (int dv = -win_len_v; dv <= win_len_v; dv++) {
                if ((dv == 0) && (row_idx != (VIEW_H - 1))) continue;
                const int row_neib = row_idx + dv;
                if (row_neib < 0 || row_neib >= VIEW_H) continue;

                const int v_idx = dv + win_len_v;
                for (int dh = -win_len_h; dh <= win_len_h; dh++) {
                    const int h_idx = dh + win_len_h;
                    const int col_buf_idx = denoise_col_neib_buf[h_idx];

                    dist_neib[h_idx][v_idx] = dist_wave0_buffer4[col_buf_idx][row_neib];
                    valid_neib[h_idx][v_idx] = denoise_valid_buffer[col_buf_idx][row_neib];

                    // 有效性判断
                    const int nei_val = dist_neib[h_idx][v_idx];
                    if (std::abs(nei_val - dist_cur) <= diff_th && (valid_neib[h_idx][v_idx] != 6)) {
                        const int zone = noise_params.zone_matrix[v_idx][h_idx];
                        if (zone > 0 && zone <= 3) {
                            NeibValidNum[zone - 1]++;
                            ValidMask[h_idx][v_idx] = 1;
                        }
                    }
                }
            }

            // 去噪条件判断优化
            if (((NeibValidNum[0] + valid_pre_2[row_idx]) >= threshold)
                || (row_idx == (VIEW_H - 1) && (NeibValidNum[1] >= threshold1))
                || (NeibValidNum[0] + NeibValidNum[1] >= threshold)
                || (NeibValidNum[2] + NeibValidNum[1] >= threshold2)) {
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
        else if (refer_mask == 2) {
            // 直接对左右补点优化
            denoise_valid_buffer[denoise_col_valid][row_idx] = 1;

            for (int j = -noise_params.WinLenH; j <= noise_params.WinLenH; j++) {
                if (valid_pre_mask[row_idx][noise_params.WinLenH + j] == 1) {
                    int idx = denoise_col_neib_vld[3 + j - 1];
                    denoise_valid_buffer[idx][row_idx] = 1;
                }
            }
        }
    }
}

/*******************************************************************************
 * \brief  Matrix calculate inverse function
 * 
 * \param[in] mat: input matrix
 *              Range: N/A. Accuracy: N/A.
 * \param[out] inv: output inverse matrix
 *              Range: N/A. Accuracy: N/A.
*******************************************************************************/
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

/*******************************************************************************
 * \brief  Matrix calculate vector mutilply function
 * 
 * \param[in] A: input matrix
 *              Range: N/A. Accuracy: N/A.
 * \param[in] v: input vector
 *              Range: N/A. Accuracy: N/A.
 * \param[out] result: output mutilply matrix
 *              Range: N/A. Accuracy: N/A.
*******************************************************************************/
void AlgoFunction::matVecMulti(const Matrix3x3& A, const Vector3& v, Vector3& result) {
    // 矩阵-向量乘法: A(3x3) * v(3x1) -> result(3x1)
    result[0] = A(0,0)*v[0] + A(0,1)*v[1] + A(0,2)*v[2];
    result[1] = A(1,0)*v[0] + A(1,1)*v[1] + A(1,2)*v[2];
    result[2] = A(2,0)*v[0] + A(2,1)*v[1] + A(2,2)*v[2];
}

/*******************************************************************************
 * \brief  Select ground fit main function
 * 
 * \param[out] best_model: output best model result
 *              Range: N/A. Accuracy: N/A.
*******************************************************************************/
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
    
    // return best_model;
}

// 高性能环形索引计算器（适配MATLAB的1-based索引模式）
/*******************************************************************************
 * \brief  Calculate circular index of the buffer
 * 
 * \param[in] rear: rear index of the buffer
 *              Range: N/A. Accuracy: N/A.
 * \param[in] size: size of the buffer
 *              Range: N/A. Accuracy: N/A.
*******************************************************************************/
inline void AlgoFunction::circularCalcIdx(int& rear, int size)
{
    rear = (rear + 1 + size) % size;
}

/*******************************************************************************
 * \brief  Calculate height of the buffer
 * 
 * \param[in] pu16Dist: distance buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] col: column index of the buffer
 *                 Range: 0 - 759. Accuracy: 1.
 * \param[in] fit_high: height of the buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
*******************************************************************************/
void AlgoFunction::highCalcFunc(uint16_t *pu16Dist, int col, int* fit_high)
{
    int org_high =  0 ;
    int fit_ground_high = 0 ;
    const float a = fit_Params.a;
    const float b = fit_Params.b;
    const float c = fit_Params.c;

    for (int i = 0; i < VIEW_H; ++i) {
        float dist_val = pu16Dist[i];
        float Ix_val = Ix_in[i][col];
        float Iy_val = Iy_in[i][col];
        float Iz_val = Iz_in[i][col];

        org_high = std::floor(dist_val * Iz_val / 32768.0);
        fit_ground_high = std::floor(dist_val *
            (a * Ix_val / 32768.0 + b * Iy_val / 32768.0) + c);
        fit_high[i] = org_high - fit_ground_high;
    }
}

/*******************************************************************************
 * \brief  Final decision of the algorithm, delete points with mask
 * 
 * \param[in] col_idx: column index of the buffer
 *                 Range: 0 - 759. Accuracy: 1.
 * \param[in] pu16Dist: distance buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] pu8Ref: reflectance buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
*******************************************************************************/
void AlgoFunction::algoFianlDecision(int col_idx, uint16_t *pu16Dist, uint8_t *pu8Ref, tstFrameBuffer* frame_buffer)
{
    // 提前计算环形缓冲区索引
    circularCalcIdx(rear5, buffer_size_upsampling);
    const int decision_col = col_idx;
    const int decision_col_buffer = matlabMod(rear5 - 1 + 1, buffer_size_upsampling);

    // 边界检查提前
    if (decision_col >=0 && decision_col < VIEW_W) {
        // 获取当前列的指针，减少数组访问
        uint16_t *current_dist_wave0 = stFilterFrmBuffer.dist_wave0[decision_col];
        uint8_t *current_refl_wave0 = stFilterFrmBuffer.refl_wave0[decision_col];

        uint16_t *buffer_dist_wave0 = dist_wave0_buffer5[decision_col_buffer];
        uint8_t *buffer_refl_wave0 = refl_wave0_buffer5[decision_col_buffer];

        // 一次性拷贝整列数据
        memcpy(buffer_dist_wave0, current_dist_wave0, sizeof(uint16_t) * VIEW_H);
        memcpy(buffer_refl_wave0, current_refl_wave0, sizeof(uint8_t) * VIEW_H);

        // 获取掩码指针
        int *trail_mask_out = trail_mask_out_frm[decision_col];
        int *denoise_mask_out = denoise_mask_out_frm[decision_col];

        // 优化处理循环
        for (int row_idx = 0; row_idx < VIEW_H; row_idx++)
        {
            const int trail_mask = trail_mask_out[row_idx];
            const int denoise_mask = denoise_mask_out[row_idx];

            // 按照优先级处理不同掩码情况
            if (trail_mask == 1 || denoise_mask == 0) {
                // 只清空wave0数据
                buffer_dist_wave0[row_idx] = 0;
                buffer_refl_wave0[row_idx] = 0;
            }
        }
    }

    // 上采样处理
    const int up_smpl_col = decision_col - UpSamplingDelayCol;
    const int up_smpl_col_buffer = matlabMod(decision_col_buffer + 1 - UpSamplingDelayCol - 1, buffer_size_upsampling);
    const int up_smpl_col2_buffer = matlabMod(up_smpl_col_buffer + 2 - 1, buffer_size_upsampling);

    processUpSample(up_smpl_col, up_smpl_col_buffer, up_smpl_col2_buffer, pu16Dist, pu8Ref, frame_buffer);
}

/*******************************************************************************
 * \brief  Update ground fit params
*******************************************************************************/
void AlgoFunction::updateGroundFitParams(void)
{
    //更新地面拟合结果
    fit_Params.a = best_model_fit[0];
    fit_Params.b = best_model_fit[1];
    fit_Params.c = best_model_fit[2];
}

/*******************************************************************************
 * \brief  Algorithm params initialize
*******************************************************************************/
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
        algo_Param.HighRefCrossOn = algo_switch_param.enable_high_ref_cross;
        algo_Param.FilterOn = algo_switch_param.enable_filter;
        algo_Param.HorDistSmoothOn = algo_switch_param.enable_hor_dist_smooth;
        algo_Param.HorRefSmoothOn = algo_switch_param.enable_hor_ref_smooth;
        algo_Param.VerDistSmoothOn = algo_switch_param.enable_ver_dist_smooth;
        algo_Param.VerRefSmoothOn = algo_switch_param.enable_ver_ref_smooth;
    } else {
        LogWarn(__FILE__, __LINE__, __func__,
            "Algorithm switch yaml file read FAILED, Using default switch.");
    }

    LogDebug(__FILE__, __LINE__, __func__,
            "TrailRemoveOn: {}", algo_Param.TrailRemoveOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "DenoiseOn: {}", algo_Param.DenoiseOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "HighRefCrossOn: {}", algo_Param.HighRefCrossOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "FilterOn: {}", algo_Param.FilterOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "HorDistSmoothOn: {}", algo_Param.HorDistSmoothOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "HorRefSmoothOn: {}", algo_Param.HorRefSmoothOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "VerDistSmoothOn: {}", algo_Param.VerDistSmoothOn);
    LogDebug(__FILE__, __LINE__, __func__,
            "VerRefSmoothOn: {}", algo_Param.VerRefSmoothOn);
}

/*******************************************************************************
 * \brief  Algorithm frame change
*******************************************************************************/
void AlgoFunction::algoFrameChange(void)
{
    memset(denoise_valid_buffer, 0, sizeof(denoise_valid_buffer));
    memset(filter_valid_buffer, 0, sizeof(filter_valid_buffer));
    memset(empty_buffer, 0, sizeof(empty_buffer));

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
    memset(high_wave0_buffer4, 0, sizeof(high_wave0_buffer4));

    memset(dist_wave0_buffer5, 0, sizeof(dist_wave0_buffer5));
    memset(dist_wave1_buffer5, 0, sizeof(dist_wave1_buffer5));
    memset(refl_wave0_buffer5, 0, sizeof(refl_wave0_buffer5));
    memset(refl_wave1_buffer5, 0, sizeof(refl_wave1_buffer5));

    memcpy(zone_valid_cnt_in, zone_valid_cnt,sizeof(zone_valid_cnt));
    memset(zone_valid_cnt, 0, sizeof(zone_valid_cnt));

    memcpy(EmptyMask, EmptyMask_out,sizeof(EmptyMask_out));
    memset(EmptyMask_out, 0, sizeof(EmptyMask_out));

    rear1 = -1; //去噪
    rear2 = -1; //拖点
    rear4 = -1; //帧间
    rear5 = -1; //上采样

    basic_rear = -1;
    filter_rear = -1;
    empty_rear = -1;
}

/*******************************************************************************
 * \brief  Algorithm main function
 * 
 * \param[in] col_idx: column index of the buffer
 *                 Range: 0 - 759. Accuracy: 1.
 * \param[in] pstFrameBuffer: frame buffer
 *                 Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] task_id: thread id of algorithm
 *                 Range: 0 - 4. Accuracy: 1.
*******************************************************************************/
int AlgoFunction::pcAlgoMainFunc(int col_idx, tstFrameBuffer* pstFrameBuffer, int task_id)
{
    int proc_col = -1;
    switch (task_id)
    {
        case FILTER_ALGO:
        {
        circularCalcIdx(rear1, buffer_size_filter);
        circularCalcIdx(filter_rear, filter_valid_size);
        circularCalcIdx(empty_rear, empty_size);
        //帧间数据缓存
        int filter_col_in = col_idx - FilterInDelayCol;
        if (filter_col_in >= 0 && filter_col_in < VIEW_W){
            memcpy(dist_wave0_buffer1[rear1], pstFrameBuffer->dist0[filter_col_in], sizeof(uint16_t) * VIEW_H);
            memcpy(dist_wave1_buffer1[rear1], pstFrameBuffer->dist1[filter_col_in], sizeof(uint16_t) * VIEW_H);
            memcpy(refl_wave0_buffer1[rear1], pstFrameBuffer->ref0[filter_col_in], sizeof(uint8_t) * VIEW_H);
            memcpy(refl_wave1_buffer1[rear1], pstFrameBuffer->ref1[filter_col_in], sizeof(uint8_t) * VIEW_H);
            memcpy(attr_wave0_buffer1[rear1], pstFrameBuffer->att0[filter_col_in], sizeof(uint8_t) * VIEW_H);
            memcpy(attr_wave1_buffer1[rear1], pstFrameBuffer->att1[filter_col_in], sizeof(uint8_t) * VIEW_H);
            memcpy(high_wave0_buffer1[rear1], pstFrameBuffer->high0[filter_col_in], sizeof(int) * VIEW_H);
            memcpy(high_wave1_buffer1[rear1], pstFrameBuffer->high1[filter_col_in], sizeof(int) * VIEW_H);
        }
        else {
            //超过最大列，帧间buffer补0
            memset(dist_wave0_buffer1[rear1], 0, sizeof(uint16_t) * VIEW_H);
            memset(dist_wave1_buffer1[rear1], 0, sizeof(uint16_t) * VIEW_H);
            memset(refl_wave0_buffer1[rear1], 0, sizeof(uint8_t) * VIEW_H);
            memset(refl_wave1_buffer1[rear1], 0, sizeof(uint8_t) * VIEW_H);
            memset(attr_wave0_buffer1[rear1], 0, sizeof(uint8_t) * VIEW_H);
            memset(attr_wave1_buffer1[rear1], 0, sizeof(uint8_t) * VIEW_H);
            memset(high_wave0_buffer1[rear1], 0, sizeof(int) * VIEW_H);
            memset(high_wave1_buffer1[rear1], 0, sizeof(int) * VIEW_H);
        }
        memset(filter_valid_buffer[filter_rear], 0, sizeof(int) * VIEW_H);

        //线程1:帧间
        int filter_col = filter_col_in - FilterDelayCol;
        int filter_col_buffer = matlabMod(rear1 + 1 - FilterDelayCol - 1, buffer_size_filter);
        int filter_col_neib_buf[filter_valid_size] = { 0 };
        for (int i = 0; i < filter_valid_size; i++) {
            filter_col_neib_buf[i] = matlabMod((filter_col_buffer + 1 - Filter_params.filter_L + i) - 1, buffer_size_filter);
        }
        int filter_col_valid = matlabMod(filter_rear + 1 - FilterDelayCol - 1, filter_valid_size);
        int filter_col_neib_vld[filter_valid_size] = { 0 };
        for (int i = 0; i < filter_valid_size; i++) {
            filter_col_neib_vld[i] = matlabMod((filter_col_valid + 1 - Filter_params.filter_L + i) - 1, filter_valid_size);
        }
        int empty_col = empty_rear;
        int empty_out_col = (empty_col + 1) % empty_size;

        int filter_col_del = filter_col - FilterDelDelayCol;
        int filter_col_del_buf = matlabMod(filter_col_buffer + 1 - FilterDelDelayCol - 1, buffer_size_filter);
        int filter_col_del_vld = matlabMod(filter_col_valid + 1 - FilterDelDelayCol - 1, filter_valid_size);

        //帧间
        frameFilter(filter_col, filter_col_buffer, filter_col_valid, filter_col_neib_buf,
            filter_col_neib_vld, empty_col, empty_out_col/*, filter_col_del, filter_col_del_buf*/);

        //帧间滤波删点
        if (filter_col_del >= 0 && filter_col_del < algo_Param.ViewField_Wide) // 转换到0-based范围检查
        {
            for (int row_idx = 0; row_idx < VIEW_H; ++row_idx) {
                int valid_tmp = filter_valid_buffer[filter_col_del_vld][row_idx];
                if (valid_tmp != 1)
                {
                    dist_wave0_buffer1[filter_col_del_buf][row_idx] = 0;
                    refl_wave0_buffer1[filter_col_del_buf][row_idx] = 0;
                    high_wave0_buffer1[filter_col_del_buf][row_idx] = 0;
                    attr_wave0_buffer1[filter_col_del_buf][row_idx] = 0;
                }
            }
        }

        if(col_idx >= 0 && col_idx < VIEW_W){
            if((col_idx % gnd_step) == 0){
                uint16_t dist_line[VIEW_H];
                uint16_t col_pos = col_idx / gnd_step;
                memcpy(dist_line, pstFrameBuffer->dist0[col_idx], sizeof(uint16_t) * VIEW_H);

                int surface_id = pstFrameBuffer->surface_id.load();
                int real_col;
                if(0 == surface_id)
                {
                    real_col = UP_VIEW_W - (col_idx << 1) - 2;
                }
                else
                {
                    real_col = (col_idx << 1) + 1;
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

        //水平平滑
        int hor_smooth_col = filter_col - HorSmoothDelayCol;//idx
        int hor_smooth_col_buffer = matlabMod(filter_col_buffer + 1 - HorSmoothDelayCol - 1, buffer_size_filter);//idx
        int hor_smooth_col_neib_buf[5] = { 0 };//idx
        for (int i = 0; i < 5; i++) {
            hor_smooth_col_neib_buf[i] = matlabMod(hor_smooth_col_buffer + 1 - Smooth_params.SmoothH + i - 1, buffer_size_filter);
        }
        hSmooth(hor_smooth_col, hor_smooth_col_buffer, hor_smooth_col_neib_buf);

        //垂直平滑
        int ver_smooth_col = hor_smooth_col - VerSmoothDelayCol;
        int ver_smooth_col_buffer = matlabMod(hor_smooth_col_buffer + 1 - VerSmoothDelayCol - 1, buffer_size_filter);//idx
        vSmooth(ver_smooth_col, ver_smooth_col_buffer);

        //线程一：：：：模块输出: 双回波距离、反射率
        int filter_col_out = ver_smooth_col - FilterOutDelayCol;
        int filter_col_out_buffer = matlabMod(ver_smooth_col_buffer + 1 - FilterOutDelayCol - 1, buffer_size_filter);//idx

        if(filter_col_out >= 0 && filter_col_out < VIEW_W)
        {
            memcpy(stFilterFrmBuffer.dist_wave0[filter_col_out], dist_wave0_buffer1[filter_col_out_buffer], sizeof(uint16_t) * VIEW_H);
            memcpy(stFilterFrmBuffer.dist_wave1[filter_col_out], dist_wave1_buffer1[filter_col_out_buffer], sizeof(uint16_t) * VIEW_H);
            memcpy(stFilterFrmBuffer.refl_wave0[filter_col_out], refl_wave0_buffer1[filter_col_out_buffer], sizeof(uint8_t) * VIEW_H);
            memcpy(stFilterFrmBuffer.refl_wave1[filter_col_out], refl_wave1_buffer1[filter_col_out_buffer], sizeof(uint8_t) * VIEW_H);
            proc_col = filter_col_out;
        }
        if(filter_col_out >= VIEW_W)
        {
            proc_col = VIEW_W - 1;
        }
        } break;
        case TRAIL_ALGO:
        {
        circularCalcIdx(rear2, buffer_size_trail);
        //拖点数据
        int trail_col_in = col_idx - TrailInDelayCol;
        if (trail_col_in >= 0 && trail_col_in < VIEW_W) {
                memcpy(dist_wave0_buffer2[rear2], pstFrameBuffer->dist0[trail_col_in], sizeof(uint16_t) * VIEW_H);
                memcpy(refl_wave0_buffer2[rear2], pstFrameBuffer->ref0[trail_col_in], sizeof(uint8_t) * VIEW_H);
        }
        else {
            memset(dist_wave0_buffer2[rear2], 0, sizeof(uint16_t) * VIEW_H);
            memset(refl_wave0_buffer2[rear2], 0, sizeof(uint8_t) * VIEW_H);
        }

        //线程二：：：：拖点
        int trail_col = trail_col_in - TrailDelayCol;  // 目标列偏移计算‌
        int trail_col_buffer = matlabMod(rear2 + 1 - TrailDelayCol - 1, buffer_size_trail);  // 环形缓冲区计算‌
        int trail_col_neib_buf[5] = { 0 };
        for (int i = 0; i < 5; i++) {
            trail_col_neib_buf[i] = matlabMod((trail_col_buffer + 1 - 2 + i) - 1, buffer_size_trail);
        }
        int trail_mask_out[VIEW_H] = { 0 };
        int trail_refer_mask[VIEW_H] = { 0 };
        /* 预处理：拖点算法 */
        trailPreProc(trail_col, trail_col_buffer,trail_col_neib_buf, trail_refer_mask);

        trailRemove(trail_col, trail_col_buffer,trail_col_neib_buf, trail_refer_mask,trail_mask_out);

        if(trail_col >= 0 && trail_col < VIEW_W)
        {
            memcpy(trail_mask_out_frm[trail_col], trail_mask_out, sizeof(trail_mask_out));
            proc_col = trail_col;
        }
        if(trail_col >= VIEW_W)
        {
            proc_col = VIEW_W - 1;
        }
        }break;
        case DENOISE_ALGO:
        {
        circularCalcIdx(rear4, buffer_size_denoise);
        circularCalcIdx(basic_rear, denoise_valid_size);

        //去噪
        int denoise_col_in = col_idx - DenoiseInDelayCol;

        if (denoise_col_in >= 0 && denoise_col_in < VIEW_W) {
            memcpy(dist_wave0_buffer4[rear4], pstFrameBuffer->dist0[denoise_col_in], sizeof(uint16_t) * VIEW_H);
            memcpy(refl_wave0_buffer4[rear4], pstFrameBuffer->ref0[denoise_col_in], sizeof(uint8_t) * VIEW_H);
            memcpy(high_wave0_buffer4[rear4], pstFrameBuffer->high0[denoise_col_in], sizeof(int) * VIEW_H);
        }
        else {
            memset(dist_wave0_buffer4[rear4], 0, sizeof(uint16_t) * VIEW_H);
            memset(refl_wave0_buffer4[rear4], 0, sizeof(uint8_t) * VIEW_H);
            memset(high_wave0_buffer4[rear4], 0, sizeof(int) * VIEW_H);
        }
        memset(denoise_valid_buffer[basic_rear], 0, sizeof(int) * VIEW_H);

        int valid_pre_mask[VIEW_H][5] = { 0 };
        int denoise_col = denoise_col_in - DenoiseDelayCol;
        int denoise_col_buffer = matlabMod(rear4 + 1 - DenoiseDelayCol - 1, buffer_size_denoise);
        int denoise_col_neib_buf[5];//索引
        for (int i = 0; i < 5; i++) {
            denoise_col_neib_buf[i] = matlabMod(denoise_col_buffer + 1 - 2 + i - 1, buffer_size_denoise);
        }
        int denoise_col_valid = matlabMod(basic_rear + 1 - DenoiseDelayCol - 1, denoise_valid_size);
        int denoise_col_neib_vld[5];//索引
        for (int i = 0; i < 5; i++) {
            denoise_col_neib_vld[i] = matlabMod(denoise_col_valid + 1 - 2 + i - 1, denoise_valid_size);
        }
        int denoise_col_out = denoise_col - DenoiseOutDelayCol;
        int denoise_col_out_vld = matlabMod(denoise_col_valid + 1 - DenoiseOutDelayCol - 1, denoise_valid_size);

        int denoise_refer_mask[VIEW_H] = { 0 };
        int valid_pre_2[VIEW_H] = { 0 };

        // 去噪算法预处理
        denoisePreProc(denoise_col, denoise_col_buffer, denoise_col_neib_buf,
            valid_pre_2, denoise_refer_mask, valid_pre_mask);

        // 去噪算法
        denoiseProcessing(denoise_col_buffer, denoise_col_valid, denoise_col_neib_buf,
            denoise_col_neib_vld, denoise_col, denoise_refer_mask, valid_pre_2, valid_pre_mask);

        if(denoise_col_out >= 0 && denoise_col_out < VIEW_W)
        {
            memcpy(denoise_mask_out_frm[denoise_col_out], denoise_valid_buffer[denoise_col_out_vld], sizeof(int) * VIEW_H);
        }

        circularCalcIdx(rear2, buffer_size_trail);
        //拖点数据
        int trail_col_in = col_idx - TrailInDelayCol;
        if (trail_col_in >= 0 && trail_col_in < VIEW_W) {
                memcpy(dist_wave0_buffer2[rear2], pstFrameBuffer->dist0[trail_col_in], sizeof(uint16_t) * VIEW_H);
                memcpy(refl_wave0_buffer2[rear2], pstFrameBuffer->ref0[trail_col_in], sizeof(uint8_t) * VIEW_H);
        }
        else {
            memset(dist_wave0_buffer2[rear2], 0, sizeof(uint16_t) * VIEW_H);
            memset(refl_wave0_buffer2[rear2], 0, sizeof(uint8_t) * VIEW_H);
        }

        //线程二：：：：拖点
        int trail_col = trail_col_in - TrailDelayCol;  // 目标列偏移计算‌
        int trail_col_buffer = matlabMod(rear2 + 1 - TrailDelayCol - 1, buffer_size_trail);  // 环形缓冲区计算‌
        int trail_col_neib_buf[5] = { 0 };
        for (int i = 0; i < 5; i++) {
            trail_col_neib_buf[i] = matlabMod((trail_col_buffer + 1 - 2 + i) - 1, buffer_size_trail);
        }
        int trail_mask_out[VIEW_H] = { 0 };
        int trail_refer_mask[VIEW_H] = { 0 };
        /* 预处理：拖点算法 */
        trailPreProc(trail_col, trail_col_buffer,trail_col_neib_buf, trail_refer_mask);

        trailRemove(trail_col, trail_col_buffer,trail_col_neib_buf, trail_refer_mask,trail_mask_out);

        if(trail_col >= 0 && trail_col < VIEW_W)
        {
            memcpy(trail_mask_out_frm[trail_col], trail_mask_out, sizeof(trail_mask_out));
            proc_col = trail_col;
        }
        if(trail_col >= VIEW_W)
        {
            proc_col = VIEW_W - 1;
        }
        }break;
        default:
        break;
    };

    return proc_col;
}

}   // namespace lidar
}   // namespace robosense