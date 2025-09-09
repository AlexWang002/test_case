/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/** [tile_buffer_allocation] */
#include "../upsample_commom_param.h"

#include <cupva_device.h> /* Main device-side header file */
#include <cupva_device_debug.h>
#include <string.h>

VMEM(B, uint16_t, inputDistBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint8_t, inputRefBufferVMEM,
    RDF_DOUBLE(uint8_t,TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

VMEM(A, uint16_t, inputDistRawBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(A, uint8_t, inputRefRawBufferVMEM, RDF_DOUBLE(uint8_t,TILE_WIDTH, TILE_HEIGHT));
/** Output do not use halo */
VMEM(B, uint16_t, outputDistOriBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, outputDistUpBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint8_t, outputRefOriBufferVMEM, RDF_DOUBLE(uint8_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint8_t, outputRefUpBufferVMEM, RDF_DOUBLE(uint8_t,TILE_WIDTH, TILE_HEIGHT));

/** [declare_algorithm_params] */
VMEM(C, int, algorithmParams, sizeof(InsertParam_t));

/* The handles that will be used for triggering and syncing tile transfers are declared
 * for both incoming source and outgoing destination data flows.
 * Host side code will use these handles when configuring the RDFs.
 */
/** declare_df_handles */
VMEM(C, RasterDataFlowHandler, InputDistDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputRefDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputDistRawDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputRefRawDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputDistOriDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputDistUpDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputRefOriDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputRefUpDataFlowHandler);

void checkConsistency(const int* dist, const int* diffs, int* mask, int diff_th) {
    if (diffs[0] < diff_th && dist[1] > 0 && dist[2] > 0) {
        mask[0] = 1;
        mask[1] = 1;
    }
    if (diffs[1] < diff_th && dist[2] > 0 && dist[3] > 0) {
        mask[1] = 1;
        mask[2] = 1;
    }
    if (diffs[2] < diff_th && dist[1] > 0 && dist[3] > 0) {
        mask[0] = 1;
        mask[2] = 1;
    }
}

void rebuildFunc(int* dist_tmp1, int* dist_tmp2, int* mask1, int* mask2, int* ad_mask1, int* ad_mask2, int* PLRawFillMask, int TileIdx, int col_idx, int row_idx)
{
    InsertParam_t params;
    for(int i = 0; i < 3; i++){
        mask1[i] = 0;
        mask2[i] = 0;
        ad_mask1[i] = 0;
        ad_mask2[i] = 0;
    }
    *PLRawFillMask = 0;
    // 参数解包到局部变量（减少结构体访问开销）
    const int diff_th = params.diff_th;
    const int diff_ratio_max = params.diff_ratio_max;
    const int diff2_th = params.diff2_th;
    const float ratio_scale = diff_ratio_max / 4096.0;

    // 替代 Eigen::Vector3i dist_c1 = dist_tmp1.segment(1, 3)
    int dist_c1[3] = { dist_tmp1[1], dist_tmp1[2], dist_tmp1[3] };  // 注意原始数据索引偏移
    int dist_c2[3] = { dist_tmp2[1], dist_tmp2[2], dist_tmp2[3] };

    // 计算距离差值（展开所有计算）
    if (ABS(dist_c1[2] - dist_c2[2]) < ((dist_c1[2]*diff_ratio_max) >> 12) && dist_c1[2] > 0)
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
        ABS(dist_diff11), ABS(dist_diff12), ABS(dist_diff13),
        ABS(dist_diff21), ABS(dist_diff22), ABS(dist_diff23)
    };

    // 交叉差异计算
    const int abs_cross_diff[3] = {
        ABS(dist_tmp1[1] - dist_tmp2[1]),
        ABS(dist_tmp1[2] - dist_tmp2[2]),
        ABS(dist_tmp1[3] - dist_tmp2[3])
    };

    // 阈值计算（展开为数组操作）
    int diff_th_max1[3], diff_th_max2[3];
    for (int i = 0; i < 3; ++i) {
        diff_th_max1[i] = (int)(dist_c1[i] * ratio_scale);
        diff_th_max2[i] = (int)(dist_c2[i] * ratio_scale);
    }

    // 初始化掩码数组（使用基础类型替代Eigen向量）
    int mask1_hor[3] = { 0 }, mask2_hor[3] = { 0 };
    int mask1_tri[3] = { 0 }, mask2_tri[3] = { 0 };
    int ad_tri_mask1[3] = { 0 }, ad_tri_mask2[3] = { 0 };

    // 调用检测函数（参数需要重组）
    const int diffs1[3] = { abs_dist_diff[0], abs_dist_diff[1], abs_dist_diff[2] };
    const int diffs2[3] = { abs_dist_diff[3], abs_dist_diff[4], abs_dist_diff[5] };
    checkConsistency(dist_tmp1, diffs1, mask1_hor, diff_th);
    checkConsistency(dist_tmp2, diffs2, mask2_hor, diff_th);

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
    const bool main_condition = (consis_cnt1 >= 2 && consis_cnt2 >= 2);

    if (main_condition) {
        for(int i = 0; i < 3; i++){
            mask1[i] = mask1_hor[i];
            mask2[i] = mask2_hor[i];
        }
        // memcpy(mask1, mask1_hor, sizeof(mask1_hor));
        // memcpy(mask2, mask2_hor, sizeof(mask2_hor));

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
    const int edge_diff1_0 = ABS(dist_tmp1[0] - dist_tmp1[1]);
    const int edge_diff1_3 = ABS(dist_tmp1[3] - dist_tmp1[4]);
    const int edge_diff2_0 = ABS(dist_tmp2[0] - dist_tmp2[1]);
    const int edge_diff2_3 = ABS(dist_tmp2[3] - dist_tmp2[4]);

    int mask_e1[3] = { edge_diff1_0 < diff_th, 65535 < diff_th, edge_diff1_3 < diff_th };
    int mask_e2[3] = { edge_diff2_0 < diff_th, 65535 < diff_th, edge_diff2_3 < diff_th };

    // 主条件判断逻辑（优化条件结构）
    const int mask_cross_sum = mask_cross[0] + mask_cross[1] + mask_cross[2];
    const int mask_c1_sum = mask_c1[0] + mask_c1[1] + mask_c1[2];
    const int mask_c2_sum = mask_c2[0] + mask_c2[1] + mask_c2[2];


    if ((3 == consis_cnt1 || (ABS(dist_diff12 - dist_diff13) < diff2_th &&
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
        else if (ABS(dist_tmp2[0] - dist_tmp2[2]) < params.diff_th) {
            mask2_tri[0] = 0; mask2_tri[1] = 1; mask2_tri[2] = 0;
            mask1_tri[0] = 1; mask1_tri[1] = 1; mask1_tri[2] = 1;
            ad_tri_mask2[0] = 2; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 0;
        }
        else if (ABS(dist_tmp2[4] - dist_tmp2[2]) < params.diff_th) {
            mask2_tri[0] = 0; mask2_tri[1] = 1; mask2_tri[2] = 0;
            mask1_tri[0] = 1; mask1_tri[1] = 1; mask1_tri[2] = 1;
            ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 2;
        }
        else {
            ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
        }
    }
    else if ((consis_cnt2 == 3 || (ABS(dist_diff22 - dist_diff23) < diff2_th &&
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
        else if (ABS(dist_tmp1[0] - dist_tmp1[2]) < params.diff_th) {
            mask1_tri[0] = 0; mask1_tri[1] = 1; mask1_tri[2] = 0;
            mask2_tri[0] = 1; mask2_tri[1] = 1; mask2_tri[2] = 1;
            ad_tri_mask1[0] = 2; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 0;
        }
        else if (ABS(dist_tmp1[4] - dist_tmp1[2]) < params.diff_th) {
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
            if (ABS(dist_tmp1[0] - dist_tmp1[1]) >= diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (ABS(dist_tmp2[2] - dist_tmp2[0]) < diff_th) {
                    ad_tri_mask2[0] = 2; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 0;
                }
            }
            else if (ABS(dist_tmp2[2] - dist_tmp2[0]) >= diff_th) {
                ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
            }
        }
        else if (mask_c1[2]) {
            if (ABS(dist_tmp1[3] - dist_tmp1[4]) >= diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (ABS(dist_tmp2[2] - dist_tmp2[4]) < diff_th) {
                    ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 0; ad_tri_mask2[2] = 2;
                }
            }
            else if (ABS(dist_tmp2[2] - dist_tmp2[4]) >= diff_th) {
                ad_tri_mask2[0] = 0; ad_tri_mask2[1] = 1; ad_tri_mask2[2] = 0;
            }
        }
    }
    else if (mask_cross[1] && (mask_c2_sum == 2)) {
        if (mask_c2[0] > 0) {
            if (ABS(dist_tmp2[0] - dist_tmp2[1]) >= params.diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (ABS(dist_tmp1[2] - dist_tmp1[0]) < params.diff_th) {
                    ad_tri_mask1[0] = 2; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 0;
                }
            }
            else if (ABS(dist_tmp1[2] - dist_tmp1[0]) >= params.diff_th) {
                ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 1; ad_tri_mask1[2] = 0;
            }
        }
        else if (mask_c2[2]) {
            if (ABS(dist_tmp2[3] - dist_tmp2[4]) >= params.diff_th) {
                for (int i = 0; i < 3; i++) {
                    mask1_tri[i] = mask_c1[i];
                    mask2_tri[i] = mask_c2[i];
                }
                if (ABS(dist_tmp1[2] - dist_tmp1[4]) < params.diff_th) {
                    ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 0; ad_tri_mask1[2] = 2;
                }
            }
            else if (ABS(dist_tmp1[2] - dist_tmp1[4]) >= params.diff_th) {
                ad_tri_mask1[0] = 0; ad_tri_mask1[1] = 1; ad_tri_mask1[2] = 0;
            }
        }
    }
    else if (mask_cross[1]) {
        mask1_tri[1] = 1;
        mask2_tri[1] = 1;
    }
    else if (ABS(dist_tmp1[1] - dist_tmp2[3]) < params.diff_th &&
        dist_tmp1[1] > 0 && dist_tmp2[3] > 0) {
        mask1_tri[0] = 1;
        mask2_tri[2] = 1;
    }
    else if (ABS(dist_tmp2[1] - dist_tmp1[3]) < params.diff_th &&
        dist_tmp2[1] > 0 && dist_tmp1[3] > 0) {
        mask1_tri[2] = 1;
        mask2_tri[0] = 1;
    }

    // 最终计数（使用快速求和）
    int tri_cnt1 = mask1_tri[0] + mask1_tri[1] + mask1_tri[2];
    int tri_cnt2 = mask2_tri[0] + mask2_tri[1] + mask2_tri[2];

    if (tri_cnt1 >= 1 && tri_cnt2 >= 1) {
        for(int i = 0; i < 3; i++){
            mask1[i] = mask1_tri[i];
            mask2[i] = mask2_tri[i];
            ad_mask1[i] = ad_tri_mask1[i];
            ad_mask2[i] = ad_tri_mask2[i];
        }
    }
}

CUPVA_VPU_MAIN()
{
    InsertParam_t *upsample_Param = (InsertParam_t *)algorithmParams;
    /** Calculate line pitch */
    uint16_t DistInLinePitch = cupvaRasterDataFlowGetLinePitch(InputDistDataFlowHandler);
    uint16_t DistInRawLinePitch = cupvaRasterDataFlowGetLinePitch(InputDistRawDataFlowHandler);
    uint16_t DistOutOriLinePitch = cupvaRasterDataFlowGetLinePitch(OutputDistOriDataFlowHandler);
    uint16_t DistOutUpLinePitch = cupvaRasterDataFlowGetLinePitch(OutputDistUpDataFlowHandler);
    uint8_t RefInLinePitch = cupvaRasterDataFlowGetLinePitch(InputRefDataFlowHandler);
    uint8_t RefInRawLinePitch = cupvaRasterDataFlowGetLinePitch(InputRefRawDataFlowHandler);
    uint8_t RefOutOriLinePitch = cupvaRasterDataFlowGetLinePitch(OutputRefOriDataFlowHandler);
    uint8_t RefOutUpLinePitch = cupvaRasterDataFlowGetLinePitch(OutputRefUpDataFlowHandler);

    /** Every tile offset */
    int32_t srcDistOffset = 0;
    int32_t srcDistRawOffset = 0;
    int32_t srcRefOffset = 0;
    int32_t srcRefRawOffset  = 0;
    int32_t dstDistOffset = 0;
    int32_t dstRefOffset  = 0;

    cupvaRasterDataFlowTrig(InputDistDataFlowHandler);
    cupvaRasterDataFlowTrig(InputDistRawDataFlowHandler);
    cupvaRasterDataFlowTrig(InputRefDataFlowHandler);
    cupvaRasterDataFlowTrig(InputRefRawDataFlowHandler);

    bool data_abnormal_ = false;
    /** Loop over tiles, TILE_COUNT = 10*/
    for(int TileIdx = 0; TileIdx < TILE_COUNT; TileIdx++)
    {
        /** Trigger the data flow and switch to the next tile */
        cupvaRasterDataFlowSync(InputDistDataFlowHandler);
        cupvaRasterDataFlowTrig(InputDistDataFlowHandler);
        cupvaRasterDataFlowSync(InputDistRawDataFlowHandler);
        cupvaRasterDataFlowTrig(InputDistRawDataFlowHandler);
        cupvaRasterDataFlowSync(InputRefDataFlowHandler);
        cupvaRasterDataFlowTrig(InputRefDataFlowHandler);
        cupvaRasterDataFlowSync(InputRefRawDataFlowHandler);
        cupvaRasterDataFlowTrig(InputRefRawDataFlowHandler);
#ifdef ALGO_ON
        uint16_t dist_ins_c[TILE_WIDTH] = {0};
        uint8_t ref_ins_c[TILE_WIDTH] = {0};
        /** Process the columns except for "halo" */
        for (int col_idx = 1; col_idx < TILE_HEIGHT - 1; ++col_idx) //95
        {
            if((TileIdx != TILE_COUNT - 1) && (col_idx < TILE_HEIGHT - 1))
            {
                const int window_size = (upsample_Param->WH << 1) + 1;

                for (int row_idx = 0; row_idx < TILE_WIDTH; ++row_idx)
                {
                    /** 初始化，目前不支持memset */
                    int dist_tmp1[5] = {0};
                    int dist_tmp2[5] = {0};
                    int ref_tmp1[5] = {0};
                    int ref_tmp2[5] = {0};
                    // for(int i = 0; i < window_size; i++){
                    //     dist_tmp1[i] = 0;
                    //     dist_tmp2[i] = 0;
                    //     ref_tmp1[i] = 0;
                    //     ref_tmp2[i] = 0;
                    // }
                    int dist_raw = inputDistRawBufferVMEM[srcDistRawOffset + (col_idx - 1) * DistInRawLinePitch + row_idx];
                    int ref_raw = inputRefRawBufferVMEM[srcRefRawOffset + (col_idx - 1) * RefInRawLinePitch + row_idx];
                    // if(TileIdx == 0 && col_idx == 2 && row_idx == 15){
                        // printf("dist_raw = %d, ref_raw = %d\n",dist_raw,ref_raw);
                    // }

                    // 合并数据提取逻辑
                    if (row_idx > upsample_Param->WH - 1 && row_idx < TILE_WIDTH - upsample_Param->WH) { //中间行
                        const int start_row = row_idx - upsample_Param->WH;
                        for (int i = 0; i < window_size; ++i) {
                            const int src_row = start_row + i;
                            dist_tmp1[i] = inputDistBufferVMEM[srcDistOffset + col_idx * DistInLinePitch + src_row];
                            dist_tmp2[i] = inputDistBufferVMEM[srcDistOffset + (col_idx + 1) * DistInLinePitch + src_row];
                            ref_tmp1[i] = inputRefBufferVMEM[srcDistOffset + col_idx * RefInLinePitch + src_row];
                            ref_tmp2[i] = inputRefBufferVMEM[srcDistOffset + (col_idx + 1) * RefInLinePitch + src_row];
                        }
                    }
                    else if (row_idx <= upsample_Param->WH - 1) {    //顶部边界行
                        const int valid_rows = row_idx + upsample_Param->WH + 1;
                        const int dest_start = upsample_Param->WH - row_idx;
                        for (int i = 0; i < valid_rows; ++i) {
                            dist_tmp1[dest_start + i] = inputDistBufferVMEM[srcDistOffset + col_idx * DistInLinePitch + i];
                            dist_tmp2[dest_start + i] = inputDistBufferVMEM[srcDistOffset + (col_idx + 1) * DistInLinePitch + i];
                            ref_tmp1[dest_start + i] = inputRefBufferVMEM[srcDistOffset + col_idx * RefInLinePitch + i];
                            ref_tmp2[dest_start + i] = inputRefBufferVMEM[srcDistOffset + (col_idx + 1) * RefInLinePitch + i];
                        }
                    }
                    else {
                        const int valid_rows = upsample_Param->WH + (TILE_WIDTH - row_idx);
                        const int src_start = row_idx - upsample_Param->WH;
                        for (int i = 0; i < valid_rows; ++i) {
                            const int src_row = src_start + i;
                            dist_tmp1[i] = inputDistBufferVMEM[srcDistOffset + col_idx * DistInLinePitch + src_row];
                            dist_tmp2[i] = inputDistBufferVMEM[srcDistOffset + (col_idx + 1) * DistInLinePitch + src_row];
                            ref_tmp1[i] = inputRefBufferVMEM[srcDistOffset + col_idx * RefInLinePitch + src_row];
                            ref_tmp2[i] = inputRefBufferVMEM[srcDistOffset + (col_idx + 1) * RefInLinePitch + src_row];
                        }
                    }
                    // if(TileIdx == 0 && col_idx == 2 && row_idx == 15){
                        // printf("dist_tmp1[0]=%d\n,dist_tmp1[1]=%d\n,dist_tmp1[2]=%d\n,dist_tmp1[3]=%d\n,dist_tmp1[4] =%d\n",dist_tmp1[0],dist_tmp1[1],dist_tmp1[2],dist_tmp1[3],dist_tmp1[4]);
                        // printf("dist_tmp2[0]=%d\n,dist_tmp2[1]=%d\n,dist_tmp2[2]=%d\n,dist_tmp2[3]=%d\n, dist_tmp2[4] =%d\n", dist_tmp2[0],dist_tmp2[1],dist_tmp2[2],dist_tmp2[3],dist_tmp2[4]);
                        // printf("ref_tmp1[0]=%d\n,ref_tmp1[1]=%d\n,ref_tmp1[2]=%d\n,ref_tmp1[3]=%d\n,ref_tmp1[4] =%d\n",ref_tmp1[0],ref_tmp1[1],ref_tmp1[2],ref_tmp1[3],ref_tmp1[4]);
                        // printf("ref_tmp2[0]=%d\n,ref_tmp2[1]=%d\n,ref_tmp2[2]=%d\n,ref_tmp2[3]=%d\n,ref_tmp2[4] =%d\n",ref_tmp2[0],ref_tmp2[1],ref_tmp2[2],ref_tmp2[3],ref_tmp2[4]);
                    // }
                    // 使用栈数组存储mask和ad_mask
                    int mask1[3], mask2[3], ad_mask1[3], ad_mask2[3];
                    int PLRawFillMask = 0;
                    rebuildFunc(dist_tmp1, dist_tmp2, mask1, mask2, ad_mask1, ad_mask2, &PLRawFillMask, TileIdx, col_idx, row_idx);
                    // if(TileIdx == 0 && col_idx == 2 && row_idx == 15){
                    //     // printf("mask1[0] = %d, mask1[1] = %d, mask1[2] = %d\n", mask1[0], mask1[1], mask1[2]);
                    //     // printf("mask2[0] = %d, mask2[1] = %d, mask2[2] = %d\n", mask2[0], mask2[1], mask2[2]);
                    //     // printf("ad_mask1[0] = %d, ad_mask1[1] = %d, ad_mask1[2] = %d\n", ad_mask1[0], ad_mask1[1], ad_mask1[2]);
                    //     // printf("ad_mask2[0] = %d, ad_mask2[1] = %d, ad_mask2[2] = %d\n", ad_mask2[0], ad_mask2[1], ad_mask2[2]);
                    //     // printf("PLRawFillMask = %d\n", PLRawFillMask);
                    // }
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
                        w1[i] = upsample_Param->weight1[i] * mask1[i];
                        w2[i] = upsample_Param->weight2[i] * mask2[i];
                        w1_sum += w1[i];
                        w2_sum += w2[i];
                    }
                    const int total_weight = w1_sum + w2_sum;
                    int dist_ins = 0, ref_ins = 0;
                    if(PLRawFillMask)
                    {
                        dist_ins = dist_raw;
                        ref_ins = ref_raw;
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

                        const int coef = upsample_Param->coef_inv[total_weight - 1];
                        dist_ins = ((term_dist1 + term_dist2) * coef) >> 12;
                        ref_ins = ((term_ref1 + term_ref2) * coef) >> 12;

                        // 特殊边界优化
                        if (w1[1] > 0 && w2[1] > 0) {
                            const int avg = (dist_c1[1] + dist_c2[1]) >> 1;
                            const int diff = ABS(dist_ins - avg);
                            if (diff > 30 || ((dist_ins > dist_c1[1]) == (dist_ins > dist_c2[1]))) {
                                dist_ins = avg;
                            }
                        }

                        // Ref边界优化
                        const int ref_c1_1 = ref_c1[1], ref_c2_1 = ref_c2[1];
                        if (ref_c1_1 > ref_ins && ref_c2_1 > ref_ins) {
                            ref_ins = MIN(ref_c1_1, ref_c2_1);
                        }
                        else if (ref_c1_1 < ref_ins && ref_c2_1 < ref_ins) {
                            ref_ins = MAX(ref_c1_1, ref_c2_1);
                        }

                        if(!((dist_raw > 0) && (dist_ins > 0) && (ABS(dist_raw - dist_ins) < 100))){
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
                            ref_c1[i] = MIN(ref_tmp1[i], ref_tmp1[i + 2]);
                        }

                        if (ad_mask2[i] == 1) {
                            dist_c2[i] = ref_c2[i] = 0;
                        }
                        else if (ad_mask2[i] == 2) {
                            dist_c2[i] = (dist_tmp2[i] + dist_tmp2[i + 2]) >> 1;
                            ref_c2[i] = MIN(ref_tmp2[i], ref_tmp2[i + 2]);
                        }
                    }
                    // 存储结果
                    dist_ins_c[row_idx] = dist_ins;
                    // if(TileIdx == 0 && col_idx == 2)
                    // {
                        // printf("dist_ins_c: %d\n", dist_ins_c[row_idx]);
                    // }
                    ref_ins_c[row_idx] = ref_ins;
                    outputDistUpBufferVMEM[dstDistOffset + col_idx * DistOutUpLinePitch + row_idx] = dist_ins;
                    outputRefUpBufferVMEM[dstRefOffset + col_idx * RefOutUpLinePitch + row_idx] = ref_ins;

                    // 优化缓冲区更新逻辑
                    if (row_idx > 0 && row_idx < TILE_WIDTH - 1) {
                        const int update_row = row_idx - 1;
                        for (int i = 0; i < 3; ++i) {
                            outputDistOriBufferVMEM[dstDistOffset + col_idx * DistOutOriLinePitch + update_row + i] = dist_c1[i];
                            outputDistOriBufferVMEM[dstDistOffset + (col_idx + 1) * DistOutOriLinePitch + update_row + i] = dist_c2[i];
                            outputRefOriBufferVMEM[dstRefOffset + col_idx * RefOutOriLinePitch + update_row + i] = ref_c1[i];
                            outputRefOriBufferVMEM[dstRefOffset + (col_idx + 1) * RefOutOriLinePitch + update_row + i] = ref_c2[i];
                        }
                    }
                    else if (row_idx == 0) {
                        outputDistOriBufferVMEM[dstDistOffset + col_idx * DistOutOriLinePitch] = dist_c1[1];
                        outputDistOriBufferVMEM[dstDistOffset + col_idx * DistOutOriLinePitch + 1] = dist_c1[2];
                        outputDistOriBufferVMEM[dstDistOffset + (col_idx + 1) * DistOutOriLinePitch] = dist_c2[1];
                        outputDistOriBufferVMEM[dstDistOffset + (col_idx + 1) * DistOutOriLinePitch + 1] = dist_c2[2];

                        outputRefOriBufferVMEM[dstRefOffset + col_idx * RefOutOriLinePitch] = ref_c1[1];
                        outputRefOriBufferVMEM[dstRefOffset + col_idx * RefOutOriLinePitch + 1] = ref_c1[2];
                        outputRefOriBufferVMEM[dstRefOffset + (col_idx + 1) * RefOutOriLinePitch] = ref_c2[1];
                        outputRefOriBufferVMEM[dstRefOffset + (col_idx + 1) * RefOutOriLinePitch + 1] = ref_c2[2];
                    }
                    else {
                        const int row_end = TILE_WIDTH - 1;

                        outputDistOriBufferVMEM[dstDistOffset + col_idx * DistOutOriLinePitch + row_end - 1] = dist_c1[0];
                        outputDistOriBufferVMEM[dstDistOffset + col_idx * DistOutOriLinePitch + row_end] = dist_c1[1];
                        outputDistOriBufferVMEM[dstDistOffset + (col_idx + 1) * DistOutOriLinePitch + row_end - 1] = dist_c2[0];
                        outputDistOriBufferVMEM[dstDistOffset + (col_idx + 1) * DistOutOriLinePitch + row_end] = dist_c2[1];

                        outputRefOriBufferVMEM[dstRefOffset + col_idx * RefOutOriLinePitch + row_end - 1] = ref_c1[0];
                        outputRefOriBufferVMEM[dstRefOffset + col_idx * RefOutOriLinePitch + row_end] = ref_c1[1];
                        outputRefOriBufferVMEM[dstRefOffset + (col_idx + 1) * RefOutOriLinePitch + row_end - 1] = ref_c2[0];
                        outputRefOriBufferVMEM[dstRefOffset + (col_idx + 1) * RefOutOriLinePitch + row_end] = ref_c2[1];
                    }
                    // if(TileIdx == 0 && col_idx == 2){
                    //     //TODO
                    // }
                }
            }
        }
#endif
        srcDistOffset = cupvaRasterDataFlowGetOffset(InputDistDataFlowHandler, srcDistOffset);
        srcRefOffset = cupvaRasterDataFlowGetOffset(InputRefDataFlowHandler, srcRefOffset);
        srcDistRawOffset = cupvaRasterDataFlowGetOffset(InputDistRawDataFlowHandler, srcDistRawOffset);
        srcRefRawOffset = cupvaRasterDataFlowGetOffset(InputRefRawDataFlowHandler, srcRefRawOffset);

        dstDistOffset = cupvaRasterDataFlowGetOffset(OutputDistOriDataFlowHandler, dstDistOffset);
        dstRefOffset = cupvaRasterDataFlowGetOffset(OutputRefOriDataFlowHandler, dstRefOffset);

        cupvaRasterDataFlowSync(OutputDistOriDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputDistOriDataFlowHandler);
        cupvaRasterDataFlowSync(OutputDistUpDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputDistUpDataFlowHandler);
        cupvaRasterDataFlowSync(OutputRefOriDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputRefOriDataFlowHandler);
        cupvaRasterDataFlowSync(OutputRefUpDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputRefUpDataFlowHandler);
    }
    cupvaRasterDataFlowSync(OutputDistOriDataFlowHandler);
    cupvaRasterDataFlowSync(OutputDistUpDataFlowHandler);
    cupvaRasterDataFlowSync(OutputRefOriDataFlowHandler);
    cupvaRasterDataFlowSync(OutputRefUpDataFlowHandler);
    return 0;
}
