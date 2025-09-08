#include "../trail_common_param.h"
#include <cupva_device.h> /* Main device-side header file */

/** Use double buffer, the vertical halo is 2 */
VMEM(A, uint16_t, inputDistBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(B, uint16_t, inputRefBufferVMEM,
    RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

/** Output do not use halo */
VMEM(C, uint16_t, outputValidBufferVMEM,
    RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));

/** declare_algorithm_params */
VMEM(C, TrailParam_t, trail_Param);

/** declare_df_handles */
VMEM(C, RasterDataFlowHandler, sourceDistDataFlowHandler);
VMEM(C, RasterDataFlowHandler, sourceRefDataFlowHandler);
VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler);

int clamp(int val, int min_val, int max_val) {
    return MAX(min_val, MIN(val, max_val));
}

void HorTrailRemove(int* dist_tmp, int dist_trail, int* Hortrail_judge, int* wall_judge, const TrailParam_t *trail_Param)
{
    int weight[5] = {1,2,0,2,1};
    *Hortrail_judge = 0;
    *wall_judge = 0;

    const int HL = 5;
    const int half_HL = 2;

    // 差异数组初始化
    int dif_distC_abs[5] = {0};
    int dif_dist[4] = {0};
    int dif_dist_abs[4] = {0};
    int dif2_dist_abs[3] = {0};
    // 计算自适应阈值
    const int AdjDisThreD = max(1, (dist_trail * trail_Param->DisThreRatio) >> 12);
    int zero_cnt = 0;
    int dif_dist_abs_cnt = 0;

    // 参数提取
    const int& D_H = trail_Param->D_H;

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
        dif_distC_abs[i] = ABS(dif_distC_tmp);

        // 相邻点差异计算
        if (i < HL - 1) {
            dif_dist_tmp = dist_tmp[i + 1] - dist_tmp[i];
            dif_dist[i] = dif_dist_tmp;
            dif_dist_abs[i] = ABS(dif_dist_tmp);
            // 大差异计数
            if (dif_dist_abs[i] > AdjDisThreD) {
                dif_dist_abs_cnt++;
            }

            // 二阶差分计算
            if (i > 0) {
                dif2_dist_abs[i - 1] = ABS(dif_dist[i] - dif_dist[i - 1]);
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
    int distdiff_mean = round(weighted_sum / 4.0);
    bool con2 = (distdiff_mean > AdjDisThreD) && (distdiff_mean < D_H);

    if (con0 && (con1 || con2)) {
        *Hortrail_judge = 1;
    }
    int sum_dif2_dist_abs = 0;
    for (int i = 0; i < HL; i++)
    {
        sum_dif2_dist_abs += dif2_dist_abs[i];
    }
    int wall_mask1 = sum_dif2_dist_abs > trail_Param->SlopDifThre;
    int wall_mask2 = dif2_dist_abs[half_HL - 2] > trail_Param->SlopDifThre;
    if(wall_mask1 && wall_mask2)
    {
      *wall_judge = 1;
    }
}

int VerTrailRemove(int dist_trail, int* dist_longit, int wall_judge, int near_cnt_h, int near_dist_th, const TrailParam_t *trail_Param)
{
    int Vertrail_judge = 0;
    const int HL = 5;
    const int half_HL = 2;

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
            dif_distC_ver_abs[i] = ABS(dist_longit_tmp - dist_trail);
        }
        if(dif_distC_ver_abs[i] < near_dist_th)
        {
            near_cnt_v++;
        }
        if(dif_distC_ver_abs[i] < trail_Param->SlopDifThre * 2)
        {
          ver_cnt++;
        }

    }
    //距离判断条件3 保护细杆
    bool con3_1 = near_cnt_h < trail_Param->near_cnt_th_h;
    bool con3_2 = near_cnt_v < trail_Param->near_cnt_th_v;
    bool con3 = con3_1 && con3_2;
    // 距离判断条件4 保护斜墙
    bool con4_3 = ver_cnt < 3;
    bool con4 = wall_judge || con4_3;
    if(con3 && con4)
    {
        Vertrail_judge = 1;
    }
    return Vertrail_judge;
}

CUPVA_VPU_MAIN()
{
    /** Calculate line pitch */
    uint16_t srcDistLinePitch = cupvaRasterDataFlowGetLinePitch(sourceDistDataFlowHandler);
    uint16_t srcRefLinePitch = cupvaRasterDataFlowGetLinePitch(sourceRefDataFlowHandler);
    uint16_t dstLinePitch = cupvaRasterDataFlowGetLinePitch(destinationDataFlowHandler);

    /** Every tile offset */
    int32_t srcDistOffset = 0;
    int32_t srcRefOffset = 0;
    int32_t dstOffset = 0;

    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);
    cupvaRasterDataFlowTrig(sourceRefDataFlowHandler);

    /** Loop over tiles, TILE_COUNT = 8*/
    for(int TileIdx = 0; TileIdx < TILE_COUNT; TileIdx++)
    {
        /** Trigger the data flow and switch to the next tile */
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler);
        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);
        cupvaRasterDataFlowSync(sourceRefDataFlowHandler);
        cupvaRasterDataFlowTrig(sourceRefDataFlowHandler);

        for(int i = 0; i < TILE_WIDTH * TILE_HEIGHT; i++){
            outputValidBufferVMEM[i] = 0;
        }
        // memset(&outputValidBufferVMEM[dstOffset], 0, sizeof(uint16_t) * TILE_WIDTH * TILE_HEIGHT);
#ifdef ALGO_ON
        const int HL = 5;
        const int VerticalRange = 4;
        /** Process the columns except for "halo" */
        for (int col_idx = 2; col_idx < TILE_HEIGHT + 2; ++col_idx) //95
        {
            const int near_threshold = trail_Param.near_cnt_th_h; //3
            const int bypass_distance = trail_Param.BypassDis;

            /** Trail process, judge current point based on 5*9 neighbor */
            for (int row_idx = 0; row_idx < TILE_WIDTH; ++row_idx)
            {
                int near_dist_th = 0;
                int near_cnt_h = 0;

                const int dist_trail = inputDistBufferVMEM[col_idx * srcDistLinePitch + row_idx + srcDistOffset];
                if (dist_trail <= 0 || dist_trail >= bypass_distance) continue;

                int trail_refer_mask = 0;
                near_dist_th = clamp(((dist_trail *  trail_Param.dist_th_ratio) >> 3), 2, 20);
                int dist_tmp[HL] = {0};

                for (int j = -HL/2; j <= HL/2; ++j) {
                    dist_tmp[j + HL/2] = inputDistBufferVMEM[(col_idx + j) * srcDistLinePitch + row_idx + srcDistOffset];
                }

                for (int i = 0; i < HL - 1; ++i) {
                    near_cnt_h += (ABS(dist_tmp[i + 1] - dist_tmp[i]) < near_dist_th);
                }

                trail_refer_mask = (near_cnt_h >= near_threshold);
                if (trail_refer_mask != 0) continue;
                int Hortrail_judge = 0;
                int wall_judge = 0;
                HorTrailRemove(dist_tmp, dist_trail, &Hortrail_judge, &wall_judge, &trail_Param);

                if(Hortrail_judge == 1)
                {
                    int dist_longit[9] = {0};
                    const int row_range_up = MIN(row_idx, VerticalRange);
                    const int row_range_down = MIN(TILE_WIDTH - 1 - row_idx, VerticalRange);
                    const int start_idx = 4 - row_range_up;
                    for (int k = 0; k <= row_range_up + row_range_down; ++k)
                    {
                        const int buf_idx = start_idx + k;
                        dist_longit[buf_idx] = inputDistBufferVMEM[col_idx * srcDistLinePitch + srcDistOffset + row_idx - row_range_up + k];
                    }
                    int Vertrail_judge = VerTrailRemove(dist_trail, dist_longit, wall_judge, near_cnt_h, near_dist_th, &trail_Param);
                    if(Vertrail_judge == 1)
                    {
                        outputValidBufferVMEM[(col_idx - 2) * dstLinePitch + row_idx + dstOffset] = 1;
                    }
                }
            }
        }
#endif
        srcDistOffset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler, srcDistOffset);
        srcRefOffset = cupvaRasterDataFlowGetOffset(sourceRefDataFlowHandler, srcRefOffset);
        dstOffset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler, dstOffset);

        cupvaRasterDataFlowSync(destinationDataFlowHandler);
        cupvaRasterDataFlowTrig(destinationDataFlowHandler);
    }
    cupvaRasterDataFlowSync(destinationDataFlowHandler);
    return 0;
}
