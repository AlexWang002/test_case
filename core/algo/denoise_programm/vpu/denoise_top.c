/*******************************************************************************
 * \addtogroup denoise_programm
 * \{
 * \file denoise_top.c
 * \brief
 * \version 0.1
 * \date 2025-09-11
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-11 | Init version |
 *
 ******************************************************************************/
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_device.h> /* Main device-side header file */

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../denoise_common_param.h"

/** [declare_algorithm_params] */
VMEM(C, uint16_t, input_dist_vmem,
        RDF_CIRCULAR(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

VMEM(A, int, col_idx);

VMEM(B, int, output_mask_vmem,
        RDF_SINGLE(int, TILE_WIDTH, TILE_HEIGHT + 4));

VMEM(B, int, output_last_mask_vmem,
        RDF_SINGLE(int, TILE_WIDTH, 4));

VMEM(C, int, algorithmParams, sizeof(NoiseParam_t));

VMEM_RDF_UNIFIED(A, src_dist_dataflow_handler);
VMEM_RDF_UNIFIED(B, dst_mask_dataflow_handler);
VMEM_RDF_UNIFIED(B, dst_last_mask_dataflow_handler);

void memcpyPVA(uint8_t *dst, uint8_t *src, uint32_t size)
{
    for (int i = 0; i < size; i ++) {
        dst[i] = src[i];
    }
}

void memsetPVA(uint8_t *buf, uint8_t val, uint32_t size)
{
    for (int i = 0; i < size; i ++) {
        buf[i] = val;
    }
}

uint16_t absPVA(uint16_t v1, uint16_t v2)
{
    if (v1 >= v2) {
        return (v1 - v2);
    }

    return (v2 - v1);
}

CUPVA_VPU_MAIN()
{
    NoiseParam_t *params = (NoiseParam_t *)algorithmParams;

    int32_t src_line_pitch       = cupvaRasterDataFlowGetLinePitch(src_dist_dataflow_handler); // src_line_pitch的值为192：每一列（192）上下各补0个点
    int32_t dst_line_pitch       = cupvaRasterDataFlowGetLinePitch(dst_mask_dataflow_handler);
    int32_t src_circular_buf_len = cupvaRasterDataFlowGetCbLen(src_dist_dataflow_handler);

    cupvaRasterDataFlowOpen(src_dist_dataflow_handler, &input_dist_vmem[0]);
    cupvaRasterDataFlowOpen(dst_mask_dataflow_handler, &output_mask_vmem[0]);
    cupvaRasterDataFlowOpen(dst_last_mask_dataflow_handler, &output_last_mask_vmem[0]);

    int *output_last_mask  = (int *)cupvaRasterDataFlowAcquire(dst_last_mask_dataflow_handler);

    for (int tile_idx = 0; tile_idx < TILE_CNT; tile_idx ++) {
        uint16_t *input_dist  = (uint16_t *)cupvaRasterDataFlowAcquire(src_dist_dataflow_handler);
        int *output_mask  = (int *)cupvaRasterDataFlowAcquire(dst_mask_dataflow_handler);

        int32_t src_offset = input_dist - &input_dist_vmem[0];

        /*将上一个tile的最后4列mask值拷贝到最前面，作为当前tile的首4列*/
        memcpyPVA((uint8_t *)output_mask, (uint8_t *)&output_mask[TILE_WIDTH * TILE_HEIGHT], TILE_WIDTH * 4 * sizeof(int));
        /*mask标志位清0*/
        memsetPVA((uint8_t *)&output_mask[TILE_WIDTH * 4], 0, TILE_WIDTH * TILE_HEIGHT * sizeof(int));

#if ALGO_ENABLE
        int col_cnt = (tile_idx == TILE_CNT - 1) ? (TILE_HEIGHT + 2) : TILE_HEIGHT;
        for (int col = 0; col < col_cnt; col ++) {
            const int win_len_h = params->win_len_h;
            const int win_len_v = params->win_len_v;
            const int HL = 2* params->win_len_h + 1;
            const int HV = 2* params->win_len_v + 1;

            /*预加载常用参数*/
            const int dist_seg_div = 12000;

            /*主处理循环*/
            for (int row = 0; row < TILE_WIDTH; row ++) {
                /*计算当前点在halo buffer中的索引*/
                const int cur_idx = (src_offset + (col + KERNEL_RADIUS_HEIGHT/2) * src_line_pitch + (row + KERNEL_RADIUS_WIDTH)) % src_circular_buf_len;
                const int dist_cur = input_dist_vmem[cur_idx];
                const int valid_cur = output_mask[(col + 2) * TILE_WIDTH + row];

                const int block_id = (row + BlockSize) / BlockSize - 1;
                const int dist_seg = (dist_cur / dist_seg_div);
                const int region = params->region_list[block_id] - 1;
                const int threshold = params->denoise_point[dist_seg][region][0];
                const int threshold1 = params->denoise_point[dist_seg][region][1];
                const int threshold2 = params->denoise_point[dist_seg][region][2];

                if (dist_cur > 8000 || (valid_cur != 1 && dist_cur > 0)) {
                    int denoise_refer_mask = 0;
                    const int diff_th = params->denoise_offset[dist_seg]; // 数组访问优化

                    int valid_mask[5][3] = {0};
                    /*邻域有效性统计优化*/
                    int valid_cnt = 0;
                    for (int c = -win_len_h; c <= win_len_h; c++) {
                        int neib_col = col + c;
                        int neib_idx = (src_offset + (neib_col + KERNEL_RADIUS_HEIGHT/2) * src_line_pitch + (row + KERNEL_RADIUS_WIDTH)) % src_circular_buf_len;

                        bool is_valid = absPVA(input_dist_vmem[neib_idx], dist_cur) <= diff_th;
                        valid_mask[c + win_len_h][win_len_v] = is_valid;
                        valid_cnt += is_valid;
                    }

                    /*两侧有效性统计*/
                    int valid_pre_2 = valid_mask[win_len_h - 1][win_len_v] + valid_mask[win_len_h + 1][win_len_v];

                    denoise_refer_mask = (valid_pre_2 >= threshold1) ? ((valid_cnt > 2) ? 2 : 1) : 3;

                    if (denoise_refer_mask == 1) {
                        /*领域窗口构建优化*/
                        int valid_pre_3 = valid_mask[0][win_len_v] + valid_mask[HL - 1][win_len_v];
                        int neib_valid_num[3] = {0};

                        for (int r = -win_len_v; r <= win_len_v; r ++) {
                            if (r == 0 && row != (TILE_WIDTH - 1)) continue;
                            const int neib_row = row + r;
                            if (neib_row < 0 || neib_row >= TILE_WIDTH || r == row) continue;

                            int r_loc = r + win_len_v;
                            for (int c = -win_len_h; c <= win_len_h; c ++) {
                                int c_loc = c + win_len_h;
                                
                                const int neib_col = col + c;
                                int neib_idx = (src_offset + (neib_col + KERNEL_RADIUS_HEIGHT/2) * src_line_pitch + (neib_row + KERNEL_RADIUS_WIDTH)) % src_circular_buf_len;

                                int dist_neib = input_dist_vmem[neib_idx];
                                int valid_neib = output_mask[(neib_col + 2) * TILE_WIDTH + neib_row];

                                /*有效性判断*/
                                if (absPVA(dist_neib, dist_cur) <= diff_th && (valid_neib != 6)) {
                                    const int zone = params->zone_matrix[r_loc][c_loc];
                                    if (zone > 0) {
                                        neib_valid_num[zone - 1] ++;
                                        valid_mask[c_loc][r_loc] = 1;
                                    }
                                }
                            }
                        }

                        /*去噪条件判断优化*/
                        bool cond = ((neib_valid_num[0] + valid_pre_2) >= threshold
                                    || (row == (TILE_WIDTH - 1) && valid_pre_2 >= threshold1)
                                    || (neib_valid_num[0] + valid_pre_2 >= threshold)
                                    || (valid_pre_3 + valid_pre_2 >= threshold2));

                        if (cond) {
                            output_mask[(col + 2) * TILE_WIDTH + row] = 1;

                            /*传播有效标记优化*/
                            for (int r = -win_len_v; r <= win_len_v; r ++) {
                                if (r == 0 && row != (TILE_WIDTH - 1)) continue;
                                const int r_loc = r + win_len_v;
                                const int r_idx = row + r;
                                if (r_idx < 0 || r_idx >= TILE_WIDTH) continue;

                                for (int c = -win_len_h; c <= win_len_h; c ++) {
                                    const int c_loc = c + win_len_h;
                                    const int c_idx = col + c;

                                    if (1 == valid_mask[c_loc][r_loc]) {
                                        output_mask[(c_idx + 2) * TILE_WIDTH + r_idx] = 1;
                                    }
                                }
                            }
                        }
                    }
                    else if (denoise_refer_mask == 2) {
                        /*直接对左右补点优化*/
                        output_mask[(col + 2) * TILE_WIDTH + row] = 1;

                        for (int c = -win_len_h; c <= win_len_h; c ++) {
                            const int c_loc = c + win_len_h;
                            const int c_idx = col + c;

                            if (valid_mask[c_loc][win_len_v] == 1) {
                                output_mask[(c_idx + 2) * TILE_WIDTH + row] = 1;
                            }
                        }
                    }
                }
            }
        }

        if (tile_idx == (TILE_CNT - 1)) {
            memcpyPVA((uint8_t *)output_last_mask, (uint8_t *)&output_mask[TILE_WIDTH * TILE_HEIGHT], 4 * TILE_WIDTH * sizeof(int));
        }

#endif
        cupvaRasterDataFlowRelease(src_dist_dataflow_handler);
        cupvaRasterDataFlowRelease(dst_mask_dataflow_handler);
    }

    cupvaRasterDataFlowRelease(dst_last_mask_dataflow_handler);

    /*Close*/
    cupvaRasterDataFlowClose(src_dist_dataflow_handler);
    cupvaRasterDataFlowClose(dst_mask_dataflow_handler);
    cupvaRasterDataFlowClose(dst_last_mask_dataflow_handler);

    return 0;
}
/** [release_and_close] */

