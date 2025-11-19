/*******************************************************************************
 * \addtogroup spray_programm
 * \{
 * \file spray_top.c
 * \brief
 * \version 0.2
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
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-09-29 | Spray vector version |

 ******************************************************************************/
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_device.h>
#include <cupva_device_debug.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../spray_common_param.h"

/** Use double buffer, the vertical halo is 1, the horizontal halo is 2*/
VMEM(A, uint16_t, inputDistBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint16_t, inputDistBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint16_t, inputRefBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint16_t, inputRefBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(C, uint16_t, inputAttBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(C, uint16_t, inputAttBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(B, uint16_t, inputGlkBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT * 4));

/** Output do not use halo */
VMEM(C, uint16_t, outputRainBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, outputRainBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));

/** declare algorithm params */
VMEM(C, int, algorithmParams, sizeof(SprayParam_t));
/** declare dataflow handles */
VMEM(A, RasterDataFlowHandler, sourceDistDataFlowHandler0);
VMEM(A, RasterDataFlowHandler, sourceDistDataFlowHandler1);
VMEM(B, RasterDataFlowHandler, sourceRefDataFlowHandler0);
VMEM(B, RasterDataFlowHandler, sourceRefDataFlowHandler1);
VMEM(C, RasterDataFlowHandler, sourceAttDataFlowHandler0);
VMEM(C, RasterDataFlowHandler, sourceAttDataFlowHandler1);
VMEM(B, RasterDataFlowHandler, sourceGlkDataFlowHandler);

VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler0);
VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler1);

int32_t dist0_offset = 0, dist1_offset = 0;
int32_t ref0_offset = 0, ref1_offset = 0;
int32_t att0_offset = 0, att1_offset = 0;
int32_t glk_offset = 0, mask0_offset = 0, mask1_offset = 0;
SprayParam_t *spray_Param;
uint16_t zone_ids[192];

dvshortx vec0;        // 0向量
dvshortx zone_idx[6];

dvshortx gnd0;
dvshortx gnd1;
dvshortx glk0;
dvshortx glk1;
dvshortx cur_dist_1;
dvshortx cur_ref_1;
dvshortx cur_dist_2;
dvshortx cur_ref_2;

/*二维去噪算法AGEN结构体*/
typedef struct {
    AgenCFG input_dist0;     // 输入dist0 agen
    AgenCFG input_dist1;     // 输入dist1 agen
    AgenCFG input_ref0;      // 输入ref0 agen
    AgenCFG input_ref1;      // 输入ref1 agen
    AgenCFG input_gnd0;      // 输入gnd0 agen
    AgenCFG input_gnd1;      // 输入gnd1 agen
    AgenCFG input_att0;      // 输入att0 agen
    AgenCFG input_att1;      // 输入att1 agen
    AgenCFG input_glk0;      // 输入glk0 agen
    AgenCFG input_glk1;      // 输入glk1 agen
    AgenCFG output_mask0;    // 输出mask0 agen
    AgenCFG output_mask1;    // 输出mask1 agen

    AgenCFG zone_idx;
    AgenCFG thr_seq0;
    AgenCFG thr_seq1;

    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} SprayConfig_t;

void agenConfigInit(SprayConfig_t *config)
{
    /*获取向量宽度（每个dvshortx包含的元素数）*/
    config->vecw = pva_elementsof(dvshortx); // 32
    int32_t vecw = config->vecw;

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper0;
    input_wrapper0.size = sizeof(uint16_t);
    input_wrapper0.n1   = 5;
    input_wrapper0.s1   = TILE_WIDTH + 2;
    input_wrapper0.n2   = 3;
    input_wrapper0.s2   = 1;
    input_wrapper0.n3   = TILE_WIDTH / vecw;
    input_wrapper0.s3   = vecw;
    input_wrapper0.n4   = TILE_HEIGHT;
    input_wrapper0.s4   = TILE_WIDTH + 2;
    agen input_agen0 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen0, input_wrapper0);
    config->input_dist0 = extract_agen_cfg(input_agen0);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper1;
    input_wrapper1.size = sizeof(uint16_t);
    input_wrapper1.n1   = 5;
    input_wrapper1.s1   = TILE_WIDTH + 2;
    input_wrapper1.n2   = 3;
    input_wrapper1.s2   = 1;
    input_wrapper1.n3   = TILE_WIDTH / vecw;
    input_wrapper1.s3   = vecw;
    input_wrapper1.n4   = TILE_HEIGHT;
    input_wrapper1.s4   = TILE_WIDTH + 2;
    agen input_agen1 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen1, input_wrapper1);
    config->input_dist1 = extract_agen_cfg(input_agen1);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper2;
    input_wrapper2.size = sizeof(uint16_t);
    input_wrapper2.n1   = 5;
    input_wrapper2.s1   = TILE_WIDTH + 2;
    input_wrapper2.n2   = 3;
    input_wrapper2.s2   = 1;
    input_wrapper2.n3   = TILE_WIDTH / vecw;
    input_wrapper2.s3   = vecw;
    input_wrapper2.n4   = TILE_HEIGHT;
    input_wrapper2.s4   = TILE_WIDTH + 2;
    agen input_agen2 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen2, input_wrapper2);
    config->input_ref0 = extract_agen_cfg(input_agen2);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper3;
    input_wrapper3.size = sizeof(uint16_t);
    input_wrapper3.n1   = 5;
    input_wrapper3.s1   = TILE_WIDTH + 2;
    input_wrapper3.n2   = 3;
    input_wrapper3.s2   = 1;
    input_wrapper3.n3   = TILE_WIDTH / vecw;
    input_wrapper3.s3   = vecw;
    input_wrapper3.n4   = TILE_HEIGHT;
    input_wrapper3.s4   = TILE_WIDTH + 2;
    agen input_agen3 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen3, input_wrapper3);
    config->input_ref1 = extract_agen_cfg(input_agen3);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper4;
    input_wrapper4.size = sizeof(uint16_t);
    input_wrapper4.n1   = 5;
    input_wrapper4.s1   = TILE_WIDTH + 2;
    input_wrapper4.n2   = 3;
    input_wrapper4.s2   = 1;
    input_wrapper4.n3   = TILE_WIDTH / vecw;
    input_wrapper4.s3   = vecw;
    input_wrapper4.n4   = TILE_HEIGHT;
    input_wrapper4.s4   = TILE_WIDTH + 2;
    agen input_agen4 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen4, input_wrapper4);
    config->input_att0 = extract_agen_cfg(input_agen4);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper5;
    input_wrapper5.size = sizeof(uint16_t);
    input_wrapper5.n1   = 5;
    input_wrapper5.s1   = TILE_WIDTH + 2;
    input_wrapper5.n2   = 3;
    input_wrapper5.s2   = 1;
    input_wrapper5.n3   = TILE_WIDTH / vecw;
    input_wrapper5.s3   = vecw;
    input_wrapper5.n4   = TILE_HEIGHT;
    input_wrapper5.s4   = TILE_WIDTH + 2;
    agen input_agen5 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen5, input_wrapper5);
    config->input_att1 = extract_agen_cfg(input_agen5);

    /*循环次数：6 * 20*/
    AgenWrapper input_wrapper6;
    input_wrapper6.size = sizeof(uint16_t);
    input_wrapper6.n1   = TILE_WIDTH / vecw;
    input_wrapper6.s1   = vecw;
    input_wrapper6.n2   = TILE_HEIGHT;
    input_wrapper6.s2   = TILE_WIDTH;
    agen input_agen6 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen6, input_wrapper6);
    config->input_glk0 = extract_agen_cfg(input_agen6);

    /*循环次数：6 * 20*/
    AgenWrapper input_wrapper7;
    input_wrapper7.size = sizeof(uint16_t);
    input_wrapper7.n1   = TILE_WIDTH / vecw;
    input_wrapper7.s1   = vecw;
    input_wrapper7.n2   = TILE_HEIGHT;
    input_wrapper7.s2   = TILE_WIDTH;
    agen input_agen7 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen7, input_wrapper7);
    config->input_glk1 = extract_agen_cfg(input_agen7);

    /*循环次数：6 * 20*/
    AgenWrapper input_wrapper8;
    input_wrapper8.size = sizeof(uint16_t);
    input_wrapper8.n1   = TILE_WIDTH / vecw;
    input_wrapper8.s1   = vecw;
    input_wrapper8.n2   = TILE_HEIGHT;
    input_wrapper8.s2   = TILE_WIDTH;
    agen input_agen8 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen8, input_wrapper8);
    config->input_gnd0 = extract_agen_cfg(input_agen8);

    AgenWrapper input_wrapper9;
    input_wrapper9.size = sizeof(uint16_t);
    input_wrapper9.n1   = TILE_WIDTH / vecw;
    input_wrapper9.s1   = vecw;
    input_wrapper9.n2   = TILE_HEIGHT;
    input_wrapper9.s2   = TILE_WIDTH;
    agen input_agen9 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen9, input_wrapper9);
    config->input_gnd1 = extract_agen_cfg(input_agen9);

    AgenWrapper input_wrapper10;
    input_wrapper10.size = sizeof(uint16_t);
    input_wrapper10.n1 = 1;
    input_wrapper10.s1 = 0;
    agen thr_seq0_agen = init((dvushort *)spray_Param->dist_diff_thr_seq);
    INIT_AGEN1(thr_seq0_agen, input_wrapper10);
    config->thr_seq0 = extract_agen_cfg(thr_seq0_agen);

    AgenWrapper input_wrapper11;
    input_wrapper11.size = sizeof(uint16_t);
    input_wrapper11.n1 = 1;
    input_wrapper11.s1 = 0;
    agen thr_seq1_agen = init((dvushort *)spray_Param->dist_diff_diff_thr_seq);
    INIT_AGEN1(thr_seq1_agen, input_wrapper11);
    config->thr_seq1 = extract_agen_cfg(thr_seq1_agen);

    AgenWrapper input_wrapper12;
    input_wrapper12.size = sizeof(uint16_t);
    input_wrapper12.n1 = TILE_WIDTH / vecw;
    input_wrapper12.s1 = vecw;
    agen zone_idx_agen = init((dvushort *)zone_ids);
    INIT_AGEN1(zone_idx_agen, input_wrapper12);
    config->zone_idx = extract_agen_cfg(zone_idx_agen);

    /*循环次数：6 * 95*/
    AgenWrapper output_wrapper0;
    output_wrapper0.size = sizeof(uint16_t);
    output_wrapper0.n1   = TILE_WIDTH / vecw;
    output_wrapper0.s1   = vecw;
    output_wrapper0.n2   = TILE_HEIGHT;
    output_wrapper0.s2   = TILE_WIDTH;
    agen output_agen0 = init((dvushort *)NULL);
    INIT_AGEN2(output_agen0, output_wrapper0);
    config->output_mask0 = extract_agen_cfg(output_agen0);

    /*循环次数：6 * 95*/
    AgenWrapper output_wrapper1;
    output_wrapper1.size = sizeof(uint16_t);
    output_wrapper1.n1   = TILE_WIDTH / vecw;
    output_wrapper1.s1   = vecw;
    output_wrapper1.n2   = TILE_HEIGHT;
    output_wrapper1.s2   = TILE_WIDTH;
    agen output_agen1= init((dvushort *)NULL);
    INIT_AGEN2(output_agen1, output_wrapper1);
    config->output_mask1 = extract_agen_cfg(output_agen1);

    /*计算总迭代次数（横向向量数 × 纵向行数）*/
    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}

void vectorized_column_stats(const dvshortx* local_dist,
    const dvshortx* local_ref_mask,
    const dvshortx* local_mark,
    const dvshortx* local_head,
    const dvshortx* local_tail,
    const dvshortx cur_dist,
    const int dist_thr,
    const int sel_idx,
    dvshortx& mark_count,
    dvshortx& head_count,
    dvshortx& tail_count)
{
    for (int i = 0; i < 6; i++) {
        if ((2 == sel_idx) && (3 == i)) continue;
        dvshortx valid = (local_dist[i] != 0) &
                        (cur_dist != 0) &
                        local_ref_mask[i] &
                        (dvabsdif(local_dist[i], cur_dist) <= dist_thr);
        // 谓词控制的计数累加
        mark_count += ((local_mark[i] == 1) & valid);
        head_count += (local_head[i] & valid);
        tail_count += (local_tail[i] & valid);
    }
}

dvshortx calc_sign(dvshortx value)
{
    dvshortx mask_positive = value > vec0;
    dvshortx mask_negative = value < vec0;
    return (mask_positive - mask_negative);
}

void sprayRemoveWave0(agen& output_mask_agen, agen& thr_seq0_agen, agen& thr_seq1_agen,
    dvshortx (*dist_line)[6],
    dvshortx (*ref_line)[6],
    dvshortx (*ref_mask_line)[6],
    dvshortx (*att_line)[6],
    dvshortx (*mark_line)[6],
    dvshortx (*head_line)[6],
    dvshortx (*tail_line)[6],
    int seg_id)
{
    /*第一回波*/
    dvshortx cond_self_1 = (cur_dist_1 > 0) & (cur_dist_1 > spray_Param->Dist_threshold_min) & (cur_dist_1 <= spray_Param->Dist_threshold2)
                            & (cur_ref_1 <= spray_Param->Ref_threshold) & (~mark_line[2][1]);
    vcharx cond_cnt = vdemote(cond_self_1);
    int active_count1 = vsumr_s(cond_cnt);

    dvshortx rain_final = vec0;
    if (active_count1 > 0) {
        dvshortx head_tail_cur_1 = (cur_dist_1 > spray_Param->Dist_threshold_min) & (cur_dist_1 <= spray_Param->Dist_threshold2)
                            & (cur_ref_1 <= spray_Param->Ref_threshold) & head_line[2][1] & tail_line[2][1];

        /*邻域统计*/
        dvshortx sum_mark_num_1 = vec0, sum_head_num_1 = vec0, sum_tail_num_1 = vec0;
        for(int i = 0; i < 5; i++){
            dvshortx mark_A = vec0, head_A = vec0, tail_A = vec0;

            vectorized_column_stats(dist_line[i], ref_mask_line[i], mark_line[i], head_line[i],
                                    tail_line[i], cur_dist_1, spray_Param->dist_diff_thr_2, i,
                                    mark_A, head_A, tail_A);
            sum_mark_num_1 += mark_A;
            sum_head_num_1 += head_A;
            sum_tail_num_1 += tail_A;
        }

        /*梯度计算*/
        dvshortx condition_mask = cur_dist_1 < spray_Param->Dist_threshold;
        dvshortx local_dist_a[6], local_dist_b[6], local_dist_c[6];
        dvshortx local_ref_mask_a[6], local_ref_mask_b[6], local_ref_mask_c[6];

        /*次发段，分辨率放一半，8m*/
        for(int i = 0; i < 6; i++){
            local_dist_a[i] = dvmux(condition_mask, dist_line[0][i], dist_line[1][i]);
            local_dist_b[i] = dist_line[2][i];
            local_dist_c[i] = dvmux(condition_mask, dist_line[4][i], dist_line[3][i]);
            local_ref_mask_a[i] = dvmux(condition_mask, ref_mask_line[0][i], ref_mask_line[1][i]);
            local_ref_mask_b[i] = ref_mask_line[2][i];
            local_ref_mask_c[i] = dvmux(condition_mask, ref_mask_line[4][i], ref_mask_line[3][i]);
        }

        dvshortx dist_diff_col[3], dist_diff_row[3];
        dist_diff_col[0] = local_dist_b[1] - local_dist_a[1];
        dist_diff_col[1] = local_dist_c[1] - local_dist_b[1];
        dist_diff_col[2] = local_dist_c[1] - local_dist_a[1];

        dist_diff_row[0] = local_dist_b[1] - local_dist_b[0];
        dist_diff_row[1] = local_dist_b[2] - local_dist_b[1];
        dist_diff_row[2] = local_dist_b[2] - local_dist_b[0];

        dvshortx dist_diff_thr, dist_diff_diff_thr;
        dvshortx dist_idx_tmp = cur_dist_1 >> 9;
        vcharx dist_idx = vdemote(dvmux(cond_self_1, dist_idx_tmp, vec0)); //获取下标
        dist_diff_thr = dvushort_load_perm(thr_seq0_agen, dist_idx);
        dist_diff_diff_thr = dvushort_load_perm(thr_seq1_agen, dist_idx);

        //梯度一致性判断
        dvshortx consistency_col_1, consistency_row_1;
        consistency_col_1 = (calc_sign(dist_diff_col[0]) == calc_sign(dist_diff_col[1]))
                && dvabsdif(dist_diff_col[0], vec0) <= dist_diff_thr && dvabsdif(dist_diff_col[1], vec0) <= dist_diff_thr
                && dvabsdif(dvabsdif(dist_diff_col[0], vec0), dvabsdif(dist_diff_col[1], vec0)) <= dist_diff_diff_thr
                || (dvabsdif(dist_diff_col[0], vec0) <= 30 && dvabsdif(dist_diff_col[1], vec0) <= 30);

        consistency_row_1 = calc_sign(dist_diff_row[0]) == calc_sign(dist_diff_row[1])
                && dvabsdif(dist_diff_row[0], vec0) <= dist_diff_thr && dvabsdif(dist_diff_row[1], vec0) <= dist_diff_thr
                && dvabsdif(dvabsdif(dist_diff_row[0], vec0), dvabsdif(dist_diff_row[1], vec0)) <= dist_diff_diff_thr
                || (dvabsdif(dist_diff_row[0], vec0) <= 30 && dvabsdif(dist_diff_row[1], vec0) <= 30);

        /*悬空计算*/
        dvshortx ground_link_valid_1 = vec0;
        dvshortx cond_glink_0 = (gnd0 == 0) && (ref_line[2][1] <= spray_Param->Ref_threshold2);
        dvshortx cond_glink_1 = (cur_dist_1 <= spray_Param->Dist_threshold && zone_idx[seg_id] > spray_Param->zone_idx_min_a && zone_idx[seg_id] < spray_Param->zone_idx_max_a)
                        || (cur_dist_1 > spray_Param->Dist_threshold && cur_dist_1 < spray_Param->Dist_threshold2 && zone_idx[seg_id] > spray_Param->zone_idx_min_b && zone_idx[seg_id] < spray_Param->zone_idx_max_b);
        ground_link_valid_1 = cond_glink_0 && cond_glink_1 && (glk0 == 1);

        /*组合判断*/
        //折波标记
        dvshortx cond0_0 = head_tail_cur_1;
        dvshortx cond0_1 =  ((sum_tail_num_1 >= spray_Param->min_tail_threshold) ||
                    (sum_head_num_1 + sum_tail_num_1 >= spray_Param->min_split_threshold));
        //雨雾标记
        dvshortx cond1 = sum_mark_num_1 >= spray_Param->min_mark_threshold;
        //梯度计算
        dvshortx cond3 = (consistency_col_1 || consistency_row_1);
        //地面标记
        dvshortx cond4 = gnd0;
        //接地判别
        dvshortx cond6 = ground_link_valid_1;

        dvshortx cond0_2 = (!head_line[2][1]) && tail_line[2][1] && (cur_dist_1 > 1000) && (cond0_1 || cond1);
        dvshortx cond0_3 = head_line[2][1] && (!tail_line[2][1]) && (cur_dist_1 > 1000) && (cond0_1 || cond1);

        rain_final = ((((cond0_1 || cond1 || cond0_2) && !cond3) || cond0_3 || cond0_0)
                                && !cond4) || (cond6 && !cond3) || ((!cond3 ) && !cond4);
        rain_final &= cond_self_1;
    }

    rain_final |= mark_line[2][1];
    vstore(rain_final, output_mask_agen);
}

void sprayRemoveWave1(agen& output_mask_agen, agen& thr_seq0_agen, agen& thr_seq1_agen,
    dvshortx (*dist_line)[6],
    dvshortx (*ref_line)[6],
    dvshortx (*ref_mask_line)[6],
    dvshortx (*att_line)[6],
    dvshortx (*mark_line)[6],
    dvshortx (*head_line)[6],
    dvshortx (*tail_line)[6],
    int seg_id)
{
    /*第二回波*/
    dvshortx cond_self_2 = (cur_dist_2 > 0) & (cur_dist_2 > spray_Param->Dist_threshold_min) & (cur_dist_2 <= spray_Param->Dist_threshold2)
                            & (cur_ref_2 <= spray_Param->Ref_threshold) & (~mark_line[2][4]);

    vcharx cond_cnt = vdemote(cond_self_2);
    int active_count2 = vsumr_s(cond_cnt);

    dvshortx rain_final = vec0;

    if (active_count2 > 0) {
        dvshortx head_tail_cur_2 = (cur_dist_2 > spray_Param->Dist_threshold_min) & (cur_dist_2 <= spray_Param->Dist_threshold2)
                                    & (cur_ref_2 <= spray_Param->Ref_threshold) & head_line[2][4] & tail_line[2][4];

        /*邻域统计*/
        dvshortx sum_mark_num_2 = vec0, sum_head_num_2 = vec0, sum_tail_num_2 = vec0;
        for(int i = 0; i < 5; i++){
            dvshortx mark_A = vec0, head_A = vec0, tail_A = vec0;

            vectorized_column_stats(dist_line[i], ref_mask_line[i], mark_line[i], head_line[i],
                                    tail_line[i], cur_dist_2, spray_Param->dist_diff_thr_2, i,
                                    mark_A, head_A, tail_A);
            sum_mark_num_2 += mark_A;
            sum_head_num_2 += head_A;
            sum_tail_num_2 += tail_A;
        }

        /*梯度计算*/
        dvshortx condition_mask = cur_dist_2 < spray_Param->Dist_threshold;
        dvshortx local_dist_a[6], local_dist_b[6], local_dist_c[6];
        dvshortx local_ref_mask_a[6], local_ref_mask_b[6], local_ref_mask_c[6];

        /*次发段，分辨率放一半，8m*/
        for(int i = 0; i < 6; i++){
            local_dist_a[i] = dvmux(condition_mask, dist_line[0][i], dist_line[1][i]);
            local_dist_b[i] = dist_line[2][i];
            local_dist_c[i] = dvmux(condition_mask, dist_line[4][i], dist_line[3][i]);
            local_ref_mask_a[i] = dvmux(condition_mask, ref_mask_line[0][i], ref_mask_line[1][i]);
            local_ref_mask_b[i] = ref_mask_line[2][i];
            local_ref_mask_c[i] = dvmux(condition_mask, ref_mask_line[4][i], ref_mask_line[3][i]);
        }

        dvshortx dist_diff_col[3], dist_diff_row[3];
        dist_diff_col[0] = local_dist_b[4] - local_dist_a[4];
        dist_diff_col[1] = local_dist_c[4] - local_dist_b[4];
        dist_diff_col[2] = local_dist_c[4] - local_dist_a[4];

        dist_diff_row[0] = local_dist_b[4] - local_dist_b[3];
        dist_diff_row[1] = local_dist_b[5] - local_dist_b[4];
        dist_diff_row[2] = local_dist_b[5] - local_dist_b[3];

        dvshortx dist_diff_thr, dist_diff_diff_thr;
        dvshortx dist_idx_tmp = cur_dist_2 >> 9;
        vcharx dist_idx = vdemote(dvmux(cond_self_2, dist_idx_tmp, vec0)); //获取下标
        dist_diff_thr = dvushort_load_perm(thr_seq0_agen, dist_idx);
        dist_diff_diff_thr = dvushort_load_perm(thr_seq1_agen, dist_idx);

        //梯度一致性判断
        dvshortx consistency_col_2, consistency_row_2;
        consistency_col_2 = (calc_sign(dist_diff_col[0]) == calc_sign(dist_diff_col[1]))
                && dvabsdif(dist_diff_col[0], vec0) <= dist_diff_thr && dvabsdif(dist_diff_col[1], vec0) <= dist_diff_thr
                && dvabsdif(dvabsdif(dist_diff_col[0], vec0), dvabsdif(dist_diff_col[1], vec0)) <= dist_diff_diff_thr
                || (dvabsdif(dist_diff_col[0], vec0) <= 30 && dvabsdif(dist_diff_col[1], vec0) <= 30);

        consistency_row_2 = calc_sign(dist_diff_row[0]) == calc_sign(dist_diff_row[1])
                && dvabsdif(dist_diff_row[0], vec0) <= dist_diff_thr && dvabsdif(dist_diff_row[1], vec0) <= dist_diff_thr
                && dvabsdif(dvabsdif(dist_diff_row[0], vec0), dvabsdif(dist_diff_row[1], vec0)) <= dist_diff_diff_thr
                || (dvabsdif(dist_diff_row[0], vec0) <= 30 && dvabsdif(dist_diff_row[1], vec0) <= 30);

        /*悬空计算*/
        dvshortx ground_link_valid_2 = vec0;
        dvshortx cond_glink_0 = (gnd1 == 0) && (ref_line[2][4] <= spray_Param->Ref_threshold2);
        dvshortx cond_glink_1 = (cur_dist_2 <= spray_Param->Dist_threshold && zone_idx[seg_id] > spray_Param->zone_idx_min_a && zone_idx[seg_id] < spray_Param->zone_idx_max_a)
                        || (cur_dist_2 > spray_Param->Dist_threshold && cur_dist_2 < spray_Param->Dist_threshold2 && zone_idx[seg_id] > spray_Param->zone_idx_min_b && zone_idx[seg_id] < spray_Param->zone_idx_max_b);
        ground_link_valid_2 = cond_glink_0 && cond_glink_1 && (glk1 == 1);

        /*组合判断*/
        //折波标记
        dvshortx cond0_0 = head_tail_cur_2;
        dvshortx cond0_1 =  ((sum_tail_num_2 >= spray_Param->min_tail_threshold) ||
                    (sum_head_num_2 + sum_tail_num_2 >= spray_Param->min_split_threshold));
        //雨雾标记
        dvshortx cond1 = sum_mark_num_2 >= spray_Param->min_mark_threshold;
        //梯度计算
        dvshortx cond3 = (consistency_col_2 || consistency_row_2);
        //地面标记
        dvshortx cond4 = gnd1;
        //接地判别
        dvshortx cond6 = ground_link_valid_2;

        dvshortx cond0_2 = (!head_line[2][4]) && tail_line[2][4] && (cur_dist_2 > 1000) && (cond0_1 || cond1);
        dvshortx cond0_3 = head_line[2][4] && (!tail_line[2][4]) && (cur_dist_2 > 1000) && (cond0_1 || cond1);

        rain_final = ((((cond0_1 || cond1 || cond0_2) && !cond3) || cond0_3 || cond0_0)
                                && !cond4) || (cond6 && !cond3) || ((!cond3 ) && !cond4);
        rain_final &= cond_self_2;
    }
    rain_final |= mark_line[2][4];
    vstore(rain_final, output_mask_agen);
}

void sprayRemoveExec(SprayParam_t *spray_Param, SprayConfig_t *config, agen& thr_seq0_agen, agen& thr_seq1_agen) {
    /** Initialize AGEN */
    agen input_dist_agen0 = init_agen_from_cfg(config->input_dist0);
    agen input_dist_agen1 = init_agen_from_cfg(config->input_dist1);
    agen input_ref_agen0 = init_agen_from_cfg(config->input_ref0);
    agen input_ref_agen1 = init_agen_from_cfg(config->input_ref1);
    agen input_gnd_agen0 = init_agen_from_cfg(config->input_gnd0);
    agen input_gnd_agen1 = init_agen_from_cfg(config->input_gnd1);
    agen input_att_agen0 = init_agen_from_cfg(config->input_att0);
    agen input_att_agen1 = init_agen_from_cfg(config->input_att1);
    agen input_glk_agen0 = init_agen_from_cfg(config->input_glk0);
    agen input_glk_agen1 = init_agen_from_cfg(config->input_glk1);

    agen output_mask_agen0 = init_agen_from_cfg(config->output_mask0);
    agen output_mask_agen1 = init_agen_from_cfg(config->output_mask1);

    int32_t niter = config->niter;

    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);

    dvshortx dist_line[5][6];
    dvshortx ref_line[5][6];
    dvshortx ref_mask_line[5][6];
    dvshortx att_line[5][6];
    dvshortx mark_line[5][6];
    dvshortx head_line[5][6];
    dvshortx tail_line[5][6];

    for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
    chess_unroll_loop(2)
    {
        int seg_id = i % 6;
        glk0 = dvushort_load(input_glk_agen0);
        glk1 = dvushort_load(input_glk_agen1);
        gnd0 = dvushort_load(input_gnd_agen0);
        gnd1 = dvushort_load(input_gnd_agen1);
        for (int r = 0; r < 3; r ++) {
            for (int c = 0; c < 5; c ++) {
                dist_line[c][r] = dvushort_load(input_dist_agen0);
                dist_line[c][r + 3] = dvushort_load(input_dist_agen1);

                ref_line[c][r] = dvushort_load(input_ref_agen0);
                ref_line[c][r + 3] = dvushort_load(input_ref_agen1);

                att_line[c][r] = dvushort_load(input_att_agen0);
                att_line[c][r +3] = dvushort_load(input_att_agen1);

                mark_line[c][r] = att_line[c][r] >> 7;
                mark_line[c][r + 3] = att_line[c][r + 3] >> 7;

                head_line[c][r] = (att_line[c][r] >> 1) & 0x01;
                head_line[c][r + 3] = (att_line[c][r + 3] >> 1) & 0x01;

                tail_line[c][r] = (att_line[c][r] >> 2) & 0x01;
                tail_line[c][r + 3] = (att_line[c][r + 3] >> 2) & 0x01;

                ref_mask_line[c][r] = ref_line[c][r] <= spray_Param->Ref_threshold;
                ref_mask_line[c][r + 3] = ref_line[c][r + 3] <= spray_Param->Ref_threshold;
            }
        }

        cur_dist_1 = dist_line[2][1];
        cur_ref_1 = ref_line[2][1];
        cur_dist_2 = dist_line[2][4];
        cur_ref_2 = ref_line[2][4];

        sprayRemoveWave0(output_mask_agen0, thr_seq0_agen, thr_seq1_agen,
                            dist_line, ref_line, ref_mask_line, att_line,
                            mark_line, head_line, tail_line, seg_id);
        sprayRemoveWave1(output_mask_agen1, thr_seq0_agen, thr_seq1_agen,
                            dist_line, ref_line, ref_mask_line, att_line,
                            mark_line, head_line, tail_line, seg_id);
    }
}

void agenConfigModify(SprayConfig_t *config)
{
    /** Update agen base address */
    cupvaModifyAgenCfgBase(&config->input_dist0, &inputDistBufferVMEM0[dist0_offset]);
    cupvaModifyAgenCfgBase(&config->input_dist1, &inputDistBufferVMEM1[dist1_offset]);
    cupvaModifyAgenCfgBase(&config->input_ref0, &inputRefBufferVMEM0[ref0_offset]);
    cupvaModifyAgenCfgBase(&config->input_ref1, &inputRefBufferVMEM1[ref1_offset]);
    cupvaModifyAgenCfgBase(&config->input_att0, &inputAttBufferVMEM0[att0_offset]);
    cupvaModifyAgenCfgBase(&config->input_att1, &inputAttBufferVMEM1[att1_offset]);
    cupvaModifyAgenCfgBase(&config->input_glk0, &inputGlkBufferVMEM[glk_offset]);
    cupvaModifyAgenCfgBase(&config->input_glk1, &inputGlkBufferVMEM[glk_offset + VIEW_WIDTH * 20]);
    cupvaModifyAgenCfgBase(&config->input_gnd0, &inputGlkBufferVMEM[glk_offset + VIEW_WIDTH * 40]);
    cupvaModifyAgenCfgBase(&config->input_gnd1, &inputGlkBufferVMEM[glk_offset + VIEW_WIDTH * 60]);

    cupvaModifyAgenCfgBase(&config->output_mask0, &outputRainBufferVMEM0[mask0_offset]);
    cupvaModifyAgenCfgBase(&config->output_mask1, &outputRainBufferVMEM1[mask1_offset]);
}

void offsetUpdate()
{
    dist0_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler0, dist0_offset);
    dist1_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler1, dist1_offset);

    ref0_offset = cupvaRasterDataFlowGetOffset(sourceRefDataFlowHandler0, ref0_offset);
    ref1_offset = cupvaRasterDataFlowGetOffset(sourceRefDataFlowHandler1, ref1_offset);

    att0_offset = cupvaRasterDataFlowGetOffset(sourceAttDataFlowHandler0, att0_offset);
    att1_offset = cupvaRasterDataFlowGetOffset(sourceAttDataFlowHandler1, att1_offset);

    glk_offset = cupvaRasterDataFlowGetOffset(sourceGlkDataFlowHandler, glk_offset);

    mask0_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler0, mask0_offset);
    mask1_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler1, mask1_offset);
}

void dataFlowTest(SprayConfig_t* config)
{
    agen glk0_agen = init_agen_from_cfg(config->input_glk0);
    agen glk1_agen = init_agen_from_cfg(config->input_glk1);
    agen mask0_agen = init_agen_from_cfg(config->output_mask0);
    agen mask1_agen = init_agen_from_cfg(config->output_mask1);

    int32_t niter = config->niter;
    for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
    chess_unroll_loop(2) {
        dvshortx glk0 = dvushort_load(glk0_agen);
        dvshortx glk1 = dvushort_load(glk1_agen);

        vstore(glk0, mask0_agen);
        vstore(glk1, mask1_agen);
    }
}

void initZoneIds()
{
    for (int i = 0; i < TILE_WIDTH; i ++) {
        zone_ids[i] = i / 12;
    }
}

/**
 * \brief  VPU main function
 */
CUPVA_VPU_MAIN() {
    spray_Param = (SprayParam_t *)algorithmParams;

    initZoneIds();

    SprayConfig_t config;
    agenConfigInit(&config);

    agen zone_idx_agen = init_agen_from_cfg(config.zone_idx);
    agen thr_seq0_agen = init_agen_from_cfg(config.thr_seq0);
    agen thr_seq1_agen = init_agen_from_cfg(config.thr_seq1);

    for (int i = 0; i < 6; i ++) {
        zone_idx[i] = dvushort_load(zone_idx_agen);
    }

    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler1);
    cupvaRasterDataFlowTrig(sourceRefDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceRefDataFlowHandler1);
    cupvaRasterDataFlowTrig(sourceAttDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceAttDataFlowHandler1);
    cupvaRasterDataFlowTrig(sourceGlkDataFlowHandler);

    for (int tileIdx = 0; tileIdx < TILE_COUNT; tileIdx++)
    {
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler1);
        cupvaRasterDataFlowSync(sourceRefDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceRefDataFlowHandler1);
        cupvaRasterDataFlowSync(sourceAttDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceAttDataFlowHandler1);
        cupvaRasterDataFlowSync(sourceGlkDataFlowHandler);

        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler1);
        cupvaRasterDataFlowTrig(sourceRefDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceRefDataFlowHandler1);
        cupvaRasterDataFlowTrig(sourceAttDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceAttDataFlowHandler1);
        cupvaRasterDataFlowTrig(sourceGlkDataFlowHandler);

        agenConfigModify(&config);
        offsetUpdate();
        sprayRemoveExec(spray_Param, &config, thr_seq0_agen, thr_seq1_agen);

        cupvaRasterDataFlowSync(destinationDataFlowHandler0);
        cupvaRasterDataFlowSync(destinationDataFlowHandler1);

        cupvaRasterDataFlowTrig(destinationDataFlowHandler0);
        cupvaRasterDataFlowTrig(destinationDataFlowHandler1);
    }

    cupvaRasterDataFlowSync(destinationDataFlowHandler0);
    cupvaRasterDataFlowSync(destinationDataFlowHandler1);

    return 0;
}
