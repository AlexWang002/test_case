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

VMEM(C, uint16_t, zone_idx, TILE_WIDTH);
VMEM(C, uint16_t, wave0_cond3_buffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(C, uint16_t, wave1_cond3_buffer, TILE_WIDTH * TILE_HEIGHT);


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

SprayParam_t *spray_Param;
int32_t dist0_offset = 0, dist1_offset = 0;
int32_t ref0_offset = 0, ref1_offset = 0;
int32_t att0_offset = 0, att1_offset = 0;
int32_t glk_offset = 0, mask0_offset = 0, mask1_offset = 0;
dvshortx vec0;       // 0向量
/*二维去噪算法AGEN结构体*/
typedef struct {
    AgenCFG dist0_center;
    AgenCFG dist1_center;
    AgenCFG ref0_center;
    AgenCFG ref1_center;
    AgenCFG attr0_center;
    AgenCFG attr1_center;
    AgenCFG input_gnd0;
    AgenCFG input_gnd1;
    AgenCFG input_glk0;
    AgenCFG input_glk1;

    AgenCFG input_dist0;
    AgenCFG input_dist1;
    AgenCFG input_ref0;
    AgenCFG input_ref1;
    AgenCFG input_att0;
    AgenCFG input_att1;
    AgenCFG wave0_cond3;
    AgenCFG wave1_cond3;
    AgenCFG output_mask0;
    AgenCFG output_mask1;

    AgenCFG zone_index;
    AgenCFG thr_seq0;
    AgenCFG thr_seq1;

    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} Part1Config_t;
typedef struct {
    AgenCFG ver_dist0;
    AgenCFG ver_dist1;
    AgenCFG hor_dist0;
    AgenCFG hor_dist1;
    AgenCFG wave0_con3;
    AgenCFG wave1_con3;
    AgenCFG thr_seq0;
    AgenCFG thr_seq1;

    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} GradConfig_t;

VMEM(B, Part1Config_t, part1_cfg);
VMEM(B, GradConfig_t, grad_cfg);
/** 中心点计算 */
void Part1_Init(Part1Config_t *config, uint16_t *zone_idx, uint16_t *wave0_cond3_buffer, uint16_t *wave1_cond3_buffer)
{
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;

    AgenWrapper agen_wrapper0;
    agen_wrapper0.size = sizeof(uint16_t);
    agen_wrapper0.n1   = 5;
    agen_wrapper0.s1   = TILE_WIDTH + 2;
    agen_wrapper0.n2   = 3;
    agen_wrapper0.s2   = 1;
    agen_wrapper0.n3   = TILE_WIDTH / vecw;
    agen_wrapper0.s3   = vecw;
    agen_wrapper0.n4   = TILE_HEIGHT;
    agen_wrapper0.s4   = TILE_WIDTH + 2;
    agen dist0_agen = init((dvushort *)NULL);
    INIT_AGEN4(dist0_agen, agen_wrapper0);
    config->input_dist0 = extract_agen_cfg(dist0_agen);

    agen dist1_agen = init((dvushort *)NULL);
    INIT_AGEN4(dist1_agen, agen_wrapper0);
    config->input_dist1 = extract_agen_cfg(dist1_agen);

    agen ref0_agen = init((dvushort *)NULL);
    INIT_AGEN4(ref0_agen, agen_wrapper0);
    config->input_ref0 = extract_agen_cfg(ref0_agen);

    agen ref1_agen = init((dvushort *)NULL);
    INIT_AGEN4(ref1_agen, agen_wrapper0);
    config->input_ref1 = extract_agen_cfg(ref1_agen);

    agen attr0_agen = init((dvushort *)NULL);
    INIT_AGEN4(attr0_agen, agen_wrapper0);
    config->input_att0 = extract_agen_cfg(attr0_agen);

    agen attr1_agen = init((dvushort *)NULL);
    INIT_AGEN4(attr1_agen, agen_wrapper0);
    config->input_att1 = extract_agen_cfg(attr1_agen);

    AgenWrapper agen_wrapper1;
    agen_wrapper1.size = sizeof(uint16_t);
    agen_wrapper1.n1   = TILE_WIDTH / vecw;
    agen_wrapper1.s1   = vecw;
    agen_wrapper1.n2   = TILE_HEIGHT;
    agen_wrapper1.s2   = TILE_WIDTH;
    agen glk0_agen = init((dvushort *)NULL);
    INIT_AGEN2(glk0_agen, agen_wrapper1);
    config->input_glk0 = extract_agen_cfg(glk0_agen);

    agen glk1_agen = init((dvushort *)NULL);
    INIT_AGEN2(glk1_agen, agen_wrapper1);
    config->input_glk1 = extract_agen_cfg(glk1_agen);

    agen gnd0_agen = init((dvushort *)NULL);
    INIT_AGEN2(gnd0_agen, agen_wrapper1);
    config->input_gnd0 = extract_agen_cfg(gnd0_agen);

    agen gnd1_agen = init((dvushort *)NULL);
    INIT_AGEN2(gnd1_agen, agen_wrapper1);
    config->input_gnd1 = extract_agen_cfg(gnd1_agen);

    agen wave0_cond3_agen = init((dvushort *)wave0_cond3_buffer);
    INIT_AGEN2(wave0_cond3_agen, agen_wrapper1);
    config->wave0_cond3 = extract_agen_cfg(wave0_cond3_agen);

    agen wave1_cond3_agen = init((dvushort *)wave1_cond3_buffer);
    INIT_AGEN2(wave1_cond3_agen, agen_wrapper1);
    config->wave1_cond3 = extract_agen_cfg(wave1_cond3_agen);

    agen mask0_agen = init((dvushort *)NULL);
    INIT_AGEN2(mask0_agen, agen_wrapper1);
    config->output_mask0 = extract_agen_cfg(mask0_agen);

    agen mask1_agen= init((dvushort *)NULL);
    INIT_AGEN2(mask1_agen, agen_wrapper1);
    config->output_mask1 = extract_agen_cfg(mask1_agen);

    AgenWrapper agen_wrapper2;
    agen_wrapper2.size = sizeof(uint16_t);
    agen_wrapper2.n1   = TILE_WIDTH / vecw;
    agen_wrapper2.s1   = vecw;
    agen_wrapper2.n2   = TILE_HEIGHT;
    agen_wrapper2.s2   = TILE_WIDTH + 2;
    agen dist0_center_agen = init((dvushort *)NULL);
    INIT_AGEN2(dist0_center_agen, agen_wrapper2);
    config->dist0_center = extract_agen_cfg(dist0_center_agen);

    agen dist1_center_agen = init((dvushort *)NULL);
    INIT_AGEN2(dist1_center_agen, agen_wrapper2);
    config->dist1_center = extract_agen_cfg(dist1_center_agen);

    agen ref0_center_agen = init((dvushort *)NULL);
    INIT_AGEN2(ref0_center_agen, agen_wrapper2);
    config->ref0_center = extract_agen_cfg(ref0_center_agen);

    agen ref1_center_agen = init((dvushort *)NULL);
    INIT_AGEN2(ref1_center_agen, agen_wrapper2);
    config->ref1_center = extract_agen_cfg(ref1_center_agen);

    agen attr0_center_agen = init((dvushort *)NULL);
    INIT_AGEN2(attr0_center_agen, agen_wrapper2);
    config->attr0_center = extract_agen_cfg(attr0_center_agen);

    agen attr1_center_agen = init((dvushort *)NULL);
    INIT_AGEN2(attr1_center_agen, agen_wrapper2);
    config->attr1_center = extract_agen_cfg(attr1_center_agen);

    AgenWrapper agen_wrapper3;
    agen_wrapper3.size = sizeof(uint16_t);
    agen_wrapper3.n1 = 1;
    agen_wrapper3.s1 = 0;
    agen thr_seq0_agen = init((dvushort *)spray_Param->dist_diff_thr_seq);
    INIT_AGEN1(thr_seq0_agen, agen_wrapper3);
    config->thr_seq0 = extract_agen_cfg(thr_seq0_agen);

    agen thr_seq1_agen = init((dvushort *)spray_Param->dist_diff_diff_thr_seq);
    INIT_AGEN1(thr_seq1_agen, agen_wrapper3);
    config->thr_seq1 = extract_agen_cfg(thr_seq1_agen);

    AgenWrapper agen_wrapper4;
    agen_wrapper4.size = sizeof(uint16_t);
    agen_wrapper4.n1 = TILE_WIDTH / vecw;
    agen_wrapper4.s1 = vecw;
    agen_wrapper4.n2 = TILE_HEIGHT;
    agen_wrapper4.s2 = 0;
    agen zone_idx_agen = init((dvushort *)zone_idx);
    INIT_AGEN2(zone_idx_agen, agen_wrapper4);
    config->zone_index = extract_agen_cfg(zone_idx_agen);

    config->niter = TILE_WIDTH / config->vecw * TILE_HEIGHT;
}

void Grad_Init(GradConfig_t *config, uint16_t *wave0_cond3_buffer, uint16_t *wave1_cond3_buffer)
{
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;

    AgenWrapper agen_wrapper0;
    agen_wrapper0.size = sizeof(uint16_t);
    agen_wrapper0.n1   = 5;
    agen_wrapper0.s1   = TILE_WIDTH + 2;
    agen_wrapper0.n2   = TILE_WIDTH / vecw;
    agen_wrapper0.s2   = vecw;
    agen_wrapper0.n3   = TILE_HEIGHT;
    agen_wrapper0.s3   = TILE_WIDTH + 2;
    agen ver_dist0_agen = init((dvushort *)NULL);
    INIT_AGEN3(ver_dist0_agen, agen_wrapper0);
    config->ver_dist0 = extract_agen_cfg(ver_dist0_agen);

    agen ver_dist1_agen = init((dvushort *)NULL);
    INIT_AGEN3(ver_dist1_agen, agen_wrapper0);
    config->ver_dist1 = extract_agen_cfg(ver_dist1_agen);

    AgenWrapper agen_wrapper1;
    agen_wrapper1.size = sizeof(uint16_t);
    agen_wrapper1.n1   = 3;
    agen_wrapper1.s1   = 1;
    agen_wrapper1.n2   = TILE_WIDTH / vecw;
    agen_wrapper1.s2   = vecw;
    agen_wrapper1.n3   = TILE_HEIGHT;
    agen_wrapper1.s3   = TILE_WIDTH + 2;

    agen hor_dist0_agen = init((dvushort *)NULL);
    INIT_AGEN3(hor_dist0_agen, agen_wrapper1);
    config->hor_dist0 = extract_agen_cfg(hor_dist0_agen);

    agen hor_dist1_agen = init((dvushort *)NULL);
    INIT_AGEN3(hor_dist1_agen, agen_wrapper1);
    config->hor_dist1 = extract_agen_cfg(hor_dist1_agen);

    AgenWrapper agen_wrapper2;
    agen_wrapper2.size = sizeof(uint16_t);
    agen_wrapper2.n1   = TILE_WIDTH / vecw;
    agen_wrapper2.s1   = vecw;
    agen_wrapper2.n2   = TILE_HEIGHT;
    agen_wrapper2.s2   = TILE_WIDTH;

    agen wave0_con3_agen = init((dvushort *)wave0_cond3_buffer);
    INIT_AGEN2(wave0_con3_agen, agen_wrapper2);
    config->wave0_con3 = extract_agen_cfg(wave0_con3_agen);

    agen wave1_con3_agen = init((dvushort *)wave1_cond3_buffer);
    INIT_AGEN2(wave1_con3_agen, agen_wrapper2);
    config->wave1_con3 = extract_agen_cfg(wave1_con3_agen);

    AgenWrapper input_wrapper3;
    input_wrapper3.size = sizeof(uint16_t);
    input_wrapper3.n1 = 1;
    input_wrapper3.s1 = 0;
    agen thr_seq0_agen = init((dvushort *)spray_Param->dist_diff_thr_seq);
    INIT_AGEN1(thr_seq0_agen, input_wrapper3);
    config->thr_seq0 = extract_agen_cfg(thr_seq0_agen);

    agen thr_seq1_agen = init((dvushort *)spray_Param->dist_diff_diff_thr_seq);
    INIT_AGEN1(thr_seq1_agen, input_wrapper3);
    config->thr_seq1 = extract_agen_cfg(thr_seq1_agen);

    config->niter = TILE_WIDTH / config->vecw * TILE_HEIGHT;
}

void Part1_Config(Part1Config_t *config)
{
    cupvaModifyAgenCfgBase(&config->dist0_center, &inputDistBufferVMEM0[dist0_offset + 2 * 194 + 1]);
    cupvaModifyAgenCfgBase(&config->dist1_center, &inputDistBufferVMEM1[dist1_offset + 2 * 194 + 1]);
    cupvaModifyAgenCfgBase(&config->ref0_center, &inputRefBufferVMEM0[ref0_offset + 2 * 194 + 1]);
    cupvaModifyAgenCfgBase(&config->ref1_center, &inputRefBufferVMEM1[ref1_offset + 2 * 194 + 1]);
    cupvaModifyAgenCfgBase(&config->attr0_center, &inputAttBufferVMEM0[att0_offset + 2 * 194 + 1]);
    cupvaModifyAgenCfgBase(&config->attr1_center, &inputAttBufferVMEM1[att1_offset + 2 * 194 + 1]);

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
void Grad_Config(GradConfig_t *config)
{
    cupvaModifyAgenCfgBase(&config->ver_dist0, &inputDistBufferVMEM0[dist0_offset + 1]);
    cupvaModifyAgenCfgBase(&config->ver_dist1, &inputDistBufferVMEM1[dist1_offset + 1]);
    cupvaModifyAgenCfgBase(&config->hor_dist0, &inputDistBufferVMEM0[dist0_offset + 2 * 194]);
    cupvaModifyAgenCfgBase(&config->hor_dist1, &inputDistBufferVMEM1[dist1_offset + 2 * 194]);
}

void Part1_exec(Part1Config_t *config) {
    /** Initialize AGEN */
    agen input_dist_agen0 = init_agen_from_cfg(config->input_dist0);
    agen input_dist_agen1 = init_agen_from_cfg(config->input_dist1);
    agen input_ref_agen0 = init_agen_from_cfg(config->input_ref0);
    agen input_ref_agen1 = init_agen_from_cfg(config->input_ref1);
    agen input_att_agen0 = init_agen_from_cfg(config->input_att0);
    agen input_att_agen1 = init_agen_from_cfg(config->input_att1);
    agen input_gnd_agen0 = init_agen_from_cfg(config->input_gnd0);
    agen input_gnd_agen1 = init_agen_from_cfg(config->input_gnd1);
    agen input_glk_agen0 = init_agen_from_cfg(config->input_glk0);
    agen input_glk_agen1 = init_agen_from_cfg(config->input_glk1);

    agen dist0_center_agen = init_agen_from_cfg(config->dist0_center);
    agen dist1_center_agen = init_agen_from_cfg(config->dist1_center);
    agen ref0_center_agen = init_agen_from_cfg(config->ref0_center);
    agen ref1_center_agen = init_agen_from_cfg(config->ref1_center);
    agen attr0_center_agen = init_agen_from_cfg(config->attr0_center);
    agen attr1_center_agen = init_agen_from_cfg(config->attr1_center);

    agen wave0_cond3_agen = init_agen_from_cfg(config->wave0_cond3);
    agen wave1_cond3_agen = init_agen_from_cfg(config->wave1_cond3);
    agen zone_idx_agen = init_agen_from_cfg(config->zone_index);
    agen output_mask_agen0 = init_agen_from_cfg(config->output_mask0);
    agen output_mask_agen1 = init_agen_from_cfg(config->output_mask1);
    int32_t niter = config->niter;
    int Dist_threshold2 = spray_Param->Dist_threshold2;
    int Ref_threshold = spray_Param->Ref_threshold;
    int Dist_threshold_min = spray_Param->Dist_threshold_min;
    int Ref_threshold2 = spray_Param->Ref_threshold2;
    int Dist_threshold = spray_Param->Dist_threshold;
    int zone_idx_min_a = spray_Param->zone_idx_min_a;
    int zone_idx_max_a = spray_Param->zone_idx_max_a;
    int zone_idx_min_b = spray_Param->zone_idx_min_b;
    int zone_idx_max_b = spray_Param->zone_idx_max_b;
    int dist_diff_thr_2 = spray_Param->dist_diff_thr_2;
    int min_mark_threshold = spray_Param->min_mark_threshold;
    int min_tail_threshold = spray_Param->min_tail_threshold;
    int min_split_threshold = spray_Param->min_split_threshold;

    for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
    chess_unroll_loop(2)
    {
        dvshortx dist0_center = dvushort_load(dist0_center_agen);
        dvshortx ref0_center = dvushort_load(ref0_center_agen);
        dvshortx attr0_center = dvushort_load(attr0_center_agen);
        dvshortx mark0_center = attr0_center >> 7;
        dvshortx head0_center = (attr0_center >> 1) & 0x01;
        dvshortx tail0_center = (attr0_center >> 2) & 0x01;

        dvshortx cond_self_0 = (dist0_center > 0)
                                & (dist0_center <= Dist_threshold2)
                                & (ref0_center <= Ref_threshold)
                                & (~mark0_center);
        dvshortx dist1_center = dvushort_load(dist1_center_agen);
        dvshortx ref1_center = dvushort_load(ref1_center_agen);
        dvshortx attr1_center = dvushort_load(attr1_center_agen);
        dvshortx mark1_center = attr1_center >> 7;
        dvshortx head1_center = (attr1_center >> 1) & 0x01;
        dvshortx tail1_center = (attr1_center >> 2) & 0x01;

        dvshortx cond_self_1 = (dist1_center > 0)
                                & (dist1_center <= Dist_threshold2)
                                & (ref1_center <= Ref_threshold)
                                & (~mark1_center);

        dvshortx gnd0 = dvushort_load(input_gnd_agen0);
        dvshortx gnd1 = dvushort_load(input_gnd_agen1);
        dvshortx wave0_con4 = gnd0 > 0;
        dvshortx wave1_con4 = gnd1 > 0;

        dvshortx wave0_con0_0 = (dist0_center > Dist_threshold_min)
                                & (dist0_center <= Dist_threshold2)
                                & (ref0_center <= Ref_threshold)
                                & (head0_center > 0)
                                & (tail0_center > 0);
        dvshortx wave1_con0_0 = (dist1_center > Dist_threshold_min)
                                & (dist1_center <= Dist_threshold2)
                                & (ref1_center <= Ref_threshold)
                                & (head1_center > 0)
                                & (tail1_center > 0);

        dvshortx glink0_0 = (gnd0 == 0) & (ref0_center <= Ref_threshold2);
        dvshortx glink1_0 = (gnd1 == 0) & (ref1_center <= Ref_threshold2);
        dvshortx zone_idx = dvushort_load(zone_idx_agen);
        dvshortx glink0_1 = ((dist0_center <= Dist_threshold)
                            & (zone_idx > zone_idx_min_a)
                            & (zone_idx < zone_idx_max_a))
                            | ((dist0_center > Dist_threshold)
                            & (dist0_center < Dist_threshold2)
                            & (zone_idx > zone_idx_min_b)
                            & (zone_idx < zone_idx_max_b));
        dvshortx glink1_1 = ((dist1_center <= Dist_threshold)
                            & (zone_idx > zone_idx_min_a)
                            & (zone_idx < zone_idx_max_a))
                            | ((dist1_center > Dist_threshold)
                            & (dist1_center < Dist_threshold2)
                            & (zone_idx > zone_idx_min_b)
                            & (zone_idx < zone_idx_max_b));

        dvshortx glk0 = dvushort_load(input_glk_agen0);
        dvshortx glk1 = dvushort_load(input_glk_agen1);
        dvshortx wave0_con6 = glink0_0 & glink0_1 & glk0;
        dvshortx wave1_con6 = glink1_0 & glink1_1 & glk1;

        dvshortx wave0_mark_num = vec0;
        dvshortx wave1_mark_num = vec0;
        dvshortx wave0_head_num = vec0;
        dvshortx wave1_head_num = vec0;
        dvshortx wave0_tail_num = vec0;
        dvshortx wave1_tail_num = vec0;
        for(int j = 0; j < 15; j++)
        {
            dvshortx dist_temp0 = dvushort_load(input_dist_agen0);
            dvshortx dist_temp1 = dvushort_load(input_dist_agen1);

            dvshortx diff0_0 = dvabsdif(dist_temp0, dist0_center) <= dist_diff_thr_2;
            dvshortx diff1_0 = dvabsdif(dist_temp1, dist0_center) <= dist_diff_thr_2;
            dvshortx diff0_1 = dvabsdif(dist_temp0, dist1_center) <= dist_diff_thr_2;
            dvshortx diff1_1 = dvabsdif(dist_temp1, dist1_center) <= dist_diff_thr_2;

            dvshortx ref_temp0 = dvushort_load(input_ref_agen0);
            dvshortx ref_temp1 = dvushort_load(input_ref_agen1);
            dvshortx ref0_mask = ref_temp0 <= Ref_threshold;
            dvshortx ref1_mask = ref_temp1 <= Ref_threshold;

            dvshortx attr_temp0 = dvushort_load(input_att_agen0);
            dvshortx attr_temp1 = dvushort_load(input_att_agen1);
            dvshortx valid0_0 = (dist_temp0 > 0) & (dist0_center > 0) & ref0_mask & diff0_0;
            dvshortx valid1_0 = (dist_temp1 > 0) & (dist0_center > 0) & ref1_mask & diff1_0;

            dvshortx valid0_1 = (dist_temp0 > 0) & (dist1_center > 0) & ref0_mask & diff0_1;
            dvshortx valid1_1 = (dist_temp1 > 0) & (dist1_center > 0) & ref1_mask & diff1_1;

            dvshortx mark0 = attr_temp0 >> 7;
            dvshortx mark1 = attr_temp1 >> 7;
            wave0_mark_num += (valid0_0 & mark0) + (valid1_0 & mark1);
            wave1_mark_num += (valid0_1 & mark0) + (valid1_1 & mark1);

            dvshortx head0 = (attr_temp0 >> 1) & 0x01;
            dvshortx head1 = (attr_temp1 >> 1) & 0x01;

            wave0_head_num += (valid0_0 & head0) + (valid1_0 & head1);
            wave1_head_num += (valid0_1 & head0) + (valid1_1 & head1);

            dvshortx tail0 = (attr_temp0 >> 2) & 0x01;
            dvshortx tail1 = (attr_temp1 >> 2) & 0x01;
            wave0_tail_num += (valid0_0 & tail0) + (valid1_0 & tail1);
            wave1_tail_num += (valid0_1 & tail0) + (valid1_1 & tail1);
        }
        dvshortx wave0_con1 = wave0_mark_num >= min_mark_threshold;
        dvshortx wave1_con1 = wave1_mark_num >= min_mark_threshold;
        dvshortx wave0_con0_1 = (wave0_tail_num >= min_tail_threshold)
                                | (wave0_head_num + wave0_tail_num >= min_split_threshold);
        dvshortx wave1_con0_1 = (wave1_tail_num >= min_tail_threshold)
                                | (wave1_head_num + wave1_tail_num >= min_split_threshold);

        dvshortx wave0_con0_3 = (head0_center > 0) & (tail0_center == 0) & (dist0_center > 1000) & (wave0_con0_1 | wave0_con1);
        dvshortx wave1_con0_3 = (head1_center > 0) & (tail1_center == 0) & (dist1_center > 1000) & (wave1_con0_1 | wave1_con1);

        dvshortx wave0_con3 = dvushort_load(wave0_cond3_agen);
        dvshortx wave1_con3 = dvushort_load(wave1_cond3_agen);

        dvshortx rain0_mark = mark0_center;
        dvshortx wave0_con = (cond_self_0) & (((wave0_con3 == 0) & ((wave0_con6) | (wave0_con4 == 0)))
                            | (((wave0_con0_3) | (wave0_con0_0)) & (wave0_con4 == 0)));
        rain0_mark = rain0_mark | wave0_con;
        vstore(rain0_mark, output_mask_agen0);

        dvshortx rain1_mark = mark1_center;
        dvshortx wave1_con = cond_self_1 & (((wave1_con3 == 0) & (wave1_con6 | (wave1_con4 == 0)))
                            | ((wave1_con0_3 | wave1_con0_0) & (wave1_con4 == 0)));
        rain1_mark = rain1_mark | wave1_con;
        vstore(rain1_mark, output_mask_agen1);
    }
}

void Grad_exec(GradConfig_t *config)
{
    agen ver_dist0_agen  = init_agen_from_cfg(config->ver_dist0);
    agen ver_dist1_agen  = init_agen_from_cfg(config->ver_dist1);
    agen hor_dist0_agen  = init_agen_from_cfg(config->hor_dist0);
    agen hor_dist1_agen  = init_agen_from_cfg(config->hor_dist1);
    agen wave0_con3_agen = init_agen_from_cfg(config->wave0_con3);
    agen wave1_con3_agen = init_agen_from_cfg(config->wave1_con3);
    agen thr_seq0_agen   = init_agen_from_cfg(config->thr_seq0);
    agen thr_seq1_agen   = init_agen_from_cfg(config->thr_seq1);

    int32_t niter = config->niter;
    for(int i = 0; i < niter; i++) {
        dvshortx wave0_diff_row0, wave0_diff_row1;
        dvshortx wave0_diff_col0, wave0_diff_col1;

        dvshortx wave1_diff_row0, wave1_diff_row1;
        dvshortx wave1_diff_col0, wave1_diff_col1;

        dvshortx wave0_ver_dist0 = dvushort_load(ver_dist0_agen);
        dvshortx wave0_ver_dist1 = dvushort_load(ver_dist0_agen);
        dvshortx wave0_ver_dist2 = dvushort_load(ver_dist0_agen);
        dvshortx wave0_center_valid = wave0_ver_dist2 < spray_Param->Dist_threshold;
        wave0_diff_col0 = dvmux(wave0_center_valid,
                                    wave0_ver_dist2 - wave0_ver_dist0,
                                    wave0_ver_dist2 - wave0_ver_dist1);
        dvshortx wave0_ver_dist3 = dvushort_load(ver_dist0_agen);
        dvshortx wave0_ver_dist4 = dvushort_load(ver_dist0_agen);
        wave0_diff_col1 = dvmux(wave0_center_valid,
                                    wave0_ver_dist4 - wave0_ver_dist2,
                                    wave0_ver_dist3 - wave0_ver_dist2);

        dvshortx wave1_ver_dist0 = dvushort_load(ver_dist1_agen);
        dvshortx wave1_ver_dist1 = dvushort_load(ver_dist1_agen);
        dvshortx wave1_ver_dist2 = dvushort_load(ver_dist1_agen);
        dvshortx wave1_center_valid = wave1_ver_dist2 < spray_Param->Dist_threshold;
        wave1_diff_col0 = dvmux(wave1_center_valid,
                                    wave1_ver_dist2 - wave1_ver_dist0,
                                    wave1_ver_dist2 - wave1_ver_dist1);
        dvshortx wave1_ver_dist3 = dvushort_load(ver_dist1_agen);
        dvshortx wave1_ver_dist4 = dvushort_load(ver_dist1_agen);
        wave1_diff_col1 = dvmux(wave1_center_valid,
                                    wave1_ver_dist4 - wave1_ver_dist2,
                                    wave1_ver_dist3 - wave1_ver_dist2);

        dvshortx wave0_hor_dist0 = dvushort_load(hor_dist0_agen);
        dvshortx wave0_hor_dist1 = dvushort_load(hor_dist0_agen);

        wave0_diff_row0 = wave0_hor_dist1 - wave0_hor_dist0;
        dvshortx wave0_hor_dist2 = dvushort_load(hor_dist0_agen);
        wave0_diff_row1 = wave0_hor_dist2 - wave0_hor_dist1;

        dvshortx wave1_hor_dist0 = dvushort_load(hor_dist1_agen);
        dvshortx wave1_hor_dist1 = dvushort_load(hor_dist1_agen);
        wave1_diff_row0 = wave1_hor_dist1 - wave1_hor_dist0;
        dvshortx wave1_hor_dist2 = dvushort_load(hor_dist1_agen);
        wave1_diff_row1 = wave1_hor_dist2 - wave1_hor_dist1;

        dvshortx dist0_diff_thr, dist0_diff_diff_thr;
        dvshortx dist1_diff_thr, dist1_diff_diff_thr;
        dvshortx dist0_idx_tmp = wave0_ver_dist2 >> 9;
        dvshortx dist1_idx_tmp = wave1_ver_dist2 >> 9;
        dist0_idx_tmp = dvmin(dist0_idx_tmp, 15);
        dist1_idx_tmp = dvmin(dist1_idx_tmp, 15);
        vcharx dist0_idx = vdemote(dist0_idx_tmp);
        vcharx dist1_idx = vdemote(dist1_idx_tmp);
        dist0_diff_thr = dvushort_load_perm(thr_seq0_agen, dist0_idx);
        dist0_diff_diff_thr = dvushort_load_perm(thr_seq1_agen, dist0_idx);
        dist1_diff_thr = dvushort_load_perm(thr_seq0_agen, dist1_idx);
        dist1_diff_diff_thr = dvushort_load_perm(thr_seq1_agen, dist1_idx);
        //梯度一致性判断
        dvshortx consistency_col_0, consistency_row_0;
        dvshortx consistency_col_1, consistency_row_1;
        dvshortx col_sign, row_sign;
        col_sign = (wave0_diff_col0 > 0 & wave0_diff_col1 > 0) | (wave0_diff_col0 < 0 & wave0_diff_col1 < 0)
                    | (wave0_diff_col0 == 0 | wave0_diff_col1 == 0);
        row_sign = (wave0_diff_row0 > 0 & wave0_diff_row1 > 0) | (wave0_diff_row0 < 0 & wave0_diff_row1 < 0)
                    | (wave0_diff_row0 == 0 | wave0_diff_row1 == 0);

        consistency_col_0 = ((col_sign)
                & (dvabsdif(wave0_diff_col0, vec0) <= dist0_diff_thr)
                & (dvabsdif(wave0_diff_col1, vec0) <= dist0_diff_thr)
                & (dvabsdif(dvabsdif(wave0_diff_col0, vec0), dvabsdif(wave0_diff_col1, vec0)) <= dist0_diff_diff_thr))
                | ((dvabsdif(wave0_diff_col0, vec0) <= 30) & (dvabsdif(wave0_diff_col1, vec0) <= 30));

        consistency_row_0 = ((row_sign)
                & (dvabsdif(wave0_diff_row0, vec0) <= dist0_diff_thr)
                & (dvabsdif(wave0_diff_row1, vec0) <= dist0_diff_thr)
                & (dvabsdif(dvabsdif(wave0_diff_row0, vec0), dvabsdif(wave0_diff_row1, vec0)) <= dist0_diff_diff_thr))
                | ((dvabsdif(wave0_diff_row0, vec0) <= 30) & (dvabsdif(wave0_diff_row1, vec0) <= 30));

        col_sign = (wave1_diff_col0 > 0 & wave1_diff_col1 > 0) | (wave1_diff_col0 < 0 & wave1_diff_col1 < 0)
                    | (wave1_diff_col0 == 0 | wave1_diff_col1 == 0);
        row_sign = (wave1_diff_row0 > 0 & wave1_diff_row1 > 0) | (wave1_diff_row0 < 0 & wave1_diff_row1 < 0)
                    | (wave1_diff_row0 == 0 | wave1_diff_row1 == 0);

        consistency_col_1 = ((col_sign)
                & (dvabsdif(wave1_diff_col0, vec0) <= dist1_diff_thr)
                & (dvabsdif(wave1_diff_col1, vec0) <= dist1_diff_thr)
                & (dvabsdif(dvabsdif(wave1_diff_col0, vec0), dvabsdif(wave1_diff_col1, vec0)) <= dist1_diff_diff_thr))
                | ((dvabsdif(wave1_diff_col0, vec0) <= 30) & (dvabsdif(wave1_diff_col1, vec0) <= 30));

        consistency_row_1 = ((row_sign)
                & (dvabsdif(wave1_diff_row0, vec0) <= dist1_diff_thr)
                & (dvabsdif(wave1_diff_row1, vec0) <= dist1_diff_thr)
                & dvabsdif(dvabsdif(wave1_diff_row0, vec0), dvabsdif(wave1_diff_row1, vec0)) <= dist1_diff_diff_thr)
                | ((dvabsdif(wave1_diff_row0, vec0) <= 30) & (dvabsdif(wave1_diff_row1, vec0) <= 30));

        dvshortx wave0_con3 = consistency_col_0 | consistency_row_0;
        dvshortx wave1_con3 = consistency_col_1 | consistency_row_1;

        vstore(wave0_con3, wave0_con3_agen);
        vstore(wave1_con3, wave1_con3_agen);
    }
}

/**
 * \brief  VPU main function
 */
CUPVA_VPU_MAIN() {
    spray_Param = (SprayParam_t *)algorithmParams;

    for (int i = 0; i < TILE_WIDTH; i ++) {
        zone_idx[i] = i / 12;
    }
    vec0 = chess_dont_care(dvshortx) & 0;

    Grad_Init(&grad_cfg, wave0_cond3_buffer, wave1_cond3_buffer);
    Part1_Init(&part1_cfg, zone_idx, wave0_cond3_buffer, wave1_cond3_buffer);

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

        Grad_Config(&grad_cfg);
        Part1_Config(&part1_cfg);

        Grad_exec(&grad_cfg);
        Part1_exec(&part1_cfg);

        dist0_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler0, dist0_offset);
        dist1_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler1, dist1_offset);
        ref0_offset = cupvaRasterDataFlowGetOffset(sourceRefDataFlowHandler0, ref0_offset);
        ref1_offset = cupvaRasterDataFlowGetOffset(sourceRefDataFlowHandler1, ref1_offset);
        att0_offset = cupvaRasterDataFlowGetOffset(sourceAttDataFlowHandler0, att0_offset);
        att1_offset = cupvaRasterDataFlowGetOffset(sourceAttDataFlowHandler1, att1_offset);
        glk_offset = cupvaRasterDataFlowGetOffset(sourceGlkDataFlowHandler, glk_offset);
        mask0_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler0, mask0_offset);
        mask1_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler1, mask1_offset);

        cupvaRasterDataFlowSync(destinationDataFlowHandler0);
        cupvaRasterDataFlowSync(destinationDataFlowHandler1);

        cupvaRasterDataFlowTrig(destinationDataFlowHandler0);
        cupvaRasterDataFlowTrig(destinationDataFlowHandler1);
    }

    cupvaRasterDataFlowSync(destinationDataFlowHandler0);
    cupvaRasterDataFlowSync(destinationDataFlowHandler1);

    return 0;
}
