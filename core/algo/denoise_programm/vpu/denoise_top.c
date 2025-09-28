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
VMEM(A, int, col_idx);
VMEM(A, uint16_t, vpu_mask_vmem,
    RDF_SINGLE(uint16_t, TILE_WIDTH + 2, TILE_HEIGHT + 4));

VMEM(B, uint16_t, output_mask_vmem,
    RDF_SINGLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));

VMEM(B, uint16_t, output_last_mask_vmem,
    RDF_SINGLE(uint16_t, TILE_WIDTH, 2));

VMEM(C, int, algorithmParams, sizeof(NoiseParam_t));
VMEM(C, uint16_t, input_dist_vmem,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

VMEM(C, RasterDataFlowHandler, src_dist_dataflow_handler);
VMEM_RDF_UNIFIED(B, dst_mask_dataflow_handler);
VMEM_RDF_UNIFIED(B, dst_last_mask_dataflow_handler);

uint16_t threshold_32[32];
uint16_t threshold1_32[32];
uint16_t threshold2_32[32];
uint16_t dist_seg_32[32];

dvshortx dist[5][3];
dvshortx vec0;

typedef struct {
    AgenCFG threshold;
    AgenCFG threshold1;
    AgenCFG threshold2;
    AgenCFG dist_seg;

    AgenCFG input_dist;
    AgenCFG output_mask;
    AgenCFG load_mask;
    AgenCFG load_mask2;
    AgenCFG store_mask;

    AgenCFG pre_tile;
    AgenCFG next_tile;

    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} DenoiseConfig_t;


void agenConfigVar(uint16_t *threshold_32, uint16_t *threshold1_32, uint16_t *threshold2_32, uint16_t *dist_seg_32, DenoiseConfig_t *config) {
    // 获取向量宽度（每个dvshortx包含的元素数）
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;

    AgenWrapper threshold_wrapper;
    threshold_wrapper.size = sizeof(uint16_t);       
    threshold_wrapper.n1   = 570;                      
    threshold_wrapper.s1   = 0;                  
    agen threshold_agen = init((dvushort *)threshold_32);
    INIT_AGEN3(threshold_agen, threshold_wrapper);      
    config->threshold = extract_agen_cfg(threshold_agen);

    AgenWrapper threshold1_wrapper;
    threshold1_wrapper.size = sizeof(uint16_t);       
    threshold1_wrapper.n1   = 570;                      
    threshold1_wrapper.s1   = 0;
    agen threshold1_agen = init((dvushort *)threshold1_32);
    INIT_AGEN3(threshold1_agen, threshold1_wrapper);
    config->threshold1 = extract_agen_cfg(threshold1_agen);

    AgenWrapper threshold2_wrapper;
    threshold2_wrapper.size = sizeof(uint16_t);       
    threshold2_wrapper.n1   = 570;                      
    threshold2_wrapper.s1   = 0;                  
    agen threshold2_agen = init((dvushort *)threshold2_32);
    INIT_AGEN3(threshold2_agen, threshold2_wrapper);      
    config->threshold2 = extract_agen_cfg(threshold2_agen);

    AgenWrapper dist_seg_wrapper;
    dist_seg_wrapper.size = sizeof(uint16_t);      
    dist_seg_wrapper.n1   = 570;                    
    dist_seg_wrapper.s1   = 0;                 
    agen dist_seg_agen = init((dvushort *)dist_seg_32);
    INIT_AGEN3(dist_seg_agen, dist_seg_wrapper);      
    config->dist_seg = extract_agen_cfg(dist_seg_agen);
}

void agenConfigLast(uint16_t *vpu_mask, uint16_t *output_lask_mask, DenoiseConfig_t *config)
{
    int32_t vecw = config->vecw;
    AgenWrapper pre_tile_wrapper;
    pre_tile_wrapper.size = sizeof(uint16_t);
    pre_tile_wrapper.n1 = TILE_WIDTH / vecw;
    pre_tile_wrapper.s1 = vecw;
    pre_tile_wrapper.n2 = 2;
    pre_tile_wrapper.s2 = TILE_WIDTH + 2;
    agen pre_tile_agen = init((dvushort *)(&vpu_mask[(TILE_WIDTH + 2) * (TILE_HEIGHT) + 1]));
    INIT_AGEN2(pre_tile_agen, pre_tile_wrapper);
    config->pre_tile = extract_agen_cfg(pre_tile_agen);

    AgenWrapper next_tile_wrapper;
    next_tile_wrapper.size = sizeof(uint16_t);
    next_tile_wrapper.n1 = TILE_WIDTH / vecw;
    next_tile_wrapper.s1 = vecw;
    next_tile_wrapper.n2 = 2;
    next_tile_wrapper.s2 = TILE_WIDTH;
    agen next_tile_agen = init((dvushort *)(output_lask_mask));
    INIT_AGEN2(next_tile_agen, next_tile_wrapper);
    config->next_tile = extract_agen_cfg(next_tile_agen);
}

/**
 * 初始化函数：为所有瓦片配置AGEN和计算参数
 * @param input_dist: 输入距离数据缓冲区
 * @param output_valid: 输出有效缓冲区
 * @param input_line_pitch: 输入行间距
 * @param dst_line_pitch: 输出行间距
 * @param config: 输出的配置结构体
 */
void agenConfig(uint16_t *input_dist, uint16_t *vpu_mask, uint16_t *output_mask, int32_t input_line_pitch, DenoiseConfig_t *config) {
    // 获取向量宽度（每个dvshortx包含的元素数）
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;

    // 1. 配置水平5邻域距离数据的3维AGEN
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = 5;
    input_wrapper.s1   = input_line_pitch;
    input_wrapper.n2   = 3;
    input_wrapper.s2   = 1;
    input_wrapper.n3   = TILE_WIDTH / vecw;
    input_wrapper.s3   = vecw;
    input_wrapper.n4   = TILE_HEIGHT;
    input_wrapper.s4   = input_line_pitch;
    agen input_agen = init((dvushort *)input_dist);
    INIT_AGEN4(input_agen, input_wrapper);
    config->input_dist = extract_agen_cfg(input_agen);

    AgenWrapper output_wrapper;
    output_wrapper.size = sizeof(uint16_t);
    output_wrapper.n1   = TILE_WIDTH / vecw;
    output_wrapper.s1   = vecw;
    output_wrapper.n2   = TILE_HEIGHT;
    output_wrapper.s2   = TILE_WIDTH;
    agen output_agen = init((dvushort *)output_mask);
    INIT_AGEN2(output_agen, output_wrapper);
    config->output_mask = extract_agen_cfg(output_agen);

    AgenWrapper load_mask_wrapper;
    load_mask_wrapper.size = sizeof(uint16_t);
    load_mask_wrapper.n1 = 5;
    load_mask_wrapper.s1 = TILE_WIDTH + 2;
    load_mask_wrapper.n2 = 3;
    load_mask_wrapper.s2 = 1;
    load_mask_wrapper.n3 = TILE_WIDTH / vecw;
    load_mask_wrapper.s3 = vecw;
    load_mask_wrapper.n4 = TILE_HEIGHT;
    load_mask_wrapper.s4 = TILE_WIDTH + 2;
    agen load_agen  = init((dvushort *)vpu_mask);
    INIT_AGEN4(load_agen, load_mask_wrapper);
    config->load_mask = extract_agen_cfg(load_agen);

    AgenWrapper load_mask_wrapper2;
    load_mask_wrapper2.size = sizeof(uint16_t);
    load_mask_wrapper2.n1 = TILE_WIDTH / vecw;
    load_mask_wrapper2.s1 = vecw;
    load_mask_wrapper2.n2 = TILE_HEIGHT;
    load_mask_wrapper2.s2 = TILE_WIDTH + 2;
    agen load_agen2  = init((dvushort *)(vpu_mask + 1));
    INIT_AGEN2(load_agen2, load_mask_wrapper2);
    config->load_mask2 = extract_agen_cfg(load_agen2);

    AgenWrapper store_mask_wrapper;
    store_mask_wrapper.size = sizeof(uint16_t);
    store_mask_wrapper.n1 = 5;
    store_mask_wrapper.s1 = TILE_WIDTH + 2;
    store_mask_wrapper.n2 = 3;
    store_mask_wrapper.s2 = 1;
    store_mask_wrapper.n3 = TILE_WIDTH / vecw;
    store_mask_wrapper.s3 = vecw;
    store_mask_wrapper.n4 = TILE_HEIGHT;
    store_mask_wrapper.s4 = TILE_WIDTH + 2;
    agen store_agen  = init((dvushort *)(vpu_mask));
    INIT_AGEN4(store_agen, store_mask_wrapper);
    config->store_mask = extract_agen_cfg(store_agen);

    AgenWrapper pre_tile_wrapper;
    pre_tile_wrapper.size = sizeof(uint16_t);
    pre_tile_wrapper.n1 = TILE_WIDTH / vecw;
    pre_tile_wrapper.s1 = vecw;
    pre_tile_wrapper.n2 = 4;
    pre_tile_wrapper.s2 = TILE_WIDTH + 2;
    agen pre_tile_agen = init((dvushort *)(&vpu_mask[(TILE_WIDTH + 2) * (TILE_HEIGHT) + 1]));
    INIT_AGEN2(pre_tile_agen, pre_tile_wrapper);
    config->pre_tile = extract_agen_cfg(pre_tile_agen);

    AgenWrapper next_tile_wrapper;
    next_tile_wrapper.size = sizeof(uint16_t);
    next_tile_wrapper.n1 = TILE_WIDTH / vecw;
    next_tile_wrapper.s1 = vecw;
    next_tile_wrapper.n2 = TILE_HEIGHT + 4;
    next_tile_wrapper.s2 = TILE_WIDTH + 2;
    agen next_tile_agen = init((dvushort *)(&vpu_mask[1]));
    INIT_AGEN2(next_tile_agen, next_tile_wrapper);
    config->next_tile = extract_agen_cfg(next_tile_agen);

    // 计算总迭代次数（横向向量数 × 纵向行数）
    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}

CUPVA_VPU_MAIN()
{
    NoiseParam_t *params = (NoiseParam_t *)algorithmParams;
    DenoiseConfig_t config;

    int32_t src_line_pitch       = cupvaRasterDataFlowGetLinePitch(src_dist_dataflow_handler); // src_line_pitch的值为192：每一列（192）上下各补0个点
    int32_t dst_line_pitch       = cupvaRasterDataFlowGetLinePitch(dst_mask_dataflow_handler);

    cupvaRasterDataFlowOpen(dst_mask_dataflow_handler, &output_mask_vmem[0]);
    cupvaRasterDataFlowOpen(dst_last_mask_dataflow_handler, &output_last_mask_vmem[0]);

    uint16_t *output_last_mask  = (uint16_t *)cupvaRasterDataFlowAcquire(dst_last_mask_dataflow_handler);

    cupvaRasterDataFlowTrig(src_dist_dataflow_handler);

    int32_t src_offset = 0;
    int32_t dst_offset = 0;

    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);

    for (int tile_idx = 0; tile_idx < TILE_CNT; tile_idx ++) {
        cupvaRasterDataFlowSync(src_dist_dataflow_handler);
        cupvaRasterDataFlowTrig(src_dist_dataflow_handler);
        
        uint16_t *output_mask  = (uint16_t *)cupvaRasterDataFlowAcquire(dst_mask_dataflow_handler);
#if 1 
        agenConfig(input_dist_vmem + src_offset, vpu_mask_vmem, output_mask, src_line_pitch, &config);
        agenConfigVar(threshold_32, threshold1_32, threshold2_32, dist_seg_32, &config);
        
        agen threshold_agen = init_agen_from_cfg(config.threshold);
        agen threshold1_agen = init_agen_from_cfg(config.threshold1);
        agen threshold2_agen = init_agen_from_cfg(config.threshold2);
        agen dist_seg_agen = init_agen_from_cfg(config.dist_seg);

        agen input_agen = init_agen_from_cfg(config.input_dist);
        agen output_agen = init_agen_from_cfg(config.output_mask);
        agen load_mask_agen  = init_agen_from_cfg(config.load_mask);
        agen load_mask_agen2  = init_agen_from_cfg(config.load_mask2);
        agen store_mask_agen  = init_agen_from_cfg(config.store_mask);

        agen pre_agen = init_agen_from_cfg(config.pre_tile);
        agen next_agen = init_agen_from_cfg(config.next_tile);

        int32_t niter = config.niter;

        for (int32_t i = 0; i < 24; i ++) chess_prepare_for_pipelining
        chess_unroll_loop(2)
        {
            dvshortx pre_tile_mask = dvushort_load(pre_agen);
            vstore(pre_tile_mask, next_agen);
        }

        for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
        chess_unroll_loop(2)
        {
            vstore(vec0, next_agen);
        }

        for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
        chess_unroll_loop(2)
        {
            for (int r = 0; r < 3; r ++) {
                for (int c = 0; c < 5; c ++) {
                    dist[c][r] = dvushort_load(input_agen);
                }
            }

            dvshortx dist_seg, cmp_mask, diff_th, dist_valid;
            dist_seg = vec0;

            dist_valid = dist[2][1] > 0;

            /*获取dist_seg*/
            for (int k = 1; k <= 3; k ++) {
                cmp_mask = dist[2][1] >= (k * 12000);
                dist_seg = dvmux(cmp_mask, dist_seg + 1, dist_seg);
            }
            
            vstore(dist_seg, dist_seg_agen);
            
            /*计算threshold, threshold1, threshold2*/
            int start_row = (i % 6) * 32;
            int idx = 0;
            for (int row = start_row; row < (start_row + 32); row ++) {
                int block_id = (row + 24) / 24 - 1;
                int region = params->region_list[block_id] - 1;
                threshold_32[idx] = params->denoise_point[dist_seg_32[idx]][region][0];
                threshold1_32[idx] = params->denoise_point[dist_seg_32[idx]][region][1];
                threshold2_32[idx] = params->denoise_point[dist_seg_32[idx]][region][2];
                idx ++;
            }

            dvshortx threshold = dvushort_load(threshold_agen);
            dvshortx threshold1 = dvushort_load(threshold1_agen);
            dvshortx threshold2 = dvushort_load(threshold2_agen);

            /*获取diff_th*/
            diff_th = dist_seg;
            cmp_mask = dist_seg == 0;
            diff_th = dvmux(cmp_mask, vec0 + 128, diff_th);
            cmp_mask = dist_seg == 1;
            diff_th = dvmux(cmp_mask, vec0 + 200, diff_th);
            cmp_mask = dist_seg == 2;
            diff_th = dvmux(cmp_mask, vec0 + 200, diff_th);
            cmp_mask = dist_seg == 3;
            diff_th = dvmux(cmp_mask, vec0 + 300, diff_th);

            dvshortx valid_mask[5][3];
            for (int m = 0; m < 5; m ++) {
                for (int n = 0; n < 3; n ++) {
                    valid_mask[m][n] = vec0;
                }
            }

            dvshortx valid_cnt = vec0;

            /*邻域有效性统计优化*/
            for (int c = 0; c < 5; c ++) {
                cmp_mask = (dvabsdif(dist[2][1], dist[c][1]) <= diff_th);
                valid_mask[c][1] = cmp_mask;
                valid_cnt += cmp_mask;
            }

            /*两侧有效性统计*/
            dvshortx valid_pre_2 = valid_mask[1][1] + valid_mask[3][1];
            dvshortx valid_pre_3 = valid_mask[0][1] + valid_mask[4][1];

            dvshortx denoise_refer_mask = vec0;
            denoise_refer_mask = dvmux(valid_pre_2 >= threshold1, dvmux(valid_cnt > 2, denoise_refer_mask + 2, denoise_refer_mask + 1), denoise_refer_mask + 3);

            dvshortx neib_valid_num = vec0;
            dvshortx cond;

            /*领域窗口构建优化*/
            dvshortx valid_mask2[5][3];
            for (int c = 0; c < 5; c ++) {
                for (int r = 0; r < 3; r ++) {
                    valid_mask2[c][r] = valid_mask[c][r];
                    /*有效性判断*/
                    cmp_mask = (dvabsdif(dist[2][1], dist[c][r]) <= diff_th);
                    int zone = params->zone_matrix[r][c];
                    if (zone > 0) {
                        valid_mask2[c][r] = cmp_mask;
                    }
                    if (zone == 1) {
                        neib_valid_num +=1;
                    }
                }
            }

            /*去噪条件判断优化*/
            cond = ((neib_valid_num + valid_pre_2 >= threshold)
                    || (neib_valid_num + valid_pre_2 >= threshold)
                    || (valid_pre_3 + valid_pre_2 >= threshold2));

            dvshortx center_valid = dist_valid;

            for (int r = 0; r < 3; r ++) {
                for (int c = 0; c < 5; c ++) {
                    dvshortx output_valid = dvushort_load(load_mask_agen);
                    /*传播有效标记优化*/
                    output_valid = (dist_valid && denoise_refer_mask == 1 && cond && valid_mask2[c][r]) | output_valid;
                    if (r == 1) {
                        if (c == 2) {
                            output_valid |= (dist_valid && (denoise_refer_mask == 2 || (denoise_refer_mask == 1 && cond)));
                        }
                        else {
                            /*直接对左右补点优化*/
                            output_valid |= (denoise_refer_mask == 2 && valid_mask[c][r] && dist_valid);
                        }
                    }
                    vstore(output_valid, store_mask_agen);
                }
            }
        }

        /*去除mask数组的halo*/
        for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
        chess_unroll_loop(2)
        {
            dvshortx output_mask_32 = dvushort_load(load_mask_agen2);
            vstore(output_mask_32, output_agen);
        }

        /*传输最后2列数据*/
        if (tile_idx == TILE_CNT - 1) {
            agenConfigLast(vpu_mask_vmem, output_last_mask, &config);
            pre_agen = init_agen_from_cfg(config.pre_tile);
            next_agen = init_agen_from_cfg(config.next_tile);

            for (int32_t i = 0; i < 12; i ++) chess_prepare_for_pipelining
            chess_unroll_loop(2)
            {
                dvshortx lask_mask = dvushort_load(pre_agen);
                vstore(lask_mask, next_agen);
            }
        }

#endif
        src_offset = cupvaRasterDataFlowGetOffset(src_dist_dataflow_handler, src_offset);
        cupvaRasterDataFlowRelease(dst_mask_dataflow_handler);

    }

    cupvaRasterDataFlowRelease(dst_last_mask_dataflow_handler);

    /*Close*/
    cupvaRasterDataFlowClose(dst_mask_dataflow_handler);
    cupvaRasterDataFlowClose(dst_last_mask_dataflow_handler);

    return 0;
}
/** [release_and_close] */

