/*******************************************************************************
 * \addtogroup denoise_programm
 * \{
 * \file denoise_top.c
 * \brief
 * \version 0.2
 * \date 2025-09-29
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
 * | 0.2 | 2025-09-29 | Use vpu's Wide-SIMD vector processor to accelerate denoise algo|
 * 
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.3 | 2025-11-18 | Add row id judgement|
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

dvshortx row_id[6];
dvshortx vec0;        // 0向量

/*二维去噪算法AGEN结构体*/
typedef struct {
    AgenCFG row_id;

    AgenCFG input_dist;       // 输入dist agen
    AgenCFG output_mask;      // 输出mask agen
    AgenCFG load_mask;        // 加载vpu_mask agen(包含halo)
    AgenCFG load_mask2;       // 加载vpu_mask agen2(不包含halo)
    AgenCFG store_mask;       // 存储vpu_mask agen(包含halo)

    AgenCFG pre_tile;         // 加载前一个tile 后4列 agen
    AgenCFG next_tile;        // 存储当前tile 前4列agen

    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} DenoiseConfig_t;

/**
 * 初始化函数：为所有瓦片配置AGEN和计算参数
 * @param vpu_mask: vpu临时有效缓冲区
 * @param output_lask_mask: 最后2列输出有效缓冲区
 * @param config: 输出的配置结构体
 */
void agenConfigLast(uint16_t *vpu_mask, uint16_t *output_lask_mask, DenoiseConfig_t *config)
{
    int32_t vecw = config->vecw;

    /*每一帧的最后2列*/
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
 * @param vpu_mask: vpu临时有效缓冲区
 * @param output_mask: 输出有效缓冲区
 * @param input_line_pitch: 输入行间距
 * @param config: 输出的配置结构体
 */
void agenConfig(uint16_t *input_dist, uint16_t *vpu_mask, uint16_t *output_mask, int32_t input_line_pitch, DenoiseConfig_t *config) {
    /*获取向量宽度（每个dvshortx包含的元素数）*/
    config->vecw = pva_elementsof(dvshortx); // 32
    int32_t vecw = config->vecw;

    /*循环次数：5 * 3 * 6 * 95*/
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

    /*循环次数：6 * 95*/
    AgenWrapper output_wrapper;
    output_wrapper.size = sizeof(uint16_t);
    output_wrapper.n1   = TILE_WIDTH / vecw;
    output_wrapper.s1   = vecw;
    output_wrapper.n2   = TILE_HEIGHT;
    output_wrapper.s2   = TILE_WIDTH;
    agen output_agen = init((dvushort *)output_mask);
    INIT_AGEN2(output_agen, output_wrapper);
    config->output_mask = extract_agen_cfg(output_agen);

    /*循环次数：5 * 3 * 6 * 95*/
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

    /*循环次数：6 * 95*/
    AgenWrapper load_mask_wrapper2;
    load_mask_wrapper2.size = sizeof(uint16_t);
    load_mask_wrapper2.n1 = TILE_WIDTH / vecw;
    load_mask_wrapper2.s1 = vecw;
    load_mask_wrapper2.n2 = TILE_HEIGHT;
    load_mask_wrapper2.s2 = TILE_WIDTH + 2;
    agen load_agen2  = init((dvushort *)(vpu_mask + 1));
    INIT_AGEN2(load_agen2, load_mask_wrapper2);
    config->load_mask2 = extract_agen_cfg(load_agen2);

    /*循环次数：5 * 3 * 6 * 95*/
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

    /*上个tile的最后4列*/
    AgenWrapper pre_tile_wrapper;
    pre_tile_wrapper.size = sizeof(uint16_t);
    pre_tile_wrapper.n1 = TILE_WIDTH / vecw;
    pre_tile_wrapper.s1 = vecw;
    pre_tile_wrapper.n2 = 4;
    pre_tile_wrapper.s2 = TILE_WIDTH + 2;
    agen pre_tile_agen = init((dvushort *)(&vpu_mask[(TILE_WIDTH + 2) * (TILE_HEIGHT) + 1]));
    INIT_AGEN2(pre_tile_agen, pre_tile_wrapper);
    config->pre_tile = extract_agen_cfg(pre_tile_agen);

    /*下个tile的前4列*/
    AgenWrapper next_tile_wrapper;
    next_tile_wrapper.size = sizeof(uint16_t);
    next_tile_wrapper.n1 = TILE_WIDTH / vecw;
    next_tile_wrapper.s1 = vecw;
    next_tile_wrapper.n2 = TILE_HEIGHT + 4;
    next_tile_wrapper.s2 = TILE_WIDTH + 2;
    agen next_tile_agen = init((dvushort *)(&vpu_mask[1]));
    INIT_AGEN2(next_tile_agen, next_tile_wrapper);
    config->next_tile = extract_agen_cfg(next_tile_agen);

    /*计算总迭代次数（横向向量数 × 纵向行数）*/
    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}

void initRowIds(uint16_t *row_ids)
{
    for (int i = 0; i < 192; i ++) {
        row_ids[i] = i;
    }
}

void agenConfigInit(DenoiseConfig_t *config, uint16_t *row_ids)
{
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;

    AgenWrapper row_id_wrapper;
    row_id_wrapper.size = sizeof(uint16_t);
    row_id_wrapper.n1 = TILE_WIDTH / vecw;
    row_id_wrapper.s1 = vecw;
    agen row_id_agen = init((dvushort *)row_ids);
    INIT_AGEN1(row_id_agen, row_id_wrapper);
    config->row_id = extract_agen_cfg(row_id_agen);
}

CUPVA_VPU_MAIN()
{
    NoiseParam_t *params = (NoiseParam_t *)algorithmParams;
    DenoiseConfig_t config;

    dvshortx dist[5][3];  // 5*3滑窗dist值

    uint16_t row_ids[192];
    initRowIds(row_ids);
    agenConfigInit(&config, row_ids);

    agen row_id_agen = init_agen_from_cfg(config.row_id);
    for (int i = 0; i < 6; i ++) {
        row_id[i] = dvushort_load(row_id_agen);
    }

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
        agenConfig(input_dist_vmem + src_offset, vpu_mask_vmem, output_mask, src_line_pitch, &config);

        agen input_agen = init_agen_from_cfg(config.input_dist);
        agen output_agen = init_agen_from_cfg(config.output_mask);
        agen load_mask_agen  = init_agen_from_cfg(config.load_mask);
        agen load_mask_agen2  = init_agen_from_cfg(config.load_mask2);
        agen store_mask_agen  = init_agen_from_cfg(config.store_mask);

        agen pre_agen = init_agen_from_cfg(config.pre_tile);
        agen next_agen = init_agen_from_cfg(config.next_tile);

        int32_t niter = config.niter;

        /*将前一个tile的最后4列mask作为当前tile的头4列mask*/
        for (int32_t i = 0; i < 24; i ++) chess_prepare_for_pipelining
        chess_unroll_loop(2)
        {
            dvshortx pre_tile_mask = dvushort_load(pre_agen);
            vstore(pre_tile_mask, next_agen);
        }

        /*mask值清零*/
        for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
        chess_unroll_loop(2)
        {
            vstore(vec0, next_agen);
        }

        for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
        chess_unroll_loop(2)
        {
            int seg_id = i % 6;

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

            dvshortx threshold = vec0 + 1;
            dvshortx threshold1 = dist_seg > 0;
            dvshortx threshold2 = vec0 + 2;

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

            /*初始化5*3滑窗的valid_mask*/
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
            for (int c = 0; c < 5; c ++) {
                for (int r = 0; r < 3; r ++) {
                    if (r == 1) continue;
                    /*有效性判断*/
                    cmp_mask = (dvabsdif(dist[2][1], dist[c][r]) <= diff_th);
                    int zone = params->zone_matrix[r][c];
                    if (zone > 0) {
                        valid_mask[c][r] = cmp_mask;
                    }
                    if (zone == 1) {
                        neib_valid_num += cmp_mask;
                    }
                }
            }

            /*去噪条件判断优化*/
            cond = ((neib_valid_num + valid_pre_2 >= threshold)
                    | (row_id[seg_id] == (TILE_WIDTH - 1) && (valid_pre_2 >= threshold1))
                    | (neib_valid_num + valid_pre_2 >= threshold)
                    | (valid_pre_3 + valid_pre_2 >= threshold2));

            dvshortx center_valid = dist_valid;

            for (int r = 0; r < 3; r ++) {
                for (int c = 0; c < 5; c ++) {
                    dvshortx output_valid = dvushort_load(load_mask_agen);
                    /*传播有效标记优化*/
                    dvshortx valid_tmp = dist_valid & denoise_refer_mask == 1 & cond & valid_mask[c][r];
                    if (r == 1) {
                        if (c == 2) {
                            output_valid |= (dist_valid & (denoise_refer_mask == 2 | (denoise_refer_mask == 1 & cond)));
                        }
                        else {
                            /*直接对左右补点优化*/
                            output_valid |= (denoise_refer_mask == 2 & valid_mask[c][r] & dist_valid);
                        }
                    }
                    else {
                        /*传播有效标记优化*/
                        output_valid |= (valid_tmp & row_id[seg_id] > 0 & row_id[seg_id] < (TILE_WIDTH - 1));
                    }

                    output_valid |= (valid_tmp & (row_id[seg_id] == 0 | row_id[seg_id] == (TILE_WIDTH - 1)));

                    vstore(output_valid, store_mask_agen);
                }
            }
        }

        /*去除mask数组的halo，并传回host端*/
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

