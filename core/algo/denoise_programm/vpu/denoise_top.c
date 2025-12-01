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
VMEM(A, uint16_t, DistBuffer, RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(B, uint16_t, MaskBuffer, RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, ColBuffer, TILE_WIDTH);
VMEM(C, int, algorithmParams, sizeof(NoiseParam_t));

VMEM(C, RasterDataFlowHandler, DistHandler);
VMEM(C, RasterDataFlowHandler, MaskHandler);

dvshortx vec0;
dvshortx vec1;
dvshortx vec2;
typedef struct {
    AgenCFG input_dist;       // 输入dist agen
    AgenCFG center_dist;      // 当前点dist
    AgenCFG input_col;        // 输入列号
    AgenCFG output_mask;      // 输出mask agen
    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} DenoiseConfig_t;

VMEM(B, DenoiseConfig_t, DenoiseParam);
void agenConfig(uint16_t *ColIdx, int32_t LinePitch, DenoiseConfig_t *config) {
    /*获取向量宽度（每个dvshortx包含的元素数）*/
    config->vecw = pva_elementsof(dvshortx); // 32
    int32_t vecw = config->vecw;

    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = 5;
    input_wrapper.s1   = LinePitch;
    input_wrapper.n2   = 3;
    input_wrapper.s2   = 1;
    input_wrapper.n3   = TILE_WIDTH / vecw;
    input_wrapper.s3   = vecw;
    input_wrapper.n4   = TILE_HEIGHT;
    input_wrapper.s4   = LinePitch;
    agen input_agen = init((dvushort *)NULL);
    INIT_AGEN4(input_agen, input_wrapper);
    config->input_dist = extract_agen_cfg(input_agen);

    AgenWrapper center_wrapper;
    center_wrapper.size = sizeof(uint16_t);
    center_wrapper.n1   = TILE_WIDTH / vecw;
    center_wrapper.s1   = vecw;
    center_wrapper.n2   = TILE_HEIGHT;
    center_wrapper.s2   = LinePitch;
    agen center_agen = init((dvushort *)NULL);
    INIT_AGEN4(center_agen, center_wrapper);
    config->center_dist = extract_agen_cfg(center_agen);

    AgenWrapper output_wrapper;
    output_wrapper.size = sizeof(uint16_t);
    output_wrapper.n1   = TILE_WIDTH / vecw;
    output_wrapper.s1   = vecw;
    output_wrapper.n2   = TILE_HEIGHT;
    output_wrapper.s2   = TILE_WIDTH;
    agen output_agen = init((dvushort *)NULL);
    INIT_AGEN2(output_agen, output_wrapper);
    config->output_mask = extract_agen_cfg(output_agen);

    AgenWrapper col_wrapper;
    col_wrapper.size = sizeof(uint16_t);
    col_wrapper.n1   = TILE_WIDTH / vecw;
    col_wrapper.s1   = vecw;
    col_wrapper.n2   = TILE_HEIGHT;
    col_wrapper.s2   = 0;

    agen col_agen = init((dvushort *)ColIdx);
    INIT_AGEN2(col_agen, col_wrapper);
    config->input_col = extract_agen_cfg(col_agen);

    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}

void Denoise_exec(DenoiseConfig_t *config, NoiseParam_t *params)
{
    agen dist_agen        = init_agen_from_cfg(config->input_dist);
    agen center_dist_agen = init_agen_from_cfg(config->center_dist);
    agen col_agen         = init_agen_from_cfg(config->input_col);
    agen mask_agen        = init_agen_from_cfg(config->output_mask);
    int32_t niter = config->niter;
    dvshortx dist_tmp[15];
    dvshortx valid_mask[15];
    dvshortx th1,th2,th3;
    th1.lo = replicateh(128);
    th1.hi = replicateh(128);
    th2.lo = replicateh(200);
    th2.hi = replicateh(200);
    th3.lo = replicateh(300);
    th3.hi = replicateh(300);
    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(6, )
    chess_unroll_loop(2) {
        dvshortx dist_center = dvushort_load(center_dist_agen);
        dvshortx col_idx = dvushort_load(col_agen);

        dist_tmp[0]  = dvushort_load(dist_agen);
        dist_tmp[1]  = dvushort_load(dist_agen);
        dist_tmp[2]  = dvushort_load(dist_agen);
        dist_tmp[3]  = dvushort_load(dist_agen);
        dist_tmp[4]  = dvushort_load(dist_agen);
        dist_tmp[5]  = dvushort_load(dist_agen);
        dist_tmp[6]  = dvushort_load(dist_agen);
        dist_tmp[7]  = dvushort_load(dist_agen);
        dist_tmp[8]  = dvushort_load(dist_agen);
        dist_tmp[9]  = dvushort_load(dist_agen);
        dist_tmp[10] = dvushort_load(dist_agen);
        dist_tmp[11] = dvushort_load(dist_agen);
        dist_tmp[12] = dvushort_load(dist_agen);
        dist_tmp[13] = dvushort_load(dist_agen);
        dist_tmp[14] = dvushort_load(dist_agen);

        dvshortx dist_valid = dist_center > 0;

        dvshortx dist_seg = dist_center >> 12;
        dist_seg = dvmin(dist_seg, 11);
        dvshortx threshold1 = dist_seg > 2;
        /** 邻域有效性统计 */
        dvshortx diff_th = dvmux(dist_seg <= 2, th1, dvmux(dist_seg <= 8, th2, th3));

        valid_mask[5] = (dvabsdif(dist_tmp[5], dist_center) <= diff_th);
        valid_mask[6] = (dvabsdif(dist_tmp[6], dist_center) <= diff_th);
        valid_mask[7] = dist_valid;
        valid_mask[8] = (dvabsdif(dist_tmp[8], dist_center) <= diff_th);
        valid_mask[9] = (dvabsdif(dist_tmp[9], dist_center) <= diff_th);

        dvshortx valid_cnt = valid_mask[5] + valid_mask[6] + valid_mask[7]
                             + valid_mask[8] + valid_mask[9];

        /** 两侧有效性统计 */
        dvshortx valid_pre_2 = valid_mask[6] + valid_mask[8];

        dvshortx con1_1 = valid_pre_2 >= threshold1;
        dvshortx con1_2 = valid_cnt > 2;

        dvshortx con1 = con1_1 & ~con1_2;
        dvshortx con2 = con1_1 & con1_2;

        dvshortx valid_pre_3 = dvmux(con1, valid_mask[5] + valid_mask[9], vec0);

        valid_mask[1]  = dvmux(con1, (dvabsdif(dist_tmp[1], dist_center) <= diff_th), vec0);
        valid_mask[2]  = dvmux(con1, (dvabsdif(dist_tmp[2], dist_center) <= diff_th), vec0);
        valid_mask[3]  = dvmux(con1, (dvabsdif(dist_tmp[3], dist_center) <= diff_th), vec0);

        valid_mask[11] = dvmux(con1, (dvabsdif(dist_tmp[11], dist_center) <= diff_th), vec0);
        valid_mask[12] = dvmux(con1, (dvabsdif(dist_tmp[12], dist_center) <= diff_th), vec0);
        valid_mask[13] = dvmux(con1, (dvabsdif(dist_tmp[13], dist_center) <= diff_th), vec0);

        dvshortx neib_valid_num = valid_mask[1] + valid_mask[2] + valid_mask[3]
                        + valid_mask[11] + valid_mask[12] + valid_mask[13];
        dvshortx con3_1 = (neib_valid_num + valid_pre_2) >= vec1;
        dvshortx con3_2 = (col_idx == TILE_WIDTH) & (valid_pre_2 >= threshold1);
        dvshortx con3_3 = (valid_pre_3 + valid_pre_2) >= vec2;
        dvshortx con3_4 = con3_1 | con3_2 | con3_3;

        dvshortx con3 = con1 & con3_4;
        vstore(dist_valid & (con3 | con2), mask_agen);
    }
}

CUPVA_VPU_MAIN()
{
    NoiseParam_t *params = (NoiseParam_t *)algorithmParams;
    for(int i = 1; i <= TILE_WIDTH; i++){
        ColBuffer[i - 1] = i;
    }
    int32_t LinePitch = cupvaRasterDataFlowGetLinePitch(DistHandler);
    int32_t DistOffset = 0;
    int32_t MaskOffset = 0;

    cupvaRasterDataFlowTrig(DistHandler);

    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);
    vec1.lo = replicateh(1);
    vec1.hi = replicateh(1);
    vec2.lo = replicateh(2);
    vec2.hi = replicateh(2);
    agenConfig(ColBuffer, LinePitch, &DenoiseParam);

    for (int TileIdx = 0; TileIdx < TILE_CNT; TileIdx ++)
    {
        cupvaRasterDataFlowSync(DistHandler);
        cupvaRasterDataFlowTrig(DistHandler);
        cupvaModifyAgenCfgBase(&DenoiseParam.input_dist, &DistBuffer[DistOffset]);
        cupvaModifyAgenCfgBase(&DenoiseParam.center_dist, &DistBuffer[DistOffset + 2 * LinePitch + KERNEL_RADIUS_WIDTH]);
        cupvaModifyAgenCfgBase(&DenoiseParam.output_mask, &MaskBuffer[MaskOffset]);

        Denoise_exec(&DenoiseParam, params);

        DistOffset = cupvaRasterDataFlowGetOffset(DistHandler, DistOffset);
        MaskOffset = cupvaRasterDataFlowGetOffset(MaskHandler, MaskOffset);

        cupvaRasterDataFlowSync(MaskHandler);
        cupvaRasterDataFlowTrig(MaskHandler);
    }
    cupvaRasterDataFlowSync(MaskHandler);
    return 0;
}
/** [release_and_close] */

