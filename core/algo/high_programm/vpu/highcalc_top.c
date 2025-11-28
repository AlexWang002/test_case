/*******************************************************************************
 * \addtogroup highcalc_programm
 * \{
 * \file highcalc_top.c
 * \brief
 * \version 0.1
 * \date 2025-11-26
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-11-26 | Init version |
 *
 ******************************************************************************/
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_device.h>
#include <cupva_device_debug.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../highcalc_common_param.h"

/** Use double buffer, the vertical halo is 1, the horizontal halo is 2*/
VMEM(A, uint16_t, inputDistBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint16_t, inputDistBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint16_t, inputHighBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(B, uint16_t, inputHighBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(B, uint16_t, inputProjDistBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(C, uint16_t, inputProjDistBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

/** Output do not use halo */
VMEM(C, uint16_t, outputGndBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, outputGndBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));

/** declare algorithm params */
VMEM(B, int, algorithmParams, sizeof(HighcalcParam_t));
/** declare dataflow handles */
VMEM(A, RasterDataFlowHandler, sourceDistDataFlowHandler0);
VMEM(A, RasterDataFlowHandler, sourceDistDataFlowHandler1);
VMEM(A, RasterDataFlowHandler, sourceHighDataFlowHandler0);
VMEM(B, RasterDataFlowHandler, sourceHighDataFlowHandler1);
VMEM(B, RasterDataFlowHandler, sourceProjDistDataFlowHandler0);
VMEM(C, RasterDataFlowHandler, sourceProjDistDataFlowHandler1);

VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler0);
VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler1);

int32_t dist0_offset = 0, dist1_offset = 0;
int32_t high0_offset = 0, high1_offset = 0;
int32_t proj0_offset = 0, proj1_offset = 0;
int32_t mask0_offset = 0, mask1_offset = 0;
HighcalcParam_t *highcalc_Param;

dvshortx vec0;        // 0向量
dvshortx vec2;        // 1向量

int32_t dist_diff_th;
int32_t delta_z_th;
int32_t z_th;
int32_t dist_max_th;

/*二维去噪算法AGEN结构体*/
typedef struct {
    AgenCFG input_dist0;     // 输入dist0 agen
    AgenCFG input_dist1;     // 输入dist1 agen
    AgenCFG input_high0;      // 输入ref0 agen
    AgenCFG input_high1;      // 输入ref1 agen
    AgenCFG input_proj0;      // 输入att0 agen
    AgenCFG input_proj1;      // 输入att1 agen
    AgenCFG output_mask0;    // 输出mask0 agen
    AgenCFG output_mask1;    // 输出mask1 agen

    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} HighcalcConfig_t;

void agenConfigInit(HighcalcConfig_t *config)
{
    /*获取向量宽度（每个dvshortx包含的元素数）*/
    config->vecw = pva_elementsof(dvshortx); // 32
    int32_t vecw = config->vecw;

    dist_diff_th = highcalc_Param->dist_diff_th;
    delta_z_th = highcalc_Param->delta_z_th;
    z_th = highcalc_Param->z_th;
    dist_max_th = highcalc_Param->dist_max_th;

    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);
    vec2.lo = replicateh(2);
    vec2.hi = replicateh(2);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper0;
    input_wrapper0.size = sizeof(uint16_t);
    input_wrapper0.n1   = KERNEL_RADIUS_WIDTH * 2 + 1;
    input_wrapper0.s1   = 1;
    input_wrapper0.n2   = TILE_WIDTH / vecw;
    input_wrapper0.s2   = vecw;
    input_wrapper0.n3   = TILE_HEIGHT;
    input_wrapper0.s3   = TILE_WIDTH + KERNEL_RADIUS_WIDTH * 2;
    agen input_agen0 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen0, input_wrapper0);
    config->input_dist0 = extract_agen_cfg(input_agen0);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper1;
    input_wrapper1.size = sizeof(uint16_t);
    input_wrapper1.n1   = KERNEL_RADIUS_WIDTH * 2 + 1;
    input_wrapper1.s1   = 1;
    input_wrapper1.n2   = TILE_WIDTH / vecw;
    input_wrapper1.s2   = vecw;
    input_wrapper1.n3   = TILE_HEIGHT;
    input_wrapper1.s3   = TILE_WIDTH + KERNEL_RADIUS_WIDTH * 2;
    agen input_agen1 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen1, input_wrapper1);
    config->input_dist1 = extract_agen_cfg(input_agen1);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper2;
    input_wrapper2.size = sizeof(uint16_t);
    input_wrapper2.n1   = KERNEL_RADIUS_WIDTH * 2 + 1;
    input_wrapper2.s1   = 1;
    input_wrapper2.n2   = TILE_WIDTH / vecw;
    input_wrapper2.s2   = vecw;
    input_wrapper2.n3   = TILE_HEIGHT;
    input_wrapper2.s3   = TILE_WIDTH + KERNEL_RADIUS_WIDTH * 2;
    agen input_agen2 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen2, input_wrapper2);
    config->input_high0 = extract_agen_cfg(input_agen2);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper3;
    input_wrapper3.size = sizeof(uint16_t);
    input_wrapper3.n1   = KERNEL_RADIUS_WIDTH * 2 + 1;
    input_wrapper3.s1   = 1;
    input_wrapper3.n2   = TILE_WIDTH / vecw;
    input_wrapper3.s2   = vecw;
    input_wrapper3.n3   = TILE_HEIGHT;
    input_wrapper3.s3   = TILE_WIDTH + KERNEL_RADIUS_WIDTH * 2;
    agen input_agen3 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen3, input_wrapper3);
    config->input_high1 = extract_agen_cfg(input_agen3);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper4;
    input_wrapper4.size = sizeof(uint16_t);
    input_wrapper4.n1   = KERNEL_RADIUS_WIDTH * 2 + 1;
    input_wrapper4.s1   = 1;
    input_wrapper4.n2   = TILE_WIDTH / vecw;
    input_wrapper4.s2   = vecw;
    input_wrapper4.n3   = TILE_HEIGHT;
    input_wrapper4.s3   = TILE_WIDTH + KERNEL_RADIUS_WIDTH * 2;
    agen input_agen4 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen4, input_wrapper4);
    config->input_proj0 = extract_agen_cfg(input_agen4);

    /*循环次数：5 * 3 * 6 * 95*/
    AgenWrapper input_wrapper5;
    input_wrapper5.size = sizeof(uint16_t);
    input_wrapper5.n1   = KERNEL_RADIUS_WIDTH * 2 + 1;
    input_wrapper5.s1   = 1;
    input_wrapper5.n2   = TILE_WIDTH / vecw;
    input_wrapper5.s2   = vecw;
    input_wrapper5.n3   = TILE_HEIGHT;
    input_wrapper5.s3   = TILE_WIDTH + KERNEL_RADIUS_WIDTH * 2;
    agen input_agen5 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen5, input_wrapper5);
    config->input_proj1 = extract_agen_cfg(input_agen5);

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

void groundFlagCalc(dvshortx dist_cur, dvshortx* dist0,
                    dvshortx z_cur, dvshortx* z0,
                    dvshortx x_cur, dvshortx* x0,
                    dvshortx* dist1, dvshortx* z1, dvshortx* x1,
                    dvshortx* flag)
{
    dvshortx near_up_1 = (dist0[2] > 0) & (dist0[2] - dist_cur > 0) & (dist0[2] - dist_cur <= dist_diff_th);
    dvshortx near_up_2 = (dist1[2] > 0) & (dist1[2] - dist_cur > 0) & (dist1[2] - dist_cur <= dist_diff_th);
    dvshortx near_down_1 = (dist0[0] > 0) & (dist_cur - dist0[0] > 0) & (dist_cur - dist0[0] <= dist_diff_th);
    dvshortx near_down_2 = (dist1[0] > 0) & (dist_cur - dist1[0] > 0) & (dist_cur - dist1[0] <= dist_diff_th);

    dvshortx dx_up = dvabsdif(x0[2], x_cur);
    dvshortx dx_down = dvabsdif(x0[0], x_cur);
    dvshortx dx_up2 = dvabsdif(x1[2], x_cur);
    dvshortx dx_down2 = dvabsdif(x1[0], x_cur);
    dvshortx dz_up = dvabsdif(z0[2], z_cur);
    dvshortx dz_down = dvabsdif(z0[0], z_cur);
    dvshortx dz_up2 = dvabsdif(z1[2], z_cur);
    dvshortx dz_down2 = dvabsdif(z1[0], z_cur);

    dvshortx flat_flag11 = ((dz_up << 1) <= dx_up) & ((dz_down << 1) <= dx_down);
    dvshortx flat_flag12 = ((dz_up << 1) <= dx_up) & ((dz_down2 << 1) <= dx_down2);
    dvshortx flat_flag21 = ((dz_up2 << 1) <= dx_up2) & ((dz_down << 1) <= dx_down);
    dvshortx flat_flag22 = ((dz_up2 << 1) <= dx_up2) & ((dz_down2 << 1) <= dx_down2);

    // 计算角度标志 (第一回波)
    dvshortx angle_flag11 = dvabsdif(z0[2] + z0[0], (z_cur << 1)) <= delta_z_th;
    dvshortx angle_flag12 = dvabsdif(z0[2] + z1[0], (z_cur << 1)) <= delta_z_th;
    dvshortx angle_flag21 = dvabsdif(z1[2] + z0[0], (z_cur << 1)) <= delta_z_th;
    dvshortx angle_flag22 = dvabsdif(z1[2] + z1[0], (z_cur << 1)) <= delta_z_th;

    flag[0] = near_up_1 & near_down_1 & flat_flag11 & angle_flag11;
    flag[1] = near_up_1 & near_down_2 & flat_flag12 & angle_flag12;
    flag[2] = near_up_2 & near_down_1 & flat_flag21 & angle_flag21;
    flag[3] = near_up_2 & near_down_2 & flat_flag22 & angle_flag22;
}

void highcalcRemoveExec(HighcalcConfig_t *config) {
    /** Initialize AGEN */
    agen input_dist_agen0 = init_agen_from_cfg(config->input_dist0);
    agen input_dist_agen1 = init_agen_from_cfg(config->input_dist1);
    agen input_high_agen0 = init_agen_from_cfg(config->input_high0);
    agen input_high_agen1 = init_agen_from_cfg(config->input_high1);
    agen input_proj_agen0 = init_agen_from_cfg(config->input_proj0);
    agen input_proj_agen1 = init_agen_from_cfg(config->input_proj1);

    agen output_mask_agen0 = init_agen_from_cfg(config->output_mask0);
    agen output_mask_agen1 = init_agen_from_cfg(config->output_mask1);

    int32_t niter = config->niter;

    dvshortx dist0[5];
    dvshortx dist1[5];
    dvshortx x0[5];
    dvshortx x1[5];
    dvshortx z0[5];
    dvshortx z1[5];
    dvshortx flag0[4];
    dvshortx flag1[4];
    dvshortx ground_fillflag1 = vec0;
    dvshortx ground_fillflag2 = vec0;
    dvshortx gnd_mark0 = vec0;
    dvshortx gnd_mark1 = vec0;

    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_unroll_loop(2)
    {
        for(int32_t j = 0; j < 5; j++){
            dist0[j] = dvushort_load(input_dist_agen0);
            dist1[j] = dvushort_load(input_dist_agen1);
            x0[j] = dvushort_load(input_proj_agen0);
            x1[j] = dvushort_load(input_proj_agen1);
            z0[j] = dvshort_load(input_high_agen0);
            z1[j] = dvshort_load(input_high_agen1);
        }

        groundFlagCalc(dist0[3], &dist0[2], z0[3], &z0[2], x0[3], &x0[2], &dist1[2], &z1[2], &x1[2], &flag0[0]);
        groundFlagCalc(dist1[3], &dist0[2], z1[3], &z0[2], x1[3], &x0[2], &dist1[2], &z1[2], &x1[2], &flag1[0]);

        dvshortx roi_flag = (dist0[3] > 0 & dist0[3] <= dist_max_th & dvabsdif(z0[3], 0) <= z_th);
        dvshortx ground_flag = roi_flag & (flag0[0] | flag0[1] | flag0[2] | flag0[3]);
        dvshortx roi_flag2 = (dist1[3] > 0 & dist1[3] <= dist_max_th & dvabsdif(z1[3], 0) <= z_th);
        dvshortx ground_flag2 = roi_flag2 & (flag1[0] | flag1[1] | flag1[2] | flag1[3]);

        ground_fillflag1 = (ground_flag & (flag0[0] | flag0[2])) | (ground_flag2 & (flag1[0] | flag1[2]));
        ground_fillflag2 = (ground_flag & (flag0[1] | flag0[3])) | (ground_flag2 & (flag1[1] | flag1[3]));

        groundFlagCalc(dist0[1], &dist0[0], z0[1], &z0[0], x0[1], &x0[0], &dist1[0], &z1[0], &x1[0], &flag0[0]);
        groundFlagCalc(dist1[1], &dist0[0], z1[1], &z0[0], x1[1], &x0[0], &dist1[0], &z1[0], &x1[0], &flag1[0]);

        roi_flag = (dist0[1] > 0 & dist0[1] <= dist_max_th & dvabsdif(z0[1], 0) <= z_th);
        ground_flag = roi_flag & (flag0[0] | flag0[1] | flag0[2] | flag0[3]);
        roi_flag2 = (dist1[1] > 0 & dist1[1] <= dist_max_th & dvabsdif(z1[1], 0) <= z_th);
        ground_flag2 = roi_flag2 & (flag1[0] | flag1[1] | flag1[2] | flag1[3]);

        ground_fillflag1 |= (ground_flag & (flag0[0] | flag0[1])) | (ground_flag2 & (flag1[0] | flag1[1]));
        ground_fillflag2 |= (ground_flag & (flag0[2] | flag0[3])) | (ground_flag2 & (flag1[2] | flag1[3]));

        groundFlagCalc(dist0[2], &dist0[1], z0[2], &z0[1], x0[2], &x0[1], &dist1[1], &z1[1], &x1[1], &flag0[0]);
        groundFlagCalc(dist1[2], &dist0[1], z1[2], &z0[1], x1[2], &x0[1], &dist1[1], &z1[1], &x1[1], &flag1[0]);

        roi_flag = (dist0[2] > 0 & dist0[2] <= dist_max_th & dvabsdif(z0[2], 0) <= z_th);
        roi_flag2 = (dist1[2] > 0 & dist1[2] <= dist_max_th & dvabsdif(z1[2], 0) <= z_th);

        gnd_mark0 = ((roi_flag & (flag0[0] | flag0[1] | flag0[2] | flag0[3])) | ground_fillflag1)
                    | (dvabsdif(z0[2], 0) < 20 & dist0[2] < 4000);
        gnd_mark1 = (!gnd_mark0 & ((roi_flag2 & (flag1[0] | flag1[1] | flag1[2] | flag1[3])) | ground_fillflag2))
                    | (dvabsdif(z1[2], 0) < 20 & dist1[2] < 4000);
        gnd_mark0 = dvmux(gnd_mark1, vec0, gnd_mark0);

        vstore(gnd_mark0, output_mask_agen0);
        vstore(gnd_mark1, output_mask_agen1);
    }
}

void agenConfigModify(HighcalcConfig_t *config)
{
    /** Update agen base address */
    cupvaModifyAgenCfgBase(&config->input_dist0, &inputDistBufferVMEM0[dist0_offset]);
    cupvaModifyAgenCfgBase(&config->input_dist1, &inputDistBufferVMEM1[dist1_offset]);
    cupvaModifyAgenCfgBase(&config->input_high0, &inputHighBufferVMEM0[high0_offset]);
    cupvaModifyAgenCfgBase(&config->input_high1, &inputHighBufferVMEM1[high1_offset]);
    cupvaModifyAgenCfgBase(&config->input_proj0, &inputProjDistBufferVMEM0[proj0_offset]);
    cupvaModifyAgenCfgBase(&config->input_proj1, &inputProjDistBufferVMEM1[proj1_offset]);

    cupvaModifyAgenCfgBase(&config->output_mask0, &outputGndBufferVMEM0[mask0_offset]);
    cupvaModifyAgenCfgBase(&config->output_mask1, &outputGndBufferVMEM1[mask1_offset]);
}

void offsetUpdate()
{
    dist0_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler0, dist0_offset);
    dist1_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler1, dist1_offset);

    high0_offset = cupvaRasterDataFlowGetOffset(sourceHighDataFlowHandler0, high0_offset);
    high1_offset = cupvaRasterDataFlowGetOffset(sourceHighDataFlowHandler1, high1_offset);

    proj0_offset = cupvaRasterDataFlowGetOffset(sourceProjDistDataFlowHandler0, proj0_offset);
    proj1_offset = cupvaRasterDataFlowGetOffset(sourceProjDistDataFlowHandler1, proj1_offset);

    mask0_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler0, mask0_offset);
    mask1_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler1, mask1_offset);
}

/**
 * \brief  VPU main function
 */
CUPVA_VPU_MAIN() {
    highcalc_Param = (HighcalcParam_t *)algorithmParams;

    HighcalcConfig_t config;
    agenConfigInit(&config);

    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler1);
    cupvaRasterDataFlowTrig(sourceHighDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceHighDataFlowHandler1);
    cupvaRasterDataFlowTrig(sourceProjDistDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceProjDistDataFlowHandler1);

    // highcalc_agen_cfg(srcDistLinePitch, dstLinePitch, &config);
    for (int tileIdx = 0; tileIdx < TILE_COUNT; tileIdx++)
    {
        // for(int i = 0; i < TILE_WIDTH * TILE_HEIGHT; i++){
        //     outputGndBufferVMEM0[i + dstOffset] = 0;
        // }
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler1);
        cupvaRasterDataFlowSync(sourceHighDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceHighDataFlowHandler1);
        cupvaRasterDataFlowSync(sourceProjDistDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceProjDistDataFlowHandler1);

        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler1);
        cupvaRasterDataFlowTrig(sourceHighDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceHighDataFlowHandler1);
        cupvaRasterDataFlowTrig(sourceProjDistDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceProjDistDataFlowHandler1);

        agenConfigModify(&config);
        offsetUpdate();
        highcalcRemoveExec(&config);

        cupvaRasterDataFlowSync(destinationDataFlowHandler0);
        cupvaRasterDataFlowSync(destinationDataFlowHandler1);

        cupvaRasterDataFlowTrig(destinationDataFlowHandler0);
        cupvaRasterDataFlowTrig(destinationDataFlowHandler1);
    }

    cupvaRasterDataFlowSync(destinationDataFlowHandler0);
    cupvaRasterDataFlowSync(destinationDataFlowHandler1);

    return 0;
}
