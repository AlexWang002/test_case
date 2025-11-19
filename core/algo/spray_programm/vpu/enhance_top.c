/*******************************************************************************
 * \addtogroup spray_programm
 * \{
 * \file enhance_top.c
 * \brief
 * \version 0.1
 * \date 2025-11-13
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

VMEM(B, uint16_t, inputRefBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(B, uint16_t, inputRefBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));

VMEM(B, uint16_t, inputGlkBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT * 4));

VMEM(B, uint16_t, inputRainBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(B, uint16_t, inputRainBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

/** Output do not use halo */
VMEM(C, uint16_t, outputFinalBufferVMEM0,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, outputFinalBufferVMEM1,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));

/** declare algorithm params */
VMEM(C, int, algorithmParams, sizeof(SprayParam_t));

/** declare dataflow handles */
VMEM(A, RasterDataFlowHandler, sourceDistDataFlowHandler0);
VMEM(A, RasterDataFlowHandler, sourceDistDataFlowHandler1);
VMEM(B, RasterDataFlowHandler, sourceRefDataFlowHandler0);
VMEM(B, RasterDataFlowHandler, sourceRefDataFlowHandler1);
VMEM(B, RasterDataFlowHandler, sourceGlkDataFlowHandler);
VMEM(B, RasterDataFlowHandler, sourceRainDataFlowHandler0);
VMEM(B, RasterDataFlowHandler, sourceRainDataFlowHandler1);

VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler0);
VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler1);

SprayParam_t *spray_Param;
int32_t dist0_offset = 0, dist1_offset = 0, ref0_offset = 0, ref1_offset = 0;
int32_t glk_offset = 0, rain0_offset = 0, rain1_offset = 0;
int32_t final0_offset = 0, final1_offset = 0;
dvshortx vec0;

/*二维去噪算法AGEN结构体*/
typedef struct {
    AgenCFG input_dist0;     // 输入dist0 agen
    AgenCFG input_dist1;     // 输入dist1 agen
    AgenCFG input_gnd0;
    AgenCFG input_gnd1;
    AgenCFG input_rain0;      //
    AgenCFG input_rain1;      //
    AgenCFG input_ref0;
    AgenCFG input_ref1;

    AgenCFG output_final0;    // 输出mask0 agen
    AgenCFG output_final1;    // 输出mask1 agen

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

    /*循环次数：6 * 95*/
    AgenWrapper input_wrapper2;
    input_wrapper2.size = sizeof(uint16_t);
    input_wrapper2.n1   = TILE_WIDTH / vecw;
    input_wrapper2.s1   = vecw;
    input_wrapper2.n2   = TILE_HEIGHT;
    input_wrapper2.s2   = TILE_WIDTH;
    agen input_agen2 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen2, input_wrapper2);
    config->input_gnd0 = extract_agen_cfg(input_agen2);

    /*循环次数：6 * 95*/
    AgenWrapper input_wrapper3;
    input_wrapper3.size = sizeof(uint16_t);
    input_wrapper3.n1   = TILE_WIDTH / vecw;
    input_wrapper3.s1   = vecw;
    input_wrapper3.n2   = TILE_HEIGHT;
    input_wrapper3.s2   = TILE_WIDTH;
    agen input_agen3 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen3, input_wrapper3);
    config->input_gnd1 = extract_agen_cfg(input_agen3);

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
    config->input_rain0 = extract_agen_cfg(input_agen4);

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
    config->input_rain1 = extract_agen_cfg(input_agen5);

    /*ref0 agen, 循环次数：6 * 95*/
    AgenWrapper input_wrapper6;
    input_wrapper6.size = sizeof(uint16_t);
    input_wrapper6.n1   = TILE_WIDTH / vecw;
    input_wrapper6.s1   = vecw;
    input_wrapper6.n2   = TILE_HEIGHT;
    input_wrapper6.s2   = TILE_WIDTH;
    agen input_agen6 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen6, input_wrapper6);
    config->input_ref0 = extract_agen_cfg(input_agen6);

    /*ref1 agen, 循环次数：6 * 95*/
    AgenWrapper input_wrapper7;
    input_wrapper7.size = sizeof(uint16_t);
    input_wrapper7.n1   = TILE_WIDTH / vecw;
    input_wrapper7.s1   = vecw;
    input_wrapper7.n2   = TILE_HEIGHT;
    input_wrapper7.s2   = TILE_WIDTH;
    agen input_agen7 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen7, input_wrapper7);
    config->input_ref1 = extract_agen_cfg(input_agen7);

    /*循环次数：6 * 95*/
    AgenWrapper output_wrapper0;
    output_wrapper0.size = sizeof(uint16_t);
    output_wrapper0.n1   = TILE_WIDTH / vecw;
    output_wrapper0.s1   = vecw;
    output_wrapper0.n2   = TILE_HEIGHT;
    output_wrapper0.s2   = TILE_WIDTH;
    agen output_agen0 = init((dvushort *)NULL);
    INIT_AGEN2(output_agen0, output_wrapper0);
    config->output_final0 = extract_agen_cfg(output_agen0);

    /*循环次数：6 * 95*/
    AgenWrapper output_wrapper1;
    output_wrapper1.size = sizeof(uint16_t);
    output_wrapper1.n1   = TILE_WIDTH / vecw;
    output_wrapper1.s1   = vecw;
    output_wrapper1.n2   = TILE_HEIGHT;
    output_wrapper1.s2   = TILE_WIDTH;
    agen output_agen1= init((dvushort *)NULL);
    INIT_AGEN2(output_agen1, output_wrapper1);
    config->output_final1 = extract_agen_cfg(output_agen1);

    /*计算总迭代次数（横向向量数 × 纵向行数）*/
    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}

void rainEnhanceExec(SprayParam_t *spray_Param, SprayConfig_t *config)
{
    /*Initialize AGEN*/
    agen input_dist_agen0 = init_agen_from_cfg(config->input_dist0);
    agen input_dist_agen1 = init_agen_from_cfg(config->input_dist1);
    agen input_ref_agen0 = init_agen_from_cfg(config->input_ref0);
    agen input_ref_agen1 = init_agen_from_cfg(config->input_ref1);
    agen input_gnd_agen0 = init_agen_from_cfg(config->input_gnd0);
    agen input_gnd_agen1 = init_agen_from_cfg(config->input_gnd1);
    agen input_rain_agen0 = init_agen_from_cfg(config->input_rain0);
    agen input_rain_agen1 = init_agen_from_cfg(config->input_rain1);

    agen output_final_agen0 = init_agen_from_cfg(config->output_final0);
    agen output_final_agen1 = init_agen_from_cfg(config->output_final1);

    int32_t niter = config->niter;

    dvshortx dist0[5][3];  // 5*3滑窗dist值
    dvshortx dist1[5][3];
    dvshortx ref0;
    dvshortx ref1;
    dvshortx gnd0;
    dvshortx gnd1;
    dvshortx rain0[5][3];
    dvshortx rain1[5][3];

    dvshortx final0;
    dvshortx final1;

    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);

    //printf("[filter_dist_max] %d\n", spray_Param->filter_dist_max);

    for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
    chess_unroll_loop(2)
    {
        ref0 = dvushort_load(input_ref_agen0);
        ref1 = dvushort_load(input_ref_agen1);
        gnd0 = dvushort_load(input_gnd_agen0);
        gnd1 = dvushort_load(input_gnd_agen1);
        for (int r = 0; r < 3; r ++) {
            for (int c = 0; c < 5; c ++) {
                dist0[c][r] = dvushort_load(input_dist_agen0);
                dist1[c][r] = dvushort_load(input_dist_agen1);
                rain0[c][r] = dvushort_load(input_rain_agen0);
                rain1[c][r] = dvushort_load(input_rain_agen1);
            }
        }

        /*rain enhance part*/
        dvshortx count_0;
        count_0.lo = replicateh(15);
        count_0.hi = replicateh(15);

        dvshortx tmp_cond0 = dist0[2][1] && (rain0[2][1] == 0) && dist0[2][1] <= spray_Param->filter_dist_max && ref0 <= spray_Param->filter_ref_max;
        //dvshortx tmp_cond0 = dist0[2][1] <= spray_Param->filter_dist_max;
        count_0 = dvmux(tmp_cond0, vec0, count_0);

        dvshortx count_1;
        count_1.lo = replicateh(15);
        count_1.hi = replicateh(15);

        dvshortx tmp_cond1 = dist1[2][1] && (rain1[2][1] == 0) && dist1[2][1] <= spray_Param->filter_dist_max && ref1 <= spray_Param->filter_ref_max;
        count_1 = dvmux(tmp_cond1, vec0, count_1);

        for (int i = 0; i < RAIN_SIZE_X; i++) {
            for (int j = 0; j < RAIN_SIZE_Y; j++) {
                if (i == CENTER_X && j == CENTER_Y) continue;

                // 计算第一回波条件
                dvshortx cond0_1 = (dist0[i][j] != 0) && (rain0[i][j] == 0) &&
                                (dvabsdif(dist0[i][j], dist0[2][1]) <= spray_Param->dist_diff_thr);
                dvshortx cond0_2 = (dist1[i][j] != 0) && (rain1[i][j] == 0) &&
                                (dvabsdif(dist1[i][j], dist0[2][1]) <= spray_Param->dist_diff_thr);

                // 计算第二回波条件
                dvshortx cond1_1 = (dist0[i][j] != 0) && (rain0[i][j] == 0) &&
                                (dvabsdif(dist0[i][j], dist1[2][1]) <= spray_Param->dist_diff_thr);
                dvshortx cond1_2 = (dist1[i][j] != 0) && (rain1[i][j] == 0) &&
                                (dvabsdif(dist1[i][j], dist1[2][1]) <= spray_Param->dist_diff_thr);

                count_0 += (cond0_1 || cond0_2) && tmp_cond0;
                count_1 += (cond1_1 || cond1_2) && tmp_cond1;
            }
        }

        dvshortx cur_filter_mask_0 = vec0;
        cur_filter_mask_0 = (rain0[2][1] || (count_0 < spray_Param->filter_count_thr)) && (gnd0 == 0);

        dvshortx cur_filter_mask_1 = vec0;
        cur_filter_mask_1 = (rain1[2][1] || (count_1 < spray_Param->filter_count_thr)) && (gnd1 == 0);


        dvshortx final0 = rain0[2][1];
        final0 |= cur_filter_mask_0;
        final0 = dvmux((!cur_filter_mask_0) && cur_filter_mask_1, vec0, final0);

        dvshortx final1 = rain1[2][1];
        final1 |= cur_filter_mask_1;
        final1 = dvmux(cur_filter_mask_0 && (!cur_filter_mask_1), vec0, final1);

        vstore(final0, output_final_agen0);
        vstore(final1, output_final_agen1);
    }
}

void agenConfigModify(SprayConfig_t *config)
{
    /** Update agen base address */
    cupvaModifyAgenCfgBase(&config->input_dist0, &inputDistBufferVMEM0[dist0_offset]);
    cupvaModifyAgenCfgBase(&config->input_dist1, &inputDistBufferVMEM1[dist1_offset]);
    cupvaModifyAgenCfgBase(&config->input_ref0, &inputRefBufferVMEM0[ref0_offset]);
    cupvaModifyAgenCfgBase(&config->input_ref1, &inputRefBufferVMEM1[ref1_offset]);
    cupvaModifyAgenCfgBase(&config->input_gnd0, &inputGlkBufferVMEM[glk_offset + VIEW_WIDTH * 40]);
    cupvaModifyAgenCfgBase(&config->input_gnd1, &inputGlkBufferVMEM[glk_offset + VIEW_WIDTH * 60]);
    cupvaModifyAgenCfgBase(&config->input_rain0, &inputRainBufferVMEM0[rain0_offset]);
    cupvaModifyAgenCfgBase(&config->input_rain1, &inputRainBufferVMEM1[rain1_offset]);

    cupvaModifyAgenCfgBase(&config->output_final0, &outputFinalBufferVMEM0[final0_offset]);
    cupvaModifyAgenCfgBase(&config->output_final1, &outputFinalBufferVMEM1[final1_offset]);
}

void offsetUpdate()
{
    dist0_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler0, dist0_offset);
    dist1_offset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler1, dist1_offset);

    ref0_offset = cupvaRasterDataFlowGetOffset(sourceRefDataFlowHandler0, ref0_offset);
    ref1_offset = cupvaRasterDataFlowGetOffset(sourceRefDataFlowHandler1, ref1_offset);

    glk_offset = cupvaRasterDataFlowGetOffset(sourceGlkDataFlowHandler, glk_offset);

    rain0_offset = cupvaRasterDataFlowGetOffset(sourceRainDataFlowHandler0, rain0_offset);
    rain1_offset = cupvaRasterDataFlowGetOffset(sourceRainDataFlowHandler1, rain1_offset);

    final0_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler0, final0_offset);
    final1_offset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler1, final1_offset);
}

CUPVA_VPU_MAIN()
{
    spray_Param = (SprayParam_t *)algorithmParams;

    SprayConfig_t config;
    agenConfigInit(&config);

    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler1);
    cupvaRasterDataFlowTrig(sourceRefDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceRefDataFlowHandler1);
    cupvaRasterDataFlowTrig(sourceGlkDataFlowHandler);
    cupvaRasterDataFlowTrig(sourceRainDataFlowHandler0);
    cupvaRasterDataFlowTrig(sourceRainDataFlowHandler1);

    for (int tileIdx = 0; tileIdx < TILE_COUNT; tileIdx ++) {
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler1);
        cupvaRasterDataFlowSync(sourceRefDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceRefDataFlowHandler1);
        cupvaRasterDataFlowSync(sourceGlkDataFlowHandler);
        cupvaRasterDataFlowSync(sourceRainDataFlowHandler0);
        cupvaRasterDataFlowSync(sourceRainDataFlowHandler1);

        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler1);
        cupvaRasterDataFlowTrig(sourceRefDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceRefDataFlowHandler1);
        cupvaRasterDataFlowTrig(sourceGlkDataFlowHandler);
        cupvaRasterDataFlowTrig(sourceRainDataFlowHandler0);
        cupvaRasterDataFlowTrig(sourceRainDataFlowHandler1);

        agenConfigModify(&config);
        offsetUpdate();
        rainEnhanceExec(spray_Param, &config);

        cupvaRasterDataFlowSync(destinationDataFlowHandler0);
        cupvaRasterDataFlowSync(destinationDataFlowHandler1);

        cupvaRasterDataFlowTrig(destinationDataFlowHandler0);
        cupvaRasterDataFlowTrig(destinationDataFlowHandler1);
    }

    cupvaRasterDataFlowSync(destinationDataFlowHandler0);
    cupvaRasterDataFlowSync(destinationDataFlowHandler1);

    return 0;
}