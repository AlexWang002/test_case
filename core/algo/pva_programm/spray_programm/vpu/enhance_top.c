/*******************************************************************************
 * \addtogroup spray_programm
 * \{
 * \file enhance_top.c
 * \brief
 * \version 0.2
 * \date 2025-11-28
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
 * | 0.2 | 2025-11-28 | Optimize time-consuming |
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
dvshortx vec15;

typedef struct {
    AgenCFG input_dist0;
    AgenCFG input_dist1;
    AgenCFG input_gnd0;
    AgenCFG input_gnd1;
    AgenCFG input_rain0;
    AgenCFG input_rain1;
    AgenCFG input_ref0;
    AgenCFG input_ref1;
    AgenCFG dist0_center;
    AgenCFG dist1_center;
    AgenCFG rain0_center;
    AgenCFG rain1_center;

    AgenCFG output_final0;
    AgenCFG output_final1;

    AgenCFG zone_idx;
    AgenCFG thr_seq0;
    AgenCFG thr_seq1;

    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} SprayConfig_t;
/**
 * \brief Rain enhance configuration initialization function
 *
 * \param[in] config: Spray configuration structure
 *                 Range: NA. Accuracy: NA.
 */
void agenConfigInit(SprayConfig_t *config)
{
    /*获取向量宽度（每个dvshortx包含的元素数）*/
    config->vecw = pva_elementsof(dvshortx); // 32
    int32_t vecw = config->vecw;

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

    agen input_agen1 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen1, input_wrapper0);
    config->input_dist1 = extract_agen_cfg(input_agen1);

    agen input_agen4 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen4, input_wrapper0);
    config->input_rain0 = extract_agen_cfg(input_agen4);

    agen input_agen5 = init((dvushort *)NULL);
    INIT_AGEN4(input_agen5, input_wrapper0);
    config->input_rain1 = extract_agen_cfg(input_agen5);

    AgenWrapper input_wrapper2;
    input_wrapper2.size = sizeof(uint16_t);
    input_wrapper2.n1   = TILE_WIDTH / vecw;
    input_wrapper2.s1   = vecw;
    input_wrapper2.n2   = TILE_HEIGHT;
    input_wrapper2.s2   = TILE_WIDTH;

    agen input_agen2 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen2, input_wrapper2);
    config->input_gnd0 = extract_agen_cfg(input_agen2);

    agen input_agen3 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen3, input_wrapper2);
    config->input_gnd1 = extract_agen_cfg(input_agen3);

    agen input_agen6 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen6, input_wrapper2);
    config->input_ref0 = extract_agen_cfg(input_agen6);

    agen input_agen7 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen7, input_wrapper2);
    config->input_ref1 = extract_agen_cfg(input_agen7);

    agen output_agen0 = init((dvushort *)NULL);
    INIT_AGEN2(output_agen0, input_wrapper2);
    config->output_final0 = extract_agen_cfg(output_agen0);

    agen output_agen1= init((dvushort *)NULL);
    INIT_AGEN2(output_agen1, input_wrapper2);
    config->output_final1 = extract_agen_cfg(output_agen1);

    AgenWrapper input_wrapper3;
    input_wrapper3.size = sizeof(uint16_t);
    input_wrapper3.n1   = TILE_WIDTH / vecw;
    input_wrapper3.s1   = vecw;
    input_wrapper3.n2   = TILE_HEIGHT;
    input_wrapper3.s2   = TILE_WIDTH + 2;

    agen input_agen8 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen8, input_wrapper3);
    config->dist0_center = extract_agen_cfg(input_agen8);

    agen input_agen9 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen9, input_wrapper3);
    config->dist1_center = extract_agen_cfg(input_agen9);

    agen input_agen10 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen10, input_wrapper3);
    config->rain0_center = extract_agen_cfg(input_agen10);

    agen input_agen11 = init((dvushort *)NULL);
    INIT_AGEN2(input_agen11, input_wrapper3);
    config->rain1_center = extract_agen_cfg(input_agen11);

    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}
/**
 * \brief Rain enhance algorithm execution function
 *
 * \param[in] spray_Param: Spray parameters structure
 *                 Range: NA. Accuracy: NA.
 * \param[in] config: Spray configuration structure
 *                 Range: NA. Accuracy: NA.
 */
void rainEnhanceExec(SprayParam_t *spray_Param, SprayConfig_t *config)
{
    agen input_dist_agen0 = init_agen_from_cfg(config->input_dist0);
    agen input_dist_agen1 = init_agen_from_cfg(config->input_dist1);
    agen input_ref_agen0 = init_agen_from_cfg(config->input_ref0);
    agen input_ref_agen1 = init_agen_from_cfg(config->input_ref1);
    agen input_gnd_agen0 = init_agen_from_cfg(config->input_gnd0);
    agen input_gnd_agen1 = init_agen_from_cfg(config->input_gnd1);
    agen input_rain_agen0 = init_agen_from_cfg(config->input_rain0);
    agen input_rain_agen1 = init_agen_from_cfg(config->input_rain1);

    agen dist0_center_agen = init_agen_from_cfg(config->dist0_center);
    agen dist1_center_agen = init_agen_from_cfg(config->dist1_center);

    agen rain0_center_agen = init_agen_from_cfg(config->rain0_center);
    agen rain1_center_agen = init_agen_from_cfg(config->rain1_center);

    agen output_final_agen0 = init_agen_from_cfg(config->output_final0);
    agen output_final_agen1 = init_agen_from_cfg(config->output_final1);

    int32_t niter = config->niter;

    dvshortx ref0;
    dvshortx ref1;
    dvshortx gnd0;
    dvshortx gnd1;
    dvshortx dist0_center;
    dvshortx dist1_center;
    dvshortx rain0_center;
    dvshortx rain1_center;

    dvshortx final0;
    dvshortx final1;

    for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
    chess_unroll_loop(2)
    {
        ref0 = dvushort_load(input_ref_agen0);
        dist0_center = dvushort_load(dist0_center_agen);
        rain0_center = dvushort_load(rain0_center_agen);
        dvshortx tmp_cond0 = (dist0_center > 0) & (rain0_center == 0) & (dist0_center <= spray_Param->filter_dist_max) & (ref0 <= spray_Param->filter_ref_max);
        dvshortx count_0 = dvmux(tmp_cond0, vec0, vec15);

        ref1 = dvushort_load(input_ref_agen1);
        dist1_center = dvushort_load(dist1_center_agen);
        rain1_center = dvushort_load(rain1_center_agen);
        dvshortx tmp_cond1 = (dist1_center > 0) & (rain1_center == 0) & (dist1_center <= spray_Param->filter_dist_max) & (ref1 <= spray_Param->filter_ref_max);
        dvshortx count_1 = dvmux(tmp_cond1, vec0, vec15);

        gnd0 = dvushort_load(input_gnd_agen0);
        gnd1 = dvushort_load(input_gnd_agen1);

        int diff_thr = spray_Param->dist_diff_thr;
        for (int r = 0; r < 15; r ++)
        {
            dvshortx dist0_temp = dvushort_load(input_dist_agen0);
            dvshortx dist1_temp = dvushort_load(input_dist_agen1);
            dvshortx rain0_temp = dvushort_load(input_rain_agen0);
            dvshortx rain1_temp = dvushort_load(input_rain_agen1);

            dvshortx valid0 = (dist0_temp > 0) & (rain0_temp == 0);
            dvshortx valid1 = (dist1_temp > 0) & (rain1_temp == 0);

            dvshortx cond0_1 = valid0 & (dvabsdif(dist0_temp, dist0_center) <= diff_thr);
            dvshortx cond0_2 = valid1 & (dvabsdif(dist1_temp, dist0_center) <= diff_thr);

            dvshortx cond1_1 = valid0 & (dvabsdif(dist0_temp, dist1_center) <= diff_thr);
            dvshortx cond1_2 = valid1 & (dvabsdif(dist1_temp, dist1_center) <= diff_thr);

            count_0 += (tmp_cond0 & (cond0_1 | cond0_2));
            count_1 += (tmp_cond1 & (cond1_1 | cond1_2));
        }

        dvshortx final0, final1;
        final0 = rain0_center;
        final1 = rain1_center;

        dvshortx cur_filter_mask_0 = (rain0_center | (count_0 < spray_Param->filter_count_thr)) & (gnd0 == 0);
        dvshortx cur_filter_mask_1 = (rain1_center | (count_1 < spray_Param->filter_count_thr)) & (gnd1 == 0);

        final0 = dvmux((!cur_filter_mask_0) & (!cur_filter_mask_1), final0, cur_filter_mask_0);
        final1 = dvmux((!cur_filter_mask_0) & (!cur_filter_mask_1), final1, cur_filter_mask_1);

        vstore(final0, output_final_agen0);
        vstore(final1, output_final_agen1);
    }
}
/**
 * \brief Update agent data configuration offset
 *
 * \param[in] config: Spray configuration structure
 *                 Range: NA. Accuracy: NA.
*/
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
    cupvaModifyAgenCfgBase(&config->dist0_center, &inputDistBufferVMEM0[dist0_offset + KERNEL_RADIUS_HEIGHT * 194 + KERNEL_RADIUS_WIDTH]);
    cupvaModifyAgenCfgBase(&config->dist1_center, &inputDistBufferVMEM1[dist1_offset + KERNEL_RADIUS_HEIGHT * 194 + KERNEL_RADIUS_WIDTH]);
    cupvaModifyAgenCfgBase(&config->rain0_center, &inputRainBufferVMEM0[rain0_offset + KERNEL_RADIUS_HEIGHT * 194 + KERNEL_RADIUS_WIDTH]);
    cupvaModifyAgenCfgBase(&config->rain1_center, &inputRainBufferVMEM1[rain1_offset + KERNEL_RADIUS_HEIGHT * 194 + KERNEL_RADIUS_WIDTH]);

    cupvaModifyAgenCfgBase(&config->output_final0, &outputFinalBufferVMEM0[final0_offset]);
    cupvaModifyAgenCfgBase(&config->output_final1, &outputFinalBufferVMEM1[final1_offset]);
}
/**
 * \brief Update data address offset
*/
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
/**
 * \brief Rain enhance VPU main function
*/
CUPVA_VPU_MAIN()
{
    spray_Param = (SprayParam_t *)algorithmParams;

    SprayConfig_t config;
    agenConfigInit(&config);

    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);
    vec15.lo = replicateh(15);
    vec15.hi = replicateh(15);

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