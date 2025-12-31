/*******************************************************************************
 * \addtogroup stray_programm
 * \{
 * \file stray_top.c
 * \brief
 * \version 0.2
 * \date 2025-12-02
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-11 | Init version |
 * | 0.2 | 2025-12-02 | Add comments |
 *
 *
 ******************************************************************************/
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_device.h> /* Main device-side header file */

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../stray_common_param.h"

/** [declare_algorithm_params] */
VMEM(A, uint16_t, raw_data,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT * RAW_DATA_CNT));
VMEM(A, RasterDataFlowHandler, raw_data_handler);

VMEM(B, uint16_t, dist_wave,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT * 2, KR_W_DIST, KR_H_DIST));
VMEM(B, RasterDataFlowHandler, dist_wave_handler);

VMEM(B, uint16_t, att0,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KR_W_PEAK, KR_H_PEAK));
VMEM(B, RasterDataFlowHandler, att0_handler);

VMEM(B, uint16_t, att1,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KR_W_PEAK, KR_H_PEAK));
VMEM(B, RasterDataFlowHandler, att1_handler);

VMEM(B, uint16_t, class_line,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KR_W_CLASS, KR_H_CLASS));
VMEM(B, RasterDataFlowHandler, class_line_handler);

VMEM(B, uint16_t, ground_height,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(B, RasterDataFlowHandler, ground_height_handler);

VMEM(B, uint16_t, stray_var,
        RDF_SINGLE(uint16_t, STRAY_VAR_CNT, VIEW_HEIGHT));
VMEM_RDF_UNIFIED(B, stray_var_handler);

VMEM(B, int, rainwall_dist, sizeof(int));
VMEM(B, int, rainwall_cnt, sizeof(int));

VMEM(C, uint16_t, stray_mask0,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(C, RasterDataFlowHandler, stray_mask0_handler);

VMEM(C, uint16_t, stray_mask1,
        RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(C, RasterDataFlowHandler, stray_mask1_handler);

int32_t dist_offset = 0, att0_offset = 0, att1_offset = 0, class_offset = 0, ground_offset = 0, raw_offset = 0;
int32_t stray_mask0_offset = 0, stray_mask1_offset = 0;

dvshortx row_id[6];
dvshortx vec0;        // 0向量

typedef struct {
    /*input agen*/
    AgenCFG row_id;

    AgenCFG dist_wave0;
    AgenCFG dist_wave1;
    AgenCFG att0;
    AgenCFG att1;
    AgenCFG class_line;
    AgenCFG ground_height;

    AgenCFG refl_wave0;
    AgenCFG refl_wave1;
    AgenCFG grnd_wave0;
    AgenCFG grnd_wave1;
    AgenCFG high_wave0;
    AgenCFG high_wave1;


    /*output agen*/
    AgenCFG stray_mask0;
    AgenCFG stray_mask1;

    int32_t niter;
    int32_t vecw;
} StrayConfig_t;

/**
 * \brief Initialize agen configuration
 *
 * \param[in]    config   : agen configuration
 * \param[in]    config   : row id array
 * \param[out]   config   : agen configuration
*/
void agenConfigInit(StrayConfig_t *config, uint16_t *row_ids)
{
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;
    config->niter = 60;

    AgenWrapper row_id_wrapper;
    row_id_wrapper.size = sizeof(uint16_t);
    row_id_wrapper.n1 = TILE_WIDTH / vecw;
    row_id_wrapper.s1 = vecw;
    agen row_id_agen = init((dvushort *)row_ids);
    INIT_AGEN1(row_id_agen, row_id_wrapper);
    config->row_id = extract_agen_cfg(row_id_agen);

    AgenWrapper dist_wave0_wrapper;
    dist_wave0_wrapper.size = sizeof(uint16_t);
    dist_wave0_wrapper.n1 = 3;
    dist_wave0_wrapper.s1 = 1;
    dist_wave0_wrapper.n2 = TILE_WIDTH / vecw;
    dist_wave0_wrapper.s2 = vecw;
    dist_wave0_wrapper.n3 = TILE_HEIGHT;
    dist_wave0_wrapper.s3 = (TILE_WIDTH + 2);
    agen dist_wave0_agen = init((dvushort *)NULL);
    INIT_AGEN3(dist_wave0_agen, dist_wave0_wrapper);
    config->dist_wave0 = extract_agen_cfg(dist_wave0_agen);

    AgenWrapper dist_wave1_wrapper;
    dist_wave1_wrapper.size = sizeof(uint16_t);
    dist_wave1_wrapper.n1 = TILE_WIDTH / vecw;
    dist_wave1_wrapper.s1 = vecw;
    dist_wave1_wrapper.n2 = TILE_HEIGHT;
    dist_wave1_wrapper.s2 = (TILE_WIDTH + 2);
    agen dist_wave1_agen = init((dvushort *)NULL);
    INIT_AGEN2(dist_wave1_agen, dist_wave1_wrapper);
    config->dist_wave1 = extract_agen_cfg(dist_wave1_agen);

    AgenWrapper att0_wrapper;
    att0_wrapper.size = sizeof(uint16_t);
    att0_wrapper.n1 = 3;
    att0_wrapper.s1 = TILE_WIDTH;
    att0_wrapper.n2 = TILE_WIDTH / vecw;
    att0_wrapper.s2 = vecw;
    att0_wrapper.n3 = TILE_HEIGHT;
    att0_wrapper.s3 = TILE_WIDTH;
    agen att0_agen = init((dvushort *)NULL);
    INIT_AGEN3(att0_agen, att0_wrapper);
    config->att0 = extract_agen_cfg(att0_agen);

    AgenWrapper att1_wrapper;
    att1_wrapper.size = sizeof(uint16_t);
    att1_wrapper.n1 = 3;
    att1_wrapper.s1 = TILE_WIDTH;
    att1_wrapper.n2 = TILE_WIDTH / vecw;
    att1_wrapper.s2 = vecw;
    att1_wrapper.n3 = TILE_HEIGHT;
    att1_wrapper.s3 = TILE_WIDTH;
    agen att1_agen = init((dvushort *)NULL);
    INIT_AGEN3(att1_agen, att1_wrapper);
    config->att1 = extract_agen_cfg(att1_agen);

    AgenWrapper class_line_wrapper;
    class_line_wrapper.size = sizeof(uint16_t);
    class_line_wrapper.n1 = 3;
    class_line_wrapper.s1 = 1;
    class_line_wrapper.n2 = TILE_WIDTH / vecw;
    class_line_wrapper.s2 = vecw;
    class_line_wrapper.n3 = TILE_HEIGHT;
    class_line_wrapper.s3 = TILE_WIDTH + 2;
    agen class_line_agen = init((dvushort *)NULL);
    INIT_AGEN3(class_line_agen, class_line_wrapper);
    config->class_line = extract_agen_cfg(class_line_agen);

    AgenWrapper ground_height_wrapper;
    ground_height_wrapper.size = sizeof(int16_t);
    ground_height_wrapper.n1 = TILE_WIDTH / vecw;
    ground_height_wrapper.s1 = vecw;
    ground_height_wrapper.n2 = TILE_HEIGHT;
    ground_height_wrapper.s2 = TILE_WIDTH;
    agen ground_height_agen = init((dvshort *)NULL);
    INIT_AGEN2(ground_height_agen, ground_height_wrapper);
    config->ground_height = extract_agen_cfg(ground_height_agen);

    AgenWrapper refl_wave0_wrapper;
    refl_wave0_wrapper.size = sizeof(uint16_t);
    refl_wave0_wrapper.n1 = TILE_WIDTH / vecw;
    refl_wave0_wrapper.s1 = vecw;
    refl_wave0_wrapper.n2 = TILE_HEIGHT;
    refl_wave0_wrapper.s2 = TILE_WIDTH;
    agen refl_wave0_agen = init((dvushort *)NULL);
    INIT_AGEN2(refl_wave0_agen, refl_wave0_wrapper);
    config->refl_wave0 = extract_agen_cfg(refl_wave0_agen);

    AgenWrapper refl_wave1_wrapper;
    refl_wave1_wrapper.size = sizeof(uint16_t);
    refl_wave1_wrapper.n1 = TILE_WIDTH / vecw;
    refl_wave1_wrapper.s1 = vecw;
    refl_wave1_wrapper.n2 = TILE_HEIGHT;
    refl_wave1_wrapper.s2 = TILE_WIDTH;
    agen refl_wave1_agen = init((dvushort *)NULL);
    INIT_AGEN2(refl_wave1_agen, refl_wave1_wrapper);
    config->refl_wave1 = extract_agen_cfg(refl_wave1_agen);

    AgenWrapper grnd_wave0_wrapper;
    grnd_wave0_wrapper.size = sizeof(uint16_t);
    grnd_wave0_wrapper.n1 = TILE_WIDTH / vecw;
    grnd_wave0_wrapper.s1 = vecw;
    grnd_wave0_wrapper.n2 = TILE_HEIGHT;
    grnd_wave0_wrapper.s2 = TILE_WIDTH;
    agen grnd_wave0_agen = init((dvushort *)NULL);
    INIT_AGEN2(grnd_wave0_agen, grnd_wave0_wrapper);
    config->grnd_wave0 = extract_agen_cfg(grnd_wave0_agen);

    AgenWrapper grnd_wave1_wrapper;
    grnd_wave1_wrapper.size = sizeof(uint16_t);
    grnd_wave1_wrapper.n1 = TILE_WIDTH / vecw;
    grnd_wave1_wrapper.s1 = vecw;
    grnd_wave1_wrapper.n2 = TILE_HEIGHT;
    grnd_wave1_wrapper.s2 = TILE_WIDTH;
    agen grnd_wave1_agen = init((dvushort *)NULL);
    INIT_AGEN2(grnd_wave1_agen, grnd_wave1_wrapper);
    config->grnd_wave1 = extract_agen_cfg(grnd_wave1_agen);

    AgenWrapper high_wave0_wrapper;
    high_wave0_wrapper.size = sizeof(int16_t);
    high_wave0_wrapper.n1 = TILE_WIDTH / vecw;
    high_wave0_wrapper.s1 = vecw;
    high_wave0_wrapper.n2 = TILE_HEIGHT;
    high_wave0_wrapper.s2 = TILE_WIDTH;
    agen high_wave0_agen = init((dvshort *)NULL);
    INIT_AGEN2(high_wave0_agen, high_wave0_wrapper);
    config->high_wave0 = extract_agen_cfg(high_wave0_agen);

    AgenWrapper high_wave1_wrapper;
    high_wave1_wrapper.size = sizeof(int16_t);
    high_wave1_wrapper.n1 = TILE_WIDTH / vecw;
    high_wave1_wrapper.s1 = vecw;
    high_wave1_wrapper.n2 = TILE_HEIGHT;
    high_wave1_wrapper.s2 = TILE_WIDTH;
    agen high_wave1_agen = init((dvshort *)NULL);
    INIT_AGEN2(high_wave1_agen, high_wave1_wrapper);
    config->high_wave1 = extract_agen_cfg(high_wave1_agen);

    AgenWrapper stray_mask0_wrapper;
    stray_mask0_wrapper.size = sizeof(uint16_t);
    stray_mask0_wrapper.n1 = TILE_WIDTH / vecw;
    stray_mask0_wrapper.s1 = vecw;
    stray_mask0_wrapper.n2 = TILE_HEIGHT;
    stray_mask0_wrapper.s2 = TILE_WIDTH;
    agen stray_mask0_agen = init((dvushort *)NULL);
    INIT_AGEN2(stray_mask0_agen, stray_mask0_wrapper);
    config->stray_mask0 = extract_agen_cfg(stray_mask0_agen);

    AgenWrapper stray_mask1_wrapper;
    stray_mask1_wrapper.size = sizeof(uint16_t);
    stray_mask1_wrapper.n1 = TILE_WIDTH / vecw;
    stray_mask1_wrapper.s1 = vecw;
    stray_mask1_wrapper.n2 = TILE_HEIGHT;
    stray_mask1_wrapper.s2 = TILE_WIDTH;
    agen stray_mask1_agen = init((dvushort *)NULL);
    INIT_AGEN2(stray_mask1_agen, stray_mask1_wrapper);
    config->stray_mask1 = extract_agen_cfg(stray_mask1_agen);
}

/**
 * \brief Modify start address of agen
 *
 * \param[in]   config   : agen configuration
 * \param[out]  config   : agen configuration
*/
void agenConfigModify(StrayConfig_t *config)
{
    /*input*/
    cupvaModifyAgenCfgBase(&config->dist_wave0, &dist_wave[dist_offset]);
    cupvaModifyAgenCfgBase(&config->dist_wave1, &dist_wave[dist_offset + (TILE_WIDTH + 2) * 10 + 1]);
    cupvaModifyAgenCfgBase(&config->att0, &att0[att0_offset]);
    cupvaModifyAgenCfgBase(&config->att1, &att1[att1_offset]);
    cupvaModifyAgenCfgBase(&config->class_line, &class_line[class_offset]);
    cupvaModifyAgenCfgBase(&config->ground_height, &ground_height[ground_offset]);
    cupvaModifyAgenCfgBase(&config->refl_wave0, &raw_data[raw_offset]);
    cupvaModifyAgenCfgBase(&config->refl_wave1, &raw_data[raw_offset + TILE_WIDTH * 10]);
    cupvaModifyAgenCfgBase(&config->grnd_wave0, &raw_data[raw_offset + TILE_WIDTH * 20]);
    cupvaModifyAgenCfgBase(&config->grnd_wave1, &raw_data[raw_offset + TILE_WIDTH * 30]);
    cupvaModifyAgenCfgBase(&config->high_wave0, &raw_data[raw_offset + TILE_WIDTH * 40]);
    cupvaModifyAgenCfgBase(&config->high_wave1, &raw_data[raw_offset + TILE_WIDTH * 50]);

    /*output*/
    cupvaModifyAgenCfgBase(&config->stray_mask0, &stray_mask0[stray_mask0_offset]);
    cupvaModifyAgenCfgBase(&config->stray_mask1, &stray_mask1[stray_mask1_offset]);
}

/**
 * \brief Set halo value
*/
void setHaloValue()
{
    for (int i = 0; i < TILE_HEIGHT * 2; i ++) {
        dist_wave[dist_offset + i * (TILE_WIDTH + 2)] = dist_wave[dist_offset + i * (TILE_WIDTH + 2) + 1];
        dist_wave[dist_offset + i * (TILE_WIDTH + 2) + TILE_WIDTH + 1] = dist_wave[dist_offset + i * (TILE_WIDTH + 2) + TILE_WIDTH];
    }
}

/**
 * \brief Update double buffer offset
*/
void offsetUpdate()
{
    dist_offset = cupvaRasterDataFlowGetOffset(dist_wave_handler, dist_offset);
    att0_offset = cupvaRasterDataFlowGetOffset(att0_handler, att0_offset);
    att1_offset = cupvaRasterDataFlowGetOffset(att1_handler, att1_offset);
    class_offset = cupvaRasterDataFlowGetOffset(class_line_handler, class_offset);
    ground_offset = cupvaRasterDataFlowGetOffset(ground_height_handler, ground_offset);
    raw_offset = cupvaRasterDataFlowGetOffset(raw_data_handler, raw_offset);

    stray_mask0_offset = cupvaRasterDataFlowGetOffset(stray_mask0_handler, stray_mask0_offset);
    stray_mask1_offset = cupvaRasterDataFlowGetOffset(stray_mask1_handler, stray_mask1_offset);
}

/**
 * \brief Stray-remove algo pva function
 *
 * \param[in]  config   : agen configuration
 * \param[in]  tile_idx : tile id
*/
void strayProc(StrayConfig_t *config, int tile_idx)
{
    agen row_id_agen = init_agen_from_cfg(config->row_id);
    agen dist_wave0_agen = init_agen_from_cfg(config->dist_wave0);
    agen dist_wave1_agen = init_agen_from_cfg(config->dist_wave1);
    agen att0_agen = init_agen_from_cfg(config->att0);
    agen att1_agen = init_agen_from_cfg(config->att1);
    agen class_line_agen = init_agen_from_cfg(config->class_line);
    agen ground_height_agen = init_agen_from_cfg(config->ground_height);
    agen refl_wave0_agen = init_agen_from_cfg(config->refl_wave0);
    agen refl_wave1_agen = init_agen_from_cfg(config->refl_wave1);
    agen grnd_wave0_agen = init_agen_from_cfg(config->grnd_wave0);
    agen grnd_wave1_agen = init_agen_from_cfg(config->grnd_wave1);
    agen high_wave0_agen = init_agen_from_cfg(config->high_wave0);
    agen high_wave1_agen = init_agen_from_cfg(config->high_wave1);

    agen stray_mask0_agen = init_agen_from_cfg(config->stray_mask0);
    agen stray_mask1_agen = init_agen_from_cfg(config->stray_mask1);

    int32_t niter = config->niter;

    // 初始化0向量
    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);

    // 杂散阈值参数
    dvshortx stray_cnt_th, ref_del_th, ground_dist_diff, blind_dist_max, ground_seg;
    stray_cnt_th.lo = replicateh(5);
    stray_cnt_th.hi = replicateh(5);
    ref_del_th.lo = replicateh(6);
    ref_del_th.hi = replicateh(6);
    blind_dist_max.lo = replicateh(600);
    blind_dist_max.hi = replicateh(600);

    dvshortx rainwall_dist_vec;
    rainwall_dist_vec.lo = replicateh(*(int *)rainwall_dist);
    rainwall_dist_vec.hi = replicateh(*(int *)rainwall_dist);
    dvshortx rainwall_cnt_vec;
    rainwall_cnt_vec.lo = replicateh(*(int *)rainwall_cnt);
    rainwall_cnt_vec.hi = replicateh(*(int *)rainwall_cnt);

    dvshortx gnd_dist_max;
    dvshortx ceil_stray_dist0, ceil_stray_dist1, ceil_stray_dist2;
    dvshortx stray_chain_dist0, stray_chain_dist1, stray_chain_dist_c;
    dvshortx stray_chain_row_st0, stray_chain_row_ed0, stray_chain_row_st1, stray_chain_row_ed1;
    dvshortx stray_chain_row_st_c, stray_chain_row_ed_c;
    dvshortx stray_chain_cnt0, stray_chain_cnt1, stray_chain_cnt_c;
    dvshortx stray_chain_height2;

    for (int32_t i = 0; i < niter; i ++) chess_prepare_for_pipelining
    chess_unroll_loop(2) {
        int seg_id = i % 6;
        if (seg_id == 0) {
            int col_idx = i / 6;
            int offset = tile_idx * 10 * STRAY_VAR_CNT;
            gnd_dist_max.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT]);
            gnd_dist_max.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT]);

            ceil_stray_dist0.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 1]);
            ceil_stray_dist0.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 1]);

            ceil_stray_dist1.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 2]);
            ceil_stray_dist1.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 2]);

            ceil_stray_dist2.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 3]);
            ceil_stray_dist2.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 3]);

            stray_chain_dist0.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 4]);
            stray_chain_dist0.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 4]);

            stray_chain_dist1.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 5]);
            stray_chain_dist1.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 5]);

            stray_chain_dist_c.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 6]);
            stray_chain_dist_c.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 6]);

            stray_chain_row_st0.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 7]);
            stray_chain_row_st0.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 7]);

            stray_chain_row_ed0.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 8]);
            stray_chain_row_ed0.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 8]);

            stray_chain_row_st1.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 9]);
            stray_chain_row_st1.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 9]);

            stray_chain_row_ed1.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 10]);
            stray_chain_row_ed1.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 10]);

            stray_chain_row_st_c.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 11]);
            stray_chain_row_st_c.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 11]);

            stray_chain_row_ed_c.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 12]);
            stray_chain_row_ed_c.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 12]);

            stray_chain_cnt0.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 13]);
            stray_chain_cnt0.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 13]);

            stray_chain_cnt1.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 14]);
            stray_chain_cnt1.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 14]);

            stray_chain_cnt_c.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 15]);
            stray_chain_cnt_c.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 15]);

            stray_chain_height2.lo = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 16]);
            stray_chain_height2.hi = replicateh(stray_var[offset + col_idx * STRAY_VAR_CNT + 16]);
        }

        // 数据加载
        dvshortx dist_down = dvushort_load(dist_wave0_agen);
        dvshortx dist_cur = dvushort_load(dist_wave0_agen);
        dvshortx dist_up = dvushort_load(dist_wave0_agen);
        dvshortx dist_cur2 = dvushort_load(dist_wave1_agen);

        dvshortx ref_cur = dvushort_load(refl_wave0_agen);
        dvshortx ref_cur2 = dvushort_load(refl_wave1_agen);

        dvshortx att0_l = dvushort_load(att0_agen);
        dvshortx att0_cur = dvushort_load(att0_agen);
        dvshortx att0_r = dvushort_load(att0_agen);

        dvshortx att1_l = dvushort_load(att1_agen);
        dvshortx att1_cur = dvushort_load(att1_agen);
        dvshortx att1_r = dvushort_load(att1_agen);

        dvshortx peak_mark_l = ((att0_l >> 3) & 0x01);
        dvshortx peak_mark_cur = ((att0_cur >> 3) & 0x01);
        dvshortx peak_mark_r = ((att0_r >> 3) & 0x01);

        dvshortx crosstalk_cur = (att0_cur & 0x01);
        dvshortx crosstalk_cur2 = (att1_cur & 0x01);
        dvshortx smlr_cur2 = ((att1_cur >> 5) & 0x03);

        dvshortx gnd_cur = dvushort_load(grnd_wave0_agen);
        dvshortx gnd_cur2 = dvushort_load(grnd_wave1_agen);

        dvshortx height_cur = dvshort_load(high_wave0_agen);
        dvshortx height_cur2 = dvshort_load(high_wave1_agen);

        dvshortx class_down = dvushort_load(class_line_agen);
        dvshortx class_raw = dvushort_load(class_line_agen);
        dvshortx class_up = dvushort_load(class_line_agen);

        dvshortx ground_height = dvshort_load(ground_height_agen);

        ground_seg = dist_cur >> 9;
        ground_seg = dvmux(ground_seg >= 28, vec0 + 27, ground_seg);
        ground_dist_diff = dvmux(ground_seg >= 9, vec0 + 2000, vec0 + 1200);

        // 更新地面标记
        gnd_cur = dvmux(((dist_cur2 > dist_cur) && gnd_cur2 && gnd_cur) || (ground_height != 32767 && (height_cur - ground_height > 50)), vec0, gnd_cur);
        gnd_cur2 = dvmux(ground_height != 32767 && height_cur2 - ground_height > 50, 0, gnd_cur2);

        // 连接标志
        dvshortx connect_ground_flag =
                    (!gnd_cur) &&
                    ((height_cur - ground_height < 400 &&
                        dvabsdif(gnd_dist_max, dist_cur) <= ground_dist_diff) ||
                        ((dist_cur > gnd_dist_max) &&
                        (dist_cur - gnd_dist_max <= 1600) &&
                        (height_cur < 400)));

        dvshortx connect_ceil_flag =
                    (dvabsdif(dist_cur, ceil_stray_dist2) <= 400) ||
                    (dvabsdif(dist_cur, ceil_stray_dist1) <= 400) ||
                    (dvabsdif(dist_cur, ceil_stray_dist0) <= 400) ||
                    dvabsdif(dist_cur, rainwall_dist_vec) < 960
                    && (rainwall_dist_vec > 4000 || height_cur > 0)
                    && rainwall_cnt_vec > 80
                    || (stray_chain_cnt_c >= 40
                        && dvabsdif(dist_cur, stray_chain_dist_c) < 240
                        && stray_chain_cnt_c * 4 > (stray_chain_row_ed_c - stray_chain_row_st_c) * 3
                        && row_id[seg_id] >= stray_chain_row_st_c
                        && row_id[seg_id] <= stray_chain_row_ed_c
                        && stray_chain_height2 > 500);

        // 更新分类标记
        dvshortx class_cur = (class_raw == 1) || (class_down == 1) || (class_up == 1) ||
                             (dvabsdif(dist_up, dist_cur) > 400 &&
                              dvabsdif(dist_down, dist_cur) > 400);

        // 杂散和镜像标志
        dvshortx stray_flag = (peak_mark_cur || peak_mark_l || peak_mark_r);
        dvshortx mirror_flag = (row_id[seg_id] < 48) &&
                               (dist_cur > 5000) &&
                               (height_cur < -600);

        // 删除条件
        dvshortx straight_del_en = crosstalk_cur && class_cur;
        dvshortx column_del_en =
                (stray_chain_cnt_c >= stray_cnt_th &&
                    dvabsdif(dist_cur, stray_chain_dist_c) < 240 && class_cur) ||
                (stray_chain_cnt0 >= stray_cnt_th &&
                    dvabsdif(dist_cur, stray_chain_dist0) < 240 &&
                    ((row_id[seg_id] >= stray_chain_row_st0 && row_id[seg_id] <= stray_chain_row_ed0) || class_cur)) ||
                (stray_chain_cnt1 >= stray_cnt_th &&
                    dvabsdif(dist_cur, stray_chain_dist1) < 240 &&
                    ((row_id[seg_id] >= stray_chain_row_st1 && row_id[seg_id] <= stray_chain_row_ed1) || class_cur));

        dvshortx frame_del_en =
                (rainwall_cnt_vec > 80) &&
                (dvabsdif(dist_cur, rainwall_dist_vec) < 960) &&
                (height_cur > 0) &&
                class_cur;

        // 最终删除标记
        dvshortx stray_tag_strong_en = (stray_flag || mirror_flag) && (straight_del_en || column_del_en && !gnd_cur || frame_del_en && !gnd_cur); // 强杂散
        dvshortx stray_tag_weak_en = class_raw == 2 && ref_cur < ref_del_th && dvabsdif(dist_cur, rainwall_dist_vec) < 960; // 弱杂散
        dvshortx stray_del_en =  (stray_tag_strong_en || stray_tag_weak_en) && !(connect_ground_flag && !connect_ceil_flag); // 非接地不接天的强杂散进行删除，弱杂散在高反模块再次判别

        // 第二回波选择逻辑
        dvshortx wave1_sel_base = (smlr_cur2 >= 1) && (crosstalk_cur2 == 0);
        dvshortx wave1_sel_column =
                    (stray_chain_cnt_c < stray_cnt_th ||
                        dvabsdif(dist_cur2, stray_chain_dist_c) > 240) &&
                    (stray_chain_cnt0 < stray_cnt_th ||
                        dvabsdif(dist_cur2, stray_chain_dist0) > 240) &&
                    (stray_chain_cnt1 < stray_cnt_th ||
                        dvabsdif(dist_cur2, stray_chain_dist1) > 240);

        dvshortx wave1_sel_frame = ((rainwall_cnt_vec > 80) &&
                            (dist_cur2 > (rainwall_dist_vec + 200)) ||
                            (dist_cur2 < (rainwall_dist_vec - 960)))
                            || (rainwall_cnt_vec <= 80);

        dvshortx wave1_sel_en = (wave1_sel_base && wave1_sel_column && wave1_sel_frame) || gnd_cur2;

        dvshortx select_cond = stray_del_en && dist_cur > blind_dist_max;
        dvshortx mark0 = select_cond;
        dvshortx mark1 = (select_cond && (!wave1_sel_en)) || ((!select_cond) && (!wave1_sel_frame));

        vstore(mark0, stray_mask0_agen);
        vstore(mark1, stray_mask1_agen);
    }
}

/**
 * \brief Initialize row id array
 *
 * \param[in]  row_ids  : row id array
 * \param[out] row_ids  : row id array
*/
void initRowIds(uint16_t *row_ids)
{
    for (int i = 0; i < 192; i ++) {
        row_ids[i] = i;
    }
}

/**
 * \brief VPU Main function
*/
CUPVA_VPU_MAIN()
{
    StrayConfig_t config;
    uint16_t row_ids[192];
    initRowIds(row_ids);
    agenConfigInit(&config, row_ids);

    agen row_id_agen = init_agen_from_cfg(config.row_id);
    for (int i = 0; i < 6; i ++) {
        row_id[i] = dvushort_load(row_id_agen);
    }

    cupvaRasterDataFlowOpen(stray_var_handler, &stray_var[0]);
    uint16_t *stray_var_mem = (uint16_t *)cupvaRasterDataFlowAcquire(stray_var_handler);

    cupvaRasterDataFlowTrig(dist_wave_handler);
    cupvaRasterDataFlowTrig(att0_handler);
    cupvaRasterDataFlowTrig(att1_handler);
    cupvaRasterDataFlowTrig(class_line_handler);
    cupvaRasterDataFlowTrig(ground_height_handler);
    cupvaRasterDataFlowTrig(raw_data_handler);

    for (int tile_idx = 0; tile_idx < TILE_CNT; tile_idx ++) {
        cupvaRasterDataFlowSync(dist_wave_handler);
        cupvaRasterDataFlowTrig(dist_wave_handler);

        cupvaRasterDataFlowSync(att0_handler);
        cupvaRasterDataFlowTrig(att0_handler);

        cupvaRasterDataFlowSync(att1_handler);
        cupvaRasterDataFlowTrig(att1_handler);

        cupvaRasterDataFlowSync(class_line_handler);
        cupvaRasterDataFlowTrig(class_line_handler);

        cupvaRasterDataFlowSync(ground_height_handler);
        cupvaRasterDataFlowTrig(ground_height_handler);

        cupvaRasterDataFlowSync(raw_data_handler);
        cupvaRasterDataFlowTrig(raw_data_handler);

        agenConfigModify(&config);
        setHaloValue();
        offsetUpdate();

        strayProc(&config, tile_idx);

        cupvaRasterDataFlowSync(stray_mask0_handler);
        cupvaRasterDataFlowTrig(stray_mask0_handler);

        cupvaRasterDataFlowSync(stray_mask1_handler);
        cupvaRasterDataFlowTrig(stray_mask1_handler);
    }

    cupvaRasterDataFlowSync(stray_mask0_handler);
    cupvaRasterDataFlowSync(stray_mask1_handler);

    cupvaRasterDataFlowRelease(stray_var_handler);
    cupvaRasterDataFlowClose(stray_var_handler);

    return 0;
}
/** [release_and_close] */

