/*******************************************************************************
 * \addtogroup trail_programm
 * \{
 * \file trail_top.c
 * \brief
 * \version 0.4
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
 * | 0.2 | 2025-09-29 | Trail vector version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.3 | 2025-11-13 | Optimize processing time and load |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.4 | 2025-11-28 | Add draw judgment function |
 ******************************************************************************/
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_device.h>
#include <cupva_device_debug.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../trail_common_param.h"

/** Use double buffer, the vertical halo is 6, the horizontal halo is 4*/
VMEM(A, uint16_t, inputDistBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
/** Output do not use halo */
VMEM(C, uint16_t, outputValidBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
/** declare algorithm params */
VMEM(C, int, algorithmParams, sizeof(TrailParam_t));
/** declare dataflow handles */
VMEM(B, RasterDataFlowHandler, sourceDistDataFlowHandler);
VMEM(B, RasterDataFlowHandler, destinationDataFlowHandler);
/** declare the local variable buffer */
VMEM(A, uint16_t, maskBuffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(B, uint16_t, nearDistThBuffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(C, uint16_t, adjDisThreDBuffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(B, uint16_t, con1_3buffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(C, uint16_t, con3_1buffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(B, uint16_t, refer_buffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(B, uint16_t, hortrail_judge_buffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(A, uint16_t, wall_judge_buffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(B, uint16_t, nearCntVBuffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(C, uint16_t, verCntBuffer, TILE_WIDTH * TILE_HEIGHT);
VMEM(A, uint16_t, draw_judge_Buffer, TILE_WIDTH * TILE_HEIGHT);

typedef struct
{
    AgenCFG input_dist;
    AgenCFG output_mask;
    AgenCFG near_dist_th;
    AgenCFG adj_dis_thre_d;
    int32_t niter;
} MaskConfig_t;

typedef struct
{
    AgenCFG input_dist;
    AgenCFG adj_dis_thre_d;
    AgenCFG near_dist_th;
    AgenCFG con1_3;
    AgenCFG con3_1;
    AgenCFG refer_mask;
    int32_t niter;
} Part1Config_t;
typedef struct
{
    AgenCFG input_dist;
    AgenCFG draw_mask;
    int32_t niter;
} DrawConfig_t;

typedef struct
{
    AgenCFG input_dist;
    AgenCFG adj_dis_thre_d;
    AgenCFG con1_3;
    AgenCFG hortrail_judge;
    int32_t niter;
} Part2Config_t;

typedef struct
{
    AgenCFG input_dist;
    AgenCFG wall_judge;
    int32_t niter;
} Part3Config_t;

typedef struct
{
    AgenCFG input_dist;
    AgenCFG center;
    AgenCFG near_dist_th;
    AgenCFG near_cnt_v;
    AgenCFG ver_cnt;
    int32_t niter1;
    int32_t niter;
} Part4Config_t;

typedef struct
{
    AgenCFG center_dist;
    AgenCFG near_cnt_v;
    AgenCFG ver_cnt;
    AgenCFG wall_judge;
    AgenCFG mask;
    AgenCFG Hortrail_judge;
    AgenCFG con3_1;
    AgenCFG refer_mask;
    AgenCFG draw_judge;
    AgenCFG output;
    int32_t niter;
} Part5Config_t;

VMEM(B, MaskConfig_t, maskParam);
VMEM(B, Part1Config_t, Part1Param);
VMEM(B, DrawConfig_t, DrawParam);
VMEM(B, Part2Config_t, Part2Param);
VMEM(B, Part3Config_t, Part3Param);
VMEM(B, Part4Config_t, Part4Param);
VMEM(B, Part5Config_t, Part5Param);

dvshortx dif_distC_abs[5];
dvshortx zero_flag[5];
dvshortx dist_tmp[5];
dvshortx vec0;

/**
 * \brief  Center check initialization function
 * \param[in] config : Center check configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] input_line_pitch : Line pitch without halo
 *                Range: 192. Accuracy: NA.
 * \param[out] mask : Center point valid mark
 *                Range: 0-1. Accuracy: 1.
 * \param[out] near_dist_th : Near distance threshold
 *                Range: 0-2^16-1. Accuracy: 1.
 * \param[out] adj_dis_thre_d : Distance segment threshold index
 *                Range: 0-2^16-1. Accuracy: 1.
 */
void check_center_init(uint16_t *mask, uint16_t *near_dist_th, uint16_t *adj_dis_thre_d, int32_t input_line_pitch,
                       MaskConfig_t *config)
{
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = TILE_WIDTH / VECW;
    input_wrapper.s1   = VECW;
    input_wrapper.n2   = TILE_HEIGHT;
    input_wrapper.s2   = input_line_pitch;
    agen input_agen    = init((dvushort *)NULL);
    INIT_AGEN2(input_agen, input_wrapper);

    AgenWrapper output_wrapper;
    output_wrapper.size = sizeof(uint16_t);
    output_wrapper.n1   = TILE_WIDTH / VECW;
    output_wrapper.s1   = VECW;
    output_wrapper.n2   = TILE_HEIGHT;
    output_wrapper.s2   = TILE_WIDTH;
    agen mask_agen = init((dvushort *)mask);
    INIT_AGEN2(mask_agen, output_wrapper);

    agen near_dist_th_agen = init((dvushort *)near_dist_th);
    INIT_AGEN2(near_dist_th_agen, output_wrapper);
    near_dist_th_agen.sat_opt    = 3; // unsigned saturation
    near_dist_th_agen.sat_lim_lo = 2;
    near_dist_th_agen.sat_val_lo = 2;
    near_dist_th_agen.sat_lim_hi = 20;
    near_dist_th_agen.sat_val_hi = 20;

    agen adj_dis_thre_d_agen = init((dvushort *)adj_dis_thre_d);
    INIT_AGEN2(adj_dis_thre_d_agen, output_wrapper);
    adj_dis_thre_d_agen.sat_opt    = 3; // unsigned saturation
    adj_dis_thre_d_agen.sat_lim_lo = 1;
    adj_dis_thre_d_agen.sat_val_lo = 1;
    adj_dis_thre_d_agen.sat_lim_hi = UINT16_MAX;
    adj_dis_thre_d_agen.sat_val_hi = UINT16_MAX;

    config->input_dist     = extract_agen_cfg(input_agen);
    config->output_mask    = extract_agen_cfg(mask_agen);
    config->near_dist_th   = extract_agen_cfg(near_dist_th_agen);
    config->adj_dis_thre_d = extract_agen_cfg(adj_dis_thre_d_agen);
    config->niter          = (TILE_WIDTH / VECW) * TILE_HEIGHT;
}

/**
 * \brief  Center check execute function
 * \param[in] config : Center check configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] trail_Param : Trail parameters structure
 *                Range: NA. Accuracy: NA.
 */
void check_center_exec(TrailParam_t *trail_Param, MaskConfig_t *config)
{
    agen_A input_agen          = init_agen_A_from_cfg(config->input_dist);
    agen_A mask_agen           = init_agen_A_from_cfg(config->output_mask);
    agen_B near_dist_th_agen   = init_agen_B_from_cfg(config->near_dist_th);
    agen_C adj_dis_thre_d_agen = init_agen_C_from_cfg(config->adj_dis_thre_d);
    int32_t niter              = config->niter;

    const int32_t bypass_distance = trail_Param->BypassDis;
    const int32_t DisThreRatio    = trail_Param->DisThreRatio;

    int32_t dist_th_ratio = (trail_Param->dist_th_ratio) << 4;

    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(9, )
    chess_unroll_loop(3)
    {
        dvshortx dist = dvushort_load(input_agen);

        dvshortx pred_con1 = dist > 0;
        dvshortx pred_con2 = dist < bypass_distance;
        dvshortx pred      = pred_con1 & pred_con2;

        dvshortx near_dist_th = dvmulh(dist, dist_th_ratio, VPU_TRUNC_7);

        dvshortx AdjDisThreD = dvmulh(dist, DisThreRatio, VPU_ROUND_0);
        AdjDisThreD          = AdjDisThreD >> 12;

        vstore(pred, mask_agen);
        vstore(AdjDisThreD, adj_dis_thre_d_agen);
        vstore(near_dist_th, near_dist_th_agen);
    }
}

/**
 * \brief Part1 initialization function
 * \param[in] config : Part1 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] input_line_pitch : Line pitch with halo
 *                Range: 194. Accuracy: NA.
 * \param[in] adj_dis_thre_d : Distance segment threshold index
 *                Range: 0-2^16-1. Accuracy: 1.
 * \param[in] near_dist_th : Near distance threshold
 *                Range: 0-2^16-1. Accuracy: 1.
 * \param[out] refer_mask : Refer mask
 *                Range: 0-1. Accuracy: 1.
 * \param[out] con1_3 : Condition 1_3
 *                Range: 0-1. Accuracy: 1.
 * \param[out] con3_1 : Condition 3_1
 *                Range: 0-1. Accuracy: 1.
 */
void trail_cond_part1_init(uint16_t *near_dist_th, uint16_t *adj_dis_thre_d, uint16_t *con1_3, uint16_t *con3_1,
                            uint16_t *refer_mask, int32_t input_line_pitch, Part1Config_t *config)
{
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = 5;
    input_wrapper.s1   = input_line_pitch; // jump to next row
    input_wrapper.n2   = TILE_WIDTH / VECW;
    input_wrapper.s2   = VECW; // horizontal direction
    input_wrapper.n3   = TILE_HEIGHT;
    input_wrapper.s3   = input_line_pitch;       // vertical direction
    agen input_agen    = init((dvushort *)NULL); // need to modify base for every tile
    INIT_AGEN3(input_agen, input_wrapper);

    AgenWrapper agen_wrapper;
    agen_wrapper.size = sizeof(uint16_t);
    agen_wrapper.n1   = TILE_WIDTH / VECW;
    agen_wrapper.s1   = VECW;
    agen_wrapper.n2   = TILE_HEIGHT;
    agen_wrapper.s2   = TILE_WIDTH;

    agen adj_dis_thre_d_agen = init((dvushort *)adj_dis_thre_d);
    INIT_AGEN2(adj_dis_thre_d_agen, agen_wrapper);
    agen near_dist_th_agen = init((dvushort *)near_dist_th);
    INIT_AGEN2(near_dist_th_agen, agen_wrapper);
    agen con1_3_agen = init((dvushort *)con1_3);
    INIT_AGEN2(con1_3_agen, agen_wrapper);
    agen con3_1_agen = init((dvushort *)con3_1);
    INIT_AGEN2(con3_1_agen, agen_wrapper);
    agen refer_mask_agen = init((dvushort *)refer_mask);
    INIT_AGEN2(refer_mask_agen, agen_wrapper);

    config->input_dist     = extract_agen_cfg(input_agen);
    config->adj_dis_thre_d = extract_agen_cfg(adj_dis_thre_d_agen);
    config->near_dist_th   = extract_agen_cfg(near_dist_th_agen);
    config->con1_3         = extract_agen_cfg(con1_3_agen);
    config->con3_1         = extract_agen_cfg(con3_1_agen);
    config->refer_mask     = extract_agen_cfg(refer_mask_agen);
    config->niter          = (TILE_WIDTH / VECW) * TILE_HEIGHT;
}

/**
 * \brief  Part1 execute function
 * \param[in] config : Part1 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] trail_Param : Trail parameters structure
 *                Range: NA. Accuracy: NA.
 */
void trail_cond_part1_exec(TrailParam_t *trail_Param, Part1Config_t *config)
{
    agen input_agen          = init_agen_from_cfg(config->input_dist);
    agen adj_dis_thre_d_agen = init_agen_from_cfg(config->adj_dis_thre_d);
    agen near_dist_th_agen   = init_agen_from_cfg(config->near_dist_th);
    agen con1_3_agen         = init_agen_from_cfg(config->con1_3);
    agen con3_1_agen         = init_agen_from_cfg(config->con3_1);
    agen refer_mask_agen         = init_agen_from_cfg(config->refer_mask);

    const int32_t near_cnt_th_h = trail_Param->near_cnt_th_h;
    const int near_threshold = trail_Param->near_cnt_th_h;

    for (int32_t i = 0; i < config->niter; i++) chess_prepare_for_pipelining
    chess_loop_range(3, )
    {
        dvshortx dist0 = dvushort_load(input_agen); // col_idx - 2
        dvshortx dist1 = dvushort_load(input_agen); // col_idx - 1
        dvshortx dist2 = dvushort_load(input_agen); // center
        dvshortx dist3 = dvushort_load(input_agen); // col_idx + 1
        dvshortx dist4 = dvushort_load(input_agen); // col_idx + 2

        dvshortx AdjDisThreD  = dvushort_load(adj_dis_thre_d_agen);
        dvshortx near_dist_th = dvushort_load(near_dist_th_agen);

        dvshortx dif_dist_abs0 = dvabsdif(dist1, dist0);
        dvshortx dif_dist_abs1 = dvabsdif(dist2, dist1);
        dvshortx dif_dist_abs2 = dvabsdif(dist3, dist2);
        dvshortx dif_dist_abs3 = dvabsdif(dist4, dist3);

        dvshortx near_cnt_h = dif_dist_abs0 < near_dist_th;
        near_cnt_h += dif_dist_abs1 < near_dist_th;
        near_cnt_h += dif_dist_abs2 < near_dist_th;
        near_cnt_h += dif_dist_abs3 < near_dist_th;

        dvshortx dif_dist_abs_cnt = dif_dist_abs0 > AdjDisThreD;
        dif_dist_abs_cnt += dif_dist_abs1 > AdjDisThreD;
        dif_dist_abs_cnt += dif_dist_abs2 > AdjDisThreD;
        dif_dist_abs_cnt += dif_dist_abs3 > AdjDisThreD;

        dvshortx refer_mask = near_cnt_h >= near_threshold;
        dvshortx con1_3 = dif_dist_abs_cnt > 3;
        dvshortx con3_1 = near_cnt_h < near_cnt_th_h;

        vstore(refer_mask, refer_mask_agen);
        vstore(con1_3, con1_3_agen);
        vstore(con3_1, con3_1_agen);
    }
}

/**
 * \brief  Draw judgment initialization function
 * \param[in] config : Draw judgment configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] input_line_pitch : Line pitch with halo
 *                Range: 194. Accuracy: NA.
 * \param[out] draw_judge : Draw judgment
 *                Range: 0-1. Accuracy: 1.
 */
void trail_cond_draw_init(uint16_t *draw_judge, int32_t input_line_pitch, DrawConfig_t *config)
{
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = 13;
    input_wrapper.s1   = input_line_pitch; // jump to next row
    input_wrapper.n2   = TILE_WIDTH / VECW;
    input_wrapper.s2   = VECW; // horizontal direction
    input_wrapper.n3   = TILE_HEIGHT;
    input_wrapper.s3   = input_line_pitch;       // vertical direction
    agen input_agen    = init((dvushort *)NULL); // need to modify base for every tile
    INIT_AGEN3(input_agen, input_wrapper);

    AgenWrapper agen_wrapper;
    agen_wrapper.size = sizeof(uint16_t);
    agen_wrapper.n1   = TILE_WIDTH / VECW;
    agen_wrapper.s1   = VECW;
    agen_wrapper.n2   = TILE_HEIGHT;
    agen_wrapper.s2   = TILE_WIDTH;

    agen draw_judge_agen = init((dvushort *)draw_judge);
    INIT_AGEN2(draw_judge_agen, agen_wrapper);

    config->input_dist = extract_agen_cfg(input_agen);
    config->draw_mask  = extract_agen_cfg(draw_judge_agen);
    config->niter      = (TILE_WIDTH / VECW) * TILE_HEIGHT;
}

/**
 * \brief  Draw judgment execute function
 * \param[in] config : Draw judgment configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] trail_Param : Trail parameters structure
 *                Range: NA. Accuracy: NA.
 */
void trail_cond_draw_exec(TrailParam_t *trail_Param, DrawConfig_t *config)
{
    agen input_agen      = init_agen_from_cfg(config->input_dist);
    agen draw_judge_agen = init_agen_from_cfg(config->draw_mask);
    int32_t niter = config->niter;
    dvshortx dist[13];
    dvshortx pred[13];
    dvshortx dif_distC_abs[13];
    dvshortx dif_dist[13];
    dvshortx corner_mask[13];
    dvshortx cono_valid[13];

    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(3, )
    {
        dvshortx left_cnt = chess_dont_care(dvshortx) & 0;
        dvshortx right_cnt = chess_dont_care(dvshortx) & 0;
        dvshortx corner_cnt = chess_dont_care(dvshortx) & 0;
        for(int idx = 0; idx < 2 * KERNEL_RADIUS_HEIGHT + 1; idx++)
        {
            dist[idx] = dvushort_load(input_agen);
            pred[idx] = (dist[idx] == 0);
            dif_dist[idx] = chess_dont_care(dvshortx) & 0;
            corner_mask[idx] = chess_dont_care(dvshortx) & 0;
            cono_valid[idx] = chess_dont_care(dvshortx) & 0;
        }
        for(int idx = 0; idx < 2 * KERNEL_RADIUS_HEIGHT + 1; idx++)
        {
            dif_distC_abs[idx] = dvmux(pred[idx], 65535, dvabsdif(dist[idx], dist[6]));

            if(idx > 0)
            {
                dif_dist[idx] = dist[idx] - dist[idx - 1];
            }
            if(idx <= 3)
            {
                left_cnt += dvmux((~pred[idx]) & (dif_distC_abs[idx] > trail_Param->Draw_D_H), 1, vec0);
            }
            if(idx >= 9)
            {
                right_cnt += dvmux((~pred[idx]) & (dif_distC_abs[idx] > trail_Param->Draw_D_H), 1, vec0);
            }
            cono_valid[idx] = (dif_distC_abs[idx] < 100) & (dif_dist[idx] > 0);
        }

        dvshortx left_valid = vec0;
        dvshortx right_valid = vec0;
        dvshortx res_mask1 = vec0;
        dvshortx res_mask2 = vec0;
        dvshortx case1 = vec0;
        dvshortx case2 = vec0;
        dvshortx con_mask_sum = vec0;
        dvshortx result1 = vec0;
        dvshortx result2 = vec0;

        for(int cnt = 1; cnt < 2 * KERNEL_RADIUS_HEIGHT; cnt++)
        {
            left_valid = vec0;
            right_valid = vec0;
            case1 = vec0;
            case2 = vec0;
            corner_mask[cnt] = (dif_distC_abs[cnt] >= dif_distC_abs[cnt - 1]) &
                                (dif_distC_abs[cnt] >= dif_distC_abs[cnt + 1]) &
                                (~pred[cnt]) & (dif_distC_abs[cnt] < 100);
            for(int j = 1; j <= cnt; j++)
            {
                left_valid += cono_valid[j];
            }
            for(int k = cnt; k < 2 * KERNEL_RADIUS_HEIGHT; k++)
            {
                right_valid += cono_valid[k];
            }
            if(cnt < 6)
            {
                case1 = right_cnt > 0;
            }
            if(cnt > 6)
            {
                case2 = left_cnt > 0;
            }

            con_mask_sum += cono_valid[cnt];
            res_mask1 += ((case1 | case2) & corner_mask[cnt]) > 0;
            res_mask2 += (case1 | case2) & corner_mask[cnt] & (left_valid > 1) & (right_valid < 3);
        }

        result1 = dvmux(res_mask1 > 0, con_mask_sum, vec0);
        result2 = dvmux(res_mask2 > 0, con_mask_sum, vec0);

        dvshortx cond1_1 = left_cnt > 0 & result1 >= trail_Param->draw_near_cnt_th_h;
        dvshortx cond1_2 = right_cnt > 0 & result1 >= trail_Param->draw_near_cnt_th_h;
        dvshortx cond1_3 = result2 > 0;

        dvshortx cond = (cond1_1 | cond1_2) & cond1_3;
        vstore(cond, draw_judge_agen);
    }
}

/**
 * \brief  Part2 initialization function
 * \param[in] config : Part2 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] input_line_pitch : Line pitch with halo
 *                Range: 194. Accuracy: NA.
 * \param[in] con1_3 : Condition 1_3
 *                Range: 0-1. Accuracy: 1.
 * \param[in] adj_dis_thre_d : Distance segment threshold index
 *                Range: 0-2^16-1. Accuracy: 1.
 * \param[out] hortrail_judge : Horizontal direction trail judgment
 *                Range: 0-1. Accuracy: 1.
 */
void trail_cond_part2_init(uint16_t *adj_dis_thre_d, uint16_t *con1_3, uint16_t *hortrail_judge,
                                 int32_t input_line_pitch, Part2Config_t *config)
{
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = 5;
    input_wrapper.s1   = input_line_pitch; // jump to next row
    input_wrapper.n2   = TILE_WIDTH / VECW;
    input_wrapper.s2   = VECW; // horizontal direction
    input_wrapper.n3   = TILE_HEIGHT;
    input_wrapper.s3   = input_line_pitch;       // vertical direction
    agen input_agen    = init((dvushort *)NULL); // need to modify base for every tile
    INIT_AGEN3(input_agen, input_wrapper);

    AgenWrapper agen_wrapper;
    agen_wrapper.size = sizeof(uint16_t);
    agen_wrapper.n1   = TILE_WIDTH / VECW;
    agen_wrapper.s1   = VECW;
    agen_wrapper.n2   = TILE_HEIGHT;
    agen_wrapper.s2   = TILE_WIDTH;

    agen adj_dis_thre_d_agen = init((dvushort *)adj_dis_thre_d);
    INIT_AGEN2(adj_dis_thre_d_agen, agen_wrapper);

    agen con1_3_agen = init((dvushort *)con1_3);
    INIT_AGEN2(con1_3_agen, agen_wrapper);

    agen hortrail_judge_agen = init((dvushort *)hortrail_judge);
    INIT_AGEN2(hortrail_judge_agen, agen_wrapper);

    config->input_dist     = extract_agen_cfg(input_agen);
    config->adj_dis_thre_d = extract_agen_cfg(adj_dis_thre_d_agen);
    config->con1_3         = extract_agen_cfg(con1_3_agen);
    config->hortrail_judge = extract_agen_cfg(hortrail_judge_agen);
    config->niter          = (TILE_WIDTH / VECW) * TILE_HEIGHT;
}

/**
 * \brief  Part2 execute function
 * \param[in] config : Part2 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] trail_Param : Trail parameters structure
 *                Range: NA. Accuracy: NA.
 */
void trail_cond_part2_exec(TrailParam_t *trail_Param, Part2Config_t *config)
{
    agen input_agen          = init_agen_from_cfg(config->input_dist);
    agen adj_dis_thre_d_agen = init_agen_from_cfg(config->adj_dis_thre_d);
    agen con1_3_agen         = init_agen_from_cfg(config->con1_3);
    agen hortrail_judge_agen = init_agen_from_cfg(config->hortrail_judge);

    int32_t niter = config->niter;
    const int32_t D_H = trail_Param->D_H;

    dvshortx v1, v2;
    v1.lo = replicateh(1);
    v1.hi = replicateh(1);
    v2.lo = replicateh(2);
    v2.hi = replicateh(2);

    dvshortx weighted_sum = chess_dont_care(dvshortx) & 0;

    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(3, )
    {
        dvshortx dist0 = dvushort_load(input_agen); // col_idx - 2
        dvshortx dist1 = dvushort_load(input_agen); // col_idx - 1
        dvshortx dist2 = dvushort_load(input_agen); // center
        dvshortx dist3 = dvushort_load(input_agen); // col_idx + 1
        dvshortx dist4 = dvushort_load(input_agen); // col_idx + 2

        // if dist2 is 0, it is invalid, it is handled in check_center_exec
        dvshortx pred0 = (dist0 == 0);
        dvshortx pred1 = (dist1 == 0);
        dvshortx pred2 = (dist2 == 0);
        dvshortx pred3 = (dist3 == 0);
        dvshortx pred4 = (dist4 == 0);

        dvshortx zero_cnt = pred0 + pred1 + pred2 + pred3 + pred4;
        dvshortx con0     = zero_cnt <= 3;

        dvshortx dif_distC_abs0 = dvmux(pred0, 65535, dvabsdif(dist0, dist2));
        dvshortx dif_distC_abs1 = dvmux(pred1, 65535, dvabsdif(dist1, dist2));
        dvshortx dif_distC_abs3 = dvmux(pred3, 65535, dvabsdif(dist3, dist2));
        dvshortx dif_distC_abs4 = dvmux(pred4, 65535, dvabsdif(dist4, dist2));

        dvshortx weight0 = dvmux(pred0, 0, v1);
        dvshortx weight1 = dvmux(pred1, 0, v2);
        dvshortx weight3 = dvmux(pred3, 0, v2);
        dvshortx weight4 = dvmux(pred4, 0, v1);
        weighted_sum     = dvmaddh(dif_distC_abs0, weight0, weighted_sum, VPU_ROUND_0, 0);          // valid 24bits
        weighted_sum     = dvmaddh(dif_distC_abs1, weight1, weighted_sum, VPU_ROUND_0, 0xFFFFFFFF); // valid 24bits
        weighted_sum     = dvmaddh(dif_distC_abs3, weight3, weighted_sum, VPU_ROUND_0, 0xFFFFFFFF); // valid 24bits
        weighted_sum     = dvmaddh(dif_distC_abs4, weight4, weighted_sum, VPU_ROUND_0, 0xFFFFFFFF); // valid 24bits

        dvshortx distdiff_mean = weighted_sum >> 2;

        dvshortx AdjDisThreD = dvushort_load(adj_dis_thre_d_agen);
        dvshortx con1_3      = dvushort_load(con1_3_agen);

        dvshortx con2   = (distdiff_mean > AdjDisThreD) & (distdiff_mean < D_H);
        dvshortx con1_1 = (dif_distC_abs1 > AdjDisThreD) & (dif_distC_abs1 < D_H);
        dvshortx con1_2 = (dif_distC_abs3 > AdjDisThreD) & (dif_distC_abs3 < D_H);

        dvshortx con1           = (con1_1 & con1_2) | con1_3;
        dvshortx Hortrail_judge = con0 & (con1 | con2);

        vstore(Hortrail_judge, hortrail_judge_agen);
    }
}

/**
 * \brief  Part3 initialization function
 * \param[in] config : Part3 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] input_line_pitch : Line pitch with halo
 *                Range: 194. Accuracy: NA.
 * \param[out] wall_judge : Wall judgment
 *                Range: 0-1. Accuracy: 1.
 */
void trail_cond_part3_init(uint16_t *wall_judge, int32_t input_line_pitch, Part3Config_t *config)
{
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = 5;
    input_wrapper.s1   = input_line_pitch; // jump to next row
    input_wrapper.n2   = TILE_WIDTH / VECW;
    input_wrapper.s2   = VECW; // horizontal direction
    input_wrapper.n3   = TILE_HEIGHT;
    input_wrapper.s3   = input_line_pitch;       // vertical direction
    agen input_agen    = init((dvushort *)NULL); // need to modify base for every tile
    INIT_AGEN3(input_agen, input_wrapper);

    AgenWrapper output_wrapper;
    output_wrapper.size  = sizeof(uint16_t);
    output_wrapper.n1    = TILE_WIDTH / VECW;
    output_wrapper.s1    = VECW;
    output_wrapper.n2    = TILE_HEIGHT;
    output_wrapper.s2    = TILE_WIDTH;
    agen wall_judge_agen = init((dvushort *)wall_judge);
    INIT_AGEN2(wall_judge_agen, output_wrapper);

    config->input_dist      = extract_agen_cfg(input_agen);
    config->wall_judge = extract_agen_cfg(wall_judge_agen);
    config->niter      = (TILE_WIDTH / VECW) * TILE_HEIGHT;
}

/**
 * \brief  Part3 execute function
 * \param[in] config : Part3 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] trail_Param : Trail parameters structure
 *                Range: NA. Accuracy: NA.
 */
void trail_cond_part3_exec(TrailParam_t *trail_Param, Part3Config_t *config)
{
    agen input_agen      = init_agen_from_cfg(config->input_dist);
    agen wall_judge_agen = init_agen_from_cfg(config->wall_judge);

    int32_t niter = config->niter;

    const int32_t SlopDifThre = trail_Param->SlopDifThre;

    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(6, )
    chess_unroll_loop(2)
    {
        dvshortx dist0 = dvushort_load(input_agen); // col_idx - 2
        dvshortx dist1 = dvushort_load(input_agen); // col_idx - 1
        dvshortx dist2 = dvushort_load(input_agen); // center
        dvshortx dist3 = dvushort_load(input_agen); // col_idx + 1
        dvshortx dist4 = dvushort_load(input_agen); // col_idx + 2

        dvshortx dif_dist0 = dist1 - dist0;
        dvshortx dif_dist1 = dist2 - dist1;
        dvshortx dif_dist2 = dist3 - dist2;
        dvshortx dif_dist3 = dist4 - dist3;

        dvshortx dif2_dist_abs0 = dvabsdif(dif_dist1, dif_dist0);
        dvshortx dif2_dist_abs1 = dvabsdif(dif_dist2, dif_dist1);
        dvshortx dif2_dist_abs2 = dvabsdif(dif_dist3, dif_dist2);

        dvshortx sum_dif2_dist_abs = dif2_dist_abs0 + dif2_dist_abs1 + dif2_dist_abs2;

        dvshortx wall_mask1 = sum_dif2_dist_abs > SlopDifThre;
        dvshortx wall_mask2 = dif2_dist_abs0 > SlopDifThre;
        dvshortx wall_judge = wall_mask1 & wall_mask2;

        vstore(wall_judge, wall_judge_agen);
    }
}

/**
 * \brief  Part4 initialization function
 * \param[in] config : Part4 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] input_line_pitch : Line pitch without halo
 *                Range: 192. Accuracy: NA.
 * \param[in] near_dist_th : Near distance threshold
 *                Range: 0-2^16-1. Accuracy: 1.
 * \param[out] near_cnt_v : Vertical distance to nearest point count
 *                Range: 0-9. Accuracy: 1.
 * \param[out] ver_cnt : Number of neighborhood points satisfying the absolute
 *                      distance difference condition relative to the center point
 *                Range: 0-9. Accuracy: 1.
 */
void trail_cond_part4_init(uint16_t *near_dist_th, uint16_t *near_cnt_v, uint16_t *ver_cnt,
                                 int32_t input_line_pitch, Part4Config_t *config)
{
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);
    input_wrapper.n1   = 9;
    input_wrapper.s1   = 1; // next pixel
    input_wrapper.n2   = TILE_WIDTH / VECW;
    input_wrapper.s2   = VECW; // horizontal direction
    input_wrapper.n3   = TILE_HEIGHT;
    input_wrapper.s3   = input_line_pitch; // vertical direction
    agen input_agen    = init((dvushort *)NULL);
    INIT_AGEN3(input_agen, input_wrapper);

    AgenWrapper center_wrapper;
    center_wrapper.size = sizeof(uint16_t);
    center_wrapper.n1   = 9;
    center_wrapper.s1   = 0;
    center_wrapper.n2   = TILE_WIDTH / VECW;
    center_wrapper.s2   = VECW; // horizontal direction
    center_wrapper.n3   = TILE_HEIGHT;
    center_wrapper.s3   = input_line_pitch; // vertical direction
    agen center_agen    = init((dvushort *)NULL);
    INIT_AGEN3(center_agen, center_wrapper);

    AgenWrapper agen_wrapper;
    agen_wrapper.size = sizeof(uint16_t);
    agen_wrapper.n1   = 9;
    agen_wrapper.s1   = 0;
    agen_wrapper.n2   = TILE_WIDTH / VECW;
    agen_wrapper.s2   = VECW;
    agen_wrapper.n3   = TILE_HEIGHT;
    agen_wrapper.s3   = TILE_WIDTH;

    agen near_dist_th_agen = init((dvushort *)near_dist_th);
    INIT_AGEN3(near_dist_th_agen, agen_wrapper);
    agen near_cnt_v_agen = init((dvushort *)near_cnt_v);
    INIT_AGEN3(near_cnt_v_agen, agen_wrapper);
    agen ver_cnt_agen = init((dvushort *)ver_cnt);
    INIT_AGEN3(ver_cnt_agen, agen_wrapper);

    config->input_dist   = extract_agen_cfg(input_agen);
    config->center       = extract_agen_cfg(center_agen);
    config->near_dist_th = extract_agen_cfg(near_dist_th_agen);
    config->near_cnt_v   = extract_agen_cfg(near_cnt_v_agen);
    config->ver_cnt      = extract_agen_cfg(ver_cnt_agen);
    config->niter        = 9 * (TILE_WIDTH / VECW) * TILE_HEIGHT;
    config->niter1       = 9;
}

/**
 * \brief  Part4 execute function
 * \param[in] config : Part4 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] trail_Param : Trail parameters structure
 *                Range: NA. Accuracy: NA.
 */
void trail_cond_part4_exec(TrailParam_t *trail_Param, Part4Config_t *config)
{
    agen input_agen        = init_agen_from_cfg(config->input_dist);
    agen center_agen       = init_agen_from_cfg(config->center);
    agen near_dist_th_agen = init_agen_from_cfg(config->near_dist_th);
    agen near_cnt_v_agen   = init_agen_from_cfg(config->near_cnt_v);
    agen ver_cnt_agen      = init_agen_from_cfg(config->ver_cnt);

    int32_t niter1 = config->niter1;
    int32_t niter  = config->niter;

    dvshortx vmax;
    vmax.lo             = replicateh(65536);
    vmax.hi             = replicateh(65536);
    dvshortx near_cnt_v = chess_dont_care(dvshortx) & 0;
    dvshortx ver_cnt    = chess_dont_care(dvshortx) & 0;

    const int32_t SlopDifThre = trail_Param->SlopDifThre * 2;
    int32_t count_store       = 0;
    int32_t count_madd        = 0;
    int32_t pred_store        = 0;
    int32_t pred_madd         = 0;

    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(12, )
    chess_unroll_loop(4)
    {
        dvshortx dist   = dvushort_load(input_agen);
        dvshortx dist_c = dvushort_load(center_agen);

        dvshortx near_dist_th = dvushort_load(near_dist_th_agen);

        count_store = mod_inc_pred_z(count_store, niter1 - 1, pred_store);//控制迭代8次后，计数器9次后置1

        dvshortx dif_distC_ver_abs = dvmux(dist, dvabsdif(dist, dist_c), vmax);

        near_cnt_v = dvmaddh((dif_distC_ver_abs < near_dist_th), 1, near_cnt_v, VPU_ROUND_0, pred_madd);//pred_madd=1：执行累加（正常计数)
        ver_cnt    = dvmaddh((dif_distC_ver_abs < SlopDifThre), 1, ver_cnt, VPU_ROUND_0, pred_madd);

        count_madd = mod_inc_pred_nz(count_madd, niter1 - 1, pred_madd);

        vstore(near_cnt_v, near_cnt_v_agen, pred_store);
        vstore(ver_cnt, ver_cnt_agen, pred_store);
    }
}

/**
 * \brief  Part5 initialization function
 * \param[in] config : Part5 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] input_line_pitch : Line pitch with halo
 *                Range: 194. Accuracy: NA.
 * \param[in] near_cnt_v : Vertical distance to nearest point count
 *                Range: 0-9. Accuracy: 1.
 * \param[in] ver_cnt : Number of neighborhood points satisfying the absolute
 *                      distance difference condition relative to the center point
 *                Range: 0-9. Accuracy: 1.
 * \param[in] mask : Center point valid mark
 *                Range: 0-1. Accuracy: 1.
 * \param[in] wall_judge : Wall judgment
 *                Range: 0-1. Accuracy: 1.
 * \param[in] hortrail_judge : Horizontal direction trail judgment
 *                Range: 0-1. Accuracy: 1.
 * \param[in] con3_1 : Condition 3_1
 *                Range: 0-1. Accuracy: 1.
 * \param[in] refer_mask : Refer mask
 *                Range: 0-1. Accuracy: 1.
 */
void trail_cond_part5_init(uint16_t *near_cnt_v, uint16_t *ver_cnt, uint16_t *wall_judge, uint16_t *mask,
                                 uint16_t *Hortrail_judge, uint16_t *con3_1, uint16_t *refer_mask, uint16_t *draw_judge,
                                 int32_t input_line_pitch, Part5Config_t *config)
{
    AgenWrapper agen_wrapper;
    agen_wrapper.size = sizeof(uint16_t);
    agen_wrapper.n1   = TILE_WIDTH / VECW;
    agen_wrapper.s1   = VECW;
    agen_wrapper.n2   = TILE_HEIGHT;
    agen_wrapper.s2   = TILE_WIDTH;

    agen near_cnt_v_agen = init((dvushort *)near_cnt_v);
    INIT_AGEN2(near_cnt_v_agen, agen_wrapper);
    agen ver_cnt_agen = init((dvushort *)ver_cnt);
    INIT_AGEN2(ver_cnt_agen, agen_wrapper);
    agen wall_judge_agen = init((dvushort *)wall_judge);
    INIT_AGEN2(wall_judge_agen, agen_wrapper);
    agen mask_agen = init((dvushort *)mask);
    INIT_AGEN2(mask_agen, agen_wrapper);
    agen Hortrail_judge_agen = init((dvushort *)Hortrail_judge);
    INIT_AGEN2(Hortrail_judge_agen, agen_wrapper);
    agen con3_1_agen = init((dvushort *)con3_1);
    INIT_AGEN2(con3_1_agen, agen_wrapper);
    agen refer_mask_agen = init((dvushort *)refer_mask);
    INIT_AGEN2(refer_mask_agen, agen_wrapper);
    agen draw_judge_agen = init((dvushort *)draw_judge);
    INIT_AGEN2(draw_judge_agen, agen_wrapper);
    agen output_agen    = init((dvushort *)NULL);
    INIT_AGEN2(output_agen, agen_wrapper);

    AgenWrapper dist_wrapper;
    dist_wrapper.size = sizeof(uint16_t);
    dist_wrapper.n1   = TILE_WIDTH / VECW;
    dist_wrapper.s1   = VECW;
    dist_wrapper.n2   = TILE_HEIGHT;
    dist_wrapper.s2   = input_line_pitch;
    agen dist_agen    = init((dvushort *)NULL);
    INIT_AGEN2(dist_agen, dist_wrapper);

    config->center_dist    = extract_agen_cfg(dist_agen);
    config->near_cnt_v     = extract_agen_cfg(near_cnt_v_agen);
    config->ver_cnt        = extract_agen_cfg(ver_cnt_agen);
    config->wall_judge     = extract_agen_cfg(wall_judge_agen);
    config->mask           = extract_agen_cfg(mask_agen);
    config->Hortrail_judge = extract_agen_cfg(Hortrail_judge_agen);
    config->con3_1         = extract_agen_cfg(con3_1_agen);
    config->refer_mask     = extract_agen_cfg(refer_mask_agen);
    config->draw_judge     = extract_agen_cfg(draw_judge_agen);
    config->output         = extract_agen_cfg(output_agen);
    config->niter          = (TILE_WIDTH / VECW) * TILE_HEIGHT;
}

/**
 * \brief  Part5 execute function
 * \param[in] config : Part5 configuration structure
 *                Range: NA. Accuracy: NA.
 * \param[in] trail_Param : Trail parameters structure
 *                Range: NA. Accuracy: NA.
 */
void trail_cond_part5_exec(TrailParam_t *trail_Param, Part5Config_t *config)
{
    agen near_cnt_v_agen     = init_agen_from_cfg(config->near_cnt_v);
    agen ver_cnt_agen        = init_agen_from_cfg(config->ver_cnt);
    agen wall_judge_agen     = init_agen_from_cfg(config->wall_judge);
    agen mask_agen           = init_agen_from_cfg(config->mask);
    agen Hortrail_judge_agen = init_agen_from_cfg(config->Hortrail_judge);
    agen con3_1_agen         = init_agen_from_cfg(config->con3_1);
    agen refer_mask_agen     = init_agen_from_cfg(config->refer_mask);
    agen draw_judge_agen     = init_agen_from_cfg(config->draw_judge);
    agen center_dist_agen     = init_agen_from_cfg(config->center_dist);
    agen output_agen         = init_agen_from_cfg(config->output);

    int32_t niter = config->niter;
    int32_t near_cnt_th_v = trail_Param->near_cnt_th_v;
    int32_t draw_dist_end = trail_Param->draw_dist_end;
    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(12, )
    chess_unroll_loop(4)
    {
        dvshortx near_cnt_v     = dvushort_load(near_cnt_v_agen);
        dvshortx ver_cnt        = dvushort_load(ver_cnt_agen);
        dvshortx wall_judge     = dvushort_load(wall_judge_agen);
        dvshortx mask           = dvushort_load(mask_agen);
        dvshortx Hortrail_judge = dvushort_load(Hortrail_judge_agen);
        dvshortx con3_1         = dvushort_load(con3_1_agen);
        dvshortx refer_mask     = dvushort_load(refer_mask_agen);
        dvshortx draw_judge     = dvushort_load(draw_judge_agen);
        dvshortx center_dist    = dvushort_load(center_dist_agen);

        dvshortx con3_2         = near_cnt_v < near_cnt_th_v;
        dvshortx con3           = con3_1 & con3_2;
        dvshortx con4_3         = ver_cnt < 3;
        dvshortx con4           = wall_judge | con4_3;
        dvshortx con5           = draw_judge & (center_dist < draw_dist_end);
        dvshortx Vertrail_judge = (con3 & con4) | con5;
        dvshortx output         = mask & ((Hortrail_judge & (refer_mask == 0)) | draw_judge) & Vertrail_judge;

        vstore(output, output_agen);
    }
}
/**
 * \brief  Trail VPU main function
 */
CUPVA_VPU_MAIN() {
    TrailParam_t *trail_Param = (TrailParam_t *)algorithmParams;

    uint16_t srcDistLinePitch = cupvaRasterDataFlowGetLinePitch(sourceDistDataFlowHandler);
    uint16_t dstLinePitch = cupvaRasterDataFlowGetLinePitch(destinationDataFlowHandler);

    int32_t srcDistOffset = 0;
    int32_t dstOffset = 0;
    vec0.lo = replicateh(0);
    vec0.hi = replicateh(0);

    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);

    check_center_init(maskBuffer, nearDistThBuffer, adjDisThreDBuffer, srcDistLinePitch, &maskParam);
    trail_cond_part1_init(nearDistThBuffer, adjDisThreDBuffer, con1_3buffer, con3_1buffer, refer_buffer,
                            srcDistLinePitch, &Part1Param);
    trail_cond_draw_init(draw_judge_Buffer, srcDistLinePitch, &DrawParam);
    trail_cond_part2_init(adjDisThreDBuffer, con1_3buffer, hortrail_judge_buffer, srcDistLinePitch, &Part2Param);
    trail_cond_part3_init(wall_judge_buffer, srcDistLinePitch, &Part3Param);
    trail_cond_part4_init(nearDistThBuffer, nearCntVBuffer, verCntBuffer,srcDistLinePitch, &Part4Param);
    trail_cond_part5_init(nearCntVBuffer, verCntBuffer, wall_judge_buffer, maskBuffer, hortrail_judge_buffer,
                            con3_1buffer, refer_buffer, draw_judge_Buffer, srcDistLinePitch, &Part5Param);

    for (int TileIdx = 0; TileIdx < TILE_COUNT; TileIdx++)
    {
        for(int i = 0; i < TILE_WIDTH * TILE_HEIGHT; i++){
            outputValidBufferVMEM[i + dstOffset] = 0;
        }
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler);
        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);

        /** Update agen base address */
        cupvaModifyAgenCfgBase(&maskParam.input_dist, &inputDistBufferVMEM[srcDistOffset + KERNEL_RADIUS_WIDTH + 6 * srcDistLinePitch]);
        cupvaModifyAgenCfgBase(&Part1Param.input_dist, &inputDistBufferVMEM[srcDistOffset + KERNEL_RADIUS_WIDTH + 4 * srcDistLinePitch]);
        cupvaModifyAgenCfgBase(&DrawParam.input_dist, &inputDistBufferVMEM[srcDistOffset + KERNEL_RADIUS_WIDTH]);
        cupvaModifyAgenCfgBase(&Part2Param.input_dist, &inputDistBufferVMEM[srcDistOffset + KERNEL_RADIUS_WIDTH + 4 * srcDistLinePitch]);
        cupvaModifyAgenCfgBase(&Part3Param.input_dist, &inputDistBufferVMEM[srcDistOffset + KERNEL_RADIUS_WIDTH + 4 * srcDistLinePitch]);
        cupvaModifyAgenCfgBase(&Part4Param.input_dist, &inputDistBufferVMEM[srcDistOffset + 6 * srcDistLinePitch]);
        cupvaModifyAgenCfgBase(&Part4Param.center, &inputDistBufferVMEM[srcDistOffset + KERNEL_RADIUS_WIDTH + 6 * srcDistLinePitch]);
        cupvaModifyAgenCfgBase(&Part5Param.center_dist, &inputDistBufferVMEM[srcDistOffset + KERNEL_RADIUS_WIDTH + 6 * srcDistLinePitch]);
        cupvaModifyAgenCfgBase(&Part5Param.output, &outputValidBufferVMEM[dstOffset]);

        check_center_exec(trail_Param, &maskParam);
        trail_cond_part1_exec(trail_Param, &Part1Param);
        trail_cond_draw_exec(trail_Param, &DrawParam);
        trail_cond_part2_exec(trail_Param, &Part2Param);
        trail_cond_part3_exec(trail_Param, &Part3Param);
        trail_cond_part4_exec(trail_Param, &Part4Param);
        trail_cond_part5_exec(trail_Param, &Part5Param);

        srcDistOffset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler, srcDistOffset);
        dstOffset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler, dstOffset);

        cupvaRasterDataFlowSync(destinationDataFlowHandler);
        cupvaRasterDataFlowTrig(destinationDataFlowHandler);
    }

    cupvaRasterDataFlowSync(destinationDataFlowHandler);
    return 0;
}
