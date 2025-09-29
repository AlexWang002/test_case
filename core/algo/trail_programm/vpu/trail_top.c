/*******************************************************************************
 * \addtogroup trail_programm
 * \{
 * \file trail_top.c
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
 * | 0.2 | 2025-09-29 | Trail vector version |

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

/** Use double buffer, the vertical halo is 2, the horizontal halo is 4*/
VMEM(C, uint16_t, inputDistBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
/** Output do not use halo */
VMEM(B, uint16_t, outputValidBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
/** declare algorithm params */
VMEM(C, int, algorithmParams, sizeof(TrailParam_t));
/** declare dataflow handles */
VMEM(C, RasterDataFlowHandler, sourceDistDataFlowHandler);
VMEM(C, RasterDataFlowHandler, destinationDataFlowHandler);

typedef struct {
    AgenCFG input_dist;       /**< AGEN Configuration for horizontal-5 Neighborhood Distance Data*/
    AgenCFG input_longit;     /**< AGEN Configuration for vertical-9 Neighborhood Distance Data*/
    AgenCFG output_valid;     /**< AGEN Configuration for output valid*/
    int32_t niter;            /**< Total number of iterations (= number of horizontal vectors × tile height)*/
    int32_t vecw;             /**< Vector width */
} TrailConfig_t;

dvshortx dif_distC_abs[5];
dvshortx zero_flag[5];
dvshortx dist_tmp[5];
/**
 * \brief  Agen initialization function
 *
 * \param[in] input_dist : Input distance tile start address for input_wrapper
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] intput_dist_longit : Input distance tile start address for longit_wrapper
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] output_valid : Output valid tile start address for output_wrapper
 *                Range: 0 - 2^16-1. Accuracy: 1.
 * \param[in] input_line_pitch : Distance linepitch
 *                Range: 200 Accuracy: 1.
 * \param[in] dst_line_pitch : Valid linepitch
 *                Range: 192. Accuracy: 1.
 * \param[in] config : Trail agen configuration
 *                Range: 0-1. Accuracy: 1.
 */
void trail_remove_init(uint16_t *input_dist, uint16_t *intput_dist_longit, uint16_t *output_valid,
                      int32_t input_line_pitch, int32_t dst_line_pitch,
                      TrailConfig_t *config) {
    /** Get Vector Width (Number of elements contained in each dvshortx)*/
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;

    /** Configuring a 3D AGEN with Vertical 5-Neighborhood Distance Data */
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);       /**< Element size */
    input_wrapper.n1   = 5;                      /**< First Dimension: Vertical 5-Neighborhood */
    input_wrapper.s1   = input_line_pitch;       /**< First Dimension Step: Line Spacing (Vertical Jump) */
    input_wrapper.n2   = TILE_WIDTH / vecw;      /**< Second Dimension: Number of Horizontal Vectors (Split by Vector per Row) */
    input_wrapper.s2   = vecw;                   /**< Second-dimension step size: Vector width (horizontal jump) */
    input_wrapper.n3   = TILE_HEIGHT;            /**< Third Dimension: Tile Height (Total Number of Rows Processed) */
    input_wrapper.s3   = input_line_pitch;       /**< Third-dimensional step size: Line spacing (vertical jump per line) */
    agen input_agen = init((dvushort *)input_dist);
    INIT_AGEN3(input_agen, input_wrapper);
    config->input_dist = extract_agen_cfg(input_agen);

    /** Configuring a 3D AGEN with Horizontal 9-Neighborhood Distance Data */
    AgenWrapper longit_wrapper;
    longit_wrapper.size = sizeof(uint16_t);      /**< Element size */
    longit_wrapper.n1   = 9;                     /**< Window size */
    longit_wrapper.s1   = 1;                     /**< Step size within the window */
    longit_wrapper.n2   = TILE_WIDTH / vecw;
    longit_wrapper.s2   = vecw;
    longit_wrapper.n3   = TILE_HEIGHT;
    longit_wrapper.s3   = input_line_pitch;
    agen longit_agen = init((dvushort *)intput_dist_longit);
    INIT_AGEN3(longit_agen, longit_wrapper);
    config->input_longit = extract_agen_cfg(longit_agen);

    /** Configuring a 2D AGEN with Output Valid Data */
    AgenWrapper output_wrapper;
    output_wrapper.size = sizeof(uint16_t);
    output_wrapper.n1   = TILE_WIDTH / vecw;
    output_wrapper.s1   = vecw;
    output_wrapper.n2   = TILE_HEIGHT;
    output_wrapper.s2   = dst_line_pitch;
    agen output_agen = init((dvushort *)output_valid);
    INIT_AGEN2(output_agen, output_wrapper);
    config->output_valid = extract_agen_cfg(output_agen);

    /** Compute total iteration count */
    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}


/**
 * \brief  Horizontal trail remove judge function
 *
 * \param[in] dist_tmp : Distance input array
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] dist_trail : Current point distance
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] trail_Param : Trail parameter
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return  Hortrail_judge : Horizontal trail success flag
 *                Range: 0-1. Accuracy: 1.
 */
dvshortx dv_HorTrailRemove(dvshortx dist_tmp[5], dvshortx dist_trail,
                          const TrailParam_t *trail_Param) {
    dvshortx weight[5];
    weight[0].lo = replicateh(1);
    weight[0].hi = replicateh(1);
    weight[1].lo = replicateh(2);
    weight[1].hi = replicateh(2);
    weight[2].lo = replicateh(0);
    weight[2].hi = replicateh(0);
    weight[3].lo = replicateh(2);
    weight[3].hi = replicateh(2);
    weight[4].lo = replicateh(1);
    weight[4].hi = replicateh(1);

    for(int i = 0; i < 5; i++) {
        dif_distC_abs[i].lo = replicateh(0);
        dif_distC_abs[i].hi = replicateh(0);
        zero_flag[i].lo = replicateh(0);
        zero_flag[i].hi = replicateh(0);
    }
    /** Handle invalid zero-value points */
    for (int i = 0; i < 5; i++) {
        zero_flag[i] = (dist_tmp[i] == 0);
        /** The absolute value of the difference at invalid points (0 value) is set to 65535;
            valid points use the actual absolute difference value. */
        dif_distC_abs[i] = dvmux(zero_flag[i], 65535, dvabsdif(dist_tmp[i], dist_trail));
        /** Reset the weight of invalid points to zero */
        weight[i] = dvmux(zero_flag[i], 0, weight[i]);
    }

    /** Count the number of zero values */
    dvshortx zero_cnt = zero_flag[0] + zero_flag[1] + zero_flag[2] + zero_flag[3] + zero_flag[4];
    dvshortx con0 = zero_cnt <= 3;

    dvshortx dif_dist_abs[4];
    dif_dist_abs[0].lo =  replicateh(0);
    dif_dist_abs[0].hi =  replicateh(0);
    dif_dist_abs[1].lo =  replicateh(0);
    dif_dist_abs[1].hi =  replicateh(0);
    dif_dist_abs[2].lo =  replicateh(0);
    dif_dist_abs[2].hi =  replicateh(0);
    dif_dist_abs[3].lo =  replicateh(0);
    dif_dist_abs[3].hi =  replicateh(0);

    dvshortx dif_dist_abs_cnt;
    dif_dist_abs_cnt.lo = replicateh(0);
    dif_dist_abs_cnt.hi = replicateh(0);
    dvshortx mul_result = dvmulh(dist_trail, trail_Param->DisThreRatio, VPU_TRUNC_0);
    dvshortx shift_result = mul_result >> 12;
    dvshortx AdjDisThreD = dvmax(shift_result, 1);

    for (int i = 0; i < 4; i++) {
        dif_dist_abs[i] = dvabsdif(dist_tmp[i+1], dist_tmp[i]);
        dif_dist_abs_cnt += (dif_dist_abs[i] > AdjDisThreD);
    }

    dvshortx con1_1 = (dif_distC_abs[1] > AdjDisThreD) &
                     (dif_distC_abs[1] < trail_Param->D_H);
    dvshortx con1_2 = (dif_distC_abs[3] > AdjDisThreD) &
                     (dif_distC_abs[3] < trail_Param->D_H);
    dvshortx con1_3 = (dif_dist_abs_cnt > 3);
    dvshortx con1 = (con1_1 & con1_2) | con1_3;

    dvshortx weighted_sum = dvmulh(weight[0], dif_distC_abs[0], VPU_TRUNC_0) +
                            dvmulh(weight[1], dif_distC_abs[1], VPU_TRUNC_0) +
                            dvmulh(weight[2], dif_distC_abs[2], VPU_TRUNC_0) +
                            dvmulh(weight[3], dif_distC_abs[3], VPU_TRUNC_0) +
                            dvmulh(weight[4], dif_distC_abs[4], VPU_TRUNC_0);
    dvshortx distdiff_mean = weighted_sum >> 2;
    dvshortx con2 = (distdiff_mean > AdjDisThreD) &
                   (distdiff_mean < trail_Param->D_H);
    return con0 & (con1 | con2);
}

/**
 * \brief  verital trail remove judge function
 *
 * \param[in] dist_trail : Current point distance
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] dist_longit : Longitudinal distance array
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] wall_judge : Wall point mark
 *                Range: 0-1. Accuracy: 1.
 * \param[in] near_cnt_h : Horizontal nearest neighbor count
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] near_dist_th : Nearest neighbor distance threshold
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] trail_Param : Trail parameter
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \return Vertrail_judge : Vertical trail mask
 */
dvshortx dv_VerTrailRemove(dvshortx dist_trail, dvshortx dist_longit[9],
                          dvshortx wall_judge, dvshortx near_cnt_h,
                          dvshortx near_dist_th, const TrailParam_t *trail_Param) {
    /** Calculate the absolute difference between the vertical neighborhood and the current point
        (ignoring invalid points with zero values) */
    dvshortx dif_distC_ver_abs[9];
    for (int i = 0; i < 9; i++) {
        dvshortx zero_flag = (dist_longit[i] == 0);
        /** The absolute value of invalid point differences is set to 65535.
            Valid points are determined by the neighborhood - current. */
        dif_distC_ver_abs[i] = dvmux(zero_flag, 65535, dvabsdif(dist_longit[i], dist_trail));
    }

    dvshortx near_cnt_v;
    near_cnt_v.lo = replicateh(0);
    near_cnt_v.hi = replicateh(0);
    dvshortx ver_cnt;
    ver_cnt.lo = replicateh(0);
    ver_cnt.hi = replicateh(0);
    for (int i = 0; i < 9; i++) {
        near_cnt_v += (dif_distC_ver_abs[i] < near_dist_th);
        ver_cnt += (dif_distC_ver_abs[i] < (trail_Param->SlopDifThre * 2));
    }

    dvshortx con3 = (near_cnt_h < trail_Param->near_cnt_th_h) &
                   (near_cnt_v < trail_Param->near_cnt_th_v);
    dvshortx con4 = wall_judge | (ver_cnt < 3);

    return con3 & con4;
}

/**
 * \brief  Trail main fucntion
 *
 * \param[in] trail_Param : Trail parameter
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] config : Trail agen configuration
 *                Range: 0 - 2^32-1. Accuracy: 1
 */
void trail_remove_exec(TrailParam_t *trail_Param, TrailConfig_t *config) {
    /** Initialize AGEN */
    agen input_dist_agen = init_agen_from_cfg(config->input_dist);
    agen input_longit_agen = init_agen_from_cfg(config->input_longit);
    agen output_valid_agen = init_agen_from_cfg(config->output_valid);
    int32_t niter = config->niter;

    const int32_t near_threshold = trail_Param->near_cnt_th_h;
    const int32_t bypass_distance = trail_Param->BypassDis;
    int32_t dist_th_ratio = (trail_Param->dist_th_ratio) << 4;

    /** Main Loop */
    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining
    chess_loop_range(6, )
    chess_unroll_loop(2) {
        dvshortx dist_tmp[5];
        dist_tmp[0].lo = replicateh(0);
        dist_tmp[0].hi = replicateh(0);
        dist_tmp[1].lo = replicateh(0);
        dist_tmp[1].hi = replicateh(0);
        dist_tmp[2].lo = replicateh(0);
        dist_tmp[2].hi = replicateh(0);
        dist_tmp[3].lo = replicateh(0);
        dist_tmp[3].hi = replicateh(0);
        dist_tmp[4].lo = replicateh(0);
        dist_tmp[4].hi = replicateh(0);

        dist_tmp[0] = dvushort_load(input_dist_agen);
        dist_tmp[1] = dvushort_load(input_dist_agen);
        dist_tmp[2] = dvushort_load(input_dist_agen);
        dist_tmp[3] = dvushort_load(input_dist_agen);
        dist_tmp[4] = dvushort_load(input_dist_agen);

        /** Filter invalid points */
        dvshortx valid_flag = (dist_tmp[2] > 0) & (dist_tmp[2] < bypass_distance);

        dvshortx near_dist_th = dvmulh(dist_tmp[2], dist_th_ratio, VPU_TRUNC_7);
        near_dist_th = dvmax(near_dist_th, 2);
        near_dist_th = dvmin(near_dist_th, 20);

        dvshortx near_cnt_h;
        near_cnt_h.lo = replicateh(0);
        near_cnt_h.hi = replicateh(0);
        near_cnt_h += (dvabsdif(dist_tmp[1], dist_tmp[0]) < near_dist_th);
        near_cnt_h += (dvabsdif(dist_tmp[2], dist_tmp[1]) < near_dist_th);
        near_cnt_h += (dvabsdif(dist_tmp[3], dist_tmp[2]) < near_dist_th);
        near_cnt_h += (dvabsdif(dist_tmp[4], dist_tmp[3]) < near_dist_th);

        dvshortx trail_refer_mask = (near_cnt_h >= near_threshold);
        /** Points requiring further processing */
        dvshortx process_flag = valid_flag & ~trail_refer_mask;

        dvshortx hortrail_judge = dv_HorTrailRemove(dist_tmp, dist_tmp[2], trail_Param);
        process_flag &= hortrail_judge;

        /** Load vertical 9-neighborhood distance data */
        dvshortx dist_longit[9];
        for (int j = 0; j < 9; j++) {
            dist_longit[j] = dvushort_load(input_longit_agen);
        }

        /** Calculate wall marker */
        dvshortx dif_dist[4];
        dif_dist[0] = dist_tmp[1] - dist_tmp[0];
        dif_dist[1] = dist_tmp[2] - dist_tmp[1];
        dif_dist[2] = dist_tmp[3] - dist_tmp[2];
        dif_dist[3] = dist_tmp[4] - dist_tmp[3];
        dvshortx dif2_dist_abs[3];
        dif2_dist_abs[0] = dvabsdif(dif_dist[1], dif_dist[0]);
        dif2_dist_abs[1] = dvabsdif(dif_dist[2], dif_dist[1]);
        dif2_dist_abs[2] = dvabsdif(dif_dist[3], dif_dist[2]);
        dvshortx sum_dif2_dist_abs = dif2_dist_abs[0] + dif2_dist_abs[1] + dif2_dist_abs[2];
        dvshortx wall_mask1 = sum_dif2_dist_abs > trail_Param->SlopDifThre;
        dvshortx wall_mask2 = dif2_dist_abs[0] > trail_Param->SlopDifThre;
        dvshortx wall_judge = wall_mask1 & wall_mask2;

        dvshortx vertrail_judge = dv_VerTrailRemove(dist_tmp[2], dist_longit,
                                                  wall_judge, near_cnt_h,
                                                  near_dist_th, trail_Param);

        dvshortx result = vertrail_judge & process_flag;
        vstore(result, output_valid_agen);
    }
}

/**
 * \brief  VPU main function
 */
CUPVA_VPU_MAIN() {
    TrailParam_t *trail_Param = (TrailParam_t *)algorithmParams;
    TrailConfig_t config;

    uint16_t srcDistLinePitch = cupvaRasterDataFlowGetLinePitch(sourceDistDataFlowHandler);
    uint16_t dstLinePitch = cupvaRasterDataFlowGetLinePitch(destinationDataFlowHandler);

    int32_t srcDistOffset = 0;
    int32_t dstOffset = 0;

    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);

    for (int TileIdx = 0; TileIdx < TILE_COUNT; TileIdx++)
    {
        for(int i = 0; i < TILE_WIDTH * TILE_HEIGHT; i++){
            outputValidBufferVMEM[i + dstOffset] = 0;
        }
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler);
        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);

        /** Update agen base address */
        trail_remove_init(&inputDistBufferVMEM[srcDistOffset+ KERNEL_RADIUS_WIDTH],
                            &inputDistBufferVMEM[srcDistOffset + 2 * srcDistLinePitch],
                            &outputValidBufferVMEM[dstOffset],
                            srcDistLinePitch,
                            dstLinePitch,
                            &config);

        trail_remove_exec(trail_Param, &config);

        srcDistOffset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler, srcDistOffset);
        dstOffset = cupvaRasterDataFlowGetOffset(destinationDataFlowHandler, dstOffset);

        cupvaRasterDataFlowSync(destinationDataFlowHandler);
        cupvaRasterDataFlowTrig(destinationDataFlowHandler);
    }

    cupvaRasterDataFlowSync(destinationDataFlowHandler);
    return 0;
}
