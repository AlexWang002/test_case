/*******************************************************************************
 * \addtogroup upsample_programm
 * \{
 * \file upsample_top_v.c
 * \brief
 * \version 0.3
 * \date 2025-11-19
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
 * | 0.2 | 2025-10-13 | Upsample vector version |
 *
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.3 | 2025-11-19 | Add attribute output |
 ******************************************************************************/
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_device.h> /* Main device-side header file */
#include <cupva_device_debug.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../upsample_commom_param.h"

VMEM(B, uint16_t, inputDistBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint16_t, inputRefBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint16_t, inputAttrBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

/** Output and raw data do not use halo */
VMEM(B, uint16_t, inputDistRawBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, inputRefRawBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));

VMEM(C, uint16_t, outputDistUpBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, outputRefUpBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint16_t, outputAttrBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));

/** declare algorithm params */
VMEM(C, int, algorithmParams, sizeof(InsertParam_t));

/** declare dataflow handles */
VMEM(C, RasterDataFlowHandler, InputDistDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputRefDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputDistRawDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputRefRawDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputAttrDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputDistUpDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputRefUpDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputAttrDataFlowHandler);

typedef struct {
    AgenCFG input_dist;       /**< AGEN Configuration for vertical-2 Neighborhood Distance Data*/
    AgenCFG input_ref;        /**< AGEN Configuration for vertical-2 Neighborhood Reflectivity Data*/
    AgenCFG input_attr;       /**< AGEN Configuration for vertical-2 Neighborhood Attribute Data*/

    AgenCFG input_dist_raw;   /**< AGEN Configuration for Distance Raw Data*/
    AgenCFG input_ref_raw;    /**< AGEN Configuration for Reflectivity Raw Data*/
    AgenCFG output_dist_up;   /**< AGEN Configuration for Distance Output Data*/
    AgenCFG output_ref_up;    /**< AGEN Configuration for Reflectivity Output Data*/
    AgenCFG output_attr_up;   /**< AGEN Configuration for Attribute Output Data*/
    int32_t vecw;
    int32_t niter;            /**< Total number of iterations */
} UpsampleConfig_t;

/**
 * \brief  Agen initialization function
 * \param[in] agen_config : Upsample agen configuration
 *                Range: 0-1. Accuracy: 1.
 * \param[in] HaloLinePitch : Halo input linepitch
 *              Range: 194. Accuracy: 1.
 * \param[in] NoHaloLinePitch : NoHalo raw input linepitch
 *              Range: 192. Accuracy: 1.
 */
static void upsample_agen_init(UpsampleConfig_t *agen_config, uint16_t HaloLinePitch,  uint16_t NoHaloLinePitch) {
    /** Get Vector Width (Number of elements contained in each dvshortx)*/
    agen_config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = agen_config->vecw;

    AgenWrapper agen_halo_wrapper;
    agen_halo_wrapper.size = sizeof(uint16_t);    /**< Element size */
    agen_halo_wrapper.n1 = 2;                     /**< First Dimension: Vertical 2-Neighborhood */
    agen_halo_wrapper.s1 = HaloLinePitch;         /**< First Dimension Step: Line Spacing (Vertical Jump) */
    agen_halo_wrapper.n2 = TILE_WIDTH / vecw;     /**< Second Dimension: Number of Horizontal Vectors (Split by Vector per Row) */
    agen_halo_wrapper.s2 = vecw;                  /**< Second-dimension step size: Vector width (horizontal jump) */
    agen_halo_wrapper.n3 = TILE_HEIGHT;           /**< Third Dimension: Tile Height (Total Number of Rows Processed) */
    agen_halo_wrapper.s3 = HaloLinePitch;         /**< Third-dimensional step size: Line spacing (vertical jump per line) */

    agen input_dist_agen = init((dvshortx *)NULL);
    INIT_AGEN3(input_dist_agen, agen_halo_wrapper);
    agen_config->input_dist = extract_agen_cfg(input_dist_agen);

    agen input_ref_agen = init((dvshortx *)NULL);
    INIT_AGEN3(input_ref_agen, agen_halo_wrapper);
    agen_config->input_ref = extract_agen_cfg(input_ref_agen);

    agen input_attr_agen = init((dvshortx *)NULL);
    INIT_AGEN3(input_attr_agen, agen_halo_wrapper);
    agen_config->input_attr = extract_agen_cfg(input_attr_agen);

    AgenWrapper agen_wrapper;
    agen_wrapper.size = sizeof(uint16_t);
    agen_wrapper.n1 = TILE_WIDTH / vecw;
    agen_wrapper.s1 = vecw;
    agen_wrapper.n2 = TILE_HEIGHT;
    agen_wrapper.s2 = NoHaloLinePitch;

    agen dist_raw_agen = init((dvshortx *)NULL);
    INIT_AGEN2(dist_raw_agen, agen_wrapper);
    agen_config->input_dist_raw = extract_agen_cfg(dist_raw_agen);

    agen ref_raw_agen = init((dvshortx *)NULL);
    INIT_AGEN2(ref_raw_agen, agen_wrapper);
    agen_config->input_ref_raw = extract_agen_cfg(ref_raw_agen);

    agen out_dist_agen = init((dvshortx *)NULL);
    INIT_AGEN2(out_dist_agen, agen_wrapper);
    agen_config->output_dist_up = extract_agen_cfg(out_dist_agen);

    agen out_ref_agen = init((dvshortx *)NULL);
    INIT_AGEN2(out_ref_agen, agen_wrapper);
    agen_config->output_ref_up = extract_agen_cfg(out_ref_agen);

    agen out_attr_agen = init((dvshortx *)NULL);
    INIT_AGEN2(out_attr_agen, agen_wrapper);
    agen_config->output_attr_up = extract_agen_cfg(out_attr_agen);
    /** Compute total iteration count */
    agen_config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}

/**
 * \brief  Upsample main fucntion
 *
 * \param[in] param : Upsample parameter
 *                Range: 0 - 2^32-1. Accuracy: 1.
 * \param[in] agen_config : Upsample agen configuration
 *                Range: 0 - 2^32-1. Accuracy: 1
 */
static void upsample_exec(InsertParam_t *param, UpsampleConfig_t *agen_config) {
    /** Initialize AGEN */
    agen input_dist_agen = init_agen_from_cfg(agen_config->input_dist);
    agen input_ref_agen = init_agen_from_cfg(agen_config->input_ref);
    agen input_attr_agen = init_agen_from_cfg(agen_config->input_attr);
    agen dist_raw_agen = init_agen_from_cfg(agen_config->input_dist_raw);
    agen ref_raw_agen = init_agen_from_cfg(agen_config->input_ref_raw);
    agen out_dist_agen = init_agen_from_cfg(agen_config->output_dist_up);
    agen out_ref_agen = init_agen_from_cfg(agen_config->output_ref_up);
    agen out_attr_agen = init_agen_from_cfg(agen_config->output_attr_up);

    /** Main Loop */
    for (int32_t i = 0; i < agen_config->niter; i++)
    chess_prepare_for_pipelining
    chess_unroll_loop(2)
    {
        dvshortx dist_raw = dvushort_load(dist_raw_agen);
        dvshortx  ref_raw  = dvushort_load(ref_raw_agen);

        dvshortx dist_tmp0;
        dvshortx dist_tmp1;

        dvshortx ref_tmp0;
        dvshortx ref_tmp1;

        dvshortx attr_tmp0;
        dvshortx attr_tmp1;

        dvshortx  dist_ins = chess_dont_care(dvshortx) & 0;
        dvshortx  ref_ins = chess_dont_care(dvshortx) & 0;
        dvshortx  attr_ins = chess_dont_care(dvshortx) & 0;


        dist_tmp0  = dvushort_load(input_dist_agen);
        dist_tmp1  = dvushort_load(input_dist_agen);
        ref_tmp0   = dvushort_load(input_ref_agen);
        ref_tmp1   = dvushort_load(input_ref_agen);
        attr_tmp0  = dvushort_load(input_attr_agen);
        attr_tmp1  = dvushort_load(input_attr_agen);

        dvshortx diff_1 = dvabsdif(dist_tmp0, dist_raw);
        dvshortx diff_2 = dvabsdif(dist_tmp1, dist_raw);

        dvshortx  cond1_1 = (diff_1 < 200);
        dvshortx  cond1_2 = (diff_2 < 200);
        dvshortx  cond1 = cond1_1 | cond1_2;
        dist_ins = dvmux(cond1, dist_raw, dist_ins);
        ref_ins  = dvmux(cond1, ref_raw, ref_ins);
        attr_ins  = dvmux(cond1, attr_tmp0 | attr_tmp1, attr_ins);

        dvshortx  cond2 = (dvabsdif(dist_tmp0, dist_tmp1)) < 200;
        dvshortx  con1_2 = (~cond1) & (cond2);

        dist_ins = dvmux(con1_2, (dist_tmp0 + dist_tmp1) >> 1, dist_ins);
        ref_ins  = dvmux(con1_2, dvmin(ref_tmp0, ref_tmp1), ref_ins);
        attr_ins = dvmux(con1_2, attr_tmp0 | attr_tmp1, attr_ins);

        vstore(dist_ins, out_dist_agen);
        vstore(ref_ins, out_ref_agen);
        vstore(attr_ins, out_attr_agen);
    }
}

/**
 * \brief  Upsample VPU main function
 */
CUPVA_VPU_MAIN() {
    InsertParam_t *upsample_param = (InsertParam_t *)algorithmParams;
    UpsampleConfig_t agen_config;

    uint16_t HaloLinePitch    = cupvaRasterDataFlowGetLinePitch(InputDistDataFlowHandler);
    uint16_t NoHaloLinePitch = cupvaRasterDataFlowGetLinePitch(InputDistRawDataFlowHandler);

    upsample_agen_init(&agen_config, HaloLinePitch, NoHaloLinePitch);

    int32_t srcDistOffset    = 0;
    int32_t srcDistRawOffset = 0;
    int32_t srcRefOffset     = 0;
    int32_t srcRefRawOffset  = 0;
    int32_t srcAttrOffset    = 0;
    int32_t dstDistOffset    = 0;
    int32_t dstRefOffset     = 0;
    int32_t dstAttrOffset    = 0;

    cupvaRasterDataFlowTrig(InputDistDataFlowHandler);
    cupvaRasterDataFlowTrig(InputRefDataFlowHandler);
    cupvaRasterDataFlowTrig(InputDistRawDataFlowHandler);
    cupvaRasterDataFlowTrig(InputRefRawDataFlowHandler);
    cupvaRasterDataFlowTrig(InputAttrDataFlowHandler);

    for (int TileIdx = 0; TileIdx < TILE_COUNT; TileIdx++) {

        for(int i = 0; i < TILE_WIDTH * TILE_HEIGHT; i++){
            outputDistUpBufferVMEM[i + dstDistOffset] = 0;
            outputRefUpBufferVMEM[i + dstRefOffset] = 0;
        }
        cupvaRasterDataFlowSync(InputDistDataFlowHandler);
        cupvaRasterDataFlowTrig(InputDistDataFlowHandler);
        cupvaRasterDataFlowSync(InputRefDataFlowHandler);
        cupvaRasterDataFlowTrig(InputRefDataFlowHandler);
        cupvaRasterDataFlowSync(InputDistRawDataFlowHandler);
        cupvaRasterDataFlowTrig(InputDistRawDataFlowHandler);
        cupvaRasterDataFlowSync(InputRefRawDataFlowHandler);
        cupvaRasterDataFlowTrig(InputRefRawDataFlowHandler);
        cupvaRasterDataFlowSync(InputAttrDataFlowHandler);
        cupvaRasterDataFlowTrig(InputAttrDataFlowHandler);

        /** Update agen base address */
        cupvaModifyAgenCfgBase(&agen_config.input_dist, &inputDistBufferVMEM[srcDistOffset + HaloLinePitch]);
        cupvaModifyAgenCfgBase(&agen_config.input_ref, &inputRefBufferVMEM[srcRefOffset + HaloLinePitch]);
        cupvaModifyAgenCfgBase(&agen_config.input_attr, &inputAttrBufferVMEM[srcAttrOffset + HaloLinePitch]);
        cupvaModifyAgenCfgBase(&agen_config.input_dist_raw, &inputDistRawBufferVMEM[srcDistRawOffset]);
        cupvaModifyAgenCfgBase(&agen_config.input_ref_raw, &inputRefRawBufferVMEM[srcRefRawOffset]);
        cupvaModifyAgenCfgBase(&agen_config.output_dist_up, &outputDistUpBufferVMEM[dstDistOffset]);
        cupvaModifyAgenCfgBase(&agen_config.output_ref_up, &outputRefUpBufferVMEM[dstRefOffset]);
        cupvaModifyAgenCfgBase(&agen_config.output_attr_up, &outputAttrBufferVMEM[dstAttrOffset]);

        upsample_exec(upsample_param, &agen_config);

        srcDistOffset = cupvaRasterDataFlowGetOffset(InputDistDataFlowHandler, srcDistOffset);
        srcRefOffset = cupvaRasterDataFlowGetOffset(InputRefDataFlowHandler, srcRefOffset);
        srcDistRawOffset = cupvaRasterDataFlowGetOffset(InputDistRawDataFlowHandler, srcDistRawOffset);
        srcRefRawOffset = cupvaRasterDataFlowGetOffset(InputRefRawDataFlowHandler, srcRefRawOffset);
        srcAttrOffset = cupvaRasterDataFlowGetOffset(InputAttrDataFlowHandler, srcAttrOffset);

        dstDistOffset = cupvaRasterDataFlowGetOffset(OutputDistUpDataFlowHandler, dstDistOffset);
        dstRefOffset = cupvaRasterDataFlowGetOffset(OutputRefUpDataFlowHandler, dstRefOffset);
        dstAttrOffset = cupvaRasterDataFlowGetOffset(OutputAttrDataFlowHandler, dstAttrOffset);

        cupvaRasterDataFlowSync(OutputDistUpDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputDistUpDataFlowHandler);
        cupvaRasterDataFlowSync(OutputRefUpDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputRefUpDataFlowHandler);
        cupvaRasterDataFlowSync(OutputAttrDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputAttrDataFlowHandler);
    }

    cupvaRasterDataFlowSync(OutputDistUpDataFlowHandler);
    cupvaRasterDataFlowSync(OutputRefUpDataFlowHandler);
    cupvaRasterDataFlowSync(OutputAttrDataFlowHandler);

    return 0;
}
