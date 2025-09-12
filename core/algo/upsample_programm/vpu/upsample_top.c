/*******************************************************************************
 * \addtogroup upsample_programm
 * \{
 * \file upsample_top.c
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
#include <cupva_device_debug.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../upsample_commom_param.h"

/** Use double buffer, the vertical halo is 1 */
VMEM(B, uint16_t, inputDistBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(A, uint8_t, inputRefBufferVMEM,
    RDF_DOUBLE(uint8_t,TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));

/** Output and raw data do not use halo */
VMEM(A, uint16_t, inputDistRawBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(A, uint8_t, inputRefRawBufferVMEM, RDF_DOUBLE(uint8_t,TILE_WIDTH, TILE_HEIGHT));

VMEM(C, uint16_t, outputDistUpBufferVMEM, RDF_DOUBLE(uint16_t,TILE_WIDTH, TILE_HEIGHT));
VMEM(C, uint8_t, outputRefUpBufferVMEM, RDF_DOUBLE(uint8_t,TILE_WIDTH, TILE_HEIGHT));

/** declare algorithm params */
VMEM(C, int, algorithmParams, sizeof(InsertParam_t));

/** declare dataflow handles */
VMEM(C, RasterDataFlowHandler, InputDistDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputRefDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputDistRawDataFlowHandler);
VMEM(C, RasterDataFlowHandler, InputRefRawDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputDistUpDataFlowHandler);
VMEM(C, RasterDataFlowHandler, OutputRefUpDataFlowHandler);

/**
 * \brief  Upsample processing main function in device-side
*/
CUPVA_VPU_MAIN()
{
    InsertParam_t *upsample_Param = (InsertParam_t *)algorithmParams;
    /** Calculate line pitch */
    uint16_t DistInLinePitch = cupvaRasterDataFlowGetLinePitch(InputDistDataFlowHandler);
    uint16_t DistInRawLinePitch = cupvaRasterDataFlowGetLinePitch(InputDistRawDataFlowHandler);
    uint16_t DistOutUpLinePitch = cupvaRasterDataFlowGetLinePitch(OutputDistUpDataFlowHandler);

    uint8_t RefInLinePitch = cupvaRasterDataFlowGetLinePitch(InputRefDataFlowHandler);
    uint8_t RefInRawLinePitch = cupvaRasterDataFlowGetLinePitch(InputRefRawDataFlowHandler);
    uint8_t RefOutUpLinePitch = cupvaRasterDataFlowGetLinePitch(OutputRefUpDataFlowHandler);

    /** Every tile offset */
    int32_t srcDistOffset = 0;
    int32_t srcDistRawOffset = 0;
    int32_t srcRefOffset = 0;
    int32_t srcRefRawOffset  = 0;
    int32_t dstDistOffset = 0;
    int32_t dstRefOffset  = 0;

    cupvaRasterDataFlowTrig(InputDistDataFlowHandler);
    cupvaRasterDataFlowTrig(InputDistRawDataFlowHandler);
    cupvaRasterDataFlowTrig(InputRefDataFlowHandler);
    cupvaRasterDataFlowTrig(InputRefRawDataFlowHandler);

    /** Loop over tiles, TILE_COUNT = 10*/
    for(int TileIdx = 0; TileIdx < TILE_COUNT; TileIdx++)
    {
        /** Trigger the data flow and switch to the next tile */
        cupvaRasterDataFlowSync(InputDistDataFlowHandler);
        cupvaRasterDataFlowTrig(InputDistDataFlowHandler);
        cupvaRasterDataFlowSync(InputDistRawDataFlowHandler);
        cupvaRasterDataFlowTrig(InputDistRawDataFlowHandler);
        cupvaRasterDataFlowSync(InputRefDataFlowHandler);
        cupvaRasterDataFlowTrig(InputRefDataFlowHandler);
        cupvaRasterDataFlowSync(InputRefRawDataFlowHandler);
        cupvaRasterDataFlowTrig(InputRefRawDataFlowHandler);

        for(int i = 0; i < TILE_HEIGHT * TILE_WIDTH; i++){
            outputDistUpBufferVMEM[dstDistOffset + i] = 0;
            outputRefUpBufferVMEM[dstRefOffset + i] = 0;
        }
#ifdef ALGO_ON
        /** Process the columns except for "halo" */
        for (int col_idx = 1; col_idx < TILE_HEIGHT + 1; ++col_idx) //95
        {
            if((TileIdx != TILE_COUNT - 1) || (col_idx < TILE_HEIGHT))
            {
                const int window_size = (upsample_Param->WH << 1) + 1;

                for (int row_idx = 0; row_idx < TILE_WIDTH; ++row_idx)
                {
                    int dist_tmp1[5] = {0};
                    int dist_tmp2[5] = {0};
                    int ref_tmp1[5] = {0};
                    int ref_tmp2[5] = {0};
                    int dist_raw = inputDistRawBufferVMEM[srcDistRawOffset + (col_idx - 1) * DistInRawLinePitch + row_idx];
                    int ref_raw = inputRefRawBufferVMEM[srcRefRawOffset + (col_idx - 1) * RefInRawLinePitch + row_idx];

                    if (row_idx > upsample_Param->WH - 1 && row_idx < TILE_WIDTH - upsample_Param->WH) { //中间行
                        const int start_row = row_idx - upsample_Param->WH;
                        for (int i = 0; i < window_size; ++i) {
                            const int src_row = start_row + i;
                            dist_tmp1[i] = inputDistBufferVMEM[srcDistOffset + col_idx * DistInLinePitch + src_row];
                            dist_tmp2[i] = inputDistBufferVMEM[srcDistOffset + (col_idx + 1) * DistInLinePitch + src_row];
                            ref_tmp1[i] = inputRefBufferVMEM[srcDistOffset + col_idx * RefInLinePitch + src_row];
                            ref_tmp2[i] = inputRefBufferVMEM[srcDistOffset + (col_idx + 1) * RefInLinePitch + src_row];
                        }
                    }
                    else if (row_idx <= upsample_Param->WH - 1) {    //顶部边界行
                        const int valid_rows = row_idx + upsample_Param->WH + 1;
                        const int dest_start = upsample_Param->WH - row_idx;
                        for (int i = 0; i < valid_rows; ++i) {
                            dist_tmp1[dest_start + i] = inputDistBufferVMEM[srcDistOffset + col_idx * DistInLinePitch + i];
                            dist_tmp2[dest_start + i] = inputDistBufferVMEM[srcDistOffset + (col_idx + 1) * DistInLinePitch + i];
                            ref_tmp1[dest_start + i] = inputRefBufferVMEM[srcDistOffset + col_idx * RefInLinePitch + i];
                            ref_tmp2[dest_start + i] = inputRefBufferVMEM[srcDistOffset + (col_idx + 1) * RefInLinePitch + i];
                        }
                    }
                    else {
                        const int valid_rows = upsample_Param->WH + (TILE_WIDTH - row_idx);
                        const int src_start = row_idx - upsample_Param->WH;
                        for (int i = 0; i < valid_rows; ++i) {
                            const int src_row = src_start + i;
                            dist_tmp1[i] = inputDistBufferVMEM[srcDistOffset + col_idx * DistInLinePitch + src_row];
                            dist_tmp2[i] = inputDistBufferVMEM[srcDistOffset + (col_idx + 1) * DistInLinePitch + src_row];
                            ref_tmp1[i] = inputRefBufferVMEM[srcDistOffset + col_idx * RefInLinePitch + src_row];
                            ref_tmp2[i] = inputRefBufferVMEM[srcDistOffset + (col_idx + 1) * RefInLinePitch + src_row];
                        }
                    }
                    int dist_ins = 0;
                    int ref_ins = 0;
                    const int dist1_mid = dist_tmp1[2];
                    const int dist2_mid = dist_tmp2[2];

                    if(ABS(dist1_mid - dist_raw) < 200 || ABS(dist2_mid - dist_raw) < 200) {
                        dist_ins = dist_raw;
                        ref_ins = ref_raw;
                    } else if (ABS(dist1_mid - dist2_mid) < 200){
                        dist_ins = (dist1_mid + dist2_mid) >> 1;
                        ref_ins = MIN(ref_tmp1[2], ref_tmp2[2]);
                    }

                    outputDistUpBufferVMEM[dstDistOffset + (col_idx - 1) * DistOutUpLinePitch + row_idx] = dist_ins;
                    outputRefUpBufferVMEM[dstRefOffset + (col_idx - 1) * RefOutUpLinePitch + row_idx] = ref_ins;
                }
            }
        }
#endif
        srcDistOffset = cupvaRasterDataFlowGetOffset(InputDistDataFlowHandler, srcDistOffset);
        srcRefOffset = cupvaRasterDataFlowGetOffset(InputRefDataFlowHandler, srcRefOffset);
        srcDistRawOffset = cupvaRasterDataFlowGetOffset(InputDistRawDataFlowHandler, srcDistRawOffset);
        srcRefRawOffset = cupvaRasterDataFlowGetOffset(InputRefRawDataFlowHandler, srcRefRawOffset);

        dstDistOffset = cupvaRasterDataFlowGetOffset(OutputDistUpDataFlowHandler, dstDistOffset);
        dstRefOffset = cupvaRasterDataFlowGetOffset(OutputRefUpDataFlowHandler, dstRefOffset);

        cupvaRasterDataFlowSync(OutputDistUpDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputDistUpDataFlowHandler);
        cupvaRasterDataFlowSync(OutputRefUpDataFlowHandler);
        cupvaRasterDataFlowTrig(OutputRefUpDataFlowHandler);
    }
    cupvaRasterDataFlowSync(OutputDistUpDataFlowHandler);
    cupvaRasterDataFlowSync(OutputRefUpDataFlowHandler);
    return 0;
}
