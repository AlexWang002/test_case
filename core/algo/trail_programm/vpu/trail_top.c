#include "../trail_common_param.h"
#include <cupva_device.h>
#include <cupva_device_debug.h>

// 轨迹算法配置结构体（聚合AGEN配置和执行参数）
typedef struct {
    AgenCFG input_dist;       // 水平5邻域距离数据的AGEN配置
    AgenCFG input_longit;     // 垂直9邻域距离数据的AGEN配置
    AgenCFG output_valid;     // 输出有效缓冲区的AGEN配置
    int32_t niter;            // 总迭代次数（= 横向向量数 × 瓦片高度）
    int32_t vecw;             // 向量宽度（pva_elementsof(dvshortx)）
} TrailConfig_t;

// VMEM缓冲区定义
VMEM(A, uint16_t, inputDistBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT, KERNEL_RADIUS_WIDTH, KERNEL_RADIUS_HEIGHT));
VMEM(B, uint16_t, outputValidBufferVMEM,
    RDF_DOUBLE(uint16_t, TILE_WIDTH, TILE_HEIGHT));
VMEM(C, int, algorithmParams, sizeof(TrailParam_t));

VMEM(C, RasterDataFlowHandler, sourceDistDataFlowHandler);
VMEM_RDF_UNIFIED(B, destinationDataFlowHandler);

dvshortx weight[5];
dvshortx dif_distC_abs[5];
dvshortx zero_flag[5];
// dvshortx dif_distC_ver_abs[9];
dvshortx dist_tmp[5];
// dvshortx dist_longit[9];
/**
 * 初始化函数：为所有瓦片配置AGEN和计算参数
 * @param input_dist: 输入距离数据缓冲区
 * @param output_valid: 输出有效缓冲区
 * @param input_line_pitch: 输入行间距
 * @param dst_line_pitch: 输出行间距
 * @param config: 输出的配置结构体
 */
void trail_remove_init(uint16_t *input_dist, uint16_t *output_valid,
                      int32_t input_line_pitch, int32_t dst_line_pitch,
                      TrailConfig_t *config) {
    // 获取向量宽度（每个dvshortx包含的元素数）
    config->vecw = pva_elementsof(dvshortx);
    int32_t vecw = config->vecw;

    // 1. 配置垂直5邻域距离数据的3维AGEN
    AgenWrapper input_wrapper;
    input_wrapper.size = sizeof(uint16_t);       // 元素大小：16位无符号整数
    input_wrapper.n1   = 5;                      // 第一维度：垂直5邻域（row_idx-2到+2）
    input_wrapper.s1   = input_line_pitch;       // 第一维度步长：行间距（垂直方向跳转）
    input_wrapper.n2   = TILE_WIDTH / vecw;      // 第二维度：横向向量数（每行按向量拆分）
    input_wrapper.s2   = vecw;                   // 第二维度步长：向量宽度（水平方向跳转）
    input_wrapper.n3   = TILE_HEIGHT;            // 第三维度：瓦片高度（处理的总行数）
    input_wrapper.s3   = input_line_pitch;       // 第三维度步长：行间距（垂直方向每行跳转）
    agen input_agen = init((dvshortx *)NULL);
    INIT_AGEN3(input_agen, input_wrapper);       // 初始化3维AGEN
    config->input_dist = extract_agen_cfg(input_agen);

    // 2. 配置水平9邻域数据的2维AGEN
    AgenWrapper longit_wrapper;
    longit_wrapper.size = sizeof(uint16_t);      // 元素大小：16位无符号整数
    longit_wrapper.n1   = 9;                     // 第一维度：水平9邻域（row_idx-4到+4）
    longit_wrapper.s1   = 1;                     // 第一维度步长：1
    longit_wrapper.n2   = TILE_WIDTH / vecw;     // 第二维度：横向向量数
    longit_wrapper.s2   = vecw;                  // 第二维度步长：向量宽度
    agen longit_agen = init((dvshortx *)NULL);
    INIT_AGEN2(longit_agen, longit_wrapper);     // 初始化2维AGEN
    config->input_longit = extract_agen_cfg(longit_agen);

    // 3. 配置输出缓冲区的2维AGEN
    AgenWrapper output_wrapper;
    output_wrapper.size = sizeof(uint16_t);      // 元素大小：16位无符号整数
    output_wrapper.n1   = TILE_WIDTH / vecw;     // 第一维度：横向向量数
    output_wrapper.s1   = vecw;                  // 第一维度步长：向量宽度
    output_wrapper.n2   = TILE_HEIGHT;           // 第二维度：瓦片高度
    output_wrapper.s2   = dst_line_pitch;        // 第二维度步长：输出行间距
    agen output_agen = init((dvshortx *)output_valid);
    INIT_AGEN2(output_agen, output_wrapper);     // 初始化2维AGEN
    config->output_valid = extract_agen_cfg(output_agen);

    // 计算总迭代次数（横向向量数 × 纵向行数）
    config->niter = (TILE_WIDTH / vecw) * TILE_HEIGHT;
}

/**
 * 向量版水平轨迹判断：并行处理向量中每个元素的水平轨迹条件
 * @param dist_tmp: 水平5邻域距离向量（5个向量）
 * @param dist_trail: 当前点距离向量
 * @param trail_Param: 算法阈值参数
 * @return: 水平轨迹判断结果向量（1=是轨迹，0=非轨迹）
 */
dvshortx dv_HorTrailRemove(dvshortx dist_tmp[5], dvshortx dist_trail,
                          const TrailParam_t *trail_Param) {
    // 权重向量初始化（[1,2,0,2,1]）
    // dvshortx weight[5];
    weight[0].lo = replicateh(1);
    weight[0].hi = replicateh(1);
    weight[1].lo = replicateh(1);
    weight[1].hi = replicateh(1);
    weight[2].lo = replicateh(0);
    weight[2].hi = replicateh(0);
    weight[3].lo = replicateh(1);
    weight[3].hi = replicateh(1);
    weight[4].lo = replicateh(1);
    weight[4].hi = replicateh(1);

    // 计算每个邻域与当前点的差异绝对值（处理0值无效点）
    // dvshortx dif_distC_abs[5];
    // dvshortx zero_flag[5];  // 标记当前位置是否为0（1=0，0=有效）
    for (int i = 0; i < 5; i++) {
        zero_flag[i] = (dist_tmp[i] == 0);
        // 无效点（0值）的差异绝对值设为65535，有效点取实际差异绝对值 // 差异 = 邻域 - 当前
        dif_distC_abs[i] = dvmux(zero_flag[i], 65535, dvabsdif(dist_tmp[i], dist_trail));
        // 无效点的权重置0
        weight[i] = dvmux(zero_flag[i], 0, weight[i]);
    }

    // 统计0值数量（用于条件判断）
    dvshortx zero_step1 = dvadd3(zero_flag[0], zero_flag[1], zero_flag[2]);
    dvshortx zero_cnt = dvadd3(zero_flag[3], zero_flag[4], zero_step1);
    // dvshortx zero_cnt = vadd3(vadd3(zero_flag[0], zero_flag[1],zero_flag[2]), zero_flag[3], zero_flag[4]);
    // 水平轨迹判断条件
    dvshortx con0 = zero_cnt <= 3;  // 0值数量≤3（有效点足够）

    // 计算相邻点差异及大差异计数
    dvshortx dif_dist_abs[4];  // 相邻差异绝对值
    vshortx v_dif_dist_abd_cnt = replicateh(0);  // 大差异计数
    dvshortx dif_dist_abs_cnt = deposit_lo(v_dif_dist_abd_cnt);  // 大差异计数
    dvshortx mul_result = dvmulh(dist_trail, trail_Param->DisThreRatio, VPU_TRUNC_0);
    dvshortx shift_result = dvsra(mul_result, 12);
    dvshortx AdjDisThreD = dvmax(shift_result, 1);

    for (int i = 0; i < 4; i++) {
        dif_dist_abs[i] = dvabsdif(dist_tmp[i+1], dist_tmp[i]);  // 相邻差异绝对值
        // 累加大差异标记（1=差异超标，0=正常）
        dif_dist_abs_cnt += (dif_dist_abs[i] > AdjDisThreD);
    }

    // 条件1：(左右邻域差异超标) 或 (大差异数>3)
    dvshortx con1_1 = (dif_distC_abs[1] > AdjDisThreD) &
                     (dif_distC_abs[1] < trail_Param->D_H);
    dvshortx con1_2 = (dif_distC_abs[3] > AdjDisThreD) &
                     (dif_distC_abs[3] < trail_Param->D_H);
    dvshortx con1_3 = (dif_dist_abs_cnt > 3);
    dvshortx con1 = (con1_1 & con1_2) | con1_3;

    // 条件2：加权平均差异在阈值范围内
    dvshortx sum_step1 = dvadd3(dvmulh(weight[0], dif_distC_abs[0], VPU_TRUNC_0),
                                dvmulh(weight[1], dif_distC_abs[1], VPU_TRUNC_0),
                                dvmulh(weight[2], dif_distC_abs[2], VPU_TRUNC_0));
    dvshortx weighted_sum = dvadd3(sum_step1,
                                 dvmulh(weight[3], dif_distC_abs[3], VPU_TRUNC_0),
                                 dvmulh(weight[4], dif_distC_abs[4], VPU_TRUNC_0));

    // dvshortx weighted_sum = dvadd3(dvadd3(dvmulh(weight[0], dif_distC_abs[0], VPU_TRUNC_0),
    //                                    dvmulh(weight[1], dif_distC_abs[1], VPU_TRUNC_0),
    //                                    dvmulh(weight[2], dif_distC_abs[2], VPU_TRUNC_0)),
    //                                    dvmulh(weight[3], dif_distC_abs[3], VPU_TRUNC_0),
    //                                    dvmulh(weight[4], dif_distC_abs[4], VPU_TRUNC_0));
    dvshortx distdiff_mean = dvsra(weighted_sum, 2);  // 除以4（权重和为6，保持原逻辑）
    dvshortx con2 = (distdiff_mean > AdjDisThreD) &
                   (distdiff_mean < trail_Param->D_H);

    // 最终水平轨迹判断：con0且(con1或con2)
    return con0 & (con1 | con2);
}

/**
 * 向量版垂直轨迹判断：并行处理向量中每个元素的垂直轨迹条件
 * @param dist_trail: 当前点距离向量
 * @param dist_longit: 垂直9邻域距离向量（9个向量）
 * @param wall_judge: 墙判断结果向量
 * @param near_cnt_h: 水平近邻计数向量
 * @param near_dist_th: 近邻距离阈值向量
 * @param trail_Param: 算法阈值参数
 * @return: 垂直轨迹判断结果向量（1=是轨迹，0=非轨迹）
 */
dvshortx dv_VerTrailRemove(dvshortx dist_trail, dvshortx dist_longit[9],
                          dvshortx wall_judge, dvshortx near_cnt_h,
                          dvshortx near_dist_th, const TrailParam_t *trail_Param) {
    // 计算垂直邻域与当前点的差异绝对值（处理0值无效点）
    dvshortx dif_distC_ver_abs[9];
    for (int i = 0; i < 9; i++) {
        dvshortx zero_flag = (dist_longit[i] == 0);  // 标记0值
        // 无效点差异绝对值设为65535，有效点取邻域 - 当前
        dif_distC_ver_abs[i] = dvmux(zero_flag, 65535, dvabsdif(dist_longit[i], dist_trail));
    }

    // 统计垂直近邻数和垂直计数
    dvshortx near_cnt_v;// 近邻数（差异 < near_dist_th）
    near_cnt_v.lo = replicateh(0);
    near_cnt_v.hi = replicateh(0);
    dvshortx ver_cnt;// 垂直计数（差异 < 2*SlopDifThre）
    ver_cnt.lo = replicateh(0);
    ver_cnt.hi = replicateh(0);
    for (int i = 0; i < 9; i++) {
        near_cnt_v += (dif_distC_ver_abs[i] < near_dist_th);
        ver_cnt += (dif_distC_ver_abs[i] < (trail_Param->SlopDifThre * 2));
    }

    // 垂直轨迹判断条件
    dvshortx con3 = (near_cnt_h < trail_Param->near_cnt_th_h) &
                   (near_cnt_v < trail_Param->near_cnt_th_v);  // 近邻数不足
    dvshortx con4 = wall_judge | (ver_cnt < 3);  // 墙标记或垂直计数不足

    return con3 & con4;  // 垂直轨迹判断结果
}

/**
 * 执行函数：为每个瓦片执行轨迹移除计算（核心向量处理逻辑）
 * @param trail_Param: 算法阈值参数
 * @param config: 初始化好的配置结构体（含AGEN配置）
 */
void trail_remove_exec(TrailParam_t *trail_Param, TrailConfig_t *config) {
    // 从配置初始化地址生成器
    agen input_dist_agen = init_agen_from_cfg(config->input_dist);
    agen input_longit_agen = init_agen_from_cfg(config->input_longit);
    agen output_valid_agen = init_agen_from_cfg(config->output_valid);
    int32_t niter = config->niter;

    // 提取算法阈值参数
    const int32_t near_threshold = trail_Param->near_cnt_th_h;
    const int32_t bypass_distance = trail_Param->BypassDis;
    int32_t dist_th_ratio = (trail_Param->dist_th_ratio) << 4;  // 左移4位（×16）
    // 主循环：按向量批次并行处理
    for (int32_t i = 0; i < niter; i++) chess_prepare_for_pipelining  // 流水线优化
    chess_loop_range(6, )                                            // 循环范围提示
    chess_unroll_loop(2) {                                           // 循环展开2次
        // 1. 加载水平5邻域距离数据（col_idx-2 到 col_idx+2）
        // dvshortx dist_tmp[5];
        dist_tmp[0] = dvushort_load(input_dist_agen);  // col_idx - 2
        dist_tmp[1] = dvushort_load(input_dist_agen);  // col_idx - 1
        dist_tmp[2] = dvushort_load(input_dist_agen);  // center（当前点）
        dist_tmp[3] = dvushort_load(input_dist_agen);  // col_idx + 1
        dist_tmp[4] = dvushort_load(input_dist_agen);  // col_idx + 2

        // 2. 过滤无效点（距离<=0 或 >=bypass_distance）
        dvshortx valid_flag = (dist_tmp[2] > 0) &
                             (dist_tmp[2] < bypass_distance);
        // if (valid_flag) continue;  // 全无效则跳过当前向量

        // 3. 计算近邻距离阈值（自适应调整）
        dvshortx near_dist_th = dvmulh(dist_tmp[2], dist_th_ratio, VPU_TRUNC_7);  // 右移7位（÷128）
        near_dist_th = dvmax(near_dist_th, 2);   // 阈值下限：2
        near_dist_th = dvmin(near_dist_th, 20);  // 阈值上限：20

        // 4. 计算水平近邻计数（相邻差异 < 阈值的次数）
        vshortx v_near_cnt_h = replicateh(0);
        dvshortx near_cnt_h = deposit_lo(v_near_cnt_h);
        near_cnt_h += (dvabsdif(dist_tmp[1], dist_tmp[0]) < near_dist_th);
        near_cnt_h += (dvabsdif(dist_tmp[2], dist_tmp[1]) < near_dist_th);
        near_cnt_h += (dvabsdif(dist_tmp[3], dist_tmp[2]) < near_dist_th);
        near_cnt_h += (dvabsdif(dist_tmp[4], dist_tmp[3]) < near_dist_th);

        // 5. 轨迹参考掩码判断（近邻计数 >= 阈值则跳过）
        dvshortx trail_refer_mask = (near_cnt_h >= near_threshold);
        dvshortx process_flag = valid_flag & ~trail_refer_mask;  // 需要继续处理的像素
        // if (process_flag) continue;

        // 6. 水平轨迹判断
        dvshortx hortrail_judge = dv_HorTrailRemove(dist_tmp, dist_tmp[2], trail_Param);
        process_flag &= hortrail_judge;  // 仅保留水平轨迹判断为1的像素
        // if (process_flag) continue;

        // 7. 加载垂直9邻域距离数据（row_idx-4 到 row_idx+4）
        dvshortx dist_longit[9];
        for (int j = 0; j < 9; j++) {
            dist_longit[j] = dvushort_load(input_longit_agen);
        }

        // 8. 计算墙判断标志
        dvshortx dif2_dist_abs[3];  // 二阶差异绝对值
        dif2_dist_abs[0] = dvabsdif(dvabsdif(dist_tmp[1], dist_tmp[0]),
                                   dvabsdif(dist_tmp[2], dist_tmp[1]));
        dif2_dist_abs[1] = dvabsdif(dvabsdif(dist_tmp[2], dist_tmp[1]),
                                   dvabsdif(dist_tmp[3], dist_tmp[2]));
        dif2_dist_abs[2] = dvabsdif(dvabsdif(dist_tmp[3], dist_tmp[2]),
                                   dvabsdif(dist_tmp[4], dist_tmp[3]));
        dvshortx sum_dif2_dist_abs = dvadd3(dif2_dist_abs[0], dif2_dist_abs[1], dif2_dist_abs[2]);
        dvshortx wall_judge = (sum_dif2_dist_abs > trail_Param->SlopDifThre) &
                             (dif2_dist_abs[0] > trail_Param->SlopDifThre);

        // 9. 垂直轨迹判断
        dvshortx vertrail_judge = dv_VerTrailRemove(dist_tmp[2], dist_longit,
                                                  wall_judge, near_cnt_h,
                                                  near_dist_th, trail_Param);

        // 10. 输出结果：有效轨迹点置1
        dvshortx result = vertrail_judge & process_flag;
        vstore(result, output_valid_agen);
    }
}

/**
 * PVA主函数：协调瓦片处理流程
 */
CUPVA_VPU_MAIN() {
    // 获取算法参数
    TrailParam_t *trail_Param = (TrailParam_t *)algorithmParams;
    TrailConfig_t config;  // 配置结构体

    // 获取输入/输出行间距
    uint16_t srcDistLinePitch = cupvaRasterDataFlowGetLinePitch(sourceDistDataFlowHandler); //192
    uint16_t dstLinePitch = cupvaRasterDataFlowGetLinePitch(destinationDataFlowHandler);    //192

    int32_t srcDistOffset = 0;
    int32_t dstOffset = 0;
    // 初始化地址生成器和配置参数
    trail_remove_init(inputDistBufferVMEM, outputValidBufferVMEM,
                     srcDistLinePitch, dstLinePitch, &config);

    // 触发输入数据流传输
    cupvaRasterDataFlowOpen(destinationDataFlowHandler, outputValidBufferVMEM);
    cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);

    // 循环处理所有瓦片
    for (int TileIdx = 0; TileIdx < TILE_COUNT; TileIdx++)
    {
        cupvaRasterDataFlowSync(sourceDistDataFlowHandler);
        cupvaRasterDataFlowTrig(sourceDistDataFlowHandler);
        void *pDstTile = cupvaRasterDataFlowAcquire(destinationDataFlowHandler);
        /** Update agen base address */
        cupvaModifyAgenCfgBase(&config.input_dist, inputDistBufferVMEM + srcDistOffset + KERNEL_RADIUS_WIDTH);
        cupvaModifyAgenCfgBase(&config.input_longit, inputDistBufferVMEM + srcDistOffset + KERNEL_RADIUS_HEIGHT * srcDistLinePitch);
        cupvaModifyAgenCfgBase(&config.output_valid, pDstTile);

        // 执行当前瓦片的轨迹移除计算
        trail_remove_exec(trail_Param, &config);

        srcDistOffset = cupvaRasterDataFlowGetOffset(sourceDistDataFlowHandler, srcDistOffset);

        // 同步输出数据流并触发下一个瓦片的输出
        cupvaRasterDataFlowRelease(destinationDataFlowHandler);
    }

    // 等待最后一个瓦片的输出完成
    cupvaRasterDataFlowClose(destinationDataFlowHandler);
    return 0;
}
