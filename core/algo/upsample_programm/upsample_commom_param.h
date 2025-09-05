/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef TRAIL_COMMON_PARAMS_H
#define TRAIL_COMMON_PARAMS_H

#include <stdint.h>
#include <math.h>

/** [params] */
#define ALGO_ON

#define TILE_WIDTH           192
#define TILE_HEIGHT          76
#define UP_TILE_HEIGHT       76
#define VIEW_WIDTH           192
#define VIEW_HEIGHT          760
#define UP_VIEW_HEIGHT       1520
#define KERNEL_RADIUS_WIDTH  0
#define KERNEL_RADIUS_HEIGHT 1
#define TILE_COUNT           10

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(a) ((a) > 0 ? (a) : -(a))

typedef struct{
    // 整型参数
    int WH;
    int diff_th;
    int diff_ratio_max; // 调整后的垂直分辨率系数
    int diff2_th;
    int weight1[3];
    int weight2[3];
    int coef_inv[10];
}InsertParam_t;

#define DEFAULT_UP_PARAM { \
    .WH = 2, \
    .diff_th = 100, \
    .diff_ratio_max = 256, \
    .diff2_th = 100, \
    .weight1 = {1,2,1}, \
    .weight2 = {1,2,1}, \
    {4096,2048,1365,1024,819,683,585,512,455,410} \
}
#endif
