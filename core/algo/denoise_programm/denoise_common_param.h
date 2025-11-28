#ifndef DENOISE_COMMON_PARAM_H
#define DENOISE_COMMON_PARAM_H

#include <stdint.h>
#include <math.h>

#define TILE_WIDTH      192
#define TILE_HEIGHT     95
#define VIEW_HEIGHT     760
#define KERNEL_RADIUS_WIDTH     1
#define KERNEL_RADIUS_HEIGHT    2
#define TILE_CNT 8

#define ALGO_ENABLE 1 // 算法开关

typedef struct {
    int win_len_h;  // 水平方向半窗长
    int win_len_v;  // 垂直方向半窗长
    int zone_matrix[3][5];
    int region_list[8]; // region_list (8个元素的一维数组)
    int denoise_offset[4]; // denoise_offset (4个元素的一维数组)
    int denoise_dist[3]; // denoise_dist (3个元素的一维数组)
    int denoise_point[4][4][3]; // denoise_point (4个3x3矩阵组成的三维数组)
}NoiseParam_t;

#define DEFAULT_DENOISE_PARAM { \
    .win_len_h = 2, \
    .win_len_v = 1, \
    .zone_matrix = {{0, 1, 1, 1, 0},{3, 2, 0, 2, 3},{0, 1, 1, 1, 0}}, \
    .region_list = { 1, 1, 2, 2, 3, 3, 4, 4 }, \
    .denoise_offset = { 128, 200, 200, 300 }, \
    .denoise_dist = { 3000, 12000, 24000 }, \
    .denoise_point = {{{1, 0, 2},{1, 0, 2},{1, 0, 2},{1, 0, 2}},{{1, 1, 2},{1, 1, 2},{1, 1, 2},{1, 1, 2}},{{1, 1, 2},{1, 1, 2},{1, 1, 2},{1, 1, 2}},{{1, 1, 2},{1, 1, 2},{1, 1, 2},{1, 1, 2}}} \
}


#endif
