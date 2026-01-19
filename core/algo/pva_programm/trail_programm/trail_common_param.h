#ifndef TRAIL_COMMON_PARAMS_H
#define TRAIL_COMMON_PARAMS_H

#include <stdint.h>
#include <math.h>

/** [params] */
#define VECW          32
#define TILE_WIDTH    96
#define TILE_HEIGHT   95

#define VIEW_WIDTH    192
#define VIEW_HEIGHT   760

#define KERNEL_RADIUS_WIDTH   4
#define KERNEL_RADIUS_HEIGHT  6

#define TILE_COUNT    16

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(a) ((a) > 0 ? (a) : -(a))

typedef struct{
    int D_H; //拖点最大距离差
    int Draw_D_H; //拖点最大距离差
    int SlopDifThre;
    int dist_th_ratio;
    int DisThreRatio;
    int BypassDis;
    int near_cnt_th_h;
    int near_cnt_th_v;
    int draw_near_cnt_th_h;
    int draw_near_cnt_th_v;
    int draw_dist_end;

    int left_coef[11];
    int right_coef[11];
}TrailParam_t;

#define DEFAULT_TRAIL_PARAM { \
    .D_H = 250, \
    .Draw_D_H = 150,\
    .SlopDifThre = 3, \
    .dist_th_ratio = 1, \
    .DisThreRatio = 71, \
    .BypassDis = 2400, \
    .near_cnt_th_h = 3, \
    .near_cnt_th_v = 5, \
    .draw_near_cnt_th_h = 1, \
    .draw_near_cnt_th_v = 6, \
    .draw_dist_end = 1400, \
    .left_coef = {1,1,1,1,1,1,0,0,0,0,0}, \
    .right_coef = {0,0,0,0,0,1,1,1,1,1,1} \
}

#endif
