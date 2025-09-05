#ifndef TRAIL_COMMON_PARAMS_H
#define TRAIL_COMMON_PARAMS_H

#include <stdint.h>
#include <math.h>

/** [params] */
#define ALGO_ON

#define TILE_WIDTH    192
#define TILE_HEIGHT   95

#define VIEW_WIDTH    192
#define VIEW_HEIGHT   760

#define KERNEL_RADIUS_WIDTH   0
#define KERNEL_RADIUS_HEIGHT  2

#define TILE_COUNT    8
#define UP_VIEW_HEIGHT   1520

#define DIS_THRE_RATIO_VALUE (sin(0.2 / 180 * 3.141592657) / tan(0.2))

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(a) ((a) > 0 ? (a) : -(a))

typedef struct{
    int D_H; //拖点最大距离差
    int SlopDifThre;
    double dist_th_ratio;
    double DisThreRatio;
    int BypassDis;
    int near_cnt_th_h;
    int near_cnt_th_v;
}TrailParam_t;
#endif
