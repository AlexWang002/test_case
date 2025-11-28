#ifndef HIGHCALC_COMMON_PARAMS_H
#define HIGHCALC_COMMON_PARAMS_H

#include <stdint.h>
#include <math.h>

/** [params] */
#define ALGO_ON

#define TILE_WIDTH            192
#define TILE_HEIGHT           40
#define TILE_COUNT            19

#define VIEW_WIDTH            192
#define VIEW_HEIGHT           760

#define KERNEL_RADIUS_WIDTH   2
#define KERNEL_RADIUS_HEIGHT  0

#define RAIN_SIZE_X           5
#define RAIN_SIZE_Y           3
#define CENTER_X              2
#define CENTER_Y              1

#define UP_VIEW_HEIGHT        1520

#define VSHITFT_R1            (-1)
#define VSHITFT_R2            (-2)
#define VSHITFT_R7            (-7)
#define VSHITFT_R9            (-9)

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(a) ((a) > 0 ? (a) : -(a))

typedef struct{
    int dist_diff_th;
    int delta_z_th;
    int z_th;
    int dist_max_th;
}HighcalcParam_t;

#define DEFAULT_HIGH_PARAM { \
    .dist_diff_th = 600,\
    .delta_z_th = 20,\
    .z_th = 300,\
    .dist_max_th = 16000\
}

#endif
