#ifndef SPRAY_COMMON_PARAMS_H
#define SPRAY_COMMON_PARAMS_H

#include <stdint.h>
#include <math.h>

/** [params] */
#define ALGO_ON

#define TILE_WIDTH            192
#define TILE_HEIGHT           20
#define TILE_COUNT            38

#define VIEW_WIDTH            192
#define VIEW_HEIGHT           760

#define KERNEL_RADIUS_WIDTH   1
#define KERNEL_RADIUS_HEIGHT  2

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
    int Area_size_x;
    int Area_size_y;

    int zone_idx_min_a;
    int zone_idx_min_b;
    int zone_idx_max_a;
    int zone_idx_max_b;

    int Ref_threshold;
    int Ref_threshold2;

    int Dist_threshold;
    int Dist_threshold2;
    int Dist_threshold_min;

    int threshold_var;
    int threshold_var2;

    int min_mark_threshold;
    int min_tail_threshold;
    int min_split_threshold;

    uint16_t dist_diff_thr_seq[16];
    uint16_t dist_diff_diff_thr_seq[16];

    int dist_diff_thr_2;
    int dist_diff_thr_fix;

    int inv_table[18];

    uint16_t consistency_en;

    int dist_cap_zone[16][2];
    int ground_cap_zone[16][2];

    int dist_diff_thr;
    int filter_dist_max;
    int filter_ref_max;
    int filter_count_thr;

    uint16_t head_tail_judge_en;
    uint16_t rain_mark_en;
    uint16_t local_var_en;
    uint16_t ground_judge_en;
    uint16_t ground_link_judge_en;
}SprayParam_t;

#define DEFAULT_SPRAY_PARAM { \
    .Area_size_x = 2,\
    .Area_size_y = 1,\
    .zone_idx_min_a = 1,\
    .zone_idx_min_b = 1,\
    .zone_idx_max_a = 11,\
    .zone_idx_max_b = 15,\
    .Ref_threshold = 10,\
    .Ref_threshold2 = 8,\
    .Dist_threshold = 1600,\
    .Dist_threshold2 = 6000,\
    .Dist_threshold_min = 0,\
    .threshold_var = 2000,\
    .threshold_var2 = 2000,\
    .min_mark_threshold = 4,\
    .min_tail_threshold = 2,\
    .min_split_threshold = 3,\
    .dist_diff_thr_seq = {40,57,75 ,92,109,127,144,161,179,196,213,231,248,265,283,300},\
    .dist_diff_diff_thr_seq = {40,48,56,64,72,80,88,96,104,112,120,128,136,144,152,160},\
    .dist_diff_thr_2 = 400,\
    .dist_diff_thr_fix = 600,\
    .inv_table = {4096, 2048, 1365, 1024, 819, 683,585, 512, 455, 410, 372, 341,315, 293, 273, 256, 241, 228},\
    .consistency_en = 1,\
    .dist_cap_zone = { 0 },\
    .ground_cap_zone = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },\
    .dist_diff_thr = 200,\
    .filter_dist_max = 5000,\
    .filter_ref_max = 8,\
    .filter_count_thr = 5,\
    .head_tail_judge_en = 1,\
    .rain_mark_en = 1,\
    .local_var_en = 1,\
    .ground_judge_en = 1,\
    .ground_link_judge_en = 1\
}

#endif
