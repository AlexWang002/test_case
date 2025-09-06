// /*
//  * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
//  *
//  * NVIDIA CORPORATION and its licensors retain all intellectual property
//  * and proprietary rights in and to this software, related documentation
//  * and any modifications thereto.  Any use, reproduction, disclosure or
//  * distribution of this software and related documentation without an express
//  * license agreement from NVIDIA CORPORATION is strictly prohibited.
//  */

// #ifndef GROUNDFIT_COMMON_PARAM_H
// #define GROUNDFIT_COMMON_PARAM_H

// #include <stdint.h>
// #include <math.h>

// /** [params] */
// #define TILE_WIDTH      192
// #define TILE_HEIGHT     19

// #define VIEW_WIDTH      192
// #define VIEW_HEIGHT     152

// #define TILE_CNT        8

// #define KERNEL_WIDTH    5
// #define KERNEL_RADIUS   (5 >> 1)

// #define VALID_HEIGHT    126
// #define GND_VIEW_W      172
// //#define GND_ORI_LEN     (VIEW_WIDTH * GND_VIEW_W)
// #define GND_ORI_LEN     10
// #define GND_LEN         (VIEW_WIDTH - 1) * GND_VIEW_W
// #define GND_STEP        5

// #define BlockSize                  24    // 数据分块大小
// #define buffer_size_denoise        5     // 数据缓存，去噪平滑
// #define denoise_valid_size         5     // 标记缓存，去噪

// #define DEBUG 0

// struct Matrix3x3 {
//     float data[9]; // 按行优先存储: [0,1,2; 3,4,5; 6,7,8]
    
//     float& operator()(int row, int col) {
//         return data[row * 3 + col];
//     }
    
//     const float& operator()(int row, int col) const {
//         return data[row * 3 + col];
//     }
// };

// struct Vector3 {
//     float data[3];
    
//     float& operator[](int idx) { return data[idx]; }
//     const float& operator[](int idx) const { return data[idx]; }
// };
// /** [params] */

// #endif // GROUNDFIT_COMMON_PARAM_H
