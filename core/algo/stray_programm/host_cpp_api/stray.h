/*******************************************************************************
 * \addtogroup stray_programm
 * \{
 * \headerfile stray.h "stray.h"
 * \brief
 * \version 0.2
 * \date 2025-09-29
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-10-13 | Init version |
 *
 ******************************************************************************/
#ifndef STRAY_H
#define STRAY_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> // Main host-side C++-API header file
#include <cupva_platform.h> // Header that includes macros for specifying PVA executables
#include <iostream>
#include <fstream>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../stray_common_param.h"

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/

struct StrayPvaBuffer {
    uint16_t *dist_wave_d;
    uint16_t *dist_wave_h;

    uint16_t *att0_d;
    uint16_t *att0_h;

    uint16_t *att1_d;
    uint16_t *att1_h;

    uint16_t *class_line_d;
    uint16_t *class_line_h;

    uint16_t *ground_height_d;
    uint16_t *ground_height_h;

    uint16_t *raw_data_d;
    uint16_t *raw_data_h;

    uint16_t *stray_var_d;
    uint16_t *stray_var_h;

    uint16_t *stray_mask0_d;
    uint16_t *stray_mask0_h;

    uint16_t *stray_mask1_d;
    uint16_t *stray_mask1_h;
};

extern StrayPvaBuffer stray_pva_buff;

extern int strayBufferAlloc();
extern int strayBufferRelease();
extern int strayProcPva(int rainwall_cnt, int rainwall_dist);

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/

#endif /* STRAY_H */
