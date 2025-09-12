/*******************************************************************************
 * \addtogroup denoise_programm
 * \{
 * \headerfile denoise.h "denoise.h"
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
#ifndef DENOISE_H
#define DENOISE_H

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
#include "../denoise_common_param.h"

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern uint16_t *denoise_dist_buffer_d; 
extern uint16_t *denoise_dist_buffer_h;

extern int *denoise_mask_buffer_d;
extern int *denoise_mask_buffer_h;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
int denoiseProcPva();
int denoiseDataAlloc();
int denoiseDataFree();

#endif /* DENOISE_H */
