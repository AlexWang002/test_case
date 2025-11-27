/*******************************************************************************
 * \addtogroup denoise_programm
 * \{
 * \headerfile denoise.h "denoise.h"
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
 * | 0.1 | 2025-09-11 | Init version |
 * 
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-09-29 | Use vpu's Wide-SIMD vector processor to accelerate denoise algo|
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
#include <string>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../denoise_common_param.h"

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern uint16_t *denoise_dist_buffer_d; 
extern uint16_t *denoise_dist_buffer_h;

extern uint16_t *denoise_mask_buffer_d;
extern uint16_t *denoise_mask_buffer_h;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
int denoiseProcPva(std::string& exception_msg, int32_t& status_code);
int denoiseDataAlloc();
int denoiseDataFree();

#endif /* DENOISE_H */
