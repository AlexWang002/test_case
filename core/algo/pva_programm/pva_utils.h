/*******************************************************************************
 * \addtogroup denoise_programm
 * \{
 * \headerfile denoise.h "denoise.h"
 * \brief
 * \version 0.3
 * \date 2025-11-28
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
#ifndef PVA_UTILS_H
#define PVA_UTILS_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> // Main host-side C++-API header file
#include <cupva_platform.h> // Header that includes macros for specifying PVA executables
#include <iostream>
#include <fstream>
#include <string>

using namespace cupva;

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/


/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/


/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern Stream& getAlgoStream();
#endif /* PVA_UTILS_H */
