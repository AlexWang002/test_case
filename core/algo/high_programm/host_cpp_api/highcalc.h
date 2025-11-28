/*******************************************************************************
 * \addtogroup highcalc_programm
 * \{
 * \headerfile highcalc.h "highcalc.h"
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
#ifndef HIGHCALC_H
#define HIGHCALC_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp>
#include <cupva_platform.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../highcalc_common_param.h"

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern uint16_t *h_dist_in0_h;
extern uint16_t *h_dist_in1_h;
extern uint16_t *h_high_in0_h;
extern uint16_t *h_high_in1_h;
extern uint16_t *h_proj_dist0_h;
extern uint16_t *h_proj_dist1_h;
extern uint16_t *h_gnd_out0_h;
extern uint16_t *h_gnd_out1_h;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern void highcalcPva();
extern void highcalcDataAlloc();
extern void highcalcDataFree();

#endif /* HIGHCALC_H */
