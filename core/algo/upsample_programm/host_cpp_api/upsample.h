/*******************************************************************************
 * \addtogroup upsample_programm
 * \{
 * \headerfile upsample.h "upsample.h"
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
#ifndef UPSAMPLE_H
#define UPSAMPLE_H
/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> // Main host-side C++-API header file
#include <cupva_platform.h> // Header that

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../upsample_commom_param.h"

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern uint16_t *DistDownIn_d;
extern uint16_t *DistDownIn_h;
extern uint16_t *DistRawIn_d;
extern uint16_t *DistRawIn_h;
extern uint16_t *RefDownIn_d;
extern uint16_t *RefDownIn_h;
extern uint16_t *RefRawIn_d;
extern uint16_t *RefRawIn_h;
extern uint16_t *AttrIn_d;
extern uint16_t *AttrIn_h;

extern uint16_t *DistOutUp_d;
extern uint16_t *DistOutUp_h;
extern uint16_t *RefOutUp_d;
extern uint16_t *RefOutUp_h;
extern uint16_t *AttrOutUp_d;
extern uint16_t *AttrOutUp_h;
/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern void upsampleDataAlloc();
extern void upsampleDataFree();
extern void upsample_main();

#endif/* UPSAMPLE_H */