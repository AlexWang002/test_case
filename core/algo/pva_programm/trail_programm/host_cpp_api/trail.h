/*******************************************************************************
 * \addtogroup trail_programm
 * \{
 * \headerfile trail.h "trail.h"
 * \brief
 * \version 0.2
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
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.2 | 2025-11-28 | Add exception log messages |
 ******************************************************************************/
#ifndef TRAIL_H
#define TRAIL_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp>
#include <cupva_platform.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../trail_common_param.h"

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern uint16_t *DistIn_d;
extern uint16_t *DistIn_h;

extern uint16_t *ValidOut_d;
extern uint16_t *ValidOut_h;

extern uint16_t TrailMask[VIEW_HEIGHT][VIEW_WIDTH];

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern int trailProcPva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time);
extern int trailDataAlloc();
extern int trailDataFree();

#endif /* TRAIL_H */
