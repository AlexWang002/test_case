/*******************************************************************************
 * \addtogroup spray_programm
 * \{
 * \headerfile spray.h "spray.h"
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
#ifndef SPRAY_H
#define SPRAY_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp>
#include <cupva_platform.h>
#include <string>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "../spray_common_param.h"

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern uint16_t *DistIn0_h;
extern uint16_t *DistIn1_h;
extern uint16_t *RefIn0_h;
extern uint16_t *RefIn1_h;
extern uint16_t *GndIn0_h;
extern uint16_t *GndIn1_h;
extern uint16_t *AttIn0_h;
extern uint16_t *AttIn1_h;
extern uint16_t *Glink_h;

extern uint16_t *RainOut0_h;
extern uint16_t *RainOut1_h;

extern uint16_t *FinalOut0_h;
extern uint16_t *FinalOut1_h;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern int sprayRemovePva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time);
extern int rainEnhancePva(std::string& exception_msg, int32_t& status_code,
    uint32_t& submit_time, uint32_t& wait_time);
extern void sprayDataAlloc();
extern void sprayDataFree();

#endif /* SPRAY_H */
