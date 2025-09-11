#ifndef TRAIL_H
#define TRAIL_H

#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> // Main host-side C++-API header file
#include <cupva_platform.h>
#include "../trail_common_param.h"

extern uint16_t *DistIn_d;
extern uint16_t *DistIn_h;

extern uint8_t *ValidOut_d;
extern uint8_t *ValidOut_h;

extern uint8_t TrailMask[VIEW_HEIGHT][VIEW_WIDTH];

extern void trail_main();
extern void TrailDataAlloc();
extern void TrailDataFree();
#endif