#ifndef UPSAMPLE_H
#define UPSAMPLE_H

#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> // Main host-side C++-API header file
#include <cupva_platform.h> // Header that
#include "../upsample_commom_param.h"

extern uint16_t DistOutOri[VIEW_HEIGHT][VIEW_WIDTH];
extern uint16_t DistOutUp[VIEW_HEIGHT][VIEW_WIDTH];
extern uint8_t RefOutOri[VIEW_HEIGHT][VIEW_WIDTH];
extern uint8_t RefOutUp[VIEW_HEIGHT][VIEW_WIDTH];

extern uint16_t *DistDownIn_d;
extern uint16_t *DistDownIn_h;
extern uint16_t *DistRawIn_d;
extern uint16_t *DistRawIn_h;
extern uint8_t *RefDownIn_d;
extern uint8_t *RefDownIn_h;
extern uint8_t *RefRawIn_d;
extern uint8_t *RefRawIn_h;
extern uint16_t *DistOutOri_d;
extern uint16_t *DistOutOri_h;
extern uint16_t *DistOutUp_d;
extern uint16_t *DistOutUp_h;
extern uint8_t *RefOutOri_d;
extern uint8_t *RefOutOri_h;
extern uint8_t *RefOutUp_d;
extern uint8_t *RefOutUp_h;

extern void upsampleDataAlloc();
extern void upsampleDataFree();
extern void upsample_main();
#endif