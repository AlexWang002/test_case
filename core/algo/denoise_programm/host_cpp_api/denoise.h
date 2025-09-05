#ifndef DENOISE_H
#define DENOISE_H

#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> // Main host-side C++-API header file
#include <cupva_platform.h> // Header that includes macros for specifying PVA executables

#include <iostream>
#include <fstream>
extern uint16_t *denoise_dist_buffer_d; 
extern uint16_t *denoise_dist_buffer_h;

extern int *denoise_mask_buffer_d;
extern int *denoise_mask_buffer_h;

int denoiseProcPva();
int denoiseDataAlloc();
int denoiseDataFree();

#endif
