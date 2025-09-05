#ifndef GROUNDFIT_H
#define GROUNDFIT_H

#include <cupva_host_nonsafety.hpp>
#include <cupva_host.hpp> // Main host-side C++-API header file
#include <cupva_platform.h> // Header that includes macros for specifying PVA executables

#include <iostream>
#include <fstream>

int GroundfitProcPva(int subframe_id);
void GroundfitAllocPva();
void GroundfitFreePva();

#endif
