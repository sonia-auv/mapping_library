//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_proc_mapping_api.h
//
// Code generation for function 'proc_mapping'
//

#ifndef _CODER_PROC_MAPPING_API_H
#define _CODER_PROC_MAPPING_API_H

// Include files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void proc_mapping();

void proc_mapping_api();

void proc_mapping_atexit();

void proc_mapping_initialize();

void proc_mapping_terminate();

void proc_mapping_xil_shutdown();

void proc_mapping_xil_terminate();

#endif
// End of code generation (_coder_proc_mapping_api.h)
