//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping_rtwutil.h
//
// Code generation for function 'proc_mapping_rtwutil'
//

#ifndef PROC_MAPPING_RTWUTIL_H
#define PROC_MAPPING_RTWUTIL_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern unsigned int mul_u32_sat(unsigned int a, unsigned int b);

extern void mul_wide_u32(unsigned int in0, unsigned int in1,
                         unsigned int *ptrOutBitsHi,
                         unsigned int *ptrOutBitsLo);

extern double rt_powd_snf(double u0, double u1);

#endif
// End of code generation (proc_mapping_rtwutil.h)
