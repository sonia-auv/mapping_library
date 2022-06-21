//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping_internal_types.h
//
// Code generation for function 'proc_mapping'
//

#ifndef PROC_MAPPING_INTERNAL_TYPES_H
#define PROC_MAPPING_INTERNAL_TYPES_H

// Include files
#include "proc_mapping_types.h"
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct struct_T {
  int xstart;
  int xend;
};

struct cell_wrap_13 {
  coder::array<char, 2U> f1;
};

struct b_struct_T {
  coder::array<double, 1U> D;
  coder::array<unsigned int, 1U> b_I;
};

#endif
// End of code generation (proc_mapping_internal_types.h)
