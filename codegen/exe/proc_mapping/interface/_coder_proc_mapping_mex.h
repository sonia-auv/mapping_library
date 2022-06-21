//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_proc_mapping_mex.h
//
// Code generation for function 'proc_mapping'
//

#ifndef _CODER_PROC_MAPPING_MEX_H
#define _CODER_PROC_MAPPING_MEX_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_proc_mapping_mexFunction(int32_T nlhs, int32_T nrhs);

#endif
// End of code generation (_coder_proc_mapping_mex.h)
