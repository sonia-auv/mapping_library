//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_proc_mapping_mex.cpp
//
// Code generation for function 'proc_mapping'
//

// Include files
#include "_coder_proc_mapping_mex.h"
#include "_coder_proc_mapping_api.h"

// Function Definitions
void mexFunction(int32_T nlhs, mxArray *[], int32_T nrhs, const mxArray *[])
{
  mexAtExit(&proc_mapping_atexit);
  // Module initialization.
  proc_mapping_initialize();
  // Dispatch the entry-point.
  unsafe_proc_mapping_mexFunction(nlhs, nrhs);
  // Module termination.
  proc_mapping_terminate();
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, (const char_T *)"UTF-8", true);
  return emlrtRootTLSGlobal;
}

void unsafe_proc_mapping_mexFunction(int32_T nlhs, int32_T nrhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 0, 4,
                        12, "proc_mapping");
  }
  if (nlhs > 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "proc_mapping");
  }
  // Call the function.
  proc_mapping_api();
}

// End of code generation (_coder_proc_mapping_mex.cpp)
