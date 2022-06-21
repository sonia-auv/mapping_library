//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping_data.h
//
// Code generation for function 'proc_mapping_data'
//

#ifndef PROC_MAPPING_DATA_H
#define PROC_MAPPING_DATA_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Variable Declarations
extern double freq;
extern bool freq_not_empty;
extern bool newSonarMsg;
extern bool bundleStarted;
extern double minIntensityValue;
extern double maxIntensityValue;
extern double minRangeValue;
extern double maxRangeValue;
extern unsigned int method;
extern unsigned int state;
extern unsigned int b_state[2];
extern unsigned int c_state[625];
extern const char cv[9];
extern bool isInitialized_proc_mapping;

#endif
// End of code generation (proc_mapping_data.h)
