//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping_data.cpp
//
// Code generation for function 'proc_mapping_data'
//

// Include files
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include <string.h>

// Variable Definitions
double freq;

bool freq_not_empty;

bool newSonarMsg;

bool bundleStarted;

double minIntensityValue;

double maxIntensityValue;

double minRangeValue;

double maxRangeValue;

unsigned int method;

unsigned int state;

unsigned int b_state[2];

unsigned int c_state[625];

const char cv[9]{'i', 'n', 't', 'e', 'n', 's', 'i', 't', 'y'};

bool isInitialized_proc_mapping{false};

// End of code generation (proc_mapping_data.cpp)
