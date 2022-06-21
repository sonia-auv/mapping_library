//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Preprocessing.cpp
//
// Code generation for function 'Preprocessing'
//

// Include files
#include "Preprocessing.h"
#include "any1.h"
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
void Preprocessing::persistentDataStore_init()
{
  minIntensityValue = 0.07;
  //                  minIntensityState = false;
  maxIntensityValue = 1.0;
  //                  maxIntensityState = false;
  minRangeValue = 0.1;
  //                  minRangeState = false;
  maxRangeValue = 100.0;
  //                  maxRangeState = false;
}

void b_binary_expand_op(coder::array<double, 1U> &in1,
                        const coder::array<double, 1U> &in2,
                        const coder::array<double, 1U> &in3)
{
  coder::array<double, 1U> b_in1;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int stride_2_0;
  if (in3.size(0) == 1) {
    if (in2.size(0) == 1) {
      i = in1.size(0);
    } else {
      i = in2.size(0);
    }
  } else {
    i = in3.size(0);
  }
  b_in1.set_size(i);
  stride_0_0 = (in1.size(0) != 1);
  stride_1_0 = (in2.size(0) != 1);
  stride_2_0 = (in3.size(0) != 1);
  if (in3.size(0) == 1) {
    if (in2.size(0) == 1) {
      loop_ub = in1.size(0);
    } else {
      loop_ub = in2.size(0);
    }
  } else {
    loop_ub = in3.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    b_in1[i] =
        (in1[i * stride_0_0] + in2[i * stride_1_0]) + in3[i * stride_2_0];
  }
  in1.set_size(b_in1.size(0));
  loop_ub = b_in1.size(0);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in1[i];
  }
}

void binary_expand_op(coder::array<bool, 1U> &in1,
                      const coder::array<double, 2U> &in2, double in3,
                      double in4, const coder::array<double, 1U> &in5,
                      double in6, const coder::array<double, 1U> &in7,
                      double in8)
{
  coder::array<bool, 1U> b_in2;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int stride_2_0;
  i = in2.size(0);
  if (in7.size(0) == 1) {
    if (in5.size(0) == 1) {
      stride_0_0 = i;
    } else {
      stride_0_0 = in5.size(0);
    }
  } else {
    stride_0_0 = in7.size(0);
  }
  b_in2.set_size(stride_0_0);
  stride_0_0 = (i != 1);
  stride_1_0 = (in5.size(0) != 1);
  stride_2_0 = (in7.size(0) != 1);
  if (in7.size(0) == 1) {
    if (in5.size(0) == 1) {
      loop_ub = i;
    } else {
      loop_ub = in5.size(0);
    }
  } else {
    loop_ub = in7.size(0);
  }
  for (i = 0; i < loop_ub; i++) {
    double in2_tmp;
    in2_tmp = in2[i * stride_0_0 + in2.size(0) * 3];
    b_in2[i] = ((in2_tmp < in3) || (in2_tmp > in4) ||
                (in5[i * stride_1_0] < in6) || (in7[i * stride_2_0] > in8));
  }
  coder::any(b_in2, in1);
}

// End of code generation (Preprocessing.cpp)
