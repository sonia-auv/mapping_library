//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Preprocessing.h
//
// Code generation for function 'Preprocessing'
//

#ifndef PREPROCESSING_H
#define PREPROCESSING_H

// Include files
#include "Subscriber.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class Preprocessing {
public:
  static void persistentDataStore_init();
  coder::ros::Subscriber *minIntensitySub;
  coder::ros::Subscriber *maxIntensitySub;
  coder::ros::b_Subscriber *minRangeSub;
  coder::ros::b_Subscriber *maxRangeSub;
  coder::ros::b_Subscriber _pobj0;
  coder::ros::b_Subscriber _pobj1;
  coder::ros::Subscriber _pobj2;
  coder::ros::Subscriber _pobj3;
};

// Function Declarations
void b_binary_expand_op(coder::array<double, 1U> &in1,
                        const coder::array<double, 1U> &in2,
                        const coder::array<double, 1U> &in3);

void binary_expand_op(coder::array<bool, 1U> &in1,
                      const coder::array<double, 2U> &in2, double in3,
                      double in4, const coder::array<double, 1U> &in5,
                      double in6, const coder::array<double, 1U> &in7,
                      double in8);

#endif
// End of code generation (Preprocessing.h)
