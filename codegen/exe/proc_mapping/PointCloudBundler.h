//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PointCloudBundler.h
//
// Code generation for function 'PointCloudBundler'
//

#ifndef POINTCLOUDBUNDLER_H
#define POINTCLOUDBUNDLER_H

// Include files
#include "Preprocessing.h"
#include "Subscriber.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class PointCloudBundler {
public:
  static void persistentDataStore_init();
  coder::array<double, 2U> bundle;
  Preprocessing mPreprocessing;
  coder::ros::c_Subscriber *startStopSub;
  coder::ros::d_Subscriber *poseSub;
  coder::ros::e_Subscriber *sonarSub;
  bool lastBundleState;
  coder::ros::e_Subscriber _pobj0;
  coder::ros::d_Subscriber _pobj1;
  coder::ros::c_Subscriber _pobj2;
};

#endif
// End of code generation (PointCloudBundler.h)
