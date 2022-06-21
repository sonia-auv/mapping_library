//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Rate.cpp
//
// Code generation for function 'Rate'
//

// Include files
#include "Rate.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "coder_posix_time.h"
#include "mlroscpp_rate.h"
#include "ros/ros.h"
#include <string.h>

// Function Definitions
namespace coder {
namespace ros {
Rate *Rate::init()
{
  Rate *obj;
  double b_expl_temp;
  double expl_temp;
  obj = this;
  obj->RateHelper = MATLABRate_create(20.0);
  MATLABRate_unused(&obj->RateHelper);
  obj->DesiredRate = 20.0;
  tic(&expl_temp, &b_expl_temp);
  tic(&obj->PreviousPeriod.tv_sec, &obj->PreviousPeriod.tv_nsec);
  return obj;
}

} // namespace ros
} // namespace coder

// End of code generation (Rate.cpp)
