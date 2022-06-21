//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Rate.h
//
// Code generation for function 'Rate'
//

#ifndef RATE_H
#define RATE_H

// Include files
#include "rtwtypes.h"
#include "coder_posix_time.h"
#include "mlroscpp_rate.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Rate {
public:
  Rate *init();
  std::unique_ptr<MATLABRate> RateHelper;
  double DesiredRate;
  coderTimespec PreviousPeriod;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (Rate.h)
