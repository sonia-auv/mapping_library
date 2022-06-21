//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// any1.cpp
//
// Code generation for function 'any1'
//

// Include files
#include "any1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
namespace coder {
void any(const ::coder::array<bool, 1U> &x, ::coder::array<bool, 1U> &y)
{
  int i1;
  int i2;
  int vstride;
  y.set_size(x.size(0));
  vstride = x.size(0);
  for (i1 = 0; i1 < vstride; i1++) {
    y[i1] = false;
  }
  vstride = x.size(0);
  i1 = 0;
  i2 = 0;
  for (int j{0}; j < vstride; j++) {
    int ix;
    bool exitg1;
    i1++;
    i2++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && ((vstride > 0) && (ix <= i2))) {
      if (x[ix - 1]) {
        y[j] = true;
        exitg1 = true;
      } else {
        ix += vstride;
      }
    }
  }
}

void b_any(const ::coder::array<bool, 2U> &x, ::coder::array<bool, 1U> &y)
{
  int i1;
  int i2;
  int vstride;
  y.set_size(x.size(0));
  vstride = x.size(0);
  for (i2 = 0; i2 < vstride; i2++) {
    y[i2] = false;
  }
  vstride = x.size(0);
  i2 = x.size(0) << 1;
  i1 = 0;
  for (int j{0}; j < vstride; j++) {
    int ix;
    bool exitg1;
    i1++;
    i2++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && ((vstride > 0) && (ix <= i2))) {
      if (x[ix - 1]) {
        y[j] = true;
        exitg1 = true;
      } else {
        ix += vstride;
      }
    }
  }
}

} // namespace coder

// End of code generation (any1.cpp)
