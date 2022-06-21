//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nullAssignment.cpp
//
// Code generation for function 'nullAssignment'
//

// Include files
#include "nullAssignment.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
namespace coder {
namespace internal {
void nullAssignment(::coder::array<double, 2U> &x,
                    const ::coder::array<bool, 1U> &idx)
{
  int b_i;
  int i;
  int nrows;
  int nrowx;
  nrowx = x.size(0);
  nrows = 0;
  i = idx.size(0);
  for (int k{0}; k < i; k++) {
    nrows += idx[k];
  }
  nrows = x.size(0) - nrows;
  b_i = 0;
  for (int k{0}; k < nrowx; k++) {
    if ((k + 1 > idx.size(0)) || (!idx[k])) {
      x[b_i] = x[k];
      x[b_i + x.size(0)] = x[k + x.size(0)];
      x[b_i + x.size(0) * 2] = x[k + x.size(0) * 2];
      x[b_i + x.size(0) * 3] = x[k + x.size(0) * 3];
      b_i++;
    }
  }
  if (nrows < 1) {
    nrows = 0;
  }
  for (i = 0; i < 4; i++) {
    for (b_i = 0; b_i < nrows; b_i++) {
      x[b_i + nrows * i] = x[b_i + x.size(0) * i];
    }
  }
  x.set_size(nrows, 4);
}

} // namespace internal
} // namespace coder

// End of code generation (nullAssignment.cpp)
