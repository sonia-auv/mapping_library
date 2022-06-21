//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// SpecialMsgUtil.cpp
//
// Code generation for function 'SpecialMsgUtil'
//

// Include files
#include "SpecialMsgUtil.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstddef>
#include <cstring>
#include <string.h>

// Function Definitions
namespace coder {
namespace ros {
namespace internal {
void SpecialMsgUtil::readFieldFromData(
    const ::coder::array<unsigned char, 1U> &data,
    const ::coder::array<double, 2U> &byteIdx,
    const ::coder::array<bool, 2U> &pointIdxIsValid,
    ::coder::array<float, 1U> &fieldPoints)
{
  array<float, 1U> r1;
  array<float, 1U> validPoints;
  array<int, 2U> r;
  array<int, 2U> r2;
  array<unsigned char, 2U> b_data;
  array<unsigned char, 1U> rawData;
  array<bool, 1U> x;
  int end;
  int ny;
  int sz_idx_0;
  bool exitg1;
  bool y;
  end = pointIdxIsValid.size(1) - 1;
  ny = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= end; sz_idx_0++) {
    if (pointIdxIsValid[sz_idx_0]) {
      ny++;
    }
  }
  r.set_size(1, ny);
  ny = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= end; sz_idx_0++) {
    if (pointIdxIsValid[sz_idx_0]) {
      r[ny] = sz_idx_0 + 1;
      ny++;
    }
  }
  sz_idx_0 = byteIdx.size(1) * r.size(1);
  ny = byteIdx.size(1);
  b_data.set_size(byteIdx.size(1), r.size(1));
  end = r.size(1);
  for (int i{0}; i < end; i++) {
    for (int i1{0}; i1 < ny; i1++) {
      b_data[i1 + b_data.size(0) * i] =
          data[static_cast<int>(byteIdx[(r[i] + byteIdx.size(0) * i1) - 1]) -
               1];
    }
  }
  ny = byteIdx.size(1) * r.size(1);
  rawData.set_size(ny);
  for (int i{0}; i < ny; i++) {
    rawData[i] = b_data[i];
  }
  if (sz_idx_0 == 0) {
    ny = 0;
  } else {
    ny = sz_idx_0 >> 2;
  }
  r1.set_size(ny);
  std::memcpy((void *)&(r1.data())[0], (void *)&(rawData.data())[0],
              (unsigned int)((size_t)ny * sizeof(float)));
  validPoints.set_size(r1.size(0));
  ny = r1.size(0);
  for (int i{0}; i < ny; i++) {
    validPoints[i] = r1[i];
  }
  x.set_size(pointIdxIsValid.size(1));
  ny = pointIdxIsValid.size(1);
  for (int i{0}; i < ny; i++) {
    x[i] = !pointIdxIsValid[i];
  }
  y = false;
  ny = 1;
  exitg1 = false;
  while ((!exitg1) && (ny <= x.size(0))) {
    if (x[ny - 1]) {
      y = true;
      exitg1 = true;
    } else {
      ny++;
    }
  }
  if (y) {
    fieldPoints.set_size(pointIdxIsValid.size(1));
    ny = pointIdxIsValid.size(1);
    for (int i{0}; i < ny; i++) {
      fieldPoints[i] = rtNaNF;
    }
    end = pointIdxIsValid.size(1) - 1;
    ny = 0;
    for (sz_idx_0 = 0; sz_idx_0 <= end; sz_idx_0++) {
      if (pointIdxIsValid[sz_idx_0]) {
        ny++;
      }
    }
    r2.set_size(1, ny);
    ny = 0;
    for (sz_idx_0 = 0; sz_idx_0 <= end; sz_idx_0++) {
      if (pointIdxIsValid[sz_idx_0]) {
        r2[ny] = sz_idx_0 + 1;
        ny++;
      }
    }
    ny = validPoints.size(0);
    for (int i{0}; i < ny; i++) {
      fieldPoints[r2[i] - 1] = validPoints[i];
    }
  } else {
    fieldPoints.set_size(validPoints.size(0));
    ny = validPoints.size(0);
    for (int i{0}; i < ny; i++) {
      fieldPoints[i] = validPoints[i];
    }
  }
}

} // namespace internal
} // namespace ros
} // namespace coder

// End of code generation (SpecialMsgUtil.cpp)
