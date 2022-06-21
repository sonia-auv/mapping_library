//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rosReadField.cpp
//
// Code generation for function 'rosReadField'
//

// Include files
#include "rosReadField.h"
#include "PointCloud2Types.h"
#include "proc_mapping_data.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstddef>
#include <cstring>
#include <string.h>

// Function Declarations
static int div_s32(int numerator, int denominator);

// Function Definitions
static int div_s32(int numerator, int denominator)
{
  int quotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    unsigned int b_denominator;
    unsigned int b_numerator;
    if (numerator < 0) {
      b_numerator = ~static_cast<unsigned int>(numerator) + 1U;
    } else {
      b_numerator = static_cast<unsigned int>(numerator);
    }
    if (denominator < 0) {
      b_denominator = ~static_cast<unsigned int>(denominator) + 1U;
    } else {
      b_denominator = static_cast<unsigned int>(denominator);
    }
    b_numerator /= b_denominator;
    if ((numerator < 0) != (denominator < 0)) {
      quotient = -static_cast<int>(b_numerator);
    } else {
      quotient = static_cast<int>(b_numerator);
    }
  }
  return quotient;
}

namespace coder {
void rosReadField(
    unsigned int msg_Height, unsigned int msg_Width,
    const ::coder::array<sensor_msgs_PointFieldStruct_T, 1U> &msg_Fields,
    unsigned int msg_PointStep,
    const ::coder::array<unsigned char, 1U> &msg_Data,
    ::coder::array<float, 2U> &fieldData)
{
  array<cell_wrap_13, 2U> allFieldNames;
  array<double, 2U> b_y;
  array<double, 2U> byteIdx;
  array<double, 2U> c;
  array<float, 2U> validPoints;
  array<float, 1U> r4;
  array<unsigned int, 2U> pointIndices;
  array<int, 2U> r1;
  array<int, 2U> r2;
  array<int, 2U> r3;
  array<int, 2U> r5;
  array<unsigned int, 1U> a;
  array<char, 2U> a__1;
  array<unsigned char, 2U> b_msg_Data;
  array<unsigned char, 1U> rawData;
  array<bool, 2U> pointIdxIsValid;
  array<bool, 2U> r;
  array<bool, 1U> x;
  double y;
  int bcoef;
  int fieldIdx;
  int i;
  int i1;
  int loop_ub;
  unsigned int numPointsActual;
  unsigned int qY;
  int sz_idx_0;
  int sz_idx_1;
  bool b_bool;
  bool exitg2;
  sz_idx_1 = msg_Fields.size(0);
  allFieldNames.set_size(1, msg_Fields.size(0));
  for (i = 0; i < sz_idx_1; i++) {
    allFieldNames[allFieldNames.size(0) * i].f1.set_size(1, 0);
  }
  if (msg_Fields.size(0) == 0) {
    allFieldNames.set_size(1, 0);
  } else {
    i = msg_Fields.size(0);
    allFieldNames.set_size(1, msg_Fields.size(0));
    for (sz_idx_0 = 0; sz_idx_0 < i; sz_idx_0++) {
      allFieldNames[allFieldNames.size(0) * sz_idx_0].f1.set_size(
          1, msg_Fields[sz_idx_0].Name.size(1));
      loop_ub = msg_Fields[sz_idx_0].Name.size(1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        allFieldNames[sz_idx_0].f1[i1] = msg_Fields[sz_idx_0].Name[i1];
      }
    }
  }
  fieldIdx = -1;
  i = allFieldNames.size(1);
  for (sz_idx_0 = 0; sz_idx_0 < i; sz_idx_0++) {
    b_bool = false;
    if (allFieldNames[sz_idx_0].f1.size(1) == 9) {
      sz_idx_1 = 0;
      int exitg1;
      do {
        exitg1 = 0;
        if (sz_idx_1 < 9) {
          if (allFieldNames[sz_idx_0].f1[sz_idx_1] != cv[sz_idx_1]) {
            exitg1 = 1;
          } else {
            sz_idx_1++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (b_bool) {
      fieldIdx = sz_idx_0;
    }
  }
  numPointsActual = mul_u32_sat(msg_Width, msg_Height);
  if (numPointsActual < 1U) {
    sz_idx_1 = 0;
  } else {
    sz_idx_1 = static_cast<int>(numPointsActual);
  }
  pointIndices.set_size(1, sz_idx_1);
  for (int k{0}; k < sz_idx_1; k++) {
    pointIndices[k] = static_cast<unsigned int>(k + 1);
  }
  numPointsActual = mul_u32_sat(msg_Height, msg_Width);
  y = std::trunc(static_cast<double>(msg_Data.size(0)) /
                 static_cast<double>(msg_PointStep));
  if (y < 4.294967296E+9) {
    qY = static_cast<unsigned int>(y);
  } else if (y >= 4.294967296E+9) {
    qY = MAX_uint32_T;
  } else {
    qY = 0U;
  }
  if ((!std::isnan(y)) && (numPointsActual > y)) {
    numPointsActual = qY;
  }
  ros::msg::sensor_msgs::internal::PointCloud2Types::rosToMATLABType(
      msg_Fields[fieldIdx].Datatype, a__1, &y);
  y *= static_cast<double>(msg_Fields[fieldIdx].Count);
  byteIdx.set_size(pointIndices.size(1), static_cast<int>(y));
  loop_ub = pointIndices.size(1) * static_cast<int>(y);
  for (i = 0; i < loop_ub; i++) {
    byteIdx[i] = 0.0;
  }
  pointIdxIsValid.set_size(1, pointIndices.size(1));
  loop_ub = pointIndices.size(1);
  for (i = 0; i < loop_ub; i++) {
    pointIdxIsValid[i] = (static_cast<int>(pointIndices[i]) > 0);
  }
  r.set_size(1, pointIndices.size(1));
  loop_ub = pointIndices.size(1);
  for (i = 0; i < loop_ub; i++) {
    r[i] = (pointIndices[i] <= numPointsActual);
  }
  bcoef = pointIdxIsValid.size(1) - 1;
  sz_idx_1 = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
    if (r[sz_idx_0]) {
      sz_idx_1++;
    }
  }
  r1.set_size(1, sz_idx_1);
  sz_idx_1 = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
    if (r[sz_idx_0]) {
      r1[sz_idx_1] = sz_idx_0 + 1;
      sz_idx_1++;
    }
  }
  if (y < 1.0) {
    b_y.set_size(1, 0);
  } else {
    b_y.set_size(1, static_cast<int>(y - 1.0) + 1);
    loop_ub = static_cast<int>(y - 1.0);
    for (i = 0; i <= loop_ub; i++) {
      b_y[i] = static_cast<double>(i) + 1.0;
    }
  }
  bcoef = pointIdxIsValid.size(1) - 1;
  sz_idx_1 = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
    if (r[sz_idx_0]) {
      sz_idx_1++;
    }
  }
  r2.set_size(1, sz_idx_1);
  sz_idx_1 = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
    if (r[sz_idx_0]) {
      r2[sz_idx_1] = sz_idx_0 + 1;
      sz_idx_1++;
    }
  }
  a.set_size(r1.size(1));
  loop_ub = r1.size(1);
  for (i = 0; i < loop_ub; i++) {
    numPointsActual = msg_Fields[fieldIdx].Offset;
    qY = numPointsActual +
         mul_u32_sat(msg_PointStep, pointIndices[r1[i] - 1] - 1U);
    if (qY < numPointsActual) {
      qY = MAX_uint32_T;
    }
    a[i] = qY;
  }
  c.set_size(a.size(0), b_y.size(1));
  if ((a.size(0) != 0) && (b_y.size(1) != 0)) {
    bcoef = (b_y.size(1) != 1);
    i = b_y.size(1) - 1;
    sz_idx_1 = (a.size(0) != 1);
    for (int k{0}; k <= i; k++) {
      sz_idx_0 = bcoef * k;
      i1 = c.size(0) - 1;
      for (loop_ub = 0; loop_ub <= i1; loop_ub++) {
        c[loop_ub + c.size(0) * k] =
            static_cast<double>(a[sz_idx_1 * loop_ub]) + b_y[sz_idx_0];
      }
    }
  }
  loop_ub = c.size(1);
  for (i = 0; i < loop_ub; i++) {
    bcoef = c.size(0);
    for (i1 = 0; i1 < bcoef; i1++) {
      byteIdx[(r2[i1] + byteIdx.size(0) * i) - 1] = c[i1 + c.size(0) * i];
    }
  }
  pointIdxIsValid.set_size(1, pointIdxIsValid.size(1));
  loop_ub = pointIdxIsValid.size(1) - 1;
  for (i = 0; i <= loop_ub; i++) {
    pointIdxIsValid[i] = (pointIdxIsValid[i] && r[i]);
  }
  bcoef = pointIdxIsValid.size(1) - 1;
  sz_idx_1 = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
    if (pointIdxIsValid[sz_idx_0]) {
      sz_idx_1++;
    }
  }
  r3.set_size(1, sz_idx_1);
  sz_idx_1 = 0;
  for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
    if (pointIdxIsValid[sz_idx_0]) {
      r3[sz_idx_1] = sz_idx_0 + 1;
      sz_idx_1++;
    }
  }
  sz_idx_0 = byteIdx.size(1) * r3.size(1);
  loop_ub = byteIdx.size(1);
  b_msg_Data.set_size(byteIdx.size(1), r3.size(1));
  bcoef = r3.size(1);
  for (i = 0; i < bcoef; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_msg_Data[i1 + b_msg_Data.size(0) * i] = msg_Data
          [static_cast<int>(byteIdx[(r3[i] + byteIdx.size(0) * i1) - 1]) - 1];
    }
  }
  sz_idx_1 = byteIdx.size(1) * r3.size(1);
  rawData.set_size(sz_idx_1);
  for (i = 0; i < sz_idx_1; i++) {
    rawData[i] = b_msg_Data[i];
  }
  if (sz_idx_0 == 0) {
    sz_idx_1 = 0;
  } else {
    sz_idx_1 = sz_idx_0 >> 2;
  }
  r4.set_size(sz_idx_1);
  std::memcpy((void *)&(r4.data())[0], (void *)&(rawData.data())[0],
              (unsigned int)((size_t)sz_idx_1 * sizeof(float)));
  if (static_cast<int>(msg_Fields[fieldIdx].Count) > 0) {
    sz_idx_1 =
        div_s32(r4.size(0), static_cast<int>(msg_Fields[fieldIdx].Count));
  } else {
    sz_idx_1 = 0;
  }
  bcoef = static_cast<int>(msg_Fields[fieldIdx].Count);
  validPoints.set_size(sz_idx_1, static_cast<int>(msg_Fields[fieldIdx].Count));
  loop_ub = static_cast<int>(msg_Fields[fieldIdx].Count);
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < sz_idx_1; i1++) {
      validPoints[i1 + validPoints.size(0) * i] = r4[i + bcoef * i1];
    }
  }
  x.set_size(pointIdxIsValid.size(1));
  loop_ub = pointIdxIsValid.size(1);
  for (i = 0; i < loop_ub; i++) {
    x[i] = !pointIdxIsValid[i];
  }
  b_bool = false;
  sz_idx_1 = 1;
  exitg2 = false;
  while ((!exitg2) && (sz_idx_1 <= x.size(0))) {
    if (x[sz_idx_1 - 1]) {
      b_bool = true;
      exitg2 = true;
    } else {
      sz_idx_1++;
    }
  }
  if (b_bool) {
    fieldData.set_size(pointIdxIsValid.size(1),
                       static_cast<int>(msg_Fields[fieldIdx].Count));
    loop_ub =
        pointIdxIsValid.size(1) * static_cast<int>(msg_Fields[fieldIdx].Count);
    for (i = 0; i < loop_ub; i++) {
      fieldData[i] = rtNaNF;
    }
    bcoef = pointIdxIsValid.size(1) - 1;
    sz_idx_1 = 0;
    for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
      if (pointIdxIsValid[sz_idx_0]) {
        sz_idx_1++;
      }
    }
    r5.set_size(1, sz_idx_1);
    sz_idx_1 = 0;
    for (sz_idx_0 = 0; sz_idx_0 <= bcoef; sz_idx_0++) {
      if (pointIdxIsValid[sz_idx_0]) {
        r5[sz_idx_1] = sz_idx_0 + 1;
        sz_idx_1++;
      }
    }
    loop_ub = validPoints.size(1);
    for (i = 0; i < loop_ub; i++) {
      bcoef = validPoints.size(0);
      for (i1 = 0; i1 < bcoef; i1++) {
        fieldData[(r5[i1] + fieldData.size(0) * i) - 1] =
            validPoints[i1 + validPoints.size(0) * i];
      }
    }
  } else {
    fieldData.set_size(validPoints.size(0), validPoints.size(1));
    loop_ub = validPoints.size(0) * validPoints.size(1);
    for (i = 0; i < loop_ub; i++) {
      fieldData[i] = validPoints[i];
    }
  }
}

} // namespace coder

// End of code generation (rosReadField.cpp)
