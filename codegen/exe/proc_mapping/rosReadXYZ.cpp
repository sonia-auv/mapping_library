//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rosReadXYZ.cpp
//
// Code generation for function 'rosReadXYZ'
//

// Include files
#include "rosReadXYZ.h"
#include "PointCloud2Types.h"
#include "SpecialMsgUtil.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
void rosReadXYZ(
    unsigned int msg_Height, unsigned int msg_Width,
    const ::coder::array<sensor_msgs_PointFieldStruct_T, 1U> &msg_Fields,
    unsigned int msg_PointStep,
    const ::coder::array<unsigned char, 1U> &msg_Data,
    ::coder::array<float, 2U> &xyz)
{
  array<cell_wrap_13, 2U> allFieldNames;
  array<cell_wrap_13, 2U> b_allFieldNames;
  array<cell_wrap_13, 2U> c_allFieldNames;
  array<double, 2U> byteIdx;
  array<double, 2U> c;
  array<double, 2U> y;
  array<float, 1U> varargin_1;
  array<float, 1U> varargin_2;
  array<float, 1U> varargin_3;
  array<unsigned int, 2U> pointIndices;
  array<int, 2U> r2;
  array<int, 2U> r3;
  array<unsigned int, 1U> a;
  array<char, 2U> a__1;
  array<bool, 2U> pointIdxIsValid;
  array<bool, 2U> r;
  array<bool, 2U> r1;
  double numBytes;
  double zOff;
  int acoef;
  int b_i;
  int i;
  int i1;
  int n;
  unsigned int numPointsActual;
  unsigned int qY;
  int xIdx;
  int yIdx;
  int zIdx;
  bool b_bool;
  n = msg_Fields.size(0);
  allFieldNames.set_size(1, msg_Fields.size(0));
  for (i = 0; i < n; i++) {
    allFieldNames[allFieldNames.size(0) * i].f1.set_size(1, 0);
  }
  if (msg_Fields.size(0) == 0) {
    allFieldNames.set_size(1, 0);
  } else {
    i = msg_Fields.size(0);
    allFieldNames.set_size(1, msg_Fields.size(0));
    for (b_i = 0; b_i < i; b_i++) {
      allFieldNames[allFieldNames.size(0) * b_i].f1.set_size(
          1, msg_Fields[b_i].Name.size(1));
      acoef = msg_Fields[b_i].Name.size(1);
      for (i1 = 0; i1 < acoef; i1++) {
        allFieldNames[b_i].f1[i1] = msg_Fields[b_i].Name[i1];
      }
    }
  }
  xIdx = -1;
  i = allFieldNames.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    b_bool = false;
    if ((allFieldNames[b_i].f1.size(1) == 1) &&
        (allFieldNames[b_i].f1[0] == 'x')) {
      b_bool = true;
    }
    if (b_bool) {
      xIdx = b_i;
    }
  }
  n = msg_Fields.size(0);
  b_allFieldNames.set_size(1, msg_Fields.size(0));
  for (i = 0; i < n; i++) {
    b_allFieldNames[b_allFieldNames.size(0) * i].f1.set_size(1, 0);
  }
  if (msg_Fields.size(0) == 0) {
    b_allFieldNames.set_size(1, 0);
  } else {
    i = msg_Fields.size(0);
    b_allFieldNames.set_size(1, msg_Fields.size(0));
    for (b_i = 0; b_i < i; b_i++) {
      b_allFieldNames[b_allFieldNames.size(0) * b_i].f1.set_size(
          1, msg_Fields[b_i].Name.size(1));
      acoef = msg_Fields[b_i].Name.size(1);
      for (i1 = 0; i1 < acoef; i1++) {
        b_allFieldNames[b_i].f1[i1] = msg_Fields[b_i].Name[i1];
      }
    }
  }
  yIdx = -1;
  i = b_allFieldNames.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    b_bool = false;
    if ((b_allFieldNames[b_i].f1.size(1) == 1) &&
        (b_allFieldNames[b_i].f1[0] == 'y')) {
      b_bool = true;
    }
    if (b_bool) {
      yIdx = b_i;
    }
  }
  n = msg_Fields.size(0);
  c_allFieldNames.set_size(1, msg_Fields.size(0));
  for (i = 0; i < n; i++) {
    c_allFieldNames[c_allFieldNames.size(0) * i].f1.set_size(1, 0);
  }
  if (msg_Fields.size(0) == 0) {
    c_allFieldNames.set_size(1, 0);
  } else {
    i = msg_Fields.size(0);
    c_allFieldNames.set_size(1, msg_Fields.size(0));
    for (b_i = 0; b_i < i; b_i++) {
      c_allFieldNames[c_allFieldNames.size(0) * b_i].f1.set_size(
          1, msg_Fields[b_i].Name.size(1));
      acoef = msg_Fields[b_i].Name.size(1);
      for (i1 = 0; i1 < acoef; i1++) {
        c_allFieldNames[b_i].f1[i1] = msg_Fields[b_i].Name[i1];
      }
    }
  }
  zIdx = -1;
  i = c_allFieldNames.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    b_bool = false;
    if ((c_allFieldNames[b_i].f1.size(1) == 1) &&
        (c_allFieldNames[b_i].f1[0] == 'z')) {
      b_bool = true;
    }
    if (b_bool) {
      zIdx = b_i;
    }
  }
  numPointsActual = mul_u32_sat(msg_Width, msg_Height);
  if (numPointsActual < 1U) {
    n = 0;
  } else {
    n = static_cast<int>(numPointsActual);
  }
  pointIndices.set_size(1, n);
  for (int k{0}; k < n; k++) {
    pointIndices[k] = static_cast<unsigned int>(k + 1);
  }
  numPointsActual = mul_u32_sat(msg_Height, msg_Width);
  numBytes = std::trunc(static_cast<double>(msg_Data.size(0)) /
                        static_cast<double>(msg_PointStep));
  if (numBytes < 4.294967296E+9) {
    qY = static_cast<unsigned int>(numBytes);
  } else if (numBytes >= 4.294967296E+9) {
    qY = MAX_uint32_T;
  } else {
    qY = 0U;
  }
  if ((!std::isnan(numBytes)) && (numPointsActual > numBytes)) {
    numPointsActual = qY;
  }
  ros::msg::sensor_msgs::internal::PointCloud2Types::rosToMATLABType(
      msg_Fields[xIdx].Datatype, a__1, &numBytes);
  byteIdx.set_size(pointIndices.size(1), static_cast<int>(numBytes));
  acoef = pointIndices.size(1) * static_cast<int>(numBytes);
  for (i = 0; i < acoef; i++) {
    byteIdx[i] = 0.0;
  }
  r.set_size(1, pointIndices.size(1));
  acoef = pointIndices.size(1);
  for (i = 0; i < acoef; i++) {
    r[i] = (static_cast<int>(pointIndices[i]) > 0);
  }
  r1.set_size(1, pointIndices.size(1));
  acoef = pointIndices.size(1);
  for (i = 0; i < acoef; i++) {
    r1[i] = (pointIndices[i] <= numPointsActual);
  }
  pointIdxIsValid.set_size(1, r.size(1));
  acoef = r.size(1);
  for (i = 0; i < acoef; i++) {
    pointIdxIsValid[i] = (r[i] && r1[i]);
  }
  acoef = r.size(1) - 1;
  n = 0;
  for (b_i = 0; b_i <= acoef; b_i++) {
    if (r1[b_i]) {
      n++;
    }
  }
  r2.set_size(1, n);
  n = 0;
  for (b_i = 0; b_i <= acoef; b_i++) {
    if (r1[b_i]) {
      r2[n] = b_i + 1;
      n++;
    }
  }
  if (numBytes < 1.0) {
    y.set_size(1, 0);
  } else {
    y.set_size(1, static_cast<int>(numBytes - 1.0) + 1);
    acoef = static_cast<int>(numBytes - 1.0);
    for (i = 0; i <= acoef; i++) {
      y[i] = static_cast<double>(i) + 1.0;
    }
  }
  acoef = r.size(1) - 1;
  n = 0;
  for (b_i = 0; b_i <= acoef; b_i++) {
    if (r1[b_i]) {
      n++;
    }
  }
  r3.set_size(1, n);
  n = 0;
  for (b_i = 0; b_i <= acoef; b_i++) {
    if (r1[b_i]) {
      r3[n] = b_i + 1;
      n++;
    }
  }
  a.set_size(r2.size(1));
  acoef = r2.size(1);
  for (i = 0; i < acoef; i++) {
    numPointsActual = msg_Fields[xIdx].Offset;
    qY = numPointsActual +
         mul_u32_sat(msg_PointStep, pointIndices[r2[i] - 1] - 1U);
    if (qY < numPointsActual) {
      qY = MAX_uint32_T;
    }
    a[i] = qY;
  }
  c.set_size(a.size(0), y.size(1));
  if ((a.size(0) != 0) && (y.size(1) != 0)) {
    n = (y.size(1) != 1);
    i = y.size(1) - 1;
    acoef = (a.size(0) != 1);
    for (int k{0}; k <= i; k++) {
      b_i = n * k;
      i1 = c.size(0) - 1;
      for (int b_k{0}; b_k <= i1; b_k++) {
        c[b_k + c.size(0) * k] = static_cast<double>(a[acoef * b_k]) + y[b_i];
      }
    }
  }
  acoef = c.size(1);
  for (i = 0; i < acoef; i++) {
    n = c.size(0);
    for (i1 = 0; i1 < n; i1++) {
      byteIdx[(r3[i1] + byteIdx.size(0) * i) - 1] = c[i1 + c.size(0) * i];
    }
  }
  numPointsActual = msg_Fields[xIdx].Offset;
  numBytes = static_cast<double>(msg_Fields[yIdx].Offset) -
             static_cast<double>(numPointsActual);
  zOff = static_cast<double>(msg_Fields[zIdx].Offset) -
         static_cast<double>(numPointsActual);
  ros::internal::SpecialMsgUtil::readFieldFromData(msg_Data, byteIdx,
                                                   pointIdxIsValid, varargin_1);
  c.set_size(byteIdx.size(0), byteIdx.size(1));
  acoef = byteIdx.size(0) * byteIdx.size(1);
  for (i = 0; i < acoef; i++) {
    c[i] = byteIdx[i] + numBytes;
  }
  ros::internal::SpecialMsgUtil::readFieldFromData(msg_Data, c, pointIdxIsValid,
                                                   varargin_2);
  c.set_size(byteIdx.size(0), byteIdx.size(1));
  acoef = byteIdx.size(0) * byteIdx.size(1);
  for (i = 0; i < acoef; i++) {
    c[i] = byteIdx[i] + zOff;
  }
  ros::internal::SpecialMsgUtil::readFieldFromData(msg_Data, c, pointIdxIsValid,
                                                   varargin_3);
  xyz.set_size(varargin_1.size(0), 3);
  acoef = varargin_1.size(0);
  for (i = 0; i < acoef; i++) {
    xyz[i] = varargin_1[i];
  }
  acoef = varargin_2.size(0);
  for (i = 0; i < acoef; i++) {
    xyz[i + xyz.size(0)] = varargin_2[i];
  }
  acoef = varargin_3.size(0);
  for (i = 0; i < acoef; i++) {
    xyz[i + xyz.size(0) * 2] = varargin_3[i];
  }
}

} // namespace coder

// End of code generation (rosReadXYZ.cpp)
