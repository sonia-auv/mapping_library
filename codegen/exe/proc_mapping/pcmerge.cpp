//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcmerge.cpp
//
// Code generation for function 'pcmerge'
//

// Include files
#include "pcmerge.h"
#include "Kdtree.h"
#include "minOrMax.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "rt_nonfinite.h"
#include "voxelGridFilter.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
pointCloud *pcmerge(const pointCloud *ptCloudA, const pointCloud *ptCloudB,
                    vision::internal::codegen::Kdtree *iobj_0,
                    pointCloud *iobj_1)
{
  pointCloud *ptCloudOut;
  array<double, 2U> b_rangeData;
  array<double, 2U> normal;
  array<double, 2U> overlapLimits;
  array<double, 2U> points;
  array<double, 2U> pointsA;
  array<double, 2U> pointsB;
  array<double, 2U> rangeData;
  array<double, 2U> rangeDataA;
  array<double, 2U> rangeDataB;
  array<double, 1U> intensity;
  array<double, 1U> intensityA;
  array<double, 1U> intensityB;
  array<unsigned char, 2U> b_color;
  array<unsigned char, 2U> color;
  array<unsigned char, 2U> colorA;
  array<unsigned char, 2U> colorB;
  ptCloudA->extractValidPoints(pointsA, colorA, overlapLimits, intensityA,
                               rangeDataA);
  ptCloudB->extractValidPoints(pointsB, colorB, rangeData, intensityB,
                               rangeDataB);
  if ((pointsA.size(0) == 0) && (pointsB.size(0) == 0)) {
    int loop_ub;
    ptCloudOut =
        iobj_1[0].init(pointsA, colorA, overlapLimits, intensityA, &iobj_0[0]);
    ptCloudOut->RangeData.set_size(rangeDataA.size(0), rangeDataA.size(1));
    loop_ub = rangeDataA.size(0) * rangeDataA.size(1);
    for (int i{0}; i < loop_ub; i++) {
      ptCloudOut->RangeData[i] = rangeDataA[i];
    }
  } else if ((pointsA.size(0) == 0) && (pointsB.size(0) != 0)) {
    int loop_ub;
    ptCloudOut =
        iobj_1[1].init(pointsB, colorB, rangeData, intensityB, &iobj_0[1]);
    ptCloudOut->RangeData.set_size(rangeDataB.size(0), rangeDataB.size(1));
    loop_ub = rangeDataB.size(0) * rangeDataB.size(1);
    for (int i{0}; i < loop_ub; i++) {
      ptCloudOut->RangeData[i] = rangeDataB[i];
    }
  } else if ((pointsB.size(0) == 0) && (pointsA.size(0) != 0)) {
    int loop_ub;
    ptCloudOut =
        iobj_1[2].init(pointsA, colorA, overlapLimits, intensityA, &iobj_0[2]);
    ptCloudOut->RangeData.set_size(rangeDataA.size(0), rangeDataA.size(1));
    loop_ub = rangeDataA.size(0) * rangeDataA.size(1);
    for (int i{0}; i < loop_ub; i++) {
      ptCloudOut->RangeData[i] = rangeDataA[i];
    }
  } else {
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    double d6;
    double d7;
    double d8;
    double d9;
    int input_sizes_idx_0;
    int loop_ub;
    int sizes_idx_0;
    bool empty_non_axis_sizes;
    points.set_size(pointsA.size(0) + pointsB.size(0), 3);
    loop_ub = pointsA.size(0);
    input_sizes_idx_0 = pointsB.size(0);
    for (int i{0}; i < 3; i++) {
      for (int i1{0}; i1 < loop_ub; i1++) {
        points[i1 + points.size(0) * i] = pointsA[i1 + pointsA.size(0) * i];
      }
      for (int i1{0}; i1 < input_sizes_idx_0; i1++) {
        points[(i1 + pointsA.size(0)) + points.size(0) * i] =
            pointsB[i1 + pointsB.size(0) * i];
      }
    }
    if ((colorA.size(0) != 0) && (colorA.size(1) != 0)) {
      loop_ub = colorA.size(1);
    } else if ((colorB.size(0) != 0) && (colorB.size(1) != 0)) {
      loop_ub = colorB.size(1);
    } else {
      loop_ub = colorA.size(1);
      if (colorB.size(1) > colorA.size(1)) {
        loop_ub = colorB.size(1);
      }
    }
    empty_non_axis_sizes = (loop_ub == 0);
    if (empty_non_axis_sizes ||
        ((colorA.size(0) != 0) && (colorA.size(1) != 0))) {
      input_sizes_idx_0 = colorA.size(0);
    } else {
      input_sizes_idx_0 = 0;
    }
    if (empty_non_axis_sizes ||
        ((colorB.size(0) != 0) && (colorB.size(1) != 0))) {
      sizes_idx_0 = colorB.size(0);
    } else {
      sizes_idx_0 = 0;
    }
    color.set_size(input_sizes_idx_0 + sizes_idx_0, loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < input_sizes_idx_0; i1++) {
        color[i1 + color.size(0) * i] = colorA[i1 + input_sizes_idx_0 * i];
      }
    }
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < sizes_idx_0; i1++) {
        color[(i1 + input_sizes_idx_0) + color.size(0) * i] =
            colorB[i1 + sizes_idx_0 * i];
      }
    }
    if (color.size(0) * color.size(1) != points.size(0) * 3) {
      color.set_size(0, 0);
    }
    if ((overlapLimits.size(0) != 0) && (overlapLimits.size(1) != 0)) {
      loop_ub = overlapLimits.size(1);
    } else if ((rangeData.size(0) != 0) && (rangeData.size(1) != 0)) {
      loop_ub = rangeData.size(1);
    } else {
      loop_ub = overlapLimits.size(1);
      if (rangeData.size(1) > overlapLimits.size(1)) {
        loop_ub = rangeData.size(1);
      }
    }
    empty_non_axis_sizes = (loop_ub == 0);
    if (empty_non_axis_sizes ||
        ((overlapLimits.size(0) != 0) && (overlapLimits.size(1) != 0))) {
      input_sizes_idx_0 = overlapLimits.size(0);
    } else {
      input_sizes_idx_0 = 0;
    }
    if (empty_non_axis_sizes ||
        ((rangeData.size(0) != 0) && (rangeData.size(1) != 0))) {
      sizes_idx_0 = rangeData.size(0);
    } else {
      sizes_idx_0 = 0;
    }
    normal.set_size(input_sizes_idx_0 + sizes_idx_0, loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < input_sizes_idx_0; i1++) {
        normal[i1 + normal.size(0) * i] =
            overlapLimits[i1 + input_sizes_idx_0 * i];
      }
    }
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < sizes_idx_0; i1++) {
        normal[(i1 + input_sizes_idx_0) + normal.size(0) * i] =
            rangeData[i1 + sizes_idx_0 * i];
      }
    }
    if (normal.size(0) * normal.size(1) != points.size(0) * 3) {
      normal.set_size(0, 0);
    }
    intensity.set_size(intensityA.size(0) + intensityB.size(0));
    loop_ub = intensityA.size(0);
    for (int i{0}; i < loop_ub; i++) {
      intensity[i] = intensityA[i];
    }
    loop_ub = intensityB.size(0);
    for (int i{0}; i < loop_ub; i++) {
      intensity[i + intensityA.size(0)] = intensityB[i];
    }
    if (intensity.size(0) != points.size(0)) {
      intensity.set_size(0);
    }
    if ((rangeDataA.size(0) != 0) && (rangeDataA.size(1) != 0)) {
      loop_ub = rangeDataA.size(1);
    } else if ((rangeDataB.size(0) != 0) && (rangeDataB.size(1) != 0)) {
      loop_ub = rangeDataB.size(1);
    } else {
      loop_ub = rangeDataA.size(1);
      if (rangeDataB.size(1) > rangeDataA.size(1)) {
        loop_ub = rangeDataB.size(1);
      }
    }
    empty_non_axis_sizes = (loop_ub == 0);
    if (empty_non_axis_sizes ||
        ((rangeDataA.size(0) != 0) && (rangeDataA.size(1) != 0))) {
      input_sizes_idx_0 = rangeDataA.size(0);
    } else {
      input_sizes_idx_0 = 0;
    }
    if (empty_non_axis_sizes ||
        ((rangeDataB.size(0) != 0) && (rangeDataB.size(1) != 0))) {
      sizes_idx_0 = rangeDataB.size(0);
    } else {
      sizes_idx_0 = 0;
    }
    rangeData.set_size(input_sizes_idx_0 + sizes_idx_0, loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < input_sizes_idx_0; i1++) {
        rangeData[i1 + rangeData.size(0) * i] =
            rangeDataA[i1 + input_sizes_idx_0 * i];
      }
    }
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < sizes_idx_0; i1++) {
        rangeData[(i1 + input_sizes_idx_0) + rangeData.size(0) * i] =
            rangeDataB[i1 + sizes_idx_0 * i];
      }
    }
    if (rangeData.size(0) * rangeData.size(1) != points.size(0) * 3) {
      rangeData.set_size(0, 0);
    }
    loop_ub = pointsA.size(0);
    intensityA.set_size(pointsA.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityA[i] = pointsA[i];
    }
    loop_ub = pointsA.size(0);
    intensityB.set_size(pointsA.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityB[i] = pointsA[i];
    }
    d = internal::minimum(intensityA);
    d1 = internal::maximum(intensityB);
    loop_ub = pointsA.size(0);
    intensityA.set_size(pointsA.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityA[i] = pointsA[i + pointsA.size(0)];
    }
    loop_ub = pointsA.size(0);
    intensityB.set_size(pointsA.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityB[i] = pointsA[i + pointsA.size(0)];
    }
    d2 = internal::minimum(intensityA);
    d3 = internal::maximum(intensityB);
    loop_ub = pointsA.size(0);
    intensityA.set_size(pointsA.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityA[i] = pointsA[i + pointsA.size(0) * 2];
    }
    loop_ub = pointsA.size(0);
    intensityB.set_size(pointsA.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityB[i] = pointsA[i + pointsA.size(0) * 2];
    }
    d4 = internal::minimum(intensityA);
    d5 = internal::maximum(intensityB);
    loop_ub = pointsB.size(0);
    intensityA.set_size(pointsB.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityA[i] = pointsB[i];
    }
    loop_ub = pointsB.size(0);
    intensityB.set_size(pointsB.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityB[i] = pointsB[i];
    }
    d6 = internal::minimum(intensityA);
    d7 = internal::maximum(intensityB);
    loop_ub = pointsB.size(0);
    intensityA.set_size(pointsB.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityA[i] = pointsB[i + pointsB.size(0)];
    }
    loop_ub = pointsB.size(0);
    intensityB.set_size(pointsB.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityB[i] = pointsB[i + pointsB.size(0)];
    }
    d8 = internal::minimum(intensityA);
    d9 = internal::maximum(intensityB);
    loop_ub = pointsB.size(0);
    intensityA.set_size(pointsB.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityA[i] = pointsB[i + pointsB.size(0) * 2];
    }
    loop_ub = pointsB.size(0);
    intensityB.set_size(pointsB.size(0));
    for (int i{0}; i < loop_ub; i++) {
      intensityB[i] = pointsB[i + pointsB.size(0) * 2];
    }
    if ((d > d7) || (d1 < d6) || (d2 > d9) || (d3 < d8)) {
      overlapLimits.set_size(0, 0);
    } else {
      double d10;
      d10 = internal::maximum(intensityB);
      if (d4 > d10) {
        overlapLimits.set_size(0, 0);
      } else {
        double d11;
        d11 = internal::minimum(intensityA);
        if (d5 < d11) {
          overlapLimits.set_size(0, 0);
        } else {
          double dv[6];
          dv[0] = std::fmax(d, d6);
          dv[1] = std::fmin(d1, d7);
          dv[2] = std::fmax(d2, d8);
          dv[3] = std::fmin(d3, d9);
          dv[4] = std::fmax(d4, d11);
          dv[5] = std::fmin(d5, d10);
          overlapLimits.set_size(1, 6);
          for (int i{0}; i < 6; i++) {
            overlapLimits[i] = dv[i];
          }
        }
      }
    }
    if ((overlapLimits.size(0) == 0) || (overlapLimits.size(1) == 0)) {
      ptCloudOut = iobj_1[3].init(points, color, normal, intensity, &iobj_0[3]);
      ptCloudOut->RangeData.set_size(rangeData.size(0), rangeData.size(1));
      loop_ub = rangeData.size(0) * rangeData.size(1);
      for (int i{0}; i < loop_ub; i++) {
        ptCloudOut->RangeData[i] = rangeData[i];
      }
    } else {
      vision::internal::codegen::pc::voxelGridFilter(
          points, color, normal, intensity, rangeData, overlapLimits, pointsA,
          b_color, pointsB, intensityA, b_rangeData);
      ptCloudOut = &iobj_1[4];
      iobj_1[4].Location.set_size(pointsA.size(0), 3);
      loop_ub = pointsA.size(0) * 3;
      for (int i{0}; i < loop_ub; i++) {
        iobj_1[4].Location[i] = pointsA[i];
      }
      iobj_1[4].Color.set_size(b_color.size(0), 3);
      loop_ub = b_color.size(0) * 3;
      for (int i{0}; i < loop_ub; i++) {
        iobj_1[4].Color[i] = b_color[i];
      }
      iobj_1[4].Normal.set_size(pointsB.size(0), 3);
      loop_ub = pointsB.size(0) * 3;
      for (int i{0}; i < loop_ub; i++) {
        iobj_1[4].Normal[i] = pointsB[i];
      }
      iobj_1[4].Intensity.set_size(intensityA.size(0));
      loop_ub = intensityA.size(0);
      for (int i{0}; i < loop_ub; i++) {
        iobj_1[4].Intensity[i] = intensityA[i];
      }
      pointclouds::internal::codegen::pc::pointCloudArray r;
      iobj_1[4].RangeData.set_size(0, 0);
      iobj_1[4].PointCloudArrayData.set_size(1, 1);
      iobj_1[4].PointCloudArrayData[0] = r;
      iobj_1[4].XLimitsInternal.set_size(0, 0);
      iobj_1[4].YLimitsInternal.set_size(0, 0);
      iobj_1[4].ZLimitsInternal.set_size(0, 0);
      iobj_1[4].Kdtree = &iobj_0[4];
      iobj_1[4].matlabCodegenIsDeleted = false;
      iobj_1[4].RangeData.set_size(b_rangeData.size(0), 3);
      loop_ub = b_rangeData.size(0) * 3;
      for (int i{0}; i < loop_ub; i++) {
        iobj_1[4].RangeData[i] = b_rangeData[i];
      }
    }
  }
  return ptCloudOut;
}

} // namespace coder

// End of code generation (pcmerge.cpp)
