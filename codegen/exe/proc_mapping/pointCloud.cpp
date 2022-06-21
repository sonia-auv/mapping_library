//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pointCloud.cpp
//
// Code generation for function 'pointCloud'
//

// Include files
#include "pointCloud.h"
#include "Kdtree.h"
#include "any1.h"
#include "minOrMax.h"
#include "pointCloudArray.h"
#include "proc_mapping_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
pointCloud *
pointCloud::b_select(const ::coder::array<double, 1U> &varargin_1,
                     ::coder::vision::internal::codegen::Kdtree *iobj_0,
                     pointCloud *iobj_1)
{
  pointCloud *ptCloudOut;
  array<double, 2U> loc;
  array<double, 2U> location;
  array<double, 2U> normal;
  array<double, 2U> nv;
  array<double, 2U> rangeData;
  array<double, 1U> b_intensity;
  array<double, 1U> intensity;
  array<unsigned char, 2U> c;
  array<unsigned char, 2U> color;
  int b_loop_ub;
  int loop_ub;
  location.set_size(Location.size(0), 3);
  loop_ub = Location.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    location[i] = Location[i];
  }
  color.set_size(Color.size(0), Color.size(1));
  loop_ub = Color.size(0) * Color.size(1);
  for (int i{0}; i < loop_ub; i++) {
    color[i] = Color[i];
  }
  normal.set_size(Normal.size(0), Normal.size(1));
  loop_ub = Normal.size(0) * Normal.size(1);
  for (int i{0}; i < loop_ub; i++) {
    normal[i] = Normal[i];
  }
  intensity.set_size(Intensity.size(0));
  loop_ub = Intensity.size(0);
  for (int i{0}; i < loop_ub; i++) {
    intensity[i] = Intensity[i];
  }
  rangeData.set_size(RangeData.size(0), RangeData.size(1));
  loop_ub = RangeData.size(0) * RangeData.size(1);
  for (int i{0}; i < loop_ub; i++) {
    rangeData[i] = RangeData[i];
  }
  if (location.size(0) != 0) {
    loc.set_size(varargin_1.size(0), 3);
    loop_ub = varargin_1.size(0);
    for (int i{0}; i < 3; i++) {
      for (int i1{0}; i1 < loop_ub; i1++) {
        loc[i1 + loc.size(0) * i] =
            location[(static_cast<int>(varargin_1[i1]) + location.size(0) * i) -
                     1];
      }
    }
  } else {
    loc.set_size(0, 3);
  }
  if ((color.size(0) != 0) && (color.size(1) != 0)) {
    loop_ub = color.size(1);
    c.set_size(varargin_1.size(0), color.size(1));
    b_loop_ub = varargin_1.size(0);
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < b_loop_ub; i1++) {
        c[i1 + c.size(0) * i] =
            color[(static_cast<int>(varargin_1[i1]) + color.size(0) * i) - 1];
      }
    }
  } else {
    c.set_size(0, 0);
  }
  if ((normal.size(0) != 0) && (normal.size(1) != 0)) {
    loop_ub = normal.size(1);
    nv.set_size(varargin_1.size(0), normal.size(1));
    b_loop_ub = varargin_1.size(0);
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < b_loop_ub; i1++) {
        nv[i1 + nv.size(0) * i] =
            normal[(static_cast<int>(varargin_1[i1]) + normal.size(0) * i) - 1];
      }
    }
  } else {
    nv.set_size(0, 0);
  }
  if (intensity.size(0) != 0) {
    b_intensity.set_size(varargin_1.size(0));
    loop_ub = varargin_1.size(0);
    for (int i{0}; i < loop_ub; i++) {
      b_intensity[i] = intensity[static_cast<int>(varargin_1[i]) - 1];
    }
  } else {
    b_intensity.set_size(0);
  }
  if ((rangeData.size(0) != 0) && (rangeData.size(1) != 0)) {
    loop_ub = rangeData.size(1);
    normal.set_size(varargin_1.size(0), rangeData.size(1));
    b_loop_ub = varargin_1.size(0);
    for (int i{0}; i < loop_ub; i++) {
      for (int i1{0}; i1 < b_loop_ub; i1++) {
        normal[i1 + normal.size(0) * i] = rangeData
            [(static_cast<int>(varargin_1[i1]) + rangeData.size(0) * i) - 1];
      }
    }
  } else {
    normal.set_size(0, 0);
  }
  ptCloudOut = iobj_1->init(loc, c, nv, b_intensity, iobj_0);
  ptCloudOut->RangeData.set_size(normal.size(0), normal.size(1));
  loop_ub = normal.size(0) * normal.size(1);
  for (int i{0}; i < loop_ub; i++) {
    ptCloudOut->RangeData[i] = normal[i];
  }
  return ptCloudOut;
}

pointCloud::pointCloud()
{
  matlabCodegenIsDeleted = true;
}

b_pointCloud::b_pointCloud()
{
  matlabCodegenIsDeleted = true;
}

pointCloud::~pointCloud()
{
  matlabCodegenDestructor();
}

b_pointCloud::~b_pointCloud()
{
  matlabCodegenDestructor();
}

void pointCloud::extractValidPoints(::coder::array<double, 2U> &location,
                                    ::coder::array<unsigned char, 2U> &color,
                                    ::coder::array<double, 2U> &normals,
                                    ::coder::array<double, 1U> &intensity,
                                    ::coder::array<double, 2U> &rangeData) const
{
  array<int, 1U> y;
  array<bool, 2U> r;
  array<bool, 2U> x;
  array<bool, 1U> b_y;
  int vstride;
  int xoffset;
  x.set_size(Location.size(0), 3);
  vstride = Location.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    x[xoffset] = std::isinf(Location[xoffset]);
  }
  r.set_size(Location.size(0), 3);
  vstride = Location.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    r[xoffset] = std::isnan(Location[xoffset]);
  }
  vstride = x.size(0) * 3;
  x.set_size(x.size(0), 3);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    x[xoffset] = ((!x[xoffset]) && (!r[xoffset]));
  }
  if (x.size(0) == 0) {
    y.set_size(0);
  } else {
    vstride = x.size(0);
    y.set_size(x.size(0));
    for (int j{0}; j < vstride; j++) {
      y[j] = x[j];
    }
    for (int k{0}; k < 2; k++) {
      xoffset = (k + 1) * vstride;
      for (int j{0}; j < vstride; j++) {
        y[j] = y[j] + x[xoffset + j];
      }
    }
  }
  b_y.set_size(y.size(0));
  vstride = y.size(0);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    b_y[xoffset] = (y[xoffset] == 3);
  }
  subsetImpl(b_y, location, color, normals, intensity, rangeData);
}

void pointCloud::get_XLimits(::coder::array<double, 2U> &xlim)
{
  array<double, 1U> b_this;
  array<double, 1U> c_this;
  array<int, 1U> r;
  array<int, 1U> y;
  array<bool, 2U> x;
  array<bool, 1U> tf;
  int vstride;
  int xoffset;
  if ((XLimitsInternal.size(0) == 0) || (XLimitsInternal.size(1) == 0)) {
    x.set_size(Location.size(0), 3);
    vstride = Location.size(0) * 3;
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = std::isnan(Location[xoffset]);
    }
    vstride = x.size(0) * 3;
    x.set_size(x.size(0), 3);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = !x[xoffset];
    }
    if (x.size(0) == 0) {
      y.set_size(0);
    } else {
      vstride = x.size(0);
      y.set_size(x.size(0));
      for (int j{0}; j < vstride; j++) {
        y[j] = x[j];
      }
      for (int k{0}; k < 2; k++) {
        xoffset = (k + 1) * vstride;
        for (int j{0}; j < vstride; j++) {
          y[j] = y[j] + x[xoffset + j];
        }
      }
    }
    tf.set_size(y.size(0));
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      tf[xoffset] = (y[xoffset] == 3);
    }
    vstride = 0;
    xoffset = tf.size(0);
    for (int k{0}; k < xoffset; k++) {
      if (tf[k]) {
        vstride++;
      }
    }
    if (vstride != 0) {
      xoffset = tf.size(0) - 1;
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          vstride++;
        }
      }
      r.set_size(vstride);
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          r[vstride] = j + 1;
          vstride++;
        }
      }
      b_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_this[xoffset] = Location[r[xoffset] - 1];
      }
      c_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        c_this[xoffset] = Location[r[xoffset] - 1];
      }
      XLimitsInternal.set_size(1, 2);
      XLimitsInternal[0] = internal::minimum(b_this);
      XLimitsInternal[1] = internal::maximum(c_this);
    } else {
      XLimitsInternal.set_size(0, 2);
    }
  }
  xlim.set_size(XLimitsInternal.size(0), XLimitsInternal.size(1));
  vstride = XLimitsInternal.size(0) * XLimitsInternal.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    xlim[xoffset] = XLimitsInternal[xoffset];
  }
}

void pointCloud::get_YLimits(::coder::array<double, 2U> &ylim)
{
  array<double, 1U> b_this;
  array<double, 1U> c_this;
  array<int, 1U> r;
  array<int, 1U> y;
  array<bool, 2U> x;
  array<bool, 1U> tf;
  int vstride;
  int xoffset;
  if ((YLimitsInternal.size(0) == 0) || (YLimitsInternal.size(1) == 0)) {
    x.set_size(Location.size(0), 3);
    vstride = Location.size(0) * 3;
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = std::isnan(Location[xoffset]);
    }
    vstride = x.size(0) * 3;
    x.set_size(x.size(0), 3);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = !x[xoffset];
    }
    if (x.size(0) == 0) {
      y.set_size(0);
    } else {
      vstride = x.size(0);
      y.set_size(x.size(0));
      for (int j{0}; j < vstride; j++) {
        y[j] = x[j];
      }
      for (int k{0}; k < 2; k++) {
        xoffset = (k + 1) * vstride;
        for (int j{0}; j < vstride; j++) {
          y[j] = y[j] + x[xoffset + j];
        }
      }
    }
    tf.set_size(y.size(0));
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      tf[xoffset] = (y[xoffset] == 3);
    }
    vstride = 0;
    xoffset = tf.size(0);
    for (int k{0}; k < xoffset; k++) {
      if (tf[k]) {
        vstride++;
      }
    }
    if (vstride != 0) {
      xoffset = tf.size(0) - 1;
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          vstride++;
        }
      }
      r.set_size(vstride);
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          r[vstride] = j + 1;
          vstride++;
        }
      }
      b_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_this[xoffset] = Location[(r[xoffset] + Location.size(0)) - 1];
      }
      c_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        c_this[xoffset] = Location[(r[xoffset] + Location.size(0)) - 1];
      }
      YLimitsInternal.set_size(1, 2);
      YLimitsInternal[0] = internal::minimum(b_this);
      YLimitsInternal[1] = internal::maximum(c_this);
    } else {
      YLimitsInternal.set_size(0, 2);
    }
  }
  ylim.set_size(YLimitsInternal.size(0), YLimitsInternal.size(1));
  vstride = YLimitsInternal.size(0) * YLimitsInternal.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    ylim[xoffset] = YLimitsInternal[xoffset];
  }
}

void pointCloud::get_ZLimits(::coder::array<double, 2U> &zlim)
{
  array<double, 1U> b_this;
  array<double, 1U> c_this;
  array<int, 1U> r;
  array<int, 1U> y;
  array<bool, 2U> x;
  array<bool, 1U> tf;
  int vstride;
  int xoffset;
  if ((ZLimitsInternal.size(0) == 0) || (ZLimitsInternal.size(1) == 0)) {
    x.set_size(Location.size(0), 3);
    vstride = Location.size(0) * 3;
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = std::isnan(Location[xoffset]);
    }
    vstride = x.size(0) * 3;
    x.set_size(x.size(0), 3);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = !x[xoffset];
    }
    if (x.size(0) == 0) {
      y.set_size(0);
    } else {
      vstride = x.size(0);
      y.set_size(x.size(0));
      for (int j{0}; j < vstride; j++) {
        y[j] = x[j];
      }
      for (int k{0}; k < 2; k++) {
        xoffset = (k + 1) * vstride;
        for (int j{0}; j < vstride; j++) {
          y[j] = y[j] + x[xoffset + j];
        }
      }
    }
    tf.set_size(y.size(0));
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      tf[xoffset] = (y[xoffset] == 3);
    }
    vstride = 0;
    xoffset = tf.size(0);
    for (int k{0}; k < xoffset; k++) {
      if (tf[k]) {
        vstride++;
      }
    }
    if (vstride != 0) {
      xoffset = tf.size(0) - 1;
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          vstride++;
        }
      }
      r.set_size(vstride);
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          r[vstride] = j + 1;
          vstride++;
        }
      }
      b_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_this[xoffset] = Location[(r[xoffset] + Location.size(0) * 2) - 1];
      }
      c_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        c_this[xoffset] = Location[(r[xoffset] + Location.size(0) * 2) - 1];
      }
      ZLimitsInternal.set_size(1, 2);
      ZLimitsInternal[0] = internal::minimum(b_this);
      ZLimitsInternal[1] = internal::maximum(c_this);
    } else {
      ZLimitsInternal.set_size(0, 2);
    }
  }
  zlim.set_size(ZLimitsInternal.size(0), ZLimitsInternal.size(1));
  vstride = ZLimitsInternal.size(0) * ZLimitsInternal.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    zlim[xoffset] = ZLimitsInternal[xoffset];
  }
}

pointCloud *
pointCloud::init(const ::coder::array<double, 2U> &varargin_1,
                 const ::coder::array<unsigned char, 2U> &varargin_3,
                 const ::coder::array<double, 2U> &varargin_5,
                 const ::coder::array<double, 1U> &varargin_7,
                 ::coder::vision::internal::codegen::Kdtree *iobj_0)
{
  pointCloud *this_;
  int loop_ub;
  this_ = this;
  this_->Location.set_size(varargin_1.size(0), 3);
  loop_ub = varargin_1.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    this_->Location[i] = varargin_1[i];
  }
  this_->Color.set_size(varargin_3.size(0), varargin_3.size(1));
  loop_ub = varargin_3.size(0) * varargin_3.size(1);
  for (int i{0}; i < loop_ub; i++) {
    this_->Color[i] = varargin_3[i];
  }
  this_->Normal.set_size(varargin_5.size(0), varargin_5.size(1));
  loop_ub = varargin_5.size(0) * varargin_5.size(1);
  for (int i{0}; i < loop_ub; i++) {
    this_->Normal[i] = varargin_5[i];
  }
  this_->Intensity.set_size(varargin_7.size(0));
  loop_ub = varargin_7.size(0);
  for (int i{0}; i < loop_ub; i++) {
    this_->Intensity[i] = varargin_7[i];
  }
  pointclouds::internal::codegen::pc::pointCloudArray r;
  this_->RangeData.set_size(0, 0);
  this_->PointCloudArrayData.set_size(1, 1);
  this_->PointCloudArrayData[0] = r;
  this_->XLimitsInternal.set_size(0, 0);
  this_->YLimitsInternal.set_size(0, 0);
  this_->ZLimitsInternal.set_size(0, 0);
  this_->Kdtree = iobj_0;
  this_->matlabCodegenIsDeleted = false;
  return this_;
}

void pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

void b_pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

void b_pointCloud::multiQueryKNNSearchImpl(
    const ::coder::array<double, 2U> &points,
    ::coder::array<unsigned int, 2U> &indices,
    ::coder::array<double, 2U> &dists, ::coder::array<unsigned int, 1U> &valid)
{
  ::coder::vision::internal::codegen::b_Kdtree *b_this;
  array<double, 2U> X;
  array<double, 2U> cutDim;
  array<double, 2U> cutVal;
  array<double, 2U> inputData;
  array<double, 2U> leftChild;
  array<double, 2U> lowBounds;
  array<double, 2U> rightChild;
  array<double, 2U> upBounds;
  array<double, 1U> c_this;
  array<double, 1U> nodeStack;
  array<int, 2U> noNanCol;
  array<unsigned int, 1U> nodeIdxThis;
  array<int, 1U> r1;
  array<bool, 2U> leafNode;
  array<bool, 2U> r;
  array<bool, 1U> wasNaNY;
  b_struct_T pq;
  double b_pRadIn[3];
  double pRadIn[3];
  int i;
  int nxout;
  int yk;
  yk = Location.size(0);
  if (!b_Kdtree->IsIndexed) {
    b_this = b_Kdtree;
    inputData.set_size(Location.size(0), 3);
    nxout = Location.size(0) * 3;
    for (i = 0; i < nxout; i++) {
      inputData[i] = Location[i];
    }
    b_this->buildIndex(inputData);
    b_this->IsIndexed = true;
  }
  b_this = b_Kdtree;
  X.set_size(b_this->InputData.size(0), b_this->InputData.size(1));
  nxout = b_this->InputData.size(0) * b_this->InputData.size(1);
  for (i = 0; i < nxout; i++) {
    X[i] = b_this->InputData[i];
  }
  r.set_size(points.size(0), 3);
  nxout = points.size(0) * 3;
  for (i = 0; i < nxout; i++) {
    r[i] = std::isnan(points[i]);
  }
  b_any(r, wasNaNY);
  yk = static_cast<int>(std::fmin(std::fmin(5.0, static_cast<double>(yk)),
                                  static_cast<double>(X.size(0))));
  if ((points.size(0) == 0) || (yk == 0)) {
    indices.set_size(yk, points.size(0));
    nxout = yk * points.size(0);
    for (i = 0; i < nxout; i++) {
      indices[i] = 0U;
    }
    dists.set_size(yk, points.size(0));
    nxout = yk * points.size(0);
    for (i = 0; i < nxout; i++) {
      dists[i] = 0.0;
    }
    valid.set_size(points.size(0));
    nxout = points.size(0);
    for (i = 0; i < nxout; i++) {
      valid[i] = 0U;
    }
  } else {
    double startNode;
    int numNN;
    startNode = b_this->NxNoNaN;
    if ((!std::isnan(startNode)) && (yk > startNode)) {
      numNN = static_cast<int>(startNode);
    } else {
      numNN = yk;
    }
    if (numNN > 0) {
      indices.set_size(yk, points.size(0));
      nxout = yk * points.size(0);
      for (i = 0; i < nxout; i++) {
        indices[i] = 0U;
      }
      dists.set_size(yk, points.size(0));
      nxout = yk * points.size(0);
      for (i = 0; i < nxout; i++) {
        dists[i] = 0.0;
      }
      valid.set_size(points.size(0));
      nxout = points.size(0);
      for (i = 0; i < nxout; i++) {
        valid[i] = 0U;
      }
      noNanCol.set_size(1, numNN);
      noNanCol[0] = 1;
      yk = 1;
      for (int k{2}; k <= numNN; k++) {
        yk++;
        noNanCol[k - 1] = yk;
      }
      i = points.size(0);
      for (int j{0}; j < i; j++) {
        if (!wasNaNY[j]) {
          double b_points[3];
          bool ballIsWithinBounds;
          cutDim.set_size(b_this->CutDim.size(0), b_this->CutDim.size(1));
          nxout = b_this->CutDim.size(0) * b_this->CutDim.size(1);
          for (yk = 0; yk < nxout; yk++) {
            cutDim[yk] = b_this->CutDim[yk];
          }
          cutVal.set_size(b_this->CutVal.size(0), b_this->CutVal.size(1));
          nxout = b_this->CutVal.size(0) * b_this->CutVal.size(1);
          for (yk = 0; yk < nxout; yk++) {
            cutVal[yk] = b_this->CutVal[yk];
          }
          leafNode.set_size(b_this->LeafNode.size(0), b_this->LeafNode.size(1));
          nxout = b_this->LeafNode.size(0) * b_this->LeafNode.size(1);
          for (yk = 0; yk < nxout; yk++) {
            leafNode[yk] = b_this->LeafNode[yk];
          }
          leftChild.set_size(b_this->LeftChild.size(0),
                             b_this->LeftChild.size(1));
          nxout = b_this->LeftChild.size(0) * b_this->LeftChild.size(1);
          for (yk = 0; yk < nxout; yk++) {
            leftChild[yk] = b_this->LeftChild[yk];
          }
          rightChild.set_size(b_this->RightChild.size(0),
                              b_this->RightChild.size(1));
          nxout = b_this->RightChild.size(0) * b_this->RightChild.size(1);
          for (yk = 0; yk < nxout; yk++) {
            rightChild[yk] = b_this->RightChild[yk];
          }
          startNode = 1.0;
          while (!leafNode[static_cast<int>(startNode) - 1]) {
            if (points[j + points.size(0) *
                               (static_cast<int>(
                                    cutDim[static_cast<int>(startNode) - 1]) -
                                1)] <=
                cutVal[static_cast<int>(startNode) - 1]) {
              startNode = leftChild[static_cast<int>(startNode) - 1];
            } else {
              startNode = rightChild[static_cast<int>(startNode) - 1];
            }
          }
          pq.D.set_size(0);
          pq.b_I.set_size(0);
          vision::internal::codegen::Kdtree::getNodeFromArray(
              b_this->IdxAll, b_this->IdxDim, startNode, nodeIdxThis);
          b_points[0] = points[j];
          b_points[1] = points[j + points.size(0)];
          b_points[2] = points[j + points.size(0) * 2];
          vision::internal::codegen::Kdtree::searchNode(
              X, b_points, nodeIdxThis, numNN, &pq);
          if (pq.D.size(0) != 0) {
            nxout = b_this->LowerBounds.size(1);
            lowBounds.set_size(1, nxout);
            for (yk = 0; yk < nxout; yk++) {
              lowBounds[yk] =
                  b_this->LowerBounds[(static_cast<int>(startNode) +
                                       b_this->LowerBounds.size(0) * yk) -
                                      1];
            }
            nxout = b_this->UpperBounds.size(1);
            upBounds.set_size(1, nxout);
            for (yk = 0; yk < nxout; yk++) {
              upBounds[yk] =
                  b_this->UpperBounds[(static_cast<int>(startNode) +
                                       b_this->UpperBounds.size(0) * yk) -
                                      1];
            }
            if (lowBounds.size(1) == 3) {
              pRadIn[0] = points[j] - lowBounds[0];
              pRadIn[1] = points[j + points.size(0)] - lowBounds[1];
              pRadIn[2] = points[j + points.size(0) * 2] - lowBounds[2];
            } else {
              b_binary_expand_op(pRadIn, points, j, lowBounds);
            }
            if (upBounds.size(1) == 3) {
              b_pRadIn[0] = points[j] - upBounds[0];
              b_pRadIn[1] = points[j + points.size(0)] - upBounds[1];
              b_pRadIn[2] = points[j + points.size(0) * 2] - upBounds[2];
            } else {
              b_binary_expand_op(b_pRadIn, points, j, upBounds);
            }
            b_points[0] = pRadIn[0] * pRadIn[0];
            b_points[1] = pRadIn[1] * pRadIn[1];
            b_points[2] = pRadIn[2] * pRadIn[2];
            if (internal::minimum(b_points) <= pq.D[pq.D.size(0) - 1]) {
              ballIsWithinBounds = false;
            } else {
              b_points[0] = b_pRadIn[0] * b_pRadIn[0];
              b_points[1] = b_pRadIn[1] * b_pRadIn[1];
              b_points[2] = b_pRadIn[2] * b_pRadIn[2];
              if (internal::minimum(b_points) <= pq.D[pq.D.size(0) - 1]) {
                ballIsWithinBounds = false;
              } else {
                ballIsWithinBounds = true;
              }
            }
          } else {
            ballIsWithinBounds = false;
          }
          if ((pq.D.size(0) != numNN) || (!ballIsWithinBounds)) {
            nodeStack.set_size(1);
            nodeStack[0] = 1.0;
            while (nodeStack.size(0) != 0) {
              double currentNode;
              double sumDist;
              bool exitg1;
              currentNode = nodeStack[0];
              yk = nodeStack.size(0);
              nxout = nodeStack.size(0) - 1;
              for (int k{0}; k < nxout; k++) {
                nodeStack[k] = nodeStack[k + 1];
              }
              if (nxout < 1) {
                nxout = 0;
              } else {
                nxout = yk - 1;
              }
              nodeStack.set_size(nxout);
              nxout = b_this->LowerBounds.size(1);
              lowBounds.set_size(1, nxout);
              for (yk = 0; yk < nxout; yk++) {
                lowBounds[yk] =
                    b_this->LowerBounds[(static_cast<int>(currentNode) +
                                         b_this->LowerBounds.size(0) * yk) -
                                        1];
              }
              nxout = b_this->UpperBounds.size(1);
              upBounds.set_size(1, nxout);
              for (yk = 0; yk < nxout; yk++) {
                upBounds[yk] =
                    b_this->UpperBounds[(static_cast<int>(currentNode) +
                                         b_this->UpperBounds.size(0) * yk) -
                                        1];
              }
              ballIsWithinBounds = true;
              sumDist = 0.0;
              yk = 0;
              exitg1 = false;
              while ((!exitg1) && (yk <= X.size(1) - 1)) {
                if (points[j + points.size(0) * yk] < lowBounds[yk]) {
                  double c_pRadIn;
                  c_pRadIn = points[j + points.size(0) * yk] - lowBounds[yk];
                  sumDist += c_pRadIn * c_pRadIn;
                } else if (points[j + points.size(0) * yk] > upBounds[yk]) {
                  double c_pRadIn;
                  c_pRadIn = points[j + points.size(0) * yk] - upBounds[yk];
                  sumDist += c_pRadIn * c_pRadIn;
                }
                if (sumDist > pq.D[pq.D.size(0) - 1]) {
                  ballIsWithinBounds = false;
                  exitg1 = true;
                } else {
                  yk++;
                }
              }
              if ((pq.D.size(0) < numNN) || ballIsWithinBounds) {
                if (!b_this->LeafNode[static_cast<int>(currentNode) - 1]) {
                  yk = static_cast<int>(
                           b_this->CutDim[static_cast<int>(currentNode) - 1]) -
                       1;
                  if (points[j + points.size(0) * yk] <=
                      b_this->CutVal[static_cast<int>(currentNode) - 1]) {
                    c_this.set_size(nodeStack.size(0) + 2);
                    c_this[0] =
                        b_this->LeftChild[static_cast<int>(currentNode) - 1];
                    c_this[1] =
                        b_this->RightChild[static_cast<int>(currentNode) - 1];
                    nxout = nodeStack.size(0);
                    for (yk = 0; yk < nxout; yk++) {
                      c_this[yk + 2] = nodeStack[yk];
                    }
                    nodeStack.set_size(c_this.size(0));
                    nxout = c_this.size(0);
                    for (yk = 0; yk < nxout; yk++) {
                      nodeStack[yk] = c_this[yk];
                    }
                  } else {
                    c_this.set_size(nodeStack.size(0) + 2);
                    c_this[0] =
                        b_this->RightChild[static_cast<int>(currentNode) - 1];
                    c_this[1] =
                        b_this->LeftChild[static_cast<int>(currentNode) - 1];
                    nxout = nodeStack.size(0);
                    for (yk = 0; yk < nxout; yk++) {
                      c_this[yk + 2] = nodeStack[yk];
                    }
                    nodeStack.set_size(c_this.size(0));
                    nxout = c_this.size(0);
                    for (yk = 0; yk < nxout; yk++) {
                      nodeStack[yk] = c_this[yk];
                    }
                  }
                } else if (currentNode != startNode) {
                  vision::internal::codegen::Kdtree::getNodeFromArray(
                      b_this->IdxAll, b_this->IdxDim, currentNode, nodeIdxThis);
                  b_points[0] = points[j];
                  b_points[1] = points[j + points.size(0)];
                  b_points[2] = points[j + points.size(0) * 2];
                  vision::internal::codegen::Kdtree::searchNode(
                      X, b_points, nodeIdxThis, numNN, &pq);
                }
              }
            }
          }
          nxout = noNanCol.size(1);
          r1.set_size(noNanCol.size(1));
          for (yk = 0; yk < nxout; yk++) {
            r1[yk] = noNanCol[yk];
          }
          nxout = noNanCol.size(1);
          for (yk = 0; yk < nxout; yk++) {
            indices[(r1[yk] + indices.size(0) * j) - 1] = pq.b_I[yk];
            dists[(r1[yk] + dists.size(0) * j) - 1] = pq.D[yk];
          }
          valid[j] = static_cast<unsigned int>(noNanCol.size(1));
        }
      }
    } else {
      indices.set_size(0, points.size(0));
      dists.set_size(0, points.size(0));
      valid.set_size(points.size(0));
      nxout = points.size(0);
      for (i = 0; i < nxout; i++) {
        valid[i] = 0U;
      }
    }
  }
}

void pointCloud::subsetImpl(const ::coder::array<bool, 1U> &indices,
                            ::coder::array<double, 2U> &loc,
                            ::coder::array<unsigned char, 2U> &c,
                            ::coder::array<double, 2U> &nv,
                            ::coder::array<double, 1U> &intensity,
                            ::coder::array<double, 2U> &r) const
{
  array<int, 1U> b_r;
  array<int, 1U> r1;
  array<int, 1U> r2;
  array<int, 1U> r3;
  array<int, 1U> r4;
  int end;
  int trueCount;
  if (Location.size(0) != 0) {
    end = indices.size(0) - 1;
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        trueCount++;
      }
    }
    b_r.set_size(trueCount);
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        b_r[trueCount] = i + 1;
        trueCount++;
      }
    }
    loc.set_size(b_r.size(0), 3);
    trueCount = b_r.size(0);
    for (int i{0}; i < 3; i++) {
      for (int b_i{0}; b_i < trueCount; b_i++) {
        loc[b_i + loc.size(0) * i] =
            Location[(b_r[b_i] + Location.size(0) * i) - 1];
      }
    }
  } else {
    loc.set_size(0, 3);
  }
  if ((Color.size(0) != 0) && (Color.size(1) != 0)) {
    end = indices.size(0) - 1;
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        trueCount++;
      }
    }
    r1.set_size(trueCount);
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        r1[trueCount] = i + 1;
        trueCount++;
      }
    }
    trueCount = Color.size(1);
    c.set_size(r1.size(0), trueCount);
    for (int i{0}; i < trueCount; i++) {
      end = r1.size(0);
      for (int b_i{0}; b_i < end; b_i++) {
        c[b_i + c.size(0) * i] = Color[(r1[b_i] + Color.size(0) * i) - 1];
      }
    }
  } else {
    c.set_size(0, 0);
  }
  if ((Normal.size(0) != 0) && (Normal.size(1) != 0)) {
    end = indices.size(0) - 1;
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        trueCount++;
      }
    }
    r2.set_size(trueCount);
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        r2[trueCount] = i + 1;
        trueCount++;
      }
    }
    trueCount = Normal.size(1);
    nv.set_size(r2.size(0), trueCount);
    for (int i{0}; i < trueCount; i++) {
      end = r2.size(0);
      for (int b_i{0}; b_i < end; b_i++) {
        nv[b_i + nv.size(0) * i] = Normal[(r2[b_i] + Normal.size(0) * i) - 1];
      }
    }
  } else {
    nv.set_size(0, 0);
  }
  if (Intensity.size(0) != 0) {
    end = indices.size(0) - 1;
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        trueCount++;
      }
    }
    r3.set_size(trueCount);
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        r3[trueCount] = i + 1;
        trueCount++;
      }
    }
    intensity.set_size(r3.size(0));
    trueCount = r3.size(0);
    for (int i{0}; i < trueCount; i++) {
      intensity[i] = Intensity[r3[i] - 1];
    }
  } else {
    intensity.set_size(0);
  }
  if ((RangeData.size(0) != 0) && (RangeData.size(1) != 0)) {
    end = indices.size(0) - 1;
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        trueCount++;
      }
    }
    r4.set_size(trueCount);
    trueCount = 0;
    for (int i{0}; i <= end; i++) {
      if (indices[i]) {
        r4[trueCount] = i + 1;
        trueCount++;
      }
    }
    trueCount = RangeData.size(1);
    r.set_size(r4.size(0), trueCount);
    for (int i{0}; i < trueCount; i++) {
      end = r4.size(0);
      for (int b_i{0}; b_i < end; b_i++) {
        r[b_i + r.size(0) * i] =
            RangeData[(r4[b_i] + RangeData.size(0) * i) - 1];
      }
    }
  } else {
    r.set_size(0, 0);
  }
}

} // namespace coder

// End of code generation (pointCloud.cpp)
