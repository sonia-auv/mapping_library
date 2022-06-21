//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pointCloud.h
//
// Code generation for function 'pointCloud'
//

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// Include files
#include "Kdtree.h"
#include "pointCloudArray.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class pointCloud {
public:
  void subsetImpl(const ::coder::array<bool, 1U> &indices,
                  ::coder::array<double, 2U> &loc,
                  ::coder::array<unsigned char, 2U> &c,
                  ::coder::array<double, 2U> &nv,
                  ::coder::array<double, 1U> &intensity,
                  ::coder::array<double, 2U> &r) const;
  pointCloud *init(const ::coder::array<double, 2U> &varargin_1,
                   const ::coder::array<unsigned char, 2U> &varargin_3,
                   const ::coder::array<double, 2U> &varargin_5,
                   const ::coder::array<double, 1U> &varargin_7,
                   ::coder::vision::internal::codegen::Kdtree *iobj_0);
  pointCloud *b_select(const ::coder::array<double, 1U> &varargin_1,
                       ::coder::vision::internal::codegen::Kdtree *iobj_0,
                       pointCloud *iobj_1);
  void get_XLimits(::coder::array<double, 2U> &xlim);
  void get_YLimits(::coder::array<double, 2U> &ylim);
  void get_ZLimits(::coder::array<double, 2U> &zlim);
  void extractValidPoints(::coder::array<double, 2U> &location,
                          ::coder::array<unsigned char, 2U> &color,
                          ::coder::array<double, 2U> &normals,
                          ::coder::array<double, 1U> &intensity,
                          ::coder::array<double, 2U> &rangeData) const;
  void matlabCodegenDestructor();
  ~pointCloud();
  pointCloud();
  bool matlabCodegenIsDeleted;
  array<double, 2U> Location;
  array<unsigned char, 2U> Color;
  array<double, 2U> Normal;
  array<double, 1U> Intensity;
  array<double, 2U> RangeData;
  vision::internal::codegen::Kdtree *Kdtree;
  array<double, 2U> XLimitsInternal;
  array<double, 2U> YLimitsInternal;
  array<double, 2U> ZLimitsInternal;
  array<pointclouds::internal::codegen::pc::pointCloudArray, 2U>
      PointCloudArrayData;
};

class b_pointCloud {
public:
  void multiQueryKNNSearchImpl(const ::coder::array<double, 2U> &points,
                               ::coder::array<unsigned int, 2U> &indices,
                               ::coder::array<double, 2U> &dists,
                               ::coder::array<unsigned int, 1U> &valid);
  void matlabCodegenDestructor();
  ~b_pointCloud();
  b_pointCloud();
  bool matlabCodegenIsDeleted;
  array<double, 2U> Location;
  array<unsigned char, 2U> Color;
  array<double, 2U> Normal;
  array<double, 1U> Intensity;
  array<double, 2U> RangeData;
  vision::internal::codegen::b_Kdtree *b_Kdtree;
  array<pointclouds::internal::codegen::pc::pointCloudArray, 2U>
      PointCloudArrayData;
  vision::internal::codegen::b_Kdtree _pobj0;
};

} // namespace coder

#endif
// End of code generation (pointCloud.h)
