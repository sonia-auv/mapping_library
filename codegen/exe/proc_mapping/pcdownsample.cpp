//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcdownsample.cpp
//
// Code generation for function 'pcdownsample'
//

// Include files
#include "pcdownsample.h"
#include "Kdtree.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "rt_nonfinite.h"
#include "voxelGridFilter.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
b_pointCloud *pcdownsample(const pointCloud *ptCloudIn, b_pointCloud *iobj_0)
{
  b_pointCloud *ptCloudOut;
  pointCloud pc;
  pointCloud *b_ptCloudOut;
  vision::internal::codegen::Kdtree lobj_1;
  array<double, 2U> b_rangeData;
  array<double, 2U> location;
  array<double, 2U> normals;
  array<double, 2U> rangeData;
  array<double, 2U> tempNV;
  array<double, 1U> tempI;
  array<int, 1U> y;
  array<unsigned char, 2U> C_;
  array<unsigned char, 2U> color;
  array<bool, 2U> r;
  array<bool, 2U> x;
  array<bool, 1U> b_y;
  int vstride;
  int xoffset;
  pc.matlabCodegenIsDeleted = true;
  x.set_size(ptCloudIn->Location.size(0), 3);
  vstride = ptCloudIn->Location.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    x[xoffset] = std::isinf(ptCloudIn->Location[xoffset]);
  }
  r.set_size(ptCloudIn->Location.size(0), 3);
  vstride = ptCloudIn->Location.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    r[xoffset] = std::isnan(ptCloudIn->Location[xoffset]);
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
  ptCloudIn->subsetImpl(b_y, location, color, normals, tempI, rangeData);
  b_ptCloudOut = pc.init(location, color, normals, tempI, &lobj_1);
  b_ptCloudOut->RangeData.set_size(rangeData.size(0), rangeData.size(1));
  vstride = rangeData.size(0) * rangeData.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    b_ptCloudOut->RangeData[xoffset] = rangeData[xoffset];
  }
  vision::internal::codegen::pc::voxelGridFilter(
      pc.Location, pc.Color, pc.Normal, pc.Intensity, pc.RangeData, location,
      C_, tempNV, tempI, b_rangeData);
  pc.matlabCodegenDestructor();
  ptCloudOut = iobj_0;
  iobj_0->Location.set_size(location.size(0), 3);
  vstride = location.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    iobj_0->Location[xoffset] = location[xoffset];
  }
  iobj_0->Color.set_size(C_.size(0), 3);
  vstride = C_.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    iobj_0->Color[xoffset] = C_[xoffset];
  }
  iobj_0->Normal.set_size(tempNV.size(0), 3);
  vstride = tempNV.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    iobj_0->Normal[xoffset] = tempNV[xoffset];
  }
  iobj_0->Intensity.set_size(tempI.size(0));
  vstride = tempI.size(0);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    iobj_0->Intensity[xoffset] = tempI[xoffset];
  }
  pointclouds::internal::codegen::pc::pointCloudArray r1;
  iobj_0->RangeData.set_size(0, 0);
  iobj_0->PointCloudArrayData.set_size(1, 1);
  iobj_0->PointCloudArrayData[0] = r1;
  iobj_0->_pobj0.InputData.set_size(0, 0);
  iobj_0->_pobj0.NxNoNaN = 0.0;
  iobj_0->_pobj0.CutDim.set_size(0, 0);
  iobj_0->_pobj0.CutVal.set_size(0, 0);
  iobj_0->_pobj0.LowerBounds.set_size(0, 0);
  iobj_0->_pobj0.UpperBounds.set_size(0, 0);
  iobj_0->_pobj0.LeftChild.set_size(0, 0);
  iobj_0->_pobj0.RightChild.set_size(0, 0);
  iobj_0->_pobj0.LeafNode.set_size(0, 0);
  iobj_0->_pobj0.IdxAll.set_size(0);
  iobj_0->_pobj0.IdxDim.set_size(0);
  iobj_0->_pobj0.IsIndexed = false;
  iobj_0->b_Kdtree = &iobj_0->_pobj0;
  iobj_0->matlabCodegenIsDeleted = false;
  iobj_0->RangeData.set_size(b_rangeData.size(0), 3);
  vstride = b_rangeData.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    iobj_0->RangeData[xoffset] = b_rangeData[xoffset];
  }
  return ptCloudOut;
}

} // namespace coder

// End of code generation (pcdownsample.cpp)
