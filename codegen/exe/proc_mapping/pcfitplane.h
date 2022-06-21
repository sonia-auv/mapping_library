//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcfitplane.h
//
// Code generation for function 'pcfitplane'
//

#ifndef PCFITPLANE_H
#define PCFITPLANE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class planeModel;

} // namespace coder

// Function Declarations
namespace coder {
void pcfitplane(const pointCloud *varargin_1, planeModel *iobj_0,
                planeModel **model, ::coder::array<double, 1U> &inlierIndices,
                ::coder::array<double, 1U> &outlierIndices);

}

#endif
// End of code generation (pcfitplane.h)
