//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// quatUtilities.h
//
// Code generation for function 'quatUtilities'
//

#ifndef QUATUTILITIES_H
#define QUATUTILITIES_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class planeModel;

class pointCloud;

} // namespace coder

// Type Definitions
class quatUtilities {
public:
  static void getOrientedPointOnPlanarFace(const coder::planeModel *model,
                                           coder::pointCloud *plane,
                                           double p[3], double q[4]);
};

#endif
// End of code generation (quatUtilities.h)
