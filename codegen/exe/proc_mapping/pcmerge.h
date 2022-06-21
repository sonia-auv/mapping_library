//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcmerge.h
//
// Code generation for function 'pcmerge'
//

#ifndef PCMERGE_H
#define PCMERGE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

namespace vision {
namespace internal {
namespace codegen {
class Kdtree;

}
} // namespace internal
} // namespace vision
} // namespace coder

// Function Declarations
namespace coder {
pointCloud *pcmerge(const pointCloud *ptCloudA, const pointCloud *ptCloudB,
                    vision::internal::codegen::Kdtree *iobj_0,
                    pointCloud *iobj_1);

}

#endif
// End of code generation (pcmerge.h)
