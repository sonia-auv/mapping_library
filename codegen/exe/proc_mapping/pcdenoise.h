//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcdenoise.h
//
// Code generation for function 'pcdenoise'
//

#ifndef PCDENOISE_H
#define PCDENOISE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class b_pointCloud;

namespace vision {
namespace internal {
namespace codegen {
class Kdtree;

}
} // namespace internal
} // namespace vision
class pointCloud;

} // namespace coder

// Function Declarations
namespace coder {
pointCloud *pcdenoise(b_pointCloud *ptCloudIn,
                      vision::internal::codegen::Kdtree *iobj_0,
                      pointCloud *iobj_1);

}

#endif
// End of code generation (pcdenoise.h)
