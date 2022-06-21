//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PointCloud2Types.h
//
// Code generation for function 'PointCloud2Types'
//

#ifndef POINTCLOUD2TYPES_H
#define POINTCLOUD2TYPES_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
namespace msg {
namespace sensor_msgs {
namespace internal {
class PointCloud2Types {
public:
  static void rosToMATLABType(unsigned char type,
                              ::coder::array<char, 2U> &mlType,
                              double *numBytes);
};

} // namespace internal
} // namespace sensor_msgs
} // namespace msg
} // namespace ros
} // namespace coder

#endif
// End of code generation (PointCloud2Types.h)
