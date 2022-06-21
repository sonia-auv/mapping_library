//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// voxelGridFilter.h
//
// Code generation for function 'voxelGridFilter'
//

#ifndef VOXELGRIDFILTER_H
#define VOXELGRIDFILTER_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
namespace pc {
void voxelGridFilter(const ::coder::array<double, 2U> &location,
                     const ::coder::array<unsigned char, 2U> &color,
                     const ::coder::array<double, 2U> &normal,
                     const ::coder::array<double, 1U> &intensity,
                     const ::coder::array<double, 2U> &rangeData,
                     ::coder::array<double, 2U> &filteredLocation,
                     ::coder::array<unsigned char, 2U> &filteredColor,
                     ::coder::array<double, 2U> &filteredNormal,
                     ::coder::array<double, 1U> &filteredIntensity,
                     ::coder::array<double, 2U> &filteredRangeData);

void voxelGridFilter(const ::coder::array<double, 2U> &location,
                     const ::coder::array<unsigned char, 2U> &color,
                     const ::coder::array<double, 2U> &normal,
                     const ::coder::array<double, 1U> &intensity,
                     const ::coder::array<double, 2U> &rangeData,
                     const ::coder::array<double, 2U> &rangeIn,
                     ::coder::array<double, 2U> &filteredLocation,
                     ::coder::array<unsigned char, 2U> &filteredColor,
                     ::coder::array<double, 2U> &filteredNormal,
                     ::coder::array<double, 1U> &filteredIntensity,
                     ::coder::array<double, 2U> &filteredRangeData);

} // namespace pc
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder

#endif
// End of code generation (voxelGridFilter.h)
