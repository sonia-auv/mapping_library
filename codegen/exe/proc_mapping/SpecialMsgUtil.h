//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// SpecialMsgUtil.h
//
// Code generation for function 'SpecialMsgUtil'
//

#ifndef SPECIALMSGUTIL_H
#define SPECIALMSGUTIL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
namespace internal {
class SpecialMsgUtil {
public:
  static void readFieldFromData(const ::coder::array<unsigned char, 1U> &data,
                                const ::coder::array<double, 2U> &byteIdx,
                                const ::coder::array<bool, 2U> &pointIdxIsValid,
                                ::coder::array<float, 1U> &fieldPoints);
};

} // namespace internal
} // namespace ros
} // namespace coder

#endif
// End of code generation (SpecialMsgUtil.h)
