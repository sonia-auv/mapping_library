//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// voxelGridFilter.cpp
//
// Code generation for function 'voxelGridFilter'
//

// Include files
#include "voxelGridFilter.h"
#include "minOrMax.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "stack1.h"
#include "coder_array.h"
#include "rt_defines.h"
#include <cmath>
#include <math.h>
#include <string.h>

// Function Declarations
static void Double2MultiWord(double u1, unsigned int y[]);

static int MultiWord2sLong(const unsigned int u[]);

namespace coder {
namespace vision {
namespace internal {
namespace codegen {
namespace pc {
static void insertion(::coder::array<uint64m_T, 2U> &x, int xstart, int xend,
                      int arrLen);

static void sortVoxelIndex(::coder::array<uint64m_T, 2U> &x, int xend,
                           int arrLen);

static void
voxelGridAlgImpl(const ::coder::array<double, 2U> &location,
                 unsigned int nPoints,
                 const ::coder::array<unsigned char, 2U> &color,
                 const ::coder::array<double, 2U> &normal,
                 const ::coder::array<double, 1U> &intensity,
                 const ::coder::array<double, 2U> &rangeData,
                 const ::coder::array<double, 2U> &range, bool needXYZLimits,
                 ::coder::array<double, 2U> &filteredLocation,
                 ::coder::array<unsigned char, 2U> &filteredColor,
                 ::coder::array<double, 2U> &filteredNormal,
                 ::coder::array<double, 1U> &filteredIntensity,
                 ::coder::array<double, 2U> &filteredRangeData,
                 const ::coder::array<uint64m_T, 2U> &ptrIndexVector, int nOut,
                 bool needColorFlag, bool needNormalFlag,
                 bool needIntensityFlag, bool needRangeFlag);

} // namespace pc
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder
static double rt_atan2d_snf(double u0, double u1);

static void sLong2uMultiWordSat(int u1, unsigned int y[]);

static void sMultiWord2uMultiWordSat(const unsigned int u1[], unsigned int y[]);

static void uLong2MultiWord(unsigned int u, unsigned int y[]);

static double uMultiWord2Double(const unsigned int u1[]);

static int uMultiWordCmp(const unsigned int u1[], const unsigned int u2[]);

static bool uMultiWordEq(const unsigned int u1[], const unsigned int u2[]);

static bool uMultiWordLt(const unsigned int u1[], const unsigned int u2[]);

// Function Definitions
static void Double2MultiWord(double u1, unsigned int y[])
{
  double b_yn;
  int cb;
  int currExp;
  int msl;
  int prevExp;
  bool isNegative;
  isNegative = (u1 < 0.0);
  b_yn = frexp(u1, &currExp);
  if (currExp <= 0) {
    msl = -1;
  } else {
    msl = (currExp - 1) / 32;
  }
  cb = 1;
  for (int i{msl + 1}; i < 2; i++) {
    y[i] = 0U;
  }
  if (isNegative) {
    b_yn = -b_yn;
  }
  prevExp = 32 * msl;
  for (int i{msl}; i >= 0; i--) {
    double yd;
    b_yn = std::ldexp(b_yn, currExp - prevExp);
    yd = std::floor(b_yn);
    b_yn -= yd;
    if (i < 2) {
      y[i] = static_cast<unsigned int>(yd);
    }
    currExp = prevExp;
    prevExp -= 32;
  }
  if (isNegative) {
    for (int i{0}; i < 2; i++) {
      unsigned int u1i;
      unsigned int yi;
      u1i = ~y[i];
      yi = u1i + cb;
      y[i] = yi;
      cb = (yi < u1i);
    }
  }
}

static int MultiWord2sLong(const unsigned int u[])
{
  return static_cast<int>(u[0]);
}

namespace coder {
namespace vision {
namespace internal {
namespace codegen {
namespace pc {
static void insertion(::coder::array<uint64m_T, 2U> &x, int xstart, int xend,
                      int arrLen)
{
  int i;
  i = xstart + 1;
  for (int k{i}; k <= xend; k++) {
    uint64m_T xc;
    uint64m_T xc2;
    int idx;
    xc = x[k - 1];
    xc2 = x[(k + arrLen) - 1];
    idx = k - 1;
    while ((idx >= xstart) &&
           uMultiWordLt((const unsigned int *)&xc.chunks[0U],
                        (const unsigned int *)&x[idx - 1].chunks[0U])) {
      int i1;
      x[idx] = x[idx - 1];
      i1 = idx + arrLen;
      x[i1] = x[i1 - 1];
      idx--;
    }
    x[idx] = xc;
    x[idx + arrLen] = xc2;
  }
}

static void sortVoxelIndex(::coder::array<uint64m_T, 2U> &x, int xend,
                           int arrLen)
{
  ::coder::internal::stack st;
  struct_T frame;
  struct_T r;
  struct_T r1;
  if (xend > 1) {
    if (xend <= 32) {
      insertion(x, 1, xend, arrLen);
    } else {
      frame.xstart = 1;
      frame.xend = xend;
      st.d.set_size(1);
      st.d[0] = frame;
      st.n = 1;
      while (st.n > 0) {
        int frame_tmp_tmp;
        int i;
        frame_tmp_tmp = st.n - 1;
        frame = st.d[st.n - 1];
        st.n--;
        i = frame.xend - frame.xstart;
        if (i + 1 <= 32) {
          insertion(x, frame.xstart, frame.xend, arrLen);
        } else {
          uint64m_T pivot;
          uint64m_T pivot2;
          double diff1;
          int i1;
          int j;
          int xmid;
          xmid = (frame.xstart + ((i + (i < 0)) >> 1)) - 1;
          diff1 = uMultiWord2Double(
                      (const unsigned int *)&x[frame.xstart - 1].chunks[0U]) -
                  uMultiWord2Double((const unsigned int *)&x[xmid].chunks[0U]);
          if (diff1 *
                  (uMultiWord2Double(
                       (const unsigned int *)&x[xmid].chunks[0U]) -
                   uMultiWord2Double(
                       (const unsigned int *)&x[frame.xend - 1].chunks[0U])) >
              0.0) {
            pivot = x[xmid];
            pivot2 = x[xmid + arrLen];
          } else if (diff1 * (uMultiWord2Double(
                                  (const unsigned int *)&x[frame.xstart - 1]
                                      .chunks[0U]) -
                              uMultiWord2Double(
                                  (const unsigned int *)&x[frame.xend - 1]
                                      .chunks[0U])) >
                     0.0) {
            pivot = x[frame.xend - 1];
            pivot2 = x[(frame.xend + arrLen) - 1];
            xmid = frame.xend - 1;
          } else {
            pivot = x[frame.xstart - 1];
            pivot2 = x[(frame.xstart + arrLen) - 1];
            xmid = frame.xstart - 1;
          }
          x[xmid] = x[frame.xend - 1];
          i = (frame.xend + arrLen) - 1;
          x[xmid + arrLen] = x[i];
          x[frame.xend - 1] = pivot;
          x[i] = pivot2;
          xmid = frame.xstart - 2;
          j = frame.xend - 1;
          int exitg1;
          do {
            exitg1 = 0;
            xmid++;
            while (uMultiWordLt((const unsigned int *)&x[xmid].chunks[0U],
                                (const unsigned int *)&pivot.chunks[0U])) {
              xmid++;
            }
            j--;
            while (uMultiWordLt((const unsigned int *)&pivot.chunks[0U],
                                (const unsigned int *)&x[j].chunks[0U])) {
              j--;
            }
            if (xmid + 1 >= j + 1) {
              exitg1 = 1;
            } else {
              uint64m_T t;
              uint64m_T t2;
              int t2_tmp;
              t = x[xmid];
              t2_tmp = xmid + arrLen;
              t2 = x[t2_tmp];
              x[xmid] = x[j];
              i1 = j + arrLen;
              x[t2_tmp] = x[i1];
              x[j] = t;
              x[i1] = t2;
            }
          } while (exitg1 == 0);
          x[frame.xend - 1] = x[xmid];
          i1 = xmid + arrLen;
          x[i] = x[i1];
          x[xmid] = pivot;
          x[i1] = pivot2;
          if (xmid + 2 < frame.xend) {
            r.xstart = xmid + 2;
            r.xend = frame.xend;
            if (frame_tmp_tmp == st.d.size(0)) {
              i = st.d.size(0);
              st.d.set_size(st.d.size(0) + 1);
              st.d[i] = r;
            } else {
              st.d[frame_tmp_tmp] = r;
            }
            st.n = frame_tmp_tmp + 1;
          }
          if (frame.xstart < xmid + 1) {
            r1.xstart = frame.xstart;
            r1.xend = xmid + 1;
            if (st.n == st.d.size(0)) {
              i = st.d.size(0);
              st.d.set_size(st.d.size(0) + 1);
              st.d[i] = r1;
            } else {
              st.d[st.n] = r1;
            }
            st.n++;
          }
        }
      }
    }
  }
}

static void
voxelGridAlgImpl(const ::coder::array<double, 2U> &location,
                 unsigned int nPoints,
                 const ::coder::array<unsigned char, 2U> &color,
                 const ::coder::array<double, 2U> &normal,
                 const ::coder::array<double, 1U> &intensity,
                 const ::coder::array<double, 2U> &rangeData,
                 const ::coder::array<double, 2U> &range, bool needXYZLimits,
                 ::coder::array<double, 2U> &filteredLocation,
                 ::coder::array<unsigned char, 2U> &filteredColor,
                 ::coder::array<double, 2U> &filteredNormal,
                 ::coder::array<double, 1U> &filteredIntensity,
                 ::coder::array<double, 2U> &filteredRangeData,
                 const ::coder::array<uint64m_T, 2U> &ptrIndexVector, int nOut,
                 bool needColorFlag, bool needNormalFlag,
                 bool needIntensityFlag, bool needRangeFlag)
{
  double ix;
  double nx;
  double ny;
  double nz;
  double y;
  double z;
  float cb;
  float cg;
  float cr;
  int b_index;
  int cz_tmp_tmp;
  int i;
  int matrixIndex;
  int n;
  int numPoints;
  int z_tmp;
  cr = 0.0F;
  cg = 0.0F;
  cb = 0.0F;
  ix = 0.0;
  b_index = 0;
  n = 1;
  numPoints = static_cast<int>(nPoints);
  while (n <= ptrIndexVector.size(0)) {
    double cz;
    int b_i;
    matrixIndex =
        MultiWord2sLong((const unsigned int
                             *)&ptrIndexVector[(n + ptrIndexVector.size(0)) - 1]
                            .chunks[0U]) -
        1;
    y = location[matrixIndex];
    z_tmp = matrixIndex + static_cast<int>(nPoints);
    z = location[z_tmp];
    cz_tmp_tmp = static_cast<int>(nPoints) << 1;
    cz = location[matrixIndex + cz_tmp_tmp];
    if (needColorFlag) {
      cr = color[matrixIndex];
      cg = color[z_tmp];
      cb = color[matrixIndex + cz_tmp_tmp];
    }
    if (needNormalFlag) {
      nx = normal[matrixIndex];
      ny = normal[z_tmp];
      nz = normal[matrixIndex + (static_cast<int>(nPoints) << 1)];
    }
    if (needIntensityFlag) {
      ix = intensity[matrixIndex];
    }
    b_i = n + 1;
    while (
        (b_i <= ptrIndexVector.size(0)) &&
        uMultiWordEq((const unsigned int *)&ptrIndexVector[b_i - 1].chunks[0U],
                     (const unsigned int *)&ptrIndexVector[n - 1].chunks[0U])) {
      matrixIndex =
          MultiWord2sLong(
              (const unsigned int
                   *)&ptrIndexVector[(b_i + ptrIndexVector.size(0)) - 1]
                  .chunks[0U]) -
          1;
      y += location[matrixIndex];
      z_tmp = matrixIndex + static_cast<int>(nPoints);
      z += location[z_tmp];
      cz += location[matrixIndex + cz_tmp_tmp];
      if (needColorFlag) {
        cr += static_cast<float>(color[matrixIndex]);
        cg += static_cast<float>(color[z_tmp]);
        cb += static_cast<float>(color[matrixIndex + cz_tmp_tmp]);
      }
      if (needNormalFlag) {
        nx += normal[matrixIndex];
        ny += normal[z_tmp];
        nz += normal[matrixIndex + (static_cast<int>(nPoints) << 1)];
      }
      if (needIntensityFlag) {
        ix += intensity[matrixIndex];
      }
      b_i++;
    }
    z_tmp = b_i - n;
    y /= static_cast<double>(z_tmp);
    z /= static_cast<double>(z_tmp);
    cz /= static_cast<double>(z_tmp);
    filteredLocation[b_index] = y;
    cz_tmp_tmp = b_index + nOut;
    filteredLocation[cz_tmp_tmp] = z;
    i = b_index + (nOut << 1);
    filteredLocation[i] = cz;
    if (needColorFlag) {
      float cr_tmp;
      unsigned char u;
      cr_tmp = static_cast<float>(b_i - n);
      cr /= cr_tmp;
      cg /= cr_tmp;
      cb /= cr_tmp;
      cr_tmp = std::floor(cr);
      if (cr_tmp < 256.0F) {
        if (cr_tmp >= 0.0F) {
          u = static_cast<unsigned char>(cr_tmp);
        } else {
          u = 0U;
        }
      } else if (cr_tmp >= 256.0F) {
        u = MAX_uint8_T;
      } else {
        u = 0U;
      }
      filteredColor[b_index] = u;
      cr_tmp = std::floor(cg);
      if (cr_tmp < 256.0F) {
        if (cr_tmp >= 0.0F) {
          u = static_cast<unsigned char>(cr_tmp);
        } else {
          u = 0U;
        }
      } else if (cr_tmp >= 256.0F) {
        u = MAX_uint8_T;
      } else {
        u = 0U;
      }
      filteredColor[cz_tmp_tmp] = u;
      cr_tmp = std::floor(cb);
      if (cr_tmp < 256.0F) {
        if (cr_tmp >= 0.0F) {
          u = static_cast<unsigned char>(cr_tmp);
        } else {
          u = 0U;
        }
      } else if (cr_tmp >= 256.0F) {
        u = MAX_uint8_T;
      } else {
        u = 0U;
      }
      filteredColor[i] = u;
    }
    if (needNormalFlag) {
      nx /= static_cast<double>(z_tmp);
      ny /= static_cast<double>(z_tmp);
      nz /= static_cast<double>(z_tmp);
      filteredNormal[b_index] = nx;
      filteredNormal[cz_tmp_tmp] = ny;
      filteredNormal[i] = nz;
    }
    if (needIntensityFlag) {
      ix /= static_cast<double>(z_tmp);
      filteredIntensity[b_index] = ix;
    }
    if (needRangeFlag) {
      double b_range;
      b_range = std::sqrt((y * y + z * z) + cz * cz);
      y = rt_atan2d_snf(y, z);
      if (y < 0.0) {
        y += 6.2831853071795862;
      }
      filteredRangeData[b_index] = b_range;
      filteredRangeData[cz_tmp_tmp] = std::asin(cz / b_range);
      filteredRangeData[i] = y;
    }
    n = b_i;
    b_index++;
  }
  if (!needXYZLimits) {
    for (n = 0; n < numPoints; n++) {
      matrixIndex = n + static_cast<int>(nPoints);
      y = location[matrixIndex];
      z_tmp = n + (static_cast<int>(nPoints) << 1);
      z = location[z_tmp];
      if ((location[n] < range[0]) || (location[n] > range[1]) ||
          (y < range[2]) || (y > range[3]) || (z < range[4]) ||
          (z > range[5])) {
        filteredLocation[b_index] = location[n];
        cz_tmp_tmp = b_index + nOut;
        filteredLocation[cz_tmp_tmp] = y;
        i = b_index + (nOut << 1);
        filteredLocation[i] = z;
        if (needColorFlag) {
          filteredColor[b_index] = color[n];
          filteredColor[cz_tmp_tmp] = color[matrixIndex];
          filteredColor[i] = color[z_tmp];
        }
        if (needNormalFlag) {
          filteredNormal[b_index] = normal[n];
          filteredNormal[cz_tmp_tmp] = normal[matrixIndex];
          filteredNormal[i] = normal[n + (static_cast<int>(nPoints) << 1)];
        }
        if (needIntensityFlag) {
          filteredIntensity[b_index] = intensity[n];
        }
        if (needRangeFlag) {
          filteredRangeData[b_index] = rangeData[n];
          filteredRangeData[cz_tmp_tmp] = rangeData[matrixIndex];
          filteredRangeData[i] =
              rangeData[n + (static_cast<int>(nPoints) << 1)];
        }
        b_index++;
      }
    }
  }
}

} // namespace pc
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

static void sLong2uMultiWordSat(int u1, unsigned int y[])
{
  unsigned int u;
  u = static_cast<unsigned int>(u1);
  sMultiWord2uMultiWordSat((const unsigned int *)&u, y);
}

static void sMultiWord2uMultiWordSat(const unsigned int u1[], unsigned int y[])
{
  unsigned int ys;
  if ((u1[0] & 2147483648U) != 0U) {
    ys = MAX_uint32_T;
  } else {
    ys = 0U;
  }
  if (ys != 0U) {
    ys = ~ys;
    for (int i{0}; i < 2; i++) {
      y[i] = ys;
    }
  } else {
    int i;
    y[0] = u1[0];
    i = 1;
    while (i < 2) {
      y[1] = ys;
      i = 2;
    }
  }
}

static void uLong2MultiWord(unsigned int u, unsigned int y[])
{
  y[0] = u;
  y[1] = 0U;
}

static double uMultiWord2Double(const unsigned int u1[])
{
  double y;
  int b_exp;
  y = 0.0;
  b_exp = 0;
  for (int i{0}; i < 2; i++) {
    y += std::ldexp(static_cast<double>(u1[i]), b_exp);
    b_exp += 32;
  }
  return y;
}

static int uMultiWordCmp(const unsigned int u1[], const unsigned int u2[])
{
  int i;
  int y;
  y = 0;
  i = 2;
  while ((y == 0) && (i > 0)) {
    unsigned int u1i;
    unsigned int u2i;
    i--;
    u1i = u1[i];
    u2i = u2[i];
    if (u1i != u2i) {
      if (u1i > u2i) {
        y = 1;
      } else {
        y = -1;
      }
    }
  }
  return y;
}

static bool uMultiWordEq(const unsigned int u1[], const unsigned int u2[])
{
  return uMultiWordCmp(u1, u2) == 0;
}

static bool uMultiWordLt(const unsigned int u1[], const unsigned int u2[])
{
  return uMultiWordCmp(u1, u2) < 0;
}

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
                     ::coder::array<double, 2U> &filteredRangeData)
{
  static const uint64m_T r1{
      {0U, 0U} // chunks
  };
  static const uint64m_T r2{
      {MAX_uint32_T, MAX_uint32_T} // chunks
  };
  array<uint64m_T, 2U> ptrIndexVector;
  array<double, 2U> b_location;
  uint64m_T r;
  double ix;
  double numxy;
  double nz;
  double range_idx_0;
  double range_idx_2;
  double range_idx_4;
  float cb;
  float cg;
  float cr;
  unsigned int b_i;
  unsigned int b_index;
  int c_index;
  int cz_tmp;
  int i;
  int i1;
  int matrixIndex;
  int n;
  int numOut;
  unsigned int numPoints;
  int numx;
  int ptrIndexVectorEnd;
  bool needColorFlag;
  bool needIntensityFlag;
  bool needNormalFlag;
  bool needRangeFlag;
  needColorFlag = ((color.size(0) != 0) && (color.size(1) != 0));
  needNormalFlag = ((normal.size(0) != 0) && (normal.size(1) != 0));
  needIntensityFlag = (intensity.size(0) != 0);
  needRangeFlag = ((rangeData.size(0) != 0) && (rangeData.size(1) != 0));
  numPoints = static_cast<unsigned int>(location.size(0));
  if (static_cast<unsigned int>(location.size(0)) < 1U) {
    matrixIndex = 0;
  } else {
    matrixIndex = location.size(0);
  }
  b_location.set_size(1, matrixIndex);
  for (i = 0; i < matrixIndex; i++) {
    b_location[i] = location[i];
  }
  range_idx_0 = ::coder::internal::minimum(b_location);
  if (static_cast<unsigned int>(location.size(0)) < 1U) {
    matrixIndex = 0;
  } else {
    matrixIndex = location.size(0);
  }
  b_index = static_cast<unsigned int>(location.size(0)) << 1;
  if (location.size(0) + 1U > b_index) {
    i = 0;
    i1 = 0;
  } else {
    i = location.size(0);
    i1 = static_cast<int>(b_index);
  }
  ptrIndexVectorEnd = i1 - i;
  b_location.set_size(1, ptrIndexVectorEnd);
  for (i1 = 0; i1 < ptrIndexVectorEnd; i1++) {
    b_location[i1] = location[i + i1];
  }
  range_idx_2 = ::coder::internal::minimum(b_location);
  if (location.size(0) + 1U > b_index) {
    i = 0;
    i1 = 0;
  } else {
    i = location.size(0);
    i1 = static_cast<int>(b_index);
  }
  b_i = mul_u32_sat(3U, static_cast<unsigned int>(location.size(0)));
  if (b_index + 1U > b_i) {
    numx = 0;
    cz_tmp = 0;
  } else {
    numx = static_cast<int>(b_index);
    cz_tmp = static_cast<int>(b_i);
  }
  ptrIndexVectorEnd = cz_tmp - numx;
  b_location.set_size(1, ptrIndexVectorEnd);
  for (cz_tmp = 0; cz_tmp < ptrIndexVectorEnd; cz_tmp++) {
    b_location[cz_tmp] = location[numx + cz_tmp];
  }
  range_idx_4 = ::coder::internal::minimum(b_location);
  b_location.set_size(1, matrixIndex);
  for (numx = 0; numx < matrixIndex; numx++) {
    b_location[numx] = location[numx];
  }
  numx = (static_cast<int>(
              std::floor(::coder::internal::maximum(b_location) * 20.0)) -
          static_cast<int>(std::floor(range_idx_0 * 20.0))) +
         1;
  matrixIndex = i1 - i;
  b_location.set_size(1, matrixIndex);
  for (i1 = 0; i1 < matrixIndex; i1++) {
    b_location[i1] = location[i + i1];
  }
  numxy =
      static_cast<double>(numx) *
      static_cast<double>((static_cast<int>(std::floor(
                               ::coder::internal::maximum(b_location) * 20.0)) -
                           static_cast<int>(std::floor(range_idx_2 * 20.0))) +
                          1);
  numOut = 0;
  ptrIndexVector.set_size(location.size(0), 2);
  ptrIndexVectorEnd = location.size(0);
  for (n = 0; n < ptrIndexVectorEnd; n++) {
    nz = (std::floor((location[n] - range_idx_0) * 20.0) +
          std::floor((location[n + static_cast<int>(numPoints)] - range_idx_2) *
                     20.0) *
              static_cast<double>(numx)) +
         std::floor(
             (location[n + (static_cast<int>(numPoints) << 1)] - range_idx_4) *
             20.0) *
             numxy;
    if (nz < 1.8446744073709552E+19) {
      if (nz >= 0.0) {
        Double2MultiWord(nz, (unsigned int *)&r.chunks[0U]);
      } else {
        r = r1;
      }
    } else if (nz >= 1.8446744073709552E+19) {
      r = r2;
    } else {
      r = r1;
    }
    sLong2uMultiWordSat(MultiWord2sLong((const unsigned int *)&r.chunks[0U]),
                        (unsigned int *)&ptrIndexVector[n].chunks[0U]);
    sLong2uMultiWordSat(
        n + 1,
        (unsigned int *)&ptrIndexVector[n + ptrIndexVector.size(0)].chunks[0U]);
  }
  sortVoxelIndex(ptrIndexVector, location.size(0), location.size(0));
  b_index = 1U;
  while (b_index <= numPoints) {
    b_i = b_index + 1U;
    while (
        (b_i <= numPoints) &&
        uMultiWordEq(
            (const unsigned int *)&ptrIndexVector[static_cast<int>(b_i) - 1]
                .chunks[0U],
            (const unsigned int *)&ptrIndexVector[static_cast<int>(b_index) - 1]
                .chunks[0U])) {
      b_i++;
    }
    if (static_cast<double>(b_i) - static_cast<double>(b_index) >= 1.0) {
      numOut++;
    }
    b_index = b_i;
  }
  filteredLocation.set_size(numOut, 3);
  if (needColorFlag) {
    filteredColor.set_size(numOut, 3);
  } else {
    filteredColor.set_size(0, 3);
  }
  if (needNormalFlag) {
    filteredNormal.set_size(numOut, 3);
  } else {
    filteredNormal.set_size(0, 3);
  }
  if (needIntensityFlag) {
    filteredIntensity.set_size(numOut);
  } else {
    filteredIntensity.set_size(0);
  }
  if (needRangeFlag) {
    filteredRangeData.set_size(numOut, 3);
  } else {
    filteredRangeData.set_size(0, 3);
  }
  cr = 0.0F;
  cg = 0.0F;
  cb = 0.0F;
  ix = 0.0;
  c_index = 0;
  n = 1;
  while (n <= ptrIndexVector.size(0)) {
    double cz;
    int c_i;
    matrixIndex =
        MultiWord2sLong((const unsigned int
                             *)&ptrIndexVector[(n + ptrIndexVector.size(0)) - 1]
                            .chunks[0U]) -
        1;
    range_idx_0 = location[matrixIndex];
    numx = matrixIndex + static_cast<int>(numPoints);
    range_idx_2 = location[numx];
    cz_tmp = static_cast<int>(numPoints) << 1;
    ptrIndexVectorEnd = matrixIndex + cz_tmp;
    cz = location[ptrIndexVectorEnd];
    if (needColorFlag) {
      cr = color[matrixIndex];
      cg = color[numx];
      cb = color[ptrIndexVectorEnd];
    }
    if (needNormalFlag) {
      range_idx_4 = normal[matrixIndex];
      numxy = normal[numx];
      nz = normal[ptrIndexVectorEnd];
    }
    if (needIntensityFlag) {
      ix = intensity[matrixIndex];
    }
    c_i = n + 1;
    while (
        (c_i <= ptrIndexVector.size(0)) &&
        uMultiWordEq((const unsigned int *)&ptrIndexVector[c_i - 1].chunks[0U],
                     (const unsigned int *)&ptrIndexVector[n - 1].chunks[0U])) {
      matrixIndex =
          MultiWord2sLong(
              (const unsigned int
                   *)&ptrIndexVector[(c_i + ptrIndexVector.size(0)) - 1]
                  .chunks[0U]) -
          1;
      range_idx_0 += location[matrixIndex];
      numx = matrixIndex + static_cast<int>(numPoints);
      range_idx_2 += location[numx];
      cz += location[matrixIndex + cz_tmp];
      if (needColorFlag) {
        cr += static_cast<float>(color[matrixIndex]);
        cg += static_cast<float>(color[numx]);
        cb += static_cast<float>(
            color[matrixIndex + (static_cast<int>(numPoints) << 1)]);
      }
      if (needNormalFlag) {
        range_idx_4 += normal[matrixIndex];
        numxy += normal[numx];
        nz += normal[matrixIndex + (static_cast<int>(numPoints) << 1)];
      }
      if (needIntensityFlag) {
        ix += intensity[matrixIndex];
      }
      c_i++;
    }
    ptrIndexVectorEnd = c_i - n;
    range_idx_0 /= static_cast<double>(ptrIndexVectorEnd);
    range_idx_2 /= static_cast<double>(ptrIndexVectorEnd);
    cz /= static_cast<double>(ptrIndexVectorEnd);
    filteredLocation[c_index] = range_idx_0;
    i = c_index + numOut;
    filteredLocation[i] = range_idx_2;
    i1 = c_index + (numOut << 1);
    filteredLocation[i1] = cz;
    if (needColorFlag) {
      float cr_tmp;
      unsigned char u;
      cr_tmp = static_cast<float>(c_i - n);
      cr /= cr_tmp;
      cg /= cr_tmp;
      cb /= cr_tmp;
      cr_tmp = std::floor(cr);
      if (cr_tmp < 256.0F) {
        if (cr_tmp >= 0.0F) {
          u = static_cast<unsigned char>(cr_tmp);
        } else {
          u = 0U;
        }
      } else if (cr_tmp >= 256.0F) {
        u = MAX_uint8_T;
      } else {
        u = 0U;
      }
      filteredColor[c_index] = u;
      cr_tmp = std::floor(cg);
      if (cr_tmp < 256.0F) {
        if (cr_tmp >= 0.0F) {
          u = static_cast<unsigned char>(cr_tmp);
        } else {
          u = 0U;
        }
      } else if (cr_tmp >= 256.0F) {
        u = MAX_uint8_T;
      } else {
        u = 0U;
      }
      filteredColor[i] = u;
      cr_tmp = std::floor(cb);
      if (cr_tmp < 256.0F) {
        if (cr_tmp >= 0.0F) {
          u = static_cast<unsigned char>(cr_tmp);
        } else {
          u = 0U;
        }
      } else if (cr_tmp >= 256.0F) {
        u = MAX_uint8_T;
      } else {
        u = 0U;
      }
      filteredColor[i1] = u;
    }
    if (needNormalFlag) {
      range_idx_4 /= static_cast<double>(ptrIndexVectorEnd);
      numxy /= static_cast<double>(ptrIndexVectorEnd);
      nz /= static_cast<double>(ptrIndexVectorEnd);
      filteredNormal[c_index] = range_idx_4;
      filteredNormal[i] = numxy;
      filteredNormal[i1] = nz;
    }
    if (needIntensityFlag) {
      ix /= static_cast<double>(ptrIndexVectorEnd);
      filteredIntensity[c_index] = ix;
    }
    if (needRangeFlag) {
      double range;
      range = std::sqrt(
          (range_idx_0 * range_idx_0 + range_idx_2 * range_idx_2) + cz * cz);
      range_idx_0 = rt_atan2d_snf(range_idx_0, range_idx_2);
      if (range_idx_0 < 0.0) {
        range_idx_0 += 6.2831853071795862;
      }
      filteredRangeData[c_index] = range;
      filteredRangeData[i] = std::asin(cz / range);
      filteredRangeData[i1] = range_idx_0;
    }
    n = c_i;
    c_index++;
  }
}

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
                     ::coder::array<double, 2U> &filteredRangeData)
{
  static const uint64m_T r1{
      {0U, 0U} // chunks
  };
  static const uint64m_T r2{
      {MAX_uint32_T, MAX_uint32_T} // chunks
  };
  array<uint64m_T, 2U> ptrIndexVectorCpy;
  array<double, 2U> b_location;
  array<double, 2U> range;
  uint64m_T r;
  double numxy;
  double xmin;
  double ymin;
  double zmin;
  unsigned int b_i;
  unsigned int b_index;
  int i;
  int n;
  int numOut;
  unsigned int numPoints;
  int numx;
  int ptrIndexVectorEnd;
  bool needColorFlag;
  bool needIntensityFlag;
  bool needNormalFlag;
  bool needRangeFlag;
  bool needXYZLimits;
  needXYZLimits = ((rangeIn.size(0) == 0) || (rangeIn.size(1) == 0));
  needColorFlag = ((color.size(0) != 0) && (color.size(1) != 0));
  needNormalFlag = ((normal.size(0) != 0) && (normal.size(1) != 0));
  needIntensityFlag = (intensity.size(0) != 0);
  needRangeFlag = ((rangeData.size(0) != 0) && (rangeData.size(1) != 0));
  numPoints = static_cast<unsigned int>(location.size(0));
  if (needXYZLimits) {
    range.set_size(6, 1);
    if (static_cast<unsigned int>(location.size(0)) < 1U) {
      numx = 0;
    } else {
      numx = location.size(0);
    }
    b_location.set_size(1, numx);
    for (i = 0; i < numx; i++) {
      b_location[i] = location[i];
    }
    range[0] = ::coder::internal::minimum(b_location);
    if (static_cast<unsigned int>(location.size(0)) < 1U) {
      numx = 0;
    } else {
      numx = location.size(0);
    }
    b_location.set_size(1, numx);
    for (i = 0; i < numx; i++) {
      b_location[i] = location[i];
    }
    range[1] = ::coder::internal::maximum(b_location);
    b_index = static_cast<unsigned int>(location.size(0)) << 1;
    if (location.size(0) + 1U > b_index) {
      i = 0;
      n = 0;
    } else {
      i = location.size(0);
      n = static_cast<int>(b_index);
    }
    numx = n - i;
    b_location.set_size(1, numx);
    for (n = 0; n < numx; n++) {
      b_location[n] = location[i + n];
    }
    range[2] = ::coder::internal::minimum(b_location);
    if (location.size(0) + 1U > b_index) {
      i = 0;
      n = 0;
    } else {
      i = location.size(0);
      n = static_cast<int>(b_index);
    }
    numx = n - i;
    b_location.set_size(1, numx);
    for (n = 0; n < numx; n++) {
      b_location[n] = location[i + n];
    }
    range[3] = ::coder::internal::maximum(b_location);
    b_i = mul_u32_sat(3U, static_cast<unsigned int>(location.size(0)));
    if (b_index + 1U > b_i) {
      i = 0;
      n = 0;
    } else {
      i = static_cast<int>(b_index);
      n = static_cast<int>(b_i);
    }
    numx = n - i;
    b_location.set_size(1, numx);
    for (n = 0; n < numx; n++) {
      b_location[n] = location[i + n];
    }
    range[4] = ::coder::internal::minimum(b_location);
    b_index = (static_cast<unsigned int>(location.size(0)) << 1) + 1U;
    if (b_index > b_i) {
      i = 0;
      n = 0;
    } else {
      i = static_cast<int>(b_index) - 1;
      n = static_cast<int>(b_i);
    }
    numx = n - i;
    b_location.set_size(1, numx);
    for (n = 0; n < numx; n++) {
      b_location[n] = location[i + n];
    }
    range[5] = ::coder::internal::maximum(b_location);
  } else {
    range.set_size(rangeIn.size(0), rangeIn.size(1));
    numx = rangeIn.size(0) * rangeIn.size(1);
    for (i = 0; i < numx; i++) {
      range[i] = rangeIn[i];
    }
  }
  xmin = range[0];
  ymin = range[2];
  zmin = range[4];
  numx = (static_cast<int>(std::floor(range[1] * 100.0)) -
          static_cast<int>(std::floor(range[0] * 100.0))) +
         1;
  numxy = static_cast<double>(numx) *
          static_cast<double>((static_cast<int>(std::floor(range[3] * 100.0)) -
                               static_cast<int>(std::floor(range[2] * 100.0))) +
                              1);
  numOut = 0;
  if (needXYZLimits) {
    ptrIndexVectorCpy.set_size(location.size(0), 2);
    ptrIndexVectorEnd = location.size(0);
    for (n = 0; n < ptrIndexVectorEnd; n++) {
      double y;
      y = (std::floor((location[n] - xmin) * 100.0) +
           std::floor((location[n + static_cast<int>(numPoints)] - ymin) *
                      100.0) *
               static_cast<double>(numx)) +
          std::floor((location[n + (static_cast<int>(numPoints) << 1)] - zmin) *
                     100.0) *
              numxy;
      if (y < 1.8446744073709552E+19) {
        if (y >= 0.0) {
          Double2MultiWord(y, (unsigned int *)&r.chunks[0U]);
        } else {
          r = r1;
        }
      } else if (y >= 1.8446744073709552E+19) {
        r = r2;
      } else {
        r = r1;
      }
      sLong2uMultiWordSat(MultiWord2sLong((const unsigned int *)&r.chunks[0U]),
                          (unsigned int *)&ptrIndexVectorCpy[n].chunks[0U]);
      sLong2uMultiWordSat(
          n + 1,
          (unsigned int *)&ptrIndexVectorCpy[n + ptrIndexVectorCpy.size(0)]
              .chunks[0U]);
    }
  } else {
    ptrIndexVectorCpy.set_size(location.size(0), 2);
    ptrIndexVectorEnd = 0;
    i = location.size(0);
    for (n = 0; n < i; n++) {
      double y;
      double z;
      y = location[static_cast<int>(n + numPoints)];
      b_index = (n + (numPoints << 1)) + 1U;
      if (b_index < n + 1U) {
        b_index = MAX_uint32_T;
      }
      z = location[static_cast<int>(b_index) - 1];
      if ((location[n] < xmin) || (location[n] > range[1]) || (y < ymin) ||
          (y > range[3]) || (z < zmin) || (z > range[5])) {
        numOut++;
      } else {
        y = (std::floor((location[n] - xmin) * 100.0) +
             std::floor((y - ymin) * 100.0) * static_cast<double>(numx)) +
            std::floor((z - zmin) * 100.0) * numxy;
        if (y < 1.8446744073709552E+19) {
          if (y >= 0.0) {
            Double2MultiWord(y, (unsigned int *)&r.chunks[0U]);
          } else {
            r = r1;
          }
        } else if (y >= 1.8446744073709552E+19) {
          r = r2;
        } else {
          r = r1;
        }
        ptrIndexVectorCpy[ptrIndexVectorEnd] = r;
        uLong2MultiWord(
            n + 1U,
            (unsigned int *)&ptrIndexVectorCpy[ptrIndexVectorEnd +
                                               ptrIndexVectorCpy.size(0)]
                .chunks[0U]);
        ptrIndexVectorEnd++;
      }
    }
    if (ptrIndexVectorEnd < 1) {
      numx = 0;
    } else {
      numx = ptrIndexVectorEnd;
    }
    for (i = 0; i < 2; i++) {
      for (n = 0; n < numx; n++) {
        ptrIndexVectorCpy[n + numx * i] =
            ptrIndexVectorCpy[n + ptrIndexVectorCpy.size(0) * i];
      }
    }
    ptrIndexVectorCpy.set_size(numx, 2);
  }
  sortVoxelIndex(ptrIndexVectorCpy, ptrIndexVectorEnd, ptrIndexVectorEnd);
  b_index = 1U;
  while (static_cast<double>(b_index) <= ptrIndexVectorEnd) {
    b_i = b_index + 1U;
    while (
        (b_i <= static_cast<unsigned int>(ptrIndexVectorEnd)) &&
        uMultiWordEq(
            (const unsigned int *)&ptrIndexVectorCpy[static_cast<int>(b_i) - 1]
                .chunks[0U],
            (const unsigned int
                 *)&ptrIndexVectorCpy[static_cast<int>(b_index) - 1]
                .chunks[0U])) {
      b_i++;
    }
    if (static_cast<double>(b_i) - static_cast<double>(b_index) >= 1.0) {
      numOut++;
    }
    b_index = b_i;
  }
  filteredLocation.set_size(numOut, 3);
  if (needColorFlag) {
    filteredColor.set_size(numOut, 3);
  } else {
    filteredColor.set_size(0, 3);
  }
  if (needNormalFlag) {
    filteredNormal.set_size(numOut, 3);
  } else {
    filteredNormal.set_size(0, 3);
  }
  if (needIntensityFlag) {
    filteredIntensity.set_size(numOut);
  } else {
    filteredIntensity.set_size(0);
  }
  if (needRangeFlag) {
    filteredRangeData.set_size(numOut, 3);
  } else {
    filteredRangeData.set_size(0, 3);
  }
  voxelGridAlgImpl(location, static_cast<unsigned int>(location.size(0)), color,
                   normal, intensity, rangeData, range, needXYZLimits,
                   filteredLocation, filteredColor, filteredNormal,
                   filteredIntensity, filteredRangeData, ptrIndexVectorCpy,
                   numOut, needColorFlag, needNormalFlag, needIntensityFlag,
                   needRangeFlag);
}

} // namespace pc
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder

// End of code generation (voxelGridFilter.cpp)
