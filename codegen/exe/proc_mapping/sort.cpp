//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sort.cpp
//
// Code generation for function 'sort'
//

// Include files
#include "sort.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
namespace internal {
void sort(::coder::array<double, 1U> &x, ::coder::array<int, 1U> &idx)
{
  array<double, 1U> b_vwork;
  array<double, 1U> vwork;
  array<double, 1U> xwork;
  array<int, 1U> b_iwork;
  array<int, 1U> iidx;
  array<int, 1U> iwork;
  int bLen2;
  int dim;
  int vlen;
  int vstride;
  signed char perm[4];
  dim = 0;
  if (x.size(0) != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    bLen2 = x.size(0);
  } else {
    bLen2 = 1;
  }
  vlen = bLen2 - 1;
  vwork.set_size(bLen2);
  idx.set_size(x.size(0));
  vstride = 1;
  for (int k{0}; k <= dim; k++) {
    vstride *= x.size(0);
  }
  for (int j{0}; j < vstride; j++) {
    for (int k{0}; k <= vlen; k++) {
      vwork[k] = x[j + k * vstride];
    }
    dim = vwork.size(0);
    b_vwork.set_size(vwork.size(0));
    for (bLen2 = 0; bLen2 < dim; bLen2++) {
      b_vwork[bLen2] = vwork[bLen2];
    }
    iidx.set_size(vwork.size(0));
    dim = vwork.size(0);
    for (bLen2 = 0; bLen2 < dim; bLen2++) {
      iidx[bLen2] = 0;
    }
    bLen2 = vwork.size(0);
    if (vwork.size(0) != 0) {
      double x4[4];
      int idx4[4];
      int bLen;
      int i1;
      int i2;
      int i3;
      int i4;
      int iidx_tmp;
      int nNonNaN;
      x4[0] = 0.0;
      idx4[0] = 0;
      x4[1] = 0.0;
      idx4[1] = 0;
      x4[2] = 0.0;
      idx4[2] = 0;
      x4[3] = 0.0;
      idx4[3] = 0;
      iwork.set_size(vwork.size(0));
      dim = vwork.size(0);
      for (i1 = 0; i1 < dim; i1++) {
        iwork[i1] = 0;
      }
      xwork.set_size(vwork.size(0));
      dim = vwork.size(0);
      for (i1 = 0; i1 < dim; i1++) {
        xwork[i1] = 0.0;
      }
      bLen = 0;
      dim = -1;
      for (int k{0}; k < bLen2; k++) {
        if (std::isnan(b_vwork[k])) {
          iidx_tmp = (bLen2 - bLen) - 1;
          iidx[iidx_tmp] = k + 1;
          xwork[iidx_tmp] = b_vwork[k];
          bLen++;
        } else {
          dim++;
          idx4[dim] = k + 1;
          x4[dim] = b_vwork[k];
          if (dim + 1 == 4) {
            double d;
            double d1;
            dim = k - bLen;
            if (x4[0] <= x4[1]) {
              i1 = 1;
              i2 = 2;
            } else {
              i1 = 2;
              i2 = 1;
            }
            if (x4[2] <= x4[3]) {
              i3 = 3;
              i4 = 4;
            } else {
              i3 = 4;
              i4 = 3;
            }
            d = x4[i1 - 1];
            d1 = x4[i3 - 1];
            if (d <= d1) {
              d = x4[i2 - 1];
              if (d <= d1) {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i2);
                perm[2] = static_cast<signed char>(i3);
                perm[3] = static_cast<signed char>(i4);
              } else if (d <= x4[i4 - 1]) {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i3);
                perm[2] = static_cast<signed char>(i2);
                perm[3] = static_cast<signed char>(i4);
              } else {
                perm[0] = static_cast<signed char>(i1);
                perm[1] = static_cast<signed char>(i3);
                perm[2] = static_cast<signed char>(i4);
                perm[3] = static_cast<signed char>(i2);
              }
            } else {
              d1 = x4[i4 - 1];
              if (d <= d1) {
                if (x4[i2 - 1] <= d1) {
                  perm[0] = static_cast<signed char>(i3);
                  perm[1] = static_cast<signed char>(i1);
                  perm[2] = static_cast<signed char>(i2);
                  perm[3] = static_cast<signed char>(i4);
                } else {
                  perm[0] = static_cast<signed char>(i3);
                  perm[1] = static_cast<signed char>(i1);
                  perm[2] = static_cast<signed char>(i4);
                  perm[3] = static_cast<signed char>(i2);
                }
              } else {
                perm[0] = static_cast<signed char>(i3);
                perm[1] = static_cast<signed char>(i4);
                perm[2] = static_cast<signed char>(i1);
                perm[3] = static_cast<signed char>(i2);
              }
            }
            iidx[dim - 3] = idx4[perm[0] - 1];
            iidx[dim - 2] = idx4[perm[1] - 1];
            iidx[dim - 1] = idx4[perm[2] - 1];
            iidx[dim] = idx4[perm[3] - 1];
            b_vwork[dim - 3] = x4[perm[0] - 1];
            b_vwork[dim - 2] = x4[perm[1] - 1];
            b_vwork[dim - 1] = x4[perm[2] - 1];
            b_vwork[dim] = x4[perm[3] - 1];
            dim = -1;
          }
        }
      }
      i3 = (vwork.size(0) - bLen) - 1;
      if (dim + 1 > 0) {
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (dim + 1 == 1) {
          perm[0] = 1;
        } else if (dim + 1 == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }
        for (int k{0}; k <= dim; k++) {
          iidx_tmp = perm[k] - 1;
          i1 = (i3 - dim) + k;
          iidx[i1] = idx4[iidx_tmp];
          b_vwork[i1] = x4[iidx_tmp];
        }
      }
      dim = (bLen >> 1) + 1;
      for (int k{0}; k <= dim - 2; k++) {
        i1 = (i3 + k) + 1;
        i2 = iidx[i1];
        iidx_tmp = (bLen2 - k) - 1;
        iidx[i1] = iidx[iidx_tmp];
        iidx[iidx_tmp] = i2;
        b_vwork[i1] = xwork[iidx_tmp];
        b_vwork[iidx_tmp] = xwork[i1];
      }
      if ((bLen & 1) != 0) {
        dim += i3;
        b_vwork[dim] = xwork[dim];
      }
      nNonNaN = vwork.size(0) - bLen;
      i1 = 2;
      if (nNonNaN > 1) {
        if (vwork.size(0) >= 256) {
          int nBlocks;
          nBlocks = nNonNaN >> 8;
          if (nBlocks > 0) {
            for (int b{0}; b < nBlocks; b++) {
              double b_xwork[256];
              int c_iwork[256];
              i4 = (b << 8) - 1;
              for (int b_b{0}; b_b < 6; b_b++) {
                int nPairs;
                bLen = 1 << (b_b + 2);
                bLen2 = bLen << 1;
                nPairs = 256 >> (b_b + 3);
                for (int k{0}; k < nPairs; k++) {
                  i2 = (i4 + k * bLen2) + 1;
                  for (i1 = 0; i1 < bLen2; i1++) {
                    dim = i2 + i1;
                    c_iwork[i1] = iidx[dim];
                    b_xwork[i1] = b_vwork[dim];
                  }
                  i3 = 0;
                  i1 = bLen;
                  dim = i2 - 1;
                  int exitg1;
                  do {
                    exitg1 = 0;
                    dim++;
                    if (b_xwork[i3] <= b_xwork[i1]) {
                      iidx[dim] = c_iwork[i3];
                      b_vwork[dim] = b_xwork[i3];
                      if (i3 + 1 < bLen) {
                        i3++;
                      } else {
                        exitg1 = 1;
                      }
                    } else {
                      iidx[dim] = c_iwork[i1];
                      b_vwork[dim] = b_xwork[i1];
                      if (i1 + 1 < bLen2) {
                        i1++;
                      } else {
                        dim -= i3;
                        for (i1 = i3 + 1; i1 <= bLen; i1++) {
                          iidx_tmp = dim + i1;
                          iidx[iidx_tmp] = c_iwork[i1 - 1];
                          b_vwork[iidx_tmp] = b_xwork[i1 - 1];
                        }
                        exitg1 = 1;
                      }
                    }
                  } while (exitg1 == 0);
                }
              }
            }
            dim = nBlocks << 8;
            i1 = nNonNaN - dim;
            if (i1 > 0) {
              merge_block(iidx, b_vwork, dim, i1, 2, iwork, xwork);
            }
            i1 = 8;
          }
        }
        dim = iwork.size(0);
        b_iwork.set_size(iwork.size(0));
        for (bLen2 = 0; bLen2 < dim; bLen2++) {
          b_iwork[bLen2] = iwork[bLen2];
        }
        vwork.set_size(xwork.size(0));
        dim = xwork.size(0);
        for (bLen2 = 0; bLen2 < dim; bLen2++) {
          vwork[bLen2] = xwork[bLen2];
        }
        merge_block(iidx, b_vwork, 0, nNonNaN, i1, b_iwork, vwork);
      }
    }
    vwork.set_size(b_vwork.size(0));
    dim = b_vwork.size(0);
    for (bLen2 = 0; bLen2 < dim; bLen2++) {
      vwork[bLen2] = b_vwork[bLen2];
    }
    for (int k{0}; k <= vlen; k++) {
      bLen2 = j + k * vstride;
      x[bLen2] = b_vwork[k];
      idx[bLen2] = iidx[k];
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (sort.cpp)
