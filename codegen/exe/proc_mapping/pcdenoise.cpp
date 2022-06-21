//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcdenoise.cpp
//
// Code generation for function 'pcdenoise'
//

// Include files
#include "pcdenoise.h"
#include "Kdtree.h"
#include "blockedSummation.h"
#include "find.h"
#include "pointCloud.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Declarations
static int div_nzp_s32_floor(int numerator, int denominator);

// Function Definitions
static int div_nzp_s32_floor(int numerator, int denominator)
{
  unsigned int absDenominator;
  unsigned int absNumerator;
  int quotient;
  unsigned int tempAbsQuotient;
  bool quotientNeedsNegation;
  if (numerator < 0) {
    absNumerator = ~static_cast<unsigned int>(numerator) + 1U;
  } else {
    absNumerator = static_cast<unsigned int>(numerator);
  }
  if (denominator < 0) {
    absDenominator = ~static_cast<unsigned int>(denominator) + 1U;
  } else {
    absDenominator = static_cast<unsigned int>(denominator);
  }
  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
    quotient = -static_cast<int>(tempAbsQuotient);
  } else {
    quotient = static_cast<int>(tempAbsQuotient);
  }
  return quotient;
}

namespace coder {
pointCloud *pcdenoise(b_pointCloud *ptCloudIn,
                      vision::internal::codegen::Kdtree *iobj_0,
                      pointCloud *iobj_1)
{
  pointCloud *ptCloudOut;
  array<double, 2U> b_r;
  array<double, 2U> dists;
  array<double, 2U> finiteMeanDist;
  array<double, 2U> loc;
  array<double, 2U> meanDist;
  array<double, 1U> absdiff;
  array<double, 1U> b_finiteMeanDist;
  array<unsigned int, 2U> a__1;
  array<unsigned int, 2U> inlierIndices;
  array<int, 2U> r2;
  array<unsigned int, 1U> valids;
  array<int, 1U> y;
  array<unsigned char, 2U> c;
  array<bool, 2U> b_meanDist;
  array<bool, 2U> r;
  array<bool, 2U> r1;
  array<bool, 2U> x;
  array<bool, 1U> isValidPoints;
  double b_y;
  double bsum;
  int firstBlockLength;
  int hi;
  int i;
  int i1;
  int lastBlockLength;
  unsigned int maxval;
  int nblocks;
  int npages;
  unsigned int qY;
  int xblockoffset;
  ptCloudIn->multiQueryKNNSearchImpl(ptCloudIn->Location, a__1, dists, valids);
  npages = valids.size(0);
  maxval = valids[0];
  for (int k{2}; k <= npages; k++) {
    qY = valids[k - 1];
    if (maxval < qY) {
      maxval = qY;
    }
  }
  qY = maxval - 1U;
  if (maxval - 1U > maxval) {
    qY = 0U;
  }
  if (qY + 1U < 2U) {
    i = -1;
    i1 = -1;
  } else {
    i = 0;
    i1 = static_cast<int>(qY);
  }
  i1 -= i;
  if ((i1 == 0) || (dists.size(1) == 0) || (i1 == 0)) {
    meanDist.set_size(1, dists.size(1));
    npages = dists.size(1);
    for (i = 0; i < npages; i++) {
      meanDist[i] = 0.0;
    }
  } else {
    npages = dists.size(1) - 1;
    meanDist.set_size(1, dists.size(1));
    if (i1 <= 1024) {
      firstBlockLength = i1;
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = i1 >> 10;
      lastBlockLength = i1 - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    for (int xi{0}; xi <= npages; xi++) {
      int i2;
      int xpageoffset;
      xpageoffset = xi * i1;
      meanDist[xi] =
          dists[((i + xpageoffset % i1) +
                 dists.size(0) * div_nzp_s32_floor(xpageoffset, i1)) +
                1];
      for (int k{2}; k <= firstBlockLength; k++) {
        i2 = (xpageoffset + k) - 1;
        meanDist[xi] =
            meanDist[xi] +
            dists[((i + i2 % i1) + dists.size(0) * div_nzp_s32_floor(i2, i1)) +
                  1];
      }
      for (int ib{2}; ib <= nblocks; ib++) {
        xblockoffset = xpageoffset + ((ib - 1) << 10);
        bsum = dists[((i + xblockoffset % i1) +
                      dists.size(0) * div_nzp_s32_floor(xblockoffset, i1)) +
                     1];
        if (ib == nblocks) {
          hi = lastBlockLength;
        } else {
          hi = 1024;
        }
        for (int k{2}; k <= hi; k++) {
          i2 = (xblockoffset + k) - 1;
          bsum += dists[((i + i2 % i1) +
                         dists.size(0) * div_nzp_s32_floor(i2, i1)) +
                        1];
        }
        meanDist[xi] = meanDist[xi] + bsum;
      }
    }
  }
  meanDist.set_size(1, meanDist.size(1));
  npages = meanDist.size(1) - 1;
  for (i = 0; i <= npages; i++) {
    meanDist[i] = meanDist[i] / static_cast<double>(qY);
  }
  x.set_size(ptCloudIn->Location.size(0), 3);
  npages = ptCloudIn->Location.size(0) * 3;
  for (i = 0; i < npages; i++) {
    x[i] = std::isinf(ptCloudIn->Location[i]);
  }
  r.set_size(ptCloudIn->Location.size(0), 3);
  npages = ptCloudIn->Location.size(0) * 3;
  for (i = 0; i < npages; i++) {
    r[i] = std::isnan(ptCloudIn->Location[i]);
  }
  npages = x.size(0) * 3;
  x.set_size(x.size(0), 3);
  for (i = 0; i < npages; i++) {
    x[i] = ((!x[i]) && (!r[i]));
  }
  if (x.size(0) == 0) {
    y.set_size(0);
  } else {
    npages = x.size(0);
    y.set_size(x.size(0));
    for (nblocks = 0; nblocks < npages; nblocks++) {
      y[nblocks] = x[nblocks];
    }
    for (int k{0}; k < 2; k++) {
      firstBlockLength = (k + 1) * npages;
      for (nblocks = 0; nblocks < npages; nblocks++) {
        y[nblocks] = y[nblocks] + x[firstBlockLength + nblocks];
      }
    }
  }
  isValidPoints.set_size(y.size(0));
  npages = y.size(0);
  for (i = 0; i < npages; i++) {
    isValidPoints[i] = (y[i] == 3);
  }
  firstBlockLength = isValidPoints.size(0);
  for (nblocks = 0; nblocks < firstBlockLength; nblocks++) {
    if (!isValidPoints[nblocks]) {
      meanDist[nblocks] = rtNaN;
    }
  }
  b_meanDist.set_size(1, meanDist.size(1));
  npages = meanDist.size(1);
  for (i = 0; i < npages; i++) {
    b_meanDist[i] = std::isinf(meanDist[i]);
  }
  r1.set_size(1, meanDist.size(1));
  npages = meanDist.size(1);
  for (i = 0; i < npages; i++) {
    r1[i] = std::isnan(meanDist[i]);
  }
  b_meanDist.set_size(1, b_meanDist.size(1));
  npages = b_meanDist.size(1) - 1;
  for (i = 0; i <= npages; i++) {
    b_meanDist[i] = ((!b_meanDist[i]) && (!r1[i]));
  }
  firstBlockLength = b_meanDist.size(1) - 1;
  npages = 0;
  for (nblocks = 0; nblocks <= firstBlockLength; nblocks++) {
    if (b_meanDist[nblocks]) {
      npages++;
    }
  }
  finiteMeanDist.set_size(1, npages);
  npages = 0;
  for (nblocks = 0; nblocks <= firstBlockLength; nblocks++) {
    if (b_meanDist[nblocks]) {
      finiteMeanDist[npages] = meanDist[nblocks];
      npages++;
    }
  }
  if (finiteMeanDist.size(1) == 0) {
    b_y = 0.0;
  } else {
    if (finiteMeanDist.size(1) <= 1024) {
      firstBlockLength = finiteMeanDist.size(1);
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = finiteMeanDist.size(1) >> 10;
      lastBlockLength = finiteMeanDist.size(1) - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    b_y = finiteMeanDist[0];
    for (int k{2}; k <= firstBlockLength; k++) {
      b_y += finiteMeanDist[k - 1];
    }
    for (int ib{2}; ib <= nblocks; ib++) {
      xblockoffset = (ib - 1) << 10;
      bsum = finiteMeanDist[xblockoffset];
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }
      for (int k{2}; k <= hi; k++) {
        bsum += finiteMeanDist[(xblockoffset + k) - 1];
      }
      b_y += bsum;
    }
  }
  firstBlockLength = finiteMeanDist.size(1);
  if (finiteMeanDist.size(1) == 0) {
    bsum = rtNaN;
  } else if (finiteMeanDist.size(1) == 1) {
    if ((!std::isinf(finiteMeanDist[0])) && (!std::isnan(finiteMeanDist[0]))) {
      bsum = 0.0;
    } else {
      bsum = rtNaN;
    }
  } else {
    double scale;
    npages = finiteMeanDist.size(1);
    b_finiteMeanDist = finiteMeanDist.reshape(npages);
    bsum = blockedSummation(b_finiteMeanDist, finiteMeanDist.size(1)) /
           static_cast<double>(finiteMeanDist.size(1));
    absdiff.set_size(finiteMeanDist.size(1));
    for (int k{0}; k < firstBlockLength; k++) {
      absdiff[k] = std::abs(finiteMeanDist[k] - bsum);
    }
    bsum = 0.0;
    scale = 3.3121686421112381E-170;
    npages = finiteMeanDist.size(1);
    for (int k{0}; k < npages; k++) {
      if (absdiff[k] > scale) {
        double t;
        t = scale / absdiff[k];
        bsum = bsum * t * t + 1.0;
        scale = absdiff[k];
      } else {
        double t;
        t = absdiff[k] / scale;
        bsum += t * t;
      }
    }
    bsum = scale * std::sqrt(bsum);
    bsum /= std::sqrt(static_cast<double>(finiteMeanDist.size(1)) - 1.0);
  }
  bsum += b_y / static_cast<double>(finiteMeanDist.size(1));
  b_meanDist.set_size(1, meanDist.size(1));
  npages = meanDist.size(1);
  for (i = 0; i < npages; i++) {
    b_meanDist[i] = (meanDist[i] <= bsum);
  }
  eml_find(b_meanDist, r2);
  inlierIndices.set_size(1, r2.size(1));
  npages = r2.size(1);
  for (i = 0; i < npages; i++) {
    i1 = r2[i];
    if (i1 < 0) {
      i1 = 0;
    }
    inlierIndices[i] = static_cast<unsigned int>(i1);
  }
  if (ptCloudIn->Location.size(0) != 0) {
    loc.set_size(inlierIndices.size(1), 3);
    npages = inlierIndices.size(1);
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < npages; i1++) {
        loc[i1 + loc.size(0) * i] =
            ptCloudIn->Location[(static_cast<int>(inlierIndices[i1]) +
                                 ptCloudIn->Location.size(0) * i) -
                                1];
      }
    }
  } else {
    loc.set_size(0, 3);
  }
  if ((ptCloudIn->Color.size(0) != 0) && (ptCloudIn->Color.size(1) != 0)) {
    npages = ptCloudIn->Color.size(1);
    c.set_size(inlierIndices.size(1), npages);
    for (i = 0; i < npages; i++) {
      firstBlockLength = inlierIndices.size(1);
      for (i1 = 0; i1 < firstBlockLength; i1++) {
        c[i1 + c.size(0) * i] =
            ptCloudIn->Color[(static_cast<int>(inlierIndices[i1]) +
                              ptCloudIn->Color.size(0) * i) -
                             1];
      }
    }
  } else {
    c.set_size(0, 0);
  }
  if ((ptCloudIn->Normal.size(0) != 0) && (ptCloudIn->Normal.size(1) != 0)) {
    npages = ptCloudIn->Normal.size(1);
    dists.set_size(inlierIndices.size(1), npages);
    for (i = 0; i < npages; i++) {
      firstBlockLength = inlierIndices.size(1);
      for (i1 = 0; i1 < firstBlockLength; i1++) {
        dists[i1 + dists.size(0) * i] =
            ptCloudIn->Normal[(static_cast<int>(inlierIndices[i1]) +
                               ptCloudIn->Normal.size(0) * i) -
                              1];
      }
    }
  } else {
    dists.set_size(0, 0);
  }
  if (ptCloudIn->Intensity.size(0) != 0) {
    absdiff.set_size(inlierIndices.size(1));
    npages = inlierIndices.size(1);
    for (i = 0; i < npages; i++) {
      absdiff[i] = ptCloudIn->Intensity[static_cast<int>(inlierIndices[i]) - 1];
    }
  } else {
    absdiff.set_size(0);
  }
  if ((ptCloudIn->RangeData.size(0) != 0) &&
      (ptCloudIn->RangeData.size(1) != 0)) {
    npages = ptCloudIn->RangeData.size(1);
    b_r.set_size(inlierIndices.size(1), npages);
    for (i = 0; i < npages; i++) {
      firstBlockLength = inlierIndices.size(1);
      for (i1 = 0; i1 < firstBlockLength; i1++) {
        b_r[i1 + b_r.size(0) * i] =
            ptCloudIn->RangeData[(static_cast<int>(inlierIndices[i1]) +
                                  ptCloudIn->RangeData.size(0) * i) -
                                 1];
      }
    }
  } else {
    b_r.set_size(0, 0);
  }
  ptCloudOut = iobj_1->init(loc, c, dists, absdiff, iobj_0);
  ptCloudOut->RangeData.set_size(b_r.size(0), b_r.size(1));
  npages = b_r.size(0) * b_r.size(1);
  for (i = 0; i < npages; i++) {
    ptCloudOut->RangeData[i] = b_r[i];
  }
  return ptCloudOut;
}

} // namespace coder

// End of code generation (pcdenoise.cpp)
