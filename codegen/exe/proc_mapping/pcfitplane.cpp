//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcfitplane.cpp
//
// Code generation for function 'pcfitplane'
//

// Include files
#include "pcfitplane.h"
#include "Kdtree.h"
#include "blockedSummation.h"
#include "find.h"
#include "mod.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "proc_mapping_rtwutil.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Declarations
namespace coder {
static void evalPlane(const ::coder::array<double, 2U> &model,
                      const ::coder::array<double, 2U> &points,
                      ::coder::array<double, 1U> &dis);

}

// Function Definitions
namespace coder {
static void evalPlane(const ::coder::array<double, 2U> &model,
                      const ::coder::array<double, 2U> &points,
                      ::coder::array<double, 1U> &dis)
{
  array<double, 1U> y;
  int nx;
  nx = points.size(0);
  y.set_size(points.size(0));
  for (int i{0}; i < nx; i++) {
    y[i] = (points[i] * model[0] + points[points.size(0) + i] * model[1]) +
           points[(points.size(0) << 1) + i] * model[2];
  }
  nx = y.size(0);
  for (int i{0}; i < nx; i++) {
    y[i] = y[i] + model[3];
  }
  nx = y.size(0);
  dis.set_size(y.size(0));
  for (int i{0}; i < nx; i++) {
    dis[i] = std::abs(y[i]);
  }
}

void pcfitplane(const pointCloud *varargin_1, planeModel *iobj_0,
                planeModel **model, ::coder::array<double, 1U> &inlierIndices,
                ::coder::array<double, 1U> &outlierIndices)
{
  pointCloud pc;
  pointCloud *b_pc;
  vision::internal::codegen::Kdtree lobj_1;
  array<double, 2U> b_modelParams;
  array<double, 2U> indices;
  array<double, 2U> location;
  array<double, 2U> modelParams;
  array<double, 2U> samplePoints;
  array<double, 1U> hashTbl;
  array<double, 1U> link;
  array<double, 1U> loc;
  array<int, 1U> val;
  array<int, 1U> y;
  array<unsigned char, 2U> color;
  array<bool, 2U> r;
  array<bool, 2U> r1;
  array<bool, 2U> x;
  array<bool, 1U> b_x;
  array<bool, 1U> bestInliers;
  array<bool, 1U> r2;
  int status;
  int vstride;
  int xoffset;
  pc.matlabCodegenIsDeleted = true;
  x.set_size(varargin_1->Location.size(0), 3);
  vstride = varargin_1->Location.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    x[xoffset] = std::isinf(varargin_1->Location[xoffset]);
  }
  r.set_size(varargin_1->Location.size(0), 3);
  vstride = varargin_1->Location.size(0) * 3;
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    r[xoffset] = std::isnan(varargin_1->Location[xoffset]);
  }
  vstride = x.size(0) * 3;
  x.set_size(x.size(0), 3);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    x[xoffset] = ((!x[xoffset]) && (!r[xoffset]));
  }
  if (x.size(0) == 0) {
    y.set_size(0);
  } else {
    vstride = x.size(0);
    y.set_size(x.size(0));
    for (status = 0; status < vstride; status++) {
      y[status] = x[status];
    }
    for (int k{0}; k < 2; k++) {
      xoffset = (k + 1) * vstride;
      for (status = 0; status < vstride; status++) {
        y[status] = y[status] + x[xoffset + status];
      }
    }
  }
  bestInliers.set_size(y.size(0));
  vstride = y.size(0);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    bestInliers[xoffset] = (y[xoffset] == 3);
  }
  varargin_1->subsetImpl(bestInliers, location, color, modelParams, hashTbl,
                         b_modelParams);
  b_pc = pc.init(location, color, modelParams, hashTbl, &lobj_1);
  b_pc->RangeData.set_size(b_modelParams.size(0), b_modelParams.size(1));
  vstride = b_modelParams.size(0) * b_modelParams.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    b_pc->RangeData[xoffset] = b_modelParams[xoffset];
  }
  b_eml_find(bestInliers, y);
  vstride = b_pc->Location.size(0);
  status = (vstride < 3);
  modelParams.set_size(1, 4);
  modelParams[0] = 0.0;
  modelParams[1] = 0.0;
  modelParams[2] = 0.0;
  modelParams[3] = 0.0;
  if (status == 0) {
    double bestDis;
    int idxTrial;
    int numPts;
    int numTrials;
    int skipTrials;
    bool b_y;
    bool exitg1;
    location.set_size(pc.Location.size(0), 3);
    vstride = pc.Location.size(0) * 3;
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      location[xoffset] = pc.Location[xoffset];
    }
    numPts = location.size(0);
    idxTrial = 1;
    numTrials = 1000;
    bestDis = 0.02 * static_cast<double>(location.size(0));
    modelParams.set_size(0, 0);
    skipTrials = 0;
    bestInliers.set_size(location.size(0));
    vstride = location.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      bestInliers[xoffset] = false;
    }
    while ((idxTrial <= numTrials) && (skipTrials < 10000)) {
      double a[3];
      double b[3];
      double i;
      double j;
      double normal_idx_1;
      double normal_idx_2;
      double selectedLoc;
      double t;
      indices.set_size(1, 3);
      indices[1] = 0.0;
      indices[2] = 0.0;
      if (numPts <= 3) {
        indices[0] = 1.0;
        for (int k{0}; k <= numPts - 2; k++) {
          j = b_rand() * ((static_cast<double>(k) + 1.0) + 1.0);
          j = std::floor(j);
          indices[k + 1] = indices[static_cast<int>(j + 1.0) - 1];
          indices[static_cast<int>(j + 1.0) - 1] =
              (static_cast<double>(k) + 1.0) + 1.0;
        }
      } else if (static_cast<double>(numPts) / 4.0 <= 3.0) {
        t = 0.0;
        j = numPts;
        selectedLoc = 3.0 / static_cast<double>(numPts);
        i = b_rand();
        while (i > selectedLoc) {
          t++;
          j--;
          selectedLoc += (1.0 - selectedLoc) * (3.0 / j);
        }
        t++;
        j = b_rand();
        j = std::floor(j);
        indices[0] = 0.0;
        indices[static_cast<int>(j + 1.0) - 1] = t;
        j = static_cast<double>(numPts) - t;
        selectedLoc = 2.0 / j;
        i = b_rand();
        while (i > selectedLoc) {
          t++;
          j--;
          selectedLoc += (1.0 - selectedLoc) * (2.0 / j);
        }
        t++;
        j = b_rand() * 2.0;
        j = std::floor(j);
        indices[1] = indices[static_cast<int>(j + 1.0) - 1];
        indices[static_cast<int>(j + 1.0) - 1] = t;
        j = static_cast<double>(numPts) - t;
        selectedLoc = 1.0 / j;
        i = b_rand();
        while (i > selectedLoc) {
          t++;
          j--;
          selectedLoc += (1.0 - selectedLoc) * (1.0 / j);
        }
        t++;
        j = b_rand() * 3.0;
        j = std::floor(j);
        indices[2] = indices[static_cast<int>(j + 1.0) - 1];
        indices[static_cast<int>(j + 1.0) - 1] = t;
      } else {
        hashTbl.set_size(3);
        link.set_size(3);
        val.set_size(3);
        loc.set_size(3);
        hashTbl[0] = 0.0;
        hashTbl[1] = 0.0;
        link[1] = 0.0;
        val[1] = 0;
        loc[1] = 0.0;
        hashTbl[2] = 0.0;
        link[2] = 0.0;
        val[2] = 0;
        loc[2] = 0.0;
        selectedLoc = b_rand() * ((static_cast<double>(numPts) - 1.0) + 1.0);
        selectedLoc = std::floor(selectedLoc);
        indices[0] = selectedLoc + 1.0;
        loc[0] = selectedLoc;
        link[0] = 0.0;
        hashTbl[static_cast<int>(b_mod(selectedLoc) + 1.0) - 1] = 1.0;
        selectedLoc =
            hashTbl[static_cast<int>(b_mod(static_cast<double>(numPts) - 1.0) +
                                     1.0) -
                    1];
        while ((selectedLoc > 0.0) &&
               (loc[0] != static_cast<double>(numPts) - 1.0)) {
          selectedLoc = 0.0;
        }
        if (selectedLoc > 0.0) {
          val[0] = 0;
        } else {
          val[0] = numPts - 1;
        }
        selectedLoc = b_rand() * ((static_cast<double>(numPts) - 2.0) + 1.0);
        selectedLoc = std::floor(selectedLoc);
        i = b_mod(selectedLoc) + 1.0;
        j = hashTbl[static_cast<int>(i) - 1];
        while ((j > 0.0) && (loc[0] != selectedLoc)) {
          j = 0.0;
        }
        if (j > 0.0) {
          indices[1] = static_cast<double>(val[0]) + 1.0;
        } else {
          indices[1] = selectedLoc + 1.0;
          j = 2.0;
          loc[1] = selectedLoc;
          link[1] = hashTbl[static_cast<int>(i) - 1];
          hashTbl[static_cast<int>(i) - 1] = 2.0;
        }
        selectedLoc =
            hashTbl[static_cast<int>(b_mod(static_cast<double>(numPts) - 2.0) +
                                     1.0) -
                    1];
        while ((selectedLoc > 0.0) && (loc[static_cast<int>(selectedLoc) - 1] !=
                                       static_cast<double>(numPts) - 2.0)) {
          selectedLoc = link[static_cast<int>(selectedLoc) - 1];
        }
        if (selectedLoc > 0.0) {
          val[static_cast<int>(j) - 1] = val[static_cast<int>(selectedLoc) - 1];
        } else {
          val[static_cast<int>(j) - 1] = numPts - 2;
        }
        selectedLoc = b_rand() * ((static_cast<double>(numPts) - 3.0) + 1.0);
        selectedLoc = std::floor(selectedLoc);
        j = hashTbl[static_cast<int>(b_mod(selectedLoc) + 1.0) - 1];
        while ((j > 0.0) && (loc[static_cast<int>(j) - 1] != selectedLoc)) {
          j = link[static_cast<int>(j) - 1];
        }
        if (j > 0.0) {
          indices[2] = static_cast<double>(val[static_cast<int>(j) - 1]) + 1.0;
        } else {
          indices[2] = selectedLoc + 1.0;
        }
      }
      samplePoints.set_size(3, 3);
      for (xoffset = 0; xoffset < 3; xoffset++) {
        for (vstride = 0; vstride < 3; vstride++) {
          samplePoints[vstride + samplePoints.size(0) * xoffset] =
              location[(static_cast<int>(indices[vstride]) +
                        location.size(0) * xoffset) -
                       1];
        }
        selectedLoc = samplePoints[samplePoints.size(0) * xoffset];
        a[xoffset] =
            samplePoints[samplePoints.size(0) * xoffset + 1] - selectedLoc;
        b[xoffset] =
            samplePoints[samplePoints.size(0) * xoffset + 2] - selectedLoc;
      }
      t = a[1] * b[2] - b[1] * a[2];
      normal_idx_1 = b[0] * a[2] - a[0] * b[2];
      normal_idx_2 = a[0] * b[1] - b[0] * a[1];
      j = (t * t + normal_idx_1 * normal_idx_1) + normal_idx_2 * normal_idx_2;
      if (j < 2.2204460492503131E-16) {
        b_modelParams.set_size(0, 0);
      } else {
        selectedLoc = std::sqrt(j);
        i = t / selectedLoc;
        j = -samplePoints[0] * i;
        t = i;
        i = normal_idx_1 / selectedLoc;
        j += -samplePoints[samplePoints.size(0)] * i;
        normal_idx_1 = i;
        i = normal_idx_2 / selectedLoc;
        j += -samplePoints[samplePoints.size(0) * 2] * i;
        b_modelParams.set_size(1, 4);
        b_modelParams[0] = t;
        b_modelParams[1] = normal_idx_1;
        b_modelParams[2] = i;
        b_modelParams[3] = j;
      }
      b_x.set_size(b_modelParams.size(0) * b_modelParams.size(1));
      vstride = b_modelParams.size(0) * b_modelParams.size(1);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_x[xoffset] = std::isinf(b_modelParams[xoffset]);
      }
      r2.set_size(b_modelParams.size(0) * b_modelParams.size(1));
      vstride = b_modelParams.size(0) * b_modelParams.size(1);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        r2[xoffset] = std::isnan(b_modelParams[xoffset]);
      }
      vstride = b_x.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_x[xoffset] = ((!b_x[xoffset]) && (!r2[xoffset]));
      }
      b_y = true;
      vstride = 1;
      exitg1 = false;
      while ((!exitg1) && (vstride <= b_x.size(0))) {
        if (!b_x[vstride - 1]) {
          b_y = false;
          exitg1 = true;
        } else {
          vstride++;
        }
      }
      if ((b_modelParams.size(0) * b_modelParams.size(1) == 4) && b_y) {
        evalPlane(b_modelParams, location, hashTbl);
        xoffset = hashTbl.size(0);
        for (int k{0}; k < xoffset; k++) {
          if (hashTbl[k] > 0.02) {
            hashTbl[k] = 0.02;
          }
        }
        selectedLoc = blockedSummation(hashTbl, hashTbl.size(0));
        if (selectedLoc < bestDis) {
          bestDis = selectedLoc;
          bestInliers.set_size(hashTbl.size(0));
          vstride = hashTbl.size(0);
          for (xoffset = 0; xoffset < vstride; xoffset++) {
            bestInliers[xoffset] = (hashTbl[xoffset] < 0.02);
          }
          modelParams.set_size(b_modelParams.size(0), b_modelParams.size(1));
          vstride = b_modelParams.size(0) * b_modelParams.size(1);
          for (xoffset = 0; xoffset < vstride; xoffset++) {
            modelParams[xoffset] = b_modelParams[xoffset];
          }
          b_x.set_size(hashTbl.size(0));
          vstride = hashTbl.size(0);
          for (xoffset = 0; xoffset < vstride; xoffset++) {
            b_x[xoffset] = (hashTbl[xoffset] < 0.02);
          }
          vstride = b_x.size(0);
          if (b_x.size(0) == 0) {
            xoffset = 0;
          } else {
            xoffset = b_x[0];
            for (int k{2}; k <= vstride; k++) {
              xoffset += b_x[k - 1];
            }
          }
          selectedLoc = rt_powd_snf(
              static_cast<double>(xoffset) / static_cast<double>(numPts), 3.0);
          if (selectedLoc < 2.2204460492503131E-16) {
            vstride = MAX_int32_T;
          } else {
            selectedLoc =
                std::ceil(-1.9999999999999996 / std::log10(1.0 - selectedLoc));
            if (selectedLoc < 2.147483648E+9) {
              vstride = static_cast<int>(selectedLoc);
            } else if (selectedLoc >= 2.147483648E+9) {
              vstride = MAX_int32_T;
            } else {
              vstride = 0;
            }
          }
          if (numTrials > vstride) {
            numTrials = vstride;
          }
        }
        idxTrial++;
      } else {
        skipTrials++;
      }
    }
    b_x.set_size(modelParams.size(0) * modelParams.size(1));
    vstride = modelParams.size(0) * modelParams.size(1);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      b_x[xoffset] = std::isinf(modelParams[xoffset]);
    }
    r2.set_size(modelParams.size(0) * modelParams.size(1));
    vstride = modelParams.size(0) * modelParams.size(1);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      r2[xoffset] = std::isnan(modelParams[xoffset]);
    }
    vstride = b_x.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      b_x[xoffset] = ((!b_x[xoffset]) && (!r2[xoffset]));
    }
    b_y = true;
    vstride = 1;
    exitg1 = false;
    while ((!exitg1) && (vstride <= b_x.size(0))) {
      if (!b_x[vstride - 1]) {
        b_y = false;
        exitg1 = true;
      } else {
        vstride++;
      }
    }
    if ((modelParams.size(0) * modelParams.size(1) == 4) && b_y &&
        (bestInliers.size(0) != 0)) {
      vstride = bestInliers.size(0);
      xoffset = bestInliers[0];
      for (int k{2}; k <= vstride; k++) {
        xoffset += bestInliers[k - 1];
      }
      if (xoffset < 3) {
        status = 2;
      }
    } else {
      status = 2;
    }
  }
  if ((modelParams.size(0) == 0) || (modelParams.size(1) == 0)) {
    modelParams.set_size(1, 4);
    modelParams[0] = 0.0;
    modelParams[1] = 0.0;
    modelParams[2] = 0.0;
    modelParams[3] = 0.0;
  }
  *model = iobj_0;
  iobj_0->Parameters.set_size(1, modelParams.size(1));
  vstride = modelParams.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    iobj_0->Parameters[xoffset] = modelParams[xoffset];
  }
  if (status == 0) {
    evalPlane(modelParams, pc.Location, hashTbl);
    r1.set_size(hashTbl.size(0), 1);
    vstride = hashTbl.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      r1[xoffset] = (hashTbl[xoffset] < 0.02);
    }
    xoffset = r1.size(0) - 1;
    vstride = 0;
    for (int k{0}; k <= xoffset; k++) {
      if (r1[k]) {
        vstride++;
      }
    }
    inlierIndices.set_size(vstride);
    vstride = 0;
    for (int k{0}; k <= xoffset; k++) {
      if (r1[k]) {
        inlierIndices[vstride] = y[k];
        vstride++;
      }
    }
    vstride = varargin_1->Location.size(0);
    bestInliers.set_size(vstride);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      bestInliers[xoffset] = true;
    }
    y.set_size(inlierIndices.size(0));
    vstride = inlierIndices.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      y[xoffset] = static_cast<int>(inlierIndices[xoffset]);
    }
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      bestInliers[y[xoffset] - 1] = false;
    }
    b_eml_find(bestInliers, y);
    outlierIndices.set_size(y.size(0));
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      outlierIndices[xoffset] = y[xoffset];
    }
  } else {
    inlierIndices.set_size(0);
    outlierIndices.set_size(0);
  }
}

} // namespace coder

// End of code generation (pcfitplane.cpp)
