//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Kdtree.cpp
//
// Code generation for function 'Kdtree'
//

// Include files
#include "Kdtree.h"
#include "any1.h"
#include "blockedSummation.h"
#include "find.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "coder_array.h"
#include <cmath>
#include <math.h>
#include <string.h>

// Type Definitions
struct cell_wrap_38 {
  coder::array<double, 2U> f1;
};

// Function Definitions
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
void b_Kdtree::buildIndex(const ::coder::array<double, 2U> &inputData)
{
  array<cell_wrap_38, 1U> idxTemp1;
  array<double, 2U> lowerBoundsTemp;
  array<double, 2U> upperBoundsTemp;
  array<double, 1U> cutValTemp;
  array<double, 1U> leftChildTemp;
  array<double, 1U> rightChildTemp;
  array<double, 1U> sx;
  array<int, 2U> notnan;
  array<int, 1U> iidx;
  array<unsigned int, 1U> tempDim;
  array<signed char, 1U> cutDimTemp;
  array<bool, 2U> b_wasnan;
  array<bool, 2U> r;
  array<bool, 2U> wasnan;
  array<bool, 1U> leafNodeTemp;
  double b_tmp;
  double cc;
  int currentNode;
  int i;
  int j;
  int loop_ub;
  int m;
  int nextUnusedNode;
  int nx;
  int unusedNodes;
  bool exitg1;
  bool y;
  r.set_size(inputData.size(0), 3);
  loop_ub = inputData.size(0) * 3;
  for (i = 0; i < loop_ub; i++) {
    r[i] = std::isnan(inputData[i]);
  }
  b_any(r, leafNodeTemp);
  wasnan.set_size(1, leafNodeTemp.size(0));
  loop_ub = leafNodeTemp.size(0);
  for (i = 0; i < loop_ub; i++) {
    wasnan[i] = leafNodeTemp[i];
  }
  if (inputData.size(0) < 1) {
    notnan.set_size(1, 0);
  } else {
    loop_ub = inputData.size(0);
    notnan.set_size(1, inputData.size(0));
    for (i = 0; i < loop_ub; i++) {
      notnan[i] = i + 1;
    }
  }
  y = false;
  unusedNodes = 1;
  exitg1 = false;
  while ((!exitg1) && (unusedNodes <= wasnan.size(1))) {
    if (wasnan[unusedNodes - 1]) {
      y = true;
      exitg1 = true;
    } else {
      unusedNodes++;
    }
  }
  if (y) {
    b_wasnan.set_size(1, wasnan.size(1));
    loop_ub = wasnan.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_wasnan[i] = !wasnan[i];
    }
    eml_find(b_wasnan, notnan);
    nx = notnan.size(1);
  } else {
    nx = inputData.size(0);
  }
  cc = frexp(std::fmax(static_cast<double>(nx) / 50.0, 1.0), &m);
  if (cc == 0.5) {
    cc = static_cast<double>(m) - 1.0;
  } else if ((m == 1) && (cc < 0.75)) {
    cc = std::log(2.0 * cc) / 0.69314718055994529;
  } else {
    cc = std::log(cc) / 0.69314718055994529 + static_cast<double>(m);
  }
  cc = std::ceil(cc);
  unusedNodes = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0);
  cutDimTemp.set_size(unusedNodes);
  cutValTemp.set_size(unusedNodes);
  for (i = 0; i < unusedNodes; i++) {
    cutDimTemp[i] = 0;
    cutValTemp[i] = 0.0;
  }
  lowerBoundsTemp.set_size(unusedNodes, 3);
  m = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0) * 3;
  upperBoundsTemp.set_size(unusedNodes, 3);
  for (i = 0; i < m; i++) {
    lowerBoundsTemp[i] = rtMinusInf;
    upperBoundsTemp[i] = rtInf;
  }
  leftChildTemp.set_size(unusedNodes);
  rightChildTemp.set_size(unusedNodes);
  leafNodeTemp.set_size(unusedNodes);
  idxTemp1.set_size(unusedNodes);
  for (i = 0; i < unusedNodes; i++) {
    leftChildTemp[i] = 0.0;
    rightChildTemp[i] = 0.0;
    leafNodeTemp[i] = false;
    idxTemp1[i].f1.set_size(1, 0);
  }
  idxTemp1[0].f1.set_size(1, notnan.size(1));
  loop_ub = notnan.size(1);
  for (i = 0; i < loop_ub; i++) {
    idxTemp1[0].f1[i] = notnan[i];
  }
  currentNode = 0;
  nextUnusedNode = 1;
  while (currentNode + 1 < nextUnusedNode + 1) {
    if (idxTemp1[currentNode].f1.size(1) <= 50) {
      leafNodeTemp[currentNode] = true;
    } else {
      double ex[3];
      double temp[3];
      iidx.set_size(idxTemp1[currentNode].f1.size(1));
      loop_ub = idxTemp1[currentNode].f1.size(1);
      for (i = 0; i < loop_ub; i++) {
        iidx[i] = static_cast<int>(idxTemp1[currentNode].f1[i]);
      }
      m = iidx.size(0);
      unusedNodes = iidx.size(0);
      for (j = 0; j < 3; j++) {
        cc = inputData[(iidx[0] + inputData.size(0) * j) - 1];
        temp[j] = cc;
        for (loop_ub = 2; loop_ub <= m; loop_ub++) {
          b_tmp = inputData[(iidx[loop_ub - 1] + inputData.size(0) * j) - 1];
          if (std::isnan(b_tmp)) {
            y = false;
          } else if (std::isnan(temp[j])) {
            y = true;
          } else {
            y = (temp[j] < b_tmp);
          }
          if (y) {
            temp[j] = b_tmp;
          }
        }
        ex[j] = cc;
        for (loop_ub = 2; loop_ub <= unusedNodes; loop_ub++) {
          b_tmp = inputData[(iidx[loop_ub - 1] + inputData.size(0) * j) - 1];
          if (std::isnan(b_tmp)) {
            y = false;
          } else if (std::isnan(ex[j])) {
            y = true;
          } else {
            y = (ex[j] > b_tmp);
          }
          if (y) {
            ex[j] = b_tmp;
          }
        }
        temp[j] -= ex[j];
      }
      if (!std::isnan(temp[0])) {
        m = 1;
      } else {
        m = 0;
        unusedNodes = 2;
        exitg1 = false;
        while ((!exitg1) && (unusedNodes < 4)) {
          if (!std::isnan(temp[unusedNodes - 1])) {
            m = unusedNodes;
            exitg1 = true;
          } else {
            unusedNodes++;
          }
        }
      }
      if (m == 0) {
        m = 1;
      } else {
        cc = temp[m - 1];
        i = m + 1;
        for (unusedNodes = i; unusedNodes < 4; unusedNodes++) {
          b_tmp = temp[unusedNodes - 1];
          if (cc < b_tmp) {
            cc = b_tmp;
            m = unusedNodes;
          }
        }
      }
      sx.set_size(iidx.size(0));
      loop_ub = iidx.size(0);
      for (i = 0; i < loop_ub; i++) {
        sx[i] = inputData[(iidx[i] + inputData.size(0) * (m - 1)) - 1];
      }
      ::coder::internal::sort(sx, iidx);
      notnan.set_size(1, iidx.size(0));
      loop_ub = iidx.size(0);
      for (i = 0; i < loop_ub; i++) {
        notnan[i] = static_cast<int>(idxTemp1[currentNode].f1[iidx[i] - 1]);
      }
      unusedNodes =
          static_cast<int>(std::ceil(static_cast<double>(sx.size(0)) / 2.0));
      cc = (sx[unusedNodes - 1] + sx[unusedNodes]) / 2.0;
      cutDimTemp[currentNode] = static_cast<signed char>(m);
      cutValTemp[currentNode] = cc;
      leftChildTemp[currentNode] = nextUnusedNode + 1;
      rightChildTemp[currentNode] =
          static_cast<double>(nextUnusedNode + 1) + 1.0;
      temp[0] = upperBoundsTemp[currentNode];
      temp[1] = upperBoundsTemp[currentNode + upperBoundsTemp.size(0)];
      temp[2] = upperBoundsTemp[currentNode + upperBoundsTemp.size(0) * 2];
      upperBoundsTemp[nextUnusedNode + 1] = upperBoundsTemp[currentNode];
      upperBoundsTemp[(nextUnusedNode + upperBoundsTemp.size(0)) + 1] =
          upperBoundsTemp[currentNode + upperBoundsTemp.size(0)];
      upperBoundsTemp[(nextUnusedNode + upperBoundsTemp.size(0) * 2) + 1] =
          upperBoundsTemp[currentNode + upperBoundsTemp.size(0) * 2];
      temp[m - 1] = cc;
      upperBoundsTemp[nextUnusedNode] = temp[0];
      temp[0] = lowerBoundsTemp[currentNode];
      upperBoundsTemp[nextUnusedNode + upperBoundsTemp.size(0)] = temp[1];
      temp[1] = lowerBoundsTemp[currentNode + lowerBoundsTemp.size(0)];
      upperBoundsTemp[nextUnusedNode + upperBoundsTemp.size(0) * 2] = temp[2];
      temp[2] = lowerBoundsTemp[currentNode + lowerBoundsTemp.size(0) * 2];
      lowerBoundsTemp[nextUnusedNode] = lowerBoundsTemp[currentNode];
      lowerBoundsTemp[nextUnusedNode + lowerBoundsTemp.size(0)] =
          lowerBoundsTemp[currentNode + lowerBoundsTemp.size(0)];
      lowerBoundsTemp[nextUnusedNode + lowerBoundsTemp.size(0) * 2] =
          lowerBoundsTemp[currentNode + lowerBoundsTemp.size(0) * 2];
      temp[m - 1] = cc;
      lowerBoundsTemp[nextUnusedNode + 1] = temp[0];
      lowerBoundsTemp[(nextUnusedNode + lowerBoundsTemp.size(0)) + 1] = temp[1];
      lowerBoundsTemp[(nextUnusedNode + lowerBoundsTemp.size(0) * 2) + 1] =
          temp[2];
      idxTemp1[currentNode].f1.set_size(1, 0);
      idxTemp1[nextUnusedNode].f1.set_size(1, unusedNodes);
      for (i = 0; i < unusedNodes; i++) {
        idxTemp1[nextUnusedNode].f1[i] = notnan[i];
      }
      if (unusedNodes + 1 > iidx.size(0)) {
        unusedNodes = 0;
        i = 0;
      } else {
        i = notnan.size(1);
      }
      loop_ub = i - unusedNodes;
      idxTemp1[nextUnusedNode + 1].f1.set_size(1, loop_ub);
      for (i = 0; i < loop_ub; i++) {
        idxTemp1[nextUnusedNode + 1].f1[i] = notnan[unusedNodes + i];
      }
      nextUnusedNode += 2;
    }
    currentNode++;
  }
  unusedNodes = nextUnusedNode - 1;
  tempDim.set_size(nextUnusedNode);
  IdxAll.set_size(inputData.size(0));
  loop_ub = inputData.size(0);
  for (i = 0; i < loop_ub; i++) {
    IdxAll[i] = 0U;
  }
  cc = 1.0;
  for (m = 0; m <= unusedNodes; m++) {
    tempDim[m] = static_cast<unsigned int>(idxTemp1[m].f1.size(1));
    if (idxTemp1[m].f1.size(1) > 0) {
      b_tmp = (cc + static_cast<double>(idxTemp1[m].f1.size(1))) - 1.0;
      if (cc > b_tmp) {
        i = -1;
        j = 0;
      } else {
        i = static_cast<int>(cc) - 2;
        j = static_cast<int>(b_tmp);
      }
      loop_ub = (j - i) - 1;
      for (j = 0; j < loop_ub; j++) {
        unsigned int u;
        b_tmp = idxTemp1[m].f1[j];
        if (b_tmp >= 0.0) {
          u = static_cast<unsigned int>(b_tmp);
        } else {
          u = 0U;
        }
        IdxAll[(i + j) + 1] = u;
      }
      cc += static_cast<double>(tempDim[m]);
    }
  }
  InputData.set_size(inputData.size(0), 3);
  loop_ub = inputData.size(0) * 3;
  for (i = 0; i < loop_ub; i++) {
    InputData[i] = inputData[i];
  }
  CutDim.set_size(nextUnusedNode, 1);
  CutVal.set_size(nextUnusedNode, 1);
  for (i = 0; i < nextUnusedNode; i++) {
    CutDim[i] = cutDimTemp[i];
    CutVal[i] = cutValTemp[i];
  }
  LowerBounds.set_size(nextUnusedNode, 3);
  UpperBounds.set_size(nextUnusedNode, 3);
  for (i = 0; i < 3; i++) {
    for (j = 0; j < nextUnusedNode; j++) {
      LowerBounds[j + LowerBounds.size(0) * i] =
          lowerBoundsTemp[j + lowerBoundsTemp.size(0) * i];
      UpperBounds[j + UpperBounds.size(0) * i] =
          upperBoundsTemp[j + upperBoundsTemp.size(0) * i];
    }
  }
  IdxDim.set_size(tempDim.size(0));
  loop_ub = tempDim.size(0);
  for (i = 0; i < loop_ub; i++) {
    IdxDim[i] = tempDim[i];
  }
  LeftChild.set_size(nextUnusedNode, 1);
  RightChild.set_size(nextUnusedNode, 1);
  LeafNode.set_size(nextUnusedNode, 1);
  for (i = 0; i < nextUnusedNode; i++) {
    LeftChild[i] = leftChildTemp[i];
    RightChild[i] = rightChildTemp[i];
    LeafNode[i] = leafNodeTemp[i];
  }
  NxNoNaN = nx;
}

void Kdtree::getNodeFromArray(const ::coder::array<unsigned int, 1U> &idxAll,
                              const ::coder::array<double, 1U> &idxDim,
                              double this_node,
                              ::coder::array<unsigned int, 1U> &nodeIdxThis)
{
  array<double, 1U> b_idxDim;
  array<unsigned int, 2U> r;
  double d;
  int loop_ub_tmp;
  loop_ub_tmp = static_cast<int>(this_node) - 1;
  d = idxDim[static_cast<int>(this_node) - 1];
  if (d == 0.0) {
    nodeIdxThis.set_size(0);
  } else if (this_node == 1.0) {
    if (idxDim[0] < 1.0) {
      loop_ub_tmp = 0;
    } else {
      loop_ub_tmp = static_cast<int>(idxDim[0]);
    }
    nodeIdxThis.set_size(loop_ub_tmp);
    for (int i{0}; i < loop_ub_tmp; i++) {
      nodeIdxThis[i] = idxAll[i];
    }
  } else {
    double nIdx;
    b_idxDim.set_size(loop_ub_tmp);
    for (int i{0}; i < loop_ub_tmp; i++) {
      b_idxDim[i] = idxDim[i];
    }
    nIdx = blockedSummation(b_idxDim, static_cast<int>(this_node - 1.0));
    r.set_size(1, static_cast<int>(d - 1.0) + 1);
    loop_ub_tmp =
        static_cast<int>(idxDim[static_cast<int>(this_node) - 1] - 1.0);
    for (int i{0}; i <= loop_ub_tmp; i++) {
      r[i] = static_cast<unsigned int>(nIdx + (static_cast<double>(i) + 1.0));
    }
    nodeIdxThis.set_size(r.size(1));
    loop_ub_tmp = r.size(1);
    for (int i{0}; i < loop_ub_tmp; i++) {
      nodeIdxThis[i] = idxAll[static_cast<int>(r[i]) - 1];
    }
  }
}

void Kdtree::searchNode(const ::coder::array<double, 2U> &X,
                        const double queryPt[3],
                        const ::coder::array<unsigned int, 1U> &nodeIdxStart,
                        int numNN, b_struct_T *pq)
{
  array<double, 2U> diffAllDim;
  array<double, 1U> aDistOut;
  array<double, 1U> r;
  array<int, 1U> iidx;
  array<unsigned int, 1U> r1;
  int acoef;
  int b_acoef;
  int i;
  int varargin_2;
  diffAllDim.set_size(nodeIdxStart.size(0), 3);
  if (nodeIdxStart.size(0) != 0) {
    acoef = (X.size(1) != 1);
    b_acoef = (nodeIdxStart.size(0) != 1);
    for (int k{0}; k < 3; k++) {
      varargin_2 = acoef * k;
      i = diffAllDim.size(0) - 1;
      for (int b_k{0}; b_k <= i; b_k++) {
        diffAllDim[b_k + diffAllDim.size(0) * k] =
            X[(static_cast<int>(nodeIdxStart[b_acoef * b_k]) +
               X.size(0) * varargin_2) -
              1] -
            queryPt[k];
      }
    }
  }
  acoef = diffAllDim.size(0) * 3;
  diffAllDim.set_size(diffAllDim.size(0), 3);
  for (i = 0; i < acoef; i++) {
    diffAllDim[i] = diffAllDim[i] * diffAllDim[i];
  }
  if (diffAllDim.size(0) == 0) {
    aDistOut.set_size(0);
  } else {
    acoef = diffAllDim.size(0);
    aDistOut.set_size(diffAllDim.size(0));
    for (varargin_2 = 0; varargin_2 < acoef; varargin_2++) {
      aDistOut[varargin_2] = diffAllDim[varargin_2];
    }
    for (int k{0}; k < 2; k++) {
      b_acoef = (k + 1) * acoef;
      for (varargin_2 = 0; varargin_2 < acoef; varargin_2++) {
        aDistOut[varargin_2] =
            aDistOut[varargin_2] + diffAllDim[b_acoef + varargin_2];
      }
    }
  }
  ::coder::internal::sort(aDistOut, iidx);
  if (pq->D.size(0) == 0) {
    if (aDistOut.size(0) <= numNN) {
      pq->D.set_size(aDistOut.size(0));
      acoef = aDistOut.size(0);
      for (i = 0; i < acoef; i++) {
        pq->D[i] = aDistOut[i];
      }
      pq->b_I.set_size(iidx.size(0));
      acoef = iidx.size(0);
      for (i = 0; i < acoef; i++) {
        pq->b_I[i] = nodeIdxStart[iidx[i] - 1];
      }
    } else {
      if (numNN < 1) {
        acoef = 0;
      } else {
        acoef = numNN;
      }
      pq->D.set_size(acoef);
      for (i = 0; i < acoef; i++) {
        pq->D[i] = aDistOut[i];
      }
      if (numNN < 1) {
        acoef = 0;
      } else {
        acoef = numNN;
      }
      pq->b_I.set_size(acoef);
      for (i = 0; i < acoef; i++) {
        pq->b_I[i] = nodeIdxStart[iidx[i] - 1];
      }
    }
  } else {
    unsigned int cD1;
    unsigned int cD2;
    unsigned int x;
    bool exitg1;
    cD1 = 1U;
    cD2 = 1U;
    x = static_cast<unsigned int>(pq->D.size(0)) + aDistOut.size(0);
    if (static_cast<double>(x) > numNN) {
      acoef = numNN;
    } else {
      acoef = static_cast<int>(x);
    }
    r.set_size(acoef);
    r1.set_size(acoef);
    for (i = 0; i < acoef; i++) {
      r[i] = 0.0;
      r1[i] = 0U;
    }
    b_acoef = 0;
    exitg1 = false;
    while ((!exitg1) && (b_acoef <= acoef - 1)) {
      double d;
      double d1;
      d = aDistOut[static_cast<int>(cD2) - 1];
      d1 = pq->D[static_cast<int>(cD1) - 1];
      if (d1 <= d) {
        r[b_acoef] = d1;
        r1[b_acoef] = pq->b_I[static_cast<int>(cD1) - 1];
        cD1++;
        if (cD1 > static_cast<unsigned int>(pq->D.size(0))) {
          i = b_acoef + 2;
          for (int b_k{i}; b_k <= acoef; b_k++) {
            varargin_2 = static_cast<int>((cD2 + b_k) - b_acoef) - 3;
            r[b_k - 1] = aDistOut[varargin_2];
            r1[b_k - 1] = nodeIdxStart[iidx[varargin_2] - 1];
          }
          exitg1 = true;
        } else {
          b_acoef++;
        }
      } else {
        r[b_acoef] = d;
        r1[b_acoef] = nodeIdxStart[iidx[static_cast<int>(cD2) - 1] - 1];
        cD2++;
        if (cD2 > static_cast<unsigned int>(aDistOut.size(0))) {
          i = b_acoef + 2;
          for (int b_k{i}; b_k <= acoef; b_k++) {
            varargin_2 = static_cast<int>((cD1 + b_k) - b_acoef) - 3;
            r[b_k - 1] = pq->D[varargin_2];
            r1[b_k - 1] = pq->b_I[varargin_2];
          }
          exitg1 = true;
        } else {
          b_acoef++;
        }
      }
    }
    pq->D.set_size(r.size(0));
    acoef = r.size(0);
    for (i = 0; i < acoef; i++) {
      pq->D[i] = r[i];
    }
    pq->b_I.set_size(r1.size(0));
    acoef = r1.size(0);
    for (i = 0; i < acoef; i++) {
      pq->b_I[i] = r1[i];
    }
  }
}

} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder
void b_binary_expand_op(double in1[3], const coder::array<double, 2U> &in2,
                        int in3, const coder::array<double, 2U> &in4)
{
  int stride_0_1;
  stride_0_1 = (in4.size(1) != 1);
  in1[0] = in2[in3] - in4[0];
  in1[1] = in2[in3 + in2.size(0)] - in4[stride_0_1];
  in1[2] = in2[in3 + in2.size(0) * 2] - in4[stride_0_1 << 1];
}

// End of code generation (Kdtree.cpp)
