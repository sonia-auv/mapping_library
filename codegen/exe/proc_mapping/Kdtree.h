//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Kdtree.h
//
// Code generation for function 'Kdtree'
//

#ifndef KDTREE_H
#define KDTREE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

// Type Definitions
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
class Kdtree {
public:
  static void getNodeFromArray(const ::coder::array<unsigned int, 1U> &idxAll,
                               const ::coder::array<double, 1U> &idxDim,
                               double this_node,
                               ::coder::array<unsigned int, 1U> &nodeIdxThis);
  static void searchNode(const ::coder::array<double, 2U> &X,
                         const double queryPt[3],
                         const ::coder::array<unsigned int, 1U> &nodeIdxStart,
                         int numNN, b_struct_T *pq);
};

class b_Kdtree {
public:
  void buildIndex(const ::coder::array<double, 2U> &inputData);
  array<double, 2U> InputData;
  double NxNoNaN;
  array<double, 2U> CutDim;
  array<double, 2U> CutVal;
  array<double, 2U> LowerBounds;
  array<double, 2U> UpperBounds;
  array<double, 2U> LeftChild;
  array<double, 2U> RightChild;
  array<bool, 2U> LeafNode;
  array<unsigned int, 1U> IdxAll;
  array<double, 1U> IdxDim;
  bool IsIndexed;
};

} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder

// Function Declarations
void b_binary_expand_op(double in1[3], const coder::array<double, 2U> &in2,
                        int in3, const coder::array<double, 2U> &in4);

#endif
// End of code generation (Kdtree.h)
