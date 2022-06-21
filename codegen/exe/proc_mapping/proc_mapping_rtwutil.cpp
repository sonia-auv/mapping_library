//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping_rtwutil.cpp
//
// Code generation for function 'proc_mapping_rtwutil'
//

// Include files
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
unsigned int mul_u32_sat(unsigned int a, unsigned int b)
{
  unsigned int result;
  unsigned int u32_chi;
  mul_wide_u32(a, b, &u32_chi, &result);
  if (u32_chi) {
    result = MAX_uint32_T;
  }
  return result;
}

void mul_wide_u32(unsigned int in0, unsigned int in1,
                  unsigned int *ptrOutBitsHi, unsigned int *ptrOutBitsLo)
{
  int in0Hi;
  int in0Lo;
  int in1Hi;
  int in1Lo;
  unsigned int outBitsLo;
  unsigned int productHiLo;
  unsigned int productLoHi;
  unsigned int productLoLo;
  in0Hi = static_cast<int>(in0 >> 16U);
  in0Lo = static_cast<int>(in0 & 65535U);
  in1Hi = static_cast<int>(in1 >> 16U);
  in1Lo = static_cast<int>(in1 & 65535U);
  productHiLo = static_cast<unsigned int>(in0Hi) * in1Lo;
  productLoHi = static_cast<unsigned int>(in0Lo) * in1Hi;
  productLoLo = static_cast<unsigned int>(in0Lo) * in1Lo;
  in0Lo = 0;
  outBitsLo = productLoLo + (productLoHi << 16U);
  if (outBitsLo < productLoLo) {
    in0Lo = 1;
  }
  productLoLo = outBitsLo;
  outBitsLo += productHiLo << 16U;
  if (outBitsLo < productLoLo) {
    in0Lo = static_cast<int>(in0Lo + 1U);
  }
  *ptrOutBitsHi = ((in0Lo + static_cast<unsigned int>(in0Hi) * in1Hi) +
                   (productLoHi >> 16U)) +
                  (productHiLo >> 16U);
  *ptrOutBitsLo = outBitsLo;
}

double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

// End of code generation (proc_mapping_rtwutil.cpp)
