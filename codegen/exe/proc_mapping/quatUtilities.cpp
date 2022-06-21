//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// quatUtilities.cpp
//
// Code generation for function 'quatUtilities'
//

// Include files
#include "quatUtilities.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
void quatUtilities::getOrientedPointOnPlanarFace(const coder::planeModel *model,
                                                 coder::pointCloud *plane,
                                                 double p[3], double q[4])
{
  coder::array<double, 2U> r;
  coder::array<double, 2U> r1;
  coder::array<double, 2U> r2;
  double absxk;
  double c_idx_0;
  double c_idx_1;
  double c_idx_2;
  double n_tmp;
  double scale;
  double t;
  //  vecteur initial
  //  Trouver la transformaion angulaire du plan
  // =================================================================
  //  Fonction qui calcule la rotation entre 2 vector
  //  https://math.stackexchange.com/questions/2356649/how-to-find-the-quaternion-representing-the-rotation-between-two-3-d-vectors
  c_idx_0 = 0.0 * model->Parameters[2] - 0.0 * model->Parameters[1];
  c_idx_1 = 0.0 * model->Parameters[0] - model->Parameters[2];
  c_idx_2 = model->Parameters[1] - 0.0 * model->Parameters[0];
  scale = 3.3121686421112381E-170;
  t = c_idx_0 / 3.3121686421112381E-170;
  n_tmp = t * t;
  absxk = std::abs(c_idx_1);
  if (absxk > 3.3121686421112381E-170) {
    t = 3.3121686421112381E-170 / absxk;
    n_tmp = n_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n_tmp += t * t;
  }
  absxk = std::abs(c_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    n_tmp = n_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    n_tmp += t * t;
  }
  n_tmp = scale * std::sqrt(n_tmp);
  scale =
      std::atan(n_tmp / ((model->Parameters[0] + 0.0 * model->Parameters[1]) +
                         0.0 * model->Parameters[2]));
  t = std::sin(scale / 2.0);
  q[0] = std::cos(scale / 2.0);
  q[1] = c_idx_0 / n_tmp * t;
  q[2] = c_idx_1 / n_tmp * t;
  q[3] = c_idx_2 / n_tmp * t;
  //  Fossen eq 2.67 p.33
  //  Trouver la transformation linéaire du plan
  plane->get_XLimits(r);
  plane->get_XLimits(r1);
  plane->get_XLimits(r2);
  p[0] = (r[1] - r1[0]) / 2.0 + r2[0];
  plane->get_YLimits(r);
  plane->get_YLimits(r1);
  plane->get_YLimits(r2);
  p[1] = (r[1] - r1[0]) / 2.0 + r2[0];
  plane->get_ZLimits(r);
  plane->get_ZLimits(r1);
  plane->get_ZLimits(r2);
  p[2] = (r[1] - r1[0]) / 2.0 + r2[0];
}

// End of code generation (quatUtilities.cpp)
