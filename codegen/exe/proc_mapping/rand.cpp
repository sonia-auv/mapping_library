//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rand.cpp
//
// Code generation for function 'rand'
//

// Include files
#include "rand.h"
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
namespace coder {
double b_rand()
{
  double r;
  if (method == 4U) {
    int hi;
    unsigned int mti;
    unsigned int y;
    hi = static_cast<int>(state / 127773U);
    mti = 16807U * (state - hi * 127773U);
    y = 2836U * hi;
    if (mti < y) {
      state = ~(y - mti) & 2147483647U;
    } else {
      state = mti - y;
    }
    r = static_cast<double>(state) * 4.6566128752457969E-10;
  } else if (method == 5U) {
    unsigned int mti;
    unsigned int y;
    mti = 69069U * b_state[0] + 1234567U;
    y = b_state[1] ^ b_state[1] << 13;
    y ^= y >> 17;
    y ^= y << 5;
    b_state[0] = mti;
    b_state[1] = y;
    r = static_cast<double>(mti + y) * 2.328306436538696E-10;
  } else {
    // ========================= COPYRIGHT NOTICE ============================
    //  This is a uniform (0,1) pseudorandom number generator based on:
    //
    //  A C-program for MT19937, with initialization improved 2002/1/26.
    //  Coded by Takuji Nishimura and Makoto Matsumoto.
    //
    //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
    //  All rights reserved.
    //
    //  Redistribution and use in source and binary forms, with or without
    //  modification, are permitted provided that the following conditions
    //  are met:
    //
    //    1. Redistributions of source code must retain the above copyright
    //       notice, this list of conditions and the following disclaimer.
    //
    //    2. Redistributions in binary form must reproduce the above copyright
    //       notice, this list of conditions and the following disclaimer
    //       in the documentation and/or other materials provided with the
    //       distribution.
    //
    //    3. The names of its contributors may not be used to endorse or
    //       promote products derived from this software without specific
    //       prior written permission.
    //
    //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
    //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    //
    // =============================   END   =================================
    unsigned int u[2];
    do {
      for (int hi{0}; hi < 2; hi++) {
        unsigned int mti;
        unsigned int y;
        mti = c_state[624] + 1U;
        if (c_state[624] + 1U >= 625U) {
          for (int kk{0}; kk < 227; kk++) {
            y = (c_state[kk] & 2147483648U) | (c_state[kk + 1] & 2147483647U);
            if ((y & 1U) == 0U) {
              y >>= 1U;
            } else {
              y = y >> 1U ^ 2567483615U;
            }
            c_state[kk] = c_state[kk + 397] ^ y;
          }
          for (int kk{0}; kk < 396; kk++) {
            y = (c_state[kk + 227] & 2147483648U) |
                (c_state[kk + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              y >>= 1U;
            } else {
              y = y >> 1U ^ 2567483615U;
            }
            c_state[kk + 227] = c_state[kk] ^ y;
          }
          y = (c_state[623] & 2147483648U) | (c_state[0] & 2147483647U);
          if ((y & 1U) == 0U) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }
          c_state[623] = c_state[396] ^ y;
          mti = 1U;
        }
        y = c_state[static_cast<int>(mti) - 1];
        c_state[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        u[hi] = y ^ y >> 18U;
      }
      u[0] >>= 5U;
      u[1] >>= 6U;
      r = 1.1102230246251565E-16 * (static_cast<double>(u[0]) * 6.7108864E+7 +
                                    static_cast<double>(u[1]));
    } while (r == 0.0);
  }
  return r;
}

} // namespace coder

// End of code generation (rand.cpp)
