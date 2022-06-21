//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// std_msgs_HeaderStruct.cpp
//
// Code generation for function 'std_msgs_HeaderStruct'
//

// Include files
#include "std_msgs_HeaderStruct.h"
#include "proc_mapping_types.h"
#include "ros_TimeStruct.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
void std_msgs_HeaderStruct(std_msgs_HeaderStruct_T *msg)
{
  static const char b_cv[15]{'s', 't', 'd', '_', 'm', 's', 'g', 's',
                             '/', 'H', 'e', 'a', 'd', 'e', 'r'};
  //  Message struct definition for std_msgs/Header
  msg->Stamp = ros_TimeStruct();
  for (int i{0}; i < 15; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->Seq = 0U;
  msg->FrameId.set_size(1, 0);
  //(msg);
}

// End of code generation (std_msgs_HeaderStruct.cpp)
