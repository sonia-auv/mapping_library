//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// std_msgs_BoolStruct.cpp
//
// Code generation for function 'std_msgs_BoolStruct'
//

// Include files
#include "std_msgs_BoolStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
std_msgs_BoolStruct_T std_msgs_BoolStruct()
{
  static const std_msgs_BoolStruct_T b_msg{
      {'s', 't', 'd', '_', 'm', 's', 'g', 's', '/', 'B', 'o', 'o',
       'l'}, // MessageType
      false  // Data
  };
  std_msgs_BoolStruct_T msg;
  msg = b_msg;
  //  Message struct definition for std_msgs/Bool
  //(&b_msg);
  return msg;
}

// End of code generation (std_msgs_BoolStruct.cpp)
