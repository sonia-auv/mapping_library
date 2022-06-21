//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// std_msgs_Float32Struct.cpp
//
// Code generation for function 'std_msgs_Float32Struct'
//

// Include files
#include "std_msgs_Float32Struct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
std_msgs_Float32Struct_T std_msgs_Float32Struct()
{
  static const std_msgs_Float32Struct_T b_msg{
      {'s', 't', 'd', '_', 'm', 's', 'g', 's', '/', 'F', 'l', 'o', 'a', 't',
       '3', '2'}, // MessageType
      0.0F        // Data
  };
  std_msgs_Float32Struct_T msg;
  msg = b_msg;
  //  Message struct definition for std_msgs/Float32
  //(&b_msg);
  return msg;
}

// End of code generation (std_msgs_Float32Struct.cpp)
