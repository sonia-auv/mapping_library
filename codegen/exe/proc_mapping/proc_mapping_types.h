//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping_types.h
//
// Code generation for function 'proc_mapping'
//

#ifndef PROC_MAPPING_TYPES_H
#define PROC_MAPPING_TYPES_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct ros_TimeStruct_T {
  unsigned int Sec;
  unsigned int Nsec;
};

struct std_msgs_Float32Struct_T {
  char MessageType[16];
  float Data;
};

struct std_msgs_BoolStruct_T {
  char MessageType[13];
  bool Data;
};

struct geometry_msgs_PointStruct_T {
  char MessageType[19];
  double X;
  double Y;
  double Z;
};

struct geometry_msgs_QuaternionStruct_T {
  char MessageType[24];
  double X;
  double Y;
  double Z;
  double W;
};

struct geometry_msgs_PoseStruct_T {
  char MessageType[18];
  geometry_msgs_PointStruct_T Position;
  geometry_msgs_QuaternionStruct_T Orientation;
};

struct uint64m_T {
  unsigned int chunks[2];
};

struct sensor_msgs_PointFieldStruct_T {
  char MessageType[22];
  unsigned char INT8;
  unsigned char UINT8;
  unsigned char INT16;
  unsigned char UINT16;
  unsigned char INT32;
  unsigned char UINT32;
  unsigned char FLOAT32;
  unsigned char FLOAT64;
  coder::array<char, 2U> Name;
  unsigned int Offset;
  unsigned char Datatype;
  unsigned int Count;
};

struct std_msgs_HeaderStruct_T {
  char MessageType[15];
  unsigned int Seq;
  ros_TimeStruct_T Stamp;
  coder::array<char, 2U> FrameId;
};

struct sensor_msgs_PointCloud2Struct_T {
  char MessageType[23];
  std_msgs_HeaderStruct_T Header;
  unsigned int Height;
  unsigned int Width;
  coder::array<sensor_msgs_PointFieldStruct_T, 1U> Fields;
  bool IsBigendian;
  unsigned int PointStep;
  unsigned int RowStep;
  coder::array<unsigned char, 1U> Data;
  bool IsDense;
};

#endif
// End of code generation (proc_mapping_types.h)
