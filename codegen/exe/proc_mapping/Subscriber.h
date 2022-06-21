//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Subscriber.h
//
// Code generation for function 'Subscriber'
//

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

// Include files
#include "proc_mapping_types.h"
#include "rtwtypes.h"
#include "mlroscpp_sub.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Subscriber {
public:
  Subscriber *init();
  void callback();
  double get_MessageCount() const;
  Subscriber *b_init();
  void b_callback();
  char TopicName[40];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>>
      SubscriberHelper;
  std_msgs_Float32Struct_T MsgStruct;
  bool IsInitialized;
};

class b_Subscriber {
public:
  b_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  b_Subscriber *b_init();
  void b_callback();
  char TopicName[36];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>>
      SubscriberHelper;
  std_msgs_Float32Struct_T MsgStruct;
  bool IsInitialized;
};

class c_Subscriber {
public:
  c_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  char TopicName[24];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>>
      SubscriberHelper;
  std_msgs_BoolStruct_T MsgStruct;
  bool IsInitialized;
};

class d_Subscriber {
public:
  d_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void get_LatestMessage(double *lastSubMsg_Position_X,
                         double *lastSubMsg_Position_Y,
                         double *lastSubMsg_Position_Z,
                         double *lastSubMsg_Orientation_X,
                         double *lastSubMsg_Orientation_Y,
                         double *lastSubMsg_Orientation_Z,
                         double *lastSubMsg_Orientation_W) const;
  char TopicName[18];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<
      MATLABSubscriber<geometry_msgs::Pose, geometry_msgs_PoseStruct_T>>
      SubscriberHelper;
  geometry_msgs_PoseStruct_T MsgStruct;
};

class e_Subscriber {
public:
  e_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void get_LatestMessage(sensor_msgs_PointCloud2Struct_T *lastSubMsg) const;
  char TopicName[28];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<sensor_msgs::PointCloud2,
                                   sensor_msgs_PointCloud2Struct_T>>
      SubscriberHelper;
  sensor_msgs_PointCloud2Struct_T MsgStruct;
  bool IsInitialized;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (Subscriber.h)
