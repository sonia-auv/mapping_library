//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Subscriber.cpp
//
// Code generation for function 'Subscriber'
//

// Include files
#include "Subscriber.h"
#include "geometry_msgs_PoseStruct.h"
#include "proc_mapping_data.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_PointCloud2Struct.h"
#include "std_msgs_BoolStruct.h"
#include "std_msgs_Float32Struct.h"
#include "mlroscpp_sub.h"
#include <stdio.h>
#include <string.h>

// Function Definitions
namespace coder {
namespace ros {
void b_Subscriber::b_callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    // minIntensityState maxIntensityState minRangeState maxRangeState;
    //  Initial variables
    //  SET
    maxRangeValue = MsgStruct.Data;
    printf("INFO : proc mapping : max Range filter changed \n");
    fflush(stdout);
  }
}

void Subscriber::b_callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    // minIntensityState maxIntensityState minRangeState maxRangeState;
    //  Initial variables
    //  SET
    maxIntensityValue = MsgStruct.Data;
    printf("INFO : proc mapping : max Intensity filter changed \n");
    fflush(stdout);
  }
}

void c_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    bundleStarted = MsgStruct.Data;
    //  Initial variables
    //  SET
    //  Initial variables
    //  GET
    if (bundleStarted) {
      printf("INFO : proc mapping : Bundle record started \n");
      fflush(stdout);
    } else {
      printf("INFO : proc mapping : Bundle record stopped \n");
      fflush(stdout);
    }
  }
}

void b_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    // minIntensityState maxIntensityState minRangeState maxRangeState;
    //  Initial variables
    //  SET
    minRangeValue = MsgStruct.Data;
    printf("INFO : proc mapping : min Range filter changed \n");
    fflush(stdout);
  }
}

void Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    // minIntensityState maxIntensityState minRangeState maxRangeState;
    //  Initial variables
    //  SET
    minIntensityValue = MsgStruct.Data;
    //              Preprocessing.persistentDataStore('minIntensityState',
    //              true);
    printf("INFO : proc mapping : min Intensity filter changed \n");
    fflush(stdout);
  }
}

void e_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    newSonarMsg = true;
  }
}

void d_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
}

double d_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double e_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double c_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double b_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

Subscriber *Subscriber::b_init()
{
  static const char topic[40]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                              'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                              'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'a',
                              'x', 'I', 'n', 't', 'e', 'n', 's', 'i', 't', 'y'};
  Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 40; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_Float32Struct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>>(
      new MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    40.0, obj->BufferSize);
  obj->b_callback();
  obj->IsInitialized = true;
  return obj;
}

b_Subscriber *b_Subscriber::b_init()
{
  static const char topic[36]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                              'p', 'i', 'n', 'g', '/', 'p', 'r', 'e', 'p',
                              'r', 'o', 'c', 'e', 's', 's', 'i', 'n', 'g',
                              '/', 'm', 'a', 'x', 'R', 'a', 'n', 'g', 'e'};
  b_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 36; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_Float32Struct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>>(
      new MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    36.0, obj->BufferSize);
  obj->b_callback();
  obj->IsInitialized = true;
  return obj;
}

void e_Subscriber::get_LatestMessage(
    sensor_msgs_PointCloud2Struct_T *lastSubMsg) const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  *lastSubMsg = MsgStruct;
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

void d_Subscriber::get_LatestMessage(double *lastSubMsg_Position_X,
                                     double *lastSubMsg_Position_Y,
                                     double *lastSubMsg_Position_Z,
                                     double *lastSubMsg_Orientation_X,
                                     double *lastSubMsg_Orientation_Y,
                                     double *lastSubMsg_Orientation_Z,
                                     double *lastSubMsg_Orientation_W) const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  *lastSubMsg_Position_X = MsgStruct.Position.X;
  *lastSubMsg_Position_Y = MsgStruct.Position.Y;
  *lastSubMsg_Position_Z = MsgStruct.Position.Z;
  *lastSubMsg_Orientation_X = MsgStruct.Orientation.X;
  *lastSubMsg_Orientation_Y = MsgStruct.Orientation.Y;
  *lastSubMsg_Orientation_Z = MsgStruct.Orientation.Z;
  *lastSubMsg_Orientation_W = MsgStruct.Orientation.W;
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

e_Subscriber *e_Subscriber::init()
{
  static const char topic[28]{'/', 'p', 'r', 'o', 'v', 'i', 'd', 'e', 'r', '_',
                              's', 'o', 'n', 'a', 'r', '/', 'p', 'o', 'i', 'n',
                              't', '_', 'c', 'l', 'o', 'u', 'd', '2'};
  e_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 28; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  sensor_msgs_PointCloud2Struct(&obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<sensor_msgs::PointCloud2,
                                       sensor_msgs_PointCloud2Struct_T>>(
          new MATLABSubscriber<sensor_msgs::PointCloud2,
                               sensor_msgs_PointCloud2Struct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    28.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

Subscriber *Subscriber::init()
{
  static const char topic[40]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                              'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                              'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'i',
                              'n', 'I', 'n', 't', 'e', 'n', 's', 'i', 't', 'y'};
  Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 40; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_Float32Struct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>>(
      new MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    40.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

d_Subscriber *d_Subscriber::init()
{
  static const char topic[18]{'/', 'p', 'r', 'o', 'c', '_', 'n', 'a', 'v',
                              '/', 'a', 'u', 'v', '_', 'p', 'o', 's', 'e'};
  d_Subscriber *obj;
  obj = this;
  for (int i{0}; i < 18; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  geometry_msgs_PoseStruct(&obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<geometry_msgs::Pose, geometry_msgs_PoseStruct_T>>(
      new MATLABSubscriber<geometry_msgs::Pose, geometry_msgs_PoseStruct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    18.0, obj->BufferSize);
  obj->callback();
  return obj;
}

b_Subscriber *b_Subscriber::init()
{
  static const char topic[36]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                              'p', 'i', 'n', 'g', '/', 'p', 'r', 'e', 'p',
                              'r', 'o', 'c', 'e', 's', 's', 'i', 'n', 'g',
                              '/', 'm', 'i', 'n', 'R', 'a', 'n', 'g', 'e'};
  b_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 36; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_Float32Struct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>>(
      new MATLABSubscriber<std_msgs::Float32, std_msgs_Float32Struct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    36.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

c_Subscriber *c_Subscriber::init()
{
  static const char topic[24]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a',
                              'p', 'p', 'i', 'n', 'g', '/', 's', 't',
                              'a', 'r', 't', '_', 's', 't', 'o', 'p'};
  c_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 24; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_BoolStruct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>>(
          new MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    24.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

} // namespace ros
} // namespace coder

// End of code generation (Subscriber.cpp)
