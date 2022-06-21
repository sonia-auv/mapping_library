#include "ros_structmsg_conversion.h"


// Conversions between geometry_msgs_PointStruct_T and geometry_msgs::Point

void struct2msg(geometry_msgs::Point* msgPtr, geometry_msgs_PointStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_PointStruct_T* structPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between geometry_msgs_PoseStruct_T and geometry_msgs::Pose

void struct2msg(geometry_msgs::Pose* msgPtr, geometry_msgs_PoseStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  struct2msg(&msgPtr->orientation, &structPtr->Orientation);
  struct2msg(&msgPtr->position, &structPtr->Position);
}

void msg2struct(geometry_msgs_PoseStruct_T* structPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  msg2struct(&structPtr->Orientation, &msgPtr->orientation);
  msg2struct(&structPtr->Position, &msgPtr->position);
}


// Conversions between geometry_msgs_QuaternionStruct_T and geometry_msgs::Quaternion

void struct2msg(geometry_msgs::Quaternion* msgPtr, geometry_msgs_QuaternionStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  structPtr->W;
  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_QuaternionStruct_T* structPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  structPtr->W =  msgPtr->w;
  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between ros_TimeStruct_T and ros::Time

void struct2msg(ros::Time* msgPtr, ros_TimeStruct_T const* structPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->nsec =  structPtr->Nsec;
  msgPtr->sec =  structPtr->Sec;
}

void msg2struct(ros_TimeStruct_T* structPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  structPtr->Nsec =  msgPtr->nsec;
  structPtr->Sec =  msgPtr->sec;
}


// Conversions between sensor_msgs_PointCloud2Struct_T and sensor_msgs::PointCloud2

void struct2msg(sensor_msgs::PointCloud2* msgPtr, sensor_msgs_PointCloud2Struct_T const* structPtr)
{
  const std::string rosMessageType("sensor_msgs/PointCloud2");

  convertFromStructPrimitiveArray(msgPtr->data, structPtr->Data);
  convertFromStructNestedArray(msgPtr->fields, structPtr->Fields);
  struct2msg(&msgPtr->header, &structPtr->Header);
  msgPtr->height =  structPtr->Height;
  msgPtr->is_bigendian =  structPtr->IsBigendian;
  msgPtr->is_dense =  structPtr->IsDense;
  msgPtr->point_step =  structPtr->PointStep;
  msgPtr->row_step =  structPtr->RowStep;
  msgPtr->width =  structPtr->Width;
}

void msg2struct(sensor_msgs_PointCloud2Struct_T* structPtr, sensor_msgs::PointCloud2 const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/PointCloud2");

  convertToStructPrimitiveArray(structPtr->Data, msgPtr->data);
  convertToStructNestedArray(structPtr->Fields, msgPtr->fields);
  msg2struct(&structPtr->Header, &msgPtr->header);
  structPtr->Height =  msgPtr->height;
  structPtr->IsBigendian =  msgPtr->is_bigendian;
  structPtr->IsDense =  msgPtr->is_dense;
  structPtr->PointStep =  msgPtr->point_step;
  structPtr->RowStep =  msgPtr->row_step;
  structPtr->Width =  msgPtr->width;
}


// Conversions between sensor_msgs_PointFieldStruct_T and sensor_msgs::PointField

void struct2msg(sensor_msgs::PointField* msgPtr, sensor_msgs_PointFieldStruct_T const* structPtr)
{
  const std::string rosMessageType("sensor_msgs/PointField");

  msgPtr->count =  structPtr->Count;
  msgPtr->datatype =  structPtr->Datatype;
  convertFromStructPrimitiveArray(msgPtr->name, structPtr->Name);
  msgPtr->offset =  structPtr->Offset;
}

void msg2struct(sensor_msgs_PointFieldStruct_T* structPtr, sensor_msgs::PointField const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/PointField");

  structPtr->Count =  msgPtr->count;
  structPtr->Datatype =  msgPtr->datatype;
  convertToStructPrimitiveArray(structPtr->Name, msgPtr->name);
  structPtr->Offset =  msgPtr->offset;
}


// Conversions between std_msgs_BoolStruct_T and std_msgs::Bool

void struct2msg(std_msgs::Bool* msgPtr, std_msgs_BoolStruct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  msgPtr->data =  structPtr->Data;
}

void msg2struct(std_msgs_BoolStruct_T* structPtr, std_msgs::Bool const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  structPtr->Data =  msgPtr->data;
}


// Conversions between std_msgs_Float32Struct_T and std_msgs::Float32

void struct2msg(std_msgs::Float32* msgPtr, std_msgs_Float32Struct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/Float32");

  msgPtr->data =  structPtr->Data;
}

void msg2struct(std_msgs_Float32Struct_T* structPtr, std_msgs::Float32 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float32");

  structPtr->Data =  msgPtr->data;
}


// Conversions between std_msgs_HeaderStruct_T and std_msgs::Header

void struct2msg(std_msgs::Header* msgPtr, std_msgs_HeaderStruct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromStructPrimitiveArray(msgPtr->frame_id, structPtr->FrameId);
  msgPtr->seq =  structPtr->Seq;
  struct2msg(&msgPtr->stamp, &structPtr->Stamp);
}

void msg2struct(std_msgs_HeaderStruct_T* structPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToStructPrimitiveArray(structPtr->FrameId, msgPtr->frame_id);
  structPtr->Seq =  msgPtr->seq;
  msg2struct(&structPtr->Stamp, &msgPtr->stamp);
}

