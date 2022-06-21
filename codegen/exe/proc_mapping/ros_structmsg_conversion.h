#ifndef _ROS_STRUCTMSG_CONVERSION_H_
#define _ROS_STRUCTMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include "proc_mapping_types.h"
#include "mlroscpp_msgconvert_utils.h"


void struct2msg(geometry_msgs::Point* msgPtr, geometry_msgs_PointStruct_T const* structPtr);
void msg2struct(geometry_msgs_PointStruct_T* structPtr, geometry_msgs::Point const* msgPtr);

void struct2msg(geometry_msgs::Pose* msgPtr, geometry_msgs_PoseStruct_T const* structPtr);
void msg2struct(geometry_msgs_PoseStruct_T* structPtr, geometry_msgs::Pose const* msgPtr);

void struct2msg(geometry_msgs::Quaternion* msgPtr, geometry_msgs_QuaternionStruct_T const* structPtr);
void msg2struct(geometry_msgs_QuaternionStruct_T* structPtr, geometry_msgs::Quaternion const* msgPtr);

void struct2msg(ros::Time* msgPtr, ros_TimeStruct_T const* structPtr);
void msg2struct(ros_TimeStruct_T* structPtr, ros::Time const* msgPtr);

void struct2msg(sensor_msgs::PointCloud2* msgPtr, sensor_msgs_PointCloud2Struct_T const* structPtr);
void msg2struct(sensor_msgs_PointCloud2Struct_T* structPtr, sensor_msgs::PointCloud2 const* msgPtr);

void struct2msg(sensor_msgs::PointField* msgPtr, sensor_msgs_PointFieldStruct_T const* structPtr);
void msg2struct(sensor_msgs_PointFieldStruct_T* structPtr, sensor_msgs::PointField const* msgPtr);

void struct2msg(std_msgs::Bool* msgPtr, std_msgs_BoolStruct_T const* structPtr);
void msg2struct(std_msgs_BoolStruct_T* structPtr, std_msgs::Bool const* msgPtr);

void struct2msg(std_msgs::Float32* msgPtr, std_msgs_Float32Struct_T const* structPtr);
void msg2struct(std_msgs_Float32Struct_T* structPtr, std_msgs::Float32 const* msgPtr);

void struct2msg(std_msgs::Header* msgPtr, std_msgs_HeaderStruct_T const* structPtr);
void msg2struct(std_msgs_HeaderStruct_T* structPtr, std_msgs::Header const* msgPtr);


#endif
