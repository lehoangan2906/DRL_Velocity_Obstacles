// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cnn_msgs:msg/CnnData.idl
// generated code does not contain a copyright notice

#ifndef CNN_MSGS__MSG__DETAIL__CNN_DATA__STRUCT_H_
#define CNN_MSGS__MSG__DETAIL__CNN_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ped_pos_map'
// Member 'scan'
// Member 'scan_all'
// Member 'image_gray'
// Member 'depth'
// Member 'goal_cart'
// Member 'goal_final_polar'
// Member 'vel'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/CnnData in the package cnn_msgs.
/**
  * This file defines a custom message format for the cnn_msgs package.
  * It aggregates several types of sensor data.
 */
typedef struct cnn_msgs__msg__CnnData
{
  /// pedestrian's position costmap in Cartesian coordinate: 2 channel, 20m * 20m
  rosidl_runtime_c__float__Sequence ped_pos_map;
  /// 720 range data from the laser scan
  rosidl_runtime_c__float__Sequence scan;
  /// 1080 range data from the laser scan
  rosidl_runtime_c__float__Sequence scan_all;
  /// image data from the zed camera
  rosidl_runtime_c__float__Sequence image_gray;
  /// depth image data from the zed camera
  rosidl_runtime_c__float__Sequence depth;
  /// current goal in robot frame
  rosidl_runtime_c__float__Sequence goal_cart;
  /// final goal in robot frame
  rosidl_runtime_c__float__Sequence goal_final_polar;
  /// current velocity in robot frame
  rosidl_runtime_c__float__Sequence vel;
} cnn_msgs__msg__CnnData;

// Struct for a sequence of cnn_msgs__msg__CnnData.
typedef struct cnn_msgs__msg__CnnData__Sequence
{
  cnn_msgs__msg__CnnData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cnn_msgs__msg__CnnData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CNN_MSGS__MSG__DETAIL__CNN_DATA__STRUCT_H_
