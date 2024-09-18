// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_ped_msgs:msg/TrackedPerson.idl
// generated code does not contain a copyright notice

#ifndef TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__STRUCT_H_
#define TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/TrackedPerson in the package track_ped_msgs.
/**
  * Bounding box coordinates (in pixel coordinates)
 */
typedef struct track_ped_msgs__msg__TrackedPerson
{
  int32_t bbox_upper_left_x;
  int32_t bbox_upper_left_y;
  int32_t bbox_bottom_right_x;
  int32_t bbox_bottom_right_y;
  /// Unique ID for the pedestrian
  int32_t id;
  /// Distance from the camera to the pedestrian (depth in meters)
  float depth;
  /// Angle relative to the camera's perpendicular bisector (in radians)
  float angle;
  /// Pedestrian velocity
  /// Twist contains linear and angular velocity
  geometry_msgs__msg__Twist twist;
  /// Real life coordinate of the pedestrian relative to the camera
  geometry_msgs__msg__Pose pose;
} track_ped_msgs__msg__TrackedPerson;

// Struct for a sequence of track_ped_msgs__msg__TrackedPerson.
typedef struct track_ped_msgs__msg__TrackedPerson__Sequence
{
  track_ped_msgs__msg__TrackedPerson * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_ped_msgs__msg__TrackedPerson__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__STRUCT_H_
