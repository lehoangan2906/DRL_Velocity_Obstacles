// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_ped_msgs:msg/TrackedPersons.idl
// generated code does not contain a copyright notice

#ifndef TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSONS__STRUCT_H_
#define TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'tracks'
#include "track_ped_msgs/msg/detail/tracked_person__struct.h"

/// Struct defined in msg/TrackedPersons in the package track_ped_msgs.
/**
  * TrackedPersons.msg
 */
typedef struct track_ped_msgs__msg__TrackedPersons
{
  /// Header information (includes timestamp and frame id)
  std_msgs__msg__Header header;
  /// A list of tracked pedestrians
  track_ped_msgs__msg__TrackedPerson__Sequence tracks;
} track_ped_msgs__msg__TrackedPersons;

// Struct for a sequence of track_ped_msgs__msg__TrackedPersons.
typedef struct track_ped_msgs__msg__TrackedPersons__Sequence
{
  track_ped_msgs__msg__TrackedPersons * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_ped_msgs__msg__TrackedPersons__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSONS__STRUCT_H_
