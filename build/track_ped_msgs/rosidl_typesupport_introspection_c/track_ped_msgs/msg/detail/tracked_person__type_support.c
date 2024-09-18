// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from track_ped_msgs:msg/TrackedPerson.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "track_ped_msgs/msg/detail/tracked_person__rosidl_typesupport_introspection_c.h"
#include "track_ped_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "track_ped_msgs/msg/detail/tracked_person__functions.h"
#include "track_ped_msgs/msg/detail/tracked_person__struct.h"


// Include directives for member types
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  track_ped_msgs__msg__TrackedPerson__init(message_memory);
}

void track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_fini_function(void * message_memory)
{
  track_ped_msgs__msg__TrackedPerson__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_member_array[9] = {
  {
    "bbox_upper_left_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, bbox_upper_left_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox_upper_left_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, bbox_upper_left_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox_bottom_right_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, bbox_bottom_right_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox_bottom_right_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, bbox_bottom_right_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, depth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPerson, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_members = {
  "track_ped_msgs__msg",  // message namespace
  "TrackedPerson",  // message name
  9,  // number of fields
  sizeof(track_ped_msgs__msg__TrackedPerson),
  track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_member_array,  // message members
  track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_init_function,  // function to initialize message memory (memory has to be allocated)
  track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_type_support_handle = {
  0,
  &track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_track_ped_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, track_ped_msgs, msg, TrackedPerson)() {
  track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_type_support_handle.typesupport_identifier) {
    track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &track_ped_msgs__msg__TrackedPerson__rosidl_typesupport_introspection_c__TrackedPerson_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
