// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from track_ped_msgs:msg/TrackedPersons.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "track_ped_msgs/msg/detail/tracked_persons__rosidl_typesupport_introspection_c.h"
#include "track_ped_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "track_ped_msgs/msg/detail/tracked_persons__functions.h"
#include "track_ped_msgs/msg/detail/tracked_persons__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `tracks`
#include "track_ped_msgs/msg/tracked_person.h"
// Member `tracks`
#include "track_ped_msgs/msg/detail/tracked_person__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  track_ped_msgs__msg__TrackedPersons__init(message_memory);
}

void track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_fini_function(void * message_memory)
{
  track_ped_msgs__msg__TrackedPersons__fini(message_memory);
}

size_t track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__size_function__TrackedPersons__tracks(
  const void * untyped_member)
{
  const track_ped_msgs__msg__TrackedPerson__Sequence * member =
    (const track_ped_msgs__msg__TrackedPerson__Sequence *)(untyped_member);
  return member->size;
}

const void * track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__get_const_function__TrackedPersons__tracks(
  const void * untyped_member, size_t index)
{
  const track_ped_msgs__msg__TrackedPerson__Sequence * member =
    (const track_ped_msgs__msg__TrackedPerson__Sequence *)(untyped_member);
  return &member->data[index];
}

void * track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__get_function__TrackedPersons__tracks(
  void * untyped_member, size_t index)
{
  track_ped_msgs__msg__TrackedPerson__Sequence * member =
    (track_ped_msgs__msg__TrackedPerson__Sequence *)(untyped_member);
  return &member->data[index];
}

void track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__fetch_function__TrackedPersons__tracks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const track_ped_msgs__msg__TrackedPerson * item =
    ((const track_ped_msgs__msg__TrackedPerson *)
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__get_const_function__TrackedPersons__tracks(untyped_member, index));
  track_ped_msgs__msg__TrackedPerson * value =
    (track_ped_msgs__msg__TrackedPerson *)(untyped_value);
  *value = *item;
}

void track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__assign_function__TrackedPersons__tracks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  track_ped_msgs__msg__TrackedPerson * item =
    ((track_ped_msgs__msg__TrackedPerson *)
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__get_function__TrackedPersons__tracks(untyped_member, index));
  const track_ped_msgs__msg__TrackedPerson * value =
    (const track_ped_msgs__msg__TrackedPerson *)(untyped_value);
  *item = *value;
}

bool track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__resize_function__TrackedPersons__tracks(
  void * untyped_member, size_t size)
{
  track_ped_msgs__msg__TrackedPerson__Sequence * member =
    (track_ped_msgs__msg__TrackedPerson__Sequence *)(untyped_member);
  track_ped_msgs__msg__TrackedPerson__Sequence__fini(member);
  return track_ped_msgs__msg__TrackedPerson__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPersons, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tracks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_ped_msgs__msg__TrackedPersons, tracks),  // bytes offset in struct
    NULL,  // default value
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__size_function__TrackedPersons__tracks,  // size() function pointer
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__get_const_function__TrackedPersons__tracks,  // get_const(index) function pointer
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__get_function__TrackedPersons__tracks,  // get(index) function pointer
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__fetch_function__TrackedPersons__tracks,  // fetch(index, &value) function pointer
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__assign_function__TrackedPersons__tracks,  // assign(index, value) function pointer
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__resize_function__TrackedPersons__tracks  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_members = {
  "track_ped_msgs__msg",  // message namespace
  "TrackedPersons",  // message name
  2,  // number of fields
  sizeof(track_ped_msgs__msg__TrackedPersons),
  track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_member_array,  // message members
  track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_init_function,  // function to initialize message memory (memory has to be allocated)
  track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_type_support_handle = {
  0,
  &track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_track_ped_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, track_ped_msgs, msg, TrackedPersons)() {
  track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, track_ped_msgs, msg, TrackedPerson)();
  if (!track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_type_support_handle.typesupport_identifier) {
    track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &track_ped_msgs__msg__TrackedPersons__rosidl_typesupport_introspection_c__TrackedPersons_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
