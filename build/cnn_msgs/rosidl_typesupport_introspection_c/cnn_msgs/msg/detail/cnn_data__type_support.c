// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cnn_msgs:msg/CnnData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cnn_msgs/msg/detail/cnn_data__rosidl_typesupport_introspection_c.h"
#include "cnn_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cnn_msgs/msg/detail/cnn_data__functions.h"
#include "cnn_msgs/msg/detail/cnn_data__struct.h"


// Include directives for member types
// Member `ped_pos_map`
// Member `scan`
// Member `scan_all`
// Member `image_gray`
// Member `depth`
// Member `goal_cart`
// Member `goal_final_polar`
// Member `vel`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cnn_msgs__msg__CnnData__init(message_memory);
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_fini_function(void * message_memory)
{
  cnn_msgs__msg__CnnData__fini(message_memory);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__ped_pos_map(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__ped_pos_map(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__ped_pos_map(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__ped_pos_map(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__ped_pos_map(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__ped_pos_map(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__ped_pos_map(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__ped_pos_map(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__scan(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__scan(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__scan(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__scan(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__scan(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__scan(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__scan(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__scan(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__scan_all(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__scan_all(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__scan_all(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__scan_all(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__scan_all(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__scan_all(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__scan_all(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__scan_all(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__image_gray(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__image_gray(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__image_gray(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__image_gray(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__image_gray(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__image_gray(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__image_gray(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__image_gray(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__depth(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__depth(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__depth(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__depth(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__depth(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__depth(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__depth(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__depth(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__goal_cart(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__goal_cart(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__goal_cart(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__goal_cart(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__goal_cart(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__goal_cart(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__goal_cart(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__goal_cart(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__goal_final_polar(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__goal_final_polar(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__goal_final_polar(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__goal_final_polar(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__goal_final_polar(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__goal_final_polar(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__goal_final_polar(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__goal_final_polar(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__vel(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__vel(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__vel(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__vel(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__vel(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__vel(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_member_array[8] = {
  {
    "ped_pos_map",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, ped_pos_map),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__ped_pos_map,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__ped_pos_map,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__ped_pos_map,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__ped_pos_map,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__ped_pos_map,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__ped_pos_map  // resize(index) function pointer
  },
  {
    "scan",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, scan),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__scan,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__scan,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__scan,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__scan,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__scan,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__scan  // resize(index) function pointer
  },
  {
    "scan_all",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, scan_all),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__scan_all,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__scan_all,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__scan_all,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__scan_all,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__scan_all,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__scan_all  // resize(index) function pointer
  },
  {
    "image_gray",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, image_gray),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__image_gray,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__image_gray,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__image_gray,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__image_gray,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__image_gray,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__image_gray  // resize(index) function pointer
  },
  {
    "depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, depth),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__depth,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__depth,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__depth,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__depth,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__depth,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__depth  // resize(index) function pointer
  },
  {
    "goal_cart",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, goal_cart),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__goal_cart,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__goal_cart,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__goal_cart,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__goal_cart,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__goal_cart,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__goal_cart  // resize(index) function pointer
  },
  {
    "goal_final_polar",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, goal_final_polar),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__goal_final_polar,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__goal_final_polar,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__goal_final_polar,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__goal_final_polar,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__goal_final_polar,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__goal_final_polar  // resize(index) function pointer
  },
  {
    "vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs__msg__CnnData, vel),  // bytes offset in struct
    NULL,  // default value
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__size_function__CnnData__vel,  // size() function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_const_function__CnnData__vel,  // get_const(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__get_function__CnnData__vel,  // get(index) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__fetch_function__CnnData__vel,  // fetch(index, &value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__assign_function__CnnData__vel,  // assign(index, value) function pointer
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__resize_function__CnnData__vel  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_members = {
  "cnn_msgs__msg",  // message namespace
  "CnnData",  // message name
  8,  // number of fields
  sizeof(cnn_msgs__msg__CnnData),
  cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_member_array,  // message members
  cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_init_function,  // function to initialize message memory (memory has to be allocated)
  cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_type_support_handle = {
  0,
  &cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cnn_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cnn_msgs, msg, CnnData)() {
  if (!cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_type_support_handle.typesupport_identifier) {
    cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cnn_msgs__msg__CnnData__rosidl_typesupport_introspection_c__CnnData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
