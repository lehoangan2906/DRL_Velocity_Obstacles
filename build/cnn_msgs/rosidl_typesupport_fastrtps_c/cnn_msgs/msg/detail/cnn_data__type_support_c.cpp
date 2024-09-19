// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from cnn_msgs:msg/CnnData.idl
// generated code does not contain a copyright notice
#include "cnn_msgs/msg/detail/cnn_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "cnn_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "cnn_msgs/msg/detail/cnn_data__struct.h"
#include "cnn_msgs/msg/detail/cnn_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // depth, goal_cart, goal_final_polar, image_gray, ped_pos_map, scan, scan_all, vel
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // depth, goal_cart, goal_final_polar, image_gray, ped_pos_map, scan, scan_all, vel

// forward declare type support functions


using _CnnData__ros_msg_type = cnn_msgs__msg__CnnData;

static bool _CnnData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CnnData__ros_msg_type * ros_message = static_cast<const _CnnData__ros_msg_type *>(untyped_ros_message);
  // Field name: ped_pos_map
  {
    size_t size = ros_message->ped_pos_map.size;
    auto array_ptr = ros_message->ped_pos_map.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: scan
  {
    size_t size = ros_message->scan.size;
    auto array_ptr = ros_message->scan.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: scan_all
  {
    size_t size = ros_message->scan_all.size;
    auto array_ptr = ros_message->scan_all.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: image_gray
  {
    size_t size = ros_message->image_gray.size;
    auto array_ptr = ros_message->image_gray.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: depth
  {
    size_t size = ros_message->depth.size;
    auto array_ptr = ros_message->depth.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: goal_cart
  {
    size_t size = ros_message->goal_cart.size;
    auto array_ptr = ros_message->goal_cart.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: goal_final_polar
  {
    size_t size = ros_message->goal_final_polar.size;
    auto array_ptr = ros_message->goal_final_polar.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: vel
  {
    size_t size = ros_message->vel.size;
    auto array_ptr = ros_message->vel.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _CnnData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CnnData__ros_msg_type * ros_message = static_cast<_CnnData__ros_msg_type *>(untyped_ros_message);
  // Field name: ped_pos_map
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->ped_pos_map.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->ped_pos_map);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->ped_pos_map, size)) {
      fprintf(stderr, "failed to create array for field 'ped_pos_map'");
      return false;
    }
    auto array_ptr = ros_message->ped_pos_map.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: scan
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->scan.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->scan);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->scan, size)) {
      fprintf(stderr, "failed to create array for field 'scan'");
      return false;
    }
    auto array_ptr = ros_message->scan.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: scan_all
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->scan_all.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->scan_all);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->scan_all, size)) {
      fprintf(stderr, "failed to create array for field 'scan_all'");
      return false;
    }
    auto array_ptr = ros_message->scan_all.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: image_gray
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->image_gray.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->image_gray);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->image_gray, size)) {
      fprintf(stderr, "failed to create array for field 'image_gray'");
      return false;
    }
    auto array_ptr = ros_message->image_gray.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: depth
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->depth.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->depth);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->depth, size)) {
      fprintf(stderr, "failed to create array for field 'depth'");
      return false;
    }
    auto array_ptr = ros_message->depth.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: goal_cart
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->goal_cart.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->goal_cart);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->goal_cart, size)) {
      fprintf(stderr, "failed to create array for field 'goal_cart'");
      return false;
    }
    auto array_ptr = ros_message->goal_cart.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: goal_final_polar
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->goal_final_polar.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->goal_final_polar);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->goal_final_polar, size)) {
      fprintf(stderr, "failed to create array for field 'goal_final_polar'");
      return false;
    }
    auto array_ptr = ros_message->goal_final_polar.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: vel
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->vel.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->vel);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->vel, size)) {
      fprintf(stderr, "failed to create array for field 'vel'");
      return false;
    }
    auto array_ptr = ros_message->vel.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cnn_msgs
size_t get_serialized_size_cnn_msgs__msg__CnnData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CnnData__ros_msg_type * ros_message = static_cast<const _CnnData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ped_pos_map
  {
    size_t array_size = ros_message->ped_pos_map.size;
    auto array_ptr = ros_message->ped_pos_map.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scan
  {
    size_t array_size = ros_message->scan.size;
    auto array_ptr = ros_message->scan.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scan_all
  {
    size_t array_size = ros_message->scan_all.size;
    auto array_ptr = ros_message->scan_all.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name image_gray
  {
    size_t array_size = ros_message->image_gray.size;
    auto array_ptr = ros_message->image_gray.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name depth
  {
    size_t array_size = ros_message->depth.size;
    auto array_ptr = ros_message->depth.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goal_cart
  {
    size_t array_size = ros_message->goal_cart.size;
    auto array_ptr = ros_message->goal_cart.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goal_final_polar
  {
    size_t array_size = ros_message->goal_final_polar.size;
    auto array_ptr = ros_message->goal_final_polar.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel
  {
    size_t array_size = ros_message->vel.size;
    auto array_ptr = ros_message->vel.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CnnData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_cnn_msgs__msg__CnnData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cnn_msgs
size_t max_serialized_size_cnn_msgs__msg__CnnData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: ped_pos_map
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: scan
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: scan_all
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: image_gray
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: depth
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: goal_cart
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: goal_final_polar
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vel
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = cnn_msgs__msg__CnnData;
    is_plain =
      (
      offsetof(DataType, vel) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CnnData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_cnn_msgs__msg__CnnData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CnnData = {
  "cnn_msgs::msg",
  "CnnData",
  _CnnData__cdr_serialize,
  _CnnData__cdr_deserialize,
  _CnnData__get_serialized_size,
  _CnnData__max_serialized_size
};

static rosidl_message_type_support_t _CnnData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CnnData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cnn_msgs, msg, CnnData)() {
  return &_CnnData__type_support;
}

#if defined(__cplusplus)
}
#endif
