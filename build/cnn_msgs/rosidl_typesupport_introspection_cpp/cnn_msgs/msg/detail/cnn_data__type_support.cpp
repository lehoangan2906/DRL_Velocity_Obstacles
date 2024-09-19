// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from cnn_msgs:msg/CnnData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "cnn_msgs/msg/detail/cnn_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cnn_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CnnData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cnn_msgs::msg::CnnData(_init);
}

void CnnData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cnn_msgs::msg::CnnData *>(message_memory);
  typed_message->~CnnData();
}

size_t size_function__CnnData__ped_pos_map(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__ped_pos_map(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__ped_pos_map(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__ped_pos_map(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__ped_pos_map(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__ped_pos_map(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__ped_pos_map(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__ped_pos_map(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CnnData__scan(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__scan(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__scan(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__scan(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__scan(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__scan(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__scan(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__scan(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CnnData__scan_all(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__scan_all(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__scan_all(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__scan_all(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__scan_all(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__scan_all(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__scan_all(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__scan_all(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CnnData__image_gray(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__image_gray(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__image_gray(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__image_gray(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__image_gray(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__image_gray(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__image_gray(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__image_gray(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CnnData__depth(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__depth(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__depth(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__depth(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__depth(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__depth(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__depth(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__depth(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CnnData__goal_cart(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__goal_cart(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__goal_cart(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__goal_cart(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__goal_cart(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__goal_cart(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__goal_cart(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__goal_cart(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CnnData__goal_final_polar(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__goal_final_polar(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__goal_final_polar(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__goal_final_polar(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__goal_final_polar(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__goal_final_polar(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__goal_final_polar(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__goal_final_polar(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CnnData__vel(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CnnData__vel(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CnnData__vel(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__CnnData__vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__CnnData__vel(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__CnnData__vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__CnnData__vel(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__CnnData__vel(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CnnData_message_member_array[8] = {
  {
    "ped_pos_map",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, ped_pos_map),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__ped_pos_map,  // size() function pointer
    get_const_function__CnnData__ped_pos_map,  // get_const(index) function pointer
    get_function__CnnData__ped_pos_map,  // get(index) function pointer
    fetch_function__CnnData__ped_pos_map,  // fetch(index, &value) function pointer
    assign_function__CnnData__ped_pos_map,  // assign(index, value) function pointer
    resize_function__CnnData__ped_pos_map  // resize(index) function pointer
  },
  {
    "scan",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, scan),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__scan,  // size() function pointer
    get_const_function__CnnData__scan,  // get_const(index) function pointer
    get_function__CnnData__scan,  // get(index) function pointer
    fetch_function__CnnData__scan,  // fetch(index, &value) function pointer
    assign_function__CnnData__scan,  // assign(index, value) function pointer
    resize_function__CnnData__scan  // resize(index) function pointer
  },
  {
    "scan_all",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, scan_all),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__scan_all,  // size() function pointer
    get_const_function__CnnData__scan_all,  // get_const(index) function pointer
    get_function__CnnData__scan_all,  // get(index) function pointer
    fetch_function__CnnData__scan_all,  // fetch(index, &value) function pointer
    assign_function__CnnData__scan_all,  // assign(index, value) function pointer
    resize_function__CnnData__scan_all  // resize(index) function pointer
  },
  {
    "image_gray",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, image_gray),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__image_gray,  // size() function pointer
    get_const_function__CnnData__image_gray,  // get_const(index) function pointer
    get_function__CnnData__image_gray,  // get(index) function pointer
    fetch_function__CnnData__image_gray,  // fetch(index, &value) function pointer
    assign_function__CnnData__image_gray,  // assign(index, value) function pointer
    resize_function__CnnData__image_gray  // resize(index) function pointer
  },
  {
    "depth",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, depth),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__depth,  // size() function pointer
    get_const_function__CnnData__depth,  // get_const(index) function pointer
    get_function__CnnData__depth,  // get(index) function pointer
    fetch_function__CnnData__depth,  // fetch(index, &value) function pointer
    assign_function__CnnData__depth,  // assign(index, value) function pointer
    resize_function__CnnData__depth  // resize(index) function pointer
  },
  {
    "goal_cart",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, goal_cart),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__goal_cart,  // size() function pointer
    get_const_function__CnnData__goal_cart,  // get_const(index) function pointer
    get_function__CnnData__goal_cart,  // get(index) function pointer
    fetch_function__CnnData__goal_cart,  // fetch(index, &value) function pointer
    assign_function__CnnData__goal_cart,  // assign(index, value) function pointer
    resize_function__CnnData__goal_cart  // resize(index) function pointer
  },
  {
    "goal_final_polar",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, goal_final_polar),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__goal_final_polar,  // size() function pointer
    get_const_function__CnnData__goal_final_polar,  // get_const(index) function pointer
    get_function__CnnData__goal_final_polar,  // get(index) function pointer
    fetch_function__CnnData__goal_final_polar,  // fetch(index, &value) function pointer
    assign_function__CnnData__goal_final_polar,  // assign(index, value) function pointer
    resize_function__CnnData__goal_final_polar  // resize(index) function pointer
  },
  {
    "vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cnn_msgs::msg::CnnData, vel),  // bytes offset in struct
    nullptr,  // default value
    size_function__CnnData__vel,  // size() function pointer
    get_const_function__CnnData__vel,  // get_const(index) function pointer
    get_function__CnnData__vel,  // get(index) function pointer
    fetch_function__CnnData__vel,  // fetch(index, &value) function pointer
    assign_function__CnnData__vel,  // assign(index, value) function pointer
    resize_function__CnnData__vel  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CnnData_message_members = {
  "cnn_msgs::msg",  // message namespace
  "CnnData",  // message name
  8,  // number of fields
  sizeof(cnn_msgs::msg::CnnData),
  CnnData_message_member_array,  // message members
  CnnData_init_function,  // function to initialize message memory (memory has to be allocated)
  CnnData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CnnData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CnnData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace cnn_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cnn_msgs::msg::CnnData>()
{
  return &::cnn_msgs::msg::rosidl_typesupport_introspection_cpp::CnnData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cnn_msgs, msg, CnnData)() {
  return &::cnn_msgs::msg::rosidl_typesupport_introspection_cpp::CnnData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
