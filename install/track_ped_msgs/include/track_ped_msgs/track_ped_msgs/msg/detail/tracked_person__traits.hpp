// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from track_ped_msgs:msg/TrackedPerson.idl
// generated code does not contain a copyright notice

#ifndef TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__TRAITS_HPP_
#define TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "track_ped_msgs/msg/detail/tracked_person__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace track_ped_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrackedPerson & msg,
  std::ostream & out)
{
  out << "{";
  // member: bbox_upper_left_x
  {
    out << "bbox_upper_left_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_upper_left_x, out);
    out << ", ";
  }

  // member: bbox_upper_left_y
  {
    out << "bbox_upper_left_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_upper_left_y, out);
    out << ", ";
  }

  // member: bbox_bottom_right_x
  {
    out << "bbox_bottom_right_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_bottom_right_x, out);
    out << ", ";
  }

  // member: bbox_bottom_right_y
  {
    out << "bbox_bottom_right_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_bottom_right_y, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: depth
  {
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrackedPerson & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bbox_upper_left_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_upper_left_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_upper_left_x, out);
    out << "\n";
  }

  // member: bbox_upper_left_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_upper_left_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_upper_left_y, out);
    out << "\n";
  }

  // member: bbox_bottom_right_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_bottom_right_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_bottom_right_x, out);
    out << "\n";
  }

  // member: bbox_bottom_right_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_bottom_right_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_bottom_right_y, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth: ";
    rosidl_generator_traits::value_to_yaml(msg.depth, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackedPerson & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace track_ped_msgs

namespace rosidl_generator_traits
{

[[deprecated("use track_ped_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const track_ped_msgs::msg::TrackedPerson & msg,
  std::ostream & out, size_t indentation = 0)
{
  track_ped_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use track_ped_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const track_ped_msgs::msg::TrackedPerson & msg)
{
  return track_ped_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<track_ped_msgs::msg::TrackedPerson>()
{
  return "track_ped_msgs::msg::TrackedPerson";
}

template<>
inline const char * name<track_ped_msgs::msg::TrackedPerson>()
{
  return "track_ped_msgs/msg/TrackedPerson";
}

template<>
struct has_fixed_size<track_ped_msgs::msg::TrackedPerson>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<track_ped_msgs::msg::TrackedPerson>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<track_ped_msgs::msg::TrackedPerson>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__TRAITS_HPP_
