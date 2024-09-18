// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from track_ped_msgs:msg/TrackedPerson.idl
// generated code does not contain a copyright notice

#ifndef TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__STRUCT_HPP_
#define TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__track_ped_msgs__msg__TrackedPerson __attribute__((deprecated))
#else
# define DEPRECATED__track_ped_msgs__msg__TrackedPerson __declspec(deprecated)
#endif

namespace track_ped_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackedPerson_
{
  using Type = TrackedPerson_<ContainerAllocator>;

  explicit TrackedPerson_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bbox_upper_left_x = 0l;
      this->bbox_upper_left_y = 0l;
      this->bbox_bottom_right_x = 0l;
      this->bbox_bottom_right_y = 0l;
      this->id = 0l;
      this->depth = 0.0f;
      this->angle = 0.0f;
    }
  }

  explicit TrackedPerson_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_alloc, _init),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bbox_upper_left_x = 0l;
      this->bbox_upper_left_y = 0l;
      this->bbox_bottom_right_x = 0l;
      this->bbox_bottom_right_y = 0l;
      this->id = 0l;
      this->depth = 0.0f;
      this->angle = 0.0f;
    }
  }

  // field types and members
  using _bbox_upper_left_x_type =
    int32_t;
  _bbox_upper_left_x_type bbox_upper_left_x;
  using _bbox_upper_left_y_type =
    int32_t;
  _bbox_upper_left_y_type bbox_upper_left_y;
  using _bbox_bottom_right_x_type =
    int32_t;
  _bbox_bottom_right_x_type bbox_bottom_right_x;
  using _bbox_bottom_right_y_type =
    int32_t;
  _bbox_bottom_right_y_type bbox_bottom_right_y;
  using _id_type =
    int32_t;
  _id_type id;
  using _depth_type =
    float;
  _depth_type depth;
  using _angle_type =
    float;
  _angle_type angle;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__bbox_upper_left_x(
    const int32_t & _arg)
  {
    this->bbox_upper_left_x = _arg;
    return *this;
  }
  Type & set__bbox_upper_left_y(
    const int32_t & _arg)
  {
    this->bbox_upper_left_y = _arg;
    return *this;
  }
  Type & set__bbox_bottom_right_x(
    const int32_t & _arg)
  {
    this->bbox_bottom_right_x = _arg;
    return *this;
  }
  Type & set__bbox_bottom_right_y(
    const int32_t & _arg)
  {
    this->bbox_bottom_right_y = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__depth(
    const float & _arg)
  {
    this->depth = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    track_ped_msgs::msg::TrackedPerson_<ContainerAllocator> *;
  using ConstRawPtr =
    const track_ped_msgs::msg::TrackedPerson_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      track_ped_msgs::msg::TrackedPerson_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      track_ped_msgs::msg::TrackedPerson_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__track_ped_msgs__msg__TrackedPerson
    std::shared_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__track_ped_msgs__msg__TrackedPerson
    std::shared_ptr<track_ped_msgs::msg::TrackedPerson_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackedPerson_ & other) const
  {
    if (this->bbox_upper_left_x != other.bbox_upper_left_x) {
      return false;
    }
    if (this->bbox_upper_left_y != other.bbox_upper_left_y) {
      return false;
    }
    if (this->bbox_bottom_right_x != other.bbox_bottom_right_x) {
      return false;
    }
    if (this->bbox_bottom_right_y != other.bbox_bottom_right_y) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->depth != other.depth) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackedPerson_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackedPerson_

// alias to use template instance with default allocator
using TrackedPerson =
  track_ped_msgs::msg::TrackedPerson_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace track_ped_msgs

#endif  // TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__STRUCT_HPP_
