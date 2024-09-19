// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cnn_msgs:msg/CnnData.idl
// generated code does not contain a copyright notice

#ifndef CNN_MSGS__MSG__DETAIL__CNN_DATA__STRUCT_HPP_
#define CNN_MSGS__MSG__DETAIL__CNN_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cnn_msgs__msg__CnnData __attribute__((deprecated))
#else
# define DEPRECATED__cnn_msgs__msg__CnnData __declspec(deprecated)
#endif

namespace cnn_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CnnData_
{
  using Type = CnnData_<ContainerAllocator>;

  explicit CnnData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit CnnData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _ped_pos_map_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _ped_pos_map_type ped_pos_map;
  using _scan_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _scan_type scan;
  using _scan_all_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _scan_all_type scan_all;
  using _image_gray_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _image_gray_type image_gray;
  using _depth_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _depth_type depth;
  using _goal_cart_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _goal_cart_type goal_cart;
  using _goal_final_polar_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _goal_final_polar_type goal_final_polar;
  using _vel_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _vel_type vel;

  // setters for named parameter idiom
  Type & set__ped_pos_map(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->ped_pos_map = _arg;
    return *this;
  }
  Type & set__scan(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->scan = _arg;
    return *this;
  }
  Type & set__scan_all(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->scan_all = _arg;
    return *this;
  }
  Type & set__image_gray(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->image_gray = _arg;
    return *this;
  }
  Type & set__depth(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->depth = _arg;
    return *this;
  }
  Type & set__goal_cart(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->goal_cart = _arg;
    return *this;
  }
  Type & set__goal_final_polar(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->goal_final_polar = _arg;
    return *this;
  }
  Type & set__vel(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cnn_msgs::msg::CnnData_<ContainerAllocator> *;
  using ConstRawPtr =
    const cnn_msgs::msg::CnnData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cnn_msgs::msg::CnnData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cnn_msgs::msg::CnnData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cnn_msgs__msg__CnnData
    std::shared_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cnn_msgs__msg__CnnData
    std::shared_ptr<cnn_msgs::msg::CnnData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CnnData_ & other) const
  {
    if (this->ped_pos_map != other.ped_pos_map) {
      return false;
    }
    if (this->scan != other.scan) {
      return false;
    }
    if (this->scan_all != other.scan_all) {
      return false;
    }
    if (this->image_gray != other.image_gray) {
      return false;
    }
    if (this->depth != other.depth) {
      return false;
    }
    if (this->goal_cart != other.goal_cart) {
      return false;
    }
    if (this->goal_final_polar != other.goal_final_polar) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const CnnData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CnnData_

// alias to use template instance with default allocator
using CnnData =
  cnn_msgs::msg::CnnData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cnn_msgs

#endif  // CNN_MSGS__MSG__DETAIL__CNN_DATA__STRUCT_HPP_
