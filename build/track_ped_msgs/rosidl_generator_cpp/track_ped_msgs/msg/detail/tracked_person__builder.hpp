// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from track_ped_msgs:msg/TrackedPerson.idl
// generated code does not contain a copyright notice

#ifndef TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__BUILDER_HPP_
#define TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "track_ped_msgs/msg/detail/tracked_person__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace track_ped_msgs
{

namespace msg
{

namespace builder
{

class Init_TrackedPerson_pose
{
public:
  explicit Init_TrackedPerson_pose(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  ::track_ped_msgs::msg::TrackedPerson pose(::track_ped_msgs::msg::TrackedPerson::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_twist
{
public:
  explicit Init_TrackedPerson_twist(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  Init_TrackedPerson_pose twist(::track_ped_msgs::msg::TrackedPerson::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_TrackedPerson_pose(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_angle
{
public:
  explicit Init_TrackedPerson_angle(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  Init_TrackedPerson_twist angle(::track_ped_msgs::msg::TrackedPerson::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_TrackedPerson_twist(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_depth
{
public:
  explicit Init_TrackedPerson_depth(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  Init_TrackedPerson_angle depth(::track_ped_msgs::msg::TrackedPerson::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return Init_TrackedPerson_angle(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_id
{
public:
  explicit Init_TrackedPerson_id(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  Init_TrackedPerson_depth id(::track_ped_msgs::msg::TrackedPerson::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_TrackedPerson_depth(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_bbox_bottom_right_y
{
public:
  explicit Init_TrackedPerson_bbox_bottom_right_y(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  Init_TrackedPerson_id bbox_bottom_right_y(::track_ped_msgs::msg::TrackedPerson::_bbox_bottom_right_y_type arg)
  {
    msg_.bbox_bottom_right_y = std::move(arg);
    return Init_TrackedPerson_id(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_bbox_bottom_right_x
{
public:
  explicit Init_TrackedPerson_bbox_bottom_right_x(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  Init_TrackedPerson_bbox_bottom_right_y bbox_bottom_right_x(::track_ped_msgs::msg::TrackedPerson::_bbox_bottom_right_x_type arg)
  {
    msg_.bbox_bottom_right_x = std::move(arg);
    return Init_TrackedPerson_bbox_bottom_right_y(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_bbox_upper_left_y
{
public:
  explicit Init_TrackedPerson_bbox_upper_left_y(::track_ped_msgs::msg::TrackedPerson & msg)
  : msg_(msg)
  {}
  Init_TrackedPerson_bbox_bottom_right_x bbox_upper_left_y(::track_ped_msgs::msg::TrackedPerson::_bbox_upper_left_y_type arg)
  {
    msg_.bbox_upper_left_y = std::move(arg);
    return Init_TrackedPerson_bbox_bottom_right_x(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

class Init_TrackedPerson_bbox_upper_left_x
{
public:
  Init_TrackedPerson_bbox_upper_left_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackedPerson_bbox_upper_left_y bbox_upper_left_x(::track_ped_msgs::msg::TrackedPerson::_bbox_upper_left_x_type arg)
  {
    msg_.bbox_upper_left_x = std::move(arg);
    return Init_TrackedPerson_bbox_upper_left_y(msg_);
  }

private:
  ::track_ped_msgs::msg::TrackedPerson msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::track_ped_msgs::msg::TrackedPerson>()
{
  return track_ped_msgs::msg::builder::Init_TrackedPerson_bbox_upper_left_x();
}

}  // namespace track_ped_msgs

#endif  // TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSON__BUILDER_HPP_
