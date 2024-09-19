// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cnn_msgs:msg/CnnData.idl
// generated code does not contain a copyright notice

#ifndef CNN_MSGS__MSG__DETAIL__CNN_DATA__TRAITS_HPP_
#define CNN_MSGS__MSG__DETAIL__CNN_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cnn_msgs/msg/detail/cnn_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cnn_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CnnData & msg,
  std::ostream & out)
{
  out << "{";
  // member: ped_pos_map
  {
    if (msg.ped_pos_map.size() == 0) {
      out << "ped_pos_map: []";
    } else {
      out << "ped_pos_map: [";
      size_t pending_items = msg.ped_pos_map.size();
      for (auto item : msg.ped_pos_map) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: scan
  {
    if (msg.scan.size() == 0) {
      out << "scan: []";
    } else {
      out << "scan: [";
      size_t pending_items = msg.scan.size();
      for (auto item : msg.scan) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: scan_all
  {
    if (msg.scan_all.size() == 0) {
      out << "scan_all: []";
    } else {
      out << "scan_all: [";
      size_t pending_items = msg.scan_all.size();
      for (auto item : msg.scan_all) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: image_gray
  {
    if (msg.image_gray.size() == 0) {
      out << "image_gray: []";
    } else {
      out << "image_gray: [";
      size_t pending_items = msg.image_gray.size();
      for (auto item : msg.image_gray) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: depth
  {
    if (msg.depth.size() == 0) {
      out << "depth: []";
    } else {
      out << "depth: [";
      size_t pending_items = msg.depth.size();
      for (auto item : msg.depth) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: goal_cart
  {
    if (msg.goal_cart.size() == 0) {
      out << "goal_cart: []";
    } else {
      out << "goal_cart: [";
      size_t pending_items = msg.goal_cart.size();
      for (auto item : msg.goal_cart) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: goal_final_polar
  {
    if (msg.goal_final_polar.size() == 0) {
      out << "goal_final_polar: []";
    } else {
      out << "goal_final_polar: [";
      size_t pending_items = msg.goal_final_polar.size();
      for (auto item : msg.goal_final_polar) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel
  {
    if (msg.vel.size() == 0) {
      out << "vel: []";
    } else {
      out << "vel: [";
      size_t pending_items = msg.vel.size();
      for (auto item : msg.vel) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CnnData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ped_pos_map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ped_pos_map.size() == 0) {
      out << "ped_pos_map: []\n";
    } else {
      out << "ped_pos_map:\n";
      for (auto item : msg.ped_pos_map) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: scan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scan.size() == 0) {
      out << "scan: []\n";
    } else {
      out << "scan:\n";
      for (auto item : msg.scan) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: scan_all
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scan_all.size() == 0) {
      out << "scan_all: []\n";
    } else {
      out << "scan_all:\n";
      for (auto item : msg.scan_all) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: image_gray
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.image_gray.size() == 0) {
      out << "image_gray: []\n";
    } else {
      out << "image_gray:\n";
      for (auto item : msg.image_gray) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.depth.size() == 0) {
      out << "depth: []\n";
    } else {
      out << "depth:\n";
      for (auto item : msg.depth) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: goal_cart
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.goal_cart.size() == 0) {
      out << "goal_cart: []\n";
    } else {
      out << "goal_cart:\n";
      for (auto item : msg.goal_cart) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: goal_final_polar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.goal_final_polar.size() == 0) {
      out << "goal_final_polar: []\n";
    } else {
      out << "goal_final_polar:\n";
      for (auto item : msg.goal_final_polar) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel.size() == 0) {
      out << "vel: []\n";
    } else {
      out << "vel:\n";
      for (auto item : msg.vel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CnnData & msg, bool use_flow_style = false)
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

}  // namespace cnn_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cnn_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cnn_msgs::msg::CnnData & msg,
  std::ostream & out, size_t indentation = 0)
{
  cnn_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cnn_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cnn_msgs::msg::CnnData & msg)
{
  return cnn_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cnn_msgs::msg::CnnData>()
{
  return "cnn_msgs::msg::CnnData";
}

template<>
inline const char * name<cnn_msgs::msg::CnnData>()
{
  return "cnn_msgs/msg/CnnData";
}

template<>
struct has_fixed_size<cnn_msgs::msg::CnnData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cnn_msgs::msg::CnnData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cnn_msgs::msg::CnnData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CNN_MSGS__MSG__DETAIL__CNN_DATA__TRAITS_HPP_
