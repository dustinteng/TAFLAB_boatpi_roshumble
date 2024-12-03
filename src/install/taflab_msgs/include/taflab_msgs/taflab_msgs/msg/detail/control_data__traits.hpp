// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__TRAITS_HPP_
#define TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "taflab_msgs/msg/detail/control_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace taflab_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlData & msg,
  std::ostream & out)
{
  out << "{";
  // member: servo_rudder
  {
    out << "servo_rudder: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_rudder, out);
    out << ", ";
  }

  // member: servo_sail
  {
    out << "servo_sail: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_sail, out);
    out << ", ";
  }

  // member: esc
  {
    out << "esc: ";
    rosidl_generator_traits::value_to_yaml(msg.esc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: servo_rudder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo_rudder: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_rudder, out);
    out << "\n";
  }

  // member: servo_sail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo_sail: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_sail, out);
    out << "\n";
  }

  // member: esc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "esc: ";
    rosidl_generator_traits::value_to_yaml(msg.esc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlData & msg, bool use_flow_style = false)
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

}  // namespace taflab_msgs

namespace rosidl_generator_traits
{

[[deprecated("use taflab_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const taflab_msgs::msg::ControlData & msg,
  std::ostream & out, size_t indentation = 0)
{
  taflab_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use taflab_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const taflab_msgs::msg::ControlData & msg)
{
  return taflab_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<taflab_msgs::msg::ControlData>()
{
  return "taflab_msgs::msg::ControlData";
}

template<>
inline const char * name<taflab_msgs::msg::ControlData>()
{
  return "taflab_msgs/msg/ControlData";
}

template<>
struct has_fixed_size<taflab_msgs::msg::ControlData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<taflab_msgs::msg::ControlData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<taflab_msgs::msg::ControlData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__TRAITS_HPP_
