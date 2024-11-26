// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__TRAITS_HPP_
#define TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "taflab_msgs/msg/detail/calibration_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace taflab_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CalibrationData & msg,
  std::ostream & out)
{
  out << "{";
  // member: rudder_min
  {
    out << "rudder_min: ";
    rosidl_generator_traits::value_to_yaml(msg.rudder_min, out);
    out << ", ";
  }

  // member: rudder_max
  {
    out << "rudder_max: ";
    rosidl_generator_traits::value_to_yaml(msg.rudder_max, out);
    out << ", ";
  }

  // member: sail_min
  {
    out << "sail_min: ";
    rosidl_generator_traits::value_to_yaml(msg.sail_min, out);
    out << ", ";
  }

  // member: sail_max
  {
    out << "sail_max: ";
    rosidl_generator_traits::value_to_yaml(msg.sail_max, out);
    out << ", ";
  }

  // member: esc_min
  {
    out << "esc_min: ";
    rosidl_generator_traits::value_to_yaml(msg.esc_min, out);
    out << ", ";
  }

  // member: esc_max
  {
    out << "esc_max: ";
    rosidl_generator_traits::value_to_yaml(msg.esc_max, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrationData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rudder_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rudder_min: ";
    rosidl_generator_traits::value_to_yaml(msg.rudder_min, out);
    out << "\n";
  }

  // member: rudder_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rudder_max: ";
    rosidl_generator_traits::value_to_yaml(msg.rudder_max, out);
    out << "\n";
  }

  // member: sail_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sail_min: ";
    rosidl_generator_traits::value_to_yaml(msg.sail_min, out);
    out << "\n";
  }

  // member: sail_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sail_max: ";
    rosidl_generator_traits::value_to_yaml(msg.sail_max, out);
    out << "\n";
  }

  // member: esc_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "esc_min: ";
    rosidl_generator_traits::value_to_yaml(msg.esc_min, out);
    out << "\n";
  }

  // member: esc_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "esc_max: ";
    rosidl_generator_traits::value_to_yaml(msg.esc_max, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrationData & msg, bool use_flow_style = false)
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
  const taflab_msgs::msg::CalibrationData & msg,
  std::ostream & out, size_t indentation = 0)
{
  taflab_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use taflab_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const taflab_msgs::msg::CalibrationData & msg)
{
  return taflab_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<taflab_msgs::msg::CalibrationData>()
{
  return "taflab_msgs::msg::CalibrationData";
}

template<>
inline const char * name<taflab_msgs::msg::CalibrationData>()
{
  return "taflab_msgs/msg/CalibrationData";
}

template<>
struct has_fixed_size<taflab_msgs::msg::CalibrationData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<taflab_msgs::msg::CalibrationData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<taflab_msgs::msg::CalibrationData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__TRAITS_HPP_
