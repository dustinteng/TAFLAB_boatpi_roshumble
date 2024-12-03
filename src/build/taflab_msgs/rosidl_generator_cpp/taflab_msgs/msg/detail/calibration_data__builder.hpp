// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__BUILDER_HPP_
#define TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "taflab_msgs/msg/detail/calibration_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace taflab_msgs
{

namespace msg
{

namespace builder
{

class Init_CalibrationData_esc_max
{
public:
  explicit Init_CalibrationData_esc_max(::taflab_msgs::msg::CalibrationData & msg)
  : msg_(msg)
  {}
  ::taflab_msgs::msg::CalibrationData esc_max(::taflab_msgs::msg::CalibrationData::_esc_max_type arg)
  {
    msg_.esc_max = std::move(arg);
    return std::move(msg_);
  }

private:
  ::taflab_msgs::msg::CalibrationData msg_;
};

class Init_CalibrationData_esc_min
{
public:
  explicit Init_CalibrationData_esc_min(::taflab_msgs::msg::CalibrationData & msg)
  : msg_(msg)
  {}
  Init_CalibrationData_esc_max esc_min(::taflab_msgs::msg::CalibrationData::_esc_min_type arg)
  {
    msg_.esc_min = std::move(arg);
    return Init_CalibrationData_esc_max(msg_);
  }

private:
  ::taflab_msgs::msg::CalibrationData msg_;
};

class Init_CalibrationData_sail_max
{
public:
  explicit Init_CalibrationData_sail_max(::taflab_msgs::msg::CalibrationData & msg)
  : msg_(msg)
  {}
  Init_CalibrationData_esc_min sail_max(::taflab_msgs::msg::CalibrationData::_sail_max_type arg)
  {
    msg_.sail_max = std::move(arg);
    return Init_CalibrationData_esc_min(msg_);
  }

private:
  ::taflab_msgs::msg::CalibrationData msg_;
};

class Init_CalibrationData_sail_min
{
public:
  explicit Init_CalibrationData_sail_min(::taflab_msgs::msg::CalibrationData & msg)
  : msg_(msg)
  {}
  Init_CalibrationData_sail_max sail_min(::taflab_msgs::msg::CalibrationData::_sail_min_type arg)
  {
    msg_.sail_min = std::move(arg);
    return Init_CalibrationData_sail_max(msg_);
  }

private:
  ::taflab_msgs::msg::CalibrationData msg_;
};

class Init_CalibrationData_rudder_max
{
public:
  explicit Init_CalibrationData_rudder_max(::taflab_msgs::msg::CalibrationData & msg)
  : msg_(msg)
  {}
  Init_CalibrationData_sail_min rudder_max(::taflab_msgs::msg::CalibrationData::_rudder_max_type arg)
  {
    msg_.rudder_max = std::move(arg);
    return Init_CalibrationData_sail_min(msg_);
  }

private:
  ::taflab_msgs::msg::CalibrationData msg_;
};

class Init_CalibrationData_rudder_min
{
public:
  Init_CalibrationData_rudder_min()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrationData_rudder_max rudder_min(::taflab_msgs::msg::CalibrationData::_rudder_min_type arg)
  {
    msg_.rudder_min = std::move(arg);
    return Init_CalibrationData_rudder_max(msg_);
  }

private:
  ::taflab_msgs::msg::CalibrationData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::taflab_msgs::msg::CalibrationData>()
{
  return taflab_msgs::msg::builder::Init_CalibrationData_rudder_min();
}

}  // namespace taflab_msgs

#endif  // TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__BUILDER_HPP_
