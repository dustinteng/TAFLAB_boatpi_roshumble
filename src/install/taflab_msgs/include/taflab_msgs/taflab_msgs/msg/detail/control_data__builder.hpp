// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__BUILDER_HPP_
#define TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "taflab_msgs/msg/detail/control_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace taflab_msgs
{

namespace msg
{

namespace builder
{

class Init_ControlData_esc
{
public:
  explicit Init_ControlData_esc(::taflab_msgs::msg::ControlData & msg)
  : msg_(msg)
  {}
  ::taflab_msgs::msg::ControlData esc(::taflab_msgs::msg::ControlData::_esc_type arg)
  {
    msg_.esc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::taflab_msgs::msg::ControlData msg_;
};

class Init_ControlData_servo_sail
{
public:
  explicit Init_ControlData_servo_sail(::taflab_msgs::msg::ControlData & msg)
  : msg_(msg)
  {}
  Init_ControlData_esc servo_sail(::taflab_msgs::msg::ControlData::_servo_sail_type arg)
  {
    msg_.servo_sail = std::move(arg);
    return Init_ControlData_esc(msg_);
  }

private:
  ::taflab_msgs::msg::ControlData msg_;
};

class Init_ControlData_servo_rudder
{
public:
  Init_ControlData_servo_rudder()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlData_servo_sail servo_rudder(::taflab_msgs::msg::ControlData::_servo_rudder_type arg)
  {
    msg_.servo_rudder = std::move(arg);
    return Init_ControlData_servo_sail(msg_);
  }

private:
  ::taflab_msgs::msg::ControlData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::taflab_msgs::msg::ControlData>()
{
  return taflab_msgs::msg::builder::Init_ControlData_servo_rudder();
}

}  // namespace taflab_msgs

#endif  // TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__BUILDER_HPP_
