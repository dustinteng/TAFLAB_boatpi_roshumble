// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__STRUCT_HPP_
#define TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__taflab_msgs__msg__ControlData __attribute__((deprecated))
#else
# define DEPRECATED__taflab_msgs__msg__ControlData __declspec(deprecated)
#endif

namespace taflab_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControlData_
{
  using Type = ControlData_<ContainerAllocator>;

  explicit ControlData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_rudder = 0.0f;
      this->servo_sail = 0.0f;
      this->esc = 0.0f;
    }
  }

  explicit ControlData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_rudder = 0.0f;
      this->servo_sail = 0.0f;
      this->esc = 0.0f;
    }
  }

  // field types and members
  using _servo_rudder_type =
    float;
  _servo_rudder_type servo_rudder;
  using _servo_sail_type =
    float;
  _servo_sail_type servo_sail;
  using _esc_type =
    float;
  _esc_type esc;

  // setters for named parameter idiom
  Type & set__servo_rudder(
    const float & _arg)
  {
    this->servo_rudder = _arg;
    return *this;
  }
  Type & set__servo_sail(
    const float & _arg)
  {
    this->servo_sail = _arg;
    return *this;
  }
  Type & set__esc(
    const float & _arg)
  {
    this->esc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    taflab_msgs::msg::ControlData_<ContainerAllocator> *;
  using ConstRawPtr =
    const taflab_msgs::msg::ControlData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      taflab_msgs::msg::ControlData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      taflab_msgs::msg::ControlData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__taflab_msgs__msg__ControlData
    std::shared_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__taflab_msgs__msg__ControlData
    std::shared_ptr<taflab_msgs::msg::ControlData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlData_ & other) const
  {
    if (this->servo_rudder != other.servo_rudder) {
      return false;
    }
    if (this->servo_sail != other.servo_sail) {
      return false;
    }
    if (this->esc != other.esc) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlData_

// alias to use template instance with default allocator
using ControlData =
  taflab_msgs::msg::ControlData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace taflab_msgs

#endif  // TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__STRUCT_HPP_
