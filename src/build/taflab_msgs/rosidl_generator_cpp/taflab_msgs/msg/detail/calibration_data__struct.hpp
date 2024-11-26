// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__STRUCT_HPP_
#define TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__taflab_msgs__msg__CalibrationData __attribute__((deprecated))
#else
# define DEPRECATED__taflab_msgs__msg__CalibrationData __declspec(deprecated)
#endif

namespace taflab_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CalibrationData_
{
  using Type = CalibrationData_<ContainerAllocator>;

  explicit CalibrationData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rudder_min = 0.0f;
      this->rudder_max = 0.0f;
      this->sail_min = 0.0f;
      this->sail_max = 0.0f;
      this->esc_min = 0.0f;
      this->esc_max = 0.0f;
    }
  }

  explicit CalibrationData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rudder_min = 0.0f;
      this->rudder_max = 0.0f;
      this->sail_min = 0.0f;
      this->sail_max = 0.0f;
      this->esc_min = 0.0f;
      this->esc_max = 0.0f;
    }
  }

  // field types and members
  using _rudder_min_type =
    float;
  _rudder_min_type rudder_min;
  using _rudder_max_type =
    float;
  _rudder_max_type rudder_max;
  using _sail_min_type =
    float;
  _sail_min_type sail_min;
  using _sail_max_type =
    float;
  _sail_max_type sail_max;
  using _esc_min_type =
    float;
  _esc_min_type esc_min;
  using _esc_max_type =
    float;
  _esc_max_type esc_max;

  // setters for named parameter idiom
  Type & set__rudder_min(
    const float & _arg)
  {
    this->rudder_min = _arg;
    return *this;
  }
  Type & set__rudder_max(
    const float & _arg)
  {
    this->rudder_max = _arg;
    return *this;
  }
  Type & set__sail_min(
    const float & _arg)
  {
    this->sail_min = _arg;
    return *this;
  }
  Type & set__sail_max(
    const float & _arg)
  {
    this->sail_max = _arg;
    return *this;
  }
  Type & set__esc_min(
    const float & _arg)
  {
    this->esc_min = _arg;
    return *this;
  }
  Type & set__esc_max(
    const float & _arg)
  {
    this->esc_max = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    taflab_msgs::msg::CalibrationData_<ContainerAllocator> *;
  using ConstRawPtr =
    const taflab_msgs::msg::CalibrationData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      taflab_msgs::msg::CalibrationData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      taflab_msgs::msg::CalibrationData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__taflab_msgs__msg__CalibrationData
    std::shared_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__taflab_msgs__msg__CalibrationData
    std::shared_ptr<taflab_msgs::msg::CalibrationData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrationData_ & other) const
  {
    if (this->rudder_min != other.rudder_min) {
      return false;
    }
    if (this->rudder_max != other.rudder_max) {
      return false;
    }
    if (this->sail_min != other.sail_min) {
      return false;
    }
    if (this->sail_max != other.sail_max) {
      return false;
    }
    if (this->esc_min != other.esc_min) {
      return false;
    }
    if (this->esc_max != other.esc_max) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrationData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrationData_

// alias to use template instance with default allocator
using CalibrationData =
  taflab_msgs::msg::CalibrationData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace taflab_msgs

#endif  // TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__STRUCT_HPP_
