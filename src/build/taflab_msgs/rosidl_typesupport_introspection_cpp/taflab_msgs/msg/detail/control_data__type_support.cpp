// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "taflab_msgs/msg/detail/control_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace taflab_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ControlData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) taflab_msgs::msg::ControlData(_init);
}

void ControlData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<taflab_msgs::msg::ControlData *>(message_memory);
  typed_message->~ControlData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ControlData_message_member_array[3] = {
  {
    "servo_rudder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(taflab_msgs::msg::ControlData, servo_rudder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "servo_sail",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(taflab_msgs::msg::ControlData, servo_sail),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "esc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(taflab_msgs::msg::ControlData, esc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ControlData_message_members = {
  "taflab_msgs::msg",  // message namespace
  "ControlData",  // message name
  3,  // number of fields
  sizeof(taflab_msgs::msg::ControlData),
  ControlData_message_member_array,  // message members
  ControlData_init_function,  // function to initialize message memory (memory has to be allocated)
  ControlData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ControlData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ControlData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace taflab_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<taflab_msgs::msg::ControlData>()
{
  return &::taflab_msgs::msg::rosidl_typesupport_introspection_cpp::ControlData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, taflab_msgs, msg, ControlData)() {
  return &::taflab_msgs::msg::rosidl_typesupport_introspection_cpp::ControlData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
