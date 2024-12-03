// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "taflab_msgs/msg/detail/control_data__rosidl_typesupport_introspection_c.h"
#include "taflab_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "taflab_msgs/msg/detail/control_data__functions.h"
#include "taflab_msgs/msg/detail/control_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  taflab_msgs__msg__ControlData__init(message_memory);
}

void taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_fini_function(void * message_memory)
{
  taflab_msgs__msg__ControlData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_member_array[3] = {
  {
    "servo_rudder",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(taflab_msgs__msg__ControlData, servo_rudder),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "servo_sail",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(taflab_msgs__msg__ControlData, servo_sail),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "esc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(taflab_msgs__msg__ControlData, esc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_members = {
  "taflab_msgs__msg",  // message namespace
  "ControlData",  // message name
  3,  // number of fields
  sizeof(taflab_msgs__msg__ControlData),
  taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_member_array,  // message members
  taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_init_function,  // function to initialize message memory (memory has to be allocated)
  taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_type_support_handle = {
  0,
  &taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_taflab_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, taflab_msgs, msg, ControlData)() {
  if (!taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_type_support_handle.typesupport_identifier) {
    taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &taflab_msgs__msg__ControlData__rosidl_typesupport_introspection_c__ControlData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
