// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__STRUCT_H_
#define TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ControlData in the package taflab_msgs.
typedef struct taflab_msgs__msg__ControlData
{
  /// Control data for the servo rudder
  float servo_rudder;
  /// Control data for the servo sail
  float servo_sail;
  /// Optional ESC control data (or use NaN as a placeholder if not available)
  float esc;
} taflab_msgs__msg__ControlData;

// Struct for a sequence of taflab_msgs__msg__ControlData.
typedef struct taflab_msgs__msg__ControlData__Sequence
{
  taflab_msgs__msg__ControlData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} taflab_msgs__msg__ControlData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__STRUCT_H_
