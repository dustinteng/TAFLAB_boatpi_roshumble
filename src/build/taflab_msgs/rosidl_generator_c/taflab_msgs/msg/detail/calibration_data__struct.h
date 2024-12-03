// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__STRUCT_H_
#define TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CalibrationData in the package taflab_msgs.
typedef struct taflab_msgs__msg__CalibrationData
{
  /// Minimum angle for the rudder in degrees
  float rudder_min;
  /// Maximum angle for the rudder in degrees
  float rudder_max;
  /// Minimum angle for the sail in degrees
  float sail_min;
  /// Maximum angle for the sail in degrees
  float sail_max;
  /// Minimum throttle for the ESC
  float esc_min;
  /// Maximum throttle for the ESC
  float esc_max;
} taflab_msgs__msg__CalibrationData;

// Struct for a sequence of taflab_msgs__msg__CalibrationData.
typedef struct taflab_msgs__msg__CalibrationData__Sequence
{
  taflab_msgs__msg__CalibrationData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} taflab_msgs__msg__CalibrationData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__STRUCT_H_
