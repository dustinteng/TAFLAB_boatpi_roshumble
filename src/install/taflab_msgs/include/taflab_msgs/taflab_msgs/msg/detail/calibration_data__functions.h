// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__FUNCTIONS_H_
#define TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "taflab_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "taflab_msgs/msg/detail/calibration_data__struct.h"

/// Initialize msg/CalibrationData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * taflab_msgs__msg__CalibrationData
 * )) before or use
 * taflab_msgs__msg__CalibrationData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__CalibrationData__init(taflab_msgs__msg__CalibrationData * msg);

/// Finalize msg/CalibrationData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__CalibrationData__fini(taflab_msgs__msg__CalibrationData * msg);

/// Create msg/CalibrationData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * taflab_msgs__msg__CalibrationData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
taflab_msgs__msg__CalibrationData *
taflab_msgs__msg__CalibrationData__create();

/// Destroy msg/CalibrationData message.
/**
 * It calls
 * taflab_msgs__msg__CalibrationData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__CalibrationData__destroy(taflab_msgs__msg__CalibrationData * msg);

/// Check for msg/CalibrationData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__CalibrationData__are_equal(const taflab_msgs__msg__CalibrationData * lhs, const taflab_msgs__msg__CalibrationData * rhs);

/// Copy a msg/CalibrationData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__CalibrationData__copy(
  const taflab_msgs__msg__CalibrationData * input,
  taflab_msgs__msg__CalibrationData * output);

/// Initialize array of msg/CalibrationData messages.
/**
 * It allocates the memory for the number of elements and calls
 * taflab_msgs__msg__CalibrationData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__CalibrationData__Sequence__init(taflab_msgs__msg__CalibrationData__Sequence * array, size_t size);

/// Finalize array of msg/CalibrationData messages.
/**
 * It calls
 * taflab_msgs__msg__CalibrationData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__CalibrationData__Sequence__fini(taflab_msgs__msg__CalibrationData__Sequence * array);

/// Create array of msg/CalibrationData messages.
/**
 * It allocates the memory for the array and calls
 * taflab_msgs__msg__CalibrationData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
taflab_msgs__msg__CalibrationData__Sequence *
taflab_msgs__msg__CalibrationData__Sequence__create(size_t size);

/// Destroy array of msg/CalibrationData messages.
/**
 * It calls
 * taflab_msgs__msg__CalibrationData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__CalibrationData__Sequence__destroy(taflab_msgs__msg__CalibrationData__Sequence * array);

/// Check for msg/CalibrationData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__CalibrationData__Sequence__are_equal(const taflab_msgs__msg__CalibrationData__Sequence * lhs, const taflab_msgs__msg__CalibrationData__Sequence * rhs);

/// Copy an array of msg/CalibrationData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__CalibrationData__Sequence__copy(
  const taflab_msgs__msg__CalibrationData__Sequence * input,
  taflab_msgs__msg__CalibrationData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TAFLAB_MSGS__MSG__DETAIL__CALIBRATION_DATA__FUNCTIONS_H_
