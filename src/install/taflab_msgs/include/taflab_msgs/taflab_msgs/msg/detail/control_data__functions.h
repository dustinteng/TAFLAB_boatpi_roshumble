// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice

#ifndef TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__FUNCTIONS_H_
#define TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "taflab_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "taflab_msgs/msg/detail/control_data__struct.h"

/// Initialize msg/ControlData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * taflab_msgs__msg__ControlData
 * )) before or use
 * taflab_msgs__msg__ControlData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__ControlData__init(taflab_msgs__msg__ControlData * msg);

/// Finalize msg/ControlData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__ControlData__fini(taflab_msgs__msg__ControlData * msg);

/// Create msg/ControlData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * taflab_msgs__msg__ControlData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
taflab_msgs__msg__ControlData *
taflab_msgs__msg__ControlData__create();

/// Destroy msg/ControlData message.
/**
 * It calls
 * taflab_msgs__msg__ControlData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__ControlData__destroy(taflab_msgs__msg__ControlData * msg);

/// Check for msg/ControlData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__ControlData__are_equal(const taflab_msgs__msg__ControlData * lhs, const taflab_msgs__msg__ControlData * rhs);

/// Copy a msg/ControlData message.
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
taflab_msgs__msg__ControlData__copy(
  const taflab_msgs__msg__ControlData * input,
  taflab_msgs__msg__ControlData * output);

/// Initialize array of msg/ControlData messages.
/**
 * It allocates the memory for the number of elements and calls
 * taflab_msgs__msg__ControlData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__ControlData__Sequence__init(taflab_msgs__msg__ControlData__Sequence * array, size_t size);

/// Finalize array of msg/ControlData messages.
/**
 * It calls
 * taflab_msgs__msg__ControlData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__ControlData__Sequence__fini(taflab_msgs__msg__ControlData__Sequence * array);

/// Create array of msg/ControlData messages.
/**
 * It allocates the memory for the array and calls
 * taflab_msgs__msg__ControlData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
taflab_msgs__msg__ControlData__Sequence *
taflab_msgs__msg__ControlData__Sequence__create(size_t size);

/// Destroy array of msg/ControlData messages.
/**
 * It calls
 * taflab_msgs__msg__ControlData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
void
taflab_msgs__msg__ControlData__Sequence__destroy(taflab_msgs__msg__ControlData__Sequence * array);

/// Check for msg/ControlData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_taflab_msgs
bool
taflab_msgs__msg__ControlData__Sequence__are_equal(const taflab_msgs__msg__ControlData__Sequence * lhs, const taflab_msgs__msg__ControlData__Sequence * rhs);

/// Copy an array of msg/ControlData messages.
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
taflab_msgs__msg__ControlData__Sequence__copy(
  const taflab_msgs__msg__ControlData__Sequence * input,
  taflab_msgs__msg__ControlData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TAFLAB_MSGS__MSG__DETAIL__CONTROL_DATA__FUNCTIONS_H_
