// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice
#include "taflab_msgs/msg/detail/calibration_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
taflab_msgs__msg__CalibrationData__init(taflab_msgs__msg__CalibrationData * msg)
{
  if (!msg) {
    return false;
  }
  // rudder_min
  // rudder_max
  // sail_min
  // sail_max
  // esc_min
  // esc_max
  return true;
}

void
taflab_msgs__msg__CalibrationData__fini(taflab_msgs__msg__CalibrationData * msg)
{
  if (!msg) {
    return;
  }
  // rudder_min
  // rudder_max
  // sail_min
  // sail_max
  // esc_min
  // esc_max
}

bool
taflab_msgs__msg__CalibrationData__are_equal(const taflab_msgs__msg__CalibrationData * lhs, const taflab_msgs__msg__CalibrationData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rudder_min
  if (lhs->rudder_min != rhs->rudder_min) {
    return false;
  }
  // rudder_max
  if (lhs->rudder_max != rhs->rudder_max) {
    return false;
  }
  // sail_min
  if (lhs->sail_min != rhs->sail_min) {
    return false;
  }
  // sail_max
  if (lhs->sail_max != rhs->sail_max) {
    return false;
  }
  // esc_min
  if (lhs->esc_min != rhs->esc_min) {
    return false;
  }
  // esc_max
  if (lhs->esc_max != rhs->esc_max) {
    return false;
  }
  return true;
}

bool
taflab_msgs__msg__CalibrationData__copy(
  const taflab_msgs__msg__CalibrationData * input,
  taflab_msgs__msg__CalibrationData * output)
{
  if (!input || !output) {
    return false;
  }
  // rudder_min
  output->rudder_min = input->rudder_min;
  // rudder_max
  output->rudder_max = input->rudder_max;
  // sail_min
  output->sail_min = input->sail_min;
  // sail_max
  output->sail_max = input->sail_max;
  // esc_min
  output->esc_min = input->esc_min;
  // esc_max
  output->esc_max = input->esc_max;
  return true;
}

taflab_msgs__msg__CalibrationData *
taflab_msgs__msg__CalibrationData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  taflab_msgs__msg__CalibrationData * msg = (taflab_msgs__msg__CalibrationData *)allocator.allocate(sizeof(taflab_msgs__msg__CalibrationData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(taflab_msgs__msg__CalibrationData));
  bool success = taflab_msgs__msg__CalibrationData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
taflab_msgs__msg__CalibrationData__destroy(taflab_msgs__msg__CalibrationData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    taflab_msgs__msg__CalibrationData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
taflab_msgs__msg__CalibrationData__Sequence__init(taflab_msgs__msg__CalibrationData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  taflab_msgs__msg__CalibrationData * data = NULL;

  if (size) {
    data = (taflab_msgs__msg__CalibrationData *)allocator.zero_allocate(size, sizeof(taflab_msgs__msg__CalibrationData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = taflab_msgs__msg__CalibrationData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        taflab_msgs__msg__CalibrationData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
taflab_msgs__msg__CalibrationData__Sequence__fini(taflab_msgs__msg__CalibrationData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      taflab_msgs__msg__CalibrationData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

taflab_msgs__msg__CalibrationData__Sequence *
taflab_msgs__msg__CalibrationData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  taflab_msgs__msg__CalibrationData__Sequence * array = (taflab_msgs__msg__CalibrationData__Sequence *)allocator.allocate(sizeof(taflab_msgs__msg__CalibrationData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = taflab_msgs__msg__CalibrationData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
taflab_msgs__msg__CalibrationData__Sequence__destroy(taflab_msgs__msg__CalibrationData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    taflab_msgs__msg__CalibrationData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
taflab_msgs__msg__CalibrationData__Sequence__are_equal(const taflab_msgs__msg__CalibrationData__Sequence * lhs, const taflab_msgs__msg__CalibrationData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!taflab_msgs__msg__CalibrationData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
taflab_msgs__msg__CalibrationData__Sequence__copy(
  const taflab_msgs__msg__CalibrationData__Sequence * input,
  taflab_msgs__msg__CalibrationData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(taflab_msgs__msg__CalibrationData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    taflab_msgs__msg__CalibrationData * data =
      (taflab_msgs__msg__CalibrationData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!taflab_msgs__msg__CalibrationData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          taflab_msgs__msg__CalibrationData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!taflab_msgs__msg__CalibrationData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
