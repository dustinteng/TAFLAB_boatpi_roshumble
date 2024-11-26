// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from taflab_msgs:msg/ControlData.idl
// generated code does not contain a copyright notice
#include "taflab_msgs/msg/detail/control_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
taflab_msgs__msg__ControlData__init(taflab_msgs__msg__ControlData * msg)
{
  if (!msg) {
    return false;
  }
  // servo_rudder
  // servo_sail
  // esc
  return true;
}

void
taflab_msgs__msg__ControlData__fini(taflab_msgs__msg__ControlData * msg)
{
  if (!msg) {
    return;
  }
  // servo_rudder
  // servo_sail
  // esc
}

bool
taflab_msgs__msg__ControlData__are_equal(const taflab_msgs__msg__ControlData * lhs, const taflab_msgs__msg__ControlData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // servo_rudder
  if (lhs->servo_rudder != rhs->servo_rudder) {
    return false;
  }
  // servo_sail
  if (lhs->servo_sail != rhs->servo_sail) {
    return false;
  }
  // esc
  if (lhs->esc != rhs->esc) {
    return false;
  }
  return true;
}

bool
taflab_msgs__msg__ControlData__copy(
  const taflab_msgs__msg__ControlData * input,
  taflab_msgs__msg__ControlData * output)
{
  if (!input || !output) {
    return false;
  }
  // servo_rudder
  output->servo_rudder = input->servo_rudder;
  // servo_sail
  output->servo_sail = input->servo_sail;
  // esc
  output->esc = input->esc;
  return true;
}

taflab_msgs__msg__ControlData *
taflab_msgs__msg__ControlData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  taflab_msgs__msg__ControlData * msg = (taflab_msgs__msg__ControlData *)allocator.allocate(sizeof(taflab_msgs__msg__ControlData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(taflab_msgs__msg__ControlData));
  bool success = taflab_msgs__msg__ControlData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
taflab_msgs__msg__ControlData__destroy(taflab_msgs__msg__ControlData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    taflab_msgs__msg__ControlData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
taflab_msgs__msg__ControlData__Sequence__init(taflab_msgs__msg__ControlData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  taflab_msgs__msg__ControlData * data = NULL;

  if (size) {
    data = (taflab_msgs__msg__ControlData *)allocator.zero_allocate(size, sizeof(taflab_msgs__msg__ControlData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = taflab_msgs__msg__ControlData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        taflab_msgs__msg__ControlData__fini(&data[i - 1]);
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
taflab_msgs__msg__ControlData__Sequence__fini(taflab_msgs__msg__ControlData__Sequence * array)
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
      taflab_msgs__msg__ControlData__fini(&array->data[i]);
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

taflab_msgs__msg__ControlData__Sequence *
taflab_msgs__msg__ControlData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  taflab_msgs__msg__ControlData__Sequence * array = (taflab_msgs__msg__ControlData__Sequence *)allocator.allocate(sizeof(taflab_msgs__msg__ControlData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = taflab_msgs__msg__ControlData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
taflab_msgs__msg__ControlData__Sequence__destroy(taflab_msgs__msg__ControlData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    taflab_msgs__msg__ControlData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
taflab_msgs__msg__ControlData__Sequence__are_equal(const taflab_msgs__msg__ControlData__Sequence * lhs, const taflab_msgs__msg__ControlData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!taflab_msgs__msg__ControlData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
taflab_msgs__msg__ControlData__Sequence__copy(
  const taflab_msgs__msg__ControlData__Sequence * input,
  taflab_msgs__msg__ControlData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(taflab_msgs__msg__ControlData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    taflab_msgs__msg__ControlData * data =
      (taflab_msgs__msg__ControlData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!taflab_msgs__msg__ControlData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          taflab_msgs__msg__ControlData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!taflab_msgs__msg__ControlData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
