// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice
#include "taflab_msgs/msg/detail/calibration_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "taflab_msgs/msg/detail/calibration_data__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace taflab_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_taflab_msgs
cdr_serialize(
  const taflab_msgs::msg::CalibrationData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: rudder_min
  cdr << ros_message.rudder_min;
  // Member: rudder_max
  cdr << ros_message.rudder_max;
  // Member: sail_min
  cdr << ros_message.sail_min;
  // Member: sail_max
  cdr << ros_message.sail_max;
  // Member: esc_min
  cdr << ros_message.esc_min;
  // Member: esc_max
  cdr << ros_message.esc_max;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_taflab_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  taflab_msgs::msg::CalibrationData & ros_message)
{
  // Member: rudder_min
  cdr >> ros_message.rudder_min;

  // Member: rudder_max
  cdr >> ros_message.rudder_max;

  // Member: sail_min
  cdr >> ros_message.sail_min;

  // Member: sail_max
  cdr >> ros_message.sail_max;

  // Member: esc_min
  cdr >> ros_message.esc_min;

  // Member: esc_max
  cdr >> ros_message.esc_max;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_taflab_msgs
get_serialized_size(
  const taflab_msgs::msg::CalibrationData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: rudder_min
  {
    size_t item_size = sizeof(ros_message.rudder_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rudder_max
  {
    size_t item_size = sizeof(ros_message.rudder_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sail_min
  {
    size_t item_size = sizeof(ros_message.sail_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sail_max
  {
    size_t item_size = sizeof(ros_message.sail_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: esc_min
  {
    size_t item_size = sizeof(ros_message.esc_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: esc_max
  {
    size_t item_size = sizeof(ros_message.esc_max);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_taflab_msgs
max_serialized_size_CalibrationData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: rudder_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rudder_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: sail_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: sail_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: esc_min
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: esc_max
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = taflab_msgs::msg::CalibrationData;
    is_plain =
      (
      offsetof(DataType, esc_max) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _CalibrationData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const taflab_msgs::msg::CalibrationData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CalibrationData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<taflab_msgs::msg::CalibrationData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CalibrationData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const taflab_msgs::msg::CalibrationData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CalibrationData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_CalibrationData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _CalibrationData__callbacks = {
  "taflab_msgs::msg",
  "CalibrationData",
  _CalibrationData__cdr_serialize,
  _CalibrationData__cdr_deserialize,
  _CalibrationData__get_serialized_size,
  _CalibrationData__max_serialized_size
};

static rosidl_message_type_support_t _CalibrationData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CalibrationData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace taflab_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_taflab_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<taflab_msgs::msg::CalibrationData>()
{
  return &taflab_msgs::msg::typesupport_fastrtps_cpp::_CalibrationData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, taflab_msgs, msg, CalibrationData)() {
  return &taflab_msgs::msg::typesupport_fastrtps_cpp::_CalibrationData__handle;
}

#ifdef __cplusplus
}
#endif
