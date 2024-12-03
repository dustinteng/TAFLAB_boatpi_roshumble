// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from taflab_msgs:msg/CalibrationData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "taflab_msgs/msg/detail/calibration_data__struct.h"
#include "taflab_msgs/msg/detail/calibration_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool taflab_msgs__msg__calibration_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("taflab_msgs.msg._calibration_data.CalibrationData", full_classname_dest, 49) == 0);
  }
  taflab_msgs__msg__CalibrationData * ros_message = _ros_message;
  {  // rudder_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "rudder_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rudder_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rudder_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "rudder_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rudder_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // sail_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "sail_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sail_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // sail_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "sail_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sail_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // esc_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "esc_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->esc_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // esc_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "esc_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->esc_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * taflab_msgs__msg__calibration_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CalibrationData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("taflab_msgs.msg._calibration_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CalibrationData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  taflab_msgs__msg__CalibrationData * ros_message = (taflab_msgs__msg__CalibrationData *)raw_ros_message;
  {  // rudder_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rudder_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rudder_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rudder_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rudder_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rudder_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sail_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sail_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sail_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sail_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sail_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sail_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // esc_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->esc_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "esc_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // esc_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->esc_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "esc_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
