// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from track_ped_msgs:msg/TrackedPerson.idl
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
#include "track_ped_msgs/msg/detail/tracked_person__struct.h"
#include "track_ped_msgs/msg/detail/tracked_person__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__twist__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__twist__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool track_ped_msgs__msg__tracked_person__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("track_ped_msgs.msg._tracked_person.TrackedPerson", full_classname_dest, 48) == 0);
  }
  track_ped_msgs__msg__TrackedPerson * ros_message = _ros_message;
  {  // bbox_upper_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "bbox_upper_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bbox_upper_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // bbox_upper_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "bbox_upper_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bbox_upper_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // bbox_bottom_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "bbox_bottom_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bbox_bottom_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // bbox_bottom_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "bbox_bottom_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bbox_bottom_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // twist
    PyObject * field = PyObject_GetAttrString(_pymsg, "twist");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__twist__convert_from_py(field, &ros_message->twist)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * track_ped_msgs__msg__tracked_person__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TrackedPerson */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("track_ped_msgs.msg._tracked_person");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TrackedPerson");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  track_ped_msgs__msg__TrackedPerson * ros_message = (track_ped_msgs__msg__TrackedPerson *)raw_ros_message;
  {  // bbox_upper_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->bbox_upper_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bbox_upper_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bbox_upper_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->bbox_upper_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bbox_upper_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bbox_bottom_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->bbox_bottom_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bbox_bottom_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bbox_bottom_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->bbox_bottom_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bbox_bottom_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // twist
    PyObject * field = NULL;
    field = geometry_msgs__msg__twist__convert_to_py(&ros_message->twist);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "twist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
