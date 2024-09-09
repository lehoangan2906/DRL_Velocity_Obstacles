// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cnn_msgs:msg/CnnData.idl
// generated code does not contain a copyright notice
#include "cnn_msgs/msg/detail/cnn_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ped_pos_map`
// Member `scan`
// Member `scan_all`
// Member `image_gray`
// Member `depth`
// Member `goal_cart`
// Member `goal_final_polar`
// Member `vel`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
cnn_msgs__msg__CnnData__init(cnn_msgs__msg__CnnData * msg)
{
  if (!msg) {
    return false;
  }
  // ped_pos_map
  if (!rosidl_runtime_c__float__Sequence__init(&msg->ped_pos_map, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  // scan
  if (!rosidl_runtime_c__float__Sequence__init(&msg->scan, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  // scan_all
  if (!rosidl_runtime_c__float__Sequence__init(&msg->scan_all, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  // image_gray
  if (!rosidl_runtime_c__float__Sequence__init(&msg->image_gray, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  // depth
  if (!rosidl_runtime_c__float__Sequence__init(&msg->depth, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  // goal_cart
  if (!rosidl_runtime_c__float__Sequence__init(&msg->goal_cart, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  // goal_final_polar
  if (!rosidl_runtime_c__float__Sequence__init(&msg->goal_final_polar, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  // vel
  if (!rosidl_runtime_c__float__Sequence__init(&msg->vel, 0)) {
    cnn_msgs__msg__CnnData__fini(msg);
    return false;
  }
  return true;
}

void
cnn_msgs__msg__CnnData__fini(cnn_msgs__msg__CnnData * msg)
{
  if (!msg) {
    return;
  }
  // ped_pos_map
  rosidl_runtime_c__float__Sequence__fini(&msg->ped_pos_map);
  // scan
  rosidl_runtime_c__float__Sequence__fini(&msg->scan);
  // scan_all
  rosidl_runtime_c__float__Sequence__fini(&msg->scan_all);
  // image_gray
  rosidl_runtime_c__float__Sequence__fini(&msg->image_gray);
  // depth
  rosidl_runtime_c__float__Sequence__fini(&msg->depth);
  // goal_cart
  rosidl_runtime_c__float__Sequence__fini(&msg->goal_cart);
  // goal_final_polar
  rosidl_runtime_c__float__Sequence__fini(&msg->goal_final_polar);
  // vel
  rosidl_runtime_c__float__Sequence__fini(&msg->vel);
}

bool
cnn_msgs__msg__CnnData__are_equal(const cnn_msgs__msg__CnnData * lhs, const cnn_msgs__msg__CnnData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ped_pos_map
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->ped_pos_map), &(rhs->ped_pos_map)))
  {
    return false;
  }
  // scan
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->scan), &(rhs->scan)))
  {
    return false;
  }
  // scan_all
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->scan_all), &(rhs->scan_all)))
  {
    return false;
  }
  // image_gray
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->image_gray), &(rhs->image_gray)))
  {
    return false;
  }
  // depth
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->depth), &(rhs->depth)))
  {
    return false;
  }
  // goal_cart
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->goal_cart), &(rhs->goal_cart)))
  {
    return false;
  }
  // goal_final_polar
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->goal_final_polar), &(rhs->goal_final_polar)))
  {
    return false;
  }
  // vel
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->vel), &(rhs->vel)))
  {
    return false;
  }
  return true;
}

bool
cnn_msgs__msg__CnnData__copy(
  const cnn_msgs__msg__CnnData * input,
  cnn_msgs__msg__CnnData * output)
{
  if (!input || !output) {
    return false;
  }
  // ped_pos_map
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->ped_pos_map), &(output->ped_pos_map)))
  {
    return false;
  }
  // scan
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->scan), &(output->scan)))
  {
    return false;
  }
  // scan_all
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->scan_all), &(output->scan_all)))
  {
    return false;
  }
  // image_gray
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->image_gray), &(output->image_gray)))
  {
    return false;
  }
  // depth
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->depth), &(output->depth)))
  {
    return false;
  }
  // goal_cart
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->goal_cart), &(output->goal_cart)))
  {
    return false;
  }
  // goal_final_polar
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->goal_final_polar), &(output->goal_final_polar)))
  {
    return false;
  }
  // vel
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->vel), &(output->vel)))
  {
    return false;
  }
  return true;
}

cnn_msgs__msg__CnnData *
cnn_msgs__msg__CnnData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cnn_msgs__msg__CnnData * msg = (cnn_msgs__msg__CnnData *)allocator.allocate(sizeof(cnn_msgs__msg__CnnData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cnn_msgs__msg__CnnData));
  bool success = cnn_msgs__msg__CnnData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cnn_msgs__msg__CnnData__destroy(cnn_msgs__msg__CnnData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cnn_msgs__msg__CnnData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cnn_msgs__msg__CnnData__Sequence__init(cnn_msgs__msg__CnnData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cnn_msgs__msg__CnnData * data = NULL;

  if (size) {
    data = (cnn_msgs__msg__CnnData *)allocator.zero_allocate(size, sizeof(cnn_msgs__msg__CnnData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cnn_msgs__msg__CnnData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cnn_msgs__msg__CnnData__fini(&data[i - 1]);
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
cnn_msgs__msg__CnnData__Sequence__fini(cnn_msgs__msg__CnnData__Sequence * array)
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
      cnn_msgs__msg__CnnData__fini(&array->data[i]);
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

cnn_msgs__msg__CnnData__Sequence *
cnn_msgs__msg__CnnData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cnn_msgs__msg__CnnData__Sequence * array = (cnn_msgs__msg__CnnData__Sequence *)allocator.allocate(sizeof(cnn_msgs__msg__CnnData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cnn_msgs__msg__CnnData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cnn_msgs__msg__CnnData__Sequence__destroy(cnn_msgs__msg__CnnData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cnn_msgs__msg__CnnData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cnn_msgs__msg__CnnData__Sequence__are_equal(const cnn_msgs__msg__CnnData__Sequence * lhs, const cnn_msgs__msg__CnnData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cnn_msgs__msg__CnnData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cnn_msgs__msg__CnnData__Sequence__copy(
  const cnn_msgs__msg__CnnData__Sequence * input,
  cnn_msgs__msg__CnnData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cnn_msgs__msg__CnnData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cnn_msgs__msg__CnnData * data =
      (cnn_msgs__msg__CnnData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cnn_msgs__msg__CnnData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cnn_msgs__msg__CnnData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cnn_msgs__msg__CnnData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
