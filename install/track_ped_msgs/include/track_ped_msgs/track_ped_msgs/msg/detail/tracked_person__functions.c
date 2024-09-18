// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from track_ped_msgs:msg/TrackedPerson.idl
// generated code does not contain a copyright notice
#include "track_ped_msgs/msg/detail/tracked_person__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
track_ped_msgs__msg__TrackedPerson__init(track_ped_msgs__msg__TrackedPerson * msg)
{
  if (!msg) {
    return false;
  }
  // bbox_upper_left_x
  // bbox_upper_left_y
  // bbox_bottom_right_x
  // bbox_bottom_right_y
  // id
  // depth
  // angle
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    track_ped_msgs__msg__TrackedPerson__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    track_ped_msgs__msg__TrackedPerson__fini(msg);
    return false;
  }
  return true;
}

void
track_ped_msgs__msg__TrackedPerson__fini(track_ped_msgs__msg__TrackedPerson * msg)
{
  if (!msg) {
    return;
  }
  // bbox_upper_left_x
  // bbox_upper_left_y
  // bbox_bottom_right_x
  // bbox_bottom_right_y
  // id
  // depth
  // angle
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
track_ped_msgs__msg__TrackedPerson__are_equal(const track_ped_msgs__msg__TrackedPerson * lhs, const track_ped_msgs__msg__TrackedPerson * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bbox_upper_left_x
  if (lhs->bbox_upper_left_x != rhs->bbox_upper_left_x) {
    return false;
  }
  // bbox_upper_left_y
  if (lhs->bbox_upper_left_y != rhs->bbox_upper_left_y) {
    return false;
  }
  // bbox_bottom_right_x
  if (lhs->bbox_bottom_right_x != rhs->bbox_bottom_right_x) {
    return false;
  }
  // bbox_bottom_right_y
  if (lhs->bbox_bottom_right_y != rhs->bbox_bottom_right_y) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // depth
  if (lhs->depth != rhs->depth) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
track_ped_msgs__msg__TrackedPerson__copy(
  const track_ped_msgs__msg__TrackedPerson * input,
  track_ped_msgs__msg__TrackedPerson * output)
{
  if (!input || !output) {
    return false;
  }
  // bbox_upper_left_x
  output->bbox_upper_left_x = input->bbox_upper_left_x;
  // bbox_upper_left_y
  output->bbox_upper_left_y = input->bbox_upper_left_y;
  // bbox_bottom_right_x
  output->bbox_bottom_right_x = input->bbox_bottom_right_x;
  // bbox_bottom_right_y
  output->bbox_bottom_right_y = input->bbox_bottom_right_y;
  // id
  output->id = input->id;
  // depth
  output->depth = input->depth;
  // angle
  output->angle = input->angle;
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

track_ped_msgs__msg__TrackedPerson *
track_ped_msgs__msg__TrackedPerson__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_ped_msgs__msg__TrackedPerson * msg = (track_ped_msgs__msg__TrackedPerson *)allocator.allocate(sizeof(track_ped_msgs__msg__TrackedPerson), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(track_ped_msgs__msg__TrackedPerson));
  bool success = track_ped_msgs__msg__TrackedPerson__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
track_ped_msgs__msg__TrackedPerson__destroy(track_ped_msgs__msg__TrackedPerson * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    track_ped_msgs__msg__TrackedPerson__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
track_ped_msgs__msg__TrackedPerson__Sequence__init(track_ped_msgs__msg__TrackedPerson__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_ped_msgs__msg__TrackedPerson * data = NULL;

  if (size) {
    data = (track_ped_msgs__msg__TrackedPerson *)allocator.zero_allocate(size, sizeof(track_ped_msgs__msg__TrackedPerson), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = track_ped_msgs__msg__TrackedPerson__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        track_ped_msgs__msg__TrackedPerson__fini(&data[i - 1]);
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
track_ped_msgs__msg__TrackedPerson__Sequence__fini(track_ped_msgs__msg__TrackedPerson__Sequence * array)
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
      track_ped_msgs__msg__TrackedPerson__fini(&array->data[i]);
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

track_ped_msgs__msg__TrackedPerson__Sequence *
track_ped_msgs__msg__TrackedPerson__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_ped_msgs__msg__TrackedPerson__Sequence * array = (track_ped_msgs__msg__TrackedPerson__Sequence *)allocator.allocate(sizeof(track_ped_msgs__msg__TrackedPerson__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = track_ped_msgs__msg__TrackedPerson__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
track_ped_msgs__msg__TrackedPerson__Sequence__destroy(track_ped_msgs__msg__TrackedPerson__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    track_ped_msgs__msg__TrackedPerson__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
track_ped_msgs__msg__TrackedPerson__Sequence__are_equal(const track_ped_msgs__msg__TrackedPerson__Sequence * lhs, const track_ped_msgs__msg__TrackedPerson__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!track_ped_msgs__msg__TrackedPerson__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
track_ped_msgs__msg__TrackedPerson__Sequence__copy(
  const track_ped_msgs__msg__TrackedPerson__Sequence * input,
  track_ped_msgs__msg__TrackedPerson__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(track_ped_msgs__msg__TrackedPerson);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    track_ped_msgs__msg__TrackedPerson * data =
      (track_ped_msgs__msg__TrackedPerson *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!track_ped_msgs__msg__TrackedPerson__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          track_ped_msgs__msg__TrackedPerson__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!track_ped_msgs__msg__TrackedPerson__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
