// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from track_ped_msgs:msg/TrackedPersons.idl
// generated code does not contain a copyright notice
#include "track_ped_msgs/msg/detail/tracked_persons__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `tracks`
#include "track_ped_msgs/msg/detail/tracked_person__functions.h"

bool
track_ped_msgs__msg__TrackedPersons__init(track_ped_msgs__msg__TrackedPersons * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    track_ped_msgs__msg__TrackedPersons__fini(msg);
    return false;
  }
  // tracks
  if (!track_ped_msgs__msg__TrackedPerson__Sequence__init(&msg->tracks, 0)) {
    track_ped_msgs__msg__TrackedPersons__fini(msg);
    return false;
  }
  return true;
}

void
track_ped_msgs__msg__TrackedPersons__fini(track_ped_msgs__msg__TrackedPersons * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // tracks
  track_ped_msgs__msg__TrackedPerson__Sequence__fini(&msg->tracks);
}

bool
track_ped_msgs__msg__TrackedPersons__are_equal(const track_ped_msgs__msg__TrackedPersons * lhs, const track_ped_msgs__msg__TrackedPersons * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // tracks
  if (!track_ped_msgs__msg__TrackedPerson__Sequence__are_equal(
      &(lhs->tracks), &(rhs->tracks)))
  {
    return false;
  }
  return true;
}

bool
track_ped_msgs__msg__TrackedPersons__copy(
  const track_ped_msgs__msg__TrackedPersons * input,
  track_ped_msgs__msg__TrackedPersons * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // tracks
  if (!track_ped_msgs__msg__TrackedPerson__Sequence__copy(
      &(input->tracks), &(output->tracks)))
  {
    return false;
  }
  return true;
}

track_ped_msgs__msg__TrackedPersons *
track_ped_msgs__msg__TrackedPersons__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_ped_msgs__msg__TrackedPersons * msg = (track_ped_msgs__msg__TrackedPersons *)allocator.allocate(sizeof(track_ped_msgs__msg__TrackedPersons), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(track_ped_msgs__msg__TrackedPersons));
  bool success = track_ped_msgs__msg__TrackedPersons__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
track_ped_msgs__msg__TrackedPersons__destroy(track_ped_msgs__msg__TrackedPersons * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    track_ped_msgs__msg__TrackedPersons__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
track_ped_msgs__msg__TrackedPersons__Sequence__init(track_ped_msgs__msg__TrackedPersons__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_ped_msgs__msg__TrackedPersons * data = NULL;

  if (size) {
    data = (track_ped_msgs__msg__TrackedPersons *)allocator.zero_allocate(size, sizeof(track_ped_msgs__msg__TrackedPersons), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = track_ped_msgs__msg__TrackedPersons__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        track_ped_msgs__msg__TrackedPersons__fini(&data[i - 1]);
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
track_ped_msgs__msg__TrackedPersons__Sequence__fini(track_ped_msgs__msg__TrackedPersons__Sequence * array)
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
      track_ped_msgs__msg__TrackedPersons__fini(&array->data[i]);
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

track_ped_msgs__msg__TrackedPersons__Sequence *
track_ped_msgs__msg__TrackedPersons__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_ped_msgs__msg__TrackedPersons__Sequence * array = (track_ped_msgs__msg__TrackedPersons__Sequence *)allocator.allocate(sizeof(track_ped_msgs__msg__TrackedPersons__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = track_ped_msgs__msg__TrackedPersons__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
track_ped_msgs__msg__TrackedPersons__Sequence__destroy(track_ped_msgs__msg__TrackedPersons__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    track_ped_msgs__msg__TrackedPersons__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
track_ped_msgs__msg__TrackedPersons__Sequence__are_equal(const track_ped_msgs__msg__TrackedPersons__Sequence * lhs, const track_ped_msgs__msg__TrackedPersons__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!track_ped_msgs__msg__TrackedPersons__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
track_ped_msgs__msg__TrackedPersons__Sequence__copy(
  const track_ped_msgs__msg__TrackedPersons__Sequence * input,
  track_ped_msgs__msg__TrackedPersons__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(track_ped_msgs__msg__TrackedPersons);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    track_ped_msgs__msg__TrackedPersons * data =
      (track_ped_msgs__msg__TrackedPersons *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!track_ped_msgs__msg__TrackedPersons__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          track_ped_msgs__msg__TrackedPersons__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!track_ped_msgs__msg__TrackedPersons__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
