// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from track_ped_msgs:msg/TrackedPersons.idl
// generated code does not contain a copyright notice

#ifndef TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSONS__FUNCTIONS_H_
#define TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSONS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "track_ped_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "track_ped_msgs/msg/detail/tracked_persons__struct.h"

/// Initialize msg/TrackedPersons message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * track_ped_msgs__msg__TrackedPersons
 * )) before or use
 * track_ped_msgs__msg__TrackedPersons__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
bool
track_ped_msgs__msg__TrackedPersons__init(track_ped_msgs__msg__TrackedPersons * msg);

/// Finalize msg/TrackedPersons message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
void
track_ped_msgs__msg__TrackedPersons__fini(track_ped_msgs__msg__TrackedPersons * msg);

/// Create msg/TrackedPersons message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * track_ped_msgs__msg__TrackedPersons__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
track_ped_msgs__msg__TrackedPersons *
track_ped_msgs__msg__TrackedPersons__create();

/// Destroy msg/TrackedPersons message.
/**
 * It calls
 * track_ped_msgs__msg__TrackedPersons__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
void
track_ped_msgs__msg__TrackedPersons__destroy(track_ped_msgs__msg__TrackedPersons * msg);

/// Check for msg/TrackedPersons message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
bool
track_ped_msgs__msg__TrackedPersons__are_equal(const track_ped_msgs__msg__TrackedPersons * lhs, const track_ped_msgs__msg__TrackedPersons * rhs);

/// Copy a msg/TrackedPersons message.
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
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
bool
track_ped_msgs__msg__TrackedPersons__copy(
  const track_ped_msgs__msg__TrackedPersons * input,
  track_ped_msgs__msg__TrackedPersons * output);

/// Initialize array of msg/TrackedPersons messages.
/**
 * It allocates the memory for the number of elements and calls
 * track_ped_msgs__msg__TrackedPersons__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
bool
track_ped_msgs__msg__TrackedPersons__Sequence__init(track_ped_msgs__msg__TrackedPersons__Sequence * array, size_t size);

/// Finalize array of msg/TrackedPersons messages.
/**
 * It calls
 * track_ped_msgs__msg__TrackedPersons__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
void
track_ped_msgs__msg__TrackedPersons__Sequence__fini(track_ped_msgs__msg__TrackedPersons__Sequence * array);

/// Create array of msg/TrackedPersons messages.
/**
 * It allocates the memory for the array and calls
 * track_ped_msgs__msg__TrackedPersons__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
track_ped_msgs__msg__TrackedPersons__Sequence *
track_ped_msgs__msg__TrackedPersons__Sequence__create(size_t size);

/// Destroy array of msg/TrackedPersons messages.
/**
 * It calls
 * track_ped_msgs__msg__TrackedPersons__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
void
track_ped_msgs__msg__TrackedPersons__Sequence__destroy(track_ped_msgs__msg__TrackedPersons__Sequence * array);

/// Check for msg/TrackedPersons message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
bool
track_ped_msgs__msg__TrackedPersons__Sequence__are_equal(const track_ped_msgs__msg__TrackedPersons__Sequence * lhs, const track_ped_msgs__msg__TrackedPersons__Sequence * rhs);

/// Copy an array of msg/TrackedPersons messages.
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
ROSIDL_GENERATOR_C_PUBLIC_track_ped_msgs
bool
track_ped_msgs__msg__TrackedPersons__Sequence__copy(
  const track_ped_msgs__msg__TrackedPersons__Sequence * input,
  track_ped_msgs__msg__TrackedPersons__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TRACK_PED_MSGS__MSG__DETAIL__TRACKED_PERSONS__FUNCTIONS_H_
