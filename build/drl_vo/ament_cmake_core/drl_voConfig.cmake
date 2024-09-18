# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_drl_vo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED drl_vo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(drl_vo_FOUND FALSE)
  elseif(NOT drl_vo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(drl_vo_FOUND FALSE)
  endif()
  return()
endif()
set(_drl_vo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT drl_vo_FIND_QUIETLY)
  message(STATUS "Found drl_vo: 0.0.1 (${drl_vo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'drl_vo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${drl_vo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(drl_vo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${drl_vo_DIR}/${_extra}")
endforeach()
