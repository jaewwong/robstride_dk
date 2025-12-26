# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robstride_dk_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robstride_dk_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robstride_dk_FOUND FALSE)
  elseif(NOT robstride_dk_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robstride_dk_FOUND FALSE)
  endif()
  return()
endif()
set(_robstride_dk_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robstride_dk_FIND_QUIETLY)
  message(STATUS "Found robstride_dk: 0.0.0 (${robstride_dk_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robstride_dk' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robstride_dk_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robstride_dk_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robstride_dk_DIR}/${_extra}")
endforeach()
