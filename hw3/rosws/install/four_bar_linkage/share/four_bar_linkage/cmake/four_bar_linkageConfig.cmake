# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_four_bar_linkage_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED four_bar_linkage_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(four_bar_linkage_FOUND FALSE)
  elseif(NOT four_bar_linkage_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(four_bar_linkage_FOUND FALSE)
  endif()
  return()
endif()
set(_four_bar_linkage_CONFIG_INCLUDED TRUE)

# output package information
if(NOT four_bar_linkage_FIND_QUIETLY)
  message(STATUS "Found four_bar_linkage: 0.0.0 (${four_bar_linkage_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'four_bar_linkage' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${four_bar_linkage_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(four_bar_linkage_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${four_bar_linkage_DIR}/${_extra}")
endforeach()
