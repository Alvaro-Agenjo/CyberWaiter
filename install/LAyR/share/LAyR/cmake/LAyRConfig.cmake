# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_LAyR_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED LAyR_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(LAyR_FOUND FALSE)
  elseif(NOT LAyR_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(LAyR_FOUND FALSE)
  endif()
  return()
endif()
set(_LAyR_CONFIG_INCLUDED TRUE)

# output package information
if(NOT LAyR_FIND_QUIETLY)
  message(STATUS "Found LAyR: 0.0.0 (${LAyR_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'LAyR' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${LAyR_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(LAyR_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${LAyR_DIR}/${_extra}")
endforeach()
