# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sec_description_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sec_description_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sec_description_FOUND FALSE)
  elseif(NOT sec_description_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sec_description_FOUND FALSE)
  endif()
  return()
endif()
set(_sec_description_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sec_description_FIND_QUIETLY)
  message(STATUS "Found sec_description: 0.0.0 (${sec_description_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sec_description' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sec_description_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sec_description_DIR}/${_extra}")
endforeach()
