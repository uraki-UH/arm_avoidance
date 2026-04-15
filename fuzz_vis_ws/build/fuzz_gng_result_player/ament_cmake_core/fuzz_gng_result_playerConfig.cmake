# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fuzz_gng_result_player_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fuzz_gng_result_player_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fuzz_gng_result_player_FOUND FALSE)
  elseif(NOT fuzz_gng_result_player_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fuzz_gng_result_player_FOUND FALSE)
  endif()
  return()
endif()
set(_fuzz_gng_result_player_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fuzz_gng_result_player_FIND_QUIETLY)
  message(STATUS "Found fuzz_gng_result_player: 0.1.0 (${fuzz_gng_result_player_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fuzz_gng_result_player' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fuzz_gng_result_player_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fuzz_gng_result_player_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fuzz_gng_result_player_DIR}/${_extra}")
endforeach()
