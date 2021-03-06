cmake_minimum_required(VERSION 3.6)

project(libcamera-apps)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    message(STATUS "No previous build - default to Release build")
  endif()
endif()

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE CXX_FLAGS "-Wall -Wextra -pedantic -Wno-unused-parameter -faligned-new")
add_definitions(-Wfatal-errors)
add_definitions(-Wno-psabi)
add_definitions(-DBOOST_LOG_DYN_LINK)

IF (NOT ENABLE_COMPILE_FLAGS_FOR_TARGET)
  # On a Pi this will give us armhf or arm64.
  execute_process(COMMAND dpkg-architecture -qDEB_HOST_ARCH
    OUTPUT_VARIABLE ENABLE_COMPILE_FLAGS_FOR_TARGET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
message(STATUS "Platform: ${ENABLE_COMPILE_FLAGS_FOR_TARGET}")
if ("${ENABLE_COMPILE_FLAGS_FOR_TARGET}" STREQUAL "arm64")
  # 64-bit binaries can be fully optimised.
  add_definitions(-ftree-vectorize)
elseif ("${ENABLE_COMPILE_FLAGS_FOR_TARGET}" STREQUAL "armv8-neon")
  # Only build with 32-bit Pi 3/4 specific optimisations if requested on the command line.
  add_definitions(-mfpu=neon-fp-armv8 -ftree-vectorize)
endif()

find_package(PkgConfig REQUIRED)

pkg_check_modules(CAMERA REQUIRED libcamera)
find_library(LIBCAMERA_LIBRARY libcamera.so REQUIRED)
find_library(LIBCAMERA_BASE_LIBRARY libcamera-base.so REQUIRED)
set(LIBCAMERA_LIBRARIES ${LIBCAMERA_LIBRARY} ${LIBCAMERA_BASE_LIBRARY})
message(STATUS ${LIBCAMERA_LIBRARIES})
include_directories(${CMAKE_SOURCE_DIR} ${CAMERA_INCLUDE_DIRS})

add_subdirectory(apps)
add_subdirectory(core)
add_subdirectory(encoder)
add_subdirectory(image)
add_subdirectory(output)
add_subdirectory(preview)
