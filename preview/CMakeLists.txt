cmake_minimum_required(VERSION 3.6)

pkg_check_modules(LIBDRM REQUIRED libdrm)
pkg_check_modules(X11 REQUIRED x11)
pkg_check_modules(EPOXY REQUIRED epoxy)

message(STATUS "LIBDRM_LINK_LIBRARIES=${LIBDRM_LINK_LIBRARIES}")
message(STATUS "X11_LINK_LIBRARIES=${X11_LINK_LIBRARIES}")
message(STATUS "EPOXY_LINK_LIBRARIES=${EPOXY_LINK_LIBRARIES}")

include_directories(${LIBDRM_INCLUDE_DIRS})

add_library(preview egl_preview.cpp drm_preview.cpp null_preview.cpp)
target_link_libraries(preview ${LIBDRM_LIBRARIES} ${X11_LIBRARIES} ${EPOXY_LIBRARIES})

install(TARGETS preview LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

