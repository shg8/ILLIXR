cmake_minimum_required(VERSION 3.0)
project(plugin)

# Prefer GLVND
if (POLICY CMP0072)
  cmake_policy (SET CMP0072 NEW)
endif(POLICY CMP0072)

find_package(X11 REQUIRED)
find_package(OpenGL REQUIRED)

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)

add_library(plugin SHARED plugin.cpp)

# First try openxr.pc from OpenXR SDK
include(FindPkgConfig)
pkg_search_module(OPENXR openxr)
if (OPENXR_FOUND)
  MESSAGE("OpenXR found with pkg-config")
  target_link_libraries(plugin PRIVATE ${OPENXR_LIBRARIES})

# Second, try OpenXRConfig.cmake from OpenXR SDK
else()
  MESSAGE("OpenXR not found with pkg-config, trying cmake script")

  # current issue in upstream OpenXR cmake files requires us to find Threads on our own
  find_package(Threads REQUIRED)

  find_package(OpenXR REQUIRED)
  if (NOT OpenXR_FOUND)
     MESSAGE(FATAL_ERROR "OpenXR not found!")
  endif()

  target_include_directories(plugin PRIVATE ${OpenXR_INCLUDE_DIR})
  target_link_libraries(plugin PRIVATE OpenXR::openxr_loader)
endif()

target_link_libraries(plugin PRIVATE Xrandr ${X11_LIBRARIES} ${OPENGL_LIBRARIES} ${SDL2_LIBRARIES} m)
target_include_directories(plugin PRIVATE ${SDL2_INCLUDE_DIRS})

if(MSVC)
  target_compile_options(plugin PRIVATE /W4 /WX)
else(MSVC)
  target_compile_options(plugin PRIVATE -pedantic -Wall -Wextra -Wno-unused-parameter -std=c++17 -g -O3)
endif(MSVC)
