cmake_minimum_required(VERSION 3.16)
set(CMAKE_VERBOSE_MAKEFILE ON)
project(hvvr_plugin CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

SET(EXECUTABLE_OUTPUT_PATH ".")

add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-mavx>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-mavx2>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-msse>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-msse2>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-mfma>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-mbmi>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-mf16c>)

add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:-forward-unknown-to-host-compiler>)
add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:-fPIC>)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3 REQUIRED)

add_subdirectory(hvvr)

add_library(plugin SHARED plugin.cpp hvvr_config.h)
set_property(TARGET plugin PROPERTY CUDA_ARCHITECTURES 62)
target_include_directories(plugin PRIVATE
        "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}"
        "${OPENGL_INCLUDE_DIR}"
        )

target_link_libraries(plugin PRIVATE samples_shared)
target_link_libraries(plugin PRIVATE raycaster)