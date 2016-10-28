# Use thrust to sort pixels of a PIXEL_BUFFER that will then be used to render on screen
# 
# Tested on OSX MOUNTAIN LION and Cuda6.0
cmake_minimum_required(VERSION 2.8)
project(cuda_thrust_sort)

find_package(CUDA REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})

#--- Compiler for C++ side
if(APPLE)
    set(CUDA_HOST_COMPILER "/usr/bin/clang++")
endif()

#--- NVCC Compiler Flags
site_name(HOSTNAME)
if(HOSTNAME STREQUAL "lagrange")
    set(CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50") # OK!!!!
else()
    set(CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30") # OK!!!!
endif()

cuda_add_executable(cuda_thrust_sort main.cu)
target_link_libraries(cuda_thrust_sort ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
