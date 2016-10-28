cmake_minimum_required(VERSION 3.0)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)

##############################################################################
############################# CONFIGURE COMPILER #############################
##############################################################################

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE}")

message(STATUS "Compiler(Windows): Visual Studio 12 2013 Win64")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "--> NOTE: enabled windows performance flags")
    add_definitions(-DNDEBUG) #assert macro will be preprocessed away for better performance
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Ox /Ot /fp:fast /GS- /GL")
    # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG")
endif()

##############################################################################
############################### CONFIGURE CUDA ###############################
##############################################################################


set(WITH_CUDA TRUE)
add_definitions(-DWITH_CUDA)
set(CUDA_TOOLKIT_ROOT_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.0")

#--- Import CUDA/CUBLAS
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
list(APPEND LIBRARIES ${CUDA_LIBRARIES})
list(APPEND LIBRARIES ${CUDA_CUBLAS_LIBRARIES})

#--- For matrix operations within Kernels (Eigen not supported)
find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
add_definitions(-DGLM_FORCE_CUDA) #< as mentioned in docs

#--- Card needs appropriate version
site_name(HOSTNAME)
message(STATUS "HOSTNAME ${HOSTNAME}")

set(CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50")# GTX980
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -w -Xcompiler -fPIC" )

#--- Enable debugging flags
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -g") # HOST debug mode
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -G") # DEV debug mode
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -lineinfo")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -DNDEBUG") #< disable asserts

set(CUDA_PROPAGATE_HOST_FLAGS True)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    #set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -use_fast_math")
endif()

include_directories(../../)
cuda_add_executable(${PROJECT} main.cu)
target_link_libraries(${PROJECT} ${LIBRARIES})
