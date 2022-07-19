cmake_minimum_required(VERSION 3.21)
project(CVRM2022)

set(CMAKE_CXX_STANDARD 20)

add_executable(CVRM2022 main.cpp)
include_directories(AutoAim)
add_subdirectory(AutoAim)

#include_directories(core_io)
#add_subdirectory(core_io)
#target_link_libraries(CVRM2022 core_io)

#set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4)
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

target_link_libraries(CVRM2022 AutoAim ${CUDA_LIBRARIES})
